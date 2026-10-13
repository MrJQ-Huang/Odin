"""Run and measure an isolated, headless ROS 1 Odin1 bag replay."""
import argparse
import hashlib
import json
import math
import os
import pathlib
import signal
import subprocess
import time
import xmlrpc.client


def main():
    executed_script = pathlib.Path(__file__).read_bytes()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', default='/workspace/artifacts/odin1/motion_rgbd.bag')
    parser.add_argument('--params', default='/workspace/artifacts/odin1/params')
    parser.add_argument('--output', help='New output directory; defaults to docker_sdk or docker_motion by frontend')
    parser.add_argument('--rate', type=float, default=0.5)
    parser.add_argument('--drain', type=float, default=10)
    parser.add_argument('--robot-name', default='odin1')
    parser.add_argument('--robot-id', type=int, default=0)
    parser.add_argument('--frontend', choices=['sdk', 'vio'], default='sdk',
                        help='SDK trajectory frontend (default), or independent Kimera VIO comparison')
    args = parser.parse_args()
    if not math.isfinite(args.rate) or args.rate <= 0 or args.drain < 0:
        parser.error('rate must be positive; drain must be nonnegative')
    if not 0 <= args.robot_id <= 2147483647:
        parser.error('robot-id must be a nonnegative int32 pose graph identifier')
    if args.frontend == 'sdk' and args.robot_id > 65535:
        parser.error('SDK robot-id must also fit the uint16 pose graph query interface')
    bag = pathlib.Path(args.bag).resolve()
    params = pathlib.Path(args.params).resolve()
    if not bag.is_file() or not (params / 'PipelineParams.yaml').is_file():
        parser.error('bag or generated Odin1 parameter directory is missing')
    out = pathlib.Path(args.output or ('/workspace/artifacts/odin1/docker_sdk' if args.frontend == 'sdk'
                                      else '/workspace/artifacts/odin1/docker_motion')).resolve()
    out.mkdir(parents=True, exist_ok=False)
    (out / 'replay_script.py').write_bytes(executed_script)
    pathlib.Path(os.environ['HOME']).mkdir(parents=True, exist_ok=True)
    os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:11311'
    os.environ['ROS_HOSTNAME'] = '127.0.0.1'
    os.environ['ROS_LOG_DIR'] = str(out / 'ros_logs')
    processes, logs, error = [], [], None
    odometry, graph_sizes, bows, vlc_frames, reference = [], [], [], [], []
    published_robot_ids = {'pose_graph': set(), 'bow': set(), 'vlc': set()}
    clock_samples = []
    playback_active = False
    playback_wall_s = None
    bag_duration_s = None
    diagnostics, visual_ids, visual_errors = [], set(), []
    final_graph = None
    sdk_verification = None

    def start(name, command):
        log = (out / (name + '.log')).open('w')
        logs.append(log)
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append((name, process))
        return process

    def wait_for(predicate, timeout, process, description):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if process.poll() is not None:
                raise RuntimeError(f'{description}: process exited {process.returncode}; see logs')
            try:
                if predicate():
                    return
            except (OSError, xmlrpc.client.Error):
                pass
            time.sleep(0.2)
        raise RuntimeError(f'Timed out waiting for {description}')

    try:
        master_process = start('roscore', ['roscore'])
        master = xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
        wait_for(lambda: master.getPid('/odin1_replay')[0] == 1, 20, master_process, 'ROS master')
        subprocess.run(['rosparam', 'set', '/use_sim_time', 'true'], check=True)
        import rospy
        from nav_msgs.msg import Odometry
        from rosgraph_msgs.msg import Clock
        import rosbag
        from pose_graph_tools_msgs.msg import PoseGraph, BowQueries, VLCFrames
        from std_msgs.msg import String
        rospy.init_node('odin1_replay_observer', anonymous=True, disable_signals=True)
        # RosOnlineDataProvider blocks in its constructor until /clock is valid,
        # before it creates image subscribers. Prime the real bag start time so
        # waiting for subscribers and starting rosbag play cannot deadlock.
        with rosbag.Bag(str(bag)) as source:
            simulated_start = source.get_start_time()
            bag_duration_s = source.get_end_time() - simulated_start
        clock = rospy.Publisher('/clock', Clock, queue_size=10, latch=True)
        clock.publish(Clock(clock=rospy.Time.from_sec(simulated_start)))
        prefix = '/' + args.robot_name + '/kimera_vio_ros'

        def on_odom(msg):
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
            if not all(math.isfinite(v) for v in values):
                odometry.append({'invalid': True})
                return
            odometry.append({'stamp': msg.header.stamp.to_sec(), 'stamp_ns': msg.header.stamp.to_nsec(), 'position': values[:3],
                             'quaternion_xyzw': values[3:], 'frame': msg.header.frame_id})

        def on_graph(msg):
            nonlocal final_graph
            final_graph = msg
            graph_sizes.append({'nodes': len(msg.nodes), 'edges': len(msg.edges)})
            published_robot_ids['pose_graph'].update(n.robot_id for n in msg.nodes)

        def on_bow(msg):
            bows.append(len(msg.queries))
            published_robot_ids['bow'].update(q.robot_id for q in msg.queries)

        def on_vlc(msg):
            vlc_frames.append(len(msg.frames))
            published_robot_ids['vlc'].update(f.robot_id for f in msg.frames)
            visual_ids.update(f.pose_id for f in msg.frames)
            if args.frontend == 'sdk':
                import struct
                for frame in msg.frames:
                    n = len(frame.depths)
                    image = frame.descriptors_mat
                    cloud = frame.versors
                    if not (n >= 20 and image.encoding == '8UC1' and image.width == 32 and
                            image.height == n and len(image.data) == n * image.step and
                            cloud.width * cloud.height == n):
                        visual_errors.append('Descriptor / bearing / depth row counts or layout disagree')
                        continue
                    fields = {f.name: f.offset for f in cloud.fields}
                    for i, depth in enumerate(frame.depths):
                        v = [struct.unpack_from('<f', cloud.data, i*cloud.point_step+fields[k])[0]
                             for k in ['x', 'y', 'z']]
                        if not (all(math.isfinite(x) for x in v) and .3 <= depth <= 8 and
                                abs(sum(x*x for x in v)-1) < 1e-5 and v[2] > 0):
                            visual_errors.append('Invalid visual keypoint geometry')
                            break

        def on_reference(msg):
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            reference.append({'stamp': msg.header.stamp.to_sec(), 'stamp_ns': msg.header.stamp.to_nsec(),
                              'position': [p.x, p.y, p.z],
                              'quaternion_xyzw': [q.x, q.y, q.z, q.w]})

        def on_clock(msg):
            if playback_active:
                clock_samples.append((time.monotonic(), msg.clock.to_sec()))

        subscribers = [
            rospy.Subscriber('/clock', Clock, on_clock, queue_size=1000),
            rospy.Subscriber(prefix + '/odometry', Odometry, on_odom, queue_size=1000),
            rospy.Subscriber(prefix + '/pose_graph', PoseGraph,
                             on_graph, queue_size=100),
            rospy.Subscriber(prefix + '/bow_query', BowQueries,
                             on_bow, queue_size=100),
            rospy.Subscriber(prefix + '/vlc_frames', VLCFrames,
                             on_vlc, queue_size=100),
            rospy.Subscriber('/' + args.robot_name + '/kimera/reference/odometry', Odometry,
                             on_reference,
                             queue_size=1000),
            rospy.Subscriber(prefix + '/sdk_diagnostics', String,
                             lambda m: diagnostics.append(json.loads(m.data)), queue_size=100),
        ]
        (out / 'vio_logs').mkdir()
        filename = 'kimera_sdk_odin1.launch' if args.frontend == 'sdk' else 'kimera_vio_odin1.launch'
        launch_file = pathlib.Path('/workspace/launch') / filename
        if not launch_file.is_file():
            launch_file = pathlib.Path('/opt/kimera_ws/src/kimera_multi/launch') / filename
        command = ['roslaunch', str(launch_file),
                                 'params_folder:=' + str(params), 'robot_name:=' + args.robot_name,
                                 'robot_id:=' + str(args.robot_id)]
        if args.frontend == 'vio':
            command += ['log_output:=true', 'log_output_path:=' + str(out / 'vio_logs')]
        launch = start('kimera', command)
        node = prefix + ('/odin_sdk_frontend' if args.frontend == 'sdk' else '/kimera_vio_ros_node')

        def subscribed():
            state = master.getSystemState('/odin1_replay')[2]
            topic = '/' + args.robot_name + '/kimera/rgb/image_raw'
            return any(t == topic and node in ns for t, ns in state[1])

        wait_for(subscribed, 120, launch, 'Kimera image subscriber / vocabulary initialization')
        recording = start('record', ['rosbag', 'record', '-O', str(out / 'result.bag'),
                                    prefix + '/odometry', prefix + '/pose_graph',
                                    prefix + '/bow_query', prefix + '/vlc_frames',
                                    prefix + '/pose_graph_incremental', prefix + '/keyframe_odometry',
                                    prefix + '/sdk_diagnostics',
                                    '/' + args.robot_name + '/kimera/reference/odometry', '/tf', '/clock'])
        time.sleep(2)
        playback_active = True
        playback_start = time.monotonic()
        player = start('play', ['rosbag', 'play', '--clock', '--delay=2', '-r', str(args.rate), str(bag)])
        while player.poll() is None:
            if launch.poll() is not None:
                raise RuntimeError('Kimera exited during playback; inspect kimera.log')
            time.sleep(0.2)
        playback_wall_s = time.monotonic() - playback_start
        playback_active = False
        if player.returncode:
            raise RuntimeError(f'rosbag play exited {player.returncode}')
        if '[FATAL]' in (out / 'play.log').read_text(errors='replace'):
            raise RuntimeError('rosbag player reported a fatal error despite exiting zero; inspect play.log')
        # Continue /clock briefly so the LCD's 1 Hz timer flushes its final
        # buffered VLC frames. Sensor/odometry header timestamps stay untouched.
        simulated_end = rospy.Time.now().to_sec()
        drain_start = time.monotonic()
        while time.monotonic() - drain_start < args.drain:
            clock.publish(Clock(clock=rospy.Time.from_sec(
                simulated_end + time.monotonic() - drain_start)))
            time.sleep(0.02)
        if launch.poll() is not None:
            raise RuntimeError('Kimera exited while draining the pipeline')
        if not odometry:
            raise RuntimeError('No Kimera odometry was published')
        if any('invalid' in x for x in odometry):
            raise RuntimeError('Nonfinite odometry output')
        if any(b['stamp'] <= a['stamp'] for a, b in zip(odometry, odometry[1:])):
            raise RuntimeError('Odometry timestamps are not strictly increasing')
        if not graph_sizes or not bows or not vlc_frames:
            raise RuntimeError('Missing pose graph, BoW, or VLC frame output')
        if any(ids != {args.robot_id} for ids in published_robot_ids.values()):
            raise RuntimeError('Pose graph, BoW, or VLC published an unexpected robot ID')
        if args.frontend == 'sdk':
            import numpy as np
            import yaml
            from tf.transformations import quaternion_matrix, quaternion_from_matrix
            from pose_graph_tools_msgs.srv import PoseGraphQuery, VLCFrameQuery
            if visual_errors or not diagnostics or diagnostics[-1].get('trajectory_source') != 'odin_sdk':
                raise RuntimeError('SDK frontend diagnostics or visual geometry validation failed: '+str(visual_errors[:2]))
            by_stamp = {r['stamp_ns']: r for r in reference}
            if len(odometry) != len(reference):
                raise RuntimeError('SDK output did not forward every input odometry sample')
            max_position, max_quaternion = 0., 0.
            for output in odometry:
                source = by_stamp.get(output['stamp_ns'])
                if source is None:
                    raise RuntimeError('SDK odometry output rewrote a timestamp')
                max_position = max(max_position, max(abs(a-b) for a,b in zip(output['position'],source['position'])))
                max_quaternion = max(max_quaternion, max(abs(a-b) for a,b in zip(output['quaternion_xyzw'],source['quaternion_xyzw'])))
            if max_position != 0 or max_quaternion != 0:
                raise RuntimeError('SDK trajectory was reestimated or modified')
            camera_config = yaml.safe_load((params/'LeftCameraParams.yaml').read_text().replace('%YAML:1.0',''))
            body_T_camera = np.array(camera_config['T_BS']['data']).reshape(4,4)
            original_rotation = body_T_camera[:3,:3].copy()
            u, _, vt = np.linalg.svd(original_rotation)
            body_T_camera[:3,:3] = u @ vt
            calibration_rotation_correction = float(np.linalg.norm(body_T_camera[:3,:3]-original_rotation))
            max_graph_error = 0.
            for vertex in final_graph.nodes:
                source = by_stamp.get(vertex.header.stamp.to_nsec())
                if source is None:
                    raise RuntimeError('Recorded SDK graph camera pose is not tied to a source pose')
                expected = quaternion_matrix(source['quaternion_xyzw'])
                expected[:3,3] = source['position']
                expected = expected @ body_T_camera
                actual = np.array([vertex.pose.position.x,vertex.pose.position.y,vertex.pose.position.z])
                max_graph_error = max(max_graph_error,float(np.linalg.norm(actual-expected[:3,3])))
                q = vertex.pose.orientation
                actual_q = np.array([q.x,q.y,q.z,q.w])
                if abs(np.dot(actual_q,quaternion_from_matrix(expected))) < 1-1e-9:
                    raise RuntimeError('SDK graph camera orientation does not match calibrated extrinsics')
            if max_graph_error > 1e-8 or len(final_graph.edges) != len(final_graph.nodes)-1:
                raise RuntimeError('SDK graph transform or odometry chain is inconsistent')
            expected_ids = set(range(len(final_graph.nodes)))
            if {n.key for n in final_graph.nodes} != expected_ids or not visual_ids <= expected_ids:
                raise RuntimeError('SDK visual keyframe IDs are not aligned with the odometry graph')
            node_by_key = {n.key: n for n in final_graph.nodes}
            def pose_matrix(pose):
                q = pose.orientation
                result = quaternion_matrix([q.x, q.y, q.z, q.w])
                p = pose.position
                result[:3,3] = [p.x, p.y, p.z]
                return result
            max_edge_error = 0.
            for edge in final_graph.edges:
                if edge.type != edge.ODOM or edge.key_to != edge.key_from+1 or any(edge.covariance[i*6+i] <= 0 for i in range(6)):
                    raise RuntimeError('Invalid SDK odometry graph edge')
                if edge.robot_from != args.robot_id or edge.robot_to != args.robot_id:
                    raise RuntimeError('SDK graph edge mixed robot IDs')
                expected = np.linalg.inv(pose_matrix(node_by_key[edge.key_from].pose)) @ pose_matrix(node_by_key[edge.key_to].pose)
                max_edge_error = max(max_edge_error, float(np.max(np.abs(pose_matrix(edge.pose)-expected))))
            if max_edge_error > 1e-8:
                raise RuntimeError('SDK graph relative motion disagrees with its camera nodes')
            graph_query = rospy.ServiceProxy(prefix+'/pose_graph_query',PoseGraphQuery)(args.robot_id)
            chosen = sorted(visual_ids)
            chosen = sorted({chosen[0],chosen[len(chosen)//2],chosen[-1]})
            frame_query = rospy.ServiceProxy(prefix+'/vlc_frame_query',VLCFrameQuery)(args.robot_id,chosen)
            if len(graph_query.pose_graph.nodes) != len(final_graph.nodes) or {f.pose_id for f in frame_query.frames} != set(chosen):
                raise RuntimeError('SDK graph/frame query service returned inconsistent data')
            sdk_verification = {'all_sdk_poses_forwarded':True,'maximum_position_change_m':max_position,
                                'maximum_quaternion_component_change':max_quaternion,
                                'maximum_graph_camera_position_error_m':max_graph_error,
                                'maximum_relative_camera_transform_error':max_edge_error,
                                'visual_geometry_checked':True,'queried_visual_frame_ids':chosen,
                                'graph_service_checked':True,'graph_pose_frame':'camera',
                                'calibration_rotation_so3_correction_norm':calibration_rotation_correction,
                                'odometry_pose_frame':'imu','graph_edges':'SDK relative camera motion only'}
    except (Exception, KeyboardInterrupt) as exc:
        error = str(exc)
        if isinstance(exc, KeyboardInterrupt):
            error = 'Interrupted'
    finally:
        playback_active = False
        for name, process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=10)
        for log in logs:
            log.close()
        with bag.open('rb') as data:
            digest = hashlib.sha256()
            for block in iter(lambda: data.read(1024 * 1024), b''):
                digest.update(block)
        effective_rate = None
        if bag_duration_s and len(clock_samples) > 1:
            middle = [(wall, sim) for wall, sim in clock_samples
                      if simulated_start + .1*bag_duration_s <= sim <= simulated_start + .9*bag_duration_s]
            if len(middle) > 1 and middle[-1][0] > middle[0][0]:
                effective_rate = (middle[-1][1]-middle[0][1])/(middle[-1][0]-middle[0][0])
        summary = {'bag': str(bag), 'bag_sha256': digest.hexdigest(),
                   'docker_image_id': os.environ.get('ODIN1_IMAGE_ID'),
                   'source_revisions': json.loads(pathlib.Path('/opt/odin1.repos').read_text())['repositories'],
                   'params': str(params), 'rate': args.rate,
                   'robot_name': args.robot_name, 'robot_id': args.robot_id,
                   'frontend': args.frontend, 'trajectory_source': 'odin_sdk' if args.frontend == 'sdk' else 'kimera_vio',
                   'sdk_verification': sdk_verification,
                   'sdk_diagnostics': diagnostics[-1] if diagnostics else None,
                   'bag_duration_s': bag_duration_s, 'playback_wall_s': playback_wall_s,
                   'effective_clock_rate_middle_80_percent': effective_rate,
                   'published_robot_ids': {key: sorted(ids) for key, ids in published_robot_ids.items()},
                   'replay_script_sha256': hashlib.sha256(executed_script).hexdigest(),
                   'parameter_sha256': {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                                        for p in sorted(params.rglob('*')) if p.is_file()},
                   'odometry_count': len(odometry), 'odometry': odometry,
                   'pose_graph_messages': len(graph_sizes), 'last_pose_graph': graph_sizes[-1] if graph_sizes else None,
                   'bow_messages': len(bows), 'bow_queries': sum(bows),
                   'vlc_messages': len(vlc_frames), 'vlc_frames': sum(vlc_frames),
                   'reference_odometry_count': len(reference),
                   'reference_odometry': reference,
                   'reference_note': 'Odin SDK owns the local trajectory in SDK mode. In VIO mode it is a reference, not ground truth, and is not an external VIO factor.',
                   'process_exit_codes': {n: p.returncode for n, p in processes},
                   'error': error, 'success': error is None}
        (out / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
        shutil_source = pathlib.Path('/opt/noetic-packages.tsv')
        (out / 'noetic-packages.tsv').write_bytes(shutil_source.read_bytes())
        print(json.dumps({k: v for k, v in summary.items()
                          if k not in ('odometry', 'reference_odometry', 'parameter_sha256')}, indent=2), flush=True)
    if error:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
