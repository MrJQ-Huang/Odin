"""Offline cloud accumulation and explicitly experimental geometric registration."""
import argparse
from collections import Counter
import json
from pathlib import Path
import sys
import time

import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'adapters/odin1'))
from odin1_adapter.calibration import load_config
from odin1_adapter.geometry import PoseSeries, PoseUnavailable, apply_transform, cloud_arrays
from odin1_adapter.synchronization import pose_entry, stamp_ns

BASE = ROOT / 'artifacts/dual_odin1_20261013'
OUT = BASE / 'cloud_fusion_preview'
RAW = ROOT / 'experiments/dual_odin1_20261013/motion_usb128_bag'


def voxel(points, size, min_count=1):
    keys = np.floor(points / size).astype(np.int32)
    _, inverse, counts = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
    centers = np.column_stack([np.bincount(inverse, weights=points[:, i]) for i in range(3)])
    centers /= counts[:, None]
    return centers[counts >= min_count]


def write_ply(path, points, color):
    rows = np.empty(len(points), dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                                        ('red', 'u1'), ('green', 'u1'), ('blue', 'u1')])
    for i, name in enumerate(['x', 'y', 'z']): rows[name] = points[:, i]
    colors = np.asarray(color)
    if colors.ndim == 1: colors = np.broadcast_to(colors, (len(points), 3))
    for i, name in enumerate(['red', 'green', 'blue']): rows[name] = colors[:, i]
    header = ('ply\nformat binary_little_endian 1.0\nelement vertex '+str(len(points))+
              '\nproperty float x\nproperty float y\nproperty float z\n'
              'property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n')
    with path.open('wb') as file:
        file.write(header.encode()); file.write(rows.tobytes())


def extract():
    OUT.mkdir(exist_ok=False)
    validation = json.loads((BASE / 'validation.json').read_text())
    config, lower, upper, entries, vio = {}, {}, {}, {}, {}
    for name in ['odin_a', 'odin_b']:
        config[name] = load_config(BASE / name / 'adapter.yaml')
        w = validation['corrected_motion_window'][name]
        lower[name] = round((w['first_image_stamp_s'] + w['start'])*1e9)
        upper[name] = lower[name] + round(w['duration']*1e9)
        entries[name] = []
        result = json.loads((BASE / 'docker_motion_usb128_fixed_pair' / name / 'summary.json').read_text())
        vio[name] = PoseSeries([(round(p['stamp']*1e9), p['position'], p['quaternion_xyzw'])
                                for p in result['odometry']])
    store = get_typestore(Stores.ROS2_HUMBLE)
    with AnyReader([RAW], default_typestore=store) as reader:
        # Reception time only bounds I/O; all filtering and geometry use the
        # original per-device sensor headers and per-point acquisition times.
        start_arrival = reader.start_time + int(210e9)
        cs = [c for c in reader.connections if c.topic.endswith('/odometry_highfreq')]
        for c, _, raw in reader.messages(connections=cs, start=start_arrival):
            name = c.topic.split('/')[1]
            msg = reader.deserialize(raw, c.msgtype)
            t = stamp_ns(msg)
            if lower[name]-int(1e9) <= t <= upper[name]+int(1e9):
                entries[name].append(pose_entry(msg))
    sdk = {name: PoseSeries(rows) for name, rows in entries.items()}
    alignment, origins = {}, {}
    for name in sdk:
        anchor = np.array([vio[name].times[0]])
        rs, ts = sdk[name].interpolate(anchor, 10e6)
        rv, tv = vio[name].interpolate(anchor, 500e6)
        # Put the VIO map in this device's SDK world frame at the first pose,
        # using one fixed rigid transform; do not bend or optimize trajectories.
        r = rs[0] @ rv[0].T
        alignment[name] = (r, ts[0] - r @ tv[0])
        origins[name] = ts[0]
    chunks = {name: {'sdk': [], 'vio': []} for name in sdk}
    stats = {name: Counter() for name in sdk}
    next_stamp = lower.copy()
    with AnyReader([RAW], default_typestore=store) as reader:
        cs = [c for c in reader.connections if c.topic.endswith('/cloud_raw')]
        for c, _, raw in reader.messages(connections=cs, start=start_arrival):
            name = c.topic.split('/')[1]
            msg = reader.deserialize(raw, c.msgtype)
            t = stamp_ns(msg)
            if not lower[name] <= t <= upper[name]: continue
            stats[name]['clouds_in_window'] += 1
            if t < next_stamp[name]: continue
            next_stamp[name] = t + int(.45e9)
            arrays = cloud_arrays(msg)
            xyz = np.column_stack([arrays[k] for k in ['x', 'y', 'z']]).astype(float)
            offsets = arrays['offset_time'].astype(float)
            ranges = np.linalg.norm(xyz, axis=1)
            valid = (np.isfinite(xyz).all(axis=1) & np.isfinite(offsets) &
                     (offsets >= 0) & (offsets <= .2) & (ranges >= .3) & (ranges <= 8) &
                     (arrays['confidence'] >= config[name]['confidence_threshold']))
            xyz, offsets = xyz[valid], offsets[valid]
            if not len(xyz): stats[name]['empty_filtered_scans'] += 1; continue
            times = t + np.rint(offsets*1e9).astype(np.int64)
            unique, inverse = np.unique(times, return_inverse=True)
            try:
                r, p = sdk[name].interpolate(unique, 10e6)
                rs, ps = sdk[name].interpolate(np.array([t]), 10e6)
                rv, pv = vio[name].interpolate(np.array([t]), 500e6)
            except PoseUnavailable as exc:
                stats[name]['skipped:'+str(exc).split(';')[0]] += 1
                continue
            body = apply_transform(config[name]['T_imu_lidar'], xyz)
            world = np.einsum('nij,nj->ni', r[inverse], body) + p[inverse]
            body_at_stamp = (world - ps[0]) @ rs[0]
            via_vio = body_at_stamp @ rv[0].T + pv[0]
            ar, at = alignment[name]
            via_vio = via_vio @ ar.T + at
            chunks[name]['sdk'].append(world-origins[name])
            chunks[name]['vio'].append(via_vio-origins[name])
            stats[name]['scans_used'] += 1
            stats[name]['filtered_points_accumulated'] += len(xyz)
            if stats[name]['scans_used'] % 40 == 0:
                print(name, dict(stats[name]), flush=True)
    report = {'scope': 'Raw DTOF points accumulated independently; VIO uses SDK scan deskew and one rigid anchor into each local SDK world. No inter-robot factor or PGO.',
              'sampling_interval_s': .45, 'voxel_m': .05, 'minimum_voxel_observations': 2,
              'range_m': [.3, 8], 'confidence_threshold': 35,
              'sdk_max_pose_gap_ms': 10, 'vio_interpolation_max_gap_ms': 500,
              'notes': ['The 500 ms VIO gap limit only interpolates sparse (~3.4 Hz) output poses for this preview; SDK deskew retains the 10 ms gap guard.',
                        'Odin SDK poses are a reference and not ground truth.',
                        'Initial IMU-to-LiDAR extrinsics are not independently calibrated.'],
              'devices': {}}
    for name in sdk:
        report['devices'][name] = dict(stats[name])
        for mode in ['sdk', 'vio']:
            points = voxel(np.concatenate(chunks[name][mode]), .05, min_count=2)
            np.savez_compressed(OUT / f'{name}_{mode}.npz', points=points)
            write_ply(OUT / f'{name}_{mode}.ply', points, [30, 135, 205] if name == 'odin_a' else [205, 90, 180])
            report['devices'][name][mode+'_map_points'] = len(points)
        print(name, report['devices'][name], flush=True)
    (OUT / 'accumulation.json').write_text(json.dumps(report, indent=2)+'\n')


def fit_rigid(source, target):
    a, b = source.mean(axis=0), target.mean(axis=0)
    u, _, vt = np.linalg.svd((source-a).T @ (target-b))
    r = vt.T @ u.T
    if np.linalg.det(r) < 0:
        vt[-1] *= -1
        r = vt.T @ u.T
    matrix = np.eye(4)
    matrix[:3, :3], matrix[:3, 3] = r, b-r@a
    return matrix


def icp(source, target, initial, threshold, iterations=35):
    from scipy.spatial import cKDTree
    tree = cKDTree(target)
    matrix = initial.copy()
    previous = None
    for _ in range(iterations):
        moved = apply_transform(matrix, source)
        distance, indices = tree.query(moved)
        valid = distance < threshold
        if valid.sum() < 40: break
        cutoff = np.quantile(distance[valid], .8)
        valid &= distance <= cutoff
        delta = fit_rigid(moved[valid], target[indices[valid]])
        matrix = delta @ matrix
        error = float(np.mean(distance[valid]**2))
        if previous is not None and abs(previous-error) < 1e-7: break
        previous = error
    return matrix


def normals(points):
    from scipy.spatial import cKDTree
    _, indices = cKDTree(points).query(points, k=20)
    neighbors = points[indices]
    neighbors -= neighbors.mean(axis=1, keepdims=True)
    _, vectors = np.linalg.eigh(np.einsum('nki,nkj->nij', neighbors, neighbors))
    return vectors[:, :, 0]


def plane_icp(source, target, initial, threshold=.2):
    from scipy.spatial import cKDTree
    from scipy.spatial.transform import Rotation
    tree = cKDTree(target)
    normal = normals(target)
    matrix = initial.copy()
    singular_values = []
    for _ in range(35):
        moved = apply_transform(matrix, source)
        distance, indices = tree.query(moved)
        valid = distance < threshold
        if valid.sum() < 100: break
        x, y, n = moved[valid], target[indices[valid]], normal[indices[valid]]
        residual = np.sum((x-y)*n, axis=1)
        good = np.abs(residual) <= .15
        x, n, residual = x[good], n[good], residual[good]
        weights = np.sqrt(np.minimum(1, .04/np.maximum(np.abs(residual), 1e-8)))
        jacobian = np.column_stack([np.cross(x, n), n])
        update, _, _, singular_values = np.linalg.lstsq(jacobian*weights[:,None], -residual*weights, rcond=None)
        if np.linalg.norm(update[:3]) > .2 or np.linalg.norm(update[3:]) > .3: break
        delta = np.eye(4)
        delta[:3, :3], delta[:3, 3] = Rotation.from_rotvec(update[:3]).as_matrix(), update[3:]
        matrix = delta @ matrix
        if np.linalg.norm(update) < 1e-5: break
    return matrix, list(map(float, singular_values))


def quality(source, target, matrix, threshold=.1):
    from scipy.spatial import cKDTree
    moved = apply_transform(matrix, source)
    ab = cKDTree(target).query(moved)[0]
    ba = cKDTree(moved).query(target)[0]
    ia, ib = ab < threshold, ba < threshold
    fa, fb = float(ia.mean()), float(ib.mean())
    return {'distance_threshold_m': threshold, 'b_to_a_overlap_fraction': fa,
            'a_to_b_overlap_fraction': fb,
            'symmetric_overlap_harmonic': 2*fa*fb/max(fa+fb,1e-9),
            'b_to_a_inlier_rmse_m': float(np.sqrt(np.mean(ab[ia]**2))) if ia.any() else None,
            'a_to_b_inlier_rmse_m': float(np.sqrt(np.mean(ba[ib]**2))) if ib.any() else None,
            'all_b_to_a_median_m': float(np.median(ab)),
            'all_a_to_b_median_m': float(np.median(ba))}


def register():
    from scipy.spatial.transform import Rotation
    rng = np.random.default_rng(20261013)
    results = {'scope': 'Offline geometry-only ICP candidate previews, independently fitted per map source. No Kimera-Multi cross-robot loop closure or distributed PGO.',
               'metric_note': 'Overlap and inlier nearest-neighbor RMSE are fitting diagnostics on the input maps, not trajectory/map accuracy against ground truth.',
               'modes': {}}
    for mode in ['sdk', 'vio']:
        a = np.load(OUT / f'odin_a_{mode}.npz')['points']
        b = np.load(OUT / f'odin_b_{mode}.npz')['points']
        ac, bc = voxel(a, .18), voxel(b, .18)
        if len(ac) > 6000: ac = ac[rng.choice(len(ac), 6000, replace=False)]
        if len(bc) > 6000: bc = bc[rng.choice(len(bc), 6000, replace=False)]
        candidates = []
        for degrees in range(0, 360, 15):
            rotation = Rotation.from_euler('z', degrees, degrees=True).as_matrix()
            for centered in [False, True]:
                initial = np.eye(4)
                initial[:3, :3] = rotation
                if centered: initial[:3,3] = np.median(ac,axis=0)-rotation@np.median(bc,axis=0)
                matrix = icp(bc, ac, initial, .8, iterations=40)
                q = quality(bc, ac, matrix, .25)
                candidates.append({'initial_yaw_deg': degrees, 'centroid_initialization': centered,
                                   'T_a_b': matrix.tolist(), 'coarse_quality': q})
            if degrees % 60 == 0: print(mode, 'yaw seeds', degrees, flush=True)
        candidates.sort(key=lambda c:c['coarse_quality']['symmetric_overlap_harmonic'], reverse=True)
        af, bf = voxel(a, .08), voxel(b, .08)
        if len(af)>25000: af=af[rng.choice(len(af),25000,replace=False)]
        if len(bf)>25000: bf=bf[rng.choice(len(bf),25000,replace=False)]
        fine = []
        for candidate in candidates[:5]:
            matrix = icp(bf, af, np.array(candidate['T_a_b']), .25, iterations=30)
            matrix, singular = plane_icp(bf, af, matrix)
            q = quality(b, a, matrix)
            fine.append({'T_a_b': matrix.tolist(), 'quality': q, 'normal_equation_singular_values': singular,
                         'seed': {'yaw_deg':candidate['initial_yaw_deg'],'centered':candidate['centroid_initialization']}})
        fine.sort(key=lambda c:c['quality']['symmetric_overlap_harmonic'], reverse=True)
        best = fine[0]
        results['modes'][mode] = {'best_candidate': best, 'refined_candidates': fine,
                                  'coarse_candidates': candidates,
                                  'a_points':len(a), 'b_points':len(b),
                                  'label': 'Experimental candidate alignment; visual inspection required.'}
        moved = apply_transform(np.array(best['T_a_b']), b)
        merged = np.concatenate([a,moved])
        colors = np.concatenate([np.tile([30,135,205],(len(a),1)),np.tile([205,90,180],(len(b),1))])
        np.savez_compressed(OUT / f'fused_{mode}_candidate.npz', a=a,b=moved)
        write_ply(OUT / f'fused_{mode}_candidate.ply', merged, colors)
        print(mode,'best',best,flush=True)
        (OUT / 'registration.json').write_text(json.dumps(results,indent=2)+'\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('phase', choices=['extract', 'register'])
    args = parser.parse_args()
    {'extract':extract, 'register':register}[args.phase]()
