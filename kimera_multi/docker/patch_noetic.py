"""Small, explicit Noetic compatibility changes to the pinned source copies."""
import pathlib
import subprocess
import sys

root = pathlib.Path(sys.argv[1])
subprocess.run([sys.executable, str(pathlib.Path(__file__).with_name('fix_sparse_keypoints.py')),
                str(root / 'kimera_vio')], check=True)
# DBoW2's external project otherwise fetches a moving master branch. Use the
# already pinned checkout, retaining the upstream FileStorage iterator patch.
path = root / 'dbow2_catkin/CMakeLists.txt'
text = path.read_text()
start = text.index('ExternalProject_Add(dbow2_src')
end = text.index('\n)', start) + 2
old = text[start:end]
if 'GIT_TAG master' not in old or 'fix_fs_iter.patch' not in old:
    raise RuntimeError('Unexpected pinned dbow2_catkin CMakeLists.txt')
new = '''ExternalProject_Add(dbow2_src
  SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../DBoW2
  DOWNLOAD_COMMAND "" UPDATE_COMMAND ""
  PATCH_COMMAND ""
  CMAKE_ARGS -DCMAKE_BUILD_TYPE=Release
             -DCMAKE_INSTALL_PREFIX=${CATKIN_DEVEL_PREFIX} -DBUILD_Demo=OFF
)'''
path.write_text(text[:start] + new + text[end:])
subprocess.run(['patch', '--batch', '-p1', '-i',
                str(root / 'dbow2_catkin/fix_fs_iter.patch')],
               cwd=root / 'DBoW2', check=True)
(root / 'DBoW2/CATKIN_IGNORE').touch()
(root / 'opengv/CATKIN_IGNORE').touch()
path = root / 'opengv/CMakeLists.txt'
text = path.read_text()
if text.count('add_definitions (-march=native)') != 1:
    raise RuntimeError('Unexpected pinned OpenGV march-native setting')
path.write_text(text.replace('add_definitions (-march=native)', '# march-native disabled for portable containers'))
# Upstream links this test target even when CATKIN_ENABLE_TESTING is OFF.
path = root / 'kimera_ros/CMakeLists.txt'
text = path.read_text()
old = '''catkin_add_gtest(testKimeraVioRos
  test/testKimeraVioRos.cpp
)
target_link_libraries(testKimeraVioRos ${PROJECT_NAME})'''
if text.count(old) != 1:
    raise RuntimeError('Unexpected pinned Kimera-VIO-ROS test target')
path.write_text(text.replace(old, 'if(CATKIN_ENABLE_TESTING)\n' + old + '\nendif()'))
path = root / 'mesh_rviz_plugins/CMakeLists.txt'
path.write_text(path.read_text().replace(' -march=native', ''))
# Only the messages/services are required by the Odin1 front end; the separate
# visualizer uses additional dependencies and is unnecessary for headless VIO.
for path in (root / 'pose_graph_tools').glob('*/package.xml'):
    if path.parent.name != 'pose_graph_tools_msgs':
        (path.parent / 'CATKIN_IGNORE').touch()
