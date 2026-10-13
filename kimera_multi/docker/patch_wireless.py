"""Offline, architecture-neutral patches to exact pinned upstream copies."""
import pathlib
import re
import sys
root = pathlib.Path(sys.argv[1])
for name in ('kimera_multi_lcd', 'kimera_distributed', 'dpgo_ros'):
    p = root / name / 'CMakeLists.txt'
    s = p.read_text().replace(' -march=native', '').replace('SET(CMAKE_BUILD_TYPE Debug)', 'SET(CMAKE_BUILD_TYPE Release)')
    # catkin_add_gtest does not create targets when testing is disabled.
    s = re.sub(r'(catkin_add_gtest\(([^\s)]+)[^\n]*\)\n)(target_link_libraries\(\2[^\n]*\))',
               r'if(CATKIN_ENABLE_TESTING)\n\1\3\nendif()', s)
    if name == 'kimera_multi_lcd':
        a = s.index('configure_file(cmake/json.CMakeLists.txt.in')
        b = s.index('\ncatkin_simple()', a)
        s = s[:a] + '\nfind_package(nlohmann_json 3.10 REQUIRED)\n' + s[b:]
        s = s.replace('include(cmake/vocab_download.cmake)', '# Use the same pinned ORB vocabulary as the SDK frontend.')
    p.write_text(s)
for name in ('dpgo', 'roptlib'):
    p = root / name / 'CMakeLists.txt'
    s = p.read_text().replace(' -march=native', '').replace('add_compile_options(-march=native)', '')
    if name == 'dpgo':
        a = s.index('configure_file(cmake/roptlib.cmake')
        b = s.index('\n# Dependencies', a)
        s = s[:a] + 'add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../roptlib" "${CMAKE_BINARY_DIR}/roptlib-build")\n' + s[b:]
    p.write_text(s)
for name in ('pose_graph_tools/pose_graph_tools', 'pose_graph_tools/pose_graph_tools_ros'):
    (root / name / 'CATKIN_IGNORE').unlink(missing_ok=True)
(root / 'kimera_distributed/params/odin1_dual.yaml').write_text('robot0_name: odin_a\nrobot1_name: odin_b\n')
