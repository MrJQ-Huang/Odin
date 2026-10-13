"""OpenCV4 compatibility for the pinned distributed visual backend."""
import pathlib,re,sys
root=pathlib.Path(sys.argv[1])
for package in ('kimera_multi_lcd','kimera_distributed','dpgo_ros'):
    for p in (root/package).rglob('*'):
        if p.suffix not in ('.h','.hpp','.cpp'):continue
        s=p.read_text().replace('#include <opencv/cv.hpp>','#include <opencv2/opencv.hpp>')
        s=s.replace('cv::DescriptorMatcher::create(3)', 'cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_L1)')
        p.write_text(s)
    p=root/package/'CMakeLists.txt'
    s=p.read_text()
    s=re.sub(r'(?m)^(catkin_add_gtest\([^\n]+\))$',r'if(CATKIN_ENABLE_TESTING)\n\1\nendif()',s)
    p.write_text(s)
