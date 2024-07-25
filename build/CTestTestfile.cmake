# CMake generated Testfile for 
# Source directory: /home/ilyes/ros_arduino/src/rrt_star_global_planner
# Build directory: /home/ilyes/ros_arduino/src/rrt_star_global_planner/build
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_rrt_star_global_planner_rostest_test_rrt_star.test "/home/ilyes/ros_arduino/src/rrt_star_global_planner/build/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/ilyes/ros_arduino/src/rrt_star_global_planner/build/test_results/rrt_star_global_planner/rostest-test_rrt_star.xml" "--return-code" "/usr/bin/python3 /opt/ros/noetic/share/rostest/cmake/../../../bin/rostest --pkgdir=/home/ilyes/ros_arduino/src/rrt_star_global_planner --package=rrt_star_global_planner --results-filename test_rrt_star.xml --results-base-dir \"/home/ilyes/ros_arduino/src/rrt_star_global_planner/build/test_results\" /home/ilyes/ros_arduino/src/rrt_star_global_planner/test/rrt_star.test ")
set_tests_properties(_ctest_rrt_star_global_planner_rostest_test_rrt_star.test PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/rostest/cmake/rostest-extras.cmake;52;catkin_run_tests_target;/opt/ros/noetic/share/rostest/cmake/rostest-extras.cmake;80;add_rostest;/opt/ros/noetic/share/rostest/cmake/rostest-extras.cmake;100;_add_rostest_google_test;/home/ilyes/ros_arduino/src/rrt_star_global_planner/CMakeLists.txt;58;add_rostest_gtest;/home/ilyes/ros_arduino/src/rrt_star_global_planner/CMakeLists.txt;0;")
subdirs("gtest")
