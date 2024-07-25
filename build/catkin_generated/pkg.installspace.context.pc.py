# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;costmap_2d;pluginlib;nav_core;base_local_planner;geometry_msgs".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lrrt_star_global_planner".split(';') if "-lrrt_star_global_planner" != "" else []
PROJECT_NAME = "rrt_star_global_planner"
PROJECT_SPACE_DIR = "/usr/local"
PROJECT_VERSION = "0.0.0"
