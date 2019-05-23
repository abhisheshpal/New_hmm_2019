cmake_minimum_required(VERSION 2.8.3)
project(rasberry_han)

find_package(catkin REQUIRED
        COMPONENTS
        dynamic_reconfigure
        geometry_msgs
        std_msgs
        tf
        bayes_people_tracker
)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/HumanProximityMonitor.cfg
)

catkin_package(
    CATKIN_DEPENDS
        dynamic_reconfigure
        geometry_msgs
        std_msgs
        tf
        bayes_people_tracker
)
