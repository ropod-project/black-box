cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(black_box_datalogger)

find_package(yaml-cpp REQUIRED)
find_package(roscpp REQUIRED)
find_package(topic_tools REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ropod_ros_msgs REQUIRED)
find_package(PkgConfig)
include(FindPkgConfig)

pkg_check_modules(JSONCPP jsoncpp)
link_libraries(${JSONCPP_LIBRARIES})

pkg_check_modules(LIBMONGOCXX REQUIRED libmongocxx)
pkg_check_modules(LIBBSONCXX REQUIRED libbsoncxx)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
########################################################################
# LIBZMQ dependency
########################################################################
find_package(libzmq REQUIRED)
IF (LIBZMQ_FOUND)
    include_directories(${LIBZMQ_INCLUDE_DIRS})
    list(APPEND LIBS ${LIBZMQ_LIBRARIES})
ELSE (LIBZMQ_FOUND)
    message( FATAL_ERROR "libzmq not found." )
ENDIF (LIBZMQ_FOUND)

########################################################################
# CZMQ dependency
########################################################################
find_package(czmq REQUIRED)
IF (CZMQ_FOUND)
    include_directories(${CZMQ_INCLUDE_DIRS})
    list(APPEND LIBS ${CZMQ_LIBRARIES})
ELSE (CZMQ_FOUND)
    message( FATAL_ERROR "czmq not found." )
ENDIF (CZMQ_FOUND)

########################################################################
# Zyre dependency
########################################################################
find_package(zyre REQUIRED)
IF (ZYRE_FOUND)
    include_directories(${ZYRE_INCLUDE_DIRS})
    list(APPEND LIBS ${ZYRE_LIBRARIES})
ELSE (ZYRE_FOUND)
    message( FATAL_ERROR "zyre not found." )
ENDIF (ZYRE_FOUND)


include_directories(
    ../common/include
    include/
    ${JSONCPP_INCLUDE_DIRS}
    ${LIBMONGOCXX_INCLUDE_DIRS}
    ${LIBBSONCXX_INCLUDE_DIRS}
    ${roscpp_INCLUDE_DIRS}
    ${topic_tools_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${nav_msgs_INCLUDE_DIRS}
    ${move_base_msgs_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    ${ropod_ros_msgs_INCLUDE_DIRS}
)

link_directories(
    ${LIBMONGOCXX_LIBRARY_DIRS}
    ${LIBBSONCXX_LIBRARY_DIRS}
)

##################### Data logger #####################
add_library(datalogger
    ../common/src/config/config_enums.cpp
    ../common/src/config/config_file_reader.cpp
    ../common/src/utils/zyre/zyre_utils.cpp
    ../common/src/extern/zyre/node.cpp
    ../common/src/extern/zyre/event.cpp
    ../common/src/extern/zyre/exception.cpp
    src/data_loggers/text_logger.cpp
    src/data_loggers/mongodb_logger.cpp
    src/data_readers/data_reader.cpp
    src/data_readers/data_tunnel_interface.cpp
    src/data_readers/ethercat_reader.cpp
    src/data_readers/zmq_reader.cpp
    src/data_readers/json_zmq_reader.cpp
    src/data_readers/rostopic_reader.cpp
    src/data_readers/zyre_reader.cpp
    src/rostopic_listeners/rostopic_listener_factory.cpp
    src/rostopic_listeners/rostopic_listener_base.cpp
    src/rostopic_listeners/string_listener.cpp
    src/rostopic_listeners/odom_listener.cpp
    src/rostopic_listeners/path_listener.cpp
    src/rostopic_listeners/move_base_result_listener.cpp
    src/rostopic_listeners/pose_listener.cpp
    src/rostopic_listeners/pose_stamped_listener.cpp
    src/rostopic_listeners/pose_with_covariance_stamped_listener.cpp
    src/rostopic_listeners/imu_listener.cpp
    src/rostopic_listeners/joy_listener.cpp
    src/rostopic_listeners/twist_listener.cpp
    src/rostopic_listeners/laserscan_listener.cpp
    src/rostopic_listeners/door_listener.cpp
    src/rostopic_listeners/joint_state_listener.cpp
    src/rostopic_listeners/smart_wheel_data_listener.cpp
)

target_link_libraries(datalogger
    tins
    pthread
    zmq
    yaml-cpp
    jsoncpp
    ${LIBMONGOCXX_LIBRARIES}
    ${LIBBSONCXX_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${topic_tools_LIBRARIES}
    ${std_msgs_LIBRARIES}
    ${geometry_msgs_LIBRARIES}
    ${nav_msgs_LIBRARIES}
    ${move_base_msgs_LIBRARIES}
    ${sensor_msgs_LIBRARIES}
    ${ropod_ros_msgs_LIBRARIES}
    ${LIBS}
)

target_compile_features(datalogger PRIVATE cxx_range_for)
add_executable(black_box_logger src/logger_main.cpp)
target_link_libraries(black_box_logger datalogger)
target_compile_features(black_box_logger PRIVATE cxx_range_for)
#######################################################
