cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(black_box_query_interface)

find_package(yaml-cpp REQUIRED)
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
    ../datalogger/include
    include/
    ${JSONCPP_INCLUDE_DIRS}
    ${LIBMONGOCXX_INCLUDE_DIRS}
    ${LIBBSONCXX_INCLUDE_DIRS}
)

link_directories(
    ${LIBMONGOCXX_LIBRARY_DIRS}
    ${LIBBSONCXX_LIBRARY_DIRS}
)

################### Query interface ###################
add_library(data_query_interface
    ../common/src/config/config_enums.cpp
    ../common/src/config/config_file_reader.cpp
    ../common/src/extern/zyre/node.cpp
    ../common/src/extern/zyre/event.cpp
    ../common/src/extern/zyre/exception.cpp
    ../common/src/utils/zyre/zyre_utils.cpp
    ../datalogger/src/data_loggers/mongodb_logger.cpp
    src/ethercat_query_interface.cpp
    src/ros_query_interface.cpp
    src/zmq_query_interface.cpp
    src/zyre_query_interface.cpp
    src/query_interface_factory.cpp
    src/query_interface_manager.cpp
)

target_link_libraries(data_query_interface
    zmq
    yaml-cpp
    jsoncpp
    ${LIBMONGOCXX_LIBRARIES}
    ${LIBBSONCXX_LIBRARIES}
    ${LIBS}
)

target_compile_features(data_query_interface PRIVATE cxx_range_for)
add_executable(black_box_query_interface src/query_interface_main.cpp)
target_link_libraries(black_box_query_interface data_query_interface)
target_compile_features(black_box_query_interface PRIVATE cxx_range_for)
#######################################################
