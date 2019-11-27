SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

project(ubitrack_device_xsens)

cmake_minimum_required(VERSION 3.4.0)

include(CheckIncludeFile)

include(${CMAKE_CURRENT_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup(TARGETS)

include(GNUInstallDirs)
include(cmake/UbitrackConfig)

SET(LIBXSENS_XSTYPES_FILE_LIBNAME "${CMAKE_CURRENT_SOURCE_DIR}/src/XsensSensor/x64/lib/xstypes64.lib" CACHE FILEPATH "foobar")
SET(LIBXSENS_DEVICEAPI_FILE_LIBNAME "${CMAKE_CURRENT_SOURCE_DIR}/src/XsensSensor/x64/lib/xsensdeviceapi64.lib" CACHE FILEPATH "foobar")

add_subdirectory(src/XsensSensor)

ubitrack_install_utql("doc/utql/*.xml" "doc/utql/*/*.xml"  "doc/utql/*/*/*.xml")
ubitrack_install_doc("doc/utqlDoc/*" "doc/utqlDoc/*/*" "doc/utqlDoc/*/*/*")
