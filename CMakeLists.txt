cmake_minimum_required(VERSION 3.16)
project(dds_pointcloud_viewer VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# Find the CycloneDDS package.
find_package(CycloneDDS REQUIRED)
find_package(CycloneDDS-CXX REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(iceoryx_binding_c REQUIRED) 
#set(glim_DIR "/home/manh/glim_ws/install/glim/share/glim/cmake")

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(glim REQUIRED)
# This is a convenience function, provided by the CycloneDDS package,
# that will supply a library target related the the given idl file.
# In short, it takes the idl file, generates the source files with
# the proper data types and compiles them into a library.
set(GENERATED_DIR ${CMAKE_BINARY_DIR}/generated)

idlcxx_generate(TARGET PointcloudDataLib 
OUTPUT_DIR ${GENERATED_DIR}
FILES 
  "idl/Slam3DIDL.idl"
  "idl/PointcloudIDL.idl"
  "idl/CommonIDL.idl"
WARNINGS no-implicit-extensibility)

include_directories(${CMAKE_BINARY_DIR} ${CMAKE_SOURCE_DIR}/include)
message("include dir: ${CMAKE_BINARY_DIR}")

### glim_ros ###
ament_auto_add_executable(publisher_node 
  src/example/publisher_node.cpp
  src/example/DDSPublisherExample.cpp
)

# Both executables need to be linked to the idl data type library and
# the ddsc API library.
target_link_libraries(publisher_node PointcloudDataLib CycloneDDS-CXX::ddscxx)

## client exaple
ament_auto_add_executable(subscriber_node 
  src/example/subscriber_node.cpp
  src/callbacks.cpp
  src/LidarMapSubscriber.cpp
)

# Both executables need to be linked to the idl data type library and
# the ddsc API library.
target_link_libraries(subscriber_node PointcloudDataLib glim::glim CycloneDDS-CXX::ddscxx)

ament_auto_add_executable(viewer_node
  src/viewer_node.cpp
  src/remote_viewer.cpp
  src/callbacks.cpp
  src/LidarMapSubscriber.cpp
)

# Both executables need to be linked to the idl data type library and
# the ddsc API library.
target_link_libraries(viewer_node PointcloudDataLib glim::glim CycloneDDS-CXX::ddscxx)

ament_auto_add_library(glim_dds_client SHARED
  src/glim_dds_client.cpp
  src/data_writer.cpp
)

# Both executables need to be linked to the idl data type library and
# the ddsc API library.
target_link_libraries(glim_dds_client PointcloudDataLib glim::glim CycloneDDS-CXX::ddscxx)
# ament
ament_auto_package()
