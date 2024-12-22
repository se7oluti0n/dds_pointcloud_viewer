cmake_minimum_required(VERSION 3.16)
project(dds_pointcloud_viewer VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# Find the CycloneDDS package.
find_package(CycloneDDS-CXX REQUIRED)
find_package(iceoryx_binding_c REQUIRED) 

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# This is a convenience function, provided by the CycloneDDS package,
# that will supply a library target related the the given idl file.
# In short, it takes the idl file, generates the source files with
# the proper data types and compiles them into a library.
set(GENERATED_DIR ${CMAKE_BINARY_DIR}/generated)

idlcxx_generate(TARGET ExampleData_lib 
OUTPUT_DIR ${GENERATED_DIR}
FILES 
  "idl/HelloWorldData.idl" 
  "idl/PointCloud.idl"
WARNINGS no-implicit-extensibility)

include_directories(${CMAKE_BINARY_DIR})
message("include dir: ${CMAKE_BINARY_DIR}")

### glim_ros ###
ament_auto_add_executable(dds_example 
  src/main.cpp
  src/Example.cpp
)

# Both executables need to be linked to the idl data type library and
# the ddsc API library.
target_link_libraries(dds_example ExampleData_lib CycloneDDS-CXX::ddscxx)

## client exaple
ament_auto_add_executable(client_example 
  src/client_node.cpp
  src/ClientExample.cpp
)

# Both executables need to be linked to the idl data type library and
# the ddsc API library.
target_link_libraries(client_example ExampleData_lib CycloneDDS-CXX::ddscxx)
ament_auto_package()

