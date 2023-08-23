cmake_minimum_required(VERSION 3.5)
project(octree_py)

set(CMAKE_CXX_STANDARD 20)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Python REQUIRED COMPONENTS Interpreter Development)
find_package(PythonLibs 3 REQUIRED)
find_package(pybind11 REQUIRED)
find_package(TBB REQUIRED)
list(APPEND CMAKE_MODULE_PATH "/usr/include/tbb/")

pybind11_add_module(octree_py src/octree_py.cpp)
target_include_directories(octree_py PUBLIC Octree)
ament_target_dependencies(octree_py PUBLIC
  pybind11
  Python3
)
target_link_libraries(octree_py PUBLIC TBB::tbb)
set_target_properties(octree_py PROPERTIES LINKER_LANGUAGE CXX)

# Add a hook for colcon to add the Python library directory to the PYTHONPATH environment variable
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/hook/python_path.dsv
     "prepend-non-duplicate;PYTHONPATH;lib/python3/dist-packages")
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/hook/python_path.dsv
        DESTINATION share/${PROJECT_NAME}/hook)

install(
  TARGETS octree_py
  DESTINATION lib/python3/dist-packages/${PROJECT_NAME}
)

ament_export_dependencies(pybind11)
ament_export_dependencies(Python3)

ament_package()
