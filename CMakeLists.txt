CMAKE_MINIMUM_REQUIRED(VERSION 3.0) # 💡 建议将最低版本稍微提升至 3.0 以完美支持现代 OpenMP 目标
PROJECT(PA1 CXX)

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF()

# 💡 新增：让 CMake 在当前系统中自动寻找 OpenMP 多线程编译器扩展
FIND_PACKAGE(OpenMP REQUIRED)

ADD_SUBDIRECTORY(deps/vecmath)

SET(PA1_SOURCES
        src/image.cpp
        src/main.cpp
        src/mesh.cpp
        src/scene_parser.cpp)

SET(PA1_INCLUDES
        include/camera.hpp
        include/group.hpp
        include/hit.hpp
        include/image.hpp
        include/light.hpp
        include/material.hpp
        include/mesh.hpp
        include/object3d.hpp
        include/plane.hpp
        include/ray.hpp
        include/scene_parser.hpp
        include/sphere.hpp
        include/transform.hpp
        include/triangle.hpp
        )

SET(CMAKE_CXX_STANDARD 11)

ADD_EXECUTABLE(${PROJECT_NAME} ${PA1_SOURCES} ${PA1_INCLUDES})

# 💡 修改：在编译链接目标中，加入 OpenMP::OpenMP_CXX 线程库
TARGET_LINK_LIBRARIES(${PROJECT_NAME} vecmath OpenMP::OpenMP_CXX)
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} PRIVATE include)