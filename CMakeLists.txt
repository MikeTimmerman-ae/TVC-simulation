cmake_minimum_required(VERSION 3.0.0)
project(Simulator VERSION 0.1.0)

include(CTest)
enable_testing()

add_executable(Simulator main.cpp)

add_subdirectory(src)
add_subdirectory(scripts)
add_subdirectory(libraries/eigen)

target_include_directories(${PROJECT_NAME}
    PUBLIC src
    PUBLIC scripts
    PUBLIC libraries/eigen
)

target_link_directories(${PROJECT_NAME}
    PUBLIC src
    PUBLIC scripts
    PUBLIC libraries/eigen
)

target_link_libraries(${PROJECT_NAME} eigen dynamics PIDcontroller INDIcontroller controller actuator filter saturator sensor helpers PIDattitudeControl)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)