##
##     Filename:  scripts/CMakeLists.txt
##     Author:    Mike Timmerman
##     Version:   1.0
##     Date:      2022
##


# Add PIDattitudeControl.cpp

add_library(PIDattitudeControl PIDattitudeControl.cpp)

target_include_directories(PIDattitudeControl
    PUBLIC ${CMAKE_SOURCE_DIR}/libraries/eigen
    PUBLIC ${CMAKE_SOURCE_DIR}/src/actuator
    PUBLIC ${CMAKE_SOURCE_DIR}/src/helpers
    PUBLIC ${CMAKE_SOURCE_DIR}/src/PIDcontroller
    PUBLIC ${CMAKE_SOURCE_DIR}/src/INDIcontroller
    PUBLIC ${CMAKE_SOURCE_DIR}/src/controller
    PUBLIC ${CMAKE_SOURCE_DIR}/src/sensor
    PUBLIC ${CMAKE_SOURCE_DIR}/src/saturator
    PUBLIC ${CMAKE_SOURCE_DIR}/src/filter
    PUBLIC ${CMAKE_SOURCE_DIR}/src/estimator
)

target_link_directories(PIDattitudeControl
    PUBLIC ${CMAKE_SOURCE_DIR}/libraries/eigen
    PUBLIC ${CMAKE_SOURCE_DIR}/src/actuator
    PUBLIC ${CMAKE_SOURCE_DIR}/src/helpers
    PUBLIC ${CMAKE_SOURCE_DIR}/src/PIDcontroller
    PUBLIC ${CMAKE_SOURCE_DIR}/src/INDIcontroller
    PUBLIC ${CMAKE_SOURCE_DIR}/src/controller
    PUBLIC ${CMAKE_SOURCE_DIR}/src/sensor
    PUBLIC ${CMAKE_SOURCE_DIR}/src/saturator
    PUBLIC ${CMAKE_SOURCE_DIR}/src/filter
    PUBLIC ${CMAKE_SOURCE_DIR}/src/estimator
)

target_link_libraries(PIDattitudeControl eigen actuator helpers PIDcontroller INDIcontroller controller sensor saturator estimator filter)