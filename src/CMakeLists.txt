##
##     Filename:  src/CMakeLists.txt
##     Author:    Mike Timmerman
##     Version:   1.0
##     Date:      2022
##


# Add dynamics.cpp

add_library(dynamics dynamics.cpp)

target_include_directories(dynamics
    PUBLIC ${CMAKE_SOURCE_DIR}/libraries/eigen
)

target_link_directories(dynamics
    PUBLIC ${CMAKE_SOURCE_DIR}/libraries/eigen
)

target_link_libraries(dynamics eigen)


# Add helpers.cpp

add_library(helpers helpers.cpp)

target_include_directories(helpers
    PUBLIC ${CMAKE_SOURCE_DIR}/libraries/eigen
)

target_link_directories(helpers
    PUBLIC ${CMAKE_SOURCE_DIR}/libraries/eigen
)

target_link_libraries(helpers eigen)


# Add PIDcontroller.cpp

add_library(PIDcontroller PIDcontroller.cpp)

target_include_directories(PIDcontroller
    PUBLIC ${CMAKE_SOURCE_DIR}/libraries/eigen
)

target_link_directories(PIDcontroller
    PUBLIC ${CMAKE_SOURCE_DIR}/libraries/eigen
)

target_link_libraries(PIDcontroller eigen)


# Add filter.cpp

add_library(filter filter.cpp)

target_include_directories(filter
    PUBLIC ${CMAKE_SOURCE_DIR}/libraries/eigen    
)

target_link_directories(filter
    PUBLIC ${CMAKE_SOURCE_DIR}/libraries/eigen
)

target_link_libraries(filter eigen)


# Add saturator.cpp

add_library(saturator saturator.cpp)

target_include_directories(saturator
    PUBLIC ${CMAKE_SOURCE_DIR}/libraries/eigen    
)

target_link_directories(saturator
    PUBLIC ${CMAKE_SOURCE_DIR}/libraries/eigen
)

target_link_libraries(saturator eigen)


# Add controller.cpp

add_library(controller controller.cpp)

target_include_directories(controller
    PUBLIC ${CMAKE_SOURCE_DIR}/libraries/eigen
)

target_link_directories(controller
    PUBLIC ${CMAKE_SOURCE_DIR}/libraries/eigen
)

target_link_libraries(controller eigen)


# Add actuator.cpp

add_library(actuator actuator.cpp)

target_include_directories(actuator
    PUBLIC ${CMAKE_SOURCE_DIR}/libraries/eigen    
)

target_link_directories(actuator
    PUBLIC ${CMAKE_SOURCE_DIR}/libraries/eigen
)

target_link_libraries(actuator eigen)


# Add sensor.cpp

add_library(sensor sensor.cpp)

target_include_directories(sensor
    PUBLIC ${CMAKE_SOURCE_DIR}/libraries/eigen    
)

target_link_directories(sensor
    PUBLIC ${CMAKE_SOURCE_DIR}/libraries/eigen
)

target_link_libraries(sensor eigen)


# Add INDIcontroller.cpp

add_library(INDIcontroller INDIcontroller.cpp)

target_include_directories(INDIcontroller
    PUBLIC ${CMAKE_SOURCE_DIR}/libraries/eigen    
)

target_link_directories(INDIcontroller
    PUBLIC ${CMAKE_SOURCE_DIR}/libraries/eigen
)

target_link_libraries(INDIcontroller eigen)


# Add estimator.cpp

add_library(estimator estimator.cpp)

target_include_directories(estimator
    PUBLIC ${CMAKE_SOURCE_DIR}/libraries/eigen    
)

target_link_directories(estimator
    PUBLIC ${CMAKE_SOURCE_DIR}/libraries/eigen
)

target_link_libraries(estimator eigen)

