cmake_minimum_required(VERSION 3.10)
project(hero_chassis_controller)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## -Wall -Werror: 警告当错误处理，配合 roslint 保证代码规范
add_definitions(-Wall -Werror)

find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        roslint
        controller_interface
        hardware_interface
        pluginlib
        control_toolbox
        realtime_tools
)

catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        ${PROJECT_NAME}
        CATKIN_DEPENDS
        roscpp
        # roslint
        controller_interface
        hardware_interface
        pluginlib
        control_toolbox
        realtime_tools
        DEPENDS
)

###########
## Build ##
###########

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

## 控制器是插件：由 controller_manager 运行时加载的共享库，
## 所以这里只有 add_library，没有 add_executable，也没有 main()
add_library(${PROJECT_NAME}
        src/hero_chassis_controller.cpp
        include/hero_chassis_controller/hero_chassis_controller.h
)

add_dependencies(${PROJECT_NAME}
        ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(
        TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
        DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
)

##########################
## Static code analysis ##
##########################

roslint_cpp(src/hero_chassis_controller.cpp)