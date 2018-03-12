cmake_minimum_required(VERSION 2.8.3)
project(mapviz_grape_plugins)

find_program(CCACHE_PROGRAM cache)
if(CCACHE_PROGRAM)
    set_property(GLOBAL PROPERY RULE_LAUNCH_COMPILE "${CCACHE_PROGRAM}")
endif()

set(DEPENDENCIES
  cv_bridge
  image_transport
  mapviz
  marti_common_msgs
  marti_nav_msgs
  marti_visualization_msgs
  nav_msgs
  pluginlib 
  roscpp 
  sensor_msgs
  std_msgs 
  stereo_msgs
  swri_image_util
  swri_math_util
  swri_route_util
  swri_transform_util
  swri_yaml_util
  tf 
  visualization_msgs
  grape_msgs
  rospy
)

find_package(catkin REQUIRED COMPONENTS ${DEPENDENCIES})

set(CMAKE_INCLUDE_CURRENT_DIR ON)
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5OpenGL REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Network REQUIRED)

set(QT_LIBRARIES
  ${Qt5Core_LIBRARIES}
  ${Qt5Gui_LIBRARIES}
  ${Qt5OpenGL_LIBRARIES}
  ${Qt5Widgets_LIBRARIES}
  ${Qt5Network_LIBRARIES}
)
include_directories(
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Gui_INCLUDE_DIRS}
  ${Qt5OpenGL_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${Qt5Network_INCLUDE_DIRS}
)
add_definitions(
  ${Qt5Core_DEFINITIONS}
  ${Qt5Gui_DEFINITIONS}
  ${Qt5OpenGL_DEFINITIONS}
  ${Qt5Widgets_DEFINITIONS}
  ${Qt5Network_DEFINITIONS}
)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")

find_package(GLU REQUIRED)
find_package(GLUT REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  DEPENDS Qt5Core Qt5Gui Qt5OpenGL Qt5Widgets
  CATKIN_DEPENDS ${DEPENDENCIES}
)

# Fix conflict between Boost signals used by tf and QT signals used by mapviz
add_definitions(-DQT_NO_KEYWORDS -std=c++11)

include_directories(include
    ${catkin_INCLUDE_DIRS}
)

set(UI_FILES
    ui/grape_plan_route.ui
    ui/husky_image_config.ui

    ui/tile_map_config.ui
)

set(SRC_FILES
    src/plan_route_plugin.cpp
    src/husky_image_plugin.cpp

    src/image_cache.cpp
    src/texture_cache.cpp
    src/tile_source.cpp
    src/tile_map_view.cpp
    src/wmts_source.cpp
    src/tile_map_plugin.cpp
    )

set(HEADER_FILES
    include/grape_plugins/plan_route_plugin.h
    include/grape_plugins/husky_image_plugin.h

    include/grape_tile_map/image_cache.h
    include/grape_tile_map/tile_source.h
    include/grape_tile_map/wmts_source.h
    include/grape_tile_map/tile_map_plugin.h
)
    
qt5_add_resources(RCC_SRCS src/resources/grape_icons.qrc)
qt5_wrap_ui(UI_SRC_FILES ${UI_FILES})
qt5_wrap_cpp(MOC_FILES   ${HEADER_FILES})

add_library(${PROJECT_NAME}
    ${MOC_FILES}
    ${SRC_FILES}
    ${UI_SRC_FILES}
    ${RCC_SRCS}
)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${GLU_LIBRARY}
    ${GLUT_LIBRARY}
    ${QT_LIBRARIES}
    ${QT_QTOPENGL_LIBRARIES}
)
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-std=c++11 -D__STDC_FORMAT_MACROS")

add_dependencies(${PROJECT_NAME} marti_visualization_msgs_generate_messages_cpp)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES ${PROJECT_NAME}.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


