cmake_minimum_required(VERSION 2.8.3)
project(geomagic_touch_m)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs tf)

add_message_files(FILES GeomagicButtonEvent.msg HapticState.msg)
generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs tf)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

add_executable(geomagic_touch_m src/geomagic_touch_m.cpp)

add_dependencies(geomagic_touch_m geomagic_touch_m_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

target_link_libraries(geomagic_touch_m HD HDU rt ncurses ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME} geomagic_touch_m
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )
