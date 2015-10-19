cmake_minimum_required(VERSION 2.8.7)
project(template_view)

find_package(catkin REQUIRED COMPONENTS roscpp dolphin_slam)
find_package(Qt5Widgets REQUIRED)


find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS dolphin_slam
)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/*.h)

QT5_WRAP_CPP(QT_MOC_HPP ${QT_MOC})
QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

include_directories(include ${Qt5Widgets_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})

add_executable(template_view_node ${QT_SOURCES} ${QT_FORMS_HPP} ${QT_MOC_HPP} ${QT_MOC})
target_link_libraries(template_view_node ${QT_LIBRARIES} ${ROS_LIBRARIES} ${catkin_LIBRARIES} ${Qt5Widgets_LIBRARIES} ${Boost_LIBRARIES})

qt5_use_modules(template_view_node PrintSupport)
