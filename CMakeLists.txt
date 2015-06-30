cmake_minimum_required (VERSION 2.8 FATAL_ERROR)

project (CSE395-ProjectI-Group3)

set (CMAKE_INCLUDE_CURRENT_DIR ON)
set (CMAKE_AUTOMOC ON)
set (QT_HEADERS ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR})

include (${CMAKE_ROOT}/Modules/FindQt4.cmake)
include (${QT_USE_FILE})

find_package (PCL 1.6 REQUIRED)
find_package (OpenGL REQUIRED)
find_package (GLUT REQUIRED)
find_package (Qt4 REQUIRED)

set (QtApp_SRCS main.cpp mainwindow.cpp dialog.cpp)
set (QtApp_MOCS mainwindow.h dialog.h)
set (QtApp_RCCS resources.qrc)

QT4_WRAP_CPP (QtApp_MOC_SRCS ${QtApp_MOCS})
QT4_WRAP_UI (QT_UIS mainwindow.ui dialog.ui)
QT4_ADD_RESOURCES(QtApp_RCC_SRCS ${QtApp_RCCS})

include_directories (${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR} ${QT_HEADERS} ${Qt4Widgets_INCLUDE_DIRS})
include_directories (${PCL_INCLUDE_DIRS})

link_directories (${PCL_LIBRARY_DIRS})

add_definitions (${PCL_DEFINITIONS})

add_executable (grab_points grabber.cpp rs232.c)
add_executable (concat_point  concat.cpp )
add_executable (process_points process.cpp )
add_executable (mesh_viewer mesh_viewer.cpp PLY.cpp inputModule.cpp geometry.cpp)
add_executable (scanner main.cpp mainwindow.cpp dialog.cpp ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR} ${QT_HEADERS} ${QT_UIS} ${QtApp_RCC_SRCS})


target_link_libraries (grab_points ${PCL_LIBRARIES})
target_link_libraries (concat_point ${PCL_LIBRARIES})
target_link_libraries (process_points ${PCL_LIBRARIES})
target_link_libraries (mesh_viewer ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} )
target_link_libraries (scanner ${QT_LIBRARIES} ${QT_QTMAIN_LIBRARY} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${PCL_LIBRARIES})