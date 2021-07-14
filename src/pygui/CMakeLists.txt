# support for the package so this needs to be "quiet".
set(Python_ADDITIONAL_VERSIONS 3)

find_package(PythonInterp 3)
find_package(PythonLibs 3)
if (PythonLibs_FOUND)
     message(STATUS "Found python at ${PYTHON_EXECUTABLE} VERSION ${PYTHON_VERSION_STRING}")
     message(STATUS "Found python lib at ${PYTHON_LIBRARIES}")
     message(STATUS "Found python include at ${PYTHON_INCLUDE_DIRS}")
endif()
if(DEFINED ENV{ENABLE_PYGUI})
   set(ENABLE_PYGUI 1)
endif()


set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
#set (CMAKE_BUILD_TYPE Debug)

if (PythonLibs_FOUND AND ENABLE_PYGUI)
	message(STATUS "PythonLibs Found")
	message(STATUS "Python GUI is enabled")

  add_library(pygui
      src/pygui.cpp
  )

  add_library(oruiobserver
      src/openroadPyIntf.cpp
      src/openroadIntf.cpp
      src/openroadGeom.cpp
      src/openroadView.cpp
      src/openroadCanvas.cpp
      src/openroadLayer.cpp
      src/openroadShape.cpp
      src/openroadTransform.cpp
      src/openroadMotion.cpp
      src/openroadLayoutContext.cpp
      src/staGui.cpp
  )
  option(BUILD_PYTHON "Build Python SWIG module for pygui" ON)



add_custom_target(
    pyguipython  ALL
    ${CMAKE_COMMAND} -E remove_directory ${CMAKE_CURRENT_BINARY_DIR}/pyqtUI
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${PYGUI_HOME}/pyqtUI ${CMAKE_CURRENT_BINARY_DIR}/pyqtUI
    SOURCES
    ${PYGUI_HOME}/pyqtUI/openroadUI/hierBrowser.py
    ${PYGUI_HOME}/pyqtUI/openroadUI/layoutViewer.py
    ${PYGUI_HOME}/pyqtUI/openroadUI/layerViewerNew.py
    ${PYGUI_HOME}/pyqtUI/openroadUI/mainwindow.py
    ${PYGUI_HOME}/pyqtUI/openroadUI/openroad_rc.py
    ${PYGUI_HOME}/pyqtUI/openroadUI/openroad.qrc
    ${PYGUI_HOME}/pyqtUI/openroadUI/openroadGlobals.py
    ${PYGUI_HOME}/pyqtUI/openroadUI/openRoadTclBridge.py
    ${PYGUI_HOME}/pyqtUI/openroadUI/scriptWidget.py
)

target_include_directories(oruiobserver
  PUBLIC
    ${PYGUI_HOME}/include
    ${OPENDB_HOME}/include
    ${OPENSTA_HOME}/include
    ${DBSTA_HOME}/include

  PRIVATE
    ${OPENROAD_HOME}/include
    ${Boost_INCLUDE_DIRS}
    ${PYTHON_INCLUDE_DIRS}
    ${TCL_INCLUDE_PATH}
)

target_link_libraries(oruiobserver
    opendb
    GL
    GLU
)

target_include_directories(pygui
  PUBLIC
    ${PYGUI_HOME}/include
    ${OPENDB_HOME}/include

  PRIVATE
    ${OPENROAD_HOME}/include
    ${Boost_INCLUDE_DIRS}
    ${PYTHON_INCLUDE_DIRS}
)

target_link_libraries(pygui
)

find_package(SWIG 3.0 REQUIRED)

include(${SWIG_USE_FILE})

if (BUILD_PYTHON)
    add_subdirectory(src/swig/python)
endif()

else()
  message(STATUS "Python-GUI is not enabled")
  add_library(pygui src/stub.cpp)
  option(BUILD_PYTHON "Build Python SWIG module for pygui" OFF)
endif()


target_include_directories(pygui
  PUBLIC
    ${PYGUI_HOME}/include
  PRIVATE
    ${OPENROAD_HOME}/include
    ${GUI_HOME}/include
    ${Boost_INCLUDE_DIRS}
    ${PYTHON_INCLUDE_DIRS}
    ${TCL_INCLUDE_PATH}
)
