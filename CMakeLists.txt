cmake_minimum_required(VERSION 2.9)
project(osgEarthOpenDrive LANGUAGES C CXX)

# hacky way of providing external references
if(NOT OSGEM_OSG_INSTALL_DIR)
    message(FATAL_ERROR "Provide OSGEM_OSG_INSTALL_DIR referring to the OSG install directory")
endif()
set(OSGEM_OSG_LIBRARY_DIR "${OSGEM_OSG_INSTALL_DIR}/lib")
if(NOT OSGEM_OSGEARTH_INSTALL_DIR)
    message(FATAL_ERROR "Provide OSGEM_OSGEARTH_INSTALL_DIR referring to the osgEarth install directory")
endif()
if(NOT OSGEM_OSGEARTH_SOURCE_DIR)
    message(FATAL_ERROR "Provide OSGEM_OSGEARTH_INSTALL_DIR referring to the osgEarth source directory")
endif()

FIND_PACKAGE(OpenSceneGraph REQUIRED COMPONENTS osg osgUtil osgDB osgGA osgViewer osgSim osgTerrain PATHS ${OSGEM_OSG_INSTALL_DIR})
FIND_PACKAGE(osgEarth REQUIRED PATHS ${OSGEM_OSGEARTH_INSTALL_DIR}/cmake)
FIND_PACKAGE(GLEW REQUIRED) # ImGUI

file(GLOB_RECURSE SOURCES_COMMON src/common/*.cpp)
file(GLOB_RECURSE SOURCES_ESMINI src/esMini/*.cpp)
file(GLOB_RECURSE SOURCES_OEARTH src/oe/*.cpp)
set(SOURCES ${SOURCES_COMMON})
list(APPEND SOURCES ${SOURCES_ESMINI} ${SOURCES_OEARTH})

# hacky way of getting ImGUI to build and link
#set(IMGUI_HEADERS_DIR /usr/include/imgui) # t./hat would be the unix default dir, but versions are different resulting in compile error, so take the osgEarth provided
set(IMGUI_HEADERS_DIR ${OSGEM_OSGEARTH_SOURCE_DIR}/src/third_party/imgui)
file(GLOB SOURCES_IMGUI_BACKEND  ${IMGUI_HEADERS_DIR}/backends/*opengl*.cpp)
file(GLOB SOURCES_IMGUI  ${IMGUI_HEADERS_DIR}/*.cpp)
list(APPEND SOURCES ${SOURCES_IMGUI} ${SOURCES_IMGUI} ${SOURCES_IMGUI_BACKEND} ${OSGEM_OSGEARTH_SOURCE_DIR}/src/osgEarth/ImGui/ImGui.cpp)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(LINK_LIBS_LOCAL 
    ${OpenSceneGraph_LIBRARIES_DEBUG}
    ${osgEarth_LIBRARIES_DEBUG})
else()
    set(LINK_LIBS_LOCAL
    ${OpenSceneGraph_LIBRARIES}
    ${osgEarth_LIBRARIES})
endif()

add_executable(${PROJECT_NAME} ${SOURCES})

target_compile_options(${PROJECT_NAME} PUBLIC -Wno-format-security -Wno-format) # ImGUI emits many of those
target_include_directories(${PROJECT_NAME} PRIVATE
    src/common
    src/covise
    src/covise/cover
    src/esMini
    src/esMini/pugixml
    src/oe
    ${osgEarth_INCLUDE_DIRS}
    ${IMGUI_HEADERS_DIR}
    ${OpenSceneGraph_INCLUDE_DIRS}
    )
target_link_libraries(${PROJECT_NAME} PUBLIC
   ${LINK_LIBS_LOCAL}
   # covise:
   xerces-c
   coviseRoad
   # ImGUI:
   GL
   GLEW
    )
target_link_directories(${PROJECT_NAME} PUBLIC ${OSGEM_OSG_LIBRARY_DIR})
set(COMMON_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/common)
add_subdirectory(src/covise)
