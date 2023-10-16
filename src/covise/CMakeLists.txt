project(coviseRoad)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(OpenSceneGraph REQUIRED COMPONENTS osgSim osgTerrain PATHS ${OSGEM_OSG_INSTALL_DIR})
find_package(GDAL)
find_package(XERCESC REQUIRED)

file(GLOB_RECURSE SOURCES ./*.cpp)
file(GLOB_RECURSE HEADERS ./*.h)

add_library(${PROJECT_NAME} SHARED ${SOURCES})
add_subdirectory(thirdparty/opencrg)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${OpenSceneGraph_INCLUDE_DIRS}
    ${GDAL_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/cover
    ${CMAKE_CURRENT_SOURCE_DIR}/RoadSystem
    ${CMAKE_CURRENT_SOURCE_DIR}/RoadTerrain
    thirdparty/opencrg/inc
    ${COMMON_INCLUDE_DIR}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    ${OpenSceneGraph_LIBRARIES}
    gdal
    xerces-c
    proj
    OpenCRG
)
