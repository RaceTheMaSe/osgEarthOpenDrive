#pragma once
#include "convertMapCoords.h"

#include <osgEarth/GeoData>
#include <osgEarth/SpatialReference>
#include <osgEarth/Map>

void InitMapSRS(const osgEarth::SpatialReference* mapSrs);
void InitConvertMapFromOdrHeader(const double& north, const double& south, const double& east, const double& west, const char* geoReference);
void InitConvertMapCoordinates(const osgEarth::SpatialReference* odr, const osgEarth::SpatialReference* map, osg::Vec2d input_center={});
void ConvertToMapCoordinates(osg::Vec3d& p);
extern osg::Vec3d ConvertToMapCoordinates(const osg::Vec3d& p); // wrapper function for the above
extern osg::Vec3 ConvertToMapCoordinatesFloat(const osg::Vec3d& p); // wrapper function for the above
void ConvertToMapCoordinatesVec(osg::Vec3dArray& a);
osg::Vec3f ConvertToMapCoordinatesXYZ(double x, double y, double z);
osg::Matrixd MapGeocentricOriginMatrix(const osgEarth::SpatialReference* odr_srs, const osgEarth::SpatialReference* map_srs);
osg::Matrixd MapGeocentricOriginMatrix();
const osgEarth::SpatialReference* GetSourceSrs();
const osgEarth::SpatialReference* GetDestinationSrs();
const osgEarth::GeoPoint& SourceReferencePoint();

void InitElevationSampler(osgEarth::Map* map);
double sampleElevation(osg::Vec3d p); // 2d vector would be sufficient

extern double deltaHeightFiddleHackShit;
extern double deltaHeight;
