#pragma once
#include <osg/Vec3d>

extern osg::Vec3d ConvertToMapCoordinates(const osg::Vec3d& p);
extern osg::Vec3 ConvertToMapCoordinatesFloat(const osg::Vec3d& p);
extern void InitConvertMapFromOdrHeader(const double& north, const double& south, const double& east, const double& west, const char* geoReference);
extern double sampleElevation(osg::Vec3d p); // 2d would be sufficient
