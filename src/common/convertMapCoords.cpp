#include "convertMapCoords.h"
#include "ElevationSampler"
#include <iostream>
#include <iomanip>

osgEarth::GeoPoint originOdr = {};
osgEarth::GeoPoint originMap = {};
osg::Vec3d originGeographic = {};
osg::Matrixd originGeoCentric = {};
osg::Matrixd originGeocentricInv = {};
const osgEarth::SpatialReference* odr_srs = nullptr;;
const osgEarth::SpatialReference* map_srs = nullptr;
double hae = 0.0;
double deltaHeightFiddleHackShit = -20.0; // thats the fiddle part that should not be here
// double deltaHeightFiddleHackShit = 27.0; // osm to xodr maps
double deltaHeight = 0.0;
osgEarth::Contrib::ElevationSampler* elevSampler = nullptr;

void InitMapSRS(const osgEarth::SpatialReference* map) 
{ 
    map_srs = map;
}

void InitElevationSampler(osgEarth::Map* map)
{
    elevSampler = new osgEarth::Contrib::ElevationSampler(map);
}

void InitConvertMapFromOdrHeader(const double& north, const double& south, const double& east, const double& west, const char* geoReference)
{
    double midLatitude = (north+south)/2.0;
    double midLongitude = (east+west)/2.0;
    std::cout << "Mid: " << midLatitude << ", " << midLongitude << std::endl;
    const osgEarth::SpatialReference* odrSrs = osgEarth::SpatialReference::create(std::string(geoReference));
    bool someConditionToEitherInputZeroOrMidPoint = false;
    InitConvertMapCoordinates(odrSrs,map_srs,someConditionToEitherInputZeroOrMidPoint ? osg::Vec2d() : osg::Vec2d(midLongitude,midLatitude) );
}

void InitConvertMapCoordinates(const osgEarth::SpatialReference* odr, const osgEarth::SpatialReference* map, osg::Vec2d odrCenterPoint)
{
    odr_srs=odr;
    map_srs=map;
    originOdr = osgEarth::GeoPoint(odr_srs, odrCenterPoint.x(),odrCenterPoint.y(),0);
    osgEarth::Bounds b;
    odr_srs->getBounds(b);
    std::cout << "Input SRS: " << odr_srs->getDatumName() << " - " << odr_srs->getName() << " - " << odr_srs->getHorizInitString() << " - " << std::fixed << std::setprecision(2) << b.xMin()<< " - " << b.xMax() << " - "<< b.yMin() << " - "<< b.yMax() << " - " << std::endl;
    originMap = originOdr.transform(map_srs);
    originGeographic = originMap.vec3d();
    std::cout << "WGS84 origin: " << originGeographic.x() << ", " << originGeographic.y() << std::endl;
    map_srs->createLocalToWorld(originGeographic,originGeoCentric);
    originGeocentricInv = osg::Matrixd::inverse(originGeoCentric);
    hae = map_srs->getVerticalDatum()->msl2hae(originGeographic.x(),originGeographic.y(),originGeographic.z()); // thats the more likely proper conversion of msl2hae / hae2msl
    deltaHeight = hae - originGeographic.z() + deltaHeightFiddleHackShit; 
}

osg::Vec3d ConvertToMapCoordinates(const osg::Vec3d& point)
{
    osg::Vec3d coord = point;
    ConvertToMapCoordinates(coord);
    return coord;
}

osg::Vec3 ConvertToMapCoordinatesFloat(const osg::Vec3d& point)
{
    osg::Vec3d coord = point;
    ConvertToMapCoordinates(coord);
    return {static_cast<float>(coord.x()),static_cast<float>(coord.y()),static_cast<float>(coord.z())};
}

void ConvertToMapCoordinates(osg::Vec3d& p)
{
    osgEarth::GeoPoint geopoint(odr_srs, p.x(),p.y(),p.z()+deltaHeight);
    osg::Vec3d mapCoord = geopoint.transform(map_srs).vec3d();
    osg::Vec3d geoCentric;
    map_srs->transformToWorld(mapCoord,geoCentric);
    p = geoCentric * originGeocentricInv;
}

osg::Vec3f ConvertToMapCoordinatesXYZ(double x, double y, double z)
{
    return ConvertToMapCoordinatesFloat({x,y,z});
}

void ConvertToMapCoordinatesVec(osg::Vec3dArray& a)
{
    for(auto& p : a)
        ConvertToMapCoordinates(p);
}

osg::Matrixd MapGeocentricOriginMatrix(const osgEarth::SpatialReference* odr_srs, const osgEarth::SpatialReference* map_srs)
{
    osg::Matrixd originGeoCentric = {};
    map_srs->createLocalToWorld(osgEarth::GeoPoint(odr_srs, 0,0,0).transform(map_srs).vec3d(),originGeoCentric);
    return originGeoCentric;
}

osg::Matrixd MapGeocentricOriginMatrix()
{
    return originGeoCentric;
}

const osgEarth::SpatialReference* GetSourceSrs() { return odr_srs; }
const osgEarth::SpatialReference* GetDestinationSrs() { return map_srs; }
const osgEarth::GeoPoint& SourceReferencePoint() { return originOdr; }

double sampleElevation(osg::Vec3d p)
{
    return elevSampler->getElevation(odr_srs, p);
}
