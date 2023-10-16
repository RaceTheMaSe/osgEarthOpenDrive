#pragma once
#include <osg/ArgumentParser>
#include <osg/Group>
#include <osgEarth/MapNode>
#include <osgEarth/Viewpoint>

osg::ref_ptr<osg::Group> LoadOdr(osg::ArgumentParser arguments, osgEarth::MapNode* mapNode);
osgEarth::Viewpoint      OdrRefPoint();
