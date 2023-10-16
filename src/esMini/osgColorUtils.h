#pragma once
#include <osg/Vec4>
#include "RoadManager.hpp"

inline osg::Vec4 ODR2OSGColor(roadmanager::RoadMarkColor color)
{
    osg::Vec4 osgc;

    if (color == roadmanager::RoadMarkColor::YELLOW)
    {
        osgc.set(0.9f, 0.9f, 0.25f, 1.0f);
    }
    else if (color == roadmanager::RoadMarkColor::GREEN)
    {
        osgc.set(0.2f, 0.8f, 0.4f, 1.0f);
    }
    else if (color == roadmanager::RoadMarkColor::RED)
    {
        osgc.set(0.95f, 0.4f, 0.3f, 1.0f);
    }
    else if (color == roadmanager::RoadMarkColor::BLUE)
    {
        osgc.set(0.2f, 0.5f, 0.9f, 1.0f);
    }
    else
    {
        osgc.set(0.95f, 0.95f, 0.92f, 1.0f);
    }

    return osgc;
}
