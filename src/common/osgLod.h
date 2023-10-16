#pragma once
#include <osg/Group>
#include <osg/LOD>

// move somewhere else:
const float LOD_MIN_PIXEL=1.0F;
const float LOD_MAX_PIXEL=2000000.0F;

inline void AddNodeWithLod(osg::Group* to, osg::Node* node)
{
    auto* lod = new osg::LOD;
    lod->setRangeMode(osg::LOD::PIXEL_SIZE_ON_SCREEN);
    to->addChild(lod);
    lod->addChild(node,LOD_MIN_PIXEL,LOD_MAX_PIXEL);
}
