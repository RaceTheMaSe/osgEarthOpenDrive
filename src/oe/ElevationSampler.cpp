#include "ElevationSampler"
#include <osgEarth/ElevationLayer>

using namespace osgEarth;
using namespace osgEarth::Contrib;

#define LC "[ElevationSampler] "

ElevationSampler::ElevationSampler(Map* map)
{
    _pool = new ElevationPool();
    _pool->setMap(map);
    // Collect all elevation layers
    ElevationLayerVector layers;
    map->getLayers(layers);
    for (ElevationLayerVector::iterator i = layers.begin(); i != layers.end(); ++i) {
        // Skip flatteningLayer
        if (std::string(i->get()->className()).find("FlatteningLayer")!=std::string::npos) {
            layers.erase(i);
            break;
        }
        else {
            OE_INFO << LC << "Using: " << i->get()->getName() << "\n";
        }
    }
    if (!layers.empty())
    {
        _elevWorkingSet.setElevationLayers(layers);
    }
}

double ElevationSampler::getElevation(const SpatialReference* geomSRS, const osg::Vec3d& p)
{
    GeoPoint EP(geomSRS, p.x(), p.y(), 0);
    Distance elevSample = _pool->getSample(EP, &_elevWorkingSet).elevation();
    return elevSample.getValue();
}
