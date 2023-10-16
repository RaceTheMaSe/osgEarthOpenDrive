#include "ODRFeatureSource"
#include <osgEarth/GeometryUtils>
#include <osgEarth/FeatureCursor>
#include <osgEarth/Filter>

#include <osgEarth/Registry>
#include <osgEarth/StringUtils>

#define LC "[ODRFeatureSource] "

using namespace osgEarth;

//........................................................................

Config ODRFeatureSource::Options::getConfig() const
{
    Config conf = FeatureSource::Options::getConfig();
    return conf;
}

void ODRFeatureSource::Options::fromConfig(const Config& conf)
{
}

//........................................................................

REGISTER_OSGEARTH_LAYER(odrfeatures, ODRFeatureSource);

ODRFeatureSource::~ODRFeatureSource()
{
    close();
}

Status ODRFeatureSource::create(FeatureList&& features)
{
    if (_features.size())
    {
        OE_WARN << "Create called on an non-empty ODRFeatureSource" << std::endl;
        return Status::Code::AssertionFailure;
    }
    _features = std::move(features);
    return Status::Code::NoError;
}

FeatureCursor* ODRFeatureSource::createFeatureCursorImplementation(const Query& query, ProgressCallback* progress)
{
    return new FeatureListCursor(_features);
}

int ODRFeatureSource::getFeatureCount() const
{
    return _features.size();
}

bool ODRFeatureSource::supportsGetFeature() const
{
    return true;
}

Feature* ODRFeatureSource::getFeature(FeatureID fid)
{
    for (auto& f : _features)
    {
        if (f && f->getFID() == fid)
            return f;
    }
    return nullptr;
}

bool ODRFeatureSource::insertFeature(Feature* feature)
{
    if (feature == nullptr)
        return false;
    if (getFeature(feature->getFID()) == nullptr)
        _features.push_back(feature);
    return true;
}

osgEarth::Geometry::Type ODRFeatureSource::getGeometryType() const
{
    return osgEarth::Geometry::TYPE_POLYGON;
}
