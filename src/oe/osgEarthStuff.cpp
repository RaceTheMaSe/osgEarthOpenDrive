#include "osgEarthStuff.h"
// osgEarth - base
#include <osgEarth/MapNode>
#include <osgEarth/GeoTransform>
// osgEarth - layers
#include "FlatteningLayerAlternative"
#include <osgEarth/FeatureModelLayer>
#include <osgEarth/ModelLayer>
#include <osgEarth/TerrainConstraintLayer>
#include <osgEarth/TMS>
#include <osgEarth/XYZ>
// osgEarth - cache
#include <osgEarth/Registry>
#include <osgEarthDrivers/cache_filesystem/FileSystemCache>
#include <osgEarth/CacheSeed>
// osg
#include <osg/ArgumentParser>
#include <osg/PositionAttitudeTransform>
#include <osgDB/FileUtils>

using namespace osgEarth;
using namespace osgEarth::Util;

// covise:
#include "fasi.h"
// esmini
#include "RoadManager.hpp"
#include "roadgeom.hpp"
std::unique_ptr<RoadGeom> roadGeom;
// common:
#include "ODRFeatureSource"
#include "convertMapCoords.h"
std::map<long long, osg::ref_ptr<osg::Vec3dArray>> roadOutlines = {};
// osg::ref_ptr<const Profile> wgs84profile = Profile::create(Profile::GLOBAL_GEODETIC);
osg::ref_ptr<const Profile> wgs84profile = Profile::create_with_vdatum("global-geodetic","egm84");
// vertical datum is still an issue ... egm84 and egm96 give same results in a location in Germany, egm2008 is ~1,5m different there ... not sure whats correct
// with the test map, there is no vertical datum in the init string but the difference is the difference between msl and hae plus something

// Cache
struct CacheSeedProgressReporter : public osgEarth::ProgressCallback
{
    CacheSeedProgressReporter() : _first(true), _start(0)
    {
    }

    bool reportProgress(double current, double total, unsigned int /*currentStage*/, unsigned int /*totalStages*/, const std::string & /*msg*/)
    {
        ScopedMutexLock lock(_mutex);

        if (_first)
        {
            _first = false;
            _start = osg::Timer::instance()->tick();
        }
        osg::Timer_t now = osg::Timer::instance()->tick();

        if (total > 0.0)
        {
            double percentage = current / total;

            double timeSoFar          = osg::Timer::instance()->delta_s(_start, now);
            double projectedTotalTime = timeSoFar / percentage;
            double timeToGo           = projectedTotalTime - timeSoFar;

            constexpr double secondsPerMinute = 60.0;
            constexpr double secondsPerHour   = secondsPerMinute * 60.0;
            constexpr double secondsPerDay    = secondsPerHour * 24.0;

            double timeToGoRemainder = timeToGo;
            double daysToGo          = timeToGoRemainder / secondsPerDay;
            timeToGoRemainder        = fmod(timeToGoRemainder, secondsPerDay);
            double hoursToGo         = timeToGoRemainder / secondsPerHour;
            timeToGoRemainder        = fmod(timeToGoRemainder, secondsPerHour);
            double minsToGo          = timeToGoRemainder / secondsPerMinute;
            double secsToGo          = fmod(timeToGoRemainder, secondsPerMinute);

            double projectedTotalTimeRemainder = projectedTotalTime;
            double daysTotal                   = projectedTotalTimeRemainder / secondsPerDay;
            projectedTotalTimeRemainder        = fmod(projectedTotalTimeRemainder, secondsPerDay);
            double hoursTotal                  = projectedTotalTimeRemainder / secondsPerHour;
            projectedTotalTimeRemainder        = fmod(projectedTotalTimeRemainder, secondsPerHour);
            double minsTotal                   = projectedTotalTimeRemainder / secondsPerMinute;
            double secsTotal                   = fmod(projectedTotalTimeRemainder, secondsPerMinute);

            std::cout << std::fixed << std::setprecision(1) << "\r" << static_cast<int>(current) << "/" << static_cast<int>(total) << " "
                      << int(100.0 * percentage) << "% complete, " << static_cast<int>(daysTotal) << "d" << static_cast<int>(hoursTotal) << "h"
                      << static_cast<int>(minsTotal) << "m" << static_cast<int>(secsTotal) << "s projected, " << static_cast<int>(daysToGo) << "d"
                      << static_cast<int>(hoursToGo) << "h" << static_cast<int>(minsToGo) << "m" << static_cast<int>(secsToGo)
                      << "s remaining          " << std::flush;

            if (percentage >= 100.0)
                std::cout << std::endl;
        }
        else
        {
            double timeSoFar = osg::Timer::instance()->delta_s(_start, now);

            std::cout << std::fixed << std::setprecision(1) << "\r" << static_cast<int>(current) << "/"
                      << " " << timeSoFar << "s elapsed" << std::flush;
        }

        return false;
    }

    Threading::Mutex _mutex;
    bool             _first;
    osg::Timer_t     _start;
};

void initCache(bool cacheOnly)
{
    std::string cacheDir = osgDB::getCurrentWorkingDirectory() + "/cache/";
    OE_INFO << "Cache dir: " << cacheDir << std::endl;
    osgEarth::Registry *oe_registry = osgEarth::Registry::instance();
    if (oe_registry)
    {
        if (cacheOnly)
            oe_registry->setDefaultCachePolicy(CachePolicy(CachePolicy::Usage::USAGE_READ_ONLY));
        else
            oe_registry->setDefaultCachePolicy(CachePolicy(CachePolicy::Usage::USAGE_READ_WRITE));
        auto       &cp        = oe_registry->defaultCachePolicy();
        const char *cachePath = ::getenv(OSGEARTH_ENV_CACHE_PATH);
        if (cachePath == nullptr)  // not sure if possible to modify environment variables
            ::setenv(OSGEARTH_ENV_CACHE_PATH, cacheDir.c_str(), 1);
        cachePath    = ::getenv(OSGEARTH_ENV_CACHE_PATH);
        Cache *cache = oe_registry->getDefaultCache();
        if (cache)
        {
            osgEarth::Status status = cache->getStatus();
            if (status != osgEarth::Status::OK())
                OE_WARN << "Cache error: " << status.toString();
        }
        OE_INFO << "Cache active: " << int(cache != nullptr) << " status: " << cp.get().usageString()
                << " driver: " << oe_registry->getDefaultCacheDriverName() << std::endl;
        if (!cache)
        {
            OE_INFO << "Add OSGEARTH_CACHE_PATH environment variable, so cache gets activated" << std::endl;
        }
    }
}

void seedMapCache(MapNode *mapNode, GeoExtent bounds)
{
    osg::ref_ptr<TileVisitor> visitor;
    // Create a single thread visitor
    visitor = new TileVisitor();
    if (true /*verbose*/)
        visitor->setProgressCallback(new CacheSeedProgressReporter());
    visitor->addExtentToVisit(bounds);
    // Initialize the seeder
    Contrib::CacheSeed seeder;
    seeder.setVisitor(visitor.get());

    TileLayerVector terrainLayers;
    Map            *map = mapNode->getMap();
    map->getLayers(terrainLayers);

    // Seed all the map layers
    for (unsigned int i = 0; i < terrainLayers.size(); ++i)
    {
        osg::ref_ptr<TileLayer> layer = terrainLayers[i].get();
        OE_NOTICE << "Seeding layer" << layer->getName() << std::endl;
        osg::ref_ptr<ImageLayer> imagelayer = dynamic_cast<ImageLayer *>(layer.get());
        if (imagelayer)  // elevations are rather sparse data in this example - images are more heavy, so restrict what is loaded
        {
            visitor->setMinLevel(0);
            visitor->setMaxLevel(13);
        }
        else
        {
            visitor->setMinLevel(-1);
            visitor->setMaxLevel(-1);
        }
        osg::Timer_t start = osg::Timer::instance()->tick();
        seeder.run(layer.get(), map);
        osg::Timer_t end = osg::Timer::instance()->tick();
        if (true /*verbose*/)
        {
            OE_NOTICE << "Completed seeding layer " << layer->getName() << " in " << prettyPrintTime(osg::Timer::instance()->delta_s(start, end))
                      << std::endl;
        }
    }
}

// map layers
void addBaseMaps(Map *map, bool cacheOnly)
{
    OE_INFO << "Adding base maps..." << std::endl;
    TMSElevationLayer *elevation = new TMSElevationLayer();
    //elevation->setURL("http://readymap.org/readymap/tiles/1.0.0/116/"); // activate this line for - probably SRTM based - elevation data provided by Pelican Mapping - they say its free for osgEarth developers, which probably applies here.
    map->addLayer(elevation);
    XYZImageLayer *osm = new XYZImageLayer();
    osm->setURL("http://[abc].tile.openstreetmap.org/{z}/{x}/{y}.png");
    osm->setProfile(Profile::create(Profile::SPHERICAL_MERCATOR));
    osm->setOpacity(0.5f);
    map->addLayer(osm);
}

void addFlattenLayer(Map *map, FeatureSource *features)
{
    OE_INFO << "Adding flattening / smooth transition layer..." << std::endl;
    Contrib::FlatteningLayerAlternative *flatten = new Contrib::FlatteningLayerAlternative();
    flatten->options().fill() = false;
    flatten->options().maxDataLevel() = 23; // not sure ... has no dataExtend but has to be set to something due to some lod calculations around map layers in elevationPool of osgEarth
    flatten->setLineWidth(1.0);
    flatten->setBufferWidth(10.0);
    flatten->setFeatureSource(features);
    flatten->options().minLevel() = 14;
    flatten->options().maxLevel() = 18;
    flatten->options().set_name("FlattenToFeatures");
    map->addLayer(flatten);
}

void addCutoutLayer(Map *map, FeatureSource *features)
{
    OE_INFO << "Adding terrain cut out layer..." << std::endl;
    TerrainConstraintLayer *cutOutLayer = new TerrainConstraintLayer();
    cutOutLayer->setFeatureSource(features);
    cutOutLayer->setHasElevation(true);
    cutOutLayer->setRemoveInterior(true);
    cutOutLayer->setMinLevel(0);
    cutOutLayer->setLineWidth(1.0);
    cutOutLayer->setBufferWidth(55.0);
    cutOutLayer->setFillElevation(false);
    cutOutLayer->setVisible(true);
    cutOutLayer->options().set_name("CutOutFeatures");
    map->addLayer(cutOutLayer);
}

void addDrawFeaturesLayer(Map *map, FeatureSource *features)
{
    OE_INFO << "Adding outline draw layer..." << std::endl;
    Style       style;
    LineSymbol *ls        = style.getOrCreateSymbol<LineSymbol>();
    ls->stroke()->color() = Color::Red;
    ls->stroke()->width() = 2.0f;
    ls->tessellationSize()->set(100, Units::KILOMETERS);
    AltitudeSymbol *alt              = style.getOrCreate<AltitudeSymbol>();
    alt->clamping()                  = alt->CLAMP_NONE;     // or CLAMP_ABSOLUTE
    alt->technique()                 = alt->TECHNIQUE_GPU;  // should not be relevant
    RenderSymbol *render             = style.getOrCreate<RenderSymbol>();
    render->depthOffset()->enabled() = true;
    StyleSheet *styleSheet           = new StyleSheet();
    styleSheet->addStyle(style);

    FeatureModelLayer *featureLayer       = new FeatureModelLayer();
    featureLayer->options().cachePolicy() = CachePolicy::USAGE_NO_CACHE;
    featureLayer->setFeatureSource(features);
    featureLayer->setStyleSheet(styleSheet);
    featureLayer->options().set_name("FeatureModels");
    featureLayer->open();
    map->addLayer(featureLayer);
}

void addModelLayer(osg::ref_ptr<osg::Node> model, const GeoPoint &srsBase, Map *map, MapNode *mapNode, bool asMapLayer)
{
    if (model.valid())
    {
        if (map->getSRS() && map->getSRS()->valid())
        {
            OE_INFO << "Adding OpenDrive road model mesh " << (asMapLayer ? "as a map model layer" : "fixed") << std::endl;
            osg::PositionAttitudeTransform *pat = new osg::PositionAttitudeTransform();
            if (asMapLayer)
            {
                GeoTransform *xform = new GeoTransform();
                xform->setPosition(srsBase);
                xform->addChild(pat);
                ModelLayer *layer = new ModelLayer();
                layer->setNode(xform);
                map->addLayer(layer);
            }
            else
            {
                // deactivated as the transform is already in the model scene graphs
                // osg::Matrixd local2world;
                // srsBase.createLocalToWorld(local2world);
                // osg::Vec3d worldPos, scale;
                // osg::Quat  worldRotation, scaleOrientation;
                // local2world.decompose(worldPos, worldRotation, scale, scaleOrientation);
                // pat->setPosition(worldPos);
                // pat->setAttitude(worldRotation);
                mapNode->addChild(pat);
            }
            pat->addChild(model.get());
            model->setNodeMask(~0);
            pat->setNodeMask(~0);
        }
        else
        {
            OE_FATAL << "No valid geo reference" << std::endl;
        }
    }
    else
    {
        OE_WARN << "No model loaded" << std::endl;
    }
}

// road outlines to osgEarth feature layer
GeoExtent calcFeaturesExtent(FeatureSource *features, const SpatialReference *mapSrs)
{
    GeoExtent                   extentsUnion;
    osg::ref_ptr<FeatureCursor> cursor = features->createFeatureCursor(Query(), 0L);
    while (cursor.valid() && cursor->hasMore())
    {
        osg::ref_ptr<Feature> feature       = cursor->nextFeature();
        osgEarth::Bounds      featureBounds = feature->getGeometry()->getBounds();
        GeoExtent             featureSrsExtent(feature->getSRS(), featureBounds);
        GeoExtent             featureMapExtent = featureSrsExtent.transform(mapSrs);
        extentsUnion.expandToInclude(featureMapExtent);
    }
    return extentsUnion;
}

GeoExtent calcFeaturesExtent(std::map<FeatureID, osg::ref_ptr<Geometry>> geometries, const SpatialReference *mapSrs)
{
    GeoExtent extentsUnion;
    for (auto &g : geometries)
    {
        osgEarth::Bounds bounds = g.second->getBounds();
        extentsUnion.expandToInclude(GeoExtent(mapSrs, bounds));
    }
    return extentsUnion;
}

std::map<FeatureID, osg::ref_ptr<Geometry>> convertFeaturesToGeometries(const SpatialReference                                   *srs,
                                                                        const SpatialReference                                   *mapSrs,
                                                                        ODRFeatureSource                                         *features,
                                                                        const std::map<long long, osg::ref_ptr<osg::Vec3dArray>> &outlines,
                                                                        double                                                    heightAdjust)
{
    std::map<FeatureID, osg::ref_ptr<Geometry>> geometries;
    for (auto &outline : outlines)
    {
        size_t num       = (*outline.second).size();
        auto   latLonVec = new Vec3dVector(num);
        for (size_t i = 0; i < (*outline.second).size(); ++i)
        {
            GeoPoint gpp(srs, (*outline.second)[i]);
            // this step is essential to have accurate world aligned data!
            // odr srs local coords to map srs
            (*latLonVec)[i] = gpp.transform(mapSrs).vec3d();
            (*latLonVec)[i].z() += heightAdjust;
            // geocentric coords to map srs:
            // mapSrs->transformFromWorld((*outline.second)[i],(*latLonVec)[i]);
        }
        Polygon *geom = new Polygon(latLonVec);
        // ring and polygon are opened with the vector copy constructor even if input data is closed
        geom->close();
        geometries[outline.first] = geom;
    }
    return geometries;
}

void openAndCreateFeatures(const SpatialReference                                   *srcSrc,
                           const SpatialReference                                   *mapSrs,
                           ODRFeatureSource                                         *features,
                           const std::map<long long, osg::ref_ptr<osg::Vec3dArray>> &polygons,
                           double                                                    heightAdjust)
{
    Style                                       emptyStyle = {};
    std::map<FeatureID, osg::ref_ptr<Geometry>> geos       = convertFeaturesToGeometries(srcSrc, mapSrs, features, polygons, heightAdjust);
    if (features->getFeatureCount() == 0)
    {
        osg::ref_ptr<FeatureProfile> featureProfile = new FeatureProfile(calcFeaturesExtent(geos, mapSrs));
        features->setFeatureProfile(featureProfile.get());
    }
    for (const auto &g : geos)
        features->insertFeature(new Feature(g.second.get(), mapSrs, emptyStyle, g.first));
    features->open();
}

ODRFeatureSource *initFeatureLayer(const SpatialReference                                   *srs,
                                   const Profile                                            *mapProfile,
                                   const std::map<long long, osg::ref_ptr<osg::Vec3dArray>> &polygons,
                                   double                                                    heightAdjust = 0.0)
{
    OE_INFO << "Initializing features (lane section outlines) for map layers..." << std::endl;
    ODRFeatureSource *features = new ODRFeatureSource();
    openAndCreateFeatures(srs, mapProfile->getSRS(), features, polygons, heightAdjust);
    return features;
}

// common
void openAllLayers(Map *map)
{
    osg::MixinVector<osg::ref_ptr<Layer>> layers;
    map->getLayers(layers);
    for (auto &l : layers)
        l->open();
}

void closeAllLayers(Map *map)
{
    osg::MixinVector<osg::ref_ptr<Layer>> layers;
    map->getLayers(layers);
    for (auto &l : layers)
        l->close();
}

const std::map<long long, osg::ref_ptr<osg::Vec3dArray>> &roadOutline()
{
    if (roadGeom)
    {
        for (auto &ol : roadGeom->outlines)
            roadOutlines[66666 + ol.first] = ol.second;
    }
    return roadOutlines;
}

// merge all the stuff:
MapNode *openOsgEarthOdr(MapNode *mapNode, osg::ref_ptr<osg::Node> model, std::string geoRef, bool cacheOnly, bool seedCache)
{
    osgEarth::setNotifyLevel(osg::NotifySeverity::INFO);
    OE_INFO << "Initializing cache" << std::endl;
    initCache(cacheOnly);
    ODRFeatureSource *features     = initFeatureLayer(GetSourceSrs(), wgs84profile, roadOutline(), deltaHeight);
    Map *map                       = mapNode->getMap();
    map->clear(); // basemap elevation layer had to be opened so sampleElevation works on odr load ... clear so FlatteningLayer works
    map->beginUpdate();
    addDrawFeaturesLayer(map, features);
    addBaseMaps(map, cacheOnly);
    addFlattenLayer(map, features);
    addCutoutLayer(map, features); 
    // maybe some post processing to merge terrain and model meshes needed! 
    // they should be close but are two distinct meshes and due to floating point precision
    // and different routes the data took, may not be at the same position and not seamless
    addModelLayer(model,
                  SourceReferencePoint(),
                  map,
                  mapNode,
                  false);
    map->endUpdate();
    openAllLayers(map);
    OE_INFO << "Geo ref point: " << SourceReferencePoint().transform(GetDestinationSrs()).toString() << " - Number of features: " << features->getFeatureCount() << std::endl;
    if (seedCache)
        seedMapCache(mapNode, features->getExtent());
    return mapNode;
}

void esMini_logger(const char *str)
{
    printf("%s\n", str);
}

osg::ref_ptr<osg::Group> LoadOdr(osg::ArgumentParser arguments, MapNode *mapNode)
{
    // file
    std::string odrFile;
    arguments.read("--odr", odrFile);
    // setup search path for resources
    std::string resourcePath;
    arguments.read("--resources",resourcePath);
    // choose implementation - defaults to esMini
    bool esMini = true;
    bool covise = false;
    std::string libraryName;
    arguments.read("--from", libraryName);
    if(libraryName=="covise")
    {
        covise=true;
        esMini=false;
    }
    else if(libraryName=="esmini")
    {
        covise=false;
        esMini=true;
    }
    else if(libraryName=="both")
    {
        covise=true;
        esMini=true;
    }
    osg::ref_ptr<osg::Group> node = new osg::Group;
    std::string geoRef;

    addBaseMaps(mapNode->getMap(), false);
    openAllLayers(mapNode->getMap());

    InitMapSRS(wgs84profile->getSRS());
    InitElevationSampler(mapNode->getMap());

    if(covise)
    {
        if(resourcePath.size()) // a bit hacky and only assuming one file path!
        {
            // : on unix ; on windows! -.-
            std::string result = resourcePath+= ":"+resourcePath+"share/covise/"; // covise root install path should be the input, and there may be files referring differently, so add that variant
            osgDB::Registry::instance()->setDataFilePathList(resourcePath);
        }
        OE_INFO << "Loading and meshing Covise OpenDrive map..." << std::endl;
        opencover::fasi *fasi = new opencover::fasi(odrFile.c_str());
        if (!fasi)
        {
            OE_WARN << "Failed to load Covise OpenDrive map" << std::endl;
        }
        else
        {
            node->addChild(fasi->system->initOsg(MapGeocentricOriginMatrix()));
            roadOutlines = fasi->system->initOsgOutlines();
        }
    }
    if(esMini)
    {
        OE_INFO << "Loading EsMini OpenDrive map..." << std::endl;
        SE_Env::Inst().AddPath(osgDB::getCurrentWorkingDirectory());
        if (!roadmanager::Position::LoadOpenDrive(odrFile.c_str()))
        {
            OE_WARN << "Failed to load EsMini OpenDrive map" << std::endl;
        }
        else
        {
            OE_INFO << "Meshing EsMini OpenDrive map..." << std::endl;
            geoRef = roadmanager::Position::GetOpenDrive()->GetGeoReference()->rawString;
            InitConvertMapCoordinates(SpatialReference::create(geoRef),wgs84profile->getSRS());
            roadGeom = std::make_unique<RoadGeom>(roadmanager::Position::GetOpenDrive(),MapGeocentricOriginMatrix(SpatialReference::create(geoRef),wgs84profile->getSRS()));
            node->addChild(roadGeom->root_);
        }
    }
    if(node->getNumChildren())
        openOsgEarthOdr(mapNode, node, geoRef, arguments.read("--cache_only"), arguments.read("--seed_cache"));
    else
        OE_INFO << "No OpenDrive map loaded" << std::endl;
    return node;
}

osgEarth::Viewpoint OdrRefPoint()
{
    osgEarth::GeoPoint p = SourceReferencePoint().transform(GetDestinationSrs());
    return Viewpoint("odrRoot", p.x(), p.y(), p.alt(), 0, -90, 1000);
}
