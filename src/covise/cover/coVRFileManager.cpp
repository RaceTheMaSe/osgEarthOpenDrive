#include "coVRFileManager.h"
#include "osgDB/ReadFile"

opencover::coVRFileManager* opencover::coVRFileManager::fm = nullptr;
const char*                 opencover::coVRFileManager::getName(const std::string& s)  // probably implemented before osg handled file paths quite well
{
    openFiles.push_back(s);
    return openFiles.back().c_str();
}

bool opencover::coVRFileManager::fileExist(const std::string& s)
{
    return true;
}

inline static bool loadLibrary(const std::string& ext)
{
    std::string libName = osgDB::Registry::instance()->createLibraryNameForExtension(ext);
    if (osgDB::Registry::instance()->loadLibrary(libName) == osgDB::Registry::NOT_LOADED)
    {
        std::cout << "Failed osgDB handler for: ." << ext << " files (" << libName << ")" << std::endl;
        return false;
    }
    return true;
}

osg::Node* opencover::coVRFileManager::loadFile(const char* filename, void*, osg::Group* group)
{
    osg::Node* node = osgDB::readNodeFile(filename);
    if (node)
        group->addChild(node);
    return node;
}

opencover::coVRFileManager* opencover::coVRFileManager::instance()
{
    if (fm == nullptr)
    {
        fm = new coVRFileManager;
        bool pluginsOk = true;
        pluginsOk &= loadLibrary("osg");
        pluginsOk &= loadLibrary("osga");
        pluginsOk &= loadLibrary("jpeg");
        pluginsOk &= loadLibrary("png");
        if (!pluginsOk)
        {
            const auto& list = osgDB::Registry::instance()->getReaderWriterList();
            for (auto& le : list)
            {
                auto ext = le->supportedExtensions();
                for (auto e : ext)
                {
                    std::cout << e.second << std::endl;
                }
            }
            std::cout << "----------" << std::endl;
            const auto& imageList = osgDB::Registry::instance()->getImageProcessorList();
            for(const auto& ip : imageList)
            {
                std::cout << ip->className() << " - " << ip->libraryName() << std::endl;
            }
        }
    }
    return fm;
}
