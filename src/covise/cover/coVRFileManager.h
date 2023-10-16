#pragma once
#include <string>
#include <vector>
#include <osgDB/Registry>

// this file is a reduced and minimal set of functions from the COVISE file manager class
namespace opencover
{
    class coVRFileManager
    {
    private:
        std::vector<std::string> openFiles = {};

    public:
        const char*             getName(const std::string& s);
        bool                    fileExist(const std::string& s);
        osg::Node*              loadFile(const char* filename, void* = nullptr, osg::Group* group = nullptr);
        static coVRFileManager* instance();
        static coVRFileManager* fm;
    };

}  // namespace opencover
