/* This file is part of COVISE.

   You can use it under the terms of the GNU Lesser General Public License
   version 2.1 or later, see lgpl-2.1.txt.

 * License: LGPL 2+ */

#include "fasi.h"
#include <unistd.h>
#include "RoadSystem/RoadSystem.h"
#include "DecoratedGeometryTechnique.h"
#include "RoadTerrainLoader.h"
#include "coVRFileManager.h"

using namespace opencover;

opencover::fasi::fasi(const char *filename)
{
    system = NULL;
    loadRoadSystem(filename);
}

opencover::fasi::~fasi()
{
}

bool opencover::fasi::loadRoadSystem(const char *filename_chars)
{
    std::string filename(filename_chars);
    std::cerr << "Loading road system!" << std::endl;
    if (system == NULL)
    {
        // Building directory string to xodr file
        xodrDirectory.clear();
        if (filename[0] != '/' && filename[0] != '\\' && (!(filename[1] == ':' && (filename[2] == '/' || filename[2] == '\\'))))
        {  // / or backslash or c:/
            char *workingDir = getcwd(NULL, 0);
            xodrDirectory.assign(workingDir);
            free(workingDir);
        }
        size_t lastSlashPos  = filename.find_last_of('/');
        size_t lastSlashPos2 = filename.find_last_of('\\');
        if (lastSlashPos != filename.npos && (lastSlashPos2 == filename.npos || lastSlashPos2 < lastSlashPos))
        {
            if (!xodrDirectory.empty())
                xodrDirectory += "/";
            xodrDirectory.append(filename, 0, lastSlashPos);
        }
        if (lastSlashPos2 != filename.npos && (lastSlashPos == filename.npos || lastSlashPos < lastSlashPos2))
        {
            if (!xodrDirectory.empty())
                xodrDirectory += "\\";
            xodrDirectory.append(filename, 0, lastSlashPos2);
        }

        system = vehicleUtil::RoadSystem::Instance();

        xercesc::DOMElement *openDriveElement = getOpenDriveRootElement(filename);
        if (!openDriveElement)
        {
            std::cerr << "No regular xodr file " << filename << " at: " + xodrDirectory << std::endl;
            return false;
        }

        system->parseOpenDrive(openDriveElement);
        this->parseOpenDrive(rootElement);
    }
    return true;
}

xercesc::DOMElement *opencover::fasi::getOpenDriveRootElement(std::string filename)
{
    try
    {
        xercesc::XMLPlatformUtils::Initialize();
    }
    catch (const xercesc::XMLException &toCatch)
    {
        char *message = xercesc::XMLString::transcode(toCatch.getMessage());
        std::cout << "Error during initialization! :\n" << message << std::endl;
        xercesc::XMLString::release(&message);
        return NULL;
    }

    xercesc::XercesDOMParser *parser = new xercesc::XercesDOMParser();
    parser->setValidationScheme(xercesc::XercesDOMParser::Val_Never);

    try
    {
        parser->parse(filename.c_str());
    }
    catch (...)
    {
        std::cerr << "Couldn't parse OpenDRIVE XML-file " << filename << "!" << std::endl;
    }

    xercesc::DOMDocument *xmlDoc = parser->getDocument();
    if (xmlDoc)
    {
        rootElement = xmlDoc->getDocumentElement();
    }

    return rootElement;
}

void opencover::fasi::parseOpenDrive(xercesc::DOMElement *rootElement)
{
    xercesc::DOMNodeList *documentChildrenList = rootElement->getChildNodes();

    for (int childIndex = 0; childIndex < documentChildrenList->getLength(); ++childIndex)
    {
        xercesc::DOMElement *sceneryElement = dynamic_cast<xercesc::DOMElement *>(documentChildrenList->item(childIndex));
        if (sceneryElement && xercesc::XMLString::compareIString(sceneryElement->getTagName(), xercesc::XMLString::transcode("scenery")) == 0)
        {
            std::string fileString = xercesc::XMLString::transcode(sceneryElement->getAttribute(xercesc::XMLString::transcode("file")));
            std::string vpbString  = xercesc::XMLString::transcode(sceneryElement->getAttribute(xercesc::XMLString::transcode("vpb")));

            std::vector<BoundingArea> voidBoundingAreaVector;
            std::vector<std::string>  shapeFileNameVector;

            xercesc::DOMNodeList *sceneryChildrenList = sceneryElement->getChildNodes();
            xercesc::DOMElement  *sceneryChildElement;
            for (unsigned int childIndex = 0; childIndex < sceneryChildrenList->getLength(); ++childIndex)
            {
                sceneryChildElement = dynamic_cast<xercesc::DOMElement *>(sceneryChildrenList->item(childIndex));
                if (!sceneryChildElement)
                    continue;

                if (xercesc::XMLString::compareIString(sceneryChildElement->getTagName(), xercesc::XMLString::transcode("void")) == 0)
                {
                    double xMin = atof(xercesc::XMLString::transcode(sceneryChildElement->getAttribute(xercesc::XMLString::transcode("xMin"))));
                    double yMin = atof(xercesc::XMLString::transcode(sceneryChildElement->getAttribute(xercesc::XMLString::transcode("yMin"))));
                    double xMax = atof(xercesc::XMLString::transcode(sceneryChildElement->getAttribute(xercesc::XMLString::transcode("xMax"))));
                    double yMax = atof(xercesc::XMLString::transcode(sceneryChildElement->getAttribute(xercesc::XMLString::transcode("yMax"))));

                    voidBoundingAreaVector.push_back(BoundingArea(osg::Vec2(xMin, yMin), osg::Vec2(xMax, yMax)));
                    // voidBoundingAreaVector.push_back(BoundingArea(osg::Vec2(506426.839,5398055.357),osg::Vec2(508461.865,5399852.0)));
                }
                else if (xercesc::XMLString::compareIString(sceneryChildElement->getTagName(), xercesc::XMLString::transcode("shape")) == 0)
                {
                    std::string fileString = xercesc::XMLString::transcode(sceneryChildElement->getAttribute(xercesc::XMLString::transcode("file")));
                    shapeFileNameVector.push_back(fileString);
                }
            }

            if (!fileString.empty())
            {
                if (!coVRFileManager::instance()->fileExist((xodrDirectory + "/" + fileString).c_str()))
                {
                    std::cerr << "\n#\n# file not found: this may lead to a crash! \n#" << std::endl;
                }
                coVRFileManager::instance()->loadFile((xodrDirectory + "/" + fileString).c_str());
            }

            if (!vpbString.empty())
            {
                RoadTerrainLoader *roadTerrainLoader = new RoadTerrainLoader;
                fprintf(stderr, "loading %s\n", vpbString.c_str());
                if (roadTerrainLoader)
                {
                    osg::Vec3d                           offset(0, 0, 0);
                    const vehicleUtil::RoadSystemHeader &header = vehicleUtil::RoadSystem::Instance()->getHeader();
                    offset.set(header.xoffset, header.yoffset, 0.0);
                    fprintf(stderr, "loading %s offset: %f %f\n", (xodrDirectory + "/" + vpbString).c_str(), offset[0], offset[1]);
                    roadTerrainLoader->loadTerrain(xodrDirectory + "/" + vpbString, offset, voidBoundingAreaVector, shapeFileNameVector);
                }
            }
        }
        else if (sceneryElement &&
                 xercesc::XMLString::compareIString(sceneryElement->getTagName(), xercesc::XMLString::transcode("environment")) == 0)
        {
            std::string startRoadString = xercesc::XMLString::transcode(sceneryElement->getAttribute(xercesc::XMLString::transcode("startRoad")));
            if (startRoadString.length() > 0)
            {
            }
        }
    }
}
