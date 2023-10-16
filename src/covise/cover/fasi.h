/* This file is part of COVISE.

   You can use it under the terms of the GNU Lesser General Public License
   version 2.1 or later, see lgpl-2.1.txt.

 * License: LGPL 2+ */

#ifndef FASI_INCLUDE
#define FASI_INCLUDE
#include <list>
#include <sys/time.h>
#include <xercesc/parsers/XercesDOMParser.hpp>
#include <string>
#include "RoadSystem/RoadSystem.h"

namespace opencover
{
    class fasi
    {
    public:
        fasi(const char *filename);
        ~fasi();
        std::string              xodrDirectory;
        vehicleUtil::RoadSystem *system;
        xercesc::DOMElement     *rootElement;

        bool                 loadRoadSystem(const char *filename_chars);
        void                 parseOpenDrive(xercesc::DOMElement *rootElement);
        xercesc::DOMElement *getOpenDriveRootElement(std::string filename);
    };
}  // namespace opencover
#endif
