/* This file is part of COVISE.

   You can use it under the terms of the GNU Lesser General Public License
   version 2.1 or later, see lgpl-2.1.txt.

 * License: LGPL 2+ */

#ifndef _OPENCRG_PLUGIN_H
#define _OPENCRG_PLUGIN_H
/****************************************************************************\ 
 **                                                            (C)2001 HLRS  **
 **                                                                          **
 ** Description: OpenCRG Plugin (does nothing)                              **
 **                                                                          **
 **                                                                          **
 ** Author: F.Seybold, S. Franz		                                                **
 **                                                                          **
 ** History:  								                                **
 ** Nov-01  v1	    				       		                            **
 **                                                                          **
 **                                                                          **
\****************************************************************************/

#include "opencrg/crgSurface.h"
#include <osg/Group>

class OpenCRGPlugin
{
public:
    OpenCRGPlugin();
    ~OpenCRGPlugin();
    void processCrgFile(std::string filename, osg::Group* root);
    void shadeCrgFile(std::string filename, osg::Group* root);
private:
    opencrg::Surface *surface;
    osg::Geode *surfaceGeode;
};
#endif
