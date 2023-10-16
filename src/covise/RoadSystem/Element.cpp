/* This file is part of COVISE.

   You can use it under the terms of the GNU Lesser General Public License
   version 2.1 or later, see lgpl-2.1.txt.

 * License: LGPL 2+ */

#include "Element.h"

using namespace vehicleUtil;

Element::Element(std::string setid)
    : id(setid)
{
}

std::string Element::getId()
{
    return id;
}
