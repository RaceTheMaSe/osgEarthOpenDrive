#include "coVRConfig.h"

opencover::coVRConfig* opencover::coVRConfig::fm = nullptr;
opencover::coVRConfig* opencover::coVRConfig::instance()
{
    if (fm == nullptr)
        fm = new opencover::coVRConfig;
    return fm;
}
