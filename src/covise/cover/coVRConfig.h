#pragma once

namespace opencover
{
    class coVRConfig
    {
    public:
        bool useDisplayLists()
        {
            return true;
        }
        bool useVBOs()
        {
            return true;
        }
        static coVRConfig* instance();
        static coVRConfig* fm;
    };
}  // namespace opencover
