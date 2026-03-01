#include "cemuhook/switchpro/hidframe.h"

namespace kmicki::cemuhook::switchpro
{
    FullReport const& GetFullReport(frame_t const& frame)
    {
        return *reinterpret_cast<FullReport const*>(frame.data());
    }

    bool IsFullReport(frame_t const& frame)
    {
        return frame.size() >= sizeof(FullReport) && frame[0] == 0x30;
    }
}
