#ifndef _KMICKI_CEMUHOOK_SWITCHPRO_HIDFRAME_H_
#define _KMICKI_CEMUHOOK_SWITCHPRO_HIDFRAME_H_

#include <cstdint>
#include "hiddev/hiddev.h"

namespace kmicki::cemuhook::switchpro
{
    using frame_t = hiddev::frame_t;

    // Raw IMU sample (12 bytes, all int16_t little-endian)
    struct ImuSample
    {
        int16_t accelX;
        int16_t accelY;
        int16_t accelZ;
        int16_t gyroX;   // roll
        int16_t gyroY;   // pitch
        int16_t gyroZ;   // yaw
    };
    static_assert(sizeof(ImuSample) == 12);

    // Full input report 0x30 (49 bytes, streamed at ~60Hz over BT)
    struct FullReport
    {
        uint8_t reportId;       // 0x30
        uint8_t timer;          // incrementing 0-255
        uint8_t batteryConn;    // high nibble = battery, low nibble = connection
        uint8_t buttons[3];     // button state
        uint8_t leftStick[3];   // 12-bit packed X/Y
        uint8_t rightStick[3];  // 12-bit packed X/Y
        uint8_t vibrator;       // vibrator input report
        ImuSample imu[3];       // 3 IMU frames: [0]=oldest, [1]=middle, [2]=newest
    };
    static_assert(sizeof(FullReport) == 49);

    // Extract reference to FullReport from raw frame buffer.
    FullReport const& GetFullReport(frame_t const& frame);

    // Check if frame is a full 0x30 report
    bool IsFullReport(frame_t const& frame);
}

#endif
