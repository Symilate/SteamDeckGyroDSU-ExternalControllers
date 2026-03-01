#ifndef _KMICKI_CEMUHOOK_SWITCHPRO_CALIBRATION_H_
#define _KMICKI_CEMUHOOK_SWITCHPRO_CALIBRATION_H_

#include <cstdint>
#include "hiddev/hidapidev.h"

namespace kmicki::cemuhook::switchpro
{
    struct CalibrationData
    {
        struct AccelCal
        {
            int16_t origin;
            int16_t coeff;
            int16_t horizOffset;
        };

        struct GyroCal
        {
            int16_t offset;
            int16_t coeff;
        };

        AccelCal accel[3]; // X, Y, Z
        GyroCal gyro[3];   // X, Y, Z

        bool hasUserCal;
        bool isValid;

        // Initialize with default factory values
        void SetDefaults();

        // Apply calibration to raw accelerometer value -> G
        float CalibrateAccel(int axis, int16_t raw) const;

        // Apply calibration to raw gyroscope value -> degrees/sec
        float CalibrateGyro(int axis, int16_t raw) const;
    };

    // Read calibration data from SPI flash via subcommands.
    // Returns true if at least factory calibration was read successfully.
    bool ReadCalibration(hiddev::HidApiDev& dev, uint8_t& packetCounter,
                         CalibrationData& cal);
}

#endif
