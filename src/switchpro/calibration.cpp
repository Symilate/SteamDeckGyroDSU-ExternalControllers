#include "cemuhook/switchpro/calibration.h"
#include "cemuhook/switchpro/protocol.h"
#include "log/log.h"
#include <cstring>

using namespace kmicki::log;

namespace kmicki::cemuhook::switchpro
{
    namespace cal {
        static const std::string cLogPrefix = "SwitchPro::Calibration: ";
        #include "log/locallog.h"
    }

    void CalibrationData::SetDefaults()
    {
        for(int i = 0; i < 3; ++i)
        {
            accel[i].origin = protocol::kDefaultAccelOrigin;
            accel[i].coeff = protocol::kDefaultAccelCoeff;
            accel[i].horizOffset = 0;
            gyro[i].offset = protocol::kDefaultGyroOffset;
            gyro[i].coeff = protocol::kDefaultGyroCoeff;
        }
        hasUserCal = false;
        isValid = false;
    }

    float CalibrationData::CalibrateAccel(int axis, int16_t raw) const
    {
        float adjusted = (float)(raw - accel[axis].horizOffset);
        float range = (float)(accel[axis].coeff - accel[axis].origin);
        if(range == 0.0f) range = 1.0f;
        return adjusted * (4.0f / range);
    }

    float CalibrationData::CalibrateGyro(int axis, int16_t raw) const
    {
        float adjusted = (float)(raw - gyro[axis].offset);
        float range = (float)(gyro[axis].coeff - gyro[axis].offset);
        if(range == 0.0f) range = 1.0f;
        return adjusted * (936.0f / range);
    }

    // Parse 24 bytes of sensor calibration data from SPI flash.
    // Layout: accelOrigin[3] (6B), accelCoeff[3] (6B), gyroOffset[3] (6B), gyroCoeff[3] (6B)
    static void ParseSensorCal(const uint8_t* data, CalibrationData& cal)
    {
        const int16_t* vals = reinterpret_cast<const int16_t*>(data);
        for(int i = 0; i < 3; ++i)
        {
            cal.accel[i].origin = vals[i];
            cal.accel[i].coeff = vals[3 + i];
            cal.gyro[i].offset = vals[6 + i];
            cal.gyro[i].coeff = vals[9 + i];
        }
    }

    // Parse 6 bytes of horizontal offset data.
    // Layout: accelHorizOffset[3] (6B)
    static void ParseHorizOffset(const uint8_t* data, CalibrationData& cal)
    {
        const int16_t* vals = reinterpret_cast<const int16_t*>(data);
        for(int i = 0; i < 3; ++i)
            cal.accel[i].horizOffset = vals[i];
    }

    // Helper: send SPI read subcommand and get reply data.
    // Returns pointer to SPI data within reply (byte 20+), or nullptr on failure.
    static const uint8_t* SpiRead(hiddev::HidApiDev& dev, uint8_t& packetCounter,
                                   uint32_t address, uint8_t length,
                                   hiddev::frame_t& reply)
    {
        uint8_t args[5];
        args[0] = (uint8_t)(address & 0xFF);
        args[1] = (uint8_t)((address >> 8) & 0xFF);
        args[2] = (uint8_t)((address >> 16) & 0xFF);
        args[3] = (uint8_t)((address >> 24) & 0xFF);
        args[4] = length;

        if(!protocol::SendSubcommandAndWaitReply(dev, packetCounter,
            protocol::kSubcmdSpiFlashRead, args, 5, reply))
            return nullptr;

        // In a 0x21 reply to SPI read:
        //   byte 13: ACK byte (0x90 for success)
        //   byte 14: subcommand echo (0x10)
        //   bytes 15-18: address echo
        //   byte 19: length echo
        //   bytes 20+: SPI data
        if(reply.size() < (size_t)(20 + length))
            return nullptr;

        return &reply[20];
    }

    bool ReadCalibration(hiddev::HidApiDev& dev, uint8_t& packetCounter,
                         CalibrationData& cal)
    {
        cal.SetDefaults();
        hiddev::frame_t reply;

        // 1. Read factory sensor calibration (24 bytes at 0x6020)
        auto* data = SpiRead(dev, packetCounter, protocol::kSpiFactorySensorCal, 24, reply);
        if(data)
        {
            ParseSensorCal(data, cal);
            cal.isValid = true;
            cal::Log("Factory sensor calibration read successfully.", LogLevelDebug);
        }
        else
        {
            cal::Log("Failed to read factory sensor calibration. Using defaults.");
            return false;
        }

        // 2. Read sensor horizontal offsets (6 bytes at 0x6080)
        data = SpiRead(dev, packetCounter, protocol::kSpiSensorHorizOffset, 6, reply);
        if(data)
        {
            ParseHorizOffset(data, cal);
            cal::Log("Sensor horizontal offsets read successfully.", LogLevelDebug);
        }
        else
        {
            cal::Log("Failed to read horizontal offsets. Using zeros.", LogLevelDebug);
        }

        // 3. Read user sensor calibration (26 bytes at 0x8026)
        data = SpiRead(dev, packetCounter, protocol::kSpiUserSensorCal, 26, reply);
        if(data)
        {
            // Check magic bytes (LE: 0xB2 0xA1)
            if(data[0] == 0xB2 && data[1] == 0xA1)
            {
                ParseSensorCal(data + 2, cal);
                cal.hasUserCal = true;
                cal::Log("User sensor calibration found and applied.", LogLevelDebug);
            }
            else
            {
                cal::Log("No user sensor calibration. Using factory cal.", LogLevelDebug);
            }
        }
        else
        {
            cal::Log("Failed to read user calibration. Using factory cal.", LogLevelDebug);
        }

        return true;
    }
}
