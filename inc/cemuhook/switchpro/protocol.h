#ifndef _KMICKI_CEMUHOOK_SWITCHPRO_PROTOCOL_H_
#define _KMICKI_CEMUHOOK_SWITCHPRO_PROTOCOL_H_

#include "hiddev/hiddev.h"
#include "hiddev/hidapidev.h"
#include "calibration.h"
#include <cstdint>

namespace kmicki::cemuhook::switchpro::protocol
{
    // VID/PID
    constexpr uint16_t kVID = 0x057E;
    constexpr uint16_t kPID = 0x2009;
    constexpr int kInterfaceNumber = -1; // BT HID — no specific interface

    // Report IDs
    constexpr uint8_t kInputReportSimple = 0x3F;
    constexpr uint8_t kInputReportFull   = 0x30;
    constexpr uint8_t kInputReportReply  = 0x21;
    constexpr uint8_t kOutputReportCmd   = 0x01;

    // Subcommand IDs
    constexpr uint8_t kSubcmdDeviceInfo    = 0x02;
    constexpr uint8_t kSubcmdSetInputMode  = 0x03;
    constexpr uint8_t kSubcmdSpiFlashRead  = 0x10;
    constexpr uint8_t kSubcmdSetImuEnabled = 0x40;
    constexpr uint8_t kSubcmdSetVibEnabled = 0x48;

    // Input report mode values
    constexpr uint8_t kInputModeFullReport = 0x30;

    // SPI Flash addresses
    constexpr uint32_t kSpiFactorySensorCal  = 0x6020; // 24 bytes
    constexpr uint32_t kSpiSensorHorizOffset = 0x6080; // 6 bytes
    constexpr uint32_t kSpiUserSensorCal     = 0x8026; // 26 bytes
    constexpr uint16_t kUserCalMagic         = 0xA1B2; // LE bytes: 0xB2 0xA1

    // Frame constants
    constexpr int kFullReportLen       = 49;
    constexpr int kOutputReportLen     = 49;
    constexpr int kImuFrameSize        = 12;
    constexpr int kImuSamplesPerReport = 3;
    constexpr int kScanTimeUs          = 16667; // ~60Hz
    constexpr int kImuSampleTimeUs     = 5000;  // 5ms between sub-samples

    // Calibration defaults
    constexpr int16_t kDefaultAccelOrigin = 0;
    constexpr int16_t kDefaultAccelCoeff  = 0x4000;
    constexpr int16_t kDefaultGyroOffset  = 0;
    constexpr int16_t kDefaultGyroCoeff   = 0x343B;

    // Deadzone thresholds (raw LSBs)
    constexpr int16_t kGyroDeadzone  = 75;
    constexpr int16_t kAccelDeadzone = 205;

    // Build an output report 0x01 with subcommand.
    // packetCounter wraps 0x00-0x0F.
    hiddev::frame_t BuildSubcommand(uint8_t& packetCounter, uint8_t subcmdId,
                                     const uint8_t* args = nullptr, size_t argsLen = 0);

    // Build SPI flash read subcommand (0x10).
    hiddev::frame_t BuildSpiRead(uint8_t& packetCounter, uint32_t address, uint8_t length);

    // Send subcommand and wait for reply (report 0x21 with matching ACK).
    // reply is resized/filled with the reply data.
    // Returns true if a valid reply was received.
    bool SendSubcommandAndWaitReply(hiddev::HidApiDev& dev,
                                     uint8_t& packetCounter,
                                     uint8_t subcmdId,
                                     const uint8_t* args, size_t argsLen,
                                     hiddev::frame_t& reply,
                                     int maxRetries = 5);

    // Run the full BT initialization sequence on an open device.
    // Reads calibration, enables IMU, sets full report mode.
    // Returns true on success.
    bool InitializeController(hiddev::HidApiDev& dev, uint8_t& packetCounter,
                               switchpro::CalibrationData& calibration);
}

#endif
