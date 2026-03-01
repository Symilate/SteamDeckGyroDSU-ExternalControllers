#include "cemuhook/switchpro/protocol.h"
#include "log/log.h"
#include <cstring>

using namespace kmicki::log;

namespace kmicki::cemuhook::switchpro::protocol
{
    namespace proto {
        static const std::string cLogPrefix = "SwitchPro::Protocol: ";
        #include "log/locallog.h"
    }

    hiddev::frame_t BuildSubcommand(uint8_t& packetCounter, uint8_t subcmdId,
                                     const uint8_t* args, size_t argsLen)
    {
        hiddev::frame_t cmd(kOutputReportLen, 0x00);
        cmd[0] = kOutputReportCmd;               // 0x01
        cmd[1] = packetCounter & 0x0F;
        packetCounter = (packetCounter + 1) & 0x0F;
        // Bytes 2-9: rumble data (neutral = all zeros)
        cmd[10] = subcmdId;
        for(size_t i = 0; i < argsLen && (11 + i) < cmd.size(); ++i)
            cmd[11 + i] = args[i];
        return cmd;
    }

    hiddev::frame_t BuildSpiRead(uint8_t& packetCounter, uint32_t address, uint8_t length)
    {
        uint8_t args[5];
        args[0] = (uint8_t)(address & 0xFF);
        args[1] = (uint8_t)((address >> 8) & 0xFF);
        args[2] = (uint8_t)((address >> 16) & 0xFF);
        args[3] = (uint8_t)((address >> 24) & 0xFF);
        args[4] = length;
        return BuildSubcommand(packetCounter, kSubcmdSpiFlashRead, args, 5);
    }

    bool SendSubcommandAndWaitReply(hiddev::HidApiDev& dev,
                                     uint8_t& packetCounter,
                                     uint8_t subcmdId,
                                     const uint8_t* args, size_t argsLen,
                                     hiddev::frame_t& reply,
                                     int maxRetries)
    {
        for(int attempt = 0; attempt < maxRetries; ++attempt)
        {
            auto cmd = BuildSubcommand(packetCounter, subcmdId, args, argsLen);
            if(!dev.Write(cmd))
            {
                proto::Log("Failed to write subcommand.", LogLevelDebug);
                continue;
            }

            // Read replies until we get report 0x21 with matching ACK
            reply.resize(kFullReportLen);
            for(int reads = 0; reads < 100; ++reads)
            {
                int cnt = dev.Read(reply);
                if(cnt <= 0)
                    break; // timeout, retry

                // Check for subcommand reply: report 0x21, byte 14 = subcmdId
                if(reply[0] == kInputReportReply && cnt >= 15
                   && reply[14] == subcmdId)
                {
                    return true;
                }
                // Other reports (0x3F simple mode, 0x30 full mode) — discard and keep reading
            }

            { proto::LogF(LogLevelDebug) << "Subcommand 0x" << std::hex << (int)subcmdId
                                         << " attempt " << std::dec << (attempt + 1) << " timed out. Retrying..."; }
        }

        { proto::LogF() << "Subcommand 0x" << std::hex << (int)subcmdId << " failed after all retries."; }
        return false;
    }

    bool InitializeController(hiddev::HidApiDev& dev, uint8_t& packetCounter,
                               switchpro::CalibrationData& calibration)
    {
        hiddev::frame_t reply;

        // 1. Request device info
        proto::Log("Requesting device info...", LogLevelDebug);
        uint8_t noArgs = 0;
        if(!SendSubcommandAndWaitReply(dev, packetCounter, kSubcmdDeviceInfo,
                                        nullptr, 0, reply))
        {
            proto::Log("Failed to get device info.");
            return false;
        }
        proto::Log("Device info received.", LogLevelDebug);

        // 2. Read calibration data from SPI flash
        proto::Log("Reading calibration data...", LogLevelDebug);
        if(!ReadCalibration(dev, packetCounter, calibration))
        {
            proto::Log("Calibration read failed. Continuing with defaults.");
        }

        // 3. Enable vibration
        proto::Log("Enabling vibration...", LogLevelDebug);
        uint8_t enableArg = 0x01;
        if(!SendSubcommandAndWaitReply(dev, packetCounter, kSubcmdSetVibEnabled,
                                        &enableArg, 1, reply))
        {
            proto::Log("Failed to enable vibration. Continuing anyway.", LogLevelDebug);
        }

        // 4. Enable IMU (6-axis sensor)
        proto::Log("Enabling IMU...", LogLevelDebug);
        if(!SendSubcommandAndWaitReply(dev, packetCounter, kSubcmdSetImuEnabled,
                                        &enableArg, 1, reply))
        {
            proto::Log("Failed to enable IMU.");
            return false;
        }
        proto::Log("IMU enabled.", LogLevelDebug);

        // 5. Set input report mode to full (0x30)
        proto::Log("Setting input report mode to full...", LogLevelDebug);
        uint8_t modeArg = kInputModeFullReport;
        if(!SendSubcommandAndWaitReply(dev, packetCounter, kSubcmdSetInputMode,
                                        &modeArg, 1, reply))
        {
            proto::Log("Failed to set input report mode.");
            return false;
        }
        proto::Log("Input report mode set to 0x30. Initialization complete.");

        return true;
    }
}
