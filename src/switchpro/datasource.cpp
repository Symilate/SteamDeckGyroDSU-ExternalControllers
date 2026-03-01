#include "cemuhook/switchpro/datasource.h"
#include "cemuhook/switchpro/protocol.h"
#include "log/log.h"

#include <cmath>

using namespace kmicki::cemuhook::protocol;
using namespace kmicki::log;

namespace kmicki::cemuhook::switchpro
{
    namespace ds {
        static const std::string cLogPrefix = "SwitchPro::DataSource: ";
        #include "log/locallog.h"
    }

    DataSource::DataSource(hiddev::HidDevReader& _reader)
    : reader(_reader), frameServe(nullptr),
      lastTimer(0), currentTimestampUs(0),
      firstFrame(true), toReplicate(0)
    {
        calibration.SetDefaults();

        reader.SetInitCallback([this](hiddev::HidApiDev& dev) -> bool {
            return InitController(dev);
        });

        ds::Log("Initialized. Waiting for start of frame grab.", LogLevelDebug);
    }

    bool DataSource::InitController(hiddev::HidApiDev& dev)
    {
        uint8_t packetCounter = 0;

        if(!protocol::InitializeController(dev, packetCounter, calibration))
        {
            ds::Log("Controller initialization failed. Using default calibration.");
            calibration.SetDefaults();
            // Don't abort — the controller might still work with defaults
        }

        return true;
    }

    void DataSource::Start()
    {
        lastTimer = 0;
        currentTimestampUs = 0;
        firstFrame = true;
        toReplicate = 0;
        debugSampleCounter = 0;

        ds::Log("Starting frame grab.", LogLevelDebug);
        reader.Start();
        frameServe = &reader.GetServe();
    }

    void DataSource::Stop()
    {
        ds::Log("Stopping frame grab.", LogLevelDebug);
        reader.StopServe(*frameServe);
        frameServe = nullptr;
        reader.Stop();
    }

    bool DataSource::IsControllerConnected()
    {
        return true;
    }

    void DataSource::SampleToMotion(ImuSample const& sample,
                                     MotionData& motion,
                                     uint64_t timestampUs)
    {
        motion.timestampL = (uint32_t)(timestampUs & 0xFFFFFFFF);
        motion.timestampH = (uint32_t)(timestampUs >> 32);

        // Apply calibration
        float accX = calibration.CalibrateAccel(0, sample.accelX);
        float accY = calibration.CalibrateAccel(1, sample.accelY);
        float accZ = calibration.CalibrateAccel(2, sample.accelZ);

        float gyroX = calibration.CalibrateGyro(0, sample.gyroX);
        float gyroY = calibration.CalibrateGyro(1, sample.gyroY);
        float gyroZ = calibration.CalibrateGyro(2, sample.gyroZ);

        // Apply gyro deadzone
        if(std::abs(sample.gyroX) < protocol::kGyroDeadzone) gyroX = 0.0f;
        if(std::abs(sample.gyroY) < protocol::kGyroDeadzone) gyroY = 0.0f;
        if(std::abs(sample.gyroZ) < protocol::kGyroDeadzone) gyroZ = 0.0f;

        // Empirically determined axis mapping (8BitDo SN30 Pro / Switch Pro):
        //   Controller flat (face up): accX ≈ 1G, accY ≈ 0, accZ ≈ 0
        //   DSU expects: accY = gravity (up-positive)
        motion.accX = accY;
        motion.accY = accX;
        motion.accZ = accZ;
        motion.pitch = gyroX;
        motion.yaw = gyroY;
        motion.roll = gyroZ;
    }

    int const& DataSource::SetDataNewFrame(MotionData& motion)
    {
        auto const& dataFrame = frameServe->GetPointer();

        {
            auto lock = frameServe->GetConsumeLock();

            if(!IsFullReport(*dataFrame))
            {
                toReplicate = 0;
                return toReplicate;
            }

            auto const& report = GetFullReport(*dataFrame);

            // Detect missed reports via timer byte
            if(!firstFrame)
            {
                uint8_t diff = report.timer - lastTimer;
                if(diff > 9 && diff < 200)
                {
                    ds::LogF(LogLevelDebug) << "Missed approximately " << (int)(diff - 1)
                                            << " reports (timer gap).";
                }
            }

            lastTimer = report.timer;
            firstFrame = false;

            // Use newest IMU sample from this report
            sampleBuffer[0] = report.imu[2];
        }

        SampleToMotion(sampleBuffer[0], motion, currentTimestampUs);
        currentTimestampUs += protocol::kImuSampleTimeUs;

        // Log motion data periodically (~once per second) for diagnostics
        if(++debugSampleCounter >= 60)
        {
            ds::LogF(LogLevelDebug) << "IMU: acc("
                << motion.accX << ", " << motion.accY << ", " << motion.accZ
                << ") gyro(" << motion.pitch << ", " << motion.yaw << ", " << motion.roll << ")";
            debugSampleCounter = 0;
        }

        toReplicate = 0;
        return toReplicate;
    }
}
