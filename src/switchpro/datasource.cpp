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

        // Map to cemuhook DSU conventions.
        // Pro Controller held normally (face up, sticks toward you):
        //   accelY has gravity (~1G), accelX = left-right, accelZ = front-back
        //   gyroX = pitch, gyroY = yaw, gyroZ = roll
        //
        // DSU expects:
        //   accY = up-positive (gravity), accX = right, accZ = forward
        //   pitch, yaw, roll as rotations around respective axes
        //
        // NOTE: Axis signs may need empirical adjustment.
        motion.accX = accX;
        motion.accY = accY;
        motion.accZ = accZ;
        motion.pitch = gyroX;
        motion.yaw = gyroY;
        motion.roll = gyroZ;
    }

    int const& DataSource::SetDataNewFrame(MotionData& motion)
    {
        // Block for the next HID report and use the newest IMU sample (index 2).
        // The server calls us once per loop iteration and ignores toReplicate,
        // so buffering all 3 sub-samples would cause us to return too fast and
        // fall behind the real-time report stream.
        auto const& dataFrame = frameServe->GetPointer();

        {
            auto lock = frameServe->GetConsumeLock();

            if(!IsFullReport(*dataFrame))
            {
                // Not a 0x30 report — skip
                toReplicate = 0;
                return toReplicate;
            }

            auto const& report = GetFullReport(*dataFrame);

            // Detect missed reports via timer byte.
            // BT jitter commonly causes single-tick gaps, so only log
            // when 9+ reports appear to be missed.
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
        ++debugSampleCounter;
        if(debugSampleCounter >= 60)
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
