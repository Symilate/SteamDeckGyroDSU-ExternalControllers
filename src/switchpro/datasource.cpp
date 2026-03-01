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
    : reader(_reader), frameServe(nullptr), samplesRemaining(0),
      currentSampleIndex(0), lastTimer(0), currentTimestampUs(0),
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
        samplesRemaining = 0;
        currentSampleIndex = 0;
        toReplicate = 0;

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
        //   accelX = left-right (positive = right)
        //   accelY = front-back (positive = face-up/backward)
        //   accelZ = up-down (positive = up against gravity)
        //   gyroX = pitch, gyroY = yaw, gyroZ = roll
        //
        // DSU expects:
        //   accX = right-positive, accY = up-positive, accZ = forward-positive
        //   pitch = rotation around left-right axis
        //   yaw = rotation around up axis
        //   roll = rotation around forward axis
        //
        // NOTE: These axis signs may need empirical adjustment with a real controller.
        motion.accX = accX;
        motion.accY = accZ;
        motion.accZ = -accY;
        motion.pitch = gyroX;
        motion.yaw = -gyroY;
        motion.roll = gyroZ;
    }

    int const& DataSource::SetDataNewFrame(MotionData& motion)
    {
        if(samplesRemaining <= 0)
        {
            // Need a new HID report
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
                // when 3+ reports appear to be missed.
                if(!firstFrame)
                {
                    uint8_t diff = report.timer - lastTimer;
                    if(diff > 3 && diff < 200)
                    {
                        ds::LogF(LogLevelDebug) << "Missed approximately " << (int)(diff - 1)
                                                << " reports (timer gap).";
                    }
                }

                lastTimer = report.timer;
                firstFrame = false;

                // Buffer all 3 IMU samples
                sampleBuffer[0] = report.imu[0]; // oldest
                sampleBuffer[1] = report.imu[1]; // middle
                sampleBuffer[2] = report.imu[2]; // newest
            }

            samplesRemaining = 3;
            currentSampleIndex = 0;
        }

        // Serve the next buffered sample
        SampleToMotion(sampleBuffer[currentSampleIndex], motion, currentTimestampUs);
        currentTimestampUs += protocol::kImuSampleTimeUs;

        ++currentSampleIndex;
        --samplesRemaining;

        toReplicate = samplesRemaining;
        return toReplicate;
    }
}
