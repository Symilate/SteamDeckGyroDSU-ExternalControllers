#ifndef _KMICKI_CEMUHOOK_SWITCHPRO_DATASOURCE_H_
#define _KMICKI_CEMUHOOK_SWITCHPRO_DATASOURCE_H_

#include "hidframe.h"
#include "calibration.h"
#include "cemuhook/cemuhookprotocol.h"
#include "cemuhook/datasource.h"
#include "hiddev/hiddevreader.h"
#include "pipeline/serve.h"

namespace kmicki::cemuhook::switchpro
{
    class DataSource : public kmicki::cemuhook::DataSource
    {
        public:
        DataSource() = delete;
        DataSource(hiddev::HidDevReader& reader);

        void Start() override;
        void Stop() override;
        bool IsControllerConnected() override;
        int const& SetDataNewFrame(cemuhook::protocol::MotionData& motion) override;

        private:
        hiddev::HidDevReader& reader;
        pipeline::Serve<hiddev::frame_t>* frameServe;

        CalibrationData calibration;

        // 3-sample buffering: each 0x30 report has 3 IMU samples
        ImuSample sampleBuffer[3];
        int samplesRemaining;   // 0 = need new report, 1-3 = buffered
        int currentSampleIndex; // which sample to serve next

        // Timestamp tracking
        uint8_t lastTimer;
        uint64_t currentTimestampUs;
        bool firstFrame;

        int toReplicate;

        // Convert calibrated IMU sample to MotionData
        void SampleToMotion(ImuSample const& sample,
                            cemuhook::protocol::MotionData& motion,
                            uint64_t timestampUs);

        // Init callback for HidDevReader
        bool InitController(hiddev::HidApiDev& dev);
    };
}

#endif
