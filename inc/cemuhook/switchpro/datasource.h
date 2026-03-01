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

        // Temp buffer for newest IMU sample from current report
        ImuSample sampleBuffer[1];

        // Timestamp tracking
        uint8_t lastTimer;
        uint64_t currentTimestampUs;
        bool firstFrame;

        int toReplicate;
        int debugSampleCounter = 0;

        // Convert calibrated IMU sample to MotionData
        void SampleToMotion(ImuSample const& sample,
                            cemuhook::protocol::MotionData& motion,
                            uint64_t timestampUs);

        // Init callback for HidDevReader
        bool InitController(hiddev::HidApiDev& dev);
    };
}

#endif
