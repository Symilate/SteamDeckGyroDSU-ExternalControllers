#ifndef _KMICKI_CEMUHOOK_CONTROLLERCONFIG_H_
#define _KMICKI_CEMUHOOK_CONTROLLERCONFIG_H_

#include "config/config.h"
#include "hiddev/devicefactory.h"

namespace kmicki::cemuhook
{
    class ControllerConfig : public config::Config
    {
        public:
        ControllerConfig() = delete;
        ControllerConfig(config::Data & _configData, std::string prefix = "");

        hiddev::ControllerType const& Type() const;

        virtual void Load() override;
        virtual void Save() override;
        virtual void ToDefault() override;

        private:
        hiddev::ControllerType type;
    };
}

#endif
