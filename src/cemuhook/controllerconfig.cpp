#include "cemuhook/controllerconfig.h"
#include <sstream>
#include <algorithm>

namespace kmicki::config
{
    using ControllerType = hiddev::ControllerType;
    static const int cControllerTypeEnumLen = 3;
    static std::string cControllerTypeStrVals[cControllerTypeEnumLen] = { "auto", "steamdeck", "switchpro" };

    template<>
    bool ConfigItem<ControllerType>::Update(std::string const& value, std::string & message)
    {
        std::string strVal(value.length(), ' ');
        std::transform(value.begin(), value.end(), strVal.begin(), [](char c){ return std::tolower(c); });
        for(int i = 0; i < cControllerTypeEnumLen; ++i)
            if(strVal == cControllerTypeStrVals[i])
            {
                Val = (ControllerType)i;
                std::ostringstream str;
                str << "existing item value updated (type: controller-type), Name=" << Name << " Value=" << strVal;
                message = str.str();
                return true;
            }

        std::ostringstream str;
        str << "value invalid (type: controller-type), Name=" << Name << " Value=" << value;
        message = str.str();
        return false;
    }

    template<>
    std::string ConfigItem<ControllerType>::ValToString() const
    {
        if(static_cast<int>(Val) >= 0 && static_cast<int>(Val) < cControllerTypeEnumLen)
            return cControllerTypeStrVals[static_cast<int>(Val)];
        return "";
    }

    template struct ConfigItem<ControllerType>;
}

namespace kmicki::cemuhook
{
    static const std::string cPrefix = "controller";

    static const std::string cTypeStr = "type";
    static const hiddev::ControllerType cTypeDefault = hiddev::ControllerType::Auto;
    static const config::ConfigComment cTypeComment = { "Controller type to use. Possible values:\nauto - [default] auto-detect connected controller\nsteamdeck - force Steam Deck controller\nswitchpro - force Nintendo Switch Pro Controller (Bluetooth)" };

    ControllerConfig::ControllerConfig(config::Data & _configData, std::string prefix)
    : config::Config(_configData, (prefix.empty() ? "" : (prefix + ".")) + cPrefix)
    {
        ToDefault();
        Load();
    }

    hiddev::ControllerType const& ControllerConfig::Type() const
    {
        return type;
    }

    void ControllerConfig::Load()
    {
        bool subscription = false;
        GetValue(cTypeStr, type, subscription);
        if(subscription)
            FireSubscriptions();
    }

    void ControllerConfig::Save()
    {
        SetValue(cTypeStr, type, cTypeComment);
    }

    void ControllerConfig::ToDefault()
    {
        type = cTypeDefault;
    }
}
