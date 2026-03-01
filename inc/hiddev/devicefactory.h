#ifndef _KMICKI_HIDDEV_DEVICEFACTORY_H_
#define _KMICKI_HIDDEV_DEVICEFACTORY_H_

#include "hiddevreader.h"
#include "cemuhook/datasource.h"
#include <memory>
#include <string>

namespace kmicki::hiddev
{
    enum class ControllerType
    {
        Auto = 0,
        SteamDeck,
        SwitchPro
    };

    struct DeviceBundle
    {
        std::unique_ptr<HidDevReader> reader;
        std::unique_ptr<cemuhook::DataSource> dataSource;
        ControllerType detectedType;
        bool valid;
    };

    // Auto-detect or create specified controller.
    // If type is Auto, enumerate HID devices and pick the first supported one.
    // Priority: Steam Deck first, then Switch Pro.
    DeviceBundle CreateDevice(ControllerType requestedType);

    // Enumerate HID devices and return which controller type is available.
    // Returns Auto if none found.
    ControllerType DetectController();

    // Get display name for controller type
    std::string ControllerTypeName(ControllerType type);
}

#endif
