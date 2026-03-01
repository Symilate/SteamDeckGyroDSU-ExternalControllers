#include "hiddev/devicefactory.h"
#include "cemuhook/sdcontroller/datasource.h"
#include "cemuhook/switchpro/datasource.h"
#include "cemuhook/switchpro/protocol.h"
#include "log/log.h"
#include <hidapi/hidapi.h>

using namespace kmicki::log;

namespace kmicki::hiddev
{
    namespace df {
        static const std::string cLogPrefix = "DeviceFactory: ";
        #include "log/locallog.h"
    }

    // Steam Deck controller constants
    static constexpr uint16_t kSdVID = 0x28DE;
    static constexpr uint16_t kSdPID = 0x1205;
    static constexpr int kSdInterface = 2;
    static constexpr int kSdFrameLen = 64;
    static constexpr int kSdScanTimeUs = 4000;

    ControllerType DetectController()
    {
        auto* devs = hid_enumerate(0, 0);
        auto* cur = devs;

        ControllerType found = ControllerType::Auto; // means none found

        while(cur)
        {
            if(cur->vendor_id == cemuhook::switchpro::protocol::kVID
               && cur->product_id == cemuhook::switchpro::protocol::kPID)
            {
                found = ControllerType::SwitchPro;
                break; // External controllers take priority
            }
            if(cur->vendor_id == kSdVID && cur->product_id == kSdPID
               && cur->interface_number == kSdInterface)
            {
                found = ControllerType::SteamDeck;
                // Don't break — keep looking for external controllers
            }
            cur = cur->next;
        }

        hid_free_enumeration(devs);
        return found;
    }

    std::string ControllerTypeName(ControllerType type)
    {
        switch(type)
        {
            case ControllerType::SteamDeck: return "Steam Deck";
            case ControllerType::SwitchPro: return "Switch Pro Controller";
            default: return "Unknown";
        }
    }

    static DeviceBundle CreateSteamDeck()
    {
        DeviceBundle result;
        result.detectedType = ControllerType::SteamDeck;

        auto reader = std::make_unique<HidDevReader>(
            kSdVID, kSdPID, kSdInterface, kSdFrameLen, kSdScanTimeUs);
        auto ds = std::make_unique<cemuhook::sdcontroller::DataSource>(*reader);
        reader->SetWriteData(ds->WriteData);

        result.reader = std::move(reader);
        result.dataSource = std::move(ds);
        result.valid = true;
        return result;
    }

    static DeviceBundle CreateSwitchPro()
    {
        DeviceBundle result;
        result.detectedType = ControllerType::SwitchPro;

        namespace sp = cemuhook::switchpro::protocol;
        auto reader = std::make_unique<HidDevReader>(
            sp::kVID, sp::kPID, sp::kInterfaceNumber,
            sp::kFullReportLen, sp::kScanTimeUs);
        auto ds = std::make_unique<cemuhook::switchpro::DataSource>(*reader);
        // InitCallback is set in the DataSource constructor

        result.reader = std::move(reader);
        result.dataSource = std::move(ds);
        result.valid = true;
        return result;
    }

    DeviceBundle CreateDevice(ControllerType requestedType)
    {
        DeviceBundle result;
        result.valid = false;

        ControllerType type = requestedType;
        if(type == ControllerType::Auto)
        {
            df::Log("Auto-detecting controller...");
            type = DetectController();
            if(type == ControllerType::Auto)
            {
                df::Log("No supported controller found.");
                return result;
            }
        }

        { df::LogF() << "Creating " << ControllerTypeName(type) << " device."; }

        switch(type)
        {
            case ControllerType::SteamDeck:
                return CreateSteamDeck();
            case ControllerType::SwitchPro:
                return CreateSwitchPro();
            default:
                return result;
        }
    }
}
