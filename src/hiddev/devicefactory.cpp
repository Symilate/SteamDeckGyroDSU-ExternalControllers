#include "hiddev/devicefactory.h"
#include "cemuhook/sdcontroller/datasource.h"
#include "cemuhook/switchpro/datasource.h"
#include "cemuhook/switchpro/protocol.h"
#include "log/log.h"
#include <hidapi/hidapi.h>
#include <iomanip>
#include <filesystem>
#include <sstream>

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

    // Check if a VID/PID pair matches a known external controller.
    // Returns Auto if no match.
    static ControllerType MatchExternalController(uint16_t vid, uint16_t pid)
    {
        if(vid == cemuhook::switchpro::protocol::kVID
           && pid == cemuhook::switchpro::protocol::kPID)
            return ControllerType::SwitchPro;

        return ControllerType::Auto;
    }

    // Scan /sys/bus/hid/devices/ for Bluetooth HID devices that hid_enumerate may miss.
    // Directory names follow the format: BBBB:VVVV:PPPP.NNNN
    // where BBBB is bus type (0005 = Bluetooth), VVVV is VID, PPPP is PID.
    static ControllerType DetectBtController()
    {
        static const std::string cSysHidPath = "/sys/bus/hid/devices/";
        static const std::string cBtBusPrefix = "0005";

        std::error_code ec;
        if(!std::filesystem::exists(cSysHidPath, ec))
            return ControllerType::Auto;

        for(auto const& entry : std::filesystem::directory_iterator(cSysHidPath, ec))
        {
            auto name = entry.path().filename().string();

            // Expect format: BBBB:VVVV:PPPP.NNNN
            if(name.size() < 14 || name[4] != ':' || name[9] != ':')
                continue;

            auto busType = name.substr(0, 4);
            if(busType != cBtBusPrefix)
                continue;

            uint16_t vid = 0, pid = 0;
            std::istringstream vidStream(name.substr(5, 4));
            std::istringstream pidStream(name.substr(10, 4));
            vidStream >> std::hex >> vid;
            pidStream >> std::hex >> pid;

            { df::LogF(LogLevelDebug) << "Found BT HID device: VID=0x"
                << std::hex << std::setfill('0') << std::setw(4) << vid
                << " PID=0x" << std::setw(4) << pid; }

            auto type = MatchExternalController(vid, pid);
            if(type != ControllerType::Auto)
                return type;
        }

        return ControllerType::Auto;
    }

    ControllerType DetectController()
    {
        ControllerType found = ControllerType::Auto;
        bool sdFound = false;

        // First pass: hid_enumerate (sees USB/hidraw devices)
        auto* devs = hid_enumerate(0, 0);
        auto* cur = devs;

        while(cur)
        {
            { df::LogF(LogLevelDebug) << "Enumerated HID device: VID=0x"
                << std::hex << std::setfill('0') << std::setw(4) << cur->vendor_id
                << " PID=0x" << std::setw(4) << cur->product_id
                << std::dec << " interface=" << cur->interface_number; }

            auto extType = MatchExternalController(cur->vendor_id, cur->product_id);
            if(extType != ControllerType::Auto)
            {
                found = extType;
                break; // External controllers take priority
            }
            if(cur->vendor_id == kSdVID && cur->product_id == kSdPID
               && cur->interface_number == kSdInterface)
            {
                sdFound = true;
                // Don't break — keep looking for external controllers
            }
            cur = cur->next;
        }

        hid_free_enumeration(devs);

        // Second pass: scan sysfs for BT HID devices that hid_enumerate may miss
        if(found == ControllerType::Auto)
        {
            df::Log("No external controller found via hidapi. Scanning sysfs for BT devices...", LogLevelDebug);
            found = DetectBtController();
        }

        // Fall back to Steam Deck if no external controller found
        if(found == ControllerType::Auto && sdFound)
            found = ControllerType::SteamDeck;

        { df::LogF(LogLevelDebug) << "Detection result: " << ControllerTypeName(found); }

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
