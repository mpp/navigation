#include "phidgetdevice.h"

namespace nav {

PhidgetDevice::PhidgetDevice(const int serial)
    : serial_(serial)
{
}

PhidgetDevice::~PhidgetDevice()
{
    CPhidget_close(handle_);
    CPhidget_delete(handle_);
}

int PhidgetDevice::waitForAttach()
{
    if (!handle_)
    {
        return EPHIDGET_INVALID;
    }

    int result;
    int timeout = 500; // ms

    // Register the events

    if((result = CPhidget_waitForAttachment(handle_, timeout))) {
        //std::cerr << "Device not attached, error: " << result << std::endl;
        handle_ = nullptr;
        return result;
    }
}

}   // namespace nav
