#ifndef PHIDGETDEVICE_H
#define PHIDGETDEVICE_H

#include <vector>

#include <phidget21.h>

namespace nav {

typedef struct vec8 { int v[8]; } vec8;

class PhidgetDevice
{
public:

    /*!
     * \brief PhidgetDevice set the device serial
     * \param serial
     */
    PhidgetDevice(const int serial);

    ~PhidgetDevice();

private:

    /*!
     * \brief waitForAttach wait the attachment of the device
     * \return 0 if ok or an error code
     */
    int waitForAttach();

private:

    CPhidgetHandle
        handle_;   //!< the handle to the phidget board

    int
        serial_;    //!< the serial of the board
};

class PhidgetInterface : public PhidgetDevice
{
public:
    PhidgetInterface();

    int initialize(const vec8 &changeTrigger,
                   const vec8 &outputState,
                   const vec8 &);

private:

};

} // namespace nav


#endif // PHIDGETDEVICE_H
