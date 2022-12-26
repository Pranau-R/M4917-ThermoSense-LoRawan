/*

Module: Model4917_cMeasurementLoop_fillBuffer.cpp

Function:
    Class for transmitting accumulated measurements.

Copyright:
    See accompanying LICENSE file for copyright and license information.

Author:
    Dhinesh Kumar Pitchai, MCCI Corporation   May 2022

*/

#include <Catena_TxBuffer.h>

#include "Model4917_cMeasurementLoop.h"

#include <arduino_lmic.h>

using namespace McciCatena;
using namespace McciModel4917;

/*

Name:   McciModel4917::cMeasurementLoop::fillTxBuffer()

Function:
    Prepare a messages in a TxBuffer with data from current measurements.

Definition:
    void McciModel4917::cMeasurementLoop::fillTxBuffer(
            cMeasurementLoop::TxBuffer_t& b
            );

Description:
    A format 0x28 message is prepared from the data in the cMeasurementLoop
    object.

*/

void
cMeasurementLoop::fillTxBuffer(
    cMeasurementLoop::TxBuffer_t& b, Measurement const &mData
    )
    {
    gLed.Set(McciCatena::LedPattern::Measuring);


    // initialize the message buffer to an empty state
    b.begin();

    // insert format byte
    b.put(kMessageFormat);

    // the flags in Measurement correspond to the over-the-air flags.
    b.put(std::uint8_t(this->m_data.flags));
    gCatena.SafePrintf("Flag:    %2x\n", std::uint8_t(this->m_data.flags));

    // send Vbat
    if ((this->m_data.flags &  Flags::Vbat) !=  Flags(0))
        {
        float Vbat = mData.Vbat;
        gCatena.SafePrintf("Vbat:    %d mV\n", (int) (Vbat * 1000.0f));
        b.putV(Vbat);
        }

    // print Vbus data
    float Vbus = mData.Vbus;
    gCatena.SafePrintf("Vbus:    %d mV\n", (int) (Vbus * 1000.0f));

    // send boot count
    if ((this->m_data.flags &  Flags::Boot) !=  Flags(0))
        {
        b.putBootCountLsb(mData.BootCount);
        }

    // send compost data probe one
    if ((this->m_data.flags & Flags::Temp1) !=  Flags(0))
        {
        if (! this->m_fSht3x)
            {
            gCatena.SafePrintf(
                    "Compost (Bottom):  T: %d C\n",
                    (int) mData.bottomProbe.TempC
                    );
            }
        else
            {
            gCatena.SafePrintf(
                    "Compost (Middle):  T: %d C\n",
                    (int) mData.bottomProbe.TempC
                    );
            }
        b.putT(mData.bottomProbe.TempC);
        }

    // send compost data probe two
    if ((this->m_data.flags & Flags::Temp2) !=  Flags(0))
        {
        gCatena.SafePrintf(
                "Compost (Middle):  T: %d C\n",
                (int) mData.middleProbe.TempC
                );
        b.putT(mData.middleProbe.TempC);
        }

    if ((mData.flags & Flags::TH) != Flags(0))
        {
        gCatena.SafePrintf(
                "RH-Flex (Bottom):  T: %d C RH: %d\n",
                (int) mData.rhTemp.TempC,
                (int) mData.rhTemp.Humidity
                );
        b.putT(mData.rhTemp.TempC);
        // no method for 2-byte RH, directly encode it.
        b.put2uf((mData.rhTemp.Humidity / 100.0f) * 65535.0f);
        }

    gLed.Set(McciCatena::LedPattern::Off);
    }
