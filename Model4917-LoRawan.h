/*

Name:   Model4917-LoRawan.h

Function:
    Global linkage for Model4917-LoRawan.ino

Copyright:
    See accompanying LICENSE file for copyright and license information.

Author:
    Dhinesh Kumar Pitchai, MCCI Corporation   May 2022

*/

#ifndef _Model4917_LoRawan_h_
# define _Model4917_LoRawan_h_

#pragma once

#include <Catena.h>
#include <Catena_Led.h>
#include <Catena_Mx25v8035f.h>
#include <Catena_Timer.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SPI.h>
#include "Model4917_cMeasurementLoop.h"

// the global clock object

extern  McciCatena::Catena                      gCatena;
extern  McciCatena::cTimer                      ledTimer;
extern  McciCatena::Catena::LoRaWAN             gLoRaWAN;
extern  McciCatena::StatusLed                   gLed;

extern  SPIClass                                gSPI2;
extern  McciModel4917::cMeasurementLoop        gMeasurementLoop;

//   The Temp Probe
extern  OneWire                                 oneWire;
extern  DallasTemperature                       sensor_CompostTemp;
extern bool                                     fHasCompostTemp;

//   SHT3x
extern  cDs28e18                                gDs28e18;

//   The flash
extern  McciCatena::Catena_Mx25v8035f           gFlash;

#endif // !defined(_Model4917_LoRawan_h_)
