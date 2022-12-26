/*

Module: Model4917_cMeasurementLoop.cpp

Function:
    Class for transmitting accumulated measurements.

Copyright:
    See accompanying LICENSE file for copyright and license information.

Author:
    Dhinesh Kumar Pitchai, MCCI Corporation   May 2022

*/

#include "Model4917_cMeasurementLoop.h"

#include <arduino_lmic.h>
#include <Model4917-LoRawan.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <stdint.h>

using namespace McciModel4917;
using namespace McciCatena;

/****************************************************************************\
|
|   Variables.
|
\****************************************************************************/

extern OneWire oneWire;
extern DallasTemperature sensor_CompostTemp;
extern cMeasurementLoop gMeasurementLoop;
extern cDs28e18 gDs28e18;

// arrays to hold device addresses
DeviceAddress probe[4];

uint8_t sht3xSlaveAddress = 0x88;
uint8_t sht3xBegin[] = { sht3xSlaveAddress, 0x30, 0xA2};
uint8_t sht3xConfiguration[] = { sht3xSlaveAddress, 0x24, 0x00};
uint8_t sht3xQuery[] = { sht3xSlaveAddress | 0x01 };

unsigned short slaveReadLength;
unsigned short readingLocation;

/****************************************************************************\
|
|   An object to represent the uplink activity
|
\****************************************************************************/

void cMeasurementLoop::begin()
    {
    // register for polling.
    if (! this->m_registered)
        {
        this->m_registered = true;

        gCatena.registerObject(this);

        this->m_UplinkTimer.begin(this->m_txCycleSec * 1000);
        }

    Wire.begin();

    this->checkRhFlexSensorPresent();
    pinMode(D11, INPUT);

    // start (or restart) the FSM.
    if (! this->m_running)
        {
        this->m_exit = false;
        this->m_fsm.init(*this, &cMeasurementLoop::fsmDispatch);
        }
    }

bool cMeasurementLoop::checkRhFlexSensorPresent(void)
    {
    /* set D11 high so V_OUT2 is going to be high for onewire sensor */
    pinMode(D11, OUTPUT);
    digitalWrite(D11, HIGH);

    delay(50);

    gDs28e18.begin();

    if (gDs28e18.getAddress(this->rhFlexCable, 0))
        {
        // gCatena.SafePrintf("RH Flex Address: "); 
        // this->getRhFlexAddress(this->rhFlexCable);
        this->configSht();
        return true;
        }
    else
        {
        return false;
        }
    }

// return true if the compost sensor is attached.
bool cMeasurementLoop::checkCompostSensorPresent(void)
    {
    /* set D11 high so V_OUT2 is going to be high for onewire sensor */
    pinMode(D11, OUTPUT);
    digitalWrite(D11, HIGH);

    delay(10);

    sensor_CompostTemp.begin();

    // locate devices on the bus
    this->m_nDevices = sensor_CompostTemp.getDeviceCount();
    // gCatena.SafePrintf("Devices Found: %u devices\n", this->m_nDevices);

    for (uint8_t nIndex = 0; nIndex < this->m_nDevices; nIndex++)
        {
        sensor_CompostTemp.getAddress(probe[nIndex], nIndex);

        // gCatena.SafePrintf("Device[%u]: ", nIndex);
        // print address
        /*for (uint8_t i = 0; i < 8; i++)
            {
            // zero pad the address if necessary
            if (probe[nIndex][i] < 16)
                gCatena.SafePrintf("0");

            // gCatena.SafePrintf("%u", probe[nIndex][i], HEX);
            Serial.print(probe[nIndex][i], HEX);
            }*/
        // gCatena.SafePrintf("\n");
        }

    sensor_CompostTemp.setUserData(probe[0], this->m_userdata.kEndSensor);
    sensor_CompostTemp.setUserData(probe[1], this->m_userdata.kMiddleSensor);

    return this->m_nDevices != 0;
    }

void cMeasurementLoop::end()
    {
    if (this->m_running)
        {
        this->m_exit = true;
        this->m_fsm.eval();
        }
    }

void cMeasurementLoop::requestActive(bool fEnable)
    {
    if (fEnable)
        this->m_rqActive = true;
    else
        this->m_rqInactive = true;

    this->m_fsm.eval();
    }

/*void cMeasurementLoop::getRhFlexAddress(deviceAddress deviceAddress)
    {
    for (uint8_t i = 0; i < 8; i++)
        {
        // zero pad the address if necessary
        if (deviceAddress[i] < 16)
            gCatena.SafePrintf("0");
        // gCatena.SafePrintf("%u", deviceAddress[i], HEX);
        Serial.print(deviceAddress[i], HEX);
        }

    // gCatena.SafePrintf("\n");
    }*/

void cMeasurementLoop::configSht()
    {
    unsigned char *statusData, *configData, *setConfigData;

    gDs28e18.deviceStatus(statusData);
    gDs28e18.readConfiguration(configData);
    gDs28e18.writeConfiguration(gDs28e18.khz1000, gDs28e18.dontIgnore, gDs28e18.i2c, gDs28e18.mode0);
    gDs28e18.readConfiguration(setConfigData);

    //Build I2C sensor's sequence using sequencer functions
    gDs28e18.clearSequencerPacket();
    gDs28e18.uitilitySensVddOnBuildPacket();
    gDs28e18.uitilityDelayBuildPacket(gDs28e18.delay16);
    gDs28e18.i2cStartBuildPacket();
    gDs28e18.i2cWriteDataBuildPacket(sht3xBegin, sizeof(sht3xBegin));
    gDs28e18.i2cStopBuildPacket();
    gDs28e18.uitilityDelayBuildPacket(gDs28e18.delay32);
    gDs28e18.i2cStartBuildPacket();
    gDs28e18.i2cWriteDataBuildPacket(sht3xConfiguration, sizeof(sht3xConfiguration));
    gDs28e18.i2cStopBuildPacket();
    gDs28e18.uitilityDelayBuildPacket(gDs28e18.delay64);
    gDs28e18.i2cStartBuildPacket();
    gDs28e18.i2cWriteDataBuildPacket(sht3xQuery, sizeof(sht3xQuery));

    slaveReadLength = 6;
    readingLocation = gDs28e18.i2cReadDataNackEndBuildPacket(slaveReadLength);
    gDs28e18.i2cStopBuildPacket();
    gDs28e18.uitilitySensVddOffBuildPacket();
    }

bool cMeasurementLoop::validAddress(const uint8_t* deviceAddr)
    {
    return (oneWire.crc8(deviceAddr, 7) == deviceAddr[7]);
    }

cMeasurementLoop::State
cMeasurementLoop::fsmDispatch(
    cMeasurementLoop::State currentState,
    bool fEntry
    )
    {
    State newState = State::stNoChange;

    if (fEntry && this->isTraceEnabled(this->DebugFlags::kTrace))
        {
        gCatena.SafePrintf("cMeasurementLoop::fsmDispatch: enter %s\n",
                this->getStateName(currentState)
                );
        }

    switch (currentState)
        {
    case State::stInitial:
        newState = State::stInactive;
        this->resetMeasurements();
        break;

    case State::stInactive:
        if (fEntry)
            {
            // turn off anything that should be off while idling.
            }
        if (this->m_rqActive)
            {
            // when going active manually, start the measurement
            // cycle immediately.
            this->m_rqActive = this->m_rqInactive = false;
            this->m_active = true;
            this->m_UplinkTimer.retrigger();
            newState = State::stWarmup;
            }
        break;

    case State::stSleeping:
        if (fEntry)
            {
            // set the LEDs to flash accordingly.
            gLed.Set(McciCatena::LedPattern::Sleeping);
            }

        if (this->m_rqInactive)
            {
            this->m_rqActive = this->m_rqInactive = false;
            this->m_active = false;
            newState = State::stInactive;
            }
        else if (this->m_UplinkTimer.isready())
            newState = State::stMeasure;
        else if (this->m_UplinkTimer.getRemaining() > 1500)
            this->sleep();
        break;

      // get some data. This is only called while booting up.
	     case State::stWarmup:
	         if (fEntry)
	             {
	             //start the timer
	             this->setTimer(5 * 1000);
	             }
	         if (this->timedOut())
	             newState = State::stMeasure;
	         break;


    // fill in the measurement
    case State::stMeasure:
			if (fEntry)
            {
            this->updateSynchronousMeasurements();
            }

            newState = State::stTransmit;
        break;

    case State::stTransmit:
        if (fEntry)
            {
            TxBuffer_t b;
            this->fillTxBuffer(b, this->m_data);

            this->m_FileTxBuffer.begin();
            for (auto i = 0; i < b.getn(); ++i)
                this->m_FileTxBuffer.put(b.getbase()[i]);

            this->resetMeasurements();
            this->startTransmission(b);
            }
        if (! gLoRaWAN.IsProvisioned())
            {
            newState = State::stFinal;
            }
        if (this->txComplete())
            {
            newState = State::stSleeping;

            // calculate the new sleep interval.
            this->updateTxCycleTime();
            }
        break;

    case State::stFinal:
        break;

    default:
        break;
        }

    return newState;
    }

/****************************************************************************\
|
|   Take a measurement
|
\****************************************************************************/

void cMeasurementLoop::resetMeasurements()
    {
    memset((void *) &this->m_data, 0, sizeof(this->m_data));
    this->m_data.flags = Flags(0);
    }

void cMeasurementLoop::updateSynchronousMeasurements()
    {
    this->m_data.Vbat = gCatena.ReadVbat();
    this->m_data.flags |= Flags::Vbat;

    if (gCatena.getBootCount(this->m_data.BootCount))
        {
        this->m_data.flags |= Flags::Boot;
        }

    // SI1133 is handled separately

    /*
    || Measure and transmit the compost temperature (OneWire)
    || tranducer value. This is complicated because we want
    || to support plug/unplug and the sw interface is not
    || really hot-pluggable.
    */

    // enable boost regulator if no USB power and VBat is less than 3.1V
    if (!m_fUsbPower && (this->m_data.Vbat < 3.10f))
        {
        pinMode(D14, OUTPUT);
        digitalWrite(D14, HIGH);
        delay(90);
        }

    bool fCompostTemp = checkCompostSensorPresent();

    if (fCompostTemp)
        {
        uint8_t nIndex = 0;
        do
            {
            sensor_CompostTemp.requestTemperatures();
            float compostTempC = sensor_CompostTemp.getTempC(probe[nIndex]);

            auto userData = sensor_CompostTemp.getUserData(probe[nIndex]);

            if (userData == this->m_userdata.kEndSensor) {
                //gCatena.SafePrintf("UserData: 0x%4x :: Fetching temperature of End Probe\n", userData);
                this->m_data.bottomProbe.TempC = compostTempC;
                this->m_data.flags |= Flags::Temp1;
                }
            else if (userData == this->m_userdata.kMiddleSensor) {
                //gCatena.SafePrintf("UserData: 0x%4x :: Fetching temperature of Middle Probe\n", userData);
                this->m_data.middleProbe.TempC = compostTempC;
                this->m_data.flags |= Flags::Temp2;
                }
            ++nIndex;
            } while (nIndex < this->m_nDevices);
        }
    else if (fHasCompostTemp)
        {
        gCatena.SafePrintf("No compost temperature\n");
        }
    else if(!fCompostTemp)
        {
        gCatena.SafePrintf("Compost sensor not detected\n");
        }

    /* set D11 low to turn off after measuring */
    pinMode(D11, INPUT);
    pinMode(D14, INPUT);

    bool fSht3x = this->checkRhFlexSensorPresent();
    if (fSht3x)
        {
        pinMode(D11, OUTPUT);
        digitalWrite(D11, HIGH);
        this->m_fSht3x = true;

        delay(50);

        unsigned char readSequencerData[gDs28e18.getSequencerPacketSize()];
        unsigned char measurementRaw[6];

        gDs28e18.writeSequencer(0x000, gDs28e18.getSequencerPacket(), gDs28e18.getSequencerPacketSize());
        gDs28e18.readSequencer(0x000, readSequencerData, sizeof(readSequencerData));
        gDs28e18.runSequencer(0x000, gDs28e18.getSequencerPacketSize());
        gDs28e18.readSequencer(0x00, readSequencerData, sizeof(readSequencerData));

        for(int i = readingLocation; i < readingLocation + 6; i++)
            {
            measurementRaw[i - readingLocation] = readSequencerData[i];
            }

        float temperature = (float)((measurementRaw[0] << 8) | measurementRaw[1])/65536;
        this->m_data.rhTemp.TempC = -45 + (175 * temperature);

        float humidity = (float)((measurementRaw[3] << 8) | measurementRaw[4])/65536;
        this->m_data.rhTemp.Humidity = humidity * 100;
        this->m_data.flags |= Flags::TH;
        }
    else if(!fSht3x)
        {
        gCatena.SafePrintf("SHT3x sensor not detected\n");
        }
    }

/****************************************************************************\
|
|   Start uplink of data
|
\****************************************************************************/

void cMeasurementLoop::startTransmission(
    cMeasurementLoop::TxBuffer_t &b
    )
    {
    auto const savedLed = gLed.Set(McciCatena::LedPattern::Off);
    gLed.Set(McciCatena::LedPattern::Sending);

    // by using a lambda, we can access the private contents
    auto sendBufferDoneCb =
        [](void *pClientData, bool fSuccess)
            {
            auto const pThis = (cMeasurementLoop *)pClientData;
            pThis->m_txpending = false;
            pThis->m_txcomplete = true;
            pThis->m_txerr = ! fSuccess;
            pThis->m_fsm.eval();
            };

    bool fConfirmed = false;
    if (gCatena.GetOperatingFlags() &
        static_cast<uint32_t>(gCatena.OPERATING_FLAGS::fConfirmedUplink))
        {
        gCatena.SafePrintf("requesting confirmed tx\n");
        fConfirmed = true;
        }

    this->m_txpending = true;
    this->m_txcomplete = this->m_txerr = false;

    if (! gLoRaWAN.SendBuffer(b.getbase(), b.getn(), sendBufferDoneCb, (void *)this, fConfirmed))
        {
        // uplink wasn't launched.
        this->m_txcomplete = true;
        this->m_txerr = true;
        this->m_fsm.eval();
        }
    }

void cMeasurementLoop::sendBufferDone(bool fSuccess)
    {
    this->m_txpending = false;
    this->m_txcomplete = true;
    this->m_txerr = ! fSuccess;
    this->m_fsm.eval();
    }

/****************************************************************************\
|
|   The Polling function --
|
\****************************************************************************/

void cMeasurementLoop::poll()
    {
    bool fEvent;

    // no need to evaluate unless something happens.
    fEvent = false;

    // if we're not active, and no request, nothing to do.
    if (! this->m_active)
        {
        if (! this->m_rqActive)
            return;

        // we're asked to go active. We'll want to eval.
        fEvent = true;
        }

	   if (this->m_fTimerActive)
	        {
	        if ((millis() - this->m_timer_start) >= this->m_timer_delay)
	            {
	            this->m_fTimerActive = false;
	            this->m_fTimerEvent = true;
	            fEvent = true;
	            }
        }

    // check the transmit time.
    if (this->m_UplinkTimer.peekTicks() != 0)
        {
        fEvent = true;
        }

    if (fEvent)
        this->m_fsm.eval();

    this->m_data.Vbus = gCatena.ReadVbus();
    setVbus(this->m_data.Vbus);
    }

/****************************************************************************\
|
|   Update the TxCycle count.
|
\****************************************************************************/

void cMeasurementLoop::updateTxCycleTime()
    {
    auto txCycleCount = this->m_txCycleCount;

    // update the sleep parameters
    if (txCycleCount > 1)
            {
            // values greater than one are decremented and ultimately reset to default.
            this->m_txCycleCount = txCycleCount - 1;
            }
    else if (txCycleCount == 1)
            {
            // it's now one (otherwise we couldn't be here.)
            gCatena.SafePrintf("resetting tx cycle to default: %u\n", this->m_txCycleSec_Permanent);

            this->setTxCycleTime(this->m_txCycleSec_Permanent, 0);
            }
    else
            {
            // it's zero. Leave it alone.
            }
    }

/****************************************************************************\
|
|   Handle sleep between measurements
|
\****************************************************************************/

void cMeasurementLoop::sleep()
    {
    const bool fDeepSleep = checkDeepSleep();

    if (! this->m_fPrintedSleeping)
            this->doSleepAlert(fDeepSleep);

    if (fDeepSleep)
            this->doDeepSleep();
    }

// for now, we simply don't allow deep sleep. In the future might want to
// use interrupts on activity to wake us up; then go back to sleep when we've
// seen nothing for a while.
bool cMeasurementLoop::checkDeepSleep()
    {
    bool const fDeepSleepTest = gCatena.GetOperatingFlags() &
                    static_cast<uint32_t>(gCatena.OPERATING_FLAGS::fDeepSleepTest);
    bool fDeepSleep;
    std::uint32_t const sleepInterval = this->m_UplinkTimer.getRemaining() / 1000;

    if (sleepInterval < 2)
        fDeepSleep = false;
    else if (fDeepSleepTest)
        {
        fDeepSleep = true;
        }
#ifdef USBCON
    else if (Serial.dtr())
        {
        fDeepSleep = false;
        }
#endif
    else if (gCatena.GetOperatingFlags() &
                static_cast<uint32_t>(gCatena.OPERATING_FLAGS::fDisableDeepSleep))
        {
        fDeepSleep = false;
        }
    else if ((gCatena.GetOperatingFlags() &
                static_cast<uint32_t>(gCatena.OPERATING_FLAGS::fUnattended)) != 0)
        {
        fDeepSleep = true;
        }
    else
        {
        fDeepSleep = false;
        }

    return fDeepSleep;
    }

void cMeasurementLoop::doSleepAlert(bool fDeepSleep)
    {
    this->m_fPrintedSleeping = true;

    if (fDeepSleep)
        {
        bool const fDeepSleepTest =
                gCatena.GetOperatingFlags() &
                    static_cast<uint32_t>(gCatena.OPERATING_FLAGS::fDeepSleepTest);
        const uint32_t deepSleepDelay = fDeepSleepTest ? 10 : 30;

        gCatena.SafePrintf("using deep sleep in %u secs"
#ifdef USBCON
                            " (USB will disconnect while asleep)"
#endif
                            ": ",
                            deepSleepDelay
                            );

        // sleep and print
        gLed.Set(McciCatena::LedPattern::TwoShort);

        for (auto n = deepSleepDelay; n > 0; --n)
            {
            uint32_t tNow = millis();

            while (uint32_t(millis() - tNow) < 1000)
                {
                gCatena.poll();
                yield();
                }
            gCatena.SafePrintf(".");
            }
        gCatena.SafePrintf("\nStarting deep sleep.\n");
        uint32_t tNow = millis();
        while (uint32_t(millis() - tNow) < 100)
            {
            gCatena.poll();
            yield();
            }
        }
    else
        gCatena.SafePrintf("using light sleep\n");
    }

void cMeasurementLoop::doDeepSleep()
    {
    // bool const fDeepSleepTest = gCatena.GetOperatingFlags() &
    //                         static_cast<uint32_t>(gCatena.OPERATING_FLAGS::fDeepSleepTest);
    std::uint32_t const sleepInterval = this->m_UplinkTimer.getRemaining() / 1000;

    if (sleepInterval == 0)
        return;

    /* ok... now it's time for a deep sleep */
    gLed.Set(McciCatena::LedPattern::Off);
    this->deepSleepPrepare();

    /* sleep */
    gCatena.Sleep(sleepInterval);

    /* recover from sleep */
    this->deepSleepRecovery();

    /* and now... we're awake again. trigger another measurement */
    this->m_fsm.eval();
    }

void cMeasurementLoop::deepSleepPrepare(void)
    {
    Serial.end();
    Wire.end();
    SPI.end();
    if (this->m_pSPI2 && this->m_fSpi2Active)
        {
        this->m_pSPI2->end();
        this->m_fSpi2Active = false;
        }
    
    pinMode(A1, INPUT_PULLUP);
    pinMode(A2, INPUT_PULLUP);
    pinMode(D10, INPUT);
    delay(50);
    pinMode(D11, INPUT);
    delay(50);
    }

void cMeasurementLoop::deepSleepRecovery(void)
    {
    pinMode(D11, OUTPUT);
    delay(50);
    digitalWrite(D11, HIGH);
    pinMode(D11, OUTPUT);
    delay(50);
    digitalWrite(D11, HIGH);
    pinMode(A1, OUTPUT);
    pinMode(A2, OUTPUT);

    Serial.begin();
    Wire.begin();
    SPI.begin();
    //if (this->m_pSPI2)
    //    this->m_pSPI2->begin();
    }

/****************************************************************************\
|
|  Time-out asynchronous measurements.
|
\****************************************************************************/

// set the timer
void cMeasurementLoop::setTimer(std::uint32_t ms)
    {
    this->m_timer_start = millis();
    this->m_timer_delay = ms;
    this->m_fTimerActive = true;
    this->m_fTimerEvent = false;
    }

void cMeasurementLoop::clearTimer()
    {
    this->m_fTimerActive = false;
    this->m_fTimerEvent = false;
    }

bool cMeasurementLoop::timedOut()
    {
    bool result = this->m_fTimerEvent;
    this->m_fTimerEvent = false;
    return result;
    }
