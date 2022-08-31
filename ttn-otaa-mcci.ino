/*******************************************************************************
   Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
   Copyright (c) 2018 Terry Moore, MCCI

   Permission is hereby granted, free of charge, to anyone
   obtaining a copy of this document and accompanying files,
   to do whatever they want with them without any restriction,
   including, but not limited to, copying, modification and redistribution.
   NO WARRANTY OF ANY KIND IS PROVIDED.

   This example sends a valid LoRaWAN packet with payload "Hello,
   world!", using frequency and encryption settings matching those of
   the The Things Network.

   This uses OTAA (Over-the-air activation), where where a DevEUI and
   application key is configured, which are used in an over-the-air
   activation procedure where a DevAddr and session keys are
   assigned/generated for use with all further communication.

   Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
   g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
   violated by this sketch when left running for longer)!

   To use this sketch, first register your application and device with
   the things network, to set or generate an AppEUI, DevEUI and AppKey.
   Multiple devices can use the same AppEUI, but each device has its own
   DevEUI and AppKey.

   Do not forget to define the radio type correctly in
   arduino-lmic/project_config/lmic_project_config.h or from your BOARDS.txt.

 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Wire.h>
#include <RocketScream_RTCAVRZero.h>
#include <RocketScream_LowPowerAVRZero.h>
#define EUI64_CHIP_ADDRESS 0x50
#define EUI64_MAC_ADDRESS 0xF8
#define EUI64_MAC_LENGTH 0x08
#define MAX_DATA_SIZE 2

const uint8_t unusedPins[] = {0, 1, 8, 9, 14, 15, 16, 17, 18, 22, 23, 24, 25};
//
// For normal use, we require that you edit the sketch to replace FILLMEIN
// with values assigned by the TTN console. However, for regression tests,
// we want to be able to compile these scripts. The regression tests define
// COMPILE_REGRESSION_TEST, and in that case we define FILLMEIN to a non-
// working but innocuous value.
//
#ifdef COMPILE_REGRESSION_TEST
# define FILLMEIN 0
#else
# warning "You must replace the values marked FILLMEIN with real values from the TTN control panel!"
# define FILLMEIN (#dont edit this, edit the lines that use FILLMEIN)
#endif

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };;
void os_getArtEui (u1_t* buf) {
  memcpy_P(buf, APPEUI, 8);
}

// This should also be in little endian format, see above.
u1_t DEVEUI[EUI64_MAC_LENGTH];
void os_getDevEui (u1_t* buf) {
  memcpy(buf, DEVEUI, 8);
}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getDevKey (u1_t* buf) {
  memcpy_P(buf, APPKEY, 16);
}

static uint8_t data[MAX_DATA_SIZE];
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;

// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 12,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 11,
  .dio = {10, 13, LMIC_UNUSED_PIN},
};

void printHex2(unsigned v) {
  v &= 0xff;
  if (v < 16)
    Serial2.print('0');
  Serial2.print(v, HEX);
}

void onEvent (ev_t ev) {
  Serial2.print(os_getTime());
  Serial2.print(": ");
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial2.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial2.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial2.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial2.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial2.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial2.println(F("EV_JOINED"));
      {
        u4_t netid = 0;
        devaddr_t devaddr = 0;
        u1_t nwkKey[16];
        u1_t artKey[16];
        LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
        Serial2.print("netid: ");
        Serial2.println(netid, DEC);
        Serial2.print("devaddr: ");
        Serial2.println(devaddr, HEX);
        Serial2.print("AppSKey: ");
        for (size_t i = 0; i < sizeof(artKey); ++i) {
          if (i != 0)
            Serial2.print("-");
          printHex2(artKey[i]);
        }
        Serial2.println("");
        Serial2.print("NwkSKey: ");
        for (size_t i = 0; i < sizeof(nwkKey); ++i) {
          if (i != 0)
            Serial2.print("-");
          printHex2(nwkKey[i]);
        }
        Serial2.println();
      }
      // Disable link check validation (automatically enabled
      // during join, but because slow data rates change max TX
      // size, we don't use it in this example.
      LMIC_setLinkCheckMode(0);
      break;
    /*
      || This event is defined but not used in the code. No
      || point in wasting codespace on it.
      ||
      || case EV_RFU1:
      ||     Serial2.println(F("EV_RFU1"));
      ||     break;
    */
    case EV_JOIN_FAILED:
      Serial2.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial2.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      Serial2.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial2.println(F("Received ack"));
      if (LMIC.dataLen) {
        Serial2.print(F("Received "));
        Serial2.print(LMIC.dataLen);
        Serial2.println(F(" bytes of payload"));
      }
      digitalWrite(LED_BUILTIN, LOW);
      Serial2.flush();
      /* Time in seconds, true: repeat, false: once */
      RTCAVRZero.enableAlarm(TX_INTERVAL, false);
      RTCAVRZero.attachInterrupt(awake);
      /* RTC works down to standby mode only. In power down, PIT is required */
      LowPower.standby();
      // Schedule next transmission to be immediately after this
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(1), do_send);

      break;
    case EV_LOST_TSYNC:
      Serial2.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial2.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial2.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial2.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial2.println(F("EV_LINK_ALIVE"));
      break;
    /*
      || This event is defined but not used in the code. No
      || point in wasting codespace on it.
      ||
      || case EV_SCAN_FOUND:
      ||    Serial2.println(F("EV_SCAN_FOUND"));
      ||    break;
    */
    case EV_TXSTART:
      Serial2.println(F("EV_TXSTART"));
      break;
    case EV_TXCANCELED:
      Serial2.println(F("EV_TXCANCELED"));
      break;
    case EV_RXSTART:
      /* do not print anything -- it wrecks timing */
      break;
    case EV_JOIN_TXCOMPLETE:
      Serial2.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
      break;

    default:
      Serial2.print(F("Unknown event: "));
      Serial2.println((unsigned) ev);
      break;
  }
}

void do_send(osjob_t* j)
{
  unsigned char counter;
  float batteryVoltage;
  int adcReading;
  int voltage;
  
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial2.println(F("OP_TXRXPEND, not sending"));
  } else {
	// Enable pin D17 (A5) used for battery voltage monitoring
    LowPower.enablePinISC(17);
    pinMode(17, INPUT);
    
    delay(100);
    adcReading = analogRead(A5);
    // Discard inaccurate 1st reading
    adcReading = 0;
    // Perform averaging
    for (counter = 10; counter > 0; counter--)
    {
      adcReading += analogRead(A5);
    }
    // Disable D17 (A5) digital input buffer to reduce power consumption
    LowPower.disablePinISC(17);

    adcReading = adcReading / 10;
    // Convert to volts
    batteryVoltage = adcReading * (6.6 / 1023.0);

    Serial2.print(F("Battery: "));
    Serial2.print(batteryVoltage);
    Serial2.println(F(" V"));

    // Pack float into int with 2 decimal point resolution
    voltage = batteryVoltage * 100;
    data[0] = voltage >> 8;
    data[1] = voltage;
    // Prepare upstream data transmission at the next possible time.
    LMIC_setTxData2(1, data, sizeof(data), 0);

    Serial2.println(F("Packet queued"));
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

void setDevEui(unsigned char* buf)
{
  Wire.begin();
  Wire.beginTransmission(EUI64_CHIP_ADDRESS);
  Wire.write(EUI64_MAC_ADDRESS);
  Wire.endTransmission();
  Wire.requestFrom(EUI64_CHIP_ADDRESS, EUI64_MAC_LENGTH);

  // Format needs to be little endian (LSB...MSB)
  while (Wire.available())
  {
    *buf-- = Wire.read();
  }
}

void setup()
{
  uint8_t index;
  int count;

  /* Ensure unused pins are not floating */
  /* Pin D17 (A5) is being used but for code simplicity, we disable it first */
  /* in setup() before enabling it during operation */
  for (index = 0; index < sizeof(unusedPins); index++)
  {
    pinMode(unusedPins[index], OUTPUT);
    digitalWrite(unusedPins[index], LOW);
    LowPower.disablePinISC(unusedPins[index]);
  }
  
  Serial2.swap(1);
  Serial2.begin(115200);
  Serial2.println(F("Starting"));
  setDevEui(&DEVEUI[EUI64_MAC_LENGTH - 1]);

  Serial2.print(F("DEVEUI: "));

  for (count = EUI64_MAC_LENGTH; count > 0; count--)
  {
    printHex2(DEVEUI[count - 1]);
  }
  Serial2.println();

  /* false: internal 32.768 kHz ULP oscillator */
  RTCAVRZero.begin(true);

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();
  LMIC.dn2Dr = DR_SF9;        // TTN uses SF9 for its RX2 window.
  LMIC_setClockError(MAX_CLOCK_ERROR * 10 / 100);
  // Start job (sending automatically starts OTAA too)
  do_send(&sendjob);
}

void loop() {
  os_runloop_once();
}

void awake(void)
{
  digitalWrite(LED_BUILTIN, HIGH);
  RTCAVRZero.disableAlarm();
  RTCAVRZero.detachInterrupt();
}
