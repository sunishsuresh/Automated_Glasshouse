#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <DHT.h>


//Millis
uint32_t previousMillis1 = 0UL;
uint32_t previousMillis2 = 0UL;


//soil moisture sensor
const int moisturesensor1 = A0;
const int moisturesensor2 = A1;
const int moisturesensor3 = A2;
float avg_soilMoisturepercent=0.00;

//LDR sensor
unsigned int LightValue = analogRead(A3);
unsigned int darkness = 1000;
unsigned int light = 400;
int LightPercent;

//Water Logic Bits
bool Waterring = 0;

//soil mositure 1 calibration
const int AirValue = 620;   //replace the value with value when placed in air using calibration code
const int WaterValue = 275; //replace the value with value when placed in water using calibration code
int soilMoistureValue1 = 0;
int soilmoisturepercent1 = 0;

//soil mositure 2 calibration
//const int AirValue2 = 576;   //replace the value with value when placed in air using calibration code
//const int WaterValue2 = 301; //replace the value with value when placed in water using calibration code
int soilMoistureValue2 = 0;
int soilmoisturepercent2 = 0;

//soil mositure 3 calibration
const int AirValue3 = 576;   //replace the value with value when placed in air using calibration code
const int WaterValue3 = 301; //replace the value with value when placed in water using calibration code
int soilMoistureValue3 = 0;
int soilmoisturepercent3 = 0;


//LDR sensor
const int LDRsensor = A3;

// We define the digital pin where the DHT11 sensor is connected
#define DHTPIN 3
// We define the sensor model
#define DHTTYPE DHT11
// We initialize the DHT11 sensor
DHT dht(DHTPIN, DHTTYPE);

const int Trigger = 4;   //Digital Pin 4 for ultrasonic sensor Trigger
const int Echo = 5;   //Digital Pin 5 for ultrasonic sensor Echo
const int Empty = 20;   //replace the value with value when placed in air using calibration code
const int Full = 5; //replace the value with value when placed in water using calibration code
//const int height = 50;  //tank height in cm

// if the soil is dryer than this number, then start watering
const float dry = 25;//we'll choose this number during experiment
const int pumpPin = 31;


// LoRaWAN NwkSKey, network session key from TTN - Change this with your Unique Key
// LoRaWAN NwkSKey, network session key
// This is the default Semtech key, which is used by the prototype TTN
// network initially.
//ttn
static const PROGMEM u1_t NWKSKEY[16] = { 0x29, 0xDE, 0x26, 0x68, 0x7E, 0x25, 0x82, 0x18, 0xD9, 0xCB, 0x2F, 0x10, 0x3B, 0xD9, 0x28, 0x52 };
// LoRaWAN AppSKey, application session key
// This is the default Semtech key, which is used by the prototype TTN
// network initially.
//ttn
static const u1_t PROGMEM APPSKEY[16] = { 0x94, 0xF6, 0xA8, 0xD2, 0x33, 0x24, 0x81, 0xA9, 0x66, 0x10, 0xBB, 0xD7, 0x2A, 0xF7, 0x01, 0xDA };
//
// LoRaWAN end-device address (DevAddr)
// See http://thethingsnetwork.org/wiki/AddressSpace
// ttn
static const u4_t DEVADDR = 0x260B34DE;

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static uint8_t mydata[9] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 10;

void Data()
{
  if (millis() - previousMillis1 >= 1000)
  {
    /********************************************LDR Sensor**********************************************/
    LightPercent = map(LightValue, darkness, light, 0, 100);

    if (LightPercent > 100)
    {
      LightPercent = 100;
      Serial.print("the luminosity percent is: ");
      Serial.println(LightPercent);
    }
    else if (LightPercent < 0)
    {
      LightPercent = 0;
      Serial.print("the luminosity percent is: ");
      Serial.println(LightPercent);
    }
    else if (LightPercent > 0 && LightPercent < 100)
    {
      Serial.print("the luminosity percent is: ");
      Serial.println(LightPercent);
    }

    /********************************************DHT11 Sensor**********************************************/
    // We read the relative humidity
    int humidity = dht.readHumidity();
    // We read the temperature in degrees centigrade (by default)
    int temperature = dht.readTemperature();
    // We check if there has been an error in the reading
    if (isnan(humidity) || isnan(temperature)) {
      Serial.println("Error getting data from DHT11 sensor");
      return;
    }
    Serial.println(humidity);
    Serial.println(temperature);
    delay(1000);
    /******************************************************************************************************/

    /********************************************Soil moisture 1 Sensor**************************************/
    int soilmoisture1 = analogRead(moisturesensor1);
    Serial.println("Soil moisture is: ");
    Serial.println(soilmoisture1);
    soilmoisturepercent1 = map(soilmoisture1, AirValue, WaterValue, 0, 100);

    if (soilmoisturepercent1 >= 100)
    {
      Serial.println("100 %");
    }
    else if (soilmoisturepercent1 <= 0)
    {
      Serial.println("0 %");
      delay(250);
    }
    else if (soilmoisturepercent1 > 0 && soilmoisturepercent1 < 100)
    {
      Serial.print(soilmoisturepercent1);
      Serial.println("%");
      delay(1000);
    }


    /********************************************Soil moisture 2 Sensor**************************************/
    int soilmoisture2 = analogRead(moisturesensor2);
    Serial.println("Soil 2 moisture is: ");
    Serial.println(soilmoisture2);
    soilmoisturepercent2 = map(soilmoisture2, AirValue, WaterValue, 0, 100);

    if (soilmoisturepercent2 >= 100)
    {
      Serial.println("100 %");
    }
    else if (soilmoisturepercent2 <= 0)
    {
      Serial.println("0 %");
      delay(250);
    }
    else if (soilmoisturepercent2 > 0 && soilmoisturepercent2 < 100)
    {
      Serial.print(soilmoisturepercent2);
      Serial.println("%");
      delay(1000);
    }

    /********************************************Soil moisture 3 Sensor**************************************/
    int soilmoisture3 = analogRead(moisturesensor3);
    Serial.println("Soil moisture is: ");
    Serial.println(soilmoisture3);
    soilmoisturepercent3 = map(soilmoisture3, AirValue, WaterValue, 0, 100);

    if (soilmoisturepercent3 >= 100)
    {
      Serial.println("100 %");
    }
    else if (soilmoisturepercent3 <= 0)
    {
      Serial.println("0 %");
      delay(250);
    }
    else if (soilmoisturepercent3 > 0 && soilmoisturepercent3 < 100)
    {
      Serial.print(soilmoisturepercent3);
      Serial.println("%");
      delay(1000);
    }

    /********************************************Soil moisture Average**************************************/

    avg_soilMoisturepercent = (soilmoisturepercent1 + soilmoisturepercent2 + soilmoisturepercent3) / 3;
    Serial.print("Average Soil moisture is: ");
    Serial.print(avg_soilMoisturepercent);
    Serial.println("%");

    /********************************************ultrasonic Sensor*****************************************/
    long t; //time it takes for the echo to arrive
    long d; //distance in centimeters
    long cap; //water level in tank in cm
    digitalWrite(Trigger, HIGH);
    delayMicroseconds(10);          //We send a pulse of 10us
    digitalWrite(Trigger, LOW);
    t = pulseIn(Echo, HIGH); //we get the pulse width
    d = t * 0.034 / 2;           //we scale time to a distance in cm
    int tanklevel = map(d, Empty, Full, 0, 100);
    if (tanklevel >= 100)
    {
      tanklevel = 100;
      Serial.println("actual water level is: ");
      Serial.println(tanklevel);
    }
    else if (tanklevel <= 0)
    {
      tanklevel = 0;
      Serial.println("actual water level is: ");
      Serial.println(tanklevel);

      delay(250);
    }
    else if (tanklevel > 0 && tanklevel < 100)
    {
      Serial.println("water level is: ");
      Serial.print(tanklevel);
      Serial.println("%");
      delay(250);
    }
    /******************************************************************************************************/

    mydata[0] = humidity;
    mydata[1] = temperature;
    mydata[2] = soilmoisturepercent1;
    mydata[3] = soilmoisturepercent2;
    mydata[4] = soilmoisturepercent3;
    mydata[5] = avg_soilMoisturepercent;
    mydata[6] = tanklevel;
    mydata[7] = LightPercent;
    previousMillis1 = millis();
  }




  /****************************************************WATERRING LOGIC**************************************************/
  if (millis() - previousMillis2 >= 3000)
  {
    //water soil if needed
    Serial.print("average soil moisture");
    Serial.println(avg_soilMoisturepercent);
    if (avg_soilMoisturepercent <= 25.00) {
      // the soil is less than dry minus the tolerance value, it will start the pump water!
      Serial.println("Watering starts now..moisture is " + String(avg_soilMoisturepercent));
      Waterring = 1;
    }
    else {
      Serial.println("Moisture is adequate. No watering needed " + String(avg_soilMoisturepercent));
    }
    if (Waterring == 1 && avg_soilMoisturepercent >= 30.00) {
      // the soil is less than dry minus the tolerance value, it will start the pump water!
      Serial.println("Watering stops now..moisture is " + String(avg_soilMoisturepercent));
      Waterring = 0;
      digitalWrite(pumpPin, HIGH);
      Serial.println("Done watering.");
    }
    if (Waterring == 1)
    {
      digitalWrite(pumpPin, LOW);
      Serial.println("Watering pump running for 5 sec and the moisture is " + String(avg_soilMoisturepercent));
      // keep watering for 5 sec (we'll discuss this period)
      delay(500);
      digitalWrite(pumpPin, HIGH);
    }
    delay(5000);

  }
}
// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 9,
  .dio = {2, 6, 7},
};

void onEvent (ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      break;
    case EV_RFU1:
      Serial.println(F("EV_RFU1"));
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ack"));
      if (LMIC.dataLen) {
        Serial.println(F("Received "));
        Serial.println(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
        int y = LMIC.dataLen;

      }
      // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    default:
      Serial.println(F("Unknown event"));
      break;
  }
}
void do_send(osjob_t* j) {
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND)
  {
    Serial.println(F("OP_TXRXPEND, not sending"));
  }
  else
  {
    Data();
    // Prepare upstream data transmission at the next possible time.
    LMIC_setTxData2(1, mydata, sizeof(mydata) - 1, 0);
    Serial.println(F("Packet queued"));
  }
  // Next TX is scheduled after TX_COMPLETE event.
}
void setup() {
  //analogReference(INTERNAL1V1);
  Serial.begin(9600);

  //soil moisture sensor
  pinMode(moisturesensor1, INPUT); // Configuring pin A2 as input

  // We start the DHT sensor
  dht.begin();
  //ultrasonic sensor
  pinMode(Trigger, OUTPUT); //pin como salida
  pinMode(Echo, INPUT);  //pin como entrada
  digitalWrite(Trigger, LOW);//Inicializamos el pin con 0

  //water pump
  pinMode(pumpPin, OUTPUT);
  digitalWrite(pumpPin, HIGH);

  Serial.println(F("Starting"));

#ifdef VCC_ENABLE
  // For Pinoccio Scout boards
  pinMode(VCC_ENABLE, OUTPUT);
  digitalWrite(VCC_ENABLE, HIGH);
  delay(1000);
#endif

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  // Set static session parameters. Instead of dynamically establishing a session
  // by joining the network, precomputed session parameters are be provided.
#ifdef PROGMEM
  // On AVR, these values are stored in flash and only copied to RAM
  // once. Copy them to a temporary buffer here, LMIC_setSession will
  // copy them into a buffer of its own again.
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
#else
  // If not running an AVR with PROGMEM, just use the arrays directly
  LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
#endif

  LMIC_setLinkCheckMode(0);

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;

  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF7, 14);

  // Start job
  do_send(&sendjob);
}

void loop()
{
  os_runloop_once();
}
