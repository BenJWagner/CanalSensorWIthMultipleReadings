/*
  Bridge Sensor with GSM, SDCard, thermistor, US and RTC.
  Sketch used by MSI Sensors platform.
  Created 04JUL16
  Modified 04AUG16
*/
#include <ARTF_SDCard.h>
#include <SdFat.h>

#include <math.h>
#include <Time.h>
#include <Adafruit_FONA.h>
#include <LowPower.h>

#include <SPI.h>


// RTC Dependency
#include <Wire.h>
#include <RTClib.h>

// Analog Pins
#define THERMISTOR_PIN A1
#define ULTRASONIC_PIN A2

//Analog pins for HC-SR04. Block these if using Maxbotix US sensor.
int echoPin = 13;
int trigPin = 12;
int duration;
int cm;

//GSM pins
#define FONA_RX 9
#define FONA_TX 8
#define FONA_RST 4
#define FONA_RI 7
#define FONA_PWR 11

// this is a large buffer for replies
char replybuffer[255];

// Digital Pins
// Digital Pin for Maxbotix US sensor. Block if using HC-SR04.
//#define US_PIN    10
//GSM DTR power pin. Adafruit documentation says to cut the KEY to use this. Otherwise GMS always remains on.
#define GSM_PIN   5
//Thermistor power pin.
#define THERM_PIN        6
//GMS Power Key. Verify if this is needed.
//#define GSM_PWRKEY       9
//SD CS pin. Seemed to work on pin 10.



// Settings
// -------------------------------
//Distance from sensor to river bed in centimeters.
#define SENSOR_TO_RIVER_BED        51
//Sensor number. Jalal's unique identifier.
#define SENSOR_NUM                3
//Number of readings per text message.
#define SEND_DATA_AFTER_X_READINGS              3
//Each sleep cycle is approximately 8 seconds.
#define SLEEP_CYCLES               900
//Number of thermistor readings to be averaged.
#define NUM_THERM_READINGS         5
//Delay in miliseconds between temperature readings.
#define THERM_READING_DELAY        400
//Number of distance readings to be averaged.
#define NUM_DISTANCE_READINGS      5
//Delay in milliseconds between distance readings.
#define DISTANCE_READING_DELAY     400
//Character type between data points.
#define DATA_DELIM                 ':'
#define BACKUP_FILENAME            "backup.csv"
#define UNSENT_FILENAME            "unsent.csv"
#define ERROR_FILENAME             "error.csv"
// Google Voice phone number for msi.artf2@gmail.com
//#define PHONE_NUMBER               "+93771515622"
//char PHONE_NUMBER[21] = "+19072236094";////////////////////////////////new changes/////////////// phone number changed to char array since fona library dosen't support phone as as string
char PHONE_NUMBER[21] = "282"; // GoogleVoice for msi.artf2@gmail.com
//char PHONE_NUMBER[21] = "+93707863874"; // GoogleVoice for msi.artf2@gmail.com

char EMAIL_ID[121] = "COM msi.artf2@gmail.com ";

#define ERROR_GSM                  "GSM Failed"
#define ERROR_SMS                  "SMS Failed"

#define SD_CS_PIN                   10
ARTF_SDCard sd(SD_CS_PIN);
//Fona library requirment
#include <SoftwareSerial.h>
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;

Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);


// Custom Datatypes
typedef struct {
  int distance;
  int temperature;
  int sensorNum;
  //int vbat;
  DateTime timestamp;
} SensorReading;


// Global Variables
int numCachedReadings = 0;
int totalReadings = 0;
SensorReading sensorReadings[SEND_DATA_AFTER_X_READINGS];



RTC_PCF8523  rtc;


void setup()
{
  // SC card CS pin defined.
  pinMode(SD_CS_PIN, OUTPUT);
  //Thermistor pin defined.
  pinMode(THERM_PIN, OUTPUT);
  // PinMode settings for HC-SR04 ultrasonic sensor. Block these if using Maxbotix US sensor.
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(FONA_PWR, OUTPUT);

  //Turn on GSM.
  digitalWrite(FONA_PWR, HIGH);
  delay(2000);
  digitalWrite(FONA_PWR, LOW);
  delay(8000);

  //  while (!Serial);

  Serial.begin(115200);
  Serial.println(F("FONA SMS caller ID test"));
  Serial.println(F("Initializing....(May take 3 seconds)"));

  fonaSerial->begin(4800);
  if (! fona.begin(*fonaSerial)) {
    Serial.println(F("Couldn't find FONA"));
    while (1);
  }
  Serial.println(F("FONA is OK"));

  uint16_t vbat;
  fona.getBattPercent(&vbat);


  String testMessage = String("COM msi.artf2@gmail.com Test Message! BV: ") + String(vbat) + String("%");

  char message[121];
  strncpy(message, testMessage.c_str(), sizeof(message));
  message[sizeof(message) - 1] = 0;

  delay(20000);

  if (!fona.sendSMS(PHONE_NUMBER, message)) {
    Serial.println(F("Not Sent"));
  } else {
    Serial.println(F("SMS Sent"));
  }


  //Turn off GSM.
  digitalWrite(FONA_PWR, HIGH);
  delay(2000);
  digitalWrite(FONA_PWR, LOW);
  delay(2000);
  // Reset number of cache
  //rtc.adjust(DateTime(2016, 10, 49, 10, 50, 10));
  //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

}


void loop()
{


  for (int i = 0; i < SLEEP_CYCLES; ++i)
  {
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
    digitalWrite(FONA_PWR, HIGH);  //keeps the GSM off
  }



  //
  double temperature = takeThermReading();
  double distance = takeDistanceReading(temperature);

  int roundedTemperature = round(temperature);
  int roundedDistance = round(distance);



  //  // 12. Get time from RTC Shield.
  //  // -----------------------------
  Wire.begin();
  rtc.begin();

  DateTime   unixTime = rtc.now();

  // Cache distance and time in global array variable
  sensorReadings[numCachedReadings].sensorNum = SENSOR_NUM;
  //sensorReadings[numCachedReadings].vbat = 0 ;
  sensorReadings[numCachedReadings].timestamp = unixTime.unixtime();
  sensorReadings[numCachedReadings].distance = roundedDistance;
  sensorReadings[numCachedReadings].temperature = roundedTemperature;



  numCachedReadings += 1;



  // 14. Are there 4 unsent data strings?
  // 15. Yes. Send 4 unsent data strings in one SMS. Go to 18.
  // -------------------------------------
  if (numCachedReadings == SEND_DATA_AFTER_X_READINGS)
  {
    //Turn on GSM.
    digitalWrite(FONA_PWR, HIGH);
    delay(2000);
    digitalWrite(FONA_PWR, LOW);
    delay(15000);


    fonaSerial->begin(4800);
    if (! fona.begin(*fonaSerial)) {
      Serial.println(F("Couldn't find FONA"));
      while (1);
    }


    delay(20000);

    uint16_t vbat;
    fona.getBattPercent(&vbat);

    String textMessage = String(EMAIL_ID) + " " +
                         String(sensorReadings[0].sensorNum) + " " +
                         String(vbat) + " " +
                         String(sensorReadings[0].timestamp.unixtime()) + " " +
                         String(sensorReadings[0].distance) + " " +
                         String(sensorReadings[0].temperature);


    for (int i = 1; i < numCachedReadings; ++i)
    {
      textMessage += String(DATA_DELIM) + String(sensorReadings[i].sensorNum) + " " + String(vbat) + " " + String(sensorReadings[i].timestamp.unixtime()) + " " + String(sensorReadings[i].distance) + " " + String(sensorReadings[i].temperature);
    }



    char message[121];
    strncpy(message, textMessage.c_str(), sizeof(message));
    message[sizeof(message) - 1] = 0;


    sd.begin();
    //
    if (!fona.sendSMS(PHONE_NUMBER, message)) {
      Serial.println(F("Not sent!"));
      if (!sd.writeFile(UNSENT_FILENAME, message) ) {
        Serial.print("SD Card not available\n");
      } else {
        Serial.print("Sd card available\n");
      }
    } else {
      Serial.println(F("Sent"));
      if (!sd.writeFile(BACKUP_FILENAME, message)) {
        Serial.print("SD Card not available\n");
      } else {
        Serial.print("Sd card available\n");
      }
    }

    //Turn off GSM.
    digitalWrite(FONA_PWR, HIGH);
    delay(2000);
    digitalWrite(FONA_PWR, LOW);
    delay(2000);

    // Reset number of cached readings
    numCachedReadings = 0;


  }
}


//
//
double takeThermReading()
{
  // 2. Turn on thermistor.
  // ----------------------
  digitalWrite(THERM_PIN, HIGH);

  // 3. Take 5 thermistor readings. (one every 20ms)
  // -----------------------------------------------
  int thermReadings[NUM_THERM_READINGS];
  for (int i = 0; i < NUM_THERM_READINGS; ++i)
  {
    thermReadings[i] = analogRead(THERMISTOR_PIN);
    delay(THERM_READING_DELAY);
  }

  // 4. Turn off thermistor.
  // -----------------------
  digitalWrite(THERM_PIN, LOW);
  delay(500);

  // 5. Average 5 thermistor readings.
  // ---------------------------------
  double sumTherm = 0;
  for (int i = 0; i < NUM_THERM_READINGS; ++i)
  {
    sumTherm += thermReadings[i];
  }
  double avgTherm = sumTherm / NUM_THERM_READINGS;
  avgTherm = 1023 / avgTherm - 1;
  double R = 10000 / avgTherm;


  // 6. Convert average thermistor reading into temperature.
  // -------------------------------------------------------

  // Steinhart-Hart, modified:
  double avgTemperature = ( 3950.0 / (log( R / (10000.0 * exp( -3950.0 / 298.13 ) ) ) ) ) - 273.13;

  return avgTemperature;
}

double takeDistanceReading(double temperature)
{




  // 7. Turn on ultrasonic US sensor (MOSFET).
  // --------------------------------------
  //digitalWrite(MOSFET_US_PIN, HIGH);
  // Calibration time
  //delay(3000);


  // 8. Take 3 distance readings. (One every 200ms)
  // ----------------------------------------------
  int distanceReadings[NUM_DISTANCE_READINGS];
  for (int i = 0; i < NUM_DISTANCE_READINGS; ++i)
  {
    digitalWrite(trigPin, LOW);
    delay(200);
    digitalWrite(trigPin, HIGH);
    delay(200);
    digitalWrite(trigPin, LOW);

    distanceReadings[i] = pulseIn(echoPin, HIGH);
    //Only for US sensor
    //distanceReadings[i] = analogRead(ULTRASONIC_PIN) * DISTANCE_INCREMENT;
    delay(DISTANCE_READING_DELAY);
  }


  // 9. Turn off ultrasonic US sensor (MOSFET).
  // ---------------------------------------
  //digitalWrite(MOSFET_US_PIN, LOW);
  //delay(500);


  // 10. Average 3 distance measurements.
  // ------------------------------------
  double sumDistance = 0.0;
  for (int i = 0; i < NUM_DISTANCE_READINGS; ++i)
  {
    sumDistance += distanceReadings[i];
  }

  //averaging and converting to CM
  double avgDistance = (sumDistance / NUM_DISTANCE_READINGS) / 29 / 2;


  // 11. Use average temperature to calculate actual distance.
  // ---------------------------------------------------------
  double adjustedDistance = ( ( 331.1 + .6 * temperature ) / 344.5 ) * avgDistance;

  if (SENSOR_TO_RIVER_BED > 0)
  {
    adjustedDistance = SENSOR_TO_RIVER_BED - adjustedDistance;
  }
  if (adjustedDistance < 0)
  {
    adjustedDistance = 0;
  }

  return adjustedDistance;
}

