#include <TimerTC3.h>
#include <Adafruit_DPS310.h>
#include <math.h>
#include <SensirionI2CSdp.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <TORICA_SD.h>

SensirionI2CSdp sdp;
uint16_t error;
char errorMessage[256];

int cs_SD = 2;
TORICA_SD sd(cs_SD);
char SD_BUF[256];


Adafruit_DPS310 dps;
sensors_event_t temp_event, pressure_event;

float altitude_progress ;
float altitude = 0;
float pressure = 0;
float temperature = 0;
float rho_kgm3 = 0;
float airspeed_ms = 0;
float differentialPressure_Pa = 0;
float temperature_C = 0;

char buf[256];

void setup() {
  Serial.begin(460800);
  Serial1.begin(460800);

  //追加
  Wire.begin();

  sdp.begin(Wire, SDP8XX_I2C_ADDRESS_0);

  uint32_t productNumber;
  uint8_t serialNumber[8];
  uint8_t serialNumberSize = 8;

  sdp.stopContinuousMeasurement();

  error = sdp.readProductIdentifier(productNumber, serialNumber,
                                    serialNumberSize);
  if (error) {
    Serial.print("Error trying to execute readProductIdentifier(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  } else {
    Serial.print("ProductNumber:");
    Serial.print(productNumber);
    Serial.print("\t");
    Serial.print("SerialNumber:");
    Serial.print("0x");
    for (size_t i = 0; i < serialNumberSize; i++) {
      Serial.print(serialNumber[i], HEX);
    }
    Serial.println();
  }
  error = sdp.startContinuousMeasurementWithDiffPressureTCompAndAveraging();

  if (error) {
    Serial.print(
      "Error trying to execute "
      "startContinuousMeasurementWithDiffPressureTCompAndAveraging(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  }


  Serial.println("DPS310");
  if (! dps.begin_I2C()) {             // Can pass in I2C address here
    //if (! dps.begin_SPI(DPS310_CS)) {  // If you want to use SPI
    Serial.println("Failed to find DPS");
    while (1) yield();
  }


  Serial.println("DPS OK!");

  dps.configurePressure(DPS310_32HZ, DPS310_16SAMPLES);
  dps.configureTemperature(DPS310_32HZ, DPS310_2SAMPLES);


  //追加
  delay(100);

  NVIC_SetPriority((IRQn_Type)TC3_IRQn, 3);
  TimerTc3.initialize(2000);
  TimerTc3.attachInterrupt(timerIsr);

  sd.begin();
}


void loop() {

  // SDに書き込み
  sd.flash();
  delay(100);

}

const int readUART_BUF_SIZE = 256;
char readUART_BUF[readUART_BUF_SIZE];

void timerIsr() {
  if (Serial1.available()) {
    int read_length = Serial1.available();
    if (read_length >= readUART_BUF_SIZE - 1) {
      read_length = readUART_BUF_SIZE - 1;
    }
    Serial1.readBytes(readUART_BUF, read_length);
    readUART_BUF[read_length] = '\0';
    sd.add_str(readUART_BUF);

  }


/*
  if (dps.temperatureAvailable() && dps.pressureAvailable()) {
    dps.getEvents(&temp_event, &pressure_event);
    pressure = pressure_event.pressure;
    temperature = temp_event.temperature;

    altitude_progress = pow(1013.25 / pressure, 1 / 5.257);
    altitude = (altitude_progress - 1) * (temperature + 273.15) / 0.0065;

    //追加
    error = sdp.readMeasurement(differentialPressure_Pa, temperature_C);
    if (error) {
      Serial.print("Error trying to execute readMeasurement(): ");
      errorToString(error, errorMessage, 256);
      Serial.println(errorMessage);
    } else {
      rho_kgm3 = 0.0034837 * 101325.0 / (temperature_C + 273.5);
      airspeed_ms = sqrt(abs(2.0 * differentialPressure_Pa / rho_kgm3));

    }
*/
    sprintf(buf, "%.2f,%.2f,%.2f,%.2f,%.2f\n", pressure, temperature, altitude, differentialPressure_Pa , airspeed_ms);
    Serial1.print(buf);


  }

}
