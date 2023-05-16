#include <TimerTCC0.h>
#include <Adafruit_DPS310.h>
#include <math.h>

Adafruit_DPS310 dps;
bool timer_100Hz = false;

sensors_event_t temp_event, pressure_event;
float altitude_progress ;
float altitude = 0;
float pressure = 0;
float temperature = 0;



void setup() {
  Serial.begin(460800);
  Serial1.begin(460800);

  TimerTcc0.initialize(10000);  //10,000us=100Hz
  TimerTcc0.attachInterrupt(timerIsr);

  Serial.println("DPS310");
  if (! dps.begin_I2C()) {             // Can pass in I2C address here
    //if (! dps.begin_SPI(DPS310_CS)) {  // If you want to use SPI
    Serial.println("Failed to find DPS");
    while (1) yield();
  }

  Serial.println("DPS OK!");

  dps.configurePressure(DPS310_32HZ, DPS310_16SAMPLES);
  dps.configureTemperature(DPS310_32HZ, DPS310_2SAMPLES);
}


void loop() {
  if (timer_100Hz) {
    timer_100Hz = false;
    if (dps.temperatureAvailable() && dps.pressureAvailable()) {

      dps.getEvents(&temp_event, &pressure_event);
      pressure = pressure_event.pressure;
      temperature = temp_event.temperature;

      altitude_progress = pow(1013.25 / pressure, 1 / 5.257);
      altitude = (altitude_progress -1)* (temperature + 273.15) / 0.0065;
    }
    char buf[256];
    sprintf(buf,"%.2f,%.2f,%.2f,%.2f,%.2f\n", pressure, temperature, altitude, 0.0, 0.0);
    Serial1.print(buf);

    //Serial1.print(%d,%,pressure_event.pressure","temp_event.temperature","altitude",差圧,対気速度\n");
    //Serial1.print("1013.25,23.4,1.23,4.56,7.89\n");
    //気圧[hPa],温度[deg],気圧高度[m],差圧[Pa],対気速度[m/ss]

  }

  if (Serial1.available()) {
    char c = (char)Serial1.read();
    Serial.write(c);
  }
}

void timerIsr() {
  if (timer_100Hz) {
    //Serial.println("100Hz overrun");
  } else {
    timer_100Hz = true;
  }
}
