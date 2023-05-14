float altitude_m = 0;
float pressure_hPa = 0;
float temperature_deg = 0;
float differential_pressure_Pa = 0;
float air_speed_ms = 0;

#include <TORICA_SD.h>
int cs_SD = 2;
TORICA_SD sd(cs_SD);
char SD_BUF[256];

#include <Adafruit_DPS310.h>
Adafruit_DPS310 dps;
sensors_event_t temp_event, pressure_event;
#include <math.h>

#include <Seeed_Arduino_FreeRTOS.h>
#define  ERROR_LED_PIN  LED_BUILTIN
#define ERROR_LED_LIGHTUP_STATE  LOW
TaskHandle_t Handle_sendUART;
TaskHandle_t Handle_readUART;
TaskHandle_t Handle_flashSD;
TaskHandle_t Handle_monitorTask;
void myDelayMsUntil(TickType_t* previousWakeTime, int ms) {
  // portTICK_PERIOD_US = 1000us = 1ms
  vTaskDelayUntil(previousWakeTime, (ms * 1000) / portTICK_PERIOD_US);
}

void setup() {
  vNopDelayMS(1000); // prevents usb driver crash on startup, do not omit this
  SerialUSB.begin(460800);
  Serial1.begin(460800);

  // Set the led the rtos will blink when we have a fatal rtos error
  // RTOS also Needs to know if high/low is the state that turns on the led.
  // Error Blink Codes:
  //    3 blinks - Fatal Rtos Error, something bad happened. Think really hard about what you just changed.
  //    2 blinks - Malloc Failed, Happens when you couldn't create a rtos object.
  //               Probably ran out of heap.
  //    1 blink  - Stack overflow, Task needs more bytes defined for its stack!
  //               Use the taskMonitor thread to help gauge how much more you need
  vSetErrorLed(ERROR_LED_PIN, ERROR_LED_LIGHTUP_STATE);

  sd.begin();

  SerialUSB.println("DPS310");
  if (! dps.begin_I2C()) {
    SerialUSB.println("Failed to find DPS");
    while (1) {
      Serial1.print("-1,-1,-1,0.0,0.0\n");
      delay(30);
    };
  }
  SerialUSB.println("DPS OK!");
  dps.configurePressure(DPS310_32HZ, DPS310_16SAMPLES);
  dps.configureTemperature(DPS310_32HZ, DPS310_2SAMPLES);

  xTaskCreate(readUART, "readUART", 256, NULL, tskIDLE_PRIORITY + 2, &Handle_readUART);
  xTaskCreate(sendUART, "sendUART", 256, NULL, tskIDLE_PRIORITY + 1, &Handle_sendUART);
  xTaskCreate(flashSD, "flashSD", 256, NULL, tskIDLE_PRIORITY, &Handle_flashSD);
  //xTaskCreate(taskMonitor, "Task Monitor", 256, NULL, tskIDLE_PRIORITY, &Handle_monitorTask);

  vTaskStartScheduler();
}

// loop never run
void loop() {
}

char sendUART_BUF[256];
void sendUART(void* pvParameters) {
  portTickType xLastWakeTime = xTaskGetTickCount();
  float altitude_progress;
  while (true) {
    // DPS310_32HZ ×3 = 96Hz = 10ms
    myDelayMsUntil(&xLastWakeTime, 10);
    if (dps.temperatureAvailable() && dps.pressureAvailable()) {

      dps.getEvents(&temp_event, &pressure_event);
      pressure_hPa = pressure_event.pressure;
      temperature_deg = temp_event.temperature;

      altitude_progress = pow(1013.25 / pressure_hPa, 1 / 5.257);
      altitude_m = (altitude_progress - 1) * (temperature_deg + 273.15) / 0.0065;

      //ToDo read SDP810

      sprintf(sendUART_BUF, "%d,%.2f,%.2f,%.2f,%.2f,%.2f\n", millis(), pressure_hPa, temperature_deg, altitude_m, differential_pressure_Pa, air_speed_ms);
      Serial1.print(sendUART_BUF);
    }
  }
}

// C:\Users\XXX\AppData\Local\Arduino15\packages\Seeeduino\hardware\samd\1.x.x\cores\arduino\RingBuffer.h
const int readUART_BUF_SIZE = 256;
char readUART_BUF[readUART_BUF_SIZE];
void readUART(void* pvParameters) {
  portTickType xLastWakeTime = xTaskGetTickCount();
  while (true) {
    // 100HZ = 10ms
    // 記録できればいいのでリアルタイム性は求めない．
    //バッファに収まりデータを取りこぼさず，スループットがあればOK
    myDelayMsUntil(&xLastWakeTime, 10);
    if (Serial1.available()) {
      int i = 0;
      int read_length = Serial1.available();
      if (read_length >= readUART_BUF_SIZE - 1) {
        read_length = readUART_BUF_SIZE - 1;
      }
      Serial1.readBytes(readUART_BUF, read_length);
      readUART_BUF[read_length] = '\0';
      Serial1.write(readUART_BUF, read_length + 1);
      sd.add_str(readUART_BUF);
    }
  }
}

void flashSD(void* pvParameters) {
  while (true) {
    sd.flash();
  }
}

//*****************************************************************
// Task will periodicallt print out useful information about the tasks running
// Is a useful tool to help figure out stack sizes being used
//*****************************************************************
void taskMonitor(void* pvParameters) {
  int measurement;
  while (true) {
    SerialUSB.println("");
    SerialUSB.println("******************************");
    SerialUSB.println("[Stacks Free Bytes Remaining] ");

    measurement = uxTaskGetStackHighWaterMark(Handle_readUART);
    SerialUSB.print("Thread readUART: ");
    SerialUSB.println(measurement);

    SerialUSB.println("******************************");

    delay(1000);
  }
}
