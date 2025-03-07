#include <Arduino.h>
#include "Adafruit_VL53L0X.h"

const unsigned long SENSOR_INTERVAL_TIME = 50; // milliseconds

// Time of Flight sensor
Adafruit_VL53L0X tofSensor = Adafruit_VL53L0X();
unsigned long lastSensorReadTime = 0;
bool bSensorOk = true;

void readDistance(VL53L0X_RangingMeasurementData_t *measure);

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.begin(115200);
  // wait until serial port opens for native USB devices
  while (!Serial)
    ;

  if (tofSensor.begin())
  {
    tofSensor.setMeasurementTimingBudgetMicroSeconds(50000); // 50ms (20 updates per second)
    tofSensor.startRangeContinuous();
  }
  else
  {
    Serial.println("Failed to boot VL53L0X");
    bSensorOk = false;
  }

  digitalWrite(LED_BUILTIN, LOW);
}

void loop()
{
  // Perform distance measurements
  unsigned long currentTime = millis();
  if (currentTime - lastSensorReadTime >= SENSOR_INTERVAL_TIME)
  {
    VL53L0X_RangingMeasurementData_t measure;
    readDistance(&measure);

    // Only send updates when ready
    if (measure.RangeStatus == 0)
    {
      Serial.printf("{%d}\n", measure.RangeMilliMeter);
      lastSensorReadTime = currentTime;
    }
  }
}

void readDistance(VL53L0X_RangingMeasurementData_t *measure)
{
  if (bSensorOk)
  {
    tofSensor.getRangingMeasurement(measure, false);
  }
  else
  {
    measure->RangeMilliMeter = 500 + random(1, 10) - 5;
    measure->RangeStatus = 0;
  }
}