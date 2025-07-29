#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/binary_sensor/binary_sensor.h"

#include <esp_timer.h>

namespace esphome {
namespace htzsafe_owl_alarm {

typedef struct motionSensor {
  binary_sensor::BinarySensor *sensor;
  uint16_t id;
  uint32_t timeActivated;
  uint32_t timeoutMs;
  sensor::Sensor *dataSensor;
  bool active;
} MotionSensor;

typedef struct SensorData {
  int32_t sensorId;

  uint8_t data3;
  uint8_t data2;
  uint8_t data1;
  uint8_t data0;
} SensorData;

typedef enum state { Idle, Start2, Start1, Start0, IdHigh, IdLow, Data3, Data2, Data1, Data0 } State;



class HtzsafeOwlAlarm : public Component, public uart::UARTDevice {
 public:
  static const uint8_t MAX_SENSORS = 32;
  static const uint32_t DEFAULT_MOTION_TIMEOUT_MS = 10000;
  /*
   * Headder Bytes
   */
  const uint8_t START_BYTE2 = 235;
  const uint8_t START_BYTE1 = 175;
  const uint8_t START_BYTE0 = 5;

  HtzsafeOwlAlarm();

  void setup() override;
  void loop() override;
  void dump_config() override;

  void set_last_id_sensor(sensor::Sensor *sensor) { this->LastSensorId = sensor; }
  bool add_motion_sensor(binary_sensor::BinarySensor *sensor, uint16_t id);
  bool add_motion_sensor_timeout(binary_sensor::BinarySensor *sensor, uint16_t id, uint32_t timeout);
  bool add_data_sensor(sensor::Sensor *sensor, uint16_t id);

 protected:
  bool activate_sensor(uint16_t id, uint32_t data);
  void manage_sensors();
  int32_t sensor_state_machine(uint8_t serialByte, SensorData *data);
  uint32_t get_time() { return esp_timer_get_time() / 1000; }

  sensor::Sensor *LastSensorId{nullptr};

  uint8_t SensorCount{0};
  MotionSensor MotionSensors[MAX_SENSORS];

  State SensorState{ Idle };
  uint16_t CurrentSensorId{ 0 };

  uint8_t SensData3{ 0 };
  uint8_t SensData2{ 0 };
  uint8_t SensData1{ 0 };
  uint8_t SensData0{ 0 };
};

}  // namespace htzsafe_owl_alarm
}  // namespace esphome
