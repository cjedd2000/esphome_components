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
  bool active;
} MotionSensor;

class HtzsafeOwlAlarm : public Component, public uart::UARTDevice {
 public:
  static const uint8_t MAX_SENSORS = 32;
  static const uint32_t DEFAULT_MOTION_TIMEOUT_MS = 10000;

  HtzsafeOwlAlarm();

  void setup() override;
  void loop() override;
  void dump_config() override;

  void set_last_id_sensor(sensor::Sensor *sensor) { this->LastSensorId = sensor; }
  bool add_motion_sensor(binary_sensor::BinarySensor *sensor, uint16_t id);
  bool add_motion_sensor_timeout(binary_sensor::BinarySensor *sensor, uint16_t id, uint32_t timeout);

 protected:
  bool activate_sensor(uint16_t id);
  void manage_sensors();

  sensor::Sensor *LastSensorId{nullptr};

  uint8_t SensorCount{0};
  MotionSensor MotionSensors[MAX_SENSORS];
};

}  // namespace htzsafe_owl_alarm
}  // namespace esphome
