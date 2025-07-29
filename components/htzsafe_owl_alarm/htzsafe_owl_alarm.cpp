#include "esphome/core/log.h"
#include "htzsafe_owl_alarm.h"

#include <esp_timer.h>

namespace esphome {
namespace htzsafe_owl_alarm {

static const char *TAG = "htzsafe_owl_alarm.component";

int32_t HtzsafeOwlAlarm::sensor_state_machine(uint8_t serialByte, SensorData *data) {
  // Run State Machine
  switch (SensorState) {
    case Idle:
      if (serialByte == START_BYTE2) {
        SensorState = Start2;
      }
      break;

    case Start2:
      if (serialByte == START_BYTE1) {
        SensorState = Start1;
      } else {
        SensorState = Idle;
      }
      break;

    case Start1:
      if (serialByte == START_BYTE0) {
        SensorState = Start0;
      } else {
        SensorState = Idle;
      }
      break;

    case Start0:
      CurrentSensorId = serialByte << 8;
      SensorState = IdHigh;
      break;

    case IdHigh:
      CurrentSensorId = CurrentSensorId | (0xF & serialByte);
      SensorState = IdLow;
      break;

    case IdLow:
      SensData3 = serialByte;
      SensorState = Data3;
      break;

    case Data3:
      SensData2 = serialByte;
      SensorState = Data2;
      break;

    case Data2:
      SensData1 = serialByte;
      SensorState = Data1;
      break;

    case Data1:
      SensData0 = serialByte;
      SensorState = Data0;
      break;

    case Data0:
      SensorState = Idle;
      break;

    default:
      SensorState = Idle;
      break;
  }

  int32_t retValue;

  if (SensorState == Idle) {
    retValue = CurrentSensorId;
    CurrentSensorId = 0;

    if (data != nullptr) {
      data->data3 = SensData3;
      data->data2 = SensData2;
      data->data1 = SensData1;
      data->data0 = SensData0;
    }
  } else {
    retValue = -1;
  }

  return retValue;
}

htzsafe_owl_alarm::HtzsafeOwlAlarm::HtzsafeOwlAlarm() {
  // Init sensors
  for (uint8_t i = 0; i < MAX_SENSORS; i++) {
    MotionSensors[i].sensor = nullptr;
    MotionSensors[i].dataSensor = nullptr;
    MotionSensors[i].id = 0;
    MotionSensors[i].active = false;
  }
}

void HtzsafeOwlAlarm::setup() {
  for (uint8_t i = 0; i < SensorCount; i++) {
    MotionSensors[i].sensor->publish_initial_state(false);
  }
  ESP_LOGI(TAG, "Setup Complete");
}

void HtzsafeOwlAlarm::dump_config() {
  ESP_LOGCONFIG(TAG, "HTZSAFE Owl Sensor:");
  ESP_LOGCONFIG(TAG, "Total Sensors: %d:", SensorCount);
  for (uint8_t i = 0; i < SensorCount; i++) {
    LOG_BINARY_SENSOR("", "Motion", MotionSensors[i].sensor);
    ESP_LOGCONFIG(TAG, "  ESP ID: %d", MotionSensors[i].id);
    ESP_LOGCONFIG(TAG, "  Timeout ms: %d", MotionSensors[i].timeoutMs);
  }
  LOG_SENSOR("", "Last ID", LastSensorId);
}

void HtzsafeOwlAlarm::loop() {
  while (this->available()) {
    uint8_t serialByte;
    SensorData data;

    this->read_byte(&serialByte);

    int32_t sensorId = sensor_state_machine(serialByte, &data);

    if (sensorId > 0) {
      // char writeBuff[64];
      // uint8_t wrLen = snprintf(writeBuff, 63, "Sensor Det: %d, %d %d %d %d\n\r", sensorId, data.data3, data.data2,
      // data.data1, data.data0);
      uint8_t nib3 = (sensorId >> 12) & 0xF;
      uint8_t nib2 = (sensorId >> 8) & 0xF;
      uint8_t nib1 = (sensorId >> 4) & 0xF;
      uint8_t nib0 = sensorId & 0xF;

      // Log raw data
      ESP_LOGI(TAG, "Sensor Det: %d, %d %d %d %d : %d %d %d %d", sensorId, data.data3, data.data2, data.data1,
               data.data0, nib3, nib2, nib1, nib0);

      // Get data into different formats for logging
      uint32_t bigEnd32 = (data.data3 << 24) | (data.data2 << 16) | (data.data1 << 8) | (data.data0);
      uint32_t littleEnd32 = (data.data0 << 24) | (data.data1 << 16) | (data.data2 << 8) | (data.data3);

      uint16_t bigEnd16High = (data.data3 << 8) | (data.data2);
      uint16_t bigEnd16Low = (data.data1 << 8) | (data.data0);

      uint16_t littleEnd16High = (data.data2 << 8) | (data.data3);
      uint16_t littleEnd16Low = (data.data0 << 8) | (data.data1);

      // Log Data Out
      ESP_LOGI(TAG, "32 Big: %d", bigEnd32);
      ESP_LOGI(TAG, "32 Little: %d", littleEnd32);

      ESP_LOGI(TAG, "16 Big High: %d", bigEnd16High);
      ESP_LOGI(TAG, "16 Big Low: %d", bigEnd16Low);

      ESP_LOGI(TAG, "16 Little High: %d", littleEnd16High);
      ESP_LOGI(TAG, "16 Little Low: %d", littleEnd16Low);

      // Log Unknown Sensor if not found during activation
      if (!activate_sensor(sensorId, bigEnd32)) {
        ESP_LOGW(TAG, "Unknown");
      }

      if (LastSensorId != nullptr) {
        LastSensorId->publish_state(sensorId);
      }
    }
  }

  manage_sensors();
}

bool HtzsafeOwlAlarm::activate_sensor(uint16_t id, uint32_t data) {
  for (uint8_t i; i < SensorCount; i++) {
    if (MotionSensors[i].id == id) {
      MotionSensors[i].active = true;
      MotionSensors[i].timeActivated = get_time();
      MotionSensors[i].sensor->publish_state(true);

      if(MotionSensors[i].dataSensor != nullptr) {
        MotionSensors[i].dataSensor->publish_state(data);
      }

      return true;
    }
  }

  // Sensor not defined
  return false;
}

void htzsafe_owl_alarm::HtzsafeOwlAlarm::manage_sensors() {
  // Iterate through all sensors
  for (uint8_t i; i < SensorCount; i++) {
    // Check if need to deactivate sensor after timeout
    if (MotionSensors[i].active) {
      if (get_time() - MotionSensors[i].timeActivated > MotionSensors[i].timeoutMs) {
        MotionSensors[i].active = false;
        MotionSensors[i].sensor->publish_state(false);
      }
    }
  }
}

bool HtzsafeOwlAlarm::add_motion_sensor(binary_sensor::BinarySensor *sensor, uint16_t id) {
  return add_motion_sensor_timeout(sensor, id, DEFAULT_MOTION_TIMEOUT_MS);
}

bool htzsafe_owl_alarm::HtzsafeOwlAlarm::add_motion_sensor_timeout(binary_sensor::BinarySensor *sensor,
                                                                            uint16_t id, uint32_t timeout_ms) {
  if (SensorCount < MAX_SENSORS) {
    MotionSensors[SensorCount].sensor = sensor;
    MotionSensors[SensorCount].id = id;
    MotionSensors[SensorCount].timeoutMs = timeout_ms;

    SensorCount++;

    ESP_LOGI(TAG, "Sensor Added, id: %d, total sensors: %d", id, SensorCount);

    return true;
  }

  ESP_LOGE(TAG, "To Many Sensors Added!!!");
  return false;
}

bool htzsafe_owl_alarm::HtzsafeOwlAlarm::add_data_sensor(sensor::Sensor * sensor, uint16_t id)
{
  for(uint8_t i = 0; i < SensorCount; i++) {
    if(MotionSensors[i].id == id) {
      MotionSensors[i].dataSensor = sensor;
    }
  }
  return false;
}

}  // namespace htzsafe_owl_alarm
}  // namespace esphome
