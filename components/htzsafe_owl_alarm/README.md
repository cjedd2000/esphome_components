```yaml
# example configuration:

esphome:
  name: htzsafe-owl-alarm-dev

htzsafe_owl_alarm:
  id: OwlAlarm
  uart_id: OwlUart
  last_id:                   # Shows last received ID, used to ID new sensors
    name: Last Sensor Id

  # Define Motions Sensors
  motion_sensors:
    - name: Driveway
      sensor_id: 62276       # Sesnor ID as picked up by component

    - name: Backyard
      sensor_id: 24436
      motion_timeout: 5 # Timeout in seconds for new motion event

uart:
  id: OwlUart
  tx_pin: 17
  rx_pin: 16
  baud_rate: 9600
  rx_buffer_size: 2048

```
