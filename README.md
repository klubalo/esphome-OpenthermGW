# OpenthermGW for ESPHome / Home assistant
 
(as External component, by Reproduktor, klubalo)

## Welcome!

I have two main requirements to solve:
1. I want to monitor what my thermostat & boiler are up to
2. I want to be able to control DHW temperature and control boiler heat period

It's derived from [Reproduktor/esphome-OpenthermGW](https://github.com/Reproduktor/esphome-OpenthermGW) as I found his implementation simple and easy to expand.

I needed **value_on_request** to work as my boiler doesn't accept T_Set and room temperature from thermostat - Boiler replies with **UNKNOWN-DATAID** to those write requests.

**value_on_request** also works for override values so it can be used for overriding requests from thermostat for controlling boiler (this was already implemented by Reproduktor) and also for overriding responses from boiler. This functionality can be used for


## Acknowledgement
Ihor Myealnik's [Opentherm library](https://github.com/ihormelnyk/opentherm_library) is used by this component.

## Configuration

### Use the component

The gateway is an ESPHome external component. To use it, you only need to include it in your configuration file:

```yaml
external_components:
  - source: github://klubalo/esphome-openthermgw@main
    components: [ openthermgw ]
```

### Hardware configuration
You need to configure the pins, on which the Opentherm gateway is connected. Please note - `master` is the thermostat end, `slave` is the boiler end.
Here is example for "sandwitch" of d1_mini between [TheHogNL master shield](https://www.tindie.com/products/thehognl/opentherm-master-shield-for-wemoslolin/) and [TheHogNL slave shield](https://www.tindie.com/products/thehognl/opentherm-slave-shield-for-wemoslolin/)

```yaml
openthermgw:
  master_in_pin: 4
  master_out_pin: 5
  slave_in_pin: 12
  slave_out_pin: 14
```

### Sensors - preface
I was trying to achieve such a configurability, that the component code does not need to be changed to just observe another message, or find out the details about the request and response messages. As a result, you are able to expose any message as a sensor, using the type decoding you specify. The same message ID can be used in multiple sensors, thus you can expose a sensor with different encoding (e.g., HB and LB as two different sensors). Binary sensors are handled separately, as they create a different kind of sensor - binary sensor.

### Adding numeric sensors

For the numeric sensors, you can create a list like this:

```yaml
  acme_opentherm_sensor_list:
    - name: "Control setpoint"
      device_class: "temperature"
      accuracy_decimals: 1
      unit_of_measurement: "°C"
      message_id: 1
      value_on_request: false
      value_type: 2
    - name: "Control setpoint 2"
      device_class: "temperature"
      accuracy_decimals: 1
      unit_of_measurement: "°C"
      message_id: 8
      value_on_request: false
      value_type: 2
    - name: "Room setpoint"
      device_class: "temperature"
      accuracy_decimals: 1
      unit_of_measurement: "°C"
      message_id: 16
      value_on_request: true
      value_type: 2
    - name: "Relative modulation level"
      device_class: "signal_strength"
      accuracy_decimals: 0
      unit_of_measurement: "%"
      message_id: 17
      value_on_request: false
      value_type: 2
```

#### Configuration variables

Sensor variables are inherited from ESPHome [Sensor component](https://esphome.io/components/sensor/index.html), plus:

- **message_id** (*Required*, positive int): Opentherm Message ID to capture in the sensor
- **value_type** (*Optional*, positive int range 0-7, default 0): Type of the value to retrieve from the Opentherm message. The types supported are:
  - **0** *(u16)* unsigned 16 bit integer
  - **1** *(s16)* signed 16 bit integer
  - **2** *(f16)* 16 bit float
  - **3** *(u8LB)* unsigned 8 bit integer in the lower byte of the message data
  - **4** *(u8HB)* unsigned 8 bit integer in the higher byte of the message data
  - **5** *(s8LB)* signed 8 bit integer in the lower byte of the message data
  - **6** *(s8HB)* signed 8 bit integer in the higher byte of the message data
  - **7** Request or response code of the Opentherm message. The value is directly read from the message. Possible values are:
    - Master-To-Slave (request)
      - 0 READ-DATA
      - 1 WRITE-DATA
      - 2 INVALID-DATA
    - Slave-To-Master (response)
      - 4 READ-ACK
      - 5 WRITE-ACK
      - 6 DATA-INVALID
      - 7 UNKNOWN-DATAID
- **value_on_request** (*Optional*, boolean, default `False`): If `false`, the value is read from the slave (boiler) response message (before response override). If `true`, the value is read from the master (thermostat) request message (before request override).

### Adding binary sensors

Binary sensors are added like this:

```yaml
  acme_opentherm_binary_sensors:
    - name: "Boiler fault"
      message_id: 0
      value_on_request: false
      bitindex: 1
    - name: "Boiler CH mode"
      message_id: 0
      value_on_request: false
      bitindex: 2
    - name: "Boiler DHW mode"
      message_id: 0
      value_on_request: false
      bitindex: 3
    - name: "Boiler flame status"
      message_id: 0
      value_on_request: false
      bitindex: 4
```

#### Configuration variables

Sensor variables are inherited from ESPHome [Binary sensor component](https://esphome.io/components/binary_sensor/), plus:

- **message_id** (*Required*, positive int): Opentherm Message ID to capture in the sensor
- **bitindex** (*Required*, positive int range 1-16): The bitindex from the right (lsb) of the message data.
- **value_on_request** (*Optional*, boolean, default `False`): If `false`, the value is read from the slave (boiler) response message (after response override). If `true`, the value is read from the master (thermostat) request message (before request override).

### Overriding binary sensors

You can add an override switch like this:

```yaml
  acme_opentherm_override_binary_switches:
    - name: "Enable override DHW command"
      message_id: 0
      value_on_request: true # Modify value from Thermostat to boiler
      bitindex: 10
      acme_opentherm_override_binary_value:
        name: "Override DHW control command"
```

This example creates two switches: `Enable override DHW command` is the switch which controls, if the value in the given message is overriden, and the `Override DHW control command` switch specifies the value to override the data with.

#### Configuration variables

For every message yyou wish to override, configure an independent switch to control, if the overriding is on or off. Switch variables are inherited from ESPHome [Switch component](https://esphome.io/components/switch/), plus:

- **message_id** (*Required*, positive int): Opentherm Message ID to capture in the sensor
- **bitindex** (*Required*, positive int range 1-16): The bitindex from the right (lsb) of the message data.
- **value_on_request** (*Optional*, boolean, default `True`): If `false`, the value is overriden in the slave (boiler) response message (before sensor value is reported). If `true`, the value is overriden in the master (thermostat) request message (after sensor value is reported).
- **acme_opentherm_override_binary_value** (*Required*, Switch): Secondary switch to control the state, to which the overriding should happen.

### Overriding numeric sensors

You can add an override numeric value like this:

```yaml
acme_opentherm_override_numeric_switches:
    - name: "Enable override DHW temperature"
      message_id: 56
      value_on_request: true # Modify value from Thermostat to boiler
      value_type: 2
      id: override_dhw_switch
      acme_opentherm_override_numeric_value:
        name: "Override DHW temperature"
        device_class: "Temperature"
        mode: "Slider"
        min_value: 30
        max_value: 60
        initial_value: 50
        step: 1
        id: override_dhw_temperature
```

#### Configuration variables

- **message_id** (*Required*, positive int): Opentherm Message ID to capture in the sensor
- **value_type** (*Optional*, positive int range 0-7, default 0): Type of the value to retrieve from the Opentherm message. The types supported are:
  - **0** *(u16)* unsigned 16 bit integer
  - **1** *(s16)* signed 16 bit integer
  - **2** *(f16)* 16 bit float
  - **3** *(u8LB)* unsigned 8 bit integer in the lower byte of the message data
  - **4** *(u8HB)* unsigned 8 bit integer in the higher byte of the message data
  - **5** *(s8LB)* signed 8 bit integer in the lower byte of the message data
  - **6** *(s8HB)* signed 8 bit integer in the higher byte of the message data
  - **7** Request or response code of the Opentherm message. The value is directly read from the message. Possible values are:
    - Master-To-Slave (request)
      - 0 READ-DATA
      - 1 WRITE-DATA
      - 2 INVALID-DATA
    - Slave-To-Master (response)
      - 4 READ-ACK
      - 5 WRITE-ACK
      - 6 DATA-INVALID
      - 7 UNKNOWN-DATAID
- **value_on_request** (*Optional*, boolean, default `True`): If `false`, the value is overriden in the slave (boiler) response message (before sensor value is reported). If `true`, the value is overriden in the master (thermostat) request message (after sensor value is reported).
- **acme_opentherm_override_numeric_value** (*Required*, Switch): Secondary  control numeric input, to which the overriding should happen.

# Complete configuration
```yaml
external_components:
  - source: github://klubalo/esphome-openthermgw@main
    components: [ openthermgw ]
    refresh: 0s

openthermgw:
  master_in_pin: 4
  master_out_pin: 5
  slave_in_pin: 12
  slave_out_pin: 14
  acme_opentherm_sensor_list:
    - name: "Boiler water WSP"
      device_class: "temperature"
      accuracy_decimals: 1
      unit_of_measurement: "°C"
      message_id: 1
      value_on_request: false
      value_type: 2
    - name: "Boiler water WSP2 (none)"
      device_class: "temperature"
      accuracy_decimals: 1
      unit_of_measurement: "°C"
      message_id: 8
      value_on_request: false
      value_type: 2

    - name: "Boiler maximum flame modulation"
      device_class: "signal_strength"
      accuracy_decimals: 0
      unit_of_measurement: "%"
      message_id: 14
      value_on_request: true # Thermostat provides this value, boiler discards it
      value_type: 2
    - name: "Room setpoint temperature"
      device_class: "temperature"
      accuracy_decimals: 1
      unit_of_measurement: "°C"
      message_id: 16
      value_on_request: true # Thermostat provides this value, boiler discards it
      value_type: 2
    - name: "Boiler flame modulation"
      device_class: "signal_strength"
      accuracy_decimals: 0
      unit_of_measurement: "%"
      message_id: 17
      value_on_request: false
      value_type: 2
    - name: "Boiler water pressure (none)"
      device_class: "pressure"
      accuracy_decimals: 2
      unit_of_measurement: "bar"
      message_id: 18
      value_on_request: false
      value_type: 2

    - name: "Room setpoint temperature 2 (none)"
      device_class: "temperature"
      accuracy_decimals: 1
      unit_of_measurement: "°C"
      message_id: 23
      value_on_request: false
      value_type: 2
    - name: "Room temperature"
      device_class: "temperature"
      accuracy_decimals: 1
      unit_of_measurement: "°C"
      message_id: 24
      value_on_request: true # Thermostat provides this value, boiler discards it
      value_type: 2
    - name: "Boiler water output temperature"
      device_class: "temperature"
      accuracy_decimals: 1
      unit_of_measurement: "°C"
      message_id: 25
      value_on_request: false
      value_type: 2
    - name: "DHW temperature"
      device_class: "temperature"
      accuracy_decimals: 1
      unit_of_measurement: "°C"
      message_id: 26
      value_on_request: false
      value_type: 2
    - name: "Outside temperature (boiler)"
      device_class: "temperature"
      accuracy_decimals: 1
      unit_of_measurement: "°C"
      message_id: 27
      value_on_request: false
      value_type: 2
    - name: "Boiler water input temperature"
      device_class: "temperature"
      accuracy_decimals: 1
      unit_of_measurement: "°C"
      message_id: 28
      value_on_request: false
      value_type: 2
    - name: "Boiler flow temperature 2 (none)"
      device_class: "temperature"
      accuracy_decimals: 1
      unit_of_measurement: "°C"
      message_id: 31
      value_on_request: false
      value_type: 2
    - name: "Boiler exhaust temperature (none)"
      device_class: "temperature"
      accuracy_decimals: 1
      unit_of_measurement: "°C"
      message_id: 33
      value_on_request: false
      value_type: 1

    - name: "Minimum temperature DHW setpoint"
      device_class: "temperature"
      accuracy_decimals: 0
      unit_of_measurement: "°C"
      message_id: 48
      value_on_request: false
      value_type: 5
    - name: "Maximum temperature DHW setpoint"
      device_class: "temperature"
      accuracy_decimals: 0
      unit_of_measurement: "°C"
      message_id: 48
      value_on_request: false
      value_type: 6

    - name: "Setpoint temperature DHW"
      device_class: "temperature"
      accuracy_decimals: 1
      unit_of_measurement: "°C"
      message_id: 56
      value_on_request: false
      value_type: 2

    - name: "Setpoint boiler maximum water output temperature"
      device_class: "temperature"
      accuracy_decimals: 1
      unit_of_measurement: "°C"
      message_id: 57
      value_on_request: false
      value_type: 2

    - name: "Thermostat MemberID code"
      accuracy_decimals: 0
      message_id: 2
      value_on_request: false
      value_type: 3
    - name: "Boiler MemberID code"
      accuracy_decimals: 0
      message_id: 3
      value_on_request: false
      value_type: 3
    - name: "Thermostrat product type"
      accuracy_decimals: 0
      message_id: 126
      value_on_request: false
      value_type: 3
    - name: "Thermostat product version"
      accuracy_decimals: 0
      message_id: 126
      value_on_request: false
      value_type: 4
    - name: "Boiler product type"
      accuracy_decimals: 0
      message_id: 127
      value_on_request: false
      value_type: 3
    - name: "Boiler product version"
      accuracy_decimals: 0
      message_id: 127
      value_on_request: false
      value_type: 4

  acme_opentherm_binary_sensors:
    - name: "Boiler alarm"
      message_id: 0
      value_on_request: false
      bitindex: 1
    - name: "Boiler heating state"
      message_id: 0
      value_on_request: false
      bitindex: 2
    - name: "DHW heating state"
      message_id: 0
      value_on_request: false
      bitindex: 3
    - name: "Flame"
      message_id: 0
      value_on_request: false
      bitindex: 4
    - name: "Boiler diagnostic indication"
      message_id: 0
      value_on_request: false
      bitindex: 7

    - name: "Enable boiler heating"
      message_id: 0
      value_on_request: false
      bitindex: 9
    - name: "Enable DHW heating"
      message_id: 0
      value_on_request: false
      bitindex: 10

    - name: "DHW present"
      message_id: 3
      value_on_request: false
      bitindex: 1
    - name: "Boiler control type"
      message_id: 3
      value_on_request: false
      bitindex: 2
    - name: "Cooling present"
      message_id: 3
      value_on_request: false
      bitindex: 3
    - name: "DHW control type"
      message_id: 3
      value_on_request: false
      bitindex: 4
    - name: "Boiler pump control type"
      message_id: 3
      value_on_request: false
      bitindex: 5
    - name: "Circuit 2 present"
      message_id: 3
      value_on_request: false
      bitindex: 6

  acme_opentherm_override_numeric_switches:
    - name: "Enable override DHW temperature"
      message_id: 56
      value_on_request: true # Modify value from Thermostat to boiler
      value_type: 2
      id: override_dhw_switch
      acme_opentherm_override_numeric_value:
        name: "Override DHW temperature"
        device_class: "Temperature"
        mode: "Slider"
        min_value: 30
        max_value: 60
        initial_value: 50
        step: 1
        id: override_dhw_temperature
    - name: "Enable override room setpoint temperature"
      message_id: 16
      value_on_request: false # Modify value from boiler to thermostat
      value_type: 2
      id: override_t_set_switch
      acme_opentherm_override_numeric_value:
        name: "Override room setpoint temperature"
        device_class: "Temperature"
        mode: "Slider"
        min_value: 5
        max_value: 35
        initial_value: 22
        step: 0.1
        id: override_t_set_temperature
  acme_opentherm_override_binary_switches:
    - name: "Enable override DHW command"
      value_on_request: true # Modify value from Thermostat to boiler
      bitindex: 10
      message_id: 0
      acme_opentherm_override_binary_value:
        name: "Override DHW control command"
    - name: "Enable override boiler heating command"
      value_on_request: true
      bitindex: 9
      message_id: 0
      acme_opentherm_override_binary_value:
        name: "Override boiler heating command"

one_wire:
  - platform: gpio
    pin: GPIO13

sensor:
  - platform: dallas_temp
    name: "Boiler room temperature"
    resolution: 12
    update_interval: 60s

```