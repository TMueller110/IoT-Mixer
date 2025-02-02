## Smart Mixer
Author: Matthias Marten mm225fv

### Overview
This project demonstrates the creation of a small IoT mixer application using a Raspberry Pi Pico W controlled via Home Assistant, with local control options through a push button and a potentiometer. The system is designed to mix any content, but a specific Home Assistant interface is tailored to ensure the correct mixing time when sugar, water, and invertase enzyme are combined.

Invertase is an enzyme that catalyzes the hydrolysis of sucrose (table sugar) into glucose and fructose. When sugar, water, and invertase are mixed, the enzyme converts the sugar into fructose and glucose. The time required for this conversion depends on the proportions of the ingredients and the temperature, as the activity of invertase is temperature-dependent. The author has this as smaller hobby, making syrup and artificial honey 1 or 2 every year.

The entire process is controlled and monitored through a Home Assistant dashboard, enabling precise control over the mixing parameters and real-time status updates. The dashboard provides real-time updates on mixing status, speed, temperature, and the remaining mixing time, ensuring efficient and accurate mixing.

* Example of a commercial mixer (not the one created in the project).
  
![UntitledTe](https://github.com/TMueller110/IoT-Mixer/assets/155649436/2c25f52b-a529-4e8a-bb42-fcef176aaa75)


#### Tech stack used:
* Raspberry Pi Pico W: Microcontroller used for controlling the mixing system and interfacing with sensors and actuators.
* MicroPython: Programming language used to write the firmware for the Raspberry Pi Pico W.
* Home Assistant: Open-source platform for managing and automating smart home devices, used to create the control dashboard and manage automation.
* MQTT: Messaging protocol used for communication between Home Assistant and the Raspberry Pi Pico W.
* DHT11 Sensor: Sensor used to measure temperature and humidity.
* L298N Motor Driver: Motor driver module used to control the motors.
* YAML: Data serialization format used for Home Assistant configuration files.
* ESPHome: Used for integrating the Raspberry Pi Pico W with Home Assistant via MQTT.
* Lovelace UI: Home Assistant's interface for creating custom dashboards.

#### Estimated time to reproduce this project
5h

## Objective 

I chose to build this automated mixing system to streamline the process of making syrup and artificial honey. The device serves the purpose of precisely mixing sugar, water, and invertase enzyme, adjusting for optimal reaction times based on real-time temperature data. By automating the mixing process and monitoring it through Home Assistant, I aim to achieve better estimate of the mixing time needed and a easier mixing process. The data collected will provide insights into the optimal conditions for the enzymatic reaction, helping to refine the process and calculation for the mixing time. 

Additionally, this project may inspire others to explore home automation solutions for various tasks, showcasing the potential of integrating smart technology into everyday activities.

**Note that a limitation of the project is that the mixer itself (the mechanical part) still has to be created and mounted on the motor axis.**

![bild](https://github.com/TMueller110/IoT-Mixer/assets/155649436/3563b7c3-2bc5-4e95-9c6f-34d0a66dbcb1)

### Materials

#### List of Materials

| Item                | Description                                                                 | Specifications                                      | Where Purchased | Cost       |
|---------------------|-----------------------------------------------------------------------------|-----------------------------------------------------|-----------------|------------|
| Raspberry Pi Pico W | Microcontroller used for controlling the system                             | Dual-core ARM Cortex-M0+ processor, Wi-Fi enabled   | Elektrokit      | 428SEK     |
| DHT11 Sensor        | Sensor used to measure temperature and humidity                             | Temperature range: 0-50°C, Humidity range: 20-90%   | Elektrokit      | -          |
| Resistors           | Pull down resistor for pushbutton                                           | 10kOhm                                              | Elektrokit      | -          |
| L298N Motor Driver  | Motor driver module used to control the motors                              | Dual H-Bridge motor driver, 5-35V, 2A per channel   | Elektrokit      | 79SEK      |
| Push Button         | Used for local control of the motor                                         | Momentary push button                               | Elektrokit      | 10SEK      |
| Potentiometer       | Used for local speed control of the motor                                   | 10k Ohm                                             | Elektrokit      | 10SEK      |
| 24VDC Power Supply  | Power supply for the motor driver                                           | 24V DC, 5A                                          | Local Store     | 300SEK     |
| Terminals           | Connectors for wiring                                                       | Screw terminals                                     | Local Store     | 30SEK      |
| Wires               | Used to connect components                                                  | Various lengths and gauges                          | Local Store     | 50SEK      |
| 24VDC Motor         | 24 VDC Motor for steering the mixer. (recycled from broken robot lawn mower)| 33rpm Mn 3.0m                                       | For free        | 0SEK       |
| Raspberry Pi 5      | Small single board computer for running Home Assistant                      | Arm Cortex A76, 4GB ram, Wi-Fi enabled              | Local Store     | 820SEK     |
| Fuse                | 230VAC Fuse                                                                 | Siemens C6 6A                                       | For free        | 0SEK       |


## Computer setup

For this project, Visual Studio Code (VS Code) was chosen as the IDE. 

Following installations where needed.

* Visual studios (download from official website)
* Python (download from official website)
* Node.js (downlaod from official website)
* Pymakr extension (download from extensions in VSCode)
* MicroPython on Rapsberry Pi Pico W (as explained in https://marketplace.visualstudio.com/items?itemName=paulober.pico-w-go)
* MQTT and File Editor in home assistant
* umqtt.simple library used (already in repository), it originally created by "dpgeorge" https://github.com/micropython/micropython-lib/tree/master/micropython/umqtt.simple

### Pymakr settings 

"Ctrl+Shift+P" and typing Pymakr > Global Settings to configure the Pymakr settings (for COM port setting mainly)

### Upload Code to Raspberry PI Pico 

First upload the project with "Ctrl+Shift+P" -> MicroPico: Upload Project To pico

Then run the main as seen in the figure below: 

![bild](https://github.com/TMueller110/IoT-Mixer/assets/155649436/8736ce84-b4ea-400c-8a4e-a5d5df9a7c28)
![bild](https://github.com/TMueller110/Smart_Mixer/assets/155649436/46421c3c-8bcd-4898-b7e9-22e3fe958328)

### Home Assistant

Following installations/configurations need to be done for home assistant.

* Install Home Assistant on RaspberryPi (guide found on official website).

* Download the MQTT broker (Mosquitto Broker Add-on). Configurations are done in the configurations.yaml file (please see this file in the repository for more information). 

* Configuration files need to be adjusted in Home Assistant. configuration.yaml and automation.yaml.
  
* Home assistant dashboard. Edit the dashboard with the correct enteties 

### Putting Everything Together

#### Wiring and Electronics Setup

Below is a  guide on how to wire everything.

#### Components

- **Fuse**
- **Raspberry Pi Pico W**
- **DHT11 Temperature and Humidity Sensor**
- **Rotary Encoder**
- **L298N Motor Driver**
- **Push Button**
- **Potentiometer**
- **24VDC Motor**
- **24VDC Power Supply**
- **Terminals**
- **Connecting Wires**

#### Wiring Diagram

![bild](https://github.com/TMueller110/IoT-Mixer/assets/155649436/2a69c40b-fa29-458f-b142-071ab16d6200)



#### Wiring Instructions

1. **Power Supply:**
   - Connect a stripped power cable to fuse and from fuse to the power supply. Ground to the busbar. 

2. **Power Supply:**
   - Connect the 24V DC power supply to the L298N motor driver.

3. **Raspberry Pi Pico W:**
   - **GND:** Connect GND to ground busbar. 
   - **Motor Driver Inputs:** Connect the Pico W GPIO pins to the L298N motor driver inputs (ENA, IN1, IN2, IN3, IN4, ENB).
     - `GPIO9` to `ENA`
     - `GPIO8` to `IN1`
     - `GPIO7` to `IN2`
     - `GPIO3` to `ENB`
     - `GPIO5` to `IN3`
     - `GPIO4` to `IN4`

4. **Rotary Encoder:**
   - Connect the rotary encoder to the Pico W GPIO pins.
     - `CLK` to `GPIO12`
     - `DT` to `GPIO11`
     - `SW` to `GPIO13`

5. **DHT11 Sensor:**
   - Connect the DHT11 sensor to the Pico W.
     - `VCC` to `3.3V`
     - `GND` to `GND`
     - `DATA` to `GPIO15`

6. **Push Button:**
   - Connect one terminal of the push button to `GPIO2` on the Pico W.
   - Connect the other terminal to the ground.

7. **Potentiometer:**
   - Connect the potentiometer to the Pico W
     - `DT` to `GPI11` 
     - `CLK` to `GPI12`
     - `SW` to `GPI13`

8. **Motor Connections:**
   - Connect the DC motor to the L298N motor driver outputs.

#### Resistors and Considerations

- **Pull-down Resistors:** Pull-down resistors used for the push button and rotary encoder to ensure they read low (0V) when not pressed or rotated.
- **24VDC and Pico W grounded:** This is to have a common ground. If not done this way a "floating" voltage would be the case (when measured from V+ to ground). 

### Code

This section highlights the core functionalities of the automated mixing system's code. The system uses MicroPython to control the Raspberry Pi Pico W, and Home Assistant for remote monitoring and control. Below are the essential code snippets and explanations.

#### 1. Setting Up Network and MQTT

The first step is to connect the Pico W to the Wi-Fi network and set up the MQTT client for communication with Home Assistant.

# MQTT broker details
```python 
MQTT_BROKER = '192.168.0.232'
MQTT_USER = 'XXX'
MQTT_PASSWORD = 'XXX'
MQTT_CLIENT_ID = 'pico'

def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(SSID, PASSWORD)
    while wlan.status() != 3:
        print('Waiting for connection...')
        time.sleep(1)
    print('Connected to WiFi:', wlan.ifconfig())

def connect_mqtt():
    client = MQTTClient(MQTT_CLIENT_ID, MQTT_BROKER, user=MQTT_USER, password=MQTT_PASSWORD)
    client.connect()
    return client

connect_wifi()
client = connect_mqtt()
```
#### 2. Controlling the Motor
The motor is controlled via the L298N motor driver. The following functions turn the motor on and off, and set the speed based on the rotary encoder or Home Assistant input.

# Motor control functions
```python 
def motorOn():
    ena.duty_u16(65535)
    enb.duty_u16(65535)
    in1.value(1)
    in2.value(0)
    in3.value(1)
    in4.value(0)
    client.publish('homeassistant/switch/pico_motor/state', "ON", retain=True)
    print("Motor ON")

def setMotorSpeed(counter):
    speedToMotor = int(65535 * (counter / 10))
    ena.duty_u16(speedToMotor)
    enb.duty_u16(speedToMotor)
    client.publish('homeassistant/sensor/pico_motor_speed/state', str(counter), retain=True)
    print(f"Motor speed set to: {counter} (Steps)")

```
#### 3. Handling MQTT Messages
```python
def sub_cb(topic, msg):
    global counter
    print((topic, msg))
    if topic == b'homeassistant/switch/pico_motor/set':
        if msg == b"ON":
            motorOn()
        elif msg == b"OFF":
            motorOff()
    elif topic == b'homeassistant/number/pico_motor_speed_control/set':
        try:
            counter = int(msg)
            if 0 <= counter <= 10:
                setMotorSpeed(counter)
                print(f"Speed set to: {counter} (MQTT)")
        except ValueError:
            print("Invalid speed value")

client.set_callback(sub_cb)
client.subscribe('homeassistant/switch/pico_motor/set')
client.subscribe('homeassistant/number/pico_motor_speed_control/set')
```
#### 3. Reading Sensor Data
The DHT11 sensor provides temperature and humidity readings, which are published to Home Assistant for monitoring.
```python
def read_dht11():
    try:
        dht11.measure()
        temp = dht11.temperature()
        hum = dht11.humidity()
        print(f"Temperature: {temp}C, Humidity: {hum}%")
        client.publish('homeassistant/sensor/pico_temperature/state', str(temp), retain=True)
        client.publish('homeassistant/sensor/pico_humidity/state', str(hum), retain=True)
    except Exception as e:
        print("Failed to read from DHT11 sensor:", e)
```
#### 5. Main Loop
The main loop handles the rotary encoder input, button press, sensor readings, and MQTT message checking. It ensures that the system operates continuously and responds to user inputs.
```python
try:
    last_dht11_read_time = time.ticks_ms()
    while True:
        currentStateCLK = clk.value()
        buttonState = button.value()
................
```

### Data Transmission Overview

#### Frequency of Data Transmission:

* Temperature and humidity data from the DHT11 sensor are sent to the MQTT broker every 60 seconds. There is no need to update this more frequent in this application. The temperature is measuring the room temperature where the mixing is conducted and thus the temperature will not flactuate rapidly.

* Motor speed and state updates are published to the MQTT broker whenever they change, ensuring real-time updates. The motor speed is set by the user either through the frontend or the local potentiometer. When the automated mixing is turned on the speed is set to the last speed that has been given (and can manually be changed during the automated mixing time). 
  
Wireless Protocols:

#### Wi-Fi: 
* The Raspberry Pi Pico W connects to a local Wi-Fi network to communicate with the MQTT broker hosted on a local server (Home Assistant).
  
Transport Protocols:

#### MQTT:
Some key factors why MQTT was choosen
* Lightweight messaging protocol great for efficient data transmission.
* MQTT is ideal for IoT applications due to low bandwidth usage and support for persistent connections.

### Presenting the Data

#### Home Assistant Dashboard
There are two Home Assistant dashboard. One for controlling the mixer directly and one for setting the parameters sugar, water and invertase. The home atuomation is then calculating the time and when the user hits the botton "mix" the mixer will start and run untill the time seen is finished. 

The data is not stored but is just showing the actual temperature. This can be easily added in home assistant by the built in recorder (that uses SQlite per default, can also be easily configured to use an external database). Just add the following to the configuration.yaml to add this functionality: 

```python recorder:
  purge_keep_days: 30  #Keeps data for 30 days
  include:
    entities:
      - sensor.pico_temperature
      - sensor.pico_humidity
```

* Direct Mixer Control
  
![bild](https://github.com/TMueller110/IoT-Mixer/assets/155649436/25b04470-3b8b-4eb4-aab3-31f6a5e3e5e4)


* Automatic Mixer Control
  
![bild](https://github.com/TMueller110/IoT-Mixer/assets/155649436/5f21832c-f3df-4a6d-91e5-7ddd01af4f44)


### Finalizing the Design
The final design of the automated mixing system using the Raspberry Pi Pico W and Home Assistant was a success. Below are the final results and reflections on the project, along with some pictures.

#### Final Results
* Real-Time Monitoring: The Home Assistant dashboard shows real-time updates on temperature, humidity, motor speed, and mixing status.
* Automated Control: Mixing time is calculated automatically based on input parameters and current temperature.
* Local Control: Local control is possible through a push button and potentiometer.
#### Project Reflections
* Successes: Seamless integration, effective automation logic, and a modular design for adaptability.
* Improvements: Enhanced error handling and better scalability for larger volumes or different processes.
#### Pictures
Below are some pictures of the final setup:
![bild](https://github.com/TMueller110/IoT-Mixer/assets/155649436/ed66182d-fdc7-49e7-a3f9-feed449bd033)
![DSC_0213](https://github.com/TMueller110/IoT-Mixer/assets/155649436/b3000f02-a2d1-4e07-90e1-8bba0bc75b71)
![DSC_0212](https://github.com/TMueller110/IoT-Mixer/assets/155649436/db278e51-c5e3-4a32-b658-622bc9da8eac)
![DSC_0211](https://github.com/TMueller110/IoT-Mixer/assets/155649436/2316b721-d2a3-4f86-a18e-7d45d8587ff1)
![DSC_0214](https://github.com/TMueller110/IoT-Mixer/assets/155649436/4d92d0b8-b82d-4d18-a110-99fd72cb48ed)


#### Video
To see video click the following link:
https://1drv.ms/v/s!AqpQvNS5aziNkQA0zGGipVsP77jH
