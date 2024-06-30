import network
import time
from machine import Pin, PWM
from umqtt.simple import MQTTClient
import dht
import gc

#WiFi credentials
SSID = 'TP-Link_225B'
PASSWORD = 'vwm2itmi'

# MQTT broker details
MQTT_BROKER = '192.168.0.232'  #Raspberry Pi's IP address
MQTT_USER = 'marten'
MQTT_PASSWORD = 'kakaimun'
MQTT_DISCOVERY_PREFIX = 'homeassistant'
MQTT_CLIENT_ID = 'pico'
MQTT_COMMAND_TOPIC = 'homeassistant/switch/pico_motor/set'
MQTT_STATE_TOPIC = 'homeassistant/switch/pico_motor/state'
MQTT_TEMP_TOPIC = 'homeassistant/sensor/pico_temperature/state'
MQTT_HUM_TOPIC = 'homeassistant/sensor/pico_humidity/state'
MQTT_SPEED_COMMAND_TOPIC = 'homeassistant/number/pico_motor_speed_control/set'
MQTT_SPEED_STATE_TOPIC = 'homeassistant/sensor/pico_motor_speed/state'

#Rotary Encoder Inputs
CLK = 12
DT = 11
SW = 13

#Motor A connections
enA = 9
in1 = 8
in2 = 7

#Motor B connections
enB = 3
in3 = 5
in4 = 4

# utton input
buttonPin = 2

#DHT11 sensor
DHT_PIN = 15

counter = 0
max_counter = 10  #Maximum value for the speed incrementations 10 steps
speedToMotor = 0
currentStateCLK = 0
lastStateCLK = 0
currentDir = ""
lastButtonPress = 0
buttonState = 0
lastState = 0

#Encoder pins as inputs
clk = Pin(CLK, Pin.IN)
dt = Pin(DT, Pin.IN)
sw = Pin(SW, Pin.IN, Pin.PULL_UP)

#Motor control pins as outputs
ena = PWM(Pin(enA))
ena.freq(1000)
enb = PWM(Pin(enB))
enb.freq(1000)
in1 = Pin(in1, Pin.OUT)
in2 = Pin(in2, Pin.OUT)
in3 = Pin(in3, Pin.OUT)
in4 = Pin(in4, Pin.OUT)

#Button input
button = Pin(buttonPin, Pin.IN)

#DHT11 sensor setup
dht11 = dht.DHT11(Pin(DHT_PIN))

#Connect to WiFi
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect(SSID, PASSWORD)

#Wait for connect or fail
max_wait = 10
while max_wait > 0:
    if wlan.status() < 0 or wlan.status() >= 3:
        break
    max_wait -= 1
    print('waiting for connection...')
    time.sleep(1)

#Handle connection error
if wlan.status() != 3:
    raise RuntimeError('network connection failed')
else:
    print('connected')
    status = wlan.ifconfig()
    print('ip = ' + status[0])

#Motor control functions
def motorOn():
    global client
    ena.duty_u16(65535)  #PWM maximum possible values are 0 to 65535
    enb.duty_u16(65535)
    in1.value(1)
    in2.value(0)
    in3.value(1)
    in4.value(0)
    client.publish(MQTT_STATE_TOPIC, "ON", retain=True)
    print("Motor ON")

def motorOff():
    global client
    ena.duty_u16(0)
    enb.duty_u16(0)
    in1.value(0)
    in2.value(0)
    in3.value(0)
    in4.value(0)
    client.publish(MQTT_STATE_TOPIC, "OFF", retain=True)
    print("Motor OFF")

def setMotorSpeed(counter):
    global client
    speedToMotor = int(65535 * (counter / max_counter))
    ena.duty_u16(speedToMotor)
    enb.duty_u16(speedToMotor)
    client.publish(MQTT_SPEED_STATE_TOPIC, str(counter), retain=True)
    print(f"Motor speed set to: {counter} (Steps)")

def speedControl(counter):
    setMotorSpeed(counter)

#MQTT callback function
def sub_cb(topic, msg):
    global counter
    print((topic, msg))
    if topic == MQTT_COMMAND_TOPIC.encode():
        if msg == b"ON":
            motorOn()
        elif msg == b"OFF":
            motorOff()
    elif topic == MQTT_SPEED_COMMAND_TOPIC.encode():
        try:
            new_counter = int(msg)
            if 0 <= new_counter <= max_counter:
                counter = new_counter
                speedControl(counter)
                print(f"Speed set to: {counter} (MQTT)")
        except ValueError:
            print("Invalid speed value")

def connect_mqtt():
    global client
    client = MQTTClient(MQTT_CLIENT_ID, MQTT_BROKER, user=MQTT_USER, password=MQTT_PASSWORD)
    client.set_callback(sub_cb)
    while True:
        try:
            client.connect()
            print('Connected to %s MQTT broker' % MQTT_BROKER)
            client.subscribe(MQTT_COMMAND_TOPIC)
            client.subscribe(MQTT_SPEED_COMMAND_TOPIC)
            print('Subscribed to %s and %s topics' % (MQTT_COMMAND_TOPIC, MQTT_SPEED_COMMAND_TOPIC))
            return client
        except OSError as e:
            print('Failed to connect to MQTT broker. Reconnecting...')
            time.sleep(5)

client = connect_mqtt()

#Send discovery messages
client.publish('homeassistant/switch/pico_motor/config', '{"name": "Pico Motor", "command_topic": "' + MQTT_COMMAND_TOPIC + '", "state_topic": "' + MQTT_STATE_TOPIC + '", "payload_on": "ON", "payload_off": "OFF"}', retain=True)
client.publish('homeassistant/sensor/pico_temperature/config', '{"name": "Pico Temperature", "state_topic": "' + MQTT_TEMP_TOPIC + '", "unit_of_measurement": "°C"}', retain=True)
client.publish('homeassistant/sensor/pico_humidity/config', '{"name": "Pico Humidity", "state_topic": "' + MQTT_HUM_TOPIC + '", "unit_of_measurement": "%"}', retain=True)
client.publish('homeassistant/sensor/pico_motor_speed/config', '{"name": "Pico Motor Speed", "state_topic": "' + MQTT_SPEED_STATE_TOPIC + '", "unit_of_measurement": "Steps"}', retain=True)

def read_dht11():
    global client
    try:
        dht11.measure()
        temp = dht11.temperature()
        hum = dht11.humidity()
        print(f"Temperature: {temp}C, Humidity: {hum}%")
        client.publish(MQTT_TEMP_TOPIC, str(temp), retain=True)
        client.publish(MQTT_HUM_TOPIC, str(hum), retain=True)
    except Exception as e:
        print("Failed to read from DHT11 sensor:", e)

#Read the initial state of CLK
lastStateCLK = clk.value()

try:
    last_dht11_read_time = time.ticks_ms()
    last_speed_publish_time = time.ticks_ms()

    while True:
        currentStateCLK = clk.value()
        buttonState = button.value()

        if buttonState == 1 and lastState != buttonState:
            motorOn()
            lastState = 1
            print("Motor ON:")
        elif buttonState == 0 and lastState != buttonState:
            motorOff()
            lastState = 0
            print("Motor OFF:")

        if currentStateCLK != lastStateCLK and currentStateCLK == 1:
            if dt.value() != currentStateCLK and counter > 0:
                counter -= 1
                currentDir = "CCW"
            elif dt.value() == currentStateCLK and counter < max_counter:
                counter += 1
                currentDir = "CW"

            speedControl(counter)
            print(f"Direction: {currentDir} | Counter: {counter} | Button State: {buttonState}")

        lastStateCLK = currentStateCLK

        btnState = sw.value()
        if btnState == 0:
            if time.ticks_ms() - lastButtonPress > 50:
                print("Button pressed!")
            lastButtonPress = time.ticks_ms()

        #Read DHT11 sensor every 60 seconds
        if time.ticks_diff(time.ticks_ms(), last_dht11_read_time) > 60000:
            read_dht11()
            last_dht11_read_time = time.ticks_ms()

        try:
            client.check_msg()  #Check for new MQTT messages
        except OSError as e:
            print('MQTT connection error. Reconnecting...')
            client = connect_mqtt()

        #Publish motor speed every 60 seconds
        if time.ticks_diff(time.ticks_ms(), last_speed_publish_time) > 60000:
            client.publish(MQTT_SPEED_STATE_TOPIC, str(counter), retain=True)
            last_speed_publish_time = time.ticks_ms()

        #Run garbage collector
        gc.collect()
        time.sleep(0.001)

finally:
    client.disconnect()
    wlan.disconnect()
