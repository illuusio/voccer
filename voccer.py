""" VOCCER is simple Python 3.x script which
    Tries to read from diffrent sensors
    and forward them to MQTT-server which """

# Copyright (c) 2019 Tuukka Pasanen
# SPDX-License-Identifier: MIT

# pylint: disable=E0401
import csv
import datetime
import io
import json
import time
import logging
import serial
import sys
import getopt
import threading
import paho.mqtt.client as mqtt
import RPi.GPIO as GPIO
import bme680
from smbus2 import SMBus
from sgp30 import Sgp30
from pms5003 import PMS5003


class BaseSensor:
    """ Base sensor class """
    def __init__(self, logger, mqttc, sensor_id):
        self.logger = logger
        self.mqttc = mqttc
        self.sensor_id = sensor_id

    def measure_hash(self, sensor_id, measure_value, measure_type):
        """Create measurement JSON"""
        # pylint: disable=R0201
        typeid = 100
        unitid = 100
        if measure_type == "voc":
            typeid = 105
            unitid = 105
        elif measure_type == "temperature":
            typeid = 100
            unitid = 100
        elif measure_type == "humidity":
            typeid = 101
            unitid = 101
        elif measure_type == "pressure":
            typeid = 102
            unitid = 102
        elif measure_type == "quality":
            typeid = 103
            unitid = 103
        elif measure_type == "gas":
            typeid = 104
            unitid = 104
        elif measure_type == "co2":
            typeid = 106
            unitid = 105
        elif measure_type == "PM1":
            typeid = 107
            unitid = 107
        elif measure_type == "PM2.5":
            typeid = 108
            unitid = 107
        elif measure_type == "PM10":
            typeid = 109
            unitid = 107
        elif measure_type == "atmospheric_PM1":
            typeid = 110
            unitid = 107
        elif measure_type == "atmospheric_PM2.5":
            typeid = 111
            unitid = 107

        return {
            'timestamp': round(time.time(), 2),
            'id': int(sensor_id),
            'value': measure_value,
            'type': measure_type,
            'typeid': typeid,
            'unitid': unitid
        }

    def enable_sensor(self):
        """Empty enable if we don't need it"""

    def disable_sensor(self):
        """Empty disalbe if we don't need it"""

    def step(self, queue):
        """Empty step just for to be sure"""


class ArduinoSensor(BaseSensor):
    """ Arduino CSS811 sensor class """
    def __init__(self, logger, mqttc, sensor_id, serial_device):
        super().__init__(logger, mqttc, sensor_id)

        # Open Serial port for reading
        self.serial = serial.Serial(serial_device, 9600, timeout=15)
        self.sio = io.TextIOWrapper(io.BufferedRWPair(self.serial,
                                                      self.serial))
        self.data = None
        self.sensor1_co2 = None
        self.sensor1_tvoc = None
        self.sensor1_temp = None
        self.sensor2_co2 = None
        self.sensor2_tvoc = None
        self.sensor2_temp = None

        try:
            threading.Thread(target=self._read_thread,
                             args=(self.serial, )).start()
        except ThreadError:
            print(
                "Arduino: Can't start reading thread. You can't get anything out of this"
            )

    def _read_thread(self, serial):
        """ Read from serial device """
        while True:
            try:
                self.data = self.sio.readline()
                if '#' in self.data or len(self.data) == 0:
                    continue
            except:
                print("Arduino: Can't read from serial port")

            csv_reader = csv.reader([self.data])

            for row in csv_reader:
                if (len(row) >= 2 and row[0].isdigit() and row[1].isdigit()):

                    if int(row[0]) == 1:
                        self.sensor1_co2 = int(row[1])
                        self.sensor1_tvoc = int(row[2])
                    else:
                        self.sensor2_co2 = int(row[1])
                        self.sensor2_tvoc = int(row[2])

                if (len(row) >= 4 and row[0].isdigit() and row[1].isdigit()):
                    if int(row[0]) == 1:
                        self.sensor1_temp = round(float(row[3]), 2)
                    else:
                        self.sensor2_temp = round(float(row[3]), 2)

    def step(self, queue):
        with threading.Lock():
            if self.data is not None:
                queue.append([
                    "co2",
                    self.measure_hash(self.sensor_id, self.sensor1_co2, "co2"),
                    False
                ])
                queue.append([
                    "voc",
                    self.measure_hash(self.sensor_id, self.sensor1_tvoc,
                                      "voc"), False
                ])
                queue.append([
                    "co2",
                    self.measure_hash((self.sensor_id + 1), self.sensor2_co2,
                                      "co2"), False
                ])
                queue.append([
                    "voc",
                    self.measure_hash((self.sensor_id + 1), self.sensor2_tvoc,
                                      "voc"), False
                ])
                self.logger.debug(
                    'Arduino: Sensor1 CO2: {0:d} ug/m3 (ppm), VOC: {1:d} ng/m3 (ppb)'
                    .format(self.sensor1_co2, self.sensor1_tvoc))
                self.logger.debug(
                    'Arduino: Sensor2 CO2: {0:d} ug/m3 (ppm), VOC: {1:d} ng/m3 (ppb)'
                    .format(self.sensor2_co2, self.sensor2_tvoc))


class PMS5003Sensor(BaseSensor):
    """ PMS5003 sensor class """
    def __init__(self, logger, mqttc, sensor_id, serial_device):
        super().__init__(logger, mqttc, sensor_id)

        # Configure the PMS5003 for Enviro+
        self.pms5003 = PMS5003(device=serial_device,
                               baudrate=9600,
                               pin_enable=22,
                               pin_reset=27)

        self.stopafter = 3 * 60
        self.enable = True
        self.data = None
        self.oldtime = None
        try:
            threading.Thread(target=self._read_thread,
                             args=(self.pms5003, )).start()
        except ThreadError:
            print(
                "PMS5003: Can't start reading thread. You can't get anything out of this"
            )

    def _read_thread(self, pms5003):
        """ Read because PMS5003 is serial outputting device """
        while True:
            if self.enable is True:
                self.oldtime = time.time()
                self.stopafter = 3 * 60
                self.enable = False
                try:
                    while (time.time() - self.oldtime) < self.stopafter:
                        self.data = pms5003.read()
                except:
                    self.enable = False
            time.sleep(1)

    def enable_sensor(self):
        with threading.Lock():
            self.enable = True
            self.pms5003.setup()

    def disable_sensor(self):
        with threading.Lock():
            self.enable = False
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.pms5003._pin_enable, GPIO.OUT, initial=GPIO.LOW)

    def step(self, queue):
        with threading.Lock():
            if self.data is not None:
                queue.append([
                    "airatmospheric_environment_PM1",
                    self.measure_hash(self.sensor_id,
                                      self.data.pm_ug_per_m3(1.0, True),
                                      "atmospheric_PM1"), False
                ])
                queue.append([
                    "airatmospheric_environment_PM2.5",
                    self.measure_hash(self.sensor_id,
                                      self.data.pm_ug_per_m3(2.5, True),
                                      "atmospheric_PM2.5"), False
                ])
                queue.append([
                    "PM1",
                    self.measure_hash(self.sensor_id,
                                      self.data.pm_ug_per_m3(1.0, False),
                                      "PM1"), False
                ])
                queue.append([
                    "PM2.5",
                    self.measure_hash(self.sensor_id,
                                      self.data.pm_ug_per_m3(2.5, False),
                                      "PM2.5"), False
                ])
                queue.append([
                    "PM10",
                    self.measure_hash(self.sensor_id,
                                      self.data.pm_ug_per_m3(10, False),
                                      "PM10"), False
                ])

                self.logger.debug(
                    'PMS5003: Airatmospheric env PM1: {0:d} ug/m3, airatmospheric env PM2.5: {1:d} ug/m3'
                    .format(self.data.pm_ug_per_m3(1.0, True),
                            self.data.pm_ug_per_m3(2.5, True)))
                self.logger.debug(
                    'PMS5003: PM1 {0:d}, PM2.5: {1:d} PM10 {2:d}'.format(
                        self.data.pm_ug_per_m3(1.0, False),
                        self.data.pm_ug_per_m3(2.5, False),
                        self.data.pm_ug_per_m3(10, False)))


class BME680Sensor(BaseSensor):
    """ BME680 sensor class """
    def __init__(self, logger, mqttc, sensor_id, addr, temp_offset):
        super().__init__(logger, mqttc, sensor_id)
        self.addr = addr
        self.temp_offset = temp_offset
        self.sensor = bme680.BME680(addr)
        self.config(temp_offset)
        self.gas_baseline = self.get_gas_baseline()

    def air_quality_score(self, sensor, gas_baseline, hum_baseline):
        """Calculate air quality score from BME680 or else"""
        # pylint: disable=R0201
        # This sets the balance between humidity and gas reading in the
        # calculation of air_quality_score (25:75, humidity:gas)
        hum_weighting = 0.25

        sensor.get_sensor_data()

        if sensor.get_sensor_data() and sensor.data.heat_stable:
            gas = sensor.data.gas_resistance
            gas_offset = gas_baseline - gas
            hum = sensor.data.humidity
            hum_offset = hum - hum_baseline

            # Calculate hum_score as the distance from the hum_baseline.
            if hum_offset > 0:
                hum_score = (100 - hum_baseline - hum_offset)
                hum_score /= (100 - hum_baseline)
                hum_score *= (hum_weighting * 100)
            else:
                hum_score = (hum_baseline + hum_offset)
                hum_score /= hum_baseline
                hum_score *= (hum_weighting * 100)

            # Calculate gas_score as the distance from the gas_baseline.
            if gas_offset > 0:
                gas_score = (gas / gas_baseline)
                gas_score *= (100 - (hum_weighting * 100))
            else:
                gas_score = 100 - (hum_weighting * 100)

            # Calculate air_quality_score.
            return hum_score + gas_score

        return 0

    def get_gas_baseline(self):
        """ Calculate Gas baseline """
        start_time = time.time()
        curr_time = time.time()
        # burn_in_time = (5 * 60)
        burn_in_time = (20)

        burn_in_data = []

        try:
            # Collect gas resistance burn-in values, then use the average
            # of the last 50 values to set the upper limit for calculating
            # gas_baseline.
            self.logger.debug(
                'BME680: Collecting gas resistance burn-in data for 5 mins')
            while curr_time - start_time < burn_in_time:
                left_time = int(burn_in_time - (curr_time - start_time))
                curr_time = time.time()
                if self.sensor.get_sensor_data(
                ) and self.sensor.data.heat_stable:
                    gas = self.sensor.data.gas_resistance
                    burn_in_data.append(gas)
                    self.logger.debug('BME680: Gas {0}: {1} Ohms'.format(
                        left_time, round(gas, 1)))
                    time.sleep(10)

            gas_baseline = sum(burn_in_data[-50:]) / 50.0
        except KeyboardInterrupt:
            pass
        return gas_baseline

    def config(self, temp_offset):
        """ Function to set confis for BOSCH BME680
            These oversampling settings can be tweaked to
            change the balance between accuracy and noise in
            the data """
        # pylint: disable=R0201
        # We don't need that speedy readings and we want
        # Nice and steady correct answers
        self.sensor.set_humidity_oversample(bme680.OS_8X)
        self.sensor.set_pressure_oversample(bme680.OS_8X)
        self.sensor.set_temperature_oversample(bme680.OS_8X)
        # Stedier and more correct values
        self.sensor.set_filter(bme680.FILTER_SIZE_7)
        self.sensor.set_gas_status(bme680.ENABLE_GAS_MEAS)
        self.sensor.set_temp_offset(temp_offset)

        #    self.logger.debug('\n\nInitial reading:')
        #    for name in dir(sensor.data):
        #      value = getattr(sensor.data, name)
        #
        #      if not name.startswith('_'):
        #          self.logger.debug('{}: {}'.format(name, value))

        self.sensor.set_gas_heater_temperature(320)
        self.sensor.set_gas_heater_duration(150)
        self.sensor.select_gas_heater_profile(0)

    #    self.logger.debug('Calibration data:')
    #    for name in dir(sensor.calibration_data):
    #
    #        if not name.startswith('_'):
    #            value = getattr(sensor.calibration_data, name)
    #
    #            if isinstance(value, int):
    #                self.logger.debug('{}: {}'.format(name, value))

    def step(self, queue):
        """ Main loop step for Bosch BME680 sensor """
        # pylint: disable=W0612,W0613

        # Set the humidity baseline to 40%, an optimal indoor humidity.
        hum_baseline = 40.0

        if self.sensor is None:
            return False

        if self.sensor.get_sensor_data():
            queue.append([
                "temperature",
                self.measure_hash(self.sensor_id,
                                  round(self.sensor.data.temperature, 2),
                                  "temperature"), False
            ])
            queue.append([
                "pressure",
                self.measure_hash(self.sensor_id,
                                  int(self.sensor.data.pressure), "pressure"),
                False
            ])
            queue.append([
                "humidity",
                self.measure_hash(self.sensor_id,
                                  int(self.sensor.data.humidity), "humidity"),
                False
            ])
            queue.append([
                "gas",
                self.measure_hash(self.sensor_id,
                                  round(self.sensor.data.gas_resistance, 2),
                                  "gas"), False
            ])

            air_quality_scr = self.air_quality_score(self.sensor,
                                                     self.gas_baseline,
                                                     hum_baseline)

            queue.append([
                "quality",
                self.measure_hash(self.sensor_id, round(air_quality_scr, 2),
                                  "quality"), False
            ])

            self.logger.debug(
                'BME680: Temperature: {0:.2f}C, Pressure: {1:.2f} HPa'.format(
                    self.sensor.data.temperature, self.sensor.data.pressure))
            self.logger.debug(
                'BME680: Humidity {0:.2f} %RH, Resistance: {1:.2f} Ohm Quality Indx {2:.2f}'
                .format(self.sensor.data.humidity,
                        self.sensor.data.gas_resistance, air_quality_scr))

        return True


class SGP30Sensor(BaseSensor):
    """ BME680 sensor class """
    def __init__(self, logger, mqttc, sensor_id):
        super().__init__(logger, mqttc, sensor_id)
        self.bus = self.get_smbus()
        self.sensor = self.set_baseline(self.bus)

    def get_smbus(self):
        """ Mainly needed by SGP30 to operate in SMBus (I2C) """
        # pylint: disable=R0201
        return SMBus(bus=1, force=0)

    def set_baseline(self, bus, file="/tmp/mySGP30_baseline"):
        """ Baseline initialization """
        # pylint: disable=R0201

        sgp = Sgp30(bus, baseline_filename=file)
        # self.logger.debug("resetting all i2c devices")

        sgp.i2c_geral_call()

        # self.logger.debug(sgp.read_features())
        # self.logger.debug(sgp.read_serial())
        sgp.init_sgp()

        return sgp

    def step(self, queue):
        """ Main loop step for SGP30 sensor """
        # pylint: disable=W0612,W0613

        if self.sensor is None:
            return False

        sgp_measurements = self.sensor.read_measurements()

        queue.append([
            "co2",
            self.measure_hash(self.sensor_id, int(sgp_measurements.data[0]),
                              "co2"), False
        ])
        queue.append([
            "tvoc",
            self.measure_hash(self.sensor_id, int(sgp_measurements.data[1]),
                              "voc"), False
        ])

        self.logger.debug("SGP30: eCO2: " + str(sgp_measurements.data[0]) +
                          " ug/m3 (ppm)"
                          " tVOC: " + str(sgp_measurements.data[1]) +
                          " ng/m3 (ppb)")

        return True


class Voccer:
    """Voccer class"""
    def __init__(self, logger, mqttc, mqtt_server, mqtt_port):
        self.logger = logger
        self.msg_queue = []

        self.mqtt_topic = "/sensor/voccer/2.0/"
        self.mqttc = mqttc
        self.mqtt_server = mqtt_server
        self.mqtt_port = mqtt_port
        self.mqtt_is_connected = False

        self.mqttc.on_message = self.on_message
        self.mqttc.on_connect = self.on_connect
        self.mqttc.on_disconnect = self.on_disconnect
        self.mqttc.on_socket_open = self.on_socket_open
        self.mqttc.on_socjet_close = self.on_socket_close
        # self.mqttc.on_publish = self.on_publish
        self.mqttc.on_subscribe = self.on_subscribe

        # Uncomment to enable debug messages
        # self.mqttc.on_log = self.on_log

        self.mqttc.loop_start()

        try:
            self.mqttc.connect(mqtt_server, mqtt_port, 60)
        except ConnectionRefusedError:
            print("Can't connect to MQTT server " + mqtt_server + ":" +
                  str(mqtt_port))
        except ValueError:
            print("Host is invalid and can connect to MQTT server " +
                  mqtt_server + ":" + str(mqtt_port))

        self.sensor_array = []

    def on_connect(self, client, userdata, flags, rc):
        """Paho Connection callback."""
        # pylint: disable=W0612,W0613,C0103
        self.logger.debug("Paho MQTT Connected: " + str(rc))
        self.mqtt_is_connected = True

    def on_disconnect(self, client, userdata, rc):
        """Paho Disconnection callback."""
        # pylint: disable=W0612,W0613,C0103
        self.logger.debug("Paho MQTT Disconnected: " + str(rc))
        self.mqtt_is_connected = False

    def on_socket_open(self, client, userdata, sock):
        """Paho Socket open callback."""
        # pylint: disable=W0612,W0613
        self.logger.debug("Paho MQTT Socket open")

    def on_socket_close(self, client, userdata, sock):
        """Paho Socjet close callback."""
        # pylint: disable=W0612,W0613
        self.logger.debug("Paho MQTT Socket close")
        self.mqtt_is_connected = False

    def on_message(self, client, userdata, msg):
        """Paho Message send callback."""
        # pylint: disable=W0612,W0613
        self.logger.debug("Paho MQTT Message:" + msg.topic + " " +
                          str(msg.qos) + " " + str(msg.payload))

    def on_publish(self, client, userdata, mid):
        """Paho Message published callback."""
        # pylint: disable=W0612,W0613
        self.logger.debug("Paho MQTT Published: " + str(mid))

    def on_subscribe(self, client, mid, granted_qos):
        """Paho Server subscribe callback."""
        # pylint: disable=W0612,W0613
        self.logger.debug("Paho MQTT Subscribed: " + str(mid) + " " +
                          str(granted_qos))

    def on_log(self, obj, level, string):
        """Paho logging callback."""
        # pylint: disable=W0612,W0613
        self.logger.debug("Log: " + string)

    def add_sensor(self, sensor):
        """ Add sensor to sensor array """
        self.sensor_array.append(sensor)

    def mainloop(self):
        """ Main loop to sensors """
        # pylint: disable=W0612,W0613

        while True:
            # First gather things
            for sensor in self.sensor_array:
                sensor.step(self.msg_queue)

            # Try to send everything in Queue
            # If MQTT server is not there of it doesn't
            # Appear then just keep them in Queue for
            # later on sending or running out of memory
            for current_pos in range(len(self.msg_queue)):
                mqtt_send_tries = 5

                while mqtt_send_tries > 0:
                    if self.mqtt_is_connected is True:
                        current_msg_array = self.msg_queue[current_pos]
                        (rtn_value, mid) = self.mqttc.publish(
                            self.mqtt_topic + current_msg_array[0],
                            json.dumps(current_msg_array[1]),
                            qos=0)

                        current_msg_array[2] = True
                        mqtt_send_tries = -1
                        time.sleep(0.1)
                    else:
                        self.logger.debug(
                            "MQTT Server not connected waiting it to come online"
                        )
                        mqtt_send_tries -= 1
                        time.sleep(1)

                if mqtt_send_tries == 0:
                    break

            new_msg_queue = []

            # Clean sended msg from Queue
            for msg in self.msg_queue:
                if msg[2] is False:
                    new_msg_queue.append(msg)

            # Replace old Queue with new one
            self.msg_queue = new_msg_queue

            # Sleep for while before disable
            time.sleep(30)
            for sensor in self.sensor_array:
                sensor.disable_sensor()
            # Sleep until give sensors couple minutes to settle down
            # and enable them
            time.sleep(7.5 * 60)
            for sensor in self.sensor_array:
                sensor.enable_sensor()
            # Then wait for while that they are good and gather data
            time.sleep(2 * 60)


def main(argv):
    """ Main function """
    # pylint: disable=W0612,W0613,R0912,R0915
    sensor_id = 1
    mqtt_server = "localhost"
    mqtt_port = 1883
    sensors = {}
    sensors['sgp30'] = False
    sensors['pms5003'] = False
    sensors['bme680_first'] = False
    sensors['bme680_second'] = False
    sensors_list = []
    temp_offset = 0
    temp_offset_array = None

    logger = logging.getLogger()

    debug_handler = logging.StreamHandler(sys.stderr)
    formatter = logging.Formatter('%(asctime)s (%(levelname)s): %(message)s')
    debug_handler.setFormatter(formatter)

    try:
        opts, args = getopt.getopt(argv, "hs:m:p:t:e:v", [
            "help", "sensorid=", "mqttserver=", "mqttport=", "tempoffset=",
            "enable=", "verbose"
        ])
    except getopt.GetoptError:
        print('voccer.py (without parameters there should')
        print('be Bosch BME680 available in I2C addr 0x76)')
        print('Parameters:')
        print('  --sensorid (-s) Sensor id')
        print('  --tempoffset (-t) How much is temperature offset (+/-)')
        print(
            '  --enable (-t) Enable sensor. comma separated list for multiple sensors (bme680, sgp30, ccs811 or pms5003)'
        )
        print(
            '                --enable=bme680,bme680,sgp30,pms5003,ccs811 (Multi stuff only works with bme680 currently)'
        )
        print(
            '              if you want to some sensor to have specific if use sensor:id notation like "bme680:1"'
        )
        print('  --mqttserver (-m) MQTT server location (default: localhost)')
        print('  --mqttport (-p) MQTT server port (default: 1883)')
        print('  --verbose (-v) Enable debug priting')
        sys.exit(2)

    for opt, arg in opts:
        if opt == '-h':
            print('voccer.py -s sensor_id -e bme680:id,sgp30:id,pms5300:id')
            sys.exit()
        elif opt in ("-s", "--sensorid"):
            sensor_id = int(arg)
        elif opt in ("-e", "--enable"):
            sensors_list = [x.strip() for x in arg.split(',')]
        elif opt in ("-m", "--mqttserver"):
            mqtt_server = arg
        elif opt in ("-p", "--mqttport"):
            mqtt_port = int(arg)
        elif opt in ("-t", "--tempoffset"):
            if "," in arg:
                temp_offset_array = [x.strip() for x in arg.split(',')]
            else:
                temp_offset = float(arg)
        elif opt in ("-v", "--verbose"):
            logger.setLevel(logging.DEBUG)
            debug_handler.setLevel(logging.DEBUG)

    logger.addHandler(debug_handler)
    mqttc = mqtt.Client()
    voccer_class = Voccer(logger, mqttc, mqtt_server, mqtt_port)
    bme680_addr = bme680.I2C_ADDR_PRIMARY
    bme680_temp = 0

    for sensor_name in sensors_list:
        sensor = None
        sensor_name_lower = sensor_name.lower()

        if ":" in sensor_name_lower:
            sensor_split_array = sensor_name_lower.split(':')
            sensor_id = int(sensor_split_array[1])
            sensor_name_lower = sensor_split_array[0]

        if sensor_name_lower == 'ccs811':
            sensor = ArduinoSensor(logger, mqttc, sensor_id, '/dev/ttyACM0')

        if sensor_name_lower == 'pms5003':
            sensor = PMS5003Sensor(logger, mqttc, sensor_id, '/dev/ttyS0')

        if sensor_name_lower == 'sgp30':
            sensor = SGP30Sensor(logger, mqttc, sensor_id)

        if sensor_name_lower == 'bme680':
            if temp_offset_array is not None:
                temp_offset = float(temp_offset_array[bme680_temp])
                if len(temp_offset_array) > bme680_temp:
                    bme680_temp += 1

            sensor = BME680Sensor(logger, mqttc, sensor_id, bme680_addr,
                                  temp_offset)
            if bme680_addr == bme680.I2C_ADDR_PRIMARY:
                bme680_addr = bme680.I2C_ADDR_SECONDARY

        if sensor is not None:
            voccer_class.add_sensor(sensor)
            sensor_id += 1

    try:
        voccer_class.mainloop()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main(sys.argv[1:])
