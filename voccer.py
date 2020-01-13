""" VOCCER is simple Python 3.x script which
    Tries to read from diffrent sensors
    and forward them to MQTT-server which """

# Copyright (c) 2019 Tuukka Pasanen
# SPDX-License-Identifier: MIT

# pylint: disable=E0401
import json
import time
import sys
import getopt
import paho.mqtt.client as mqtt
import bme680
from smbus2 import SMBusWrapper
from sgp30 import Sgp30

def on_connect(client, userdata, flags, rc):
    """Paho Connection callback."""
    # pylint: disable=W0612,W0613
    print("Paho MQTT Connected: " + str(rc))

def on_disconnect(client, userdata, rc):
    """Paho Disconnection callback."""
    # pylint: disable=W0612,W0613
    print("Paho MQTT Disconnected: " + str(rc))

def on_socket_open(client, userdata, sock):
    """Paho Socket open callback."""
    # pylint: disable=W0612,W0613
    print("Paho MQTT Socket open")

def on_socket_close(client, userdata, sock):
    """Paho Socjet close callback."""
    # pylint: disable=W0612,W0613
    print("Paho MQTT Socket close")

def on_message(client, userdata, msg):
    """Paho Message send callback."""
    # pylint: disable=W0612,W0613
    print("Paho MQTT Message:" + msg.topic + " " + str(msg.qos) + " " + str(msg.payload))

def on_publish(client, userdata, mid):
    """Paho Message published callback."""
    # pylint: disable=W0612,W0613
    print("Paho MQTT Published: " + str(mid))

def on_subscribe(client, mid, granted_qos):
    """Paho Server subscribe callback."""
    # pylint: disable=W0612,W0613
    print("Paho MQTT Subscribed: " + str(mid) + " " + str(granted_qos))

def on_log(obj, level, string):
    """Paho logging callback."""
    # pylint: disable=W0612,W0613
    print("Log: " + string)

def measure_hash(sensor_id, measure_value, measure_type):
    """Create measurement JSON"""
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
        unitid = 106

    return json.dumps({
        'timestamp': round(time.time(), 2),
        'id': int(sensor_id),
        'value': measure_value,
        'type': measure_type,
        'typeid': typeid,
        'unitid': unitid
    })


def air_quality_score(sensor, gas_baseline, hum_baseline):
    """Calculate air quality score from BME680 or else"""
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


def get_bme680_air_baseline(sensor):
    """ Calculate Gas baseline """
    start_time = time.time()
    curr_time = time.time()
    burn_in_time = 5 * 60

    burn_in_data = []

    try:
        # Collect gas resistance burn-in values, then use the average
        # of the last 50 values to set the upper limit for calculating
        # gas_baseline.
        print('Collecting gas resistance burn-in data for 5 mins')
        while curr_time - start_time < burn_in_time:
            left_time = int(burn_in_time - (curr_time - start_time))
            curr_time = time.time()
            if sensor.get_sensor_data() and sensor.data.heat_stable:
                gas = sensor.data.gas_resistance
                burn_in_data.append(gas)
                sys.stdout.write('Gas {0}: {1} Ohms     \r'.format(left_time, round(gas, 1)))
                sys.stdout.flush()
                time.sleep(2)

        gas_baseline = sum(burn_in_data[-50:]) / 50.0
    except KeyboardInterrupt:
        pass
    return gas_baseline


def set_bme680_config(sensor, temp_offset):
    """ Function to set confis for BOSCH BME680
        These oversampling settings can be tweaked to
        change the balance between accuracy and noise in
        the data """
    sensor.set_humidity_oversample(bme680.OS_2X)
    sensor.set_pressure_oversample(bme680.OS_4X)
    sensor.set_temperature_oversample(bme680.OS_8X)
    sensor.set_filter(bme680.FILTER_SIZE_3)
    sensor.set_gas_status(bme680.ENABLE_GAS_MEAS)
    sensor.set_temp_offset(temp_offset)

#    print('\n\nInitial reading:')
#    for name in dir(sensor.data):
#      value = getattr(sensor.data, name)
#
#      if not name.startswith('_'):
#          print('{}: {}'.format(name, value))

    sensor.set_gas_heater_temperature(320)
    sensor.set_gas_heater_duration(150)
    sensor.select_gas_heater_profile(0)

#    print('Calibration data:')
#    for name in dir(sensor.calibration_data):
#
#        if not name.startswith('_'):
#            value = getattr(sensor.calibration_data, name)
#
#            if isinstance(value, int):
#                print('{}: {}'.format(name, value))


def mainloop_bme680(sensor, sensor_id, mqttc, mqtt_topic, gas_baseline):
    """ Main loop to run Bosch BME680 sensor """
    # pylint: disable=W0612,W0613

    # Set the humidity baseline to 40%, an optimal indoor humidity.
    hum_baseline = 40.0

    print('Gas baseline: {0} Ohms, humidity baseline: {1:.2f} %RH\n'.format(
        gas_baseline, hum_baseline))

    while True:
        if sensor.get_sensor_data():
            temperature = measure_hash(sensor_id,
                                       round(sensor.data.temperature, 2),
                                       "temperature")
            pressure = measure_hash(sensor_id, int(sensor.data.pressure),
                                    "pressure")
            humidity = measure_hash(sensor_id, int(sensor.data.humidity),
                                    "humidity")
            gas = measure_hash(sensor_id, round(sensor.data.gas_resistance, 2),
                               "gas")

            air_quality_scr = air_quality_score(sensor, gas_baseline,
                                                hum_baseline)
            air_quality = measure_hash(sensor_id, round(air_quality_scr, 2),
                                       "quality")

            print(
                'Temperature: {0:.2f}C, Pressure: {1:.2f} HPa, Humidity {2:.2f} '
                .format(sensor.data.temperature, sensor.data.pressure,
                        sensor.data.humidity))
            print(
                '%RH, Resistance: {0:.2f} Ohm, Quality Indx {1:.2f}\n'.format(
                    sensor.data.gas_resistance, air_quality_scr))

            (rtn_value, mid) = mqttc.publish(
                mqtt_topic + "temperature", temperature, qos=0)

            (rtn_value, mid) = mqttc.publish(
                mqtt_topic + "pressure", pressure, qos=0)

            (rtn_value, mid) = mqttc.publish(
                mqtt_topic + "humidity", humidity, qos=0)

            (rtn_value, mid) = mqttc.publish(mqtt_topic + "gas", gas, qos=0)

            (rtn_value, mid) = mqttc.publish(
                mqtt_topic + "quality", air_quality, qos=0)

        time.sleep(10 * 60)


def sgp30_mainloop(sensor_id, mqttc, mqtt_topic):
    """ Main loop to run SGP30 sensor """
    # pylint: disable=W0612,W0613
    with SMBusWrapper(1) as bus:
        sgp = Sgp30(
            bus, baseline_filename="/tmp/mySGP30_baseline"
        )  #things thing with the baselinefile is dumb and will be changed in the future
        print("resetting all i2c devices")

        sgp.i2c_geral_call(
        )  #WARNING: Will reset any device on teh i2cbus that listens for general call

        print(sgp.read_features())
        print(sgp.read_serial())
        sgp.init_sgp()

        while True:
            sgp_measurements = sgp.read_measurements()

            co2 = measure_hash(sensor_id, int(sgp_measurements.data[0]), "co2")
            tvoc = measure_hash(sensor_id, int(sgp_measurements.data[1]),
                                "voc")

            (rtn_value, mid) = mqttc.publish(mqtt_topic + "co2", co2, qos=0)
            (rtn_value, mid) = mqttc.publish(mqtt_topic + "tvoc", tvoc, qos=0)

            print("eCO2: " + str(sgp_measurements.data[0]) + " tVOC: " +
                  str(sgp_measurements.data[1]))
            time.sleep(10 * 60)


def main(argv):
    """ Main function """
    # pylint: disable=W0612,W0613,R0912
    sensor_id = 1
    bme680_addr = bme680.I2C_ADDR_PRIMARY
    mqtt_server = "localhost"
    mqtt_port = 1883
    sgp_sensor = False
    temp_offset = 0

    try:
        opts, args = getopt.getopt(argv, "hs:fm:p:gt:", [
            "sensorid=", "bme680second", "mqttserver", "mqttport", "sgp30",
            "tempoffset"
        ])
    except getopt.GetoptError:
        print('voccer.py (without parameters there should')
        print('be Bosch BME680 available in I2C addr 0x76)')
        print('Parameters:')
        print('  --sensorid (-s) Sensor id')
        print('  --bme680second (-f) If there is second BME680 available in 0x77')
        print('  --sgp30 (-g) Use Sensirion SGP30 not BME680')
        print('  --tempoffset (-t) How much is temperature offset (+/-)')
        print('  --mqttserver (-m) MQTT server location (default: localhost)')
        print('  --mqttport (-p) MQTT server port (default: 1883)')
        sys.exit(2)

    for opt, arg in opts:
        if opt == '-h':
            print('voccer.py -s sensor_id -f Bosch BME680 secondary addr')
            sys.exit()
        elif opt in ("-s", "--sensorid"):
            sensor_id = int(arg)
        elif opt in ("-f", "--bme680second"):
            bme680_addr = bme680.I2C_ADDR_SECONDARY
        elif opt in ("-g", "--sgp30"):
            sgp_sensor = True
        elif opt in ("-m", "--mqttserver"):
            mqtt_server = arg
        elif opt in ("-p", "--mqttport"):
            mqtt_port = int(arg)
        elif opt in ("-t", "--tempoffset"):
            temp_offset = float(arg)

    mqttc = mqtt.Client()

    mqttc.on_message = on_message
    mqttc.on_connect = on_connect
    mqttc.on_disconnect = on_disconnect
    mqttc.on_socket_open = on_socket_open
    mqttc.on_socjet_close = on_socket_close
    # mqttc.on_publish = on_publish
    mqttc.on_subscribe = on_subscribe

    # Uncomment to enable debug messages
    # mqttc.on_log = on_log

    mqttc.loop_start()

    try:
        mqttc.connect(mqtt_server, mqtt_port, 60)
    except ConnectionRefusedError:
        print("Can't connect to " + mqtt_server + ":" + str(mqtt_port))

    mqtt_topic = "/sensor/voccer/2.0/"

    if sgp_sensor is False:
        try:
            sensor = bme680.BME680(bme680_addr)
        except IOError as execption_str:
            print("Can't open BME680 at I2C addr: "
                  + str(hex(bme680_addr)) + " (" + str(execption_str) + ")")
            sys.exit(2)
    else:
        sgp30_mainloop(sensor_id, mqttc, mqtt_topic)
        sys.exit(0)

    set_bme680_config(sensor, temp_offset)

    try:
        gas_baseline = get_bme680_air_baseline(sensor)

        mainloop_bme680(sensor, sensor_id, mqttc, mqtt_topic, gas_baseline)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main(sys.argv[1:])
