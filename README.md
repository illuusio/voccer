# Voccer
Small Python3 based utility to send tVOC, eCO2, Particulate Matter with Bosch BME680, AMS CCS811, Sensirion SGP30 and Plantower PMS5003
measurements to MQTT server

First of all read this:
**Please note, this sensor, like all VOC/gas sensors, has variability and to
get precise measurements you will want to calibrate it against known sources!**

This means that you can monitor as much as you can but if you don't know your
baseline then you can't get any good measurements. Project is bit HaX0ur so
there is rough points that you need to fill and everyone is free to commit
Pull Requests to full fill this README.md documentation.

Currently Voccer is work in process and it can change every commit

It's tested with these break-out boards and sensors

 * [Adafruit SGP30 Air Quality Sensor Breakout - VOC and eCO2](https://www.adafruit.com/product/3709)
 * [BME680 Breakout - Air Quality, Temperature, Pressure, Humidity Sensor BME680 Breakout - Air Quality, Temperature, Pressure, Humidity Sensor](https://shop.pimoroni.com/products/bme680-breakout)
 * [SparkFun Air Quality Breakout - CCS811](https://www.sparkfun.com/products/14193)
 * [PMS5003 Particulate Matter Sensor Breakout](https://shop.pimoroni.com/products/particulate-matter-sensor-breakout) and [PMS5003 Particulate Matter Sensor with Cable](https://shop.pimoroni.com/products/pms5003-particulate-matter-sensor-with-cable)

As noticed they both I2C based so one has to have skills to make them work
with Raspberry Pi you need to have module *i2c-dev* loaded before using
Python libraries. If you can see them in I2C-bus with *i2c-detect* then they
should work.

Most of the python libraries can be found with Python pip package manager
**CCS811 doesn't play well with Raspberry Pi. You need Arduino UNO to use CCS811**

**As said earlier: it's up to you make sensors work**

## Commandline args
 | Arg | what                       | preset    |
 |-----|----------------------------|-----------|
 | -h  | Help                       |           |
 | -s  | Sensor ID                  | 1         |
 | -t  | Temp offset for BME680     | 0         |
 | -e  | Enable sensor              | Nothing   |
 | -m  | MQTT server address        | localhost |
 | -p  | MQTT server port           | 1883      |

## Commandline example 
```
python3 voccer.py --enable=bme680:2,bme680:4,sgp30:2,pms5003:1,css811:5
```
Which enables first address BME680 (0x76) with ID 2, second address BME680 (0x77) with ID 4, SGP30 with ID 2
and PMS5003 with ID 1

## Getting started with BME680

 * [Getting Started with BME680 Breakout](https://learn.pimoroni.com/tutorial/sandyj/getting-started-with-bme680-breakout)
 * [Pimoroni BME680 Github](https://learn.pimoroni.com/tutorial/sandyj/getting-started-with-bme680-breakout)

## Getting started with SGP30

 * [Python library for reading co2 and TVOC from the Sensirion SGP30 ](https://pypi.org/project/sgp30/)

## Getting started with CCS811

In subdirecotory ccs811_arduino is application which you need to compile and upload to Arduino UNO.
After that your Arduino UNO has to be attached to Raspberry Pi (or machine you like use) with USB cable.
Then it should work. It outputs CSV so if you just log CCS811 (or two of them) then you can just log
serial port to file and analyze with Libreoffice or similar.

Please note that it takes 2 IDs from MQTT if you use two CSS811 sensors.

**NOTE! You have to have correct rights to read serial port in your machine.**

## Getting started with PMS5003

 * [PMS5003 Particulate Sensor](https://github.com/pimoroni/pms5003-python)

## JSON
JSON that script outputs to MQTT looks like this
```
{"timestamp": 1553413797.78, "id": 1, "value": 19.23, "type": "temperature", "typeid": 100, "unitid": 100}
```

## Types
All those numbers in typeid and unitid are just numbers started from 100.
 
 | ID  | Type                            | Unit    | UnitID |
 |-----|---------------------------------|---------|--------|
 | 100 | temperature                     | Celsius | 100    |
 | 101 | humidity                        | RH%     | 101    |
 | 102 | pressure                        | hPa     | 102    |
 | 103 | AiQ                             | AiQ     | 103    |
 | 104 | Resistance                      | Gas     | 104    |
 | 105 | tVOC                            | ppm     | 105    |
 | 106 | eCO2                            | ppm     | 105    |
 | 107 | Particle PM1                    | ug/m3   | 107    |
 | 108 | Particle PM2.5                  | ug/m3   | 107    |
 | 109 | Particle PM10                   | ug/m3   | 107    |
 | 110 | Atmospheric particle PM1        | ug/m3   | 107    |
 | 111 | atmospheric particle PM2.0      | ug/m3   | 107    |

## Listening JSON from MQTT server
and you can listen them with for example [Mosquitto MQTT-project](http://mosquitto.org/) tool *mosquitto_sub*
```
mosquitto_sub -t /sensor/voccer/2.0/+
```

# Setting up Fluentd
There is several servers which can log these into database like [Fluentd](https://www.fluentd.org/) and
[Telegraf](https://github.com/influxdata/telegraf) for example.
If you choose to use Fluentd you need [Fluent plugin for MQTT Input/Output](https://github.com/toyokazu/fluent-plugin-mqtt-io) and if
you like to log to database like MySQL or MariaDB then you need [SQL input/output plugin for Fluentd](https://github.com/fluent/fluent-plugin-sql)
**As Raspbian doesn't support Ruby very you are mostly out of luck just use Pure Raspberry Pi solution**

## MySQL table setup
Project doen't currently provice SQL-statement to create table but it's like this

| Field      | Type 	           | Null |  Key | Default | Extra          |
|------------|---------------------|------|------|---------|----------------|
| id         | bigint(20)          | NO   | PRI  | NULL    | auto_increment |
| unitType   | tinyint(4) unsigned | NO   |      | 0       |                |	
| sensorId   | tinyint(4) unsigned | NO   |      | 0 	   |                |
| sensorType | tinyint(4) unsigned | NO   |      | 0 	   |                |
| time       | int(11)             | NO   |      | 0       |                |
| value      | double              | NO   |      | 0       |                |

## Fluentd setup
There is good documentation for Fluentd configuration but this is simple config file
to get Voccer logging working
```
# In v1 configuration, type and id are @ prefix parameters.
# @type and @id are recommended. type and id are still available for backward compatibility

<source>
  @type mqtt
  host 127.0.0.1
  port 1883
  topic /sensor/voccer/2.0/+
  <parse>
    @type json
  </parse>
</source>

<match .sensor.voccer.2.0.**>
  @type sql
  host localhost
  port 3306
  database voccer
  adapter mysql2
  username voccer
  password voccer
  remove_tag_prefix .sensor.voccer.2.0

  <table>
    table measurement
    column_mapping 'timestamp:time,unitid:unitType,id:sensorId,typeid:sensorType,value:value'
  </table>
</match>
```

# Grafana setup
[Grafana](https://grafana.com/) is state of art visualition tool and suits to this purpose very well
after installing and creating dashboard with MySQL connector one can create SQL query statement for Panel
like this
```
SELECT
  time AS "time",
  value AS value
FROM measurement
WHERE
  $__unixEpochFilter(time) AND
  sensorType = 100 AND
  sensorId = 1
ORDER BY time
```
And it will start drawing temperature panel for sensor 1.
