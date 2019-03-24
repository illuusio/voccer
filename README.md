# Voccer
Small Python3 based utility to monitor tVOC and eCO2 with Bosch BME680 and SGP30
to MQTT server

First of all read this:
**Please note, this sensor, like all VOC/gas sensors, has variability and to
get precise measurements you will want to calibrate it against known sources!**

This means that you can monitor as much as you can but if you don't know your
baseline then you can't get any good measurements. Project is bit HaX0ur so
there is rough points that you need to fill and everyone is free to commit
Pull Requests to full fill this README.md documentation.

Currently Voccer is work in process and it can change every commit

It's tested with these break-out boards

 * [Adafruit SGP30 Air Quality Sensor Breakout - VOC and eCO2](https://www.adafruit.com/product/3709)
 * [BME680 Breakout - Air Quality, Temperature, Pressure, Humidity Sensor BME680 Breakout - Air Quality, Temperature, Pressure, Humidity Sensor](https://shop.pimoroni.com/products/bme680-breakout)

As noticed they both I2C based so one has to have skills to make them work
with Raspberry Pi you need to have module *i2c-dev* loaded before using
Python libraries. If you can see them in I2C-bus with *i2c-detect* then they
should work.

**As said earlier: it's up to you make sensors work**

## Commanline args
 | Arg | what                       | preset    |
 |-----|----------------------------|-----------|
 | -h  | Help                       |           |
 | -s  | Sensor ID                  | 1         |
 | -f  | Use second addr for BME680 | NO        |
 | -t  | Temp offset for BME680     | 0         |
 | -g  | Use SGP30                  | NO        |
 | -m  | MQTT server address        | localhost |
 | -p  | MQTT server port           | 1883      |

## Getting started with BME680

 * [Getting Started with BME680 Breakout](https://learn.pimoroni.com/tutorial/sandyj/getting-started-with-bme680-breakout)
 * [Pimoroni BME680 Github](https://learn.pimoroni.com/tutorial/sandyj/getting-started-with-bme680-breakout)

## Getting started with SGP30

 * [Python library for reading co2 and TVOC from the Sensirion SGP30 ](https://pypi.org/project/sgp30/)

## JSON
JSON that script outputs to MQTT looks like this
```
{"timestamp": 1553413797.78, "id": 1, "value": 19.23, "type": "temperature", "typeid": 100, "unitid": 100}
```

## Types
All those numbers in typeid and unitid are just numbers started from 100.
 
 | ID  | Type        | Unit    |
 |-----|-------------|---------|
 | 100 | temperature | Celsius |
 | 101 | humidity    | RH%     |
 | 102 | pressure    | hPa     |
 | 103 | AiQ         | AiQ     |
 | 104 | Resistance  | Gas     |
 | 105 | tVOC        | ppm     |
 | 106 | eCO2        | ppm     |

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
| sensorId   | tinyint(4) unsigned | NO   |      | 0 	   |                |
| sensorType | tinyint(4) unsigned | NO   |      | 0 	   |                |
| time 	     | int(11)             | NO   |      | 0 	   |                |
| value 	 | double 	           | NO	  |      | 0 	   |                |

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