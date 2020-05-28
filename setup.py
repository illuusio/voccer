#!/usr/bin/python3
import os
import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="voccer",
    version="0.0.1",
    author="Tuukka Pasanen",
    author_email="tuukka.pasanen@ilmi.fi",
    description=
    ("Small Python3 based utility to send tVOC, eCO2 and Particulate Matter to MQTT server"
     ),
    license="Propiertary",
    keywords="tVOC VOC eCO2 CO2 Matter",
    url="https://github.com/illuusio/voccer",
    scripts=['voccer.py'],
    packages=setuptools.find_packages(),
    install_requires=[
        'paho.mqtt.client>=1.0.0'
        'RPi.GPIO>=0.6.0'
        'bme680>=1.0.0'
        'smbus2>=0.3.0'
        'sgp30>=0.1.6'
        'pms5003>=0.0.5'
    ],
    long_description=long_description,
    long_description_content_type="text/markdown",
    classifiers=[
        "Development Status :: 2 - Beta",
        "Topic :: Utilities",
    ],
)
