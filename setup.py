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
        'paho.mqtt.client'
        'RPi.GPIO'
        'bme680'
        'smbus2'
        'sgp30'
        'pms5003'
    ],
    long_description=long_description,
    long_description_content_type="text/markdown",
    classifiers=[
        "Development Status :: 2 - Beta",
        "Topic :: Utilities",
    ],
)
