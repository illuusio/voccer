//
//  CCS811 Arduino CSV reader
//
//  Copyright (c) 2020 Tuukka Pasanen
//  SPDX-License-Identifier: MIT
//
//  https://github.com/sparkfun/CCS811_Air_Quality_Breakout
//  https://github.com/sparkfun/SparkFun_CCS811_Arduino_Library
//
//  It's tested on Sparkfun CCS811 board but should work with any version.
//  Read the TVOC and CO2 values from the SparkFun CCS811 breakout board and
//  print them as CSV to serialport with speed 9600.
//  Support 2 separated CCS811 so if you don't need 2 then just remove it.
//
//  Resources:
//  Uses Wire.h for i2c operation
//  Uses SparkFun_CCS811.h for CCS811 operations
//
//  Development environment specifics:
//  Arduino IDE 1.8.x
//  Arduino UNO
//
//  Distributed as-is; no warranty is given.
#include <Wire.h>

// You need SparkFun CCS811 Library to work with this
// Click here to get the library: http://librarymanager/All#SparkFun_CCS811
#include "SparkFunCCS811.h"

// One can use two separates CCS811 sensors
// High addr of CCS811 I2C Address (Default)
#define CCS811_HIGH_ADDR 0x5B
// Low addr of CCS811 I2C Address
#define CCS811_LOW_ADDR 0x5A

// Correct temperature as these Sparkfun
// boards have termistor but result it more than
// unpleasent
#define CCS811_HIGH_TEMPCOMP -5.5
#define CCS811_LOW_TEMPCOMP -7.5

#define CCS811_READ_DELAY 1000
#define CSS811_PRINT_NOW (60*5)

CCS811 highSensor(CCS811_HIGH_ADDR);
CCS811 lowSensor(CCS811_LOW_ADDR);
int printHeader = 100;
int printNow = 0;

void setup()
{
    Serial.begin(9600);
    Wire.begin(); //Inialize I2C Harware


    // Enable first sensor
    CCS811Core::status returnCode = highSensor.begin();
    if (returnCode != CCS811Core::SENSOR_SUCCESS)
    {
        Serial.println("# CCS811.begin() returned with an error for Addr 0x5B. (First sensor)");
        while (1); //Hang if there was a problem.
    }

    // Enable second sensor
    returnCode = lowSensor.begin();
    if (returnCode != CCS811Core::SENSOR_SUCCESS)
    {
        Serial.println("# CCS811.begin() returned with an error for Addr 0x5A. (Second sensor)");
        while (1); //Hang if there was a problem.
    }

    Serial.println("# CCS811 Voccer printer");

    delay(100);
}

/**
 * Print is data is available it as CSV
 *
 * @param id Sensor id
 * @param sensor Sensor in use
 * @param tempcom Temperrature compensation
 */

void dataAvailable(int id, CCS811 *sensor, float tempcomp)
{
    // If so, have the sensor read and calculate the results.
    // Get them later
    sensor->readAlgorithmResults();
    sensor->readNTC();
    float readTemperature = sensor->getTemperature() + tempcomp;

    // .readNTC() causes the CCS811 library to gather ADC data and save value
    // Serial.print(" Measured resistance : ");
    // After .readNTC() is called, .getResistance() can be called to actually
    // get the resistor value.  This is not needed to get the temperature,
    // but can be useful information for debugging.
    //
    // Use the resistance value for custom thermistors, and calculate the
    // temperature yourself.
    // Serial.print( highSensor.getResistance() );
    // Serial.println(" ohms");

    // Pass the temperature back into the CCS811 to compensate
    // If board doesn't have termistor skip this or readings
    // are wrong
    sensor->setEnvironmentalData(50, readTemperature);

    if(printNow <= 0)
    {
        // As this is CSV output then print what they mean once and while
        if(printHeader >= 80)
        {
            Serial.println("# CO2, tVOC, Deg C, millis from start");
            printHeader = 0;
        }
        printHeader ++;

        // This is very poor way to print CSV but we have limited amount
        // of everything so we don't waste it

        Serial.print(id);
        Serial.print(",");

        // Returns calculated CO2 reading
        Serial.print(sensor->getCO2());
        Serial.print(",");
        // Returns calculated TVOC reading
        Serial.print(sensor->getTVOC());
        Serial.print(",");
        Serial.print( readTemperature, 2);
        Serial.print(",");

        // Simply the time since program start
        Serial.print(millis());
        Serial.println();
    }

}


void loop()
{
    // Check to see if data is ready with .dataAvailable()
    if (highSensor.dataAvailable())
    {
        dataAvailable(1, &highSensor, CCS811_HIGH_TEMPCOMP);
    }
    else if (highSensor.checkForStatusError())
    {
        printSensorError(&highSensor);
    }
    //else
    //{
    //    Serial.println("# Can't get measurement or error from sensor 1");
    //}


    // Check to see if data is ready with .dataAvailable()
    if (lowSensor.dataAvailable())
    {
        dataAvailable(2, &lowSensor, CCS811_LOW_TEMPCOMP);
    }
    else if (lowSensor.checkForStatusError())
    {
        printSensorError(&lowSensor);
    }
    // else
    // {
    //  Serial.println("# Can't get measurement or error from sensor 2");
    // }

    if(printNow <= 0)
    {
       printNow = CSS811_PRINT_NOW;
    }

    printNow --;

    delay(CCS811_READ_DELAY);
}

/**
 * Print sensor errors
 *
 * @param sensor sensor which error we print
 */

void printSensorError(CCS811 *sensor)
{
    uint8_t error = sensor->getErrorRegister();

    //comm error
    if ( error == 0xFF )
    {
        Serial.println("# Failed to get ERROR_ID register.");
    }
    else
    {
        Serial.print("# Error: ");
        if (error & 1 << 5) Serial.print("# HeaterSupply");
        if (error & 1 << 4) Serial.print("# HeaterFault");
        if (error & 1 << 3) Serial.print("# MaxResistance");
        if (error & 1 << 2) Serial.print("# MeasModeInvalid");
        if (error & 1 << 1) Serial.print("# ReadRegInvalid");
        if (error & 1 << 0) Serial.print("# MsgInvalid");
        Serial.println();
    }
}
