# ESP32 Integrated Sensor Node

This repository contains the main firmware for an ESP32-based integrated sensor node.[web:1]

## Overview

The main sketch is **`Full_integratedCode.ino`**. It coordinates all system functions, including:[web:1]

- Reading data from the on-board environmental sensor  
- Driving the OLED display for real-time data
- Logging data to an SD card  
- Acting as an ESP32 client to receive data from independent PPG and accelerometer nodes  
- Streaming all collected data to a web server (SSE)  
- Storing measurements in a local SQLite database [web:1][web:3]

## Environmental tests data

The folder **`environmental_test`** contains CSV files recorded during the environmental sensor characterization and validation tests. 

## Commercial PPG

The sketch **`spare_ppg.ino`** programs the commercial heart rate sensor as a server to communicate via BLE to teh ESP32 programmed by **`Full_integratedCode.ino`**
