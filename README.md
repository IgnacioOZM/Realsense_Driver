# Realsense_Driver
Author: Ignacio Ortiz de Zúñiga Mingot
Date: June 2021
Course: 2º MII + MIC - ICAI

## Introduction
Python driver to control and calibrate an intel realsense RGB-D camera. It has been tune to be used with the L515 but it can be easily modified to work with other cameras.

## driver.py
Definition of a class to handle all functions and variables needed to controll the camera. It uses parameters and functions from the realsense SDK class pipeline.

## Camera_Calibration.py
Function to analyze and determine the intrinsic and extrinsic of a Realsense camera but it can be used with any camera. After determining all the parameters, it exports them to numpy compress folder. The Realsense driver will automatically use this parameters for image correction.

## Example_*.py
Simple scripts to visualize how to implement the driver. Database_creation can be used to obtain new images to recalibrate a camera.

## Config.json & Intrinsics&Extrinsics.npz
Contains the specfic configuration to be loaded in the camera.
