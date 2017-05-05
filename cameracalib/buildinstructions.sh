#!/bin/bash

g++ -ggdb `pkg-config --cflags opencv` -o `basename camera_calibration.cpp .cpp` camera_calibration.cpp `pkg-config --libs opencv`
