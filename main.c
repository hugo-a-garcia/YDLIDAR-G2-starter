//
// The MIT License (MIT)
//
// Copyright (c) 2019 EAIBOT. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
// Modified by Hugo A. Garcia

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif
#include "ydlidar_sdk.h"

#if defined(_MSC_VER)
#pragma comment(lib, "ydlidar_sdk.lib")
#endif

int main(int argc, const char *argv[])
{
    // os_init() will install a SIGINT handler which provides Ctrl-C handling which will
    // cause os_isOk() to return false if that happens.
    os_init();
    // Create a handle to this Lidar.
    YDLidar *laser = lidarCreate();
    // Query avaliable Lidar ports.
    char port[50] = "/dev/ydlidar";
    LidarPort ports;
    int size = lidarPortList(&ports);
    int i = 0;
    for (i = 0; i < size; i++)
    {
        printf("port: %s\n", ports.port[i].data);
        /// last port
        strcpy(port, ports.port[i].data);
    }

    // Set Lidar string property paramters. //

    //  Lidar serial port
    setlidaropt(laser, LidarPropSerialPort, port, sizeof(port));
    // Lidar ignore angle array
    strcpy(port, "");
    setlidaropt(laser, LidarPropIgnoreArray, port, sizeof(port));

    // Set Lidar int property paramters. //

    // Lidar serial baudrate
    int i_optvalue = 230400;
    setlidaropt(laser, LidarPropSerialBaudrate, &i_optvalue, sizeof(int));
    // Lidar type code
    i_optvalue = TYPE_TRIANGLE;
    setlidaropt(laser, LidarPropLidarType, &i_optvalue, sizeof(int));
    // Lidar connection type code
    i_optvalue = YDLIDAR_TYPE_SERIAL;
    setlidaropt(laser, LidarPropDeviceType, &i_optvalue, sizeof(int));
    // Lidar Sample Rate ()
    i_optvalue = 5;
    setlidaropt(laser, LidarPropSampleRate, &i_optvalue, sizeof(int));
    // Abnormal maximum check times
    i_optvalue = 4;
    setlidaropt(laser, LidarPropAbnormalCheckCount, &i_optvalue, sizeof(int));

    // Set Lidar bool property paramters. //

    bool b_optval = true;
    // Auto reconnect (hot plug)
    setlidaropt(laser, LidarPropAutoReconnect, &b_optval, sizeof(bool));
    b_optval = false;
    // Single Channel
    setlidaropt(laser, LidarPropSingleChannel, &b_optval, sizeof(bool));
    // Intensity
    setlidaropt(laser, LidarPropIntenstiy, &b_optval, sizeof(bool));
    // Counter clocwise rotation
    setlidaropt(laser, LidarPropInverted, &b_optval, sizeof(bool));
    // Rotate 180 (reverse 0 deggre point ???)
    setlidaropt(laser, LidarPropReversion, &b_optval, sizeof(bool));
    // Motor DTR control
    setlidaropt(laser, LidarPropSupportMotorDtrCtrl, &b_optval, sizeof(bool));
    // Fixed angle resolution
    setlidaropt(laser, LidarPropFixedResolution, &b_optval, sizeof(bool));
    // Heartbeat
    setlidaropt(laser, LidarPropSupportHeartBeat, &b_optval, sizeof(bool));

    // Set Lidar float property paramters. //

    // Scan Frequency (Hz) [5(7)12]
    float f_optval = 12.0f;
    setlidaropt(laser, LidarPropScanFrequency, &f_optval, sizeof(float));
    // Maxiumum angle (°)
    f_optval = 180.0f;
    setlidaropt(laser, LidarPropMaxAngle, &f_optval, sizeof(float));
    // Minimum angle (°)
    f_optval = -180.0f;
    setlidaropt(laser, LidarPropMinAngle, &f_optval, sizeof(float));
    // Maximum Range (m)
    f_optval = 16.f;
    setlidaropt(laser, LidarPropMaxRange, &f_optval, sizeof(float));
    // Minimum Range (m)
    f_optval = 0.12f;
    setlidaropt(laser, LidarPropMinRange, &f_optval, sizeof(float));

    // Print Baudrate
    getlidaropt(laser, LidarPropSerialBaudrate, &i_optvalue, sizeof(int));
    printf("baudrate: %d\n", i_optvalue);

    // Initialize the SDK and LiDAR.
    bool isInitialized = false;
    isInitialized = initialize(laser);
    if (!isInitialized)
    {
        // SDK will print error message:
        // - Serial port does not correspond to the actual Lidar.
        // - Serial port does not have read and write permissions.
        // - Lidar baud rate settings error.
        // - Incorrect Lidar type setting.
        lidarDestroy(&laser);
        fprintf(stderr, "EXIT_FAILURE : initialize()\n");
        return EXIT_FAILURE;
    }

    // Lidar is initialized. Turn on the Lidar
    // Start the device scanning routine which runs on a separate thread and enable motor.
    bool isOn = false;
    isOn = turnOn(laser);
    if (!isOn)
    {
        // SDK will print error message:
        // - Lidar stall.
        // - Lidar power suppy is unstable.
        lidarDestroy(&laser);
        fprintf(stderr, "EXIT_FAILURE : turnOn()\n");
        return EXIT_FAILURE;
    }

    // FOR REFERENCE ONLY; Copied from 'ydlidar_def.h' :
    //
    /**
    * @brief The Laser Point struct
    * @note angle unit: rad.\n
    * range unit: meter.\n
    */
    // typedef struct
    // {
    //     /// lidar angle. unit(rad)
    //     float angle;
    //     /// lidar range. unit(m)
    //     float range;
    //     /// lidar intensity
    //     float intensity;
    // } LaserPoint;
    //
    // Laser Fan struct
    // typedef struct
    // {
    //     /// System time when first range was measured in nanoseconds
    //     uint64_t stamp; ///< ns
    //     /// Array of lidar points
    //     uint32_t npoints;
    //     LaserPoint *points;
    //     /// Configuration of scan
    //     LaserConfig config;
    // } LaserFan;
    LaserFan scan;
    LaserFanInit(&scan);

    // os_isOk() will return false if:
    // - a SIGINT is received (Ctrl-C)
    // - ydlidar::os_shutdown() has been called by another part of the application.
    while (isOn && os_isOk())
    {
        if (doProcessSimple(laser, &scan))
        {
            fprintf(stdout, "Scan received[%lu]: %u ranges is [%f]Hz\n",
                    scan.stamp,
                    (unsigned int)scan.npoints, 1.0 / scan.config.scan_time);
            fflush(stdout);
        }
        else
        {
            fprintf(stderr, "Failed to get Lidar Data\n");
            fflush(stderr);
        }
    }
    LaserFanDestroy(&scan);
    turnOff(laser);
    disconnecting(laser);
    lidarDestroy(&laser);
    return EXIT_SUCCESS;
}
