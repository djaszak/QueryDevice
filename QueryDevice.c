/*****************************************************************************

Copyright (c) 2004 SensAble Technologies, Inc. All rights reserved.

OpenHaptics(TM) toolkit. The material embodied in this software and use of
this software is subject to the terms and conditions of the clickthrough
Development License Agreement.

For questions, comments or bug reports, go to forums at: 
    http://dsc.sensable.com

Module Name:
  
  QueryDevice.c

Description:

  This example demonstrates how to retrieve information from the haptic device.

*******************************************************************************/
#ifdef  _WIN64
#pragma warning (disable:4996)
#endif

#if defined(WIN32)
# include <windows.h>
# include <conio.h>
#else
# include "conio.h"
# include <string.h>
#endif

#define CURL_STATICLIB
#include <curl\curl.h>

#define _USE_MATH_DEFINES // for C
#include <math.h>

#include <stdio.h>
#include <assert.h>

#include <HD/hd.h>

#include <HDU/hduVector.h>
#include <HDU/hduError.h>

/* Holds data retrieved from HDAPI. */
typedef struct 
{
    HDboolean m_buttonState;       /* Has the device button has been pressed. */
    hduVector3Dd m_devicePosition; /* Current device coordinates. */
    hduVector3Dd m_jointAngles;
    hduVector3Dd m_gimbalAngles;
    HDErrorInfo m_error;
} DeviceData;

static DeviceData gServoDeviceData;

/*******************************************************************************
 Checks the state of the gimbal button and gets the position of the device.
*******************************************************************************/
HDCallbackCode HDCALLBACK updateDeviceCallback(void *pUserData)
{   
    int nButtons = 0;

    hdBeginFrame(hdGetCurrentDevice());

    /* Retrieve the current button(s). */
    hdGetIntegerv(HD_CURRENT_BUTTONS, &nButtons);
    
    /* In order to get the specific button 1 state, we use a bitmask to
       test for the HD_DEVICE_BUTTON_1 bit. */
    gServoDeviceData.m_buttonState = 
        (nButtons & HD_DEVICE_BUTTON_1) ? HD_TRUE : HD_FALSE;
        
    /* Get the current location of the device (HD_GET_CURRENT_POSITION)
       We declare a vector of three doubles since hdGetDoublev returns 
       the information in a vector of size 3. */
    hdGetDoublev(HD_CURRENT_POSITION, gServoDeviceData.m_devicePosition);

    hdGetDoublev(HD_CURRENT_JOINT_ANGLES, gServoDeviceData.m_jointAngles);

    hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, gServoDeviceData.m_gimbalAngles);

    /* Also check the error state of HDAPI. */
    gServoDeviceData.m_error = hdGetError();

    /* Copy the position into our device_data tructure. */
    hdEndFrame(hdGetCurrentDevice());

    return HD_CALLBACK_CONTINUE;    
}


/*******************************************************************************
 Checks the state of the gimbal button and gets the position of the device.
*******************************************************************************/
HDCallbackCode HDCALLBACK copyDeviceDataCallback(void *pUserData)
{
    DeviceData *pDeviceData = (DeviceData *) pUserData;

    memcpy(pDeviceData, &gServoDeviceData, sizeof(DeviceData));

    return HD_CALLBACK_DONE;
}


/*******************************************************************************
 Prints out a help string about using this example.
*******************************************************************************/
void printHelp(void)
{
    static const char help[] = {"\
Press and release the stylus button to print out the current device location.\n\
Press and hold the stylus button to exit the application\n"};

    fprintf(stdout, "%s\n", help);
}


/*******************************************************************************
 This routine allows the device to provide information about the current 
 location of the stylus, and contains a mechanism for terminating the 
 application.  
 Pressing the button causes the application to display the current location
 of the device.  
 Holding the button down for N iterations causes the application to exit. 
*******************************************************************************/
void mainLoop(void)
{

    /* Instantiate the structure used to capture data from the device. */
    DeviceData currentData;
    DeviceData prevData;

    /* Perform a synchronous call to copy the most current device state. */
    hdScheduleSynchronous(copyDeviceDataCallback, 
        &currentData, HD_MIN_SCHEDULER_PRIORITY);

    memcpy(&prevData, &currentData, sizeof(DeviceData));    

    printHelp();

    /* Run the main loop until the gimbal button is held. */
    while (1)
    {
        /* Perform a synchronous call to copy the most current device state.
           This synchronous scheduler call ensures that the device state
           is obtained in a thread-safe manner. */
        hdScheduleSynchronous(copyDeviceDataCallback,
                              &currentData,
                              HD_MIN_SCHEDULER_PRIORITY);

        /* If the user depresses the gimbal button, display the current 
           location information. */
        if (currentData.m_buttonState && !prevData.m_buttonState)
        {           
            fprintf(stdout, "Current position: (%g, %g, %g)\n", 
                currentData.m_devicePosition[0], 
                currentData.m_devicePosition[1], 
                currentData.m_devicePosition[2]);

            int base_deg = currentData.m_jointAngles[0] * 180 / M_PI + 90;
            int shoulder_deg = currentData.m_jointAngles[1] * 180 / M_PI + 90;
            int elbow_deg = currentData.m_jointAngles[2] * 180 / M_PI;
            int wrist_rot_deg = currentData.m_gimbalAngles[0] * 180 / M_PI + 90;
            int wrist_vert_deg = currentData.m_gimbalAngles[1] * 180 / M_PI + 90;
            int grip_deg = currentData.m_gimbalAngles[2] * 180 / M_PI + 90;
            grip_deg = (grip_deg - 0) * (73 - 10) / (360 - 0) + 10;

            fprintf(stdout, "Current Joint Angles: (%i, %i, %i) \n",
                base_deg,
                shoulder_deg,
                elbow_deg);

            fprintf(stdout, "Current Gimbal Angles: (%i, %i, %i) \n",
                wrist_vert_deg,
                wrist_rot_deg,
                grip_deg);

            CURL* curl = curl_easy_init();
            if (curl) {
                char getParams[100];
                sprintf(getParams, "192.168.3.14:4200/?command=10,%d,%d,%d,%d,%d,%d",
                    base_deg, shoulder_deg, elbow_deg, wrist_rot_deg, wrist_vert_deg, grip_deg);
                CURLcode res;
                curl_easy_setopt(curl, CURLOPT_URL, getParams);
                fprintf(stdout, "URL: %s", getParams);
                res = curl_easy_perform(curl);
                curl_easy_cleanup(curl);
            }

        }
        
        /* Check if an error occurred. */
        if (HD_DEVICE_ERROR(currentData.m_error))
        {
            hduPrintError(stderr, &currentData.m_error, "Device error detected");

            if (hduIsSchedulerError(&currentData.m_error))
            {
                /* Quit, since communication with the device was disrupted. */
                fprintf(stderr, "\nPress any key to quit.\n");
                getch();                
                break;
            }
        }

        /* Store off the current data for the next loop. */
        memcpy(&prevData, &currentData, sizeof(DeviceData));    
    }
}

/*******************************************************************************
 Main function.
 Sets up the device, runs main application loop, cleans up when finished.
*******************************************************************************/
int main(int argc, char* argv[])
{
    HDSchedulerHandle hUpdateHandle = 0;
    HDErrorInfo error;

    /* Initialize the device, must be done before attempting to call any hd 
       functions. */
    HHD hHD = hdInitDevice(HD_DEFAULT_DEVICE);
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to initialize the device");
        fprintf(stderr, "\nPress any key to quit.\n");
        getch();
        return -1;           
    }

    /* Schedule the main scheduler callback that updates the device state. */
    hUpdateHandle = hdScheduleAsynchronous(
        updateDeviceCallback, 0, HD_MAX_SCHEDULER_PRIORITY);

    /* Start the servo loop scheduler. */
    hdStartScheduler();
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to start the scheduler");
        fprintf(stderr, "\nPress any key to quit.\n");
        getch();
        return -1;           
    }
    
    /* Run the application loop. */
    mainLoop();

    /* For cleanup, unschedule callbacks and stop the servo loop. */
    hdStopScheduler();
    hdUnschedule(hUpdateHandle);
    hdDisableDevice(hHD);

    return 0;
}

/******************************************************************************/
