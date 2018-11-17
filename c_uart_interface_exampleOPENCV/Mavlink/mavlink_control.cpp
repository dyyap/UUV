/****************************************************************************
*
*   Copyright (c) 2014 MAVlink Development Team. All rights reserved.
*   Author: Trent Lukaczyk, <aerialhedgehog@gmail.com>
*           Jaycee Lock,    <jaycee.lock@gmail.com>
*           Lorenz Meier,   <lm@inf.ethz.ch>
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in
*    the documentation and/or other materials provided with the
*    distribution.
* 3. Neither the name PX4 nor the names of its contributors may be
*    used to endorse or promote products derived from this software
*    without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
* OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
****************************************************************************/

/**
 * @file mavlink_control.cpp
 *
 * @brief An example offboard control process via mavlink
 *
 * This process connects an external MAVLink UART device to send an receive data
 *
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 * @author Jaycee Lock,    <jaycee.lock@gmail.com>
 * @author Lorenz Meier,   <lm@inf.ethz.ch>
 *
 */



// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "mavlink_control.h"
#include "stdafx.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sstream>
#include <iostream>
#include "opencv/highgui.h"

// ------------------------------------------------------------------------------
//   TOP
// ------------------------------------------------------------------------------

using namespace cv;
using namespace std;

int positionReturn(int currentPos) {
        return currentPos;
}

int
top (int argc, char **argv)
{

        // --------------------------------------------------------------------------
        //   PARSE THE COMMANDS
        // --------------------------------------------------------------------------

        // Default input arguments
#ifdef __APPLE__
        char *uart_name = (char*)"/dev/tty.usbmodem1";
#else
        char *uart_name = (char*)"/dev/ttyACM0";
#endif
        int baudrate = 57600;

        // do the parse, will throw an int if it fails
        parse_commandline(argc, argv, uart_name, baudrate);




        VideoCapture capWebcam(0);  //use webcam 1 if working with the usb webcam, cam 0 is laptop webcam
        if (capWebcam.isOpened() == false) //  To check if object was associated to webcam successfully
        {
                cout << "error: Webcam connect unsuccessful\n"; // if not then print error message
                // and exit program
        }
        VideoCapture webcam2(1);
        if (webcam2.isOpened() == false) //  To check if object was associated to webcam successfully
        {
                cout << "error: Webcam2 connect unsuccessful\n"; // if not then print error message
                // and exit program
        }
        capWebcam.set(CV_CAP_PROP_FRAME_WIDTH, 600); //sets input capture to a set resolution of 1024x1024
        capWebcam.set(CV_CAP_PROP_FRAME_HEIGHT, 600);
        webcam2.set(CV_CAP_PROP_FRAME_WIDTH, 600); //sets input capture to a set resolution of 1024x1024
        webcam2.set(CV_CAP_PROP_FRAME_HEIGHT, 600);
        Mat imgOriginal;  //original image capture
        Mat imgOrig2;
        Mat hsvImg;          //image to hold the hsv values
        Mat threshImg;      //hsv image we isolate the color from

        ostringstream outputss; //for console clarity
        bool coordsOK = false;

        char charCheckForEscKey = 0; //esc key

        int lowH = 4;//0;//21;                            // Set Hue
        int highH = 20;//7;//30;

        int lowS = 100;//;150;//50;//200;                            // Set Saturation
        int highS = 280;//255;//255;

        int lowV = 150;//145;//102;                            // Set Value
        int highV = 300;//255;//225;

        int iLastX = -1;
        int iLastY = -1;

        vector<vector<Point> > contours; //holds contour positions
        vector<Point> largestContour;
        vector<vector<Point>> largestContourHolder(1);

        Scalar color = Scalar(0, 0, 255);
        // HUE for YELLOW is 21-30.

        //HSV reference table
        //Yellow H: 21-30
        //Pink H: 149-179 S: 110 - 228 V: 185,255



        //Anything in this if handles the x and y positions
        //     if (dArea > 10000) {
        //         int posX = dM10 / dArea;
        //         int posY = dM01 / dArea;
        //
        //         //Put the little circdArea > 10000le on the center
        //         if (iLastX >= 0 && iLastY >= 0 && posX >= 0 && posY >= 0) {
        //             circle(imgOriginal, cvPoint(posX, posY), 20, Scalar(0, 255, 0));
        //             coordsOK = true;
        //
        //             if (iLastX >= 0 && posX < 50) {
        //                 //hard left
        //                 outputss << "X: " << posX << " Y: " << posY << "| pos 1" << "|dArea: " << dArea << endl;
        //                 cout << outputss.str() << endl;
        //                 positionReturn(1);
        //             }
        //             else if (posX >= 50 && posX < 200) {
        //                 //soft left
        //                 outputss << "X: " << posX << " Y: " << posY << "| pos 2" << "|dArea: " << dArea << endl;
        //                 cout << outputss.str() << endl;
        //                 positionReturn(2);
        //             }
        //             else if (posX >= 200 && posX < 400) {
        //                 //straight on
        //                 outputss << "X: " << posX << " Y: " << posY << "| pos 3" << "|dArea: " << dArea << endl;
        //                 cout << outputss.str() << endl;
        //                 positionReturn(3);
        //             }
        //             else if (posX >= 400 && posX < 550) {
        //                 //soft right
        //                 outputss << "X: " << posX << " Y: " << posY << "| pos 4" << "|dArea: " << dArea << endl;
        //                 cout << outputss.str() << endl;
        //                 positionReturn(4);
        //             }
        //             else if (posX >= 550 && posX < 600) {
        //                 //hard right
        //                 outputss << "X: " << posX << " Y: " << posY << "| pos 5" << "|dArea: " << dArea << endl;
        //                 cout << outputss.str() << endl;
        //                 positionReturn(5);
        //             }
        //             else {
        //                 //rotate in place
        //                 outputss << "X: " << posX << " Y: " << posY << "| pos 6" << "|dArea: " << dArea << endl;
        //                 cout << outputss.str() << endl;
        //                 positionReturn(6);
        //             }
        //
        //         }
        //
        //         iLastX = posX;
        //         iLastY = posY;
        //
        //         /*Legend for positioning
        //             1: Make a hard left. Within 0-50
        //             2: Make a soft left. Within 50-300
        //             3: Go straight on. Within 300-600
        //             4: Make a soft right. Within 600-850
        //             5: Make a hard right. Within 850-900
        //             6: Don't move forward, but rotate in place till the rest of the algo takes over.
        //                This is for when the object is out of sight
        //         */
        // bool coordsOK = false;
        //
        //     }
        //
        //     // declare windows
        //     namedWindow("imgOriginal", CV_WINDOW_AUTOSIZE);
        //     namedWindow("threshImg", CV_WINDOW_AUTOSIZE);
        //
        //     /* Create trackbars in "threshImg" window to adjust according to object and environment.*/
        //
        //     /*
        //     createTrackbar("LowH", "threshImg", &lowH, 179);    //Hue (0 - 179)
        //     createTrackbar("HighH", "threshImg", &highH, 179);
        //
        //     createTrackbar("LowS", "threshImg", &lowS, 255);    //Saturation (0 - 255)
        //     createTrackbar("HighS", "threshImg", &highS, 255);
        //
        //     createTrackbar("LowV", "threshImg", &lowV, 255);    //Value (0 - 255)
        //     createTrackbar("HighV", "threshImg", &highV, 255);
        //     */
        //
        //
        //     imshow("imgOriginal", imgOriginal);                    // show windows
        //     imshow("threshImg", threshImg);
        //
        //     charCheckForEscKey = waitKey(1);                    // delay and get key press
        // }


        // --------------------------------------------------------------------------
        //   PORT and THREAD STARTUP
        // --------------------------------------------------------------------------

        /*
         * Instantiate a serial port object
         *
         * This object handles the opening and closing of the offboard computer's
         * serial port over which it will communicate to an autopilot.  It has
         * methods to read and write a mavlink_message_t object.  To help with read
         * and write in the context of pthreading, it gaurds port operations with a
         * pthread mutex lock.
         *
         */
        Serial_Port serial_port(uart_name, baudrate);


        /*
         * Instantiate an autopilot interface object
         *
         * This starts two threads for read and write over MAVlink. The read thread
         * listens for any MAVlink message and pushes it to the current_messages
         * attribute.  The write thread at the moment only streams a position target
         * in the local NED frame (mavlink_set_position_target_local_ned_t), which
         * is changed by using the method update_setpoint().  Sending these messages
         * are only half the requirement to get response from the autopilot, a signal
         * to enter "offboard_control" mode is sent by using the enable_offboard_control()
         * method.  Signal the exit of this mode with disable_offboard_control().  It's
         * important that one way or another this program signals offboard mode exit,
         * otherwise the vehicle will go into failsafe.
         *
         */
        Autopilot_Interface autopilot_interface(&serial_port);

        /*
         * Setup interrupt signal handler
         *
         * Responds to early exits signaled with Ctrl-C.  The handler will command
         * to exit offboard mode if required, and close threads and the port.
         * The handler in this example needs references to the above objects.
         *
         */
        serial_port_quit         = &serial_port;
        autopilot_interface_quit = &autopilot_interface;
        signal(SIGINT,quit_handler);


        // 2018 TESTING FOR SERVO
        printf("Testing servo\n");

        /*
         * Start the port and autopilot_interface
         * This is where the port is opened, and read and write threads are started.
         */
        serial_port.start();
        autopilot_interface.start();
        // autopilot_interface.write_set_servo(7,0);
        // sleep(2);
        // autopilot_interface.write_set_servo(7,1100);
        // sleep(2);
        //autopilot_interface.write_set_servo(7,1900);


        // --------------------------------------------------------------------------
        //   RUN COMMANDS
        // --------------------------------------------------------------------------

        /*
         * Now we can implement the algorithm we want on top of the autopilot interface
         */
        //commands(autopilot_interface);
         //autopilot_interface.write_set_servo(7,1900);
        //autopilot_interface.write_set_servo(7,1900);
        //autopilot_interface.write_set_servo(7,1100);
        //sleep(5);
        //autopilot_interface.write_set_servo(7,1900);

          //autopilot_interface.write_set_servo(7,1900);

        // autopilot_interface.write_set_servo(7,0);
        // autopilot_interface.write_set_servo(7,1900);
        // sleep(1);
        // autopilot_interface.write_set_servo(7,0);
        // sleep(1);
        // autopilot_interface.write_set_servo(7,1100);
        // sleep(1);
        // autopilot_interface.write_set_servo(7,0);
        // sleep(1);
        autopilot_interface.write_set_servo(9,1500);
        autopilot_interface.write_set_servo(10,1500);
        sleep(5);
        autopilot_interface.arm_pixhawk();
        sleep(1);
        autopilot_interface.arm_pixhawk();
        int grab = 0;
        int found = 0;
        int foundObj = 0;
        //Go Forwards 3 thrust
        autopilot_interface.write_set_servo(9,1710);  //1585
        autopilot_interface.write_set_servo(10,1710); //1585
        sleep(18);
        while (charCheckForEscKey != 27 && capWebcam.isOpened() && grab != 30) {                // until the Esc is pressed or webcam connection is lost
            bool blnFrameReadSuccessfully = capWebcam.read(imgOriginal);        // get next frame

            if (!blnFrameReadSuccessfully || imgOriginal.empty()) {                // if frame read unsuccessfully
                cout << "error: frame can't read \n";                        // print error message
                break;                                                    // jump out of loop
            }

            cvtColor(imgOriginal, hsvImg, CV_BGR2HSV);                        // Convert Original Image to HSV Thresh Image

            inRange(hsvImg, Scalar(lowH, lowS, lowV), Scalar(highH, highS, highV), threshImg);

            GaussianBlur(threshImg, threshImg, Size(3, 3), 0);            //Blur Effect
            dilate(threshImg, threshImg, 0);                                // Dilate Filter Effect
            erode(threshImg, threshImg, 0);                                    // Erode Filter Effect

            dilate(threshImg, threshImg, 0);
            erode(threshImg, threshImg, 0);

            Moments oMoments = moments(threshImg);

            //TODO: Fuck with the two parameters (3 and 4)
            findContours(threshImg, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

            //draws the contour
            for (int i = 0; i < contours.size(); i++) {
                //drawContours(imgOriginal, contours, i, color, 3, 8);
            }

            //TODO: Calculate an area of said contour
            //TODO: Maybe look at fitEllipse? Probably good with contour

            double dM01 = oMoments.m01;
            double dM10 = oMoments.m10;
            double dArea = oMoments.m00;

            for (unsigned int i = 1; i < contours.size(); i++)
                    {
                        //std::cout << "Contour number: " << i << " Area: " << contourArea(contours[i]) << std::endl;
                        if (contourArea(contours[i]) > contourArea(contours[i - 1])) {
                            largestContour = contours[i];
                        }
                    }
            // cout << contourArea(largestContour) << endl;
            largestContourHolder[0] = largestContour;
            //drawContours(imgOriginal, largestContourHolder, 0, color, 3, 8);




            //Anything in this if handles the x and y positions
            if (dArea > 10000) {
                int posX = dM10 / dArea;
                int posY = dM01 / dArea;

                //Put the little circle on the center
                if (iLastX >= 0 && iLastY >= 0 && posX >= 0 && posY >= 0) {
                    circle(imgOriginal, cvPoint(posX, posY), 20, Scalar(0, 255, 0));
                    coordsOK = true;
                    if (posX >= 100 && posX < 500 && dArea >= 13500000 ) {
                        //straight on

                        //autopilot_interface.write_set_servo(7,0);
                        //autopilot_interface.write_set_servo(9,1500);
                        //autopilot_interface.write_set_servo(10,1500);
                        //sleep(1);
                        if(grab == 0)
                        {
                          foundObj = dArea;
                          autopilot_interface.write_set_servo(9,1500); // Stop motors
                          autopilot_interface.write_set_servo(10,1500);
                          autopilot_interface.write_set_servo(7,1100); // Grab object
                        }
                        autopilot_interface.write_set_servo(9,1420); // Back up a bit
                        autopilot_interface.write_set_servo(10,1420);
                        outputss << "X: " << posX << " Y: " << posY << "| pos 3" << "|dArea: " << dArea << endl;
                        cout << outputss.str() << endl;
                        if(dArea >=foundObj) // Confirm if object is in claw
                        {
                          grab++;
                        }
                        else // Object not grabbed, open claw again
                        {
                          grab = 0;
                          autopilot_interface.write_set_servo(7,1700);
                        }

                    }
                    else if(dArea < foundObj && grab > 0)
                    {
                      autopilot_interface.write_set_servo(7,1700);
                      grab = 0;
                    }
                    if (posX >= 200 && posX < 400 && grab == 0) {
                        //straight on
                        outputss << "X: " << posX << " Y: " << posY << "| pos 3" << "|dArea: " << dArea << endl;
                        cout << outputss.str() << endl;
                        autopilot_interface.write_set_servo(9,1690);  //1585
                        autopilot_interface.write_set_servo(10,1690); //1585
                        positionReturn(3);
                    }
                    // 1500 (zero thrust) 1585(0.5 Thrust) 1610(1 Thrust) 1645 (1.5 Thrust) 1665 (2 Thrust) 1690 (2.5 Thrust) 1710 (3 Thrust)
                    else if (iLastX >= 0 && posX < 50) {
                        //hard left
                        outputss << "X: " << posX << " Y: " << posY << "| pos 1" << "|dArea: " << dArea << endl;
                        cout << outputss.str() << endl;
                        autopilot_interface.write_set_servo(9,1710);  //1645
                        autopilot_interface.write_set_servo(10,1500);
                        positionReturn(1);
                    }
                    // 1500 (zero thrust) 1585(0.5 Thrust) 1610(1 Thrust) 1645 (1.5 Thrust) 1665 (2 Thrust) 1690 (2.5 Thrust) 1710 (3 Thrust)
                    else if (posX >= 50 && posX < 200) {
                        //soft left
                        outputss << "X: " << posX << " Y: " << posY << "| pos 2" << "|dArea: " << dArea << endl;
                        cout << outputss.str() << endl;
                        autopilot_interface.write_set_servo(9,1690);  //1585
                        autopilot_interface.write_set_servo(10,1500);
                        positionReturn(2);
                    }
                    // 1500 (zero thrust) 1585(0.5 Thrust) 1610(1 Thrust) 1645 (1.5 Thrust) 1665 (2 Thrust) 1690 (2.5 Thrust) 1710 (3 Thrust)
                    else if (posX >= 400 && posX < 550) {
                        //soft right
                        outputss << "X: " << posX << " Y: " << posY << "| pos 4" << "|dArea: " << dArea << endl;
                        cout << outputss.str() << endl;
                        autopilot_interface.write_set_servo(10,1690);  //1585
                        autopilot_interface.write_set_servo(9,1500);
                        positionReturn(4);
                    }
                    // 1500 (zero thrust) 1585(0.5 Thrust) 1610(1 Thrust) 1645 (1.5 Thrust) 1665 (2 Thrust) 1690 (2.5 Thrust) 1710 (3 Thrust)
                    else if (posX >= 550 && posX < 600) {
                        //hard right
                        outputss << "X: " << posX << " Y: " << posY << "| pos 5" << "|dArea: " << dArea << endl;
                        cout << outputss.str() << endl;
                        autopilot_interface.write_set_servo(10,1710); //1645
                        autopilot_interface.write_set_servo(9,1500);
                        positionReturn(5);
                    }
                    else {
                        //rotate in place
                        outputss << "X: " << posX << " Y: " << posY << "| pos 6" << "|dArea: " << dArea << endl;
                        cout << outputss.str() << endl;
                        positionReturn(6);
                    }

                }


                iLastX = posX;
                iLastY = posY;

                /*Legend for positioning
                    1: Make a hard left. Within 0-50
                    2: Make a soft left. Within 50-300
                    3: Go straight on. Within 300-600
                    4: Make a soft right. Within 600-850
                    5: Make a hard right. Within 850-900
                    6: Don't move forward, but rotate in place till the rest of the algo takes over.
                       This is for when the object is out of sight
                */iLastX = posX;
                iLastY = posY;


            }
            else
            {
              autopilot_interface.write_set_servo(10,1500);
              autopilot_interface.write_set_servo(9,1500);

            }

            // declare windows
            namedWindow("imgOriginal", CV_WINDOW_AUTOSIZE);
            namedWindow("threshImg", CV_WINDOW_AUTOSIZE);

            /* Create trackbars in "threshImg" window to adjust according to object and environment.*/

            /*
            createTrackbar("LowH", "threshImg", &lowH, 179);    //Hue (0 - 179)
            createTrackbar("HighH", "threshImg", &highH, 179);

            createTrackbar("LowS", "threshImg", &lowS, 255);    //Saturation (0 - 255)
            createTrackbar("HighS", "threshImg", &highS, 255);

            createTrackbar("LowV", "threshImg", &lowV, 255);    //Value (0 - 255)
            createTrackbar("HighV", "threshImg", &highV, 255);
            */


            imshow("imgOriginal", imgOriginal);                    // show windows
            imshow("threshImg", threshImg);

            charCheckForEscKey = waitKey(1);                    // delay and get key press
        }
        int homeReach = 0;
        //Go Backwards 3 thrust
        autopilot_interface.write_set_servo(9,1275);  //1585
        autopilot_interface.write_set_servo(10,1275); //1585
        sleep(6);
        while (charCheckForEscKey != 27 && webcam2.isOpened() && homeReach == 0) {                // until the Esc is pressed or webcam connection is lost
            bool blnFrameReadSuccessfully = webcam2.read(imgOriginal);        // get next frame

            if (!blnFrameReadSuccessfully || imgOriginal.empty()) {                // if frame read unsuccessfully
                cout << "error: frame can't read \n";                        // print error message
                break;                                                    // jump out of loop
            }

            cvtColor(imgOriginal, hsvImg, CV_BGR2HSV);                        // Convert Original Image to HSV Thresh Image

            inRange(hsvImg, Scalar(lowH, lowS, lowV), Scalar(highH, highS, highV), threshImg);

            GaussianBlur(threshImg, threshImg, Size(3, 3), 0);            //Blur Effect
            dilate(threshImg, threshImg, 0);                                // Dilate Filter Effect
            erode(threshImg, threshImg, 0);                                    // Erode Filter Effect

            dilate(threshImg, threshImg, 0);
            erode(threshImg, threshImg, 0);

            Moments oMoments = moments(threshImg);

            //TODO: Fuck with the two parameters (3 and 4)
            findContours(threshImg, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

            //draws the contour
            for (int i = 0; i < contours.size(); i++) {
                //drawContours(imgOriginal, contours, i, color, 3, 8);
            }

            //TODO: Calculate an area of said contour
            //TODO: Maybe look at fitEllipse? Probably good with contour

            double dM01 = oMoments.m01;
            double dM10 = oMoments.m10;
            double dArea = oMoments.m00;

            for (unsigned int i = 1; i < contours.size(); i++)
                    {
                        //std::cout << "Contour number: " << i << " Area: " << contourArea(contours[i]) << std::endl;
                        if (contourArea(contours[i]) > contourArea(contours[i - 1])) {
                            largestContour = contours[i];
                        }
                    }
            // cout << contourArea(largestContour) << endl;
            largestContourHolder[0] = largestContour;
            //drawContours(imgOriginal, largestContourHolder, 0, color, 3, 8);




            //Anything in this if handles the x and y positions
            if (dArea > 10000) {
                int posX = dM10 / dArea;
                int posY = dM01 / dArea;

                //Put the little circle on the center
                if (iLastX >= 0 && iLastY >= 0 && posX >= 0 && posY >= 0) {
                    circle(imgOriginal, cvPoint(posX, posY), 20, Scalar(0, 255, 0));
                    coordsOK = true;

                    if (posX >= 100 && posX < 500 && dArea >= 13500000 ) {
                        //straight on
                        autopilot_interface.write_set_servo(7,0);
                        autopilot_interface.write_set_servo(9,1500);
                        autopilot_interface.write_set_servo(10,1500);
                        //sleep(1);
                        autopilot_interface.write_set_servo(7,1800);
                        outputss << "X: " << posX << " Y: " << posY << "| pos 3" << "|dArea: " << dArea << endl;
                        cout << outputss.str() << endl;
                        homeReach = 1;

                    }
                     if (posX >= 200 && posX < 400 && homeReach == 0) {
                        //straight on
                        outputss << "X: " << posX << " Y: " << posY << "| pos 3" << "|dArea: " << dArea << endl;
                        cout << outputss.str() << endl;
                        autopilot_interface.write_set_servo(9,1290);  //1420
                        autopilot_interface.write_set_servo(10,1290); //1420
                        positionReturn(3);
                    }
                    else if (iLastX >= 0 && posX < 50) {
                        //hard left
                        outputss << "X: " << posX << " Y: " << posY << "| pos 1" << "|dArea: " << dArea << endl;
                        cout << outputss.str() << endl;
                        autopilot_interface.write_set_servo(9,1500);
                        autopilot_interface.write_set_servo(10,1275);  //1380
                        positionReturn(1);
                    }
                    else if (posX >= 50 && posX < 200) {
                        //soft left
                        outputss << "X: " << posX << " Y: " << posY << "| pos 2" << "|dArea: " << dArea << endl;
                        cout << outputss.str() << endl;
                        autopilot_interface.write_set_servo(9,1500);
                        autopilot_interface.write_set_servo(10,1290); //1420
                        positionReturn(2);
                    }

                    else if (posX >= 400 && posX < 550) {
                        //soft right
                        outputss << "X: " << posX << " Y: " << posY << "| pos 4" << "|dArea: " << dArea << endl;
                        cout << outputss.str() << endl;
                        autopilot_interface.write_set_servo(10,1500);
                        autopilot_interface.write_set_servo(9,1290);  //1420
                        positionReturn(4);
                    }
                    else if (posX >= 550 && posX < 600) {
                        //hard right
                        outputss << "X: " << posX << " Y: " << posY << "| pos 5" << "|dArea: " << dArea << endl;
                        cout << outputss.str() << endl;
                        autopilot_interface.write_set_servo(10,1500);
                        autopilot_interface.write_set_servo(9,1275);  //1380
                        positionReturn(5);
                    }
                    else {
                        //rotate in place
                        outputss << "X: " << posX << " Y: " << posY << "| pos 6" << "|dArea: " << dArea << endl;
                        cout << outputss.str() << endl;
                        positionReturn(6);
                    }

                }


                iLastX = posX;
                iLastY = posY;

                /*Legend for positioning
                    1: Make a hard left. Within 0-50
                    2: Make a soft left. Within 50-300
                    3: Go straight on. Within 300-600
                    4: Make a soft right. Within 600-850
                    5: Make a hard right. Within 850-900
                    6: Don't move forward, but rotate in place till the rest of the algo takes over.
                       This is for when the object is out of sight
                */iLastX = posX;
                iLastY = posY;


            }
            else
            {
              autopilot_interface.write_set_servo(10,1500);
              autopilot_interface.write_set_servo(9,1500);

            }

            // declare windows
            namedWindow("imgOriginal", CV_WINDOW_AUTOSIZE);
            namedWindow("threshImg", CV_WINDOW_AUTOSIZE);

            /* Create trackbars in "threshImg" window to adjust according to object and environment.*/

            /*
            createTrackbar("LowH", "threshImg", &lowH, 179);    //Hue (0 - 179)
            createTrackbar("HighH", "threshImg", &highH, 179);

            createTrackbar("LowS", "threshImg", &lowS, 255);    //Saturation (0 - 255)
            createTrackbar("HighS", "threshImg", &highS, 255);

            createTrackbar("LowV", "threshImg", &lowV, 255);    //Value (0 - 255)
            createTrackbar("HighV", "threshImg", &highV, 255);
            */


            imshow("imgOriginal", imgOriginal);                    // show windows
            imshow("threshImg", threshImg);

            charCheckForEscKey = waitKey(1);                    // delay and get key press
        }
        autopilot_interface.write_set_servo(9,1500); // Stop motors
        autopilot_interface.write_set_servo(10,1500);
        sleep(3);
        autopilot_interface.disarm_pixhawk();
        sleep(1);
        autopilot_interface.disarm_pixhawk();
        serial_port.stop();


        // int pwm = 1900;
        // int servoNum = 7;
        // autopilot_interface.write_set_servo(servoNum,0);
        // sleep(1);
        // autopilot_interface.write_set_servo(servoNum,pwm);
        // sleep(1);
        // autopilot_interface.write_set_servo(servoNum,0);
        // sleep(1);



        // --------------------------------------------------------------------------
        //   THREAD and PORT SHUTDOWN
        // --------------------------------------------------------------------------

        /*
         * Now that we are done we can stop the threads and close the port
         */



        // --------------------------------------------------------------------------
        //   DONE
        // ------serial_port.start();--------------------------------------------------------------------

        // woot!
        return 0;

}


// ------------------------------------------------------------------------------
//   COMMANDS
// ------------------------------------------------------------------------------

void
commands(Autopilot_Interface &api)
{

        // --------------------------------------------------------------------------
        //   START OFFBOARD MODE
        // --------------------------------------------------------------------------

        api.enable_offboard_control();
        usleep(100); // give some time to let it sink in

        // now the autopilot is accepting setpoint commands


        // --------------------------------------------------------------------------
        //   SEND OFFBOARD COMMANDS
        // --------------------------------------------------------------------------
        printf("SEND OFFBOARD COMMANDS\n");

        // initialize command data strtuctures
        mavlink_set_position_target_local_ned_t sp;
        mavlink_set_position_target_local_ned_t ip = api.initial_position;

        // autopilot_interface.h provides some helper functions to build the command


        // Example 1 - Set Velocity
//	set_velocity( -1.0       , // [m/s]
//				  -1.0       , // [m/s]
//				   0.0       , // [m/s]
//				   sp        );

        // Example 2 - Set Position
        set_position( ip.x - 5.0, // [m]
                      ip.y - 5.0, // [m]
                      ip.z, // [m]
                      sp         );


        // Example 1.2 - Append Yaw Command
        set_yaw( ip.yaw, // [rad]
                 sp     );

        // SEND THE COMMAND
        api.update_setpoint(sp);
        // NOW pixhawk will try to move

        // Wait for 8 seconds, check position
        for (int i=0; i < 8; i++)
        {
                mavlink_local_position_ned_t pos = api.current_messages.local_position_ned;
                printf("%i CURRENT POSITION XYZ = [ % .4f , % .4f , % .4f ] \n", i, pos.x, pos.y, pos.z);
                sleep(1);
        }

        printf("\n");


        // --------------------------------------------------------------------------
        //   STOP OFFBOARD MODE
        // --------------------------------------------------------------------------

        api.disable_offboard_control();

        // now pixhawk isn't listening to setpoint commands


        // --------------------------------------------------------------------------
        //   GET A MESSAGE
        // --------------------------------------------------------------------------
        printf("READ SOME MESSAGES \n");

        // copy current messages
        Mavlink_Messages messages = api.current_messages;

        // local position in ned frame
        mavlink_local_position_ned_t pos = messages.local_position_ned;
        printf("Got message LOCAL_POSITION_NED (spec: https://pixhawk.ethz.ch/mavlink/#LOCAL_POSITION_NED)\n");
        printf("    pos  (NED):  %f %f %f (m)\n", pos.x, pos.y, pos.z );

        // hires imu
        mavlink_highres_imu_t imu = messages.highres_imu;
        printf("Got message HIGHRES_IMU (spec: https://pixhawk.ethz.ch/mavlink/#HIGHRES_IMU)\n");
        printf("    ap time:     %llu \n", imu.time_usec);
        printf("    acc  (NED):  % f % f % f (m/s^2)\n", imu.xacc, imu.yacc, imu.zacc );
        printf("    gyro (NED):  % f % f % f (rad/s)\n", imu.xgyro, imu.ygyro, imu.zgyro);
        printf("    mag  (NED):  % f % f % f (Ga)\n", imu.xmag, imu.ymag, imu.zmag );
        printf("    baro:        %f (mBar) \n", imu.abs_pressure);
        printf("    altitude:    %f (m) \n", imu.pressure_alt);
        printf("    temperature: %f C \n", imu.temperature );

        printf("\n");


        // --------------------------------------------------------------------------
        //   END OF COMMANDS
        // --------------------------------------------------------------------------

        return;

}


// ------------------------------------------------------------------------------
//   Parse Command Line
// ------------------------------------------------------------------------------
// throws EXIT_FAILURE if could not open the port
void
parse_commandline(int argc, char **argv, char *&uart_name, int &baudrate)
{

        // string for command line usage
        const char *commandline_usage = "usage: mavlink_serial -d <devicename> -b <baudrate>";

        // Read input arguments
        for (int i = 1; i < argc; i++) { // argv[0] is "mavlink"

                // Help
                if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
                        printf("%s\n",commandline_usage);
                        throw EXIT_FAILURE;
                }

                // UART device ID
                if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) {
                        if (argc > i + 1) {
                                uart_name = argv[i + 1];

                        } else {
                                printf("%s\n",commandline_usage);
                                throw EXIT_FAILURE;
                        }
                }

                // Baud rate
                if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baud") == 0) {
                        if (argc > i + 1) {
                                baudrate = atoi(argv[i + 1]);

                        } else {
                                printf("%s\n",commandline_usage);
                                throw EXIT_FAILURE;
                        }
                }

        }
        // end: for each input argument

        // Done!
        return;
}


// ------------------------------------------------------------------------------
//   Quit Signal Handler
// ------------------------------------------------------------------------------
// this function is called when you press Ctrl-C
void
quit_handler( int sig )
{
        printf("\n");
        printf("TERMINATING AT USER REQUEST\n");
        printf("\n");

        // autopilot interface
        try {
                autopilot_interface_quit->handle_quit(sig);
        }
        catch (int error) {}

        // serial port
        try {
                serial_port_quit->handle_quit(sig);
        }
        catch (int error) {}

        // end program here
        exit(0);

}


// ------------------------------------------------------------------------------
//   Main
// ------------------------------------------------------------------------------
int
main(int argc, char **argv)
{
        // This program uses throw, wrap one big try/catch here
        try
        {
                int result = top(argc,argv);
                return result;
        }

        catch ( int error )
        {
                fprintf(stderr,"mavlink_control threw exception %i \n", error);
                return error;
        }

}
