///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2010, Jason Mora Saragih, all rights reserved.
//
// This file is part of FaceTracker.
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions are met:
//
//     * The software is provided under the terms of this licence stricly for
//       academic, non-commercial, not-for-profit purposes.
//     * Redistributions of source code must retain the above copyright notice, 
//       this list of conditions (licence) and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright 
//       notice, this list of conditions (licence) and the following disclaimer 
//       in the documentation and/or other materials provided with the 
//       distribution.
//     * The name of the author may not be used to endorse or promote products 
//       derived from this software without specific prior written permission.
//     * As this software depends on other libraries, the user must adhere to 
//       and keep in place any licencing terms of those libraries.
//     * Any publications arising from the use of this software, including but
//       not limited to academic journal and conference publications, technical
//       reports and manuals, must cite the following work:
//
//       J. M. Saragih, S. Lucey, and J. F. Cohn. Face Alignment through 
//       Subspace Constrained Mean-Shifts. International Conference of Computer 
//       Vision (ICCV), September, 2009.
//
// THIS SOFTWARE IS PROVIDED BY THE AUTHOR "AS IS" AND ANY EXPRESS OR IMPLIED 
// WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO 
// EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF 
// THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
///////////////////////////////////////////////////////////////////////////////
#include "Tracker.h"
#include <opencv/highgui.h>
//#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
using namespace cv;
using namespace std;
//=============================================================================

//method for creating display image

void createDisplayImage(Mat &frame, Mat &displayImage, double scale_for_display, int color_display) {
    Mat gray;
    //display color image
    if (color_display > 0) {
        //resize to color frame for display 
        if (scale_for_display == 1)displayImage = frame.clone();
        else resize(frame, displayImage, Size(0, 0), scale_for_display, scale_for_display, INTER_LINEAR);

    } else //display gray scale image
    {
        //copy data to display frame
        cv::cvtColor(frame, gray, CV_BGR2GRAY);
        if (scale_for_display != 1) {
            resize(gray, gray, Size(0, 0), scale_for_display, scale_for_display, INTER_LINEAR);
        }

        //resize to frame for display 

        //create three channel grayscale image for display purpose
        vector<Mat> channels;
        channels.push_back(gray);
        channels.push_back(gray);
        channels.push_back(gray);
        //three channel gray scale image
        merge(channels, displayImage);
        gray.release();
    }
}

bool detectOverlap(Rect &rect1, Rect &rect2) {
    double MIN_OVERLAPREQUIRED = 0.7;
    double overlappingElements = max(0, min(rect1.x + rect1.width, rect2.x + rect2.width) - max(rect1.x, rect2.x)) *
            max(0, min(rect1.y + rect1.height, rect2.y + rect2.height) - max(rect1.y, rect2.y));

    if (overlappingElements / double(rect2.height * rect2.width) >= MIN_OVERLAPREQUIRED || overlappingElements / double(rect1.height * rect1.width) >= MIN_OVERLAPREQUIRED) {
        return true;
    } else
        return false;
}

void detectAndDisplay(Mat &processFrameGrey, Mat &displayFrameMultiChannel, CascadeClassifier &face_cascade, double scale_for_display, double scale_for_process, Rect shapeRect, bool onlyHerme) {
    RNG rng(12345);
    std::vector<Rect> faces;

    //equalize grey process frame
    //equalizeHist(processFrameGrey, processFrameGrey);

    //-- Detect faces
    face_cascade.detectMultiScale(processFrameGrey, faces, 1.1, 3);

    Rect displayFace;
    string facePositions = "";
    string trackedFacePosotion = "";
    double maxRadius = -1;
    int radius; 
    //for each detected face
    for (size_t i = 0; i < faces.size(); i++) {
        displayFace.x = double(faces[i].x) * (scale_for_display / scale_for_process);
        displayFace.y = double(faces[i].y) * (scale_for_display / scale_for_process);
        displayFace.height = double(faces[i].height) * (scale_for_display / scale_for_process);
        displayFace.width = double(faces[i].width) * (scale_for_display / scale_for_process);

        if (onlyHerme || !detectOverlap(displayFace, shapeRect)) {
            Point center(displayFace.x + displayFace.width * 0.5, displayFace.y + displayFace.height * 0.5);

            Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
            //draw circle in the display frame
            ellipse(displayFrameMultiChannel, center, Size(displayFace.width * 0.5, displayFace.height * 0.5), 0, 0, 360, color, 2, 8, 0);

            radius = int((faces[i].width + faces[i].height) * 0.25 / scale_for_process);
            if (radius > maxRadius) {
                //print coordinates back to the original coordinate space of the input image 
                facePositions = std::to_string(int((faces[i].x + faces[i].width * 0.5) / scale_for_process))
                        + " " + std::to_string(int((faces[i].y + faces[i].height * 0.5) / scale_for_process))
                        + " " + std::to_string(radius)
                        + " " + facePositions;
                maxRadius = radius;
            } else {
                //print coordinates back to the original coordinate space of the input image 
                facePositions += facePositions.length()>0?" ":"" + 
                        std::to_string(int((faces[i].x + faces[i].width * 0.5) / scale_for_process))
                        + " " + std::to_string(int((faces[i].y + faces[i].height * 0.5) / scale_for_process))
                        + " " + std::to_string(int((faces[i].width + faces[i].height) * 0.25 / scale_for_process));
            }

        } else {
            //tracked face
            trackedFacePosotion = std::to_string(int((faces[i].x + faces[i].width * 0.5) / scale_for_process))
                    + " " + std::to_string(int((faces[i].y + faces[i].height * 0.5) / scale_for_process))
                    + " " + std::to_string(int((faces[i].width + faces[i].height) * 0.25 / scale_for_process));
        }
    }
    if (onlyHerme)
        cout << facePositions << flush << endl;
    else
        cout << trackedFacePosotion + " " + facePositions << flush << endl;

    fflush(stdout);
    //display face and eyes
    // imshow("Herme", displayFrameMultiChannel); 
}

Rect Draw(cv::Mat &image, cv::Mat &shape, cv::Mat &con, cv::Mat &tri, cv::Mat &visi, double imgaeChangeRatio) {
    int i, n = shape.rows / 2;
    cv::Point p1, p2;
    cv::Scalar c;

    vector<Point> shapePoints;

    //draw triangulation
    c = CV_RGB(0, 0, 0);
    for (i = 0; i < tri.rows; i++) {
        if (visi.at<int>(tri.at<int>(i, 0), 0) == 0 ||
                visi.at<int>(tri.at<int>(i, 1), 0) == 0 ||
                visi.at<int>(tri.at<int>(i, 2), 0) == 0)continue;

        p1 = cv::Point(shape.at<double>(tri.at<int>(i, 0), 0),
                shape.at<double>(tri.at<int>(i, 0) + n, 0)) * imgaeChangeRatio;
        p2 = cv::Point(shape.at<double>(tri.at<int>(i, 1), 0),
                shape.at<double>(tri.at<int>(i, 1) + n, 0)) * imgaeChangeRatio;
        ;

        shapePoints.push_back(p1);
        shapePoints.push_back(p2);

        cv::line(image, p1, p2, c);
        p1 = (cv::Point(shape.at<double>(tri.at<int>(i, 0), 0),
                shape.at<double>(tri.at<int>(i, 0) + n, 0))) * imgaeChangeRatio;
        p2 = (cv::Point(shape.at<double>(tri.at<int>(i, 2), 0),
                shape.at<double>(tri.at<int>(i, 2) + n, 0))) * imgaeChangeRatio;

        shapePoints.push_back(p1);
        shapePoints.push_back(p2);

        cv::line(image, p1, p2, c);
        p1 = (cv::Point(shape.at<double>(tri.at<int>(i, 2), 0),
                shape.at<double>(tri.at<int>(i, 2) + n, 0))) * imgaeChangeRatio;
        p2 = (cv::Point(shape.at<double>(tri.at<int>(i, 1), 0),
                shape.at<double>(tri.at<int>(i, 1) + n, 0))) * imgaeChangeRatio;
        shapePoints.push_back(p1);
        shapePoints.push_back(p2);

        cv::line(image, p1, p2, c);
    }
    //draw connections
    c = CV_RGB(0, 0, 255);
    for (i = 0; i < con.cols; i++) {
        if (visi.at<int>(con.at<int>(0, i), 0) == 0 ||
                visi.at<int>(con.at<int>(1, i), 0) == 0)continue;
        p1 = (cv::Point(shape.at<double>(con.at<int>(0, i), 0),
                shape.at<double>(con.at<int>(0, i) + n, 0))) * imgaeChangeRatio;
        p2 = (cv::Point(shape.at<double>(con.at<int>(1, i), 0),
                shape.at<double>(con.at<int>(1, i) + n, 0))) * imgaeChangeRatio;
        cv::line(image, p1, p2, c, 1);
    }
    //draw points
    for (i = 0; i < n; i++) {
        if (visi.at<int>(i, 0) == 0)continue;
        p1 = (cv::Point(shape.at<double>(i, 0), shape.at<double>(i + n, 0))) * imgaeChangeRatio;
        c = CV_RGB(255, 0, 0);
        cv::circle(image, p1, 2, c);
    }
    return boundingRect(shapePoints);
}

//=============================================================================

int parse_cmd(int argc, const char** argv,
        char* ftFile, char* conFile, char* triFile,
        bool &fcheck, double &scale, int &fpd) {
    int i;
    fcheck = false;
    //scale = 1;
    fpd = -1;
    /*
    for (i = 1; i < argc; i++) {
        if ((std::strcmp(argv[i], "-?") == 0) ||
                (std::strcmp(argv[i], "--help") == 0)) {
            std::cout << "track_face:- Written by Jason Saragih 2010" << std::endl
                    << "Performs automatic face tracking" << std::endl << std::endl
                    << "#" << std::endl
                    << "# usage: ./face_tracker [options]" << std::endl
                    << "#" << std::endl << std::endl
                    << "Arguments:" << std::endl
                    << "-m <string> -> Tracker model (default: model/face2.tracker)"
                    << std::endl
                    << "-c <string> -> Connectivity (default: model/face.con)"
                    << std::endl
                    << "-t <string> -> Triangulation (default: model/face.tri)"
                    << std::endl
                    << "-s <double> -> Image scaling (default: 1)" << std::endl
                    << "-d <int>    -> Frames/detections (default: -1)" << std::endl
                    << "--check     -> Check for failure" << std::endl;
            return -1;
        }
    }

    for (i = 1; i < argc; i++) {
        if (std::strcmp(argv[i], "--check") == 0) {
            fcheck = true;
            break;
        }
    }    
    
    if (i >= argc)fcheck = false;
    for (i = 1; i < argc; i++) {
        if (std::strcmp(argv[i], "-s") == 0) {
            if (argc > i + 1)scale = std::atof(argv[i + 1]);
            else scale = 1;
            break;
        }
    }
    if (i >= argc)scale = 1;
    for (i = 1; i < argc; i++) {
        if (std::strcmp(argv[i], "-d") == 0) {
            if (argc > i + 1)fpd = std::atoi(argv[i + 1]);
            else fpd = -1;
            break;
        }
    }
    */
    if (i >= argc)fpd = -1;
    for (i = 1; i < argc; i++) {
        if (std::strcmp(argv[i], "-m") == 0) {
            if (argc > i + 1)std::strcpy(ftFile, argv[i + 1]);
            else strcpy(ftFile, "model/face2.tracker");
            break;
        }
    }
    if (i >= argc)std::strcpy(ftFile, "model/face2.tracker");
    for (i = 1; i < argc; i++) {
        if (std::strcmp(argv[i], "-c") == 0) {
            if (argc > i + 1)std::strcpy(conFile, argv[i + 1]);
            else strcpy(conFile, "model/face.con");
            break;
        }
    }
    if (i >= argc)std::strcpy(conFile, "model/face.con");
    for (i = 1; i < argc; i++) {
        if (std::strcmp(argv[i], "-t") == 0) {
            if (argc > i + 1)std::strcpy(triFile, argv[i + 1]);
            else strcpy(triFile, "model/face.tri");
            break;
        }
    }
    if (i >= argc)std::strcpy(triFile, "model/face.tri");
    return 0;
}
//=============================================================================

int main(int argc, const char** argv) {
    //parse command line arguments

    //========================================================================================================================== 
    //HERME CODE INSERT
    //make stdout/err non-blocking, so we don't freeze when it's not available

    //make stdout/err non-blocking, so we don't freeze when it's not available
    int flags = fcntl(STDOUT_FILENO, F_GETFL);
    fcntl(STDOUT_FILENO, F_SETFL, flags | O_NONBLOCK);
    flags = fcntl(STDERR_FILENO, F_GETFL);
    fcntl(STDERR_FILENO, F_SETFL, flags | O_NONBLOCK);

    String face_cascade_name;
    double scale_for_process, scale_for_display;
    int color_display, capture_number;

    //cascade variables
    CascadeClassifier face_cascade;

    if (argc < 6) {
        cout << "Usage: " << argv[1] << " face_cascade_file scale_for_process scale_for_display color_display_variables capture_device_number !!" << endl << endl;
        cout << "Example: " << argv[1] << " /opt/opencv/data/haarcascades/haarcascade_frontalface_alt.xml 0.5 1.5 1 1" << endl;
        return -1;
    }

    //read the input parameters
    face_cascade_name = argv[1];
    //cout << face_cascade_name << endl; 
    scale_for_process = atof(argv[2]);
    scale_for_display = atof(argv[3]);
    color_display = int(atof(argv[4]));
    capture_number = int(atof(argv[5]));



    if (!face_cascade.load(face_cascade_name)) {
        printf("--(!)Error loading\n");
        return -1;
    };
    //======================================================================
    char ftFile[256], conFile[256], triFile[256];
    bool fcheck = false;
    int fpd = -1;
    bool show = true;
    double trackerScale = 1;

    if (parse_cmd(argc, argv, ftFile, conFile, triFile, fcheck, trackerScale, fpd) < 0)return 0;

    //set other tracking parameters
    std::vector<int> wSize1(1);
    wSize1[0] = 7;
    std::vector<int> wSize2(3);
    wSize2[0] = 11;
    wSize2[1] = 9;
    wSize2[2] = 7;
    int nIter = 5;
    double clamp = 3, fTol = 0.01;
    FACETRACKER::Tracker model(ftFile);
    cv::Mat tri = FACETRACKER::IO::LoadTri(triFile);
    cv::Mat con = FACETRACKER::IO::LoadCon(conFile);

    //initialize camera and display window
    cv::Mat frame, gray, im, grayViola, frameViola;
    double fps = 0;
    char sss[256];
    std::string text;
    CvCapture* camera = cvCreateCameraCapture(capture_number);
    if (!camera)return -1;
    int64 t1, t0 = cvGetTickCount();
    int fnum = 0;
    cvNamedWindow("Face Tracker", 1);
    std::cout << "Hot keys: " << std::endl
            << "\t ESC - quit" << std::endl
            << "\t d   - Redetect" << std::endl;

    //loop until quit (i.e user presses ESC)
    //==============================================================================================================
    //code fixture to incorporate HERME

    Rect shapeRect;
    Mat displayFrame;

    //=====================================================================================================

    bool failed = true;
    bool onlyHerme = false;
    while (1) {
        //grab image, resize and flip
        IplImage* I = cvQueryFrame(camera);
        if (!I)continue;
        frame = I;
        if (trackerScale == 1)im = frame.clone();
        else cv::resize(frame, im, cv::Size(trackerScale * frame.cols, trackerScale * frame.rows));

        if (scale_for_process == 1)frameViola = frame.clone();
        else cv::resize(frame, frameViola, cv::Size(scale_for_process * frame.cols, scale_for_process * frame.rows));

        //display image
        createDisplayImage(frame, displayFrame, scale_for_display, color_display);

        cv::flip(displayFrame, displayFrame, 1);
        cv::flip(im, im, 1);
        cv::flip(frameViola, frameViola, 1);

        cv::cvtColor(im, gray, CV_BGR2GRAY);
        cv::cvtColor(frameViola, grayViola, CV_BGR2GRAY);

        if (!onlyHerme) {
            //track this image
            std::vector<int> wSize;
            if (failed)wSize = wSize2;
            else wSize = wSize1;
            if (model.Track(gray, wSize, fpd, nIter, clamp, fTol, fcheck) == 0) {
                int idx = model._clm.GetViewIdx();
                failed = false;
                shapeRect = Draw(displayFrame, model._shape, con, tri, model._clm._visi[idx], scale_for_display / trackerScale);

            } else {
                if (show) {
                    cv::Mat R(im, cvRect(0, 0, 150, 50));
                    R = cv::Scalar(0, 0, 255);
                }
                model.FrameReset();
                failed = true;
            }
            //draw framerate on display image 
            if (fnum >= 9) {
                t1 = cvGetTickCount();
                fps = 10.0 / ((double(t1 - t0) / cvGetTickFrequency()) / 1e+6);
                t0 = t1;
                fnum = 0;
            } else fnum += 1;
            if (show) {
                sprintf(sss, "%d frames/sec", (int) round(fps));
                text = sss;
                cv::putText(im, text, cv::Point(10, 20),
                        CV_FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255, 255, 255));
            }
        }
        //show image and check for user input
        detectAndDisplay(grayViola, displayFrame, face_cascade, scale_for_display, scale_for_process, shapeRect, onlyHerme);

        imshow("Face Tracker", displayFrame);
        int c = cvWaitKey(10);
        if (c == 27)break;
        else if (char(c) == 'd')model.FrameReset();
        else if (char(c) == 'h') onlyHerme = !onlyHerme;
    }
    return 0;
}
//=============================================================================