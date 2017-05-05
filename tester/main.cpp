#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <stdio.h>

extern "C" {
#include "apriltag.h"
#include "tag36h11.h"
}

using namespace cv;

int main(int argc, char** argv)
{
    VideoCapture cap(0);
    if(!cap.isOpened()) return -1; //check for success
    //char* imageName = argv[1];
    //Mat img =imread(imageName, 1);
	Mat img(480, 640, CV_8UC3, Scalar(69,42,200));
	Mat img2(480, 640, CV_8UC3, Scalar(69,42,200));
	Mat img3(480, 640, CV_8UC3, Scalar(69,42,200));

    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_family_t *tf = tag36h11_create();

    tf->black_border = 1; //from example

    apriltag_detector_add_family(td, tf);

    td->quad_decimate = 1.0;
    td->quad_sigma = 0.0;
    td->refine_edges = 1;
    td->refine_decode = 0;
    td->refine_pose = 0;

    zarray_t *detections;

    std::cout << "Warming up camera(2sec)"<< std::endl;

    waitKey(2000); //let camera warm up

    std::cout << "CAMERA HOT" <<std::endl;

	namedWindow("COMPUTER VISION");

	//read camera calib data
	FileStorage fs("out_camera_data.xml", FileStorage::READ);
    Mat cameraMatrix1;
    Mat distortionCoefficient1;

    fs["Camera_Matrix"] >>cameraMatrix1;
	/*[fx,  0, px;
	    0, fy, py;
		0,   0, 1;]*/
    fs["Distortion_Coefficients"] >> distortionCoefficient1;
	double fx = cameraMatrix1.at<double>(0);
	double fy = cameraMatrix1.at<double>(4);
	double px = cameraMatrix1.at<double>(2);
	double py = cameraMatrix1.at<double>(5);
	std::cout << "Read camerea properties fx, fy, px, py " <<fx<<", "<<fy<<", "<<px<<", "<<py<<std::endl;


    while(1)
    {
	cap >> img; //c++ is the future
	cvtColor(img, img2, COLOR_BGR2GRAY);
	img3 = img;
	imshow("COMPUTER VISION", img3);
	std::cout <<"FRAaaME"<<std::endl;

	image_u8_t img_header = { .width = img2.cols, //convert opencv to apriltag img
        	.height = img2.rows,
        	.stride = img2.cols,
        	.buf = img2.data
    		};

	detections = apriltag_detector_detect(td, &img_header); //detect april tags

	for (int i = 0; i < zarray_size(detections); i++) { //iterate through detections
        	apriltag_detection_t *det;
        	zarray_get(detections, i, &det); //store dection at adress pointed by det

			if (det->id == 0); //populate position of player from tag id
			{
				std::cout << "found tag "<<det->id<<" with error: " << det->hamming <<std::endl;
				std::cout << "reading tag pos as x: "<< det->c[0] <<" y: "<<det->c[1] <<std::endl;
			}
		}

        apriltag_detections_destroy(detections); //not sure if neccesary
        //if(waitKey(3000) >= 0) break;
        waitKey(30); //neccesary
    }
    
    //prevent memory leaks!
    apriltag_detector_destroy(td);
    tag36h11_destroy(tf);
    return 0;
}
