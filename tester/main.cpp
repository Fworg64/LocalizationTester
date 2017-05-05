#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <Eigen/Dense>
#include <fstream>

extern "C" {
#include "apriltag.h"
#include "tag36h11.h"
}

using namespace cv;

Eigen::Matrix4d getRelativeTransform(double tag_size,double tag_p[][2], double fx, double fy, double px, double py);

int main(int argc, char** argv)
{
	double tagsize;
	int delay;
	if (argc != 3)
	{
		std::cout <<"Help:: please supply the tag size measured from edge of black border to edge of black border and arguement for waitkey in ms like this: "<<std::endl<<"main .18 30"<<std::endl;
		return -1;
	}
	else
	{
		tagsize = atof(argv[1]);
		delay = atoi(argv[2]);
	}

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

    std::cout << "Warming up camera(2sec), press \"a\" to stop"<< std::endl;

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

	Eigen::Matrix4d myT;
	double t1=0;
	double t2=0;
	double dt=0;
	unsigned int counter=0;

	std::ofstream outfile;
	outfile.open("outfile.csv");

	bool record=false;
	std::stringstream ss;
	string theame;

    while(1)
    {
	t1 = t2;
	t2 = (double)getTickCount();
	dt = (t2-t1)/getTickFrequency();
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

			//if (det->id == 0) //populate position of player from tag id
			{
				std::cout << "found tag "<<det->id<<" with error: " << det->hamming <<std::endl;
				std::cout << "reading tag pos as x: "<< det->c[0] <<" y: "<<det->c[1] <<std::endl;
				//"tag size is size between the outer black edges" -Ed
				//"I'll bet its in meters..." -Austin
				myT = getRelativeTransform(tagsize, det->p, fx, fy, px, py);
				//std::cout <<"Full TF:" <<std::endl<<myT <<std::endl;
				std::cout << "X: " <<myT(0, 3)<<std::endl;
				std::cout << "Y: " <<myT(1, 3)<<std::endl;
				std::cout << "Z: " <<myT(2, 3)<<std::endl;
				std::cout <<"dt: " <<dt<<std::endl;
				record = true;
				
			}
		}

		//record //Rober Reich, "inequality for all"
		if (record)
		{
			ss.str(std::string());
			ss <<"zimg"<<counter++<<".jpg";
			theame = ss.str();
			imwrite(theame, img3);
			//write to csv file here
			outfile << theame <<", "<<dt<<", "<<myT(0, 3)<<", "<<myT(1, 3)<<", "<<myT(2, 3)<<std::endl;
			record = false;
		}

        apriltag_detections_destroy(detections); //not sure if neccesary
        //if(waitKey(3000) >= 0) break;
        if ((char)waitKey(delay) ==97) break; //neccesary, break on "a" press
    }
	
	outfile.close();    
    //prevent memory leaks!
    apriltag_detector_destroy(td);
    tag36h11_destroy(tf);
    return 0;
}

Eigen::Matrix4d getRelativeTransform(double tag_size,double tag_p[][2], double fx, double fy, double px, double py)
{
   std::vector<cv::Point3f> objPts;
   std::vector<cv::Point2f> imgPts;
   double s = tag_size/2.;
   objPts.push_back(cv::Point3f(-s,-s, 0));
   objPts.push_back(cv::Point3f( s,-s, 0));
   objPts.push_back(cv::Point3f( s, s, 0));
   objPts.push_back(cv::Point3f(-s, s, 0));
 /*
   std::pair<float, float> p1 = p[0];
   std::pair<float, float> p2 = p[1];
   std::pair<float, float> p3 = p[2];
   std::pair<float, float> p4 = p[3];

   imgPts.push_back(cv::Point2f(p1.first, p1.second));
   imgPts.push_back(cv::Point2f(p2.first, p2.second));
   imgPts.push_back(cv::Point2f(p3.first, p3.second));
   imgPts.push_back(cv::Point2f(p4.first, p4.second));
*/
   imgPts.push_back(cv::Point2f(tag_p[0][0], tag_p[0][1]));
   imgPts.push_back(cv::Point2f(tag_p[1][0], tag_p[1][1]));
   imgPts.push_back(cv::Point2f(tag_p[2][0], tag_p[2][1]));
   imgPts.push_back(cv::Point2f(tag_p[3][0], tag_p[3][1]));
 
   cv::Mat rvec, tvec;
   cv::Matx33f cameraMatrix(
                            fx, 0, px,
                            0, fy, py,
                            0,  0,  1);
   cv::Vec4f distParam(0,0,0,0); // all 0?
   cv::solvePnP(objPts, imgPts, cameraMatrix, distParam, rvec, tvec);
   cv::Matx33d r;
   cv::Rodrigues(rvec, r);
   Eigen::Matrix3d wRo;
   wRo << r(0,0), r(0,1), r(0,2), r(1,0), r(1,1), r(1,2), r(2,0), r(2,1), r(2,2);
 
   Eigen::Matrix4d T; 
   T.topLeftCorner(3,3) = wRo;
   T.col(3).head(3) << tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2);
   T.row(3) << 0,0,0,1;
 
   return T;
}
