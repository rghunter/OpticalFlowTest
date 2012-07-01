//============================================================================
// Name        : OpticalFlow.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C, Ansi-style
//============================================================================

/*
 * mainFile.cpp
 *
 *  Created on: Jun 25, 2012
 *      Author: rghunter
 */

#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

#define APP_NAME "Lucas"
#define OUTPUT_FILE_NAME "output.m4v"
#define FOURCC ('D','I','V','X')
#define MAX_CORNERS 200
#define WINDOW_SIZE 20
#define HISTORY 2


int main(int argc, char **argv)
{

	VideoCapture video;
	VideoWriter output;

	bool isFileInput = false;

	char *output_file_name;
	output_file_name = OUTPUT_FILE_NAME;

	if(argc > 1)
	{
		char *input_file = argv[1];
		isFileInput = true;
		video = VideoCapture(input_file);
		if (argc == 2){
			output_file_name = argv[2];
	  	}else{
			cout << "Using default file name" << endl;
		}
	}else{
		video = VideoCapture(0);
	}

	if(!video.isOpened())
	{
		exit(EXIT_FAILURE);
	}

	Mat cameraMatrix, distCoeffs;

	FileStorage fs("calib.xml",FileStorage::READ);
	if(!fs.isOpened())
	{
		cout << "Could not open calibration file." << endl;
		return 0;
	}
	fs["Camera_Matrix"] >> cameraMatrix;
	fs["Distortion_Coefficients"] >> distCoeffs;

	cout << "Camera Matrix: " << cameraMatrix << endl;
	cout << "Distortion Coefficients: " << distCoeffs << endl;

	TermCriteria subPixCriteria;
	subPixCriteria.epsilon = 0.3;
	subPixCriteria.maxCount = 20;
	subPixCriteria.type = (CV_TERMCRIT_ITER | CV_TERMCRIT_EPS);
	Size subPixWindowSize(WINDOW_SIZE,WINDOW_SIZE);

	fs.release();

	Size video_frame_size;
	video_frame_size.width = video.get(CV_CAP_PROP_FRAME_WIDTH);
	video_frame_size.height = video.get(CV_CAP_PROP_FRAME_HEIGHT);


	if(isFileInput){
		double video_fps = video.get(CV_CAP_PROP_FPS);
		long video_frame_count = video.get(CV_CAP_PROP_FRAME_COUNT);
		output = VideoWriter("output.m4v",FOURCC,video_fps,video_frame_size,true);
	}

	Mat frame_corrected(video_frame_size,CV_8U,3);
	Mat frame_raw(video_frame_size,CV_8U,3);
	Mat frame_corrected_gray(video_frame_size,CV_8U,3);

	Mat eigen_image(video_frame_size,CV_8U,1);
	Mat temp_image(video_frame_size,CV_8U,1);

	int corner_count = MAX_CORNERS;

	deque<Mat> frame_buffer;
	frame_buffer.clear();
	Mat corners[2];
	Mat status;
	Mat error;


	while(video.grab())
	{
		video >> frame_corrected;
		if(!frame_corrected.empty())
		{
		//	undistort(frame_raw,frame_corrected,cameraMatrix,distCoeffs);
			cvtColor(frame_corrected,frame_corrected_gray, CV_RGB2GRAY);

			frame_buffer.push_front(frame_corrected_gray.clone());

			goodFeaturesToTrack(frame_buffer[0],corners[0],corner_count,
					0.01, //quality
					10.0); //min seperation

			cornerSubPix(frame_buffer[0],corners[0],subPixWindowSize,Size(-1,-1),subPixCriteria);

			if((int)frame_buffer.size() > HISTORY){
				frame_buffer.pop_back();
			}else{
				continue;
			}

			calcOpticalFlowPyrLK(frame_buffer.front(),frame_buffer.back(),corners[0],corners[1],status, error);

			Mat vectors = corners[0] - corners[1];
			float vel = 0;
			float x = 0;
			float y = 0;
			int len = 0;

			for(MatConstIterator_<Point2f> velocity_vector = vectors.begin<Point2f>(); velocity_vector != vectors.end<Point2f>(); ++velocity_vector)
			{
				vel += sqrt(pow((*velocity_vector).x,2)+pow((*velocity_vector).y,2));
				x += (*velocity_vector).x / vel;
				y += (*velocity_vector).y / vel;
				len++;
			}

			vel = vel / (float)len;
			y = y / (float)len;
			x = x / (float)len;

			Point start(video_frame_size.width/2,video_frame_size.height/2);
			line(frame_corrected,start,Point(video_frame_size.width/2+(x*vel*10.0),video_frame_size.height/2+(y*vel*10.0)),Scalar(255,0,0),3);

#if 0
			cout << "Status: " << status << endl;
			cout << "Error: " << error << endl;
#endif

			imshow("FLOW!!",frame_corrected);
			if(isFileInput){
				output.write(frame_corrected);
			}
		}
		waitKey(50);
	}

	output.release();
	video.release();

	return 0;

}


