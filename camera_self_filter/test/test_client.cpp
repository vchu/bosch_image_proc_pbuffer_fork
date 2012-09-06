/*
 * test_client.cpp
 *
 *  Created on: Nov 23, 2010
 *      Author: christian
 */



#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "cv_bridge/CvBridge.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "camera_self_filter/mask.h"





int main(int argc, char **argv) {
	ros::init(argc, argv, "robot_self_filter_test");

	ros::NodeHandle nh;
	ros::ServiceClient svc = nh.serviceClient<camera_self_filter::mask>("self_mask");

	sensor_msgs::CvBridge bridge_mask;


	cvNamedWindow("alpha",1);

	while (nh.ok()){

		camera_self_filter::mask servicecall;
		servicecall.request.header.frame_id = "narrow_stereo_optical_frame";
		servicecall.request.header.stamp = ros::Time::now();
		svc.call(servicecall);

		sensor_msgs::Image mask = servicecall.response.mask_image;

		printf("width %d height %d", mask.width, mask.height);

//		for (int y=0; y<mask.height; ++y){
//			for (int x=0; x< mask.width; ++x){
//				printf("%d ", mask.data[y*mask.width + x]);
//			}
//			printf ("  >>>>endl line %d<<<<<\n", y);
//		}
		sensor_msgs::ImageConstPtr maskptr = boost::make_shared<sensor_msgs::Image>(boost::ref(servicecall.response.mask_image));
//		cv::Ptr<IplImage> iplimagemask = bridge_mask.imgMsgToCv(boost::make_shared<sensor_msgs::Image>(mask), "passthrough");
		IplImage* iplimagemask = bridge_mask.imgMsgToCv(maskptr, "passthrough");

		cvShowImage("alpha", iplimagemask);


		cvWaitKey();





	}


}
