/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Robert Bosch LLC.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Robert Bosch nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

/*
 * camera_self_filter.cpp
 *
 *  Created on: Oct 6, 2010
 *      Author: Christian Bersch
 */

#include <cstdio>

#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "cv_bridge/CvBridge.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "image_transport/image_transport.h"
#include <image_geometry/pinhole_camera_model.h>

// #define USE_GLUT_RENDERING 1
#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>
#include <GL/glext.h>

#ifdef USE_GLUT_RENDERING
#include <GL/freeglut.h>
#endif // USE_GLUT_RENDERING

#include "camera_self_filter/robotMeshModel.h"

#ifndef USE_GLUT_RENDERING
#include <GL/glx.h>
#define GLX_CONTEXT_MAJOR_VERSION_ARB       0x2091
#define GLX_CONTEXT_MINOR_VERSION_ARB       0x2092
typedef GLXContext (*glXCreateContextAttribsARBProc)(Display*, GLXFBConfig, GLXContext, Bool, const int*);

#endif // USE_GLUT_RENDERING

RobotMeshModel* robmod=NULL;
IplImage* ipl_maskBGRA = NULL;
IplImage* ipl_maskBW = NULL;

bool publish_mask;
bool inverted;
image_transport::Publisher mask_publisher;
image_transport::Publisher image_publisher;
sensor_msgs::CvBridge bridge;

#ifndef USE_GLUT_RENDERING
GLXPbuffer pbuffer;
Display* glx_display = NULL;
#endif // USE_GLUT_RENDERING
void initializeGL ()
{
    glClearColor(0, 0, 0, 0);
    // glEnable(GL_LIGHTING);
    // glEnable(GL_LIGHT0);
    glEnable(GL_NORMALIZE);
    glEnable(GL_COLOR_MATERIAL);
    glEnable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);
    glShadeModel(GL_SMOOTH);
    glTexParameterf (GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameterf (GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
                     GL_LINEAR_MIPMAP_LINEAR);
}


//function that draws the robot at current state
void displayFunc()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    robmod->setCamera();
    robmod->setCameraPose();
    robmod->paintRobot();
    glReadPixels(0, 0, robmod->cam_info_->width, robmod->cam_info_->height, GL_BGRA, GL_UNSIGNED_BYTE, ipl_maskBGRA->imageData);

#ifdef USE_GLUT_RENDERING
    glutSwapBuffers();
#else
    glXSwapBuffers(glx_display, pbuffer);
#endif
}

void calc_and_publish_BWMask(const ros::Time time_stamp, const std::string frame_id)
{
    robmod->updateRobotLinks(time_stamp);
#ifdef USE_GLUT_RENDERING
    glutPostRedisplay();
    glutMainLoopEvent();
#else
    displayFunc();
#endif // USE_GLUT_RENDERING
    cvCvtColor(ipl_maskBGRA, ipl_maskBW, CV_BGRA2GRAY);
    cvFlip(ipl_maskBW);
    if (inverted)
        cvNot(ipl_maskBW,ipl_maskBW);


    if(publish_mask){
        sensor_msgs::ImagePtr img_msg = sensor_msgs::CvBridge::cvToImgMsg(ipl_maskBW);
        img_msg->header.frame_id = frame_id;
        img_msg->header.stamp = time_stamp;
        mask_publisher.publish(img_msg);
    }

}
void alpha_image_cb(const sensor_msgs::ImageConstPtr& msg_ptr){


    calc_and_publish_BWMask(msg_ptr->header.stamp, msg_ptr->header.frame_id);
    IplImage* cam_image = bridge.imgMsgToCv(msg_ptr);
    IplImage* cam_alpha_image = cvCreateImage(cvGetSize(cam_image), IPL_DEPTH_8U, 4);

    //b
    cvSetImageCOI(cam_alpha_image, 1);
    cvSetImageCOI(cam_image, 1);
    cvCopy(cam_image, cam_alpha_image);

    //g
    cvSetImageCOI(cam_alpha_image, 2);
    cvSetImageCOI(cam_image, 2);
    cvCopy(cam_image, cam_alpha_image);

    //r
    cvSetImageCOI(cam_alpha_image, 3);
    cvSetImageCOI(cam_image, 3);
    cvCopy(cam_image, cam_alpha_image);

    //alpha
    cvSetImageCOI(cam_alpha_image, 4);
    cvCopy(ipl_maskBW, cam_alpha_image);
    cvSetImageCOI(cam_alpha_image, 0);


    sensor_msgs::ImagePtr img_msg = sensor_msgs::CvBridge::cvToImgMsg(cam_alpha_image);
    img_msg->header = msg_ptr->header;
    image_publisher.publish(img_msg);
    cvReleaseImage(&cam_alpha_image);


}


void mask_cb(const sensor_msgs::CameraInfoConstPtr& msg_ptr){
    calc_and_publish_BWMask(msg_ptr->header.stamp, msg_ptr->header.frame_id);
}

// For camera topic callback rather than the camera info topic
void test_mask_cb(const sensor_msgs::ImageConstPtr& msg_ptr){
    //ROS_INFO_STREAM(msg_ptr->header.stamp);
    //ROS_INFO_STREAM(ros::Time::now());
    if ((ros::Time::now()-msg_ptr->header.stamp) > ros::Duration(10.0)){
        ROS_INFO_STREAM("SKIPPING FIRST BECAUSE OF TIME");
    } else {
        calc_and_publish_BWMask(msg_ptr->header.stamp, msg_ptr->header.frame_id);
    }
}

int main(int argc, char **argv)
{
#ifdef USE_GLUT_RENDERING
    glutInit(&argc, argv);
#else
    const char *displayName = NULL;
    glx_display = XOpenDisplay(displayName);
    if (!glx_display)
    {
        ROS_ERROR_STREAM("No glx display!");
        return 1;
    }
    static int visual_attribs[] = {
        GLX_X_RENDERABLE    , True,
        GLX_DRAWABLE_TYPE   , GLX_WINDOW_BIT,
        GLX_RENDER_TYPE     , GLX_RGBA_BIT,
        GLX_X_VISUAL_TYPE   , GLX_TRUE_COLOR,
        GLX_RED_SIZE        , 8,
        GLX_GREEN_SIZE      , 8,
        GLX_BLUE_SIZE       , 8,
        GLX_ALPHA_SIZE      , 8,
        GLX_DEPTH_SIZE      , 24,
        GLX_STENCIL_SIZE    , 8,
        GLX_DOUBLEBUFFER    , True,
        None
    };
    int num_fb_configs = 0;
    GLXFBConfig* fbConfigs = glXChooseFBConfig(
        glx_display, DefaultScreen(glx_display), visual_attribs, &num_fb_configs);
    if (!num_fb_configs)
    {
        ROS_ERROR_STREAM("No fb configs!");
        return 1;
    }
    else
    {
        ROS_DEBUG_STREAM("Found " << num_fb_configs << " configs");
    }
#endif // USE_GLUT_RENDERING

    ros::init(argc, argv, "robot_self_filter");
    //create selfilter service
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    //set parameters
    bool publish_alpha_image;

    std::string camera_topic;
    std::string camera_info_topic;

    nh_priv.param("publish_mask",publish_mask, true);
    nh_priv.param("publish_alpha",publish_alpha_image, true);
    nh_priv.param("inverted",inverted, false);


    nh_priv.param<std::string>("camera_topic", camera_topic, "/wide_stereo/right/image_rect_color");
    nh_priv.param<std::string>("camera_info_topic", camera_info_topic, "/wide_stereo/right/camera_info");

    ROS_INFO("camera topic %s", camera_topic.c_str());
    ROS_INFO("camera info %s", camera_info_topic.c_str());

    //create robot model
    ROS_DEBUG_STREAM("Getting robmod");
    robmod = new RobotMeshModel;
    ROS_DEBUG_STREAM("Got robmod");

    //initialize glut
#ifdef USE_GLUT_RENDERING
    glutInitWindowSize (robmod->cam_info_->width, robmod->cam_info_->height);
    glutInitDisplayMode (GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
    glutCreateWindow ("dummy");
    glutHideWindow();
#else // USE_GLUT_RENDERING

    ROS_INFO_STREAM("Creating glX pbuffer");
    int pbufferAttribs[] = {
        GLX_PBUFFER_WIDTH , robmod->cam_info_->width,
        GLX_PBUFFER_HEIGHT, robmod->cam_info_->height,
        None
    };
    pbuffer = glXCreatePbuffer(glx_display, fbConfigs[0], pbufferAttribs);
    if (!pbuffer)
    {
        ROS_ERROR_STREAM("Failed to create pbuffer!");
        return 1;
    }

    glXCreateContextAttribsARBProc glXCreateContextAttribsARB = 0;
    glXCreateContextAttribsARB = (glXCreateContextAttribsARBProc)
            glXGetProcAddressARB((const GLubyte *) "glXCreateContextAttribsARB");
    GLXContext gl_context = 0;

    int context_attribs[] = {
        GLX_CONTEXT_MAJOR_VERSION_ARB, 3,
        GLX_CONTEXT_MINOR_VERSION_ARB, 0,
        None
    };

    ROS_DEBUG_STREAM("Creating OpenGL context");
    gl_context = glXCreateContextAttribsARB(glx_display, fbConfigs[0], 0,
                                            True, context_attribs);

    // Sync to ensure any errors generated are processed.
    XSync(glx_display, False);

    if (!gl_context)
    {
        ROS_ERROR_STREAM("Failed to create GL 3.0 context");
        return -1;
    }
    ROS_DEBUG_STREAM("Got openGl context");

    // clean up
    XFree(fbConfigs);
    XSync(glx_display, False);

    // Make context current
    glXMakeCurrent(glx_display, pbuffer, gl_context);
    XSync(glx_display, False);
#endif // USE_GLUT_RENDERING

    ROS_DEBUG_STREAM("Initializng GL");
    initializeGL();
    ROS_DEBUG_STREAM("Initialized GL");

#ifdef USE_GLUT_RENDERING
    // Register glut callbacks:
    glutDisplayFunc (displayFunc);
#endif //USE_GLUT_RENDERING

    ipl_maskBGRA = cvCreateImage(cvSize(robmod->cam_info_->width,
                                        robmod->cam_info_->height), IPL_DEPTH_8U, 4);
    ipl_maskBW = cvCreateImage(cvSize(robmod->cam_info_->width,
                                      robmod->cam_info_->height), IPL_DEPTH_8U, 1);

    image_transport::ImageTransport it(nh);

    if (publish_mask)
        mask_publisher = it.advertise(camera_topic + "/self_mask", 10);

    if (publish_alpha_image)
        image_publisher = it.advertise(camera_topic +
                                       "/image_rect_color_alpha_masked", 10);

    image_transport::Subscriber image_sub;
    ros::Subscriber camerea_info_sub;
    if (publish_alpha_image)
    {
        image_sub = it.subscribe(camera_topic, 10, alpha_image_cb);
    }
    else
    {
        // Change to listen to camera topic to time stamps because gazebo
        // simulation doesn't update the camera info topic time stamp
        //camerea_info_sub = nh.subscribe(camera_info_topic, 10, mask_cb);
        camerea_info_sub = nh.subscribe(camera_topic, 10, test_mask_cb);
    }



    ros::spin();


    //clean up
    cvReleaseImage(&ipl_maskBGRA);
    cvReleaseImage(&ipl_maskBW);
    delete robmod;

#ifndef USE_GLUT_RENDERING
    ROS_DEBUG_STREAM("Cleaning up GLX");
    glXMakeCurrent(glx_display, None, NULL);
    glXDestroyPbuffer(glx_display, pbuffer);
    glXDestroyContext(glx_display, gl_context);
    XCloseDisplay(glx_display);
#endif // USE_GLUT_RENDERING
    return 0;
}

