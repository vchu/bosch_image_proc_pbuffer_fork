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
 * robotMeshModel.cpp
 *
 *  Created on: Oct 7, 2010
 *      Author: Christian Bersch
 */


#include <GL/glew.h>
#include <GL/glu.h>
// #include <GL/freeglut.h>
#include "camera_self_filter/robotMeshModel.h"
#include "LinearMath/btTransform.h"




RobotMeshModel::RobotMeshModel():nh_priv_("~"){

  //Load robot description from parameter server
  std::string robot_desc_string;
  if(!nh_.getParam("robot_description", robot_desc_string)){
    ROS_ERROR("Could not get urdf from param server");

  }


  if (!urdf_.initString(robot_desc_string)){
    ROS_ERROR("Failed to parse urdf");

  }
  modelframe_= "/torso_lift_link";

  nh_priv_.param<std::string>("robot_description_package_path", description_path, "..");
  ROS_INFO("package_path %s", description_path.c_str());
  nh_priv_.param<std::string>("camera_topic", camera_topic_, "/wide_stereo/right" );
  nh_priv_.param<std::string>("camera_info_topic", camera_info_topic_, "/wide_stereo/right/camera_info" );



  //Load robot mesh for each link
  std::vector<boost::shared_ptr<urdf::Link> > links ;
  urdf_.getLinks(links);
  for (int i=0; i< links.size(); i++){
    if (links[i]->visual.get() == NULL) continue;
    if (links[i]->visual->geometry.get() == NULL) continue;
    if (links[i]->visual->geometry->type == urdf::Geometry::MESH){

      //todo: this should really be done by resource retriever
      boost::shared_ptr<urdf::Mesh> mesh = boost::dynamic_pointer_cast<urdf::Mesh> (links[i]->visual->geometry);
      std::string filename (mesh->filename);

      if (filename.substr(filename.size() - 4 , 4) != ".stl" && filename.substr(filename.size() - 4 , 4) != ".dae") continue;
      if (filename.substr(filename.size() - 4 , 4) == ".dae")
        filename.replace(filename.size() - 4 , 4, ".stl");
      ROS_INFO("adding link %d %s",i,links[i]->name.c_str());
      filename.erase(0,25);
      filename = description_path + filename;

      boost::shared_ptr<CMeshO> mesh_ptr(new CMeshO);

      if(vcg::tri::io::ImporterSTL<CMeshO>::Open(*mesh_ptr,filename.c_str())){
        ROS_ERROR("could not load mesh %s", filename.c_str());
        continue;
      }

      links_with_meshes.push_back(links[i]);
      meshes[links[i]->name] = mesh_ptr;

      tf::Vector3 origin(links[i]->visual->origin.position.x, links[i]->visual->origin.position.y, links[i]->visual->origin.position.z);
      tf::Quaternion rotation(links[i]->visual->origin.rotation.x, links[i]->visual->origin.rotation.y, links[i]->visual->origin.rotation.z, links[i]->visual->origin.rotation.w);

      offsets_[links[i]->name] = tf::Transform(rotation, origin);





    }

  }


  initRobot();

  //get camera intinsics

  ROS_INFO("waiting for %s", camera_info_topic_.c_str());
  cam_info_ = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camera_info_topic_);
  cameraframe_ = cam_info_->header.frame_id;

  ROS_INFO("%s: robot model initialization done!", ros::this_node::getName().c_str());

}

RobotMeshModel::~RobotMeshModel(){

}

void RobotMeshModel::initRobot(){
  std::vector<boost::shared_ptr<urdf::Link> > links ;

  for (int i=0; i< links_with_meshes.size(); i++){

    //draw
    // update bounding box
    boost::shared_ptr<CMeshO> mesh_ptr = meshes[links_with_meshes[i]->name];
    vcg::tri::UpdateBounding<CMeshO>::Box(*mesh_ptr);

    // update Normals
    vcg::tri::UpdateNormals<CMeshO>::PerVertexNormalizedPerFace(*mesh_ptr);
    vcg::tri::UpdateNormals<CMeshO>::PerFaceNormalized(*mesh_ptr);

    // Initialize the opengl wrapper
    boost::shared_ptr<vcg::GlTrimesh<CMeshO> > wrapper_ptr (new vcg::GlTrimesh<CMeshO>);
    wrapper_ptr->m = mesh_ptr.get();
    wrapper_ptr->Update();

    mesh_wrappers[links_with_meshes[i]->name] = wrapper_ptr;


  }





};


void RobotMeshModel::updateRobotLinks(const ros::Time time_stamp){

  // get the current configuration of the robot links
  if (current_time_stamp_ != time_stamp){
    current_time_stamp_ = time_stamp;
    tf::StampedTransform tf;
    for (int i=0; i < links_with_meshes.size(); ++i){
      if (!tf_.waitForTransform(links_with_meshes[i]->name, modelframe_, current_time_stamp_, ros::Duration(0.5))){
        ROS_ERROR("could not get transform from %s to %s", links_with_meshes[i]->name.c_str(),modelframe_.c_str() );
        continue;
      }
      tf_.lookupTransform( modelframe_, links_with_meshes[i]->name, current_time_stamp_, tf);

      tf  *= offsets_[links_with_meshes[i]->name];;
      robotLinks_[links_with_meshes[i]->name] = tf;

    }

  }
}


void RobotMeshModel::paintRobot(){


  // draw the configuration of the robot links as  specified in robotLinks_
  float d=1.0f; ///mesh.bbox.Diag();
  vcg::glScale(d);
  glDisable(GL_CULL_FACE);
  glMatrixMode(GL_MODELVIEW);

  for (int i=0; i < links_with_meshes.size(); ++i){
    boost::shared_ptr<vcg::GlTrimesh<CMeshO> > wrapper_ptr  = mesh_wrappers[links_with_meshes[i]->name];

    btScalar glTf[16];
    robotLinks_[links_with_meshes[i]->name].getOpenGLMatrix(glTf);



    glPushMatrix();

    glMultMatrixd((GLdouble*)glTf);

    wrapper_ptr->Draw<vcg::GLW::DMSmooth,   vcg::GLW::CMNone,vcg::GLW::TMNone> ();

    glPopMatrix();
  }




}

void RobotMeshModel::setCameraInfo(sensor_msgs::CameraInfoConstPtr cam_info){
  cam_info_ = cam_info;
}


void RobotMeshModel::setCamera(){

  //calc field of view for OpenGL Camera;

  double FulstrumWidth, FulstrumHeight, left, right, top, bottom;

  double near_clip = 0.1;
  double farclip = 100;


  double cx= cam_info_->P[2];
  double cy = cam_info_->P[6];

  double fx = cam_info_->P[0];
  double fy = cam_info_->P[5];


  FulstrumWidth = near_clip * cam_info_->width / fx;
  FulstrumHeight = near_clip * cam_info_->height / fy;


  left = FulstrumWidth * (- cx / cam_info_->width);
  right = FulstrumWidth * ( 1.0 - cx / cam_info_->width);
  top = FulstrumHeight * ( cy / cam_info_->height);
  bottom = FulstrumHeight * ( -1.0 + cy / cam_info_->height);


  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();

  glFrustum(left,right, bottom, top, near_clip, farclip);
  glMatrixMode(GL_MODELVIEW);

}


bool RobotMeshModel::setCameraPose(){


  //set camera pose relative to robot links
  if (!tf_.waitForTransform(modelframe_, cameraframe_, current_time_stamp_, ros::Duration(0.5))){
    ROS_ERROR("setting cam pose: could not get transform from %s to %s", modelframe_.c_str(),cameraframe_.c_str() );
    return false;
  }
  tf_.lookupTransform(modelframe_, cameraframe_, current_time_stamp_, cameraPose);


  //get offset for stereo
  double tx = -1.0 * (cam_info_->P[3] / cam_info_->P[0]);
  tf::Vector3 origin = cameraPose.getOrigin() + cameraPose.getBasis() * tf::Vector3(tx, 0.0, 0.0);

  tf::Vector3 look_at_position = origin + cameraPose.getBasis().getColumn(2);
  glLoadIdentity();
  tf::Vector3 up = -cameraPose.getBasis().getColumn(1);

  gluLookAt(origin.getX() , origin.getY() , origin.getZ() ,
            look_at_position.getX() , look_at_position.getY(), look_at_position.getZ(),
            up.getX(), up.getY(), up.getZ());
  return true;

}


