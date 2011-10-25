/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/*
 @mainpage
   Desc: GazeboRosCamera plugin for simulating cameras in Gazebo
   Author: John Hsu
   Date: 24 Sept 2008
   SVN info: $Id: gazebo_ros_camera.cpp 329 2011-05-17 09:39:57Z ricardo $
 @htmlinclude manifest.html
 @b GazeboRosCamera plugin broadcasts ROS Image messages
 */

#include <algorithm>
#include <assert.h>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

#include <gazebo_plugins/gazebo_ros_camera.h>

#include <gazebo/Timer.hh>
#include <gazebo/Sensor.hh>
#include <gazebo/Model.hh>
#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/Simulator.hh>
#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>
#include <gazebo/MonoCameraSensor.hh>

#include "sensor_msgs/Image.h"
#include "sensor_msgs/fill_image.h"

namespace gazebo
{

GZ_REGISTER_DYNAMIC_CONTROLLER("gazebo_ros_camera", GazeboRosCamera);

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosCamera::GazeboRosCamera(Entity *parent)
    : Controller(parent)
{
  this->myParent = dynamic_cast<MonoCameraSensor*>(this->parent);

  if (!this->myParent)
    gzthrow("GazeboRosCamera controller requires a Camera Sensor as its parent");

  Param::Begin(&this->parameters);
  this->robotNamespaceP = new ParamT<std::string>("robotNamespace","/",0);
  this->imageTopicNameP = new ParamT<std::string>("imageTopicName","", 1);
  this->cameraInfoTopicNameP = new ParamT<std::string>("cameraInfoTopicName","", 1);
  this->frameNameP = new ParamT<std::string>("frameName","generic_camera_link", 0);
  // camera parameters 
  this->CxPrimeP = new ParamT<double>("CxPrime",0, 0); // default to 0 for compute on the fly
  this->CxP  = new ParamT<double>("Cx" ,0, 0); // default to 0 for compute on the fly
  this->CyP  = new ParamT<double>("Cy" ,0, 0); // default to 0 for compute on the fly
  this->focal_lengthP  = new ParamT<double>("focal_length" ,0, 0); // == image_width(px) / (2*tan( hfov(radian) /2)), default to 0 for compute on the fly
  this->hack_baselineP  = new ParamT<double>("hackBaseline" ,0, 0); // hack for right stereo camera
  this->distortion_k1P  = new ParamT<double>("distortion_k1" ,0, 0);
  this->distortion_k2P  = new ParamT<double>("distortion_k2" ,0, 0);
  this->distortion_k3P  = new ParamT<double>("distortion_k3" ,0, 0);
  this->distortion_t1P  = new ParamT<double>("distortion_t1" ,0, 0);
  this->distortion_t2P  = new ParamT<double>("distortion_t2" ,0, 0);
  Param::End();

  this->imageConnectCount = 0;
  this->infoConnectCount = 0;

  // set sensor update rate to controller update rate?
  //(dynamic_cast<OgreCamera*>(this->myParent))->SetUpdateRate(this->updateRateP->GetValue());
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosCamera::~GazeboRosCamera()
{
  delete this->robotNamespaceP;
  delete this->rosnode_;
  delete this->imageTopicNameP;
  delete this->cameraInfoTopicNameP;
  delete this->frameNameP;
  delete this->CxPrimeP;
  delete this->CxP;
  delete this->CyP;
  delete this->focal_lengthP;
  delete this->hack_baselineP;
  delete this->distortion_k1P;
  delete this->distortion_k2P;
  delete this->distortion_k3P;
  delete this->distortion_t1P;
  delete this->distortion_t2P;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosCamera::LoadChild(XMLConfigNode *node)
{
  this->robotNamespaceP->Load(node);
  this->robotNamespace = this->robotNamespaceP->GetValue();
  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc,argv,"gazebo",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  }

  this->rosnode_ = new ros::NodeHandle(this->robotNamespace);

  this->imageTopicNameP->Load(node);
  this->cameraInfoTopicNameP->Load(node);
  this->frameNameP->Load(node);
  this->CxPrimeP->Load(node);
  this->CxP->Load(node);
  this->CyP->Load(node);
  this->focal_lengthP->Load(node);
  this->hack_baselineP->Load(node);
  this->distortion_k1P->Load(node);
  this->distortion_k2P->Load(node);
  this->distortion_k3P->Load(node);
  this->distortion_t1P->Load(node);
  this->distortion_t2P->Load(node);
  this->imageTopicName = this->imageTopicNameP->GetValue();
  this->cameraInfoTopicName = this->cameraInfoTopicNameP->GetValue();
  this->frameName = this->frameNameP->GetValue();
  this->CxPrime = this->CxPrimeP->GetValue();
  this->Cx = this->CxP->GetValue();
  this->Cy = this->CyP->GetValue();
  this->focal_length = this->focal_lengthP->GetValue();
  this->hack_baseline = this->hack_baselineP->GetValue();
  this->distortion_k1 = this->distortion_k1P->GetValue();
  this->distortion_k2 = this->distortion_k2P->GetValue();
  this->distortion_k3 = this->distortion_k3P->GetValue();
  this->distortion_t1 = this->distortion_t1P->GetValue();
  this->distortion_t2 = this->distortion_t2P->GetValue();

#ifdef USE_CBQ
  ros::AdvertiseOptions image_ao = ros::AdvertiseOptions::create<sensor_msgs::Image>(
    this->imageTopicName,1,
    boost::bind( &GazeboRosCamera::ImageConnect,this),
    boost::bind( &GazeboRosCamera::ImageDisconnect,this), ros::VoidPtr(), &this->camera_queue_);
  this->image_pub_ = this->rosnode_->advertise(image_ao);

  ros::AdvertiseOptions camera_info_ao = ros::AdvertiseOptions::create<sensor_msgs::CameraInfo>(
    this->cameraInfoTopicName,1,
    boost::bind( &GazeboRosCamera::InfoConnect,this),
    boost::bind( &GazeboRosCamera::InfoDisconnect,this), ros::VoidPtr(), &this->camera_queue_);
  this->camera_info_pub_ = this->rosnode_->advertise(camera_info_ao);

  ros::SubscribeOptions zoom_so = ros::SubscribeOptions::create<std_msgs::Float64>(
    "set_hfov",1,
    boost::bind( &GazeboRosCamera::SetHFOV,this,_1),
    ros::VoidPtr(), &this->camera_queue_);
  this->cameraHFOVSubscriber_ = this->rosnode_->subscribe(zoom_so);

  ros::SubscribeOptions rate_so = ros::SubscribeOptions::create<std_msgs::Float64>(
    "set_update_rate",1,
    boost::bind( &GazeboRosCamera::SetUpdateRate,this,_1),
    ros::VoidPtr(), &this->camera_queue_);
  this->cameraUpdateRateSubscriber_ = this->rosnode_->subscribe(rate_so);

#else
  this->image_pub_ = this->rosnode_->advertise<sensor_msgs::Image>(this->imageTopicName,1,
    boost::bind( &GazeboRosCamera::ImageConnect, this),
    boost::bind( &GazeboRosCamera::ImageDisconnect, this));
  this->camera_info_pub_ = this->rosnode_->advertise<sensor_msgs::CameraInfo>(this->cameraInfoTopicName,1,
    boost::bind( &GazeboRosCamera::InfoConnect, this),
    boost::bind( &GazeboRosCamera::InfoDisconnect, this));
#endif
}

////////////////////////////////////////////////////////////////////////////////
// Increment count
void GazeboRosCamera::InfoConnect()
{
  this->infoConnectCount++;
}
////////////////////////////////////////////////////////////////////////////////
// Decrement count
void GazeboRosCamera::InfoDisconnect()
{
  this->infoConnectCount--;
}

////////////////////////////////////////////////////////////////////////////////
// Set Horizontal Field of View
void GazeboRosCamera::SetHFOV(const std_msgs::Float64::ConstPtr& hfov)
{
  (dynamic_cast<OgreCamera*>(this->myParent))->SetFOV(hfov->data);
}

////////////////////////////////////////////////////////////////////////////////
// Set Update Rate
void GazeboRosCamera::SetUpdateRate(const std_msgs::Float64::ConstPtr& update_rate)
{
  (dynamic_cast<OgreCamera*>(this->myParent))->SetUpdateRate(update_rate->data);
}

////////////////////////////////////////////////////////////////////////////////
// Increment count
void GazeboRosCamera::ImageConnect()
{
  this->imageConnectCount++;
}
////////////////////////////////////////////////////////////////////////////////
// Decrement count
void GazeboRosCamera::ImageDisconnect()
{
  this->imageConnectCount--;

  if (this->imageConnectCount == 0)
    this->myParent->SetActive(false);
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void GazeboRosCamera::InitChild()
{
  // sensor generation off by default
  this->myParent->SetActive(false);

  // set buffer size
  this->width            = this->myParent->GetImageWidth();
  this->height           = this->myParent->GetImageHeight();
  this->depth            = this->myParent->GetImageDepth();
  if (this->myParent->GetImageFormat() == "L8")
  {
    this->type = sensor_msgs::image_encodings::MONO8;
    this->skip = 1;
  }
  else if (this->myParent->GetImageFormat() == "R8G8B8")
  {
    this->type = sensor_msgs::image_encodings::RGB8;
    this->skip = 3;
  }
  else if (this->myParent->GetImageFormat() == "B8G8R8")
  {
    this->type = sensor_msgs::image_encodings::BGR8;
    this->skip = 3;
  }
  else if (this->myParent->GetImageFormat() == "BAYER_RGGB8")
  {
    this->type = sensor_msgs::image_encodings::BAYER_RGGB8;
    this->skip = 1;
  }
  else if (this->myParent->GetImageFormat() == "BAYER_BGGR8")
  {
    this->type = sensor_msgs::image_encodings::BAYER_BGGR8;
    this->skip = 1;
  }
  else if (this->myParent->GetImageFormat() == "BAYER_GBRG8")
  {
    this->type = sensor_msgs::image_encodings::BAYER_GBRG8;
    this->skip = 1;
  }
  else if (this->myParent->GetImageFormat() == "BAYER_GRBG8")
  {
    this->type = sensor_msgs::image_encodings::BAYER_GRBG8;
    this->skip = 1;
  }
  else
  {
    ROS_ERROR("Unsupported Gazebo ImageFormat\n");
    this->type = sensor_msgs::image_encodings::BGR8;
    this->skip = 3;
  }

  /// Compute camera parameters if set to 0
  if (this->CxPrime == 0)
    this->CxPrime = ((double)this->width+1.0) /2.0;
  if (this->Cx == 0)
    this->Cx = ((double)this->width+1.0) /2.0;
  if (this->Cy == 0)
    this->Cy = ((double)this->height+1.0) /2.0;
  if (this->focal_length == 0)
    this->focal_length = ((double)this->width) / (2.0 *tan(this->myParent->GetHFOV().GetAsRadian()/2.0));


#ifdef USE_CBQ
  // start custom queue for camera
  this->callback_queue_thread_ = boost::thread( boost::bind( &GazeboRosCamera::CameraQueueThread,this ) );
#endif


}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosCamera::UpdateChild()
{

  // as long as ros is connected, parent is active
  //ROS_ERROR("debug image count %d",this->imageConnectCount);
  if (!this->myParent->IsActive())
  {
    if (this->imageConnectCount > 0)
      // do this first so there's chance for sensor to run 1 frame after activate
      this->myParent->SetActive(true);
  }
  else
  {
    this->PutCameraData();
  }

  /// publish CameraInfo
  if (this->infoConnectCount > 0)
    this->PublishCameraInfo();
}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void GazeboRosCamera::FiniChild()
{
  this->myParent->SetActive(false);
  this->rosnode_->shutdown();
#ifdef USE_CBQ
  this->camera_queue_.clear();
  this->camera_queue_.disable();
  this->callback_queue_thread_.join();
#endif

}

////////////////////////////////////////////////////////////////////////////////
// Put laser data to the interface
void GazeboRosCamera::PutCameraData()
{
  //ROS_ERROR("debug %d %s",this->imageConnectCount,this->GetName().c_str());
  const unsigned char *src;

  //boost::recursive_mutex::scoped_lock mr_lock(*Simulator::Instance()->GetMRMutex());

  // Get a pointer to image data
  {
    //DIAGNOSTICTIMER(timer("gazebo_ros_camera: GetImageData",6));
    src = this->myParent->GetImageData(0);
  }

  if (src)
  {
    //double tmpT0 = Simulator::Instance()->GetWallTime();

    unsigned char dst[this->width*this->height];

    this->lock.lock();
    // copy data into image
    this->imageMsg.header.frame_id = this->frameName;
    Time lastRenderTime = (dynamic_cast<OgreCamera*>(this->myParent))->GetLastRenderTime();
    //Time lastRenderTime = Simulator::Instance()->GetSimTime();
    //printf("name[%s] render[%f] vs. sim time[%f], diff[%f]\n",this->GetName().c_str(),lastRenderTime.Double(),Simulator::Instance()->GetSimTime().Double(),lastRenderTime.Double()-Simulator::Instance()->GetSimTime().Double());
    //ROS_DEBUG("camera time %f %d %d",lastRenderTime.Double(),lastRenderTime.sec,lastRenderTime.nsec);
    this->imageMsg.header.stamp.sec = lastRenderTime.sec;
    this->imageMsg.header.stamp.nsec = lastRenderTime.nsec;

    //double tmpT1 = Simulator::Instance()->GetWallTime();
    //double tmpT2;

    /// @todo: don't bother if there are no subscribers
    if (this->image_pub_.getNumSubscribers() > 0)
    {

      // do last minute conversion if Bayer pattern is requested but not provided, go from R8G8B8
      // deprecated in gazebo2 branch, keep for backwards compatibility
      if (this->myParent->GetImageFormat() == "BAYER_RGGB8" && this->depth == 3)
      {
        for (int i=0;i<this->width;i++)
        {
          for (int j=0;j<this->height;j++)
          {
            //
            // RG
            // GB
            //
            // determine position
            if (j%2) // even column
              if (i%2) // even row, red
                dst[i+j*this->width] = src[i*3+j*this->width*3+0];
              else // odd row, green
                dst[i+j*this->width] = src[i*3+j*this->width*3+1];
            else // odd column
              if (i%2) // even row, green
                dst[i+j*this->width] = src[i*3+j*this->width*3+1];
              else // odd row, blue
                dst[i+j*this->width] = src[i*3+j*this->width*3+2];
          }
        }
        src=dst;
      }
      else if (this->myParent->GetImageFormat() == "BAYER_BGGR8" && this->depth == 3)
      {
        for (int i=0;i<this->width;i++)
        {
          for (int j=0;j<this->height;j++)
          {
            //
            // BG
            // GR
            //
            // determine position
            if (j%2) // even column
              if (i%2) // even row, blue
                dst[i+j*this->width] = src[i*3+j*this->width*3+2];
              else // odd row, green
                dst[i+j*this->width] = src[i*3+j*this->width*3+1];
            else // odd column
              if (i%2) // even row, green
                dst[i+j*this->width] = src[i*3+j*this->width*3+1];
              else // odd row, red
                dst[i+j*this->width] = src[i*3+j*this->width*3+0];
          }
        }
        src=dst;
      }
      else if (this->myParent->GetImageFormat() == "BAYER_GBRG8" && this->depth == 3)
      {
        for (int i=0;i<this->width;i++)
        {
          for (int j=0;j<this->height;j++)
          {
            //
            // GB
            // RG
            //
            // determine position
            if (j%2) // even column
              if (i%2) // even row, green
                dst[i+j*this->width] = src[i*3+j*this->width*3+1];
              else // odd row, blue
                dst[i+j*this->width] = src[i*3+j*this->width*3+2];
            else // odd column
              if (i%2) // even row, red
                dst[i+j*this->width] = src[i*3+j*this->width*3+0];
              else // odd row, green
                dst[i+j*this->width] = src[i*3+j*this->width*3+1];
          }
        }
        src=dst;
      }
      else if (this->myParent->GetImageFormat() == "BAYER_GRBG8" && this->depth == 3)
      {
        for (int i=0;i<this->width;i++)
        {
          for (int j=0;j<this->height;j++)
          {
            //
            // GR
            // BG
            //
            // determine position
            if (j%2) // even column
              if (i%2) // even row, green
                dst[i+j*this->width] = src[i*3+j*this->width*3+1];
              else // odd row, red
                dst[i+j*this->width] = src[i*3+j*this->width*3+0];
            else // odd column
              if (i%2) // even row, blue
                dst[i+j*this->width] = src[i*3+j*this->width*3+2];
              else // odd row, green
                dst[i+j*this->width] = src[i*3+j*this->width*3+1];
          }
        }
        src=dst;
      }

      // copy from src to imageMsg
      fillImage(this->imageMsg,
                this->type,
                this->height,
                this->width,
                this->skip*this->width,
                (void*)src );

      //tmpT2 = Simulator::Instance()->GetWallTime();

      // publish to ros
      this->image_pub_.publish(this->imageMsg);
    }

    //double tmpT3 = Simulator::Instance()->GetWallTime();

    this->lock.unlock();
  }

}

////////////////////////////////////////////////////////////////////////////////
// Put laser data to the interface
void GazeboRosCamera::PublishCameraInfo()
{
  // fill CameraInfo
  this->cameraInfoMsg.header.frame_id = this->frameName;
  Time lastRenderTime = (dynamic_cast<OgreCamera*>(this->myParent))->GetLastRenderTime();
  //Time lastRenderTime = Simulator::Instance()->GetSimTime();
  this->cameraInfoMsg.header.stamp.sec = lastRenderTime.sec;
  this->cameraInfoMsg.header.stamp.nsec = lastRenderTime.nsec;
  this->cameraInfoMsg.height = this->height;
  this->cameraInfoMsg.width  = this->width;
  // distortion
#if ROS_VERSION_MINIMUM(1, 3, 0)
  this->cameraInfoMsg.distortion_model = "plumb_bob";
  this->cameraInfoMsg.D.resize(5);
#endif
  this->cameraInfoMsg.D[0] = this->distortion_k1;
  this->cameraInfoMsg.D[1] = this->distortion_k2;
  this->cameraInfoMsg.D[2] = this->distortion_k3;
  this->cameraInfoMsg.D[3] = this->distortion_t1;
  this->cameraInfoMsg.D[4] = this->distortion_t2;
  // original camera matrix
  this->cameraInfoMsg.K[0] = this->focal_length;
  this->cameraInfoMsg.K[1] = 0.0;
  this->cameraInfoMsg.K[2] = this->Cx;
  this->cameraInfoMsg.K[3] = 0.0;
  this->cameraInfoMsg.K[4] = this->focal_length;
  this->cameraInfoMsg.K[5] = this->Cy;
  this->cameraInfoMsg.K[6] = 0.0;
  this->cameraInfoMsg.K[7] = 0.0;
  this->cameraInfoMsg.K[8] = 1.0;
  // rectification
  this->cameraInfoMsg.R[0] = 1.0;
  this->cameraInfoMsg.R[1] = 0.0;
  this->cameraInfoMsg.R[2] = 0.0;
  this->cameraInfoMsg.R[3] = 0.0;
  this->cameraInfoMsg.R[4] = 1.0;
  this->cameraInfoMsg.R[5] = 0.0;
  this->cameraInfoMsg.R[6] = 0.0;
  this->cameraInfoMsg.R[7] = 0.0;
  this->cameraInfoMsg.R[8] = 1.0;
  // camera projection matrix (same as camera matrix due to lack of distortion/rectification) (is this generated?)
  this->cameraInfoMsg.P[0] = this->focal_length;
  this->cameraInfoMsg.P[1] = 0.0;
  this->cameraInfoMsg.P[2] = this->Cx;
  this->cameraInfoMsg.P[3] = -this->focal_length * this->hack_baseline;
  this->cameraInfoMsg.P[4] = 0.0;
  this->cameraInfoMsg.P[5] = this->focal_length;
  this->cameraInfoMsg.P[6] = this->Cy;
  this->cameraInfoMsg.P[7] = 0.0;
  this->cameraInfoMsg.P[8] = 0.0;
  this->cameraInfoMsg.P[9] = 0.0;
  this->cameraInfoMsg.P[10] = 1.0;
  this->cameraInfoMsg.P[11] = 0.0;
  this->camera_info_pub_.publish(this->cameraInfoMsg);
}


#ifdef USE_CBQ
////////////////////////////////////////////////////////////////////////////////
// Put laser data to the interface
void GazeboRosCamera::CameraQueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->camera_queue_.callAvailable(ros::WallDuration(timeout));
  }
}
#endif

}
