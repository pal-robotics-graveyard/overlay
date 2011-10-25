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
 * Desc: Ros Laser controller.
 * Author: Nathan Koenig
 * Date: 01 Feb 2007
 * SVN info: $Id: gazebo_ros_laser.cpp 329 2011-05-17 09:39:57Z ricardo $
 */

#include <algorithm>
#include <assert.h>

#include <gazebo_plugins/gazebo_ros_laser.h>

#include <gazebo/Sensor.hh>
#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/HingeJoint.hh>
#include <gazebo/World.hh>
#include <gazebo/Simulator.hh>
#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>
#include <gazebo/RaySensor.hh>

namespace gazebo
{

GZ_REGISTER_DYNAMIC_CONTROLLER("gazebo_ros_laser", GazeboRosLaser);

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosLaser::GazeboRosLaser(Entity *parent)
    : Controller(parent)
{
  this->myParent = dynamic_cast<RaySensor*>(this->parent);

  if (!this->myParent)
    gzthrow("GazeboRosLaser controller requires a Ray Sensor as its parent");

  Param::Begin(&this->parameters);
  this->robotNamespaceP = new ParamT<std::string>("robotNamespace", "/", 0);
  this->hokuyoMinIntensityP = new ParamT<double>("hokuyoMinIntensity", 101.0, 0);
  this->gaussianNoiseP = new ParamT<double>("gaussianNoise", 0.0, 0);
  this->topicNameP = new ParamT<std::string>("topicName", "", 1);
  this->deprecatedTopicNameP = new ParamT<std::string>("deprecatedTopicName", "", 0);
  this->frameNameP = new ParamT<std::string>("frameName", "default_gazebo_ros_laser_frame", 0);
  Param::End();

  this->laserConnectCount = 0;
  this->deprecatedLaserConnectCount = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosLaser::~GazeboRosLaser()
{
  delete this->robotNamespaceP;
  delete this->hokuyoMinIntensityP;
  delete this->gaussianNoiseP;
  delete this->topicNameP;
  delete this->deprecatedTopicNameP;
  delete this->frameNameP;
  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosLaser::LoadChild(XMLConfigNode *node)
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

  this->hokuyoMinIntensityP->Load(node);
  this->hokuyoMinIntensity = this->hokuyoMinIntensityP->GetValue();
  ROS_INFO("INFO: gazebo_ros_laser plugin artifically sets minimum intensity to %f due to cutoff in hokuyo filters." , this->hokuyoMinIntensity);

  this->topicNameP->Load(node);
  this->topicName = this->topicNameP->GetValue();
  this->deprecatedTopicNameP->Load(node);
  this->deprecatedTopicName = this->deprecatedTopicNameP->GetValue();
  this->frameNameP->Load(node);
  this->frameName = this->frameNameP->GetValue();
  this->gaussianNoiseP->Load(node);
  this->gaussianNoise = this->gaussianNoiseP->GetValue();

  if (this->topicName != "")
  {
#ifdef USE_CBQ
    ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<sensor_msgs::LaserScan>(
      this->topicName,1,
      boost::bind( &GazeboRosLaser::LaserConnect,this),
      boost::bind( &GazeboRosLaser::LaserDisconnect,this), ros::VoidPtr(), &this->laser_queue_);
    this->pub_ = this->rosnode_->advertise(ao);
#else
    this->pub_ = this->rosnode_->advertise<sensor_msgs::LaserScan>(this->topicName,1,
      boost::bind( &GazeboRosLaser::LaserConnect, this),
      boost::bind( &GazeboRosLaser::LaserDisconnect, this));
#endif
  }

  if (this->deprecatedTopicName != "")
  {
#ifdef USE_CBQ
    ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<sensor_msgs::LaserScan>(
      this->deprecatedTopicName,1,
      boost::bind( &GazeboRosLaser::DeprecatedLaserConnect,this),
      boost::bind( &GazeboRosLaser::DeprecatedLaserDisconnect,this), ros::VoidPtr(), &this->laser_queue_);
    this->deprecated_pub_ = this->rosnode_->advertise(ao);
#else
    this->deprecated_pub_ = this->rosnode_->advertise<sensor_msgs::LaserScan>(this->deprecatedTopicName,1,
      boost::bind( &GazeboRosLaser::DeprecatedLaserConnect, this),
      boost::bind( &GazeboRosLaser::DeprecatedLaserDisconnect, this));
#endif
  }
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void GazeboRosLaser::InitChild()
{
  // sensor generation off by default
  this->myParent->SetActive(false);
#ifdef USE_CBQ
  // start custom queue for laser
  this->callback_queue_thread_ = boost::thread( boost::bind( &GazeboRosLaser::LaserQueueThread,this ) );
#endif
}

////////////////////////////////////////////////////////////////////////////////
// Increment count
void GazeboRosLaser::LaserConnect()
{
  this->laserConnectCount++;
}
////////////////////////////////////////////////////////////////////////////////
// Decrement count
void GazeboRosLaser::LaserDisconnect()
{
  this->laserConnectCount--;

  if (this->laserConnectCount == 0 && this->deprecatedLaserConnectCount == 0)
    this->myParent->SetActive(false);
}

////////////////////////////////////////////////////////////////////////////////
// Increment count
void GazeboRosLaser::DeprecatedLaserConnect()
{
  ROS_WARN("you are subscribing to a deprecated ROS topic %s, please change your code/launch script to use new ROS topic %s",
           this->deprecatedTopicName.c_str(), this->topicName.c_str());
  this->deprecatedLaserConnectCount++;
}
////////////////////////////////////////////////////////////////////////////////
// Decrement count
void GazeboRosLaser::DeprecatedLaserDisconnect()
{
  this->deprecatedLaserConnectCount--;

  if (this->laserConnectCount == 0 && this->deprecatedLaserConnectCount == 0)
    this->myParent->SetActive(false);
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosLaser::UpdateChild()
{
  // as long as ros is connected, parent is active
  //ROS_ERROR("debug laser count %d",this->laserConnectCount);
  if (!this->myParent->IsActive())
  {
    // do this first so there's chance for sensor to run 1 frame after activate
    if ((this->laserConnectCount > 0 && this->topicName != "") ||
        (this->deprecatedLaserConnectCount > 0 && this->deprecatedTopicName != ""))
      this->myParent->SetActive(true);
  }
  else
  {
    this->PutLaserData();
  }
}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void GazeboRosLaser::FiniChild()
{
  this->rosnode_->shutdown();
  sleep(1);
#ifdef USE_CBQ
  this->callback_queue_thread_.join();
#endif
}

////////////////////////////////////////////////////////////////////////////////
// Put laser data to the interface
void GazeboRosLaser::PutLaserData()
{
  int i, ja, jb;
  double ra, rb, r, b;
  double intensity;

  Angle maxAngle = this->myParent->GetMaxAngle();
  Angle minAngle = this->myParent->GetMinAngle();

  double maxRange = this->myParent->GetMaxRange();
  double minRange = this->myParent->GetMinRange();
  int rayCount = this->myParent->GetRayCount();
  int rangeCount = this->myParent->GetRangeCount();

  /***************************************************************/
  /*                                                             */
  /*  point scan from laser                                      */
  /*                                                             */
  /***************************************************************/
  this->lock.lock();
  // Add Frame Name
  this->laserMsg.header.frame_id = this->frameName;
  this->laserMsg.header.stamp.sec = (Simulator::Instance()->GetSimTime()).sec;
  this->laserMsg.header.stamp.nsec = (Simulator::Instance()->GetSimTime()).nsec;


  double tmp_res_angle = (maxAngle.GetAsRadian() - minAngle.GetAsRadian())/((double)(rangeCount -1)); // for computing yaw
  this->laserMsg.angle_min = minAngle.GetAsRadian();
  this->laserMsg.angle_max = maxAngle.GetAsRadian();
  this->laserMsg.angle_increment = tmp_res_angle;
  this->laserMsg.time_increment  = 0; // instantaneous simulator scan
  this->laserMsg.scan_time       = 0; // FIXME: what's this?
  this->laserMsg.range_min = minRange;
  this->laserMsg.range_max = maxRange;
  this->laserMsg.ranges.clear();
  this->laserMsg.intensities.clear();

  // Interpolate the range readings from the rays
  for (i = 0; i<rangeCount; i++)
  {
    b = (double) i * (rayCount - 1) / (rangeCount - 1);
    ja = (int) floor(b);
    jb = std::min(ja + 1, rayCount - 1);
    b = b - floor(b);

    assert(ja >= 0 && ja < rayCount);
    assert(jb >= 0 && jb < rayCount);

    ra = std::min(this->myParent->GetRange(ja) , maxRange-minRange); // length of ray
    rb = std::min(this->myParent->GetRange(jb) , maxRange-minRange); // length of ray

    // Range is linear interpolation if values are close,
    // and min if they are very different
    //if (fabs(ra - rb) < 0.10)
      r = (1 - b) * ra + b * rb;
    //else r = std::min(ra, rb);

    // Intensity is averaged
    intensity = 0.5*( this->myParent->GetRetro(ja) + (int) this->myParent->GetRetro(jb));

    /***************************************************************/
    /*                                                             */
    /*  point scan from laser                                      */
    /*                                                             */
    /***************************************************************/
    this->laserMsg.ranges.push_back(std::min(r + minRange + this->GaussianKernel(0,this->gaussianNoise), maxRange));
    this->laserMsg.intensities.push_back(std::max(this->hokuyoMinIntensity,intensity + this->GaussianKernel(0,this->gaussianNoise)));
  }

  // send data out via ros message
  if (this->laserConnectCount > 0 && this->topicName != "")
      this->pub_.publish(this->laserMsg);

  if (this->deprecatedLaserConnectCount > 0 && this->deprecatedTopicName != "")
      this->deprecated_pub_.publish(this->laserMsg);

  this->lock.unlock();

}

//////////////////////////////////////////////////////////////////////////////
// Utility for adding noise
double GazeboRosLaser::GaussianKernel(double mu,double sigma)
{
  // using Box-Muller transform to generate two independent standard normally disbributed normal variables
  // see wikipedia
  double U = (double)rand()/(double)RAND_MAX; // normalized uniform random variable
  double V = (double)rand()/(double)RAND_MAX; // normalized uniform random variable
  double X = sqrt(-2.0 * ::log(U)) * cos( 2.0*M_PI * V);
  //double Y = sqrt(-2.0 * ::log(U)) * sin( 2.0*M_PI * V); // the other indep. normal variable
  // we'll just use X
  // scale to our mu and sigma
  X = sigma * X + mu;
  return X;
}

#ifdef USE_CBQ
////////////////////////////////////////////////////////////////////////////////
// Put laser data to the interface
void GazeboRosLaser::LaserQueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->laser_queue_.callAvailable(ros::WallDuration(timeout));
  }
}
#endif

}
