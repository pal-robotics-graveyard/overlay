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
 * Desc: Ros Block Laser controller.
 * Author: Nathan Koenig
 * Date: 01 Feb 2007
 * SVN info: $Id: gazebo_ros_block_laser.cpp 329 2011-05-17 09:39:57Z ricardo $
 */

#include <algorithm>
#include <assert.h>

#include <gazebo_plugins/gazebo_ros_block_laser.h>

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

#include <geometry_msgs/Point32.h>
#include <sensor_msgs/ChannelFloat32.h>

using namespace gazebo;

GZ_REGISTER_DYNAMIC_CONTROLLER("gazebo_ros_block_laser", GazeboRosBlockLaser);

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosBlockLaser::GazeboRosBlockLaser(Entity *parent)
    : Controller(parent)
{
  this->myParent = dynamic_cast<RaySensor*>(this->parent);

  if (!this->myParent)
    gzthrow("GazeboRosBlockLaser controller requires a Ray Sensor as its parent");

  Param::Begin(&this->parameters);
  this->robotNamespaceP = new ParamT<std::string>("robotNamespace", "/", 0);
  this->gaussianNoiseP = new ParamT<double>("gaussianNoise", 0.0, 0);
  this->topicNameP = new ParamT<std::string>("topicName", "", 1);
  this->frameNameP = new ParamT<std::string>("frameName", "default_ros_laser_frame", 0);
  Param::End();
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosBlockLaser::~GazeboRosBlockLaser()
{
  delete this->robotNamespaceP;
  delete this->gaussianNoiseP;
  delete this->topicNameP;
  delete this->frameNameP;
  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosBlockLaser::LoadChild(XMLConfigNode *node)
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

  this->topicNameP->Load(node);
  this->topicName = this->topicNameP->GetValue();
  this->frameNameP->Load(node);
  this->frameName = this->frameNameP->GetValue();
  this->gaussianNoiseP->Load(node);
  this->gaussianNoise = this->gaussianNoiseP->GetValue();

  // set size of cloud message, starts at 0!! FIXME: not necessary
  this->cloudMsg.points.clear();
  this->cloudMsg.channels.clear();
  this->cloudMsg.channels.push_back(sensor_msgs::ChannelFloat32());

  // Custom Callback Queue
  ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<sensor_msgs::PointCloud>(
    this->topicName,1,
    boost::bind( &GazeboRosBlockLaser::LaserConnect,this),
    boost::bind( &GazeboRosBlockLaser::LaserDisconnect,this), ros::VoidPtr(), &this->queue_);
  this->pub_ = this->rosnode_->advertise(ao);

}

////////////////////////////////////////////////////////////////////////////////
// Increment count
void GazeboRosBlockLaser::LaserConnect()
{
  this->laserConnectCount++;
}
////////////////////////////////////////////////////////////////////////////////
// Decrement count
void GazeboRosBlockLaser::LaserDisconnect()
{
  this->laserConnectCount--;

  if (this->laserConnectCount == 0)
    this->myParent->SetActive(false);
}
////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void GazeboRosBlockLaser::InitChild()
{
  // set parent sensor to inactive automatically
  this->myParent->SetActive(false);
  // Custom Callback Queue
  this->callback_queue_thread_ = boost::thread( boost::bind( &GazeboRosBlockLaser::QueueThread,this ) );
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosBlockLaser::UpdateChild()
{
  // as long as ros is connected, parent is active
  //ROS_ERROR("debug laser count %d",this->laserConnectCount);
  if (!this->myParent->IsActive())
  {
    // do this first so there's chance for sensor to run 1 frame after activate
    if (this->laserConnectCount > 0)
      this->myParent->SetActive(true);
  }
  else
  {
    this->PutLaserData();
  }
}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void GazeboRosBlockLaser::FiniChild()
{
  // Custom Callback Queue
  this->queue_.clear();
  this->queue_.disable();
  this->rosnode_->shutdown();
  this->callback_queue_thread_.join();
}

////////////////////////////////////////////////////////////////////////////////
// Put laser data to the interface
void GazeboRosBlockLaser::PutLaserData()
{
  int i, hja, hjb;
  int j, vja, vjb;
  double vb, hb;
  int    j1, j2, j3, j4; // four corners indices
  double r1, r2, r3, r4, r; // four corner values + interpolated range
  double intensity;

  Angle maxAngle = this->myParent->GetMaxAngle();
  Angle minAngle = this->myParent->GetMinAngle();

  double maxRange = this->myParent->GetMaxRange();
  double minRange = this->myParent->GetMinRange();
  int rayCount = this->myParent->GetRayCount();
  int rangeCount = this->myParent->GetRangeCount();

  int verticalRayCount = this->myParent->GetVerticalRayCount();
  int verticalRangeCount = this->myParent->GetVerticalRangeCount();
  Angle verticalMaxAngle = this->myParent->GetVerticalMaxAngle();
  Angle verticalMinAngle = this->myParent->GetVerticalMinAngle();

  double yDiff = maxAngle.GetAsRadian() - minAngle.GetAsRadian();
  double pDiff = verticalMaxAngle.GetAsRadian() - verticalMinAngle.GetAsRadian();


  // set size of cloud message everytime!
  //int r_size = rangeCount * verticalRangeCount;
  this->cloudMsg.points.clear();
  this->cloudMsg.channels.clear();
  this->cloudMsg.channels.push_back(sensor_msgs::ChannelFloat32());

  /***************************************************************/
  /*                                                             */
  /*  point scan from laser                                      */
  /*                                                             */
  /***************************************************************/
  boost::mutex::scoped_lock sclock(this->lock);
  // Add Frame Name
  this->cloudMsg.header.frame_id = this->frameName;
  this->cloudMsg.header.stamp.sec = (Simulator::Instance()->GetSimTime()).sec;
  this->cloudMsg.header.stamp.nsec = (Simulator::Instance()->GetSimTime()).nsec;

  for (j = 0; j<verticalRangeCount; j++)
  {
    // interpolating in vertical direction
    vb = (verticalRangeCount == 1) ? 0 : (double) j * (verticalRayCount - 1) / (verticalRangeCount - 1);
    vja = (int) floor(vb);
    vjb = std::min(vja + 1, verticalRayCount - 1);
    vb = vb - floor(vb); // fraction from min

    assert(vja >= 0 && vja < verticalRayCount);
    assert(vjb >= 0 && vjb < verticalRayCount);

    for (i = 0; i<rangeCount; i++)
    {
      // Interpolate the range readings from the rays in horizontal direction
      hb = (rangeCount == 1)? 0 : (double) i * (rayCount - 1) / (rangeCount - 1);
      hja = (int) floor(hb);
      hjb = std::min(hja + 1, rayCount - 1);
      hb = hb - floor(hb); // fraction from min

      assert(hja >= 0 && hja < rayCount);
      assert(hjb >= 0 && hjb < rayCount);

      // indices of 4 corners
      j1 = hja + vja * rayCount;
      j2 = hjb + vja * rayCount;
      j3 = hja + vjb * rayCount;
      j4 = hjb + vjb * rayCount;
      // range readings of 4 corners
      r1 = std::min(this->myParent->GetRange(j1) , maxRange-minRange);
      r2 = std::min(this->myParent->GetRange(j2) , maxRange-minRange);
      r3 = std::min(this->myParent->GetRange(j3) , maxRange-minRange);
      r4 = std::min(this->myParent->GetRange(j4) , maxRange-minRange);

      // Range is linear interpolation if values are close,
      // and min if they are very different
      r = (1-vb)*((1 - hb) * r1 + hb * r2)
         +   vb *((1 - hb) * r3 + hb * r4);

      // Intensity is averaged
      intensity = 0.25*(this->myParent->GetRetro(j1) + (int) this->myParent->GetRetro(j2) +
                        this->myParent->GetRetro(j3) + (int) this->myParent->GetRetro(j4));

      // std::cout << " block debug "
      //           << "  ij("<<i<<","<<j<<")"
      //           << "  j1234("<<j1<<","<<j2<<","<<j3<<","<<j4<<")"
      //           << "  r1234("<<r1<<","<<r2<<","<<r3<<","<<r4<<")"
      //           << std::endl;

      // get angles of ray to get xyz for point
      double yAngle = 0.5*(hja+hjb) * yDiff / (rayCount -1) + minAngle.GetAsRadian();
      double pAngle = 0.5*(vja+vjb) * pDiff / (verticalRayCount -1) + verticalMinAngle.GetAsRadian();

      /***************************************************************/
      /*                                                             */
      /*  point scan from laser                                      */
      /*                                                             */
      /***************************************************************/
      if (r == maxRange - minRange)
      {
        // no noise if at max range
        geometry_msgs::Point32 point;
        point.x = (r+minRange) * cos(pAngle)*cos(yAngle);
        point.y = (r+minRange) *             sin(yAngle);
        point.z = (r+minRange) * sin(pAngle)*cos(yAngle);
        this->cloudMsg.points.push_back(point);
      }
      else
      {
        geometry_msgs::Point32 point;
        point.x      = (r+minRange) * cos(pAngle)*cos(yAngle) + this->GaussianKernel(0,this->gaussianNoise) ;
        point.y      = (r+minRange) *             sin(yAngle) + this->GaussianKernel(0,this->gaussianNoise) ;
        point.z      = (r+minRange) * sin(pAngle)*cos(yAngle) + this->GaussianKernel(0,this->gaussianNoise) ;
        this->cloudMsg.points.push_back(point);
      }
      // only 1 channel
      this->cloudMsg.channels[0].values.push_back(intensity + this->GaussianKernel(0,this->gaussianNoise)) ;
    }
  }

  // send data out via ros message
  this->pub_.publish(this->cloudMsg);



}


//////////////////////////////////////////////////////////////////////////////
// Utility for adding noise
double GazeboRosBlockLaser::GaussianKernel(double mu,double sigma)
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

// Custom Callback Queue
////////////////////////////////////////////////////////////////////////////////
// custom callback queue thread
void GazeboRosBlockLaser::QueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->queue_.callAvailable(ros::WallDuration(timeout));
  }
}





