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
 * Desc: 3D position interface for ground truth.
 * Author: Sachin Chitta and John Hsu
 * Date: 1 June 2008
 * SVN info: $Id: gazebo_ros_imu.cpp 329 2011-05-17 09:39:57Z ricardo $
 */

#include <gazebo_plugins/gazebo_ros_imu.h>

#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/HingeJoint.hh>
#include <gazebo/SliderJoint.hh>
#include <gazebo/Simulator.hh>
#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>

using namespace gazebo;

GZ_REGISTER_DYNAMIC_CONTROLLER("gazebo_ros_imu", GazeboRosIMU);


////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosIMU::GazeboRosIMU(Entity *parent )
   : Controller(parent)
{
   this->myParent = dynamic_cast<Model*>(this->parent);

   if (!this->myParent)
      gzthrow("GazeboRosIMU controller requires a Model as its parent");

  Param::Begin(&this->parameters);
  this->robotNamespaceP = new ParamT<std::string>("robotNamespace", "/", 0);
  this->bodyNameP = new ParamT<std::string>("bodyName", "", 0);
  this->frameNameP = new ParamT<std::string>("frameName", "", 0); // deprecated, warning if specified by user
  this->topicNameP = new ParamT<std::string>("topicName", "", 1);
  this->deprecatedTopicNameP = new ParamT<std::string>("deprecatedTopicName", "", 0);
  this->xyzOffsetsP    = new ParamT<Vector3>("xyzOffsets", Vector3(0,0,0),0);
  this->rpyOffsetsP    = new ParamT<Vector3>("rpyOffsets", Vector3(0,0,0),0);
  this->gaussianNoiseP = new ParamT<double>("gaussianNoise",0.0,0);
  this->serviceNameP = new ParamT<std::string>("serviceName","torso_lift_imu/calibrate", 0);
  Param::End();

  this->imuConnectCount = 0;
  this->deprecatedImuConnectCount = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosIMU::~GazeboRosIMU()
{
  delete this->robotNamespaceP;
  delete this->bodyNameP;
  delete this->frameNameP;
  delete this->topicNameP;
  delete this->deprecatedTopicNameP;
  delete this->xyzOffsetsP;
  delete this->rpyOffsetsP;
  delete this->gaussianNoiseP;
  delete this->serviceNameP;
  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosIMU::LoadChild(XMLConfigNode *node)
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

  this->bodyNameP->Load(node);
  this->bodyName = this->bodyNameP->GetValue();

  // assert that the body by bodyName exists
  if (dynamic_cast<Body*>(this->myParent->GetBody(this->bodyName)) == NULL)
    ROS_FATAL("gazebo_ros_imu plugin error: bodyName: %s does not exist\n",this->bodyName.c_str());

  this->myBody = dynamic_cast<Body*>(this->myParent->GetBody(this->bodyName));

  this->frameNameP->Load(node);
  if (this->frameNameP->GetValue() != "")
    ROS_WARN("Deprecating the ability to specify imu frame_id, this now defaults to the parent imu_link name.  Angular velocity and linear acceleration is in local frame and orientation starts with the transform from gazebo world frame to imu_link frame.  This is done to mimick hardware on PR2.");

  this->topicNameP->Load(node);
  this->topicName = this->topicNameP->GetValue();
  this->deprecatedTopicNameP->Load(node);
  this->deprecatedTopicName = this->deprecatedTopicNameP->GetValue();

  this->xyzOffsetsP->Load(node);
  this->xyzOffsets = this->xyzOffsetsP->GetValue();
  this->rpyOffsetsP->Load(node);
  this->rpyOffsets = this->rpyOffsetsP->GetValue();
  this->gaussianNoiseP->Load(node);
  this->gaussianNoise = this->gaussianNoiseP->GetValue();

  if (this->topicName != "")
  {
#ifdef USE_CBQ
    ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<sensor_msgs::Imu>(
      this->topicName,1,
      boost::bind( &GazeboRosIMU::IMUConnect,this),
      boost::bind( &GazeboRosIMU::IMUDisconnect,this), ros::VoidPtr(), &this->imu_queue_);
    this->pub_ = this->rosnode_->advertise(ao);
#else
    this->pub_ = this->rosnode_->advertise<sensor_msgs::Imu>(this->topicName,10);
#endif
  }


  if (this->deprecatedTopicName != "")
  {
#ifdef USE_CBQ
    ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<sensor_msgs::Imu>(
      this->deprecatedTopicName,1,
      boost::bind( &GazeboRosIMU::DeprecatedIMUConnect,this),
      boost::bind( &GazeboRosIMU::DeprecatedIMUDisconnect,this), ros::VoidPtr(), &this->imu_queue_);
    this->deprecated_pub_ = this->rosnode_->advertise(ao);
#else
    this->deprecated_pub_ = this->rosnode_->advertise<sensor_msgs::Imu>(this->deprecatedTopicName,10);
#endif
  }

  // add service call version for position change
  this->serviceNameP->Load(node);
  this->serviceName = this->serviceNameP->GetValue();
  // advertise services on the custom queue
  ros::AdvertiseServiceOptions aso = ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
      this->serviceName,boost::bind( &GazeboRosIMU::ServiceCallback, this, _1, _2 ), ros::VoidPtr(), &this->imu_queue_);
  this->srv_ = this->rosnode_->advertiseService(aso);

}

////////////////////////////////////////////////////////////////////////////////
// returns true always, imu is always calibrated in sim
bool GazeboRosIMU::ServiceCallback(std_srvs::Empty::Request &req,
                                        std_srvs::Empty::Response &res)
{
  return true;
}

////////////////////////////////////////////////////////////////////////////////
// Increment count
void GazeboRosIMU::IMUConnect()
{
  this->imuConnectCount++;
}
////////////////////////////////////////////////////////////////////////////////
// Decrement count
void GazeboRosIMU::IMUDisconnect()
{
  this->imuConnectCount--;
}

////////////////////////////////////////////////////////////////////////////////
// Increment count
void GazeboRosIMU::DeprecatedIMUConnect()
{
  ROS_WARN("you are subscribing to a deprecated ROS topic %s, please change your code/launch script to use new ROS topic %s",
           this->deprecatedTopicName.c_str(), this->topicName.c_str());
  this->deprecatedImuConnectCount++;
}
////////////////////////////////////////////////////////////////////////////////
// Decrement count
void GazeboRosIMU::DeprecatedIMUDisconnect()
{
  this->deprecatedImuConnectCount--;
}


////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void GazeboRosIMU::InitChild()
{
  this->last_time = Simulator::Instance()->GetSimTime();
  //this->initial_pose = this->myBody->GetPose(); // get initial pose of the local link
  this->last_vpos = this->myBody->GetWorldLinearVel(); // get velocity in gazebo frame
  this->last_veul = this->myBody->GetWorldAngularVel(); // get velocity in gazebo frame
  this->apos = 0;
  this->aeul = 0;
#ifdef USE_CBQ
  // start custom queue for imu
  this->callback_queue_thread_ = boost::thread( boost::bind( &GazeboRosIMU::IMUQueueThread,this ) );
#endif
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosIMU::UpdateChild()
{
  if ((this->imuConnectCount > 0 && this->topicName != "") ||
      (this->deprecatedImuConnectCount > 0 && this->deprecatedTopicName != ""))
  {
    Pose3d pose;
    Quatern rot;
    Vector3 pos;

    // Get Pose/Orientation ///@todo: verify correctness
    pose = this->myBody->GetWorldPose(); // - this->myBody->GetCoMPose();
    // apply xyz offsets and get position and rotation components
    pos = pose.pos + this->xyzOffsets;
    rot = pose.rot;
    // std::cout << " --------- GazeboRosIMU rot " << rot.x << ", " << rot.y << ", " << rot.z << ", " << rot.u << std::endl;

    // apply rpy offsets
    Quatern qOffsets;
    qOffsets.SetFromEuler(this->rpyOffsets);
    rot = qOffsets*rot;
    rot.Normalize();

    gazebo::Time cur_time = Simulator::Instance()->GetSimTime();
    
    // get Rates
    Vector3 vpos = this->myBody->GetWorldLinearVel(); // get velocity in gazebo frame
    Vector3 veul = this->myBody->GetWorldAngularVel(); // get velocity in gazebo frame

    // differentiate to get accelerations
    double tmp_dt = this->last_time.Double() - cur_time.Double();
    if (tmp_dt != 0)
    {
      this->apos = (this->last_vpos - vpos) / tmp_dt;
      this->aeul = (this->last_veul - veul) / tmp_dt;
      this->last_vpos = vpos;
      this->last_veul = veul;
    }

    this->lock.lock();


    // copy data into pose message
    this->imuMsg.header.frame_id = this->bodyName;
    this->imuMsg.header.stamp.sec = cur_time.sec;
    this->imuMsg.header.stamp.nsec = cur_time.nsec;

    // orientation quaternion

    // uncomment this if we are reporting orientation in the local frame
    // not the case for our imu definition
    // // apply fixed orientation offsets of initial pose
    // rot = this->initial_pose.rot*rot;
    // rot.Normalize();

    this->imuMsg.orientation.x = rot.x;
    this->imuMsg.orientation.y = rot.y;
    this->imuMsg.orientation.z = rot.z;
    this->imuMsg.orientation.w = rot.u;

    // pass euler angular rates
    Vector3 linear_velocity(veul.x + this->GaussianKernel(0,this->gaussianNoise)
                           ,veul.y + this->GaussianKernel(0,this->gaussianNoise)
                           ,veul.z + this->GaussianKernel(0,this->gaussianNoise));
    // rotate into local frame
    // @todo: deal with offsets!
    linear_velocity = rot.RotateVector(linear_velocity);
    this->imuMsg.angular_velocity.x    = linear_velocity.x;
    this->imuMsg.angular_velocity.y    = linear_velocity.y;
    this->imuMsg.angular_velocity.z    = linear_velocity.z;

    // pass accelerations
    Vector3 linear_acceleration(apos.x + this->GaussianKernel(0,this->gaussianNoise)
                               ,apos.y + this->GaussianKernel(0,this->gaussianNoise)
                               ,apos.z + this->GaussianKernel(0,this->gaussianNoise));
    // rotate into local frame
    // @todo: deal with offsets!
    linear_acceleration = rot.RotateVector(linear_acceleration);
    this->imuMsg.linear_acceleration.x    = linear_acceleration.x;
    this->imuMsg.linear_acceleration.y    = linear_acceleration.y;
    this->imuMsg.linear_acceleration.z    = linear_acceleration.z;

    // fill in covariance matrix
    /// @todo: let user set separate linear and angular covariance values.
    /// @todo: apply appropriate rotations from frame_pose
    this->imuMsg.orientation_covariance[0] = this->gaussianNoise*this->gaussianNoise;
    this->imuMsg.orientation_covariance[4] = this->gaussianNoise*this->gaussianNoise;
    this->imuMsg.orientation_covariance[8] = this->gaussianNoise*this->gaussianNoise;
    this->imuMsg.angular_velocity_covariance[0] = this->gaussianNoise*this->gaussianNoise;
    this->imuMsg.angular_velocity_covariance[4] = this->gaussianNoise*this->gaussianNoise;
    this->imuMsg.angular_velocity_covariance[8] = this->gaussianNoise*this->gaussianNoise;
    this->imuMsg.linear_acceleration_covariance[0] = this->gaussianNoise*this->gaussianNoise;
    this->imuMsg.linear_acceleration_covariance[4] = this->gaussianNoise*this->gaussianNoise;
    this->imuMsg.linear_acceleration_covariance[8] = this->gaussianNoise*this->gaussianNoise;

    // publish to ros
    if (this->imuConnectCount > 0 && this->topicName != "")
        this->pub_.publish(this->imuMsg);

    if (this->deprecatedImuConnectCount > 0 && this->deprecatedTopicName != "")
      this->deprecated_pub_.publish(this->imuMsg);

    this->lock.unlock();

    // save last time stamp
    this->last_time = cur_time;
  }
}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void GazeboRosIMU::FiniChild()
{
  this->rosnode_->shutdown();
  this->callback_queue_thread_.join();
}

//////////////////////////////////////////////////////////////////////////////
// Utility for adding noise
double GazeboRosIMU::GaussianKernel(double mu,double sigma)
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
void GazeboRosIMU::IMUQueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->imu_queue_.callAvailable(ros::WallDuration(timeout));
  }
}
#endif
