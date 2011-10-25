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
 * SVN info: $Id: gazebo_ros_p3d.cpp 329 2011-05-17 09:39:57Z ricardo $
 */

#include <gazebo_plugins/gazebo_ros_p3d.h>

#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/HingeJoint.hh>
#include <gazebo/SliderJoint.hh>
#include <gazebo/Simulator.hh>
#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>
#include <gazebo/World.hh>

using namespace gazebo;

GZ_REGISTER_DYNAMIC_CONTROLLER("gazebo_ros_p3d", GazeboRosP3D);


////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosP3D::GazeboRosP3D(Entity *parent )
   : Controller(parent)
{
   this->myParent = dynamic_cast<Model*>(this->parent);

   if (!this->myParent)
      gzthrow("GazeboRosP3D controller requires a Model as its parent");

  Param::Begin(&this->parameters);
  this->robotNamespaceP = new ParamT<std::string>("robotNamespace", "/", 0);
  this->bodyNameP = new ParamT<std::string>("bodyName", "", 0);
  this->topicNameP = new ParamT<std::string>("topicName", "", 1);
  this->frameNameP = new ParamT<std::string>("frameName", "world", 0);
  this->xyzOffsetsP    = new ParamT<Vector3>("xyzOffsets", Vector3(0,0,0),0);
  this->rpyOffsetsP    = new ParamT<Vector3>("rpyOffsets", Vector3(0,0,0),0);
  this->gaussianNoiseP = new ParamT<double>("gaussianNoise",0.0,0);
  Param::End();

  this->p3dConnectCount = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosP3D::~GazeboRosP3D()
{
  delete this->robotNamespaceP;
  delete this->bodyNameP;
  delete this->topicNameP;
  delete this->frameNameP;
  delete this->xyzOffsetsP;
  delete this->rpyOffsetsP;
  delete this->gaussianNoiseP;
  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosP3D::LoadChild(XMLConfigNode *node)
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
  this->myBody = dynamic_cast<Body*>(this->myParent->GetBody(this->bodyName));
  if (this->myBody == NULL)
  {
    ROS_WARN("gazebo_ros_p3d plugin error: bodyName: %s does not exist\n",this->bodyName.c_str());
    return;
  }

  this->topicNameP->Load(node);
  this->topicName = this->topicNameP->GetValue();

  this->frameNameP->Load(node);
  this->frameName = this->frameNameP->GetValue();

  this->xyzOffsetsP->Load(node);
  this->xyzOffsets = this->xyzOffsetsP->GetValue();
  this->rpyOffsetsP->Load(node);
  this->rpyOffsets = this->rpyOffsetsP->GetValue();
  this->gaussianNoiseP->Load(node);
  this->gaussianNoise = this->gaussianNoiseP->GetValue();

  if (this->topicName != "")
  {
#ifdef USE_CBQ
  ros::AdvertiseOptions p3d_ao = ros::AdvertiseOptions::create<nav_msgs::Odometry>(
    this->topicName,1,
    boost::bind( &GazeboRosP3D::P3DConnect,this),
    boost::bind( &GazeboRosP3D::P3DDisconnect,this), ros::VoidPtr(), &this->p3d_queue_);
  this->pub_ = this->rosnode_->advertise(p3d_ao);
#else
    this->pub_ = this->rosnode_->advertise<nav_msgs::Odometry>(this->topicName,1,
                 boost::bind( &GazeboRosP3D::P3DConnect, this),
                 boost::bind( &GazeboRosP3D::P3DDisconnect, this));
#endif
  }
}

////////////////////////////////////////////////////////////////////////////////
// Increment count
void GazeboRosP3D::P3DConnect()
{
  this->p3dConnectCount++;
}
////////////////////////////////////////////////////////////////////////////////
// Decrement count
void GazeboRosP3D::P3DDisconnect()
{
  this->p3dConnectCount--;
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void GazeboRosP3D::InitChild()
{
  if (this->myBody == NULL)
    return;

  this->last_time = Simulator::Instance()->GetSimTime();
  // initialize body
  this->last_vpos = this->myBody->GetWorldLinearVel(); // get velocity in gazebo frame
  this->last_veul = this->myBody->GetWorldAngularVel(); // get velocity in gazebo frame
  this->apos = 0;
  this->aeul = 0;

  // preset myFrame to NULL, will search for the body with matching name in UpdateChild()
  // since most bodies are constructed on the fly
  this->myFrame = NULL;
  this->frame_apos = 0;
  this->frame_aeul = 0;
#ifdef USE_CBQ
  // start custom queue for p3d
  this->callback_queue_thread_ = boost::thread( boost::bind( &GazeboRosP3D::P3DQueueThread,this ) );
#endif
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosP3D::UpdateChild()
{
  if (this->myBody == NULL)
    return;

  if (this->p3dConnectCount > 0)
  {
    /// if frameName specified is "world", "/map" or "map" report back inertial values in the gazebo world
    if (this->myFrame == NULL && this->frameName != "world" && this->frameName != "/map" && this->frameName != "map")
    {
      // lock in case a model is being spawned
      boost::recursive_mutex::scoped_lock lock(*Simulator::Instance()->GetMRMutex());
      // look through all models in the world, search for body name that matches frameName
      std::vector<Model*> all_models = World::Instance()->GetModels();
      for (std::vector<Model*>::iterator iter = all_models.begin(); iter != all_models.end(); iter++)
      {
        //ROS_ERROR("checking model %s for link %s",(*iter)->GetName().c_str(),this->frameName.c_str());
        if (*iter) this->myFrame = dynamic_cast<Body*>((*iter)->GetBody(this->frameName));
        if (this->myFrame) break;
      }

      // not found
      if (this->myFrame == NULL)
      {
        ROS_DEBUG("gazebo_ros_p3d plugin: frameName: %s does not exist yet, will not publish\n",this->frameName.c_str());
        return;
      }
      else
      {
        //ROS_ERROR("got body %s",this->myFrame->GetName().c_str());
        // found!  initialize frame
        this->last_frame_vpos = this->myFrame->GetWorldLinearVel(); // get velocity in gazebo frame
        this->last_frame_veul = this->myFrame->GetWorldAngularVel(); // get velocity in gazebo frame
        this->initial_frame_pose = this->myFrame->GetWorldPose(); // get velocity in gazebo frame
      }
    }

    Pose3d pose, frame_pose;
    Quatern rot, frame_rot;
    Vector3 pos, frame_pos;

    // Get Pose/Orientation ///@todo: verify correctness
    pose = this->myBody->GetWorldPose(); // - this->myBody->GetCoMPose();
    // apply xyz offsets and get position and rotation components
    pos = pose.pos + this->xyzOffsets;
    rot = pose.rot;
    // std::cout << " --------- GazeboRosP3D rot " << rot.x << ", " << rot.y << ", " << rot.z << ", " << rot.u << std::endl;

    if (this->myFrame)
    {
      frame_pose = this->myFrame->GetWorldPose(); // - this->myBody->GetCoMPose();
      frame_pos = frame_pose.pos;
      frame_rot = frame_pose.rot;
    }

    // apply rpy offsets
    Quatern qOffsets;
    qOffsets.SetFromEuler(this->rpyOffsets);
    rot = qOffsets*rot;
    rot.Normalize();

    gazebo::Time cur_time = Simulator::Instance()->GetSimTime();
    
    // get inertial Rates
    Vector3 vpos = this->myBody->GetWorldLinearVel(); // get velocity in gazebo frame
    Vector3 veul = this->myBody->GetWorldAngularVel(); // get velocity in gazebo frame
    Vector3 frame_vpos;
    Vector3 frame_veul;
    if (this->myFrame)
    {
      frame_vpos = this->myFrame->GetWorldLinearVel(); // get velocity in gazebo frame
      frame_veul = this->myFrame->GetWorldAngularVel(); // get velocity in gazebo frame
    }

    // differentiate to get accelerations
    double tmp_dt = cur_time.Double() - this->last_time.Double();
    if (tmp_dt != 0)
    {
      this->apos = (this->last_vpos - vpos) / tmp_dt;
      this->aeul = (this->last_veul - veul) / tmp_dt;
      this->last_vpos = vpos;
      this->last_veul = veul;

      this->frame_apos = (this->last_frame_vpos - frame_vpos) / tmp_dt;
      this->frame_aeul = (this->last_frame_veul - frame_veul) / tmp_dt;
      this->last_frame_vpos = frame_vpos;
      this->last_frame_veul = frame_veul;

      this->lock.lock();

      if (this->topicName != "")
      {
        // copy data into pose message
        this->poseMsg.header.frame_id = this->frameName;  // @todo: should this be changeable?
        this->poseMsg.header.stamp.sec = cur_time.sec;
        this->poseMsg.header.stamp.nsec = cur_time.nsec;

        // pose is given in inertial frame for Gazebo, transform to the designated frame name
        // get relative pose in specified frame
        // @todo: account for offsets
        if (this->myFrame)
        {
          // get the relative transform from myFrame to myBody
          pos = (pos - frame_pos);  // linear offset from myFrame to myBody
          rot *= frame_rot.GetInverse();  // rotation between myFrame and myBody
        }
        this->poseMsg.pose.pose.position.x    = pos.x;
        this->poseMsg.pose.pose.position.y    = pos.y;
        this->poseMsg.pose.pose.position.z    = pos.z;

        this->poseMsg.pose.pose.orientation.x = rot.x;
        this->poseMsg.pose.pose.orientation.y = rot.y;
        this->poseMsg.pose.pose.orientation.z = rot.z;
        this->poseMsg.pose.pose.orientation.w = rot.u;

        // get relative rates then rotate into specified frame
        // @todo: account for offsets
        if (this->myFrame)
        {
          vpos = frame_rot.RotateVector(vpos - frame_vpos);
          veul = frame_rot.RotateVector(veul - frame_veul);
        }
        this->poseMsg.twist.twist.linear.x        = vpos.x + this->GaussianKernel(0,this->gaussianNoise) ;
        this->poseMsg.twist.twist.linear.y        = vpos.y + this->GaussianKernel(0,this->gaussianNoise) ;
        this->poseMsg.twist.twist.linear.z        = vpos.z + this->GaussianKernel(0,this->gaussianNoise) ;
        // pass euler angular rates
        this->poseMsg.twist.twist.angular.x    = veul.x + this->GaussianKernel(0,this->gaussianNoise) ;
        this->poseMsg.twist.twist.angular.y    = veul.y + this->GaussianKernel(0,this->gaussianNoise) ;
        this->poseMsg.twist.twist.angular.z    = veul.z + this->GaussianKernel(0,this->gaussianNoise) ;

        // fill in covariance matrix
        /// @todo: let user set separate linear and angular covariance values.
        this->poseMsg.pose.covariance[0] = this->gaussianNoise*this->gaussianNoise;
        this->poseMsg.pose.covariance[7] = this->gaussianNoise*this->gaussianNoise;
        this->poseMsg.pose.covariance[14] = this->gaussianNoise*this->gaussianNoise;
        this->poseMsg.pose.covariance[21] = this->gaussianNoise*this->gaussianNoise;
        this->poseMsg.pose.covariance[28] = this->gaussianNoise*this->gaussianNoise;
        this->poseMsg.pose.covariance[35] = this->gaussianNoise*this->gaussianNoise;

        this->poseMsg.twist.covariance[0] = this->gaussianNoise*this->gaussianNoise;
        this->poseMsg.twist.covariance[7] = this->gaussianNoise*this->gaussianNoise;
        this->poseMsg.twist.covariance[14] = this->gaussianNoise*this->gaussianNoise;
        this->poseMsg.twist.covariance[21] = this->gaussianNoise*this->gaussianNoise;
        this->poseMsg.twist.covariance[28] = this->gaussianNoise*this->gaussianNoise;
        this->poseMsg.twist.covariance[35] = this->gaussianNoise*this->gaussianNoise;

        // publish to ros
        this->pub_.publish(this->poseMsg);
      }

      this->lock.unlock();

      // save last time stamp
      this->last_time = cur_time;
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void GazeboRosP3D::FiniChild()
{
  if (this->myBody == NULL)
    return;
  this->rosnode_->shutdown();
#ifdef USE_CBQ
  this->p3d_queue_.clear();
  this->p3d_queue_.disable();
  this->callback_queue_thread_.join();
#endif
}

//////////////////////////////////////////////////////////////////////////////
// Utility for adding noise
double GazeboRosP3D::GaussianKernel(double mu,double sigma)
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
void GazeboRosP3D::P3DQueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->p3d_queue_.callAvailable(ros::WallDuration(timeout));
  }
}
#endif
