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
   Desc: GazeboRosSimIface plugin for manipulating objects in Gazebo
   Author: John Hsu
   Date: 24 Sept 2008
   SVN info: $Id: gazebo_ros_sim_iface.cpp 329 2011-05-17 09:39:57Z ricardo $
 @htmlinclude manifest.html
 @b GazeboRosSimIface plugin reads ROS Odometry messages
 */

#include <algorithm>
#include <assert.h>

#include <gazebo_plugins/gazebo_ros_sim_iface.h>

#include <gazebo/Sensor.hh>
#include <gazebo/Model.hh>
#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/Simulator.hh>
#include <gazebo/gazebo.h>
#include <gazebo/World.hh>
#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>

using namespace gazebo;

GZ_REGISTER_DYNAMIC_CONTROLLER("gazebo_ros_sim_iface", GazeboRosSimIface);

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosSimIface::GazeboRosSimIface(Entity *parent)
    : Controller(parent)
{
  this->myParent = parent;

  if (!this->myParent)
    gzthrow("GazeboRosSimIface controller requires an Entity as its parent");

  // todo: make default topic and service names "gazebo/Model_Name/set_pose"
  Param::Begin(&this->parameters);
  this->robotNamespaceP = new ParamT<std::string>("robotNamespace", "/", 0);
  this->topicNameP = new ParamT<std::string>("topicName","gazebo/set_pose", 0);
  this->serviceNameP = new ParamT<std::string>("serviceName","gazebo/set_pose", 0);
  this->modelNameP = new ParamT<std::string>("modelName","pr2_model", 1);
  this->xyzP  = new ParamT<Vector3>("xyz" ,Vector3(0,0,0), 0);
  this->rpyP  = new ParamT<Vector3>("rpy" ,Vector3(0,0,0), 0);
  this->velP  = new ParamT<Vector3>("vel" ,Vector3(0,0,0), 0);
  this->angVelP  = new ParamT<Vector3>("angVel" ,Vector3(0,0,0), 0);
  Param::End();
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosSimIface::~GazeboRosSimIface()
{
  delete this->rosnode_;

  delete this->robotNamespaceP;
  delete this->topicNameP;
  delete this->serviceNameP;
  delete this->modelNameP;
  delete this->xyzP;
  delete this->rpyP;
  delete this->velP;
  delete this->angVelP;

}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosSimIface::LoadChild(XMLConfigNode *node)
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
  this->modelNameP->Load(node);
  this->xyzP->Load(node);
  this->rpyP->Load(node);
  this->velP->Load(node);
  this->angVelP->Load(node);

  this->topicName = this->topicNameP->GetValue();
  this->modelName = this->modelNameP->GetValue();
  this->xyz = this->xyzP->GetValue();
  this->rpy = this->rpyP->GetValue();
  this->vel = this->velP->GetValue();
  this->angVel = this->angVelP->GetValue();

  // Custom Callback Queue
  // topic callback version for position change
  ros::SubscribeOptions so = ros::SubscribeOptions::create<nav_msgs::Odometry>(
    this->topicName,1,
    boost::bind( &GazeboRosSimIface::UpdateObjectPose,this,_1),
    ros::VoidPtr(), &this->queue_);
  this->sub_ = this->rosnode_->subscribe(so);

#if ROS_SIM_IFACE_EXPOSE_SERVICE
  // add service call version for position change
  this->serviceNameP->Load(node);
  this->serviceName = this->serviceNameP->GetValue();
  // advertise services on the custom queue
  ros::AdvertiseServiceOptions aso = ros::AdvertiseServiceOptions::create<gazebo_plugins::SetPose>(
      this->serviceName,boost::bind( &GazeboRosSimIface::ServiceCallback, this, _1, _2 ), ros::VoidPtr(), &this->queue_);
  this->srv_ = this->rosnode_->advertiseService(aso);
#endif

}

////////////////////////////////////////////////////////////////////////////////
// Service callback
bool GazeboRosSimIface::ServiceCallback(gazebo_plugins::SetPose::Request &req,
                                        gazebo_plugins::SetPose::Response &res)
{
  Model* model = dynamic_cast<Model*>(gazebo::World::Instance()->GetEntityByName(this->modelName));
  if (model)
  {
    Vector3 target_pos(req.pose.pose.pose.position.x,req.pose.pose.pose.position.y,req.pose.pose.pose.position.z);
    Quatern target_rot(req.pose.pose.pose.orientation.w,req.pose.pose.pose.orientation.x,req.pose.pose.pose.orientation.y,req.pose.pose.pose.orientation.z);
    Pose3d target_pose(target_pos,target_rot);

    this->frameName = req.pose.header.frame_id;

    this->myFrame = NULL;
    /// if a relative Body frame is specified, set pose relative to this Body frame
    /// if frameName specified is "world", "/map" or "map", set relative to the gazebo inertial world
    if (this->frameName != "world" && this->frameName != "/map" && this->frameName != "map")
    {
      // lock in case a model is being spawned
      boost::recursive_mutex::scoped_lock lock(*Simulator::Instance()->GetMRMutex());
      // look through all models in the world, search for body name that matches frameName
      std::vector<Model*> all_models = World::Instance()->GetModels();
      for (std::vector<Model*>::iterator iter = all_models.begin(); iter != all_models.end(); iter++)
      {
        if (*iter) this->myFrame = dynamic_cast<Body*>((*iter)->GetBody(this->frameName));
        if (this->myFrame) break;
      }

      // not found
      if (this->myFrame == NULL)
      {
        ROS_DEBUG("gazebo_ros_sim_iface plugin: frame_id: %s does not exist in simulation, will not set pose\n",this->frameName.c_str());
        return false;
      }
    }

    // get reference frame (body(link)) pose and
    // transform target pose to absolute world frame
    if (this->myFrame)
    {
      Pose3d  frame_pose = this->myFrame->GetWorldPose(); // - this->myBody->GetCoMPose();
      Vector3 frame_pos = frame_pose.pos;
      Quatern frame_rot = frame_pose.rot;

      //std::cout << " debug : " << this->myFrame->GetName() << " : " << frame_pose << " : " << target_pose << std::endl;
      //target_pose = frame_pose + target_pose; // seems buggy, use my own
      target_pose.pos = frame_pos + frame_rot.RotateVector(target_pos);
      target_pose.rot *= frame_rot;
    }

    //std::cout << " debug : " << target_pose << std::endl;
    {
      //boost::recursive_mutex::scoped_lock lock(*Simulator::Instance()->GetMRMutex());
      Simulator::Instance()->SetPaused(true);
      this->lock.lock();

      model->SetWorldPose(target_pose);

      // set model velocity to 0
      Vector3 vel(0,0,0);
      model->SetLinearVel(vel);
      model->SetAngularVel(vel);

      this->lock.unlock();
      Simulator::Instance()->SetPaused(false);
    }

    return true;
  }
  else
  {
    res.status_message = "trying to set pose of a model that does not exist";
    return false;
  }
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void GazeboRosSimIface::InitChild()
{
  // Custom Callback Queue
  this->callback_queue_thread_ = boost::thread( boost::bind( &GazeboRosSimIface::QueueThread,this ) );
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosSimIface::UpdateObjectPose(const nav_msgs::Odometry::ConstPtr& poseMsg)
{
  Model* model = dynamic_cast<Model*>(gazebo::World::Instance()->GetEntityByName(this->modelName));
  if (model)
  {
    // target pose
    Vector3 target_pos(poseMsg->pose.pose.position.x,poseMsg->pose.pose.position.y,poseMsg->pose.pose.position.z);
    Quatern target_rot(poseMsg->pose.pose.orientation.w,poseMsg->pose.pose.orientation.x,poseMsg->pose.pose.orientation.y,poseMsg->pose.pose.orientation.z);
    Pose3d target_pose(target_pos,target_rot);

    this->frameName = poseMsg->header.frame_id;

    this->myFrame = NULL;
    // if a relative Body frame is specified, set pose relative to this Body frame
    if (this->frameName != "world")
    {
      // lock in case a model is being spawned
      boost::recursive_mutex::scoped_lock lock(*Simulator::Instance()->GetMRMutex());
      // look through all models in the world, search for body name that matches frameName
      std::vector<Model*> all_models = World::Instance()->GetModels();
      for (std::vector<Model*>::iterator iter = all_models.begin(); iter != all_models.end(); iter++)
      {
        if (*iter) this->myFrame = dynamic_cast<Body*>((*iter)->GetBody(this->frameName));
        if (this->myFrame) break;
      }

      // not found
      if (this->myFrame == NULL)
      {
        ROS_DEBUG("gazebo_ros_sim_iface plugin: frame_id: %s does not exist in simulation, will not set pose\n",this->frameName.c_str());
        return;
      }
    }

    // get reference frame (body(link)) pose and
    // transform target pose to absolute world frame
    if (this->myFrame)
    {
      Pose3d  frame_pose = this->myFrame->GetWorldPose(); // - this->myBody->GetCoMPose();
      Vector3 frame_pos = frame_pose.pos;
      Quatern frame_rot = frame_pose.rot;

      //target_pose = frame_pose + target_pose; // seems buggy, use my own 
      target_pose.pos = frame_pos + frame_rot.RotateVector(target_pos);
      target_pose.rot *= frame_rot;
    }



    //while (model->GetLinearVel() != vel || model->GetAngularVel() != vel)
    {
      Simulator::Instance()->SetPaused(true);
      this->lock.lock();

      model->SetWorldPose(target_pose);
      // set model velocity to 0
      Vector3 vel(0,0,0);
      boost::recursive_mutex::scoped_lock lock(*Simulator::Instance()->GetMRMutex());
      model->SetLinearVel(vel);
      model->SetAngularVel(vel);

      this->lock.unlock();
      Simulator::Instance()->SetPaused(false);
    }

  }
  else
  {
    ROS_INFO("Trying to set pose of a model [%s] that does not exist",this->modelName.c_str());
  }
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosSimIface::UpdateChild()
{



}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void GazeboRosSimIface::FiniChild()
{
  // Custom Callback Queue
  this->queue_.clear();
  this->queue_.disable();
  this->rosnode_->shutdown();
  this->callback_queue_thread_.join();
}

// Custom Callback Queue
////////////////////////////////////////////////////////////////////////////////
// custom callback queue thread
void GazeboRosSimIface::QueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->queue_.callAvailable(ros::WallDuration(timeout));
  }
}



