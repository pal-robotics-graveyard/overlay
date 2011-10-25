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
   Desc: GazeboRosForce plugin for manipulating objects in Gazebo
   Author: John Hsu
   Date: 24 Sept 2008
   SVN info: $Id: gazebo_ros_force.cpp 329 2011-05-17 09:39:57Z ricardo $
 @htmlinclude manifest.html
 @b GazeboRosForce plugin reads ROS geometry_msgs/Wrench messages
 */

#include <algorithm>
#include <assert.h>

#include <gazebo_plugins/gazebo_ros_force.h>

#include <gazebo/Sensor.hh>
#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/Simulator.hh>
#include <gazebo/gazebo.h>
#include <gazebo/World.hh>
#include <gazebo/PhysicsEngine.hh>
#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>

using namespace gazebo;

GZ_REGISTER_DYNAMIC_CONTROLLER("gazebo_ros_force", GazeboRosForce);

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosForce::GazeboRosForce(Entity *parent)
    : Controller(parent)
{
  this->myParent = dynamic_cast<Model*>(this->parent);

  if (!this->myParent)
    gzthrow("GazeboRosForce controller requires an Model as its parent");

  Param::Begin(&this->parameters);
  this->robotNamespaceP = new ParamT<std::string>("robotNamespace", "/", 0);
  this->topicNameP = new ParamT<std::string>("topicName","", 1);
  this->bodyNameP = new ParamT<std::string>("bodyName","link", 1);
  Param::End();

  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc,argv,"gazebo",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  }

  this->rosnode_ = new ros::NodeHandle();

  this->wrench.force.x = 0;
  this->wrench.force.y = 0;
  this->wrench.force.z = 0;
  this->wrench.torque.x = 0;
  this->wrench.torque.y = 0;
  this->wrench.torque.z = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosForce::~GazeboRosForce()
{
  delete this->robotNamespaceP;
  delete this->rosnode_;

  delete this->topicNameP;
  delete this->bodyNameP;

}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosForce::LoadChild(XMLConfigNode *node)
{
  this->robotNamespaceP->Load(node);
  this->robotNamespace = this->robotNamespaceP->GetValue();

  int argc = 0;
  char** argv = NULL;
  ros::init(argc,argv,"gazebo");
  this->rosnode_ = new ros::NodeHandle(this->robotNamespace);

  this->topicNameP->Load(node);
  this->bodyNameP->Load(node);

  this->topicName = this->topicNameP->GetValue();
  this->bodyName = this->bodyNameP->GetValue();


  // assert that the body by bodyName exists
  if (dynamic_cast<Body*>(this->myParent->GetBody(bodyName)) == NULL)
    ROS_FATAL("gazebo_ros_force plugin error: bodyName: %s does not exist\n",bodyName.c_str());

  this->myBody = dynamic_cast<Body*>(this->myParent->GetBody(bodyName));

  // check update rate against world physics update rate
  // should be equal or higher to guarantee the wrench applied is not "diluted"
  if (this->updatePeriod > 0 &&
      //(gazebo::World::Instance()->GetPhysicsEngine()->GetUpdateRate() > 1.0/this->updatePeriod.Double()))
      (gazebo::World::Instance()->GetPhysicsEngine()->GetUpdateRate() > 1.0/this->updatePeriod))
    ROS_ERROR("gazebo_ros_force controller update rate is less than physics update rate, wrench applied will be diluted (applied intermittently)");

  // Custom Callback Queue
  ros::SubscribeOptions so = ros::SubscribeOptions::create<geometry_msgs::Wrench>(
    this->topicName,1,
    boost::bind( &GazeboRosForce::UpdateObjectForce,this,_1),
    ros::VoidPtr(), &this->queue_);
  this->sub_ = this->rosnode_->subscribe(so);
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void GazeboRosForce::InitChild()
{
  // Custom Callback Queue
  this->callback_queue_thread_ = boost::thread( boost::bind( &GazeboRosForce::QueueThread,this ) );
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosForce::UpdateObjectForce(const geometry_msgs::Wrench::ConstPtr& wrenchMsg)
{
  this->wrench.force.x = wrenchMsg->force.x;
  this->wrench.force.y = wrenchMsg->force.y;
  this->wrench.force.z = wrenchMsg->force.z;
  this->wrench.torque.x = wrenchMsg->torque.x;
  this->wrench.torque.y = wrenchMsg->torque.y;
  this->wrench.torque.z = wrenchMsg->torque.z;
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosForce::UpdateChild()
{

  this->lock.lock();
  Vector3 force(this->wrench.force.x,this->wrench.force.y,this->wrench.force.z);
  Vector3 torque(this->wrench.torque.x,this->wrench.torque.y,this->wrench.torque.z);
  this->myBody->SetForce(force);
  this->myBody->SetTorque(torque);
  this->lock.unlock();

}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void GazeboRosForce::FiniChild()
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
void GazeboRosForce::QueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->queue_.callAvailable(ros::WallDuration(timeout));
  }
}

