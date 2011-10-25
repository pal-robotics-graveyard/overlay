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
   Desc: Force Feed Back Ground Truth
   Author: Sachin Chitta and John Hsu
   Date: 1 June 2008
   SVN info: $Id: gazebo_ros_f3d.cpp 329 2011-05-17 09:39:57Z ricardo $
 @htmlinclude manifest.html
 @b GazeboRosF3D plugin broadcasts forces acting on the body specified by name.
 */

#include <gazebo_plugins/gazebo_ros_f3d.h>

#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/HingeJoint.hh>
#include <gazebo/SliderJoint.hh>
#include <gazebo/Simulator.hh>
#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>

using namespace gazebo;

GZ_REGISTER_DYNAMIC_CONTROLLER("gazebo_ros_f3d", GazeboRosF3D);


////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosF3D::GazeboRosF3D(Entity *parent )
   : Controller(parent)
{
  this->myParent = dynamic_cast<Model*>(this->parent);

  if (!this->myParent)
    gzthrow("GazeboRosF3D controller requires a Model as its parent");

  Param::Begin(&this->parameters);
  this->robotNamespaceP = new ParamT<std::string>("robotNamespace", "/", 0);
  this->bodyNameP = new ParamT<std::string>("bodyName", "", 0);
  this->topicNameP = new ParamT<std::string>("topicName", "", 1);
  this->frameNameP = new ParamT<std::string>("frameName", "default_f3d_frame", 0);
  Param::End();

  this->f3dConnectCount = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosF3D::~GazeboRosF3D()
{
  delete this->robotNamespaceP;
  delete this->bodyNameP;
  delete this->topicNameP;
  delete this->frameNameP;
  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosF3D::LoadChild(XMLConfigNode *node)
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
  this->myBody = dynamic_cast<Body*>(this->myParent->GetBody(bodyName));
  if (!this->myBody)
    ROS_FATAL("gazebo_ros_f3d plugin error: bodyName: %s does not exist\n",bodyName.c_str());

  this->topicNameP->Load(node);
  this->topicName = this->topicNameP->GetValue();
  this->frameNameP->Load(node);
  this->frameName = this->frameNameP->GetValue();

  // Custom Callback Queue
  ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<geometry_msgs::WrenchStamped>(
    this->topicName,1,
    boost::bind( &GazeboRosF3D::F3DConnect,this),
    boost::bind( &GazeboRosF3D::F3DDisconnect,this), ros::VoidPtr(), &this->queue_);
  this->pub_ = this->rosnode_->advertise(ao);
}

////////////////////////////////////////////////////////////////////////////////
// Someone subscribes to me
void GazeboRosF3D::F3DConnect()
{
  this->f3dConnectCount++;
}

////////////////////////////////////////////////////////////////////////////////
// Someone subscribes to me
void GazeboRosF3D::F3DDisconnect()
{
  this->f3dConnectCount--;
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void GazeboRosF3D::InitChild()
{
  // Custom Callback Queue
  this->callback_queue_thread_ = boost::thread( boost::bind( &GazeboRosF3D::QueueThread,this ) );
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosF3D::UpdateChild()
{
  if (this->f3dConnectCount == 0)
    return;

  Vector3 torque;
  Vector3 force;

  // get force on body
  force = this->myBody->GetWorldForce();

  // get torque on body
  torque = this->myBody->GetWorldTorque();

  this->lock.lock();
  // copy data into wrench message
  this->wrenchMsg.header.frame_id = this->frameName;
  this->wrenchMsg.header.stamp.sec = (Simulator::Instance()->GetSimTime()).sec;
  this->wrenchMsg.header.stamp.nsec = (Simulator::Instance()->GetSimTime()).nsec;

  this->wrenchMsg.wrench.force.x    = force.x;
  this->wrenchMsg.wrench.force.y    = force.y;
  this->wrenchMsg.wrench.force.z    = force.z;
  this->wrenchMsg.wrench.torque.x   = torque.x;
  this->wrenchMsg.wrench.torque.y   = torque.y;
  this->wrenchMsg.wrench.torque.z   = torque.z;

  this->pub_.publish(this->wrenchMsg);
  this->lock.unlock();

}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void GazeboRosF3D::FiniChild()
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
void GazeboRosF3D::QueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->queue_.callAvailable(ros::WallDuration(timeout));
  }
}
