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
 * Desc: Empty gazebo plugin
 * Author: John Hsu
 * Date: 24 July 2009
 * SVN info: $Id: gazebo_ros_pub_world_state.cpp 329 2011-05-17 09:39:57Z ricardo $
 */


#include <algorithm>
#include <assert.h>

#include <gazebo_plugins/gazebo_ros_pub_world_state.h>

#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>
#include <boost/bind.hpp>

using namespace gazebo;

GZ_REGISTER_DYNAMIC_CONTROLLER("gazebo_ros_pub_world_state", GazeboRosPubWorldState);

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosPubWorldState::GazeboRosPubWorldState(Entity *parent)
    : Controller(parent)
{
  this->parent_model_ = dynamic_cast<Model*>(this->parent);

  if (!this->parent_model_)
    gzthrow("GazeboMechanismControl controller requires a Model as its parent");

  Param::Begin(&this->parameters);
  this->robotNamespaceP = new ParamT<std::string>("robotNamespace", "/", 0);
  this->topicNameP = new ParamT<std::string>("topicName", "", 1);
  this->frameNameP = new ParamT<std::string>("frameName", "base_link", 0);
  Param::End();

  this->worldStateConnectCount = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosPubWorldState::~GazeboRosPubWorldState()
{
  delete this->robotNamespaceP;
  delete this->topicNameP;
  delete this->frameNameP;
  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosPubWorldState::LoadChild(XMLConfigNode *node)
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

  // Custom Callback Queue
  ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<gazebo_plugins::WorldState>(
    this->topicName,1,
    boost::bind( &GazeboRosPubWorldState::WorldStateConnect,this),
    boost::bind( &GazeboRosPubWorldState::WorldStateDisconnect,this), ros::VoidPtr(), &this->queue_);
  this->pub_ = this->rosnode_->advertise(ao);

}

////////////////////////////////////////////////////////////////////////////////
// Someone subscribes to me
void GazeboRosPubWorldState::WorldStateConnect()
{
  this->worldStateConnectCount++;
}

////////////////////////////////////////////////////////////////////////////////
// Someone subscribes to me
void GazeboRosPubWorldState::WorldStateDisconnect()
{
  this->worldStateConnectCount--;
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void GazeboRosPubWorldState::InitChild()
{
  // Custom Callback Queue
  this->callback_queue_thread_ = boost::thread( boost::bind( &GazeboRosPubWorldState::QueueThread,this ) );
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosPubWorldState::UpdateChild()
{
  /***************************************************************/
  /*                                                             */
  /*  this is called at every update simulation step             */
  /*                                                             */
  /***************************************************************/
  if (this->worldStateConnectCount == 0)
    return;

  /***************************************************************/
  /*                                                             */
  /*  publish                                                    */
  /*                                                             */
  /***************************************************************/
  Time cur_time = Simulator::Instance()->GetSimTime();

  /// \bridf: list of all models in the world
  std::vector<gazebo::Model*> models;
  std::vector<gazebo::Model*>::iterator miter;

  /// \bridf: list of all bodies in the model
  std::map<std::string,gazebo::Body*> all_bodies;
  all_bodies.clear();

  models = gazebo::World::Instance()->GetModels();

  // aggregate all bodies into a single vector
  for (miter = models.begin(); miter != models.end(); miter++)
  {
    const std::vector<gazebo::Entity*> entities = (*miter)->GetChildren();
    // Iterate through all bodies
    std::vector<Entity*>::const_iterator eiter;
    for (eiter=entities.begin(); eiter!=entities.end(); eiter++)
    {
      gazebo::Body* body = dynamic_cast<gazebo::Body*>(*eiter);
      if (body)
        all_bodies.insert(make_pair(body->GetName(),body));
    }
  }
  //ROS_ERROR("debug: %d",all_bodies.size());

  // construct world state message
  if (!all_bodies.empty())
  {
    this->lock.lock();

    // compose worldStateMsg
    this->worldStateMsg.header.frame_id = this->frameName;
    this->worldStateMsg.header.stamp.fromSec(cur_time.Double());

    // Iterate through all_bodies
    std::map<std::string, Body*>::iterator biter;
    for (biter=all_bodies.begin(); biter!=all_bodies.end(); biter++)
    {
      //ROS_ERROR("body name: %s",(biter->second)->GetName().c_str());
      // get name
      this->worldStateMsg.name.push_back(std::string(biter->second->GetName()));

      // set pose
      // get pose from simulator
      Pose3d pose;
      Quatern rot;
      Vector3 pos;
      // Get Pose/Orientation ///@todo: verify correctness
      pose = (biter->second)->GetWorldPose();

      // apply xyz offsets and get position and rotation components
      pos = pose.pos; // (add if there's offset) + this->xyzOffsets;
      rot = pose.rot;
      // apply rpy offsets
      /* add if there's offsets
      Quatern qOffsets;
      qOffsets.SetFromEuler(this->rpyOffsets);
      rot = qOffsets*rot;
      rot.Normalize();
      */
      geometry_msgs::Pose geom_pose; 
      geom_pose.position.x    = pos.x;
      geom_pose.position.y    = pos.y;
      geom_pose.position.z    = pos.z;
      geom_pose.orientation.x = rot.x;
      geom_pose.orientation.y = rot.y;
      geom_pose.orientation.z = rot.z;
      geom_pose.orientation.w = rot.u;
      this->worldStateMsg.pose.push_back(geom_pose);

      // set velocities
      // get Rates
      Vector3 vpos = (biter->second)->GetWorldLinearVel(); // get velocity in gazebo frame
      Vector3 veul = (biter->second)->GetWorldAngularVel(); // get velocity in gazebo frame

      // pass linear rates
      geometry_msgs::Twist geom_twist;
      geom_twist.linear.x        = vpos.x;
      geom_twist.linear.y        = vpos.y;
      geom_twist.linear.z        = vpos.z;
      // pass euler angular rates
      geom_twist.angular.x    = veul.x;
      geom_twist.angular.y    = veul.y;
      geom_twist.angular.z    = veul.z;
      this->worldStateMsg.twist.push_back(geom_twist);

      // get forces
      Vector3 force = (biter->second)->GetWorldForce(); // get velocity in gazebo frame
      Vector3 torque = (biter->second)->GetWorldTorque(); // get velocity in gazebo frame
      geometry_msgs::Wrench geom_wrench;
      geom_wrench.force.x = force.x;
      geom_wrench.force.y = force.y;
      geom_wrench.force.z = force.z;
      geom_wrench.torque.x = torque.x;
      geom_wrench.torque.y = torque.y;
      geom_wrench.torque.z = torque.z;
      this->worldStateMsg.wrench.push_back(geom_wrench);

    }
    this->pub_.publish(this->worldStateMsg);
    this->lock.unlock();

  }
}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void GazeboRosPubWorldState::FiniChild()
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
void GazeboRosPubWorldState::QueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->queue_.callAvailable(ros::WallDuration(timeout));
  }
}


