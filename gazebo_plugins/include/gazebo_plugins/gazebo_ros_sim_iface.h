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
 * Desc: A dynamic controller plugin that publishes ROS Odometry topic for generic simiface interface.  Puts body at the specified Pose (rate) etc.  Only pose implemented for now.
 * Author: John Hsu
 * Date: 24 Sept 2008
 * SVN: $Id: gazebo_ros_sim_iface.h 329 2011-05-17 09:39:57Z ricardo $
 */
#ifndef GAZEBO_ROS_SIM_IFACE_HH
#define GAZEBO_ROS_SIM_IFACE_HH

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>

#include <ros/ros.h>
#include "boost/thread/mutex.hpp"
#include <gazebo/Controller.hh>
#include <gazebo/Param.hh>
#include <nav_msgs/Odometry.h>

#include <gazebo/Body.hh>

#define ROS_SIM_IFACE_EXPOSE_SERVICE 1

#if ROS_SIM_IFACE_EXPOSE_SERVICE
#include <gazebo_plugins/SetPose.h>
#endif


namespace gazebo
{

/// @addtogroup gazebo_dynamic_plugins Gazebo ROS Dynamic Plugins
/// @{
/** \defgroup GazeboRosSimIface Plugin XML Reference and Example

  \brief Ros SimIface Controller.
  
  This is a controller that collects data from a ROS topic and positions a body accordingly.

  Example Usage (FIXME):
  \verbatim
  <model:physical name="camera_model">
    <body:empty name="camera_body_name">
      <sensor:camera name="camera_sensor">
        <controller:ros_camera name="controller-name" plugin="libros_camera.so">
            <alwaysOn>true</alwaysOn>
            <updateRate>15.0</updateRate>
            <topicName>camera_name/image</topicName>
        </controller:ros_camera>
      </sensor:camera>
    </body:empty>
  </model:phyiscal>
  \endverbatim
 
\{
*/

/**
           .
 
*/

class GazeboRosSimIface : public Controller
{
  /// \brief Constructor
  /// \param parent The parent entity, must be a Model or a Sensor
  public: GazeboRosSimIface(Entity *parent);

  /// \brief Destructor
  public: virtual ~GazeboRosSimIface();

  /// \brief Load the controller
  /// \param node XML config node
  protected: virtual void LoadChild(XMLConfigNode *node);

  /// \brief Init the controller
  protected: virtual void InitChild();

  /// \brief Update the controller
  protected: virtual void UpdateChild();

  /// \brief Finalize the controller, unadvertise topics
  protected: virtual void FiniChild();

  /// \brief call back when a Odometry message is published
  private: void UpdateObjectPose(const nav_msgs::Odometry::ConstPtr& poseMsg);

  /// \brief call back when using service
  private: bool ServiceCallback(gazebo_plugins::SetPose::Request &req,
                                gazebo_plugins::SetPose::Response &res);

  /// \brief A pointer to the parent entity
  private: Entity *myParent;

  /// \brief A pointer to the ROS node.  A node will be instantiated if it does not exist.
  private: ros::NodeHandle* rosnode_;
  private: ros::Subscriber sub_;
  private: ros::ServiceServer srv_;

  /// \brief A mutex to lock access to fields that are used in ROS message callbacks
  private: boost::mutex lock;

  /// \brief ROS Pose topic name
  /// \brief ROS frame transform name to use in the pose message header.
  ///        This should typically match the link name the sensor is attached.
  /// \brief inputs
  private: ParamT<std::string> *topicNameP,*modelNameP;
  private: ParamT<Vector3> *xyzP,*rpyP,*velP,*angVelP;
  private: std::string topicName,frameName,modelName;
  private: Vector3 xyz,rpy,vel,angVel;
  private: ParamT<std::string> *serviceNameP;
  private: std::string serviceName;

  /// \brief for setting ROS name space
  private: ParamT<std::string> *robotNamespaceP;
  private: std::string robotNamespace;

  // Custom Callback Queue
  private: ros::CallbackQueue queue_;
  private: void QueueThread();
  private: boost::thread callback_queue_thread_;
  private: Body* myFrame;

};

/** \} */
/// @}

}
#endif

