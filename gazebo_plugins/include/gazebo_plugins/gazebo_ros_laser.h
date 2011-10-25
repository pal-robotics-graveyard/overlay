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
 * Desc: ROS laser controller.
 * Author: John Hsu
 * Date: 24 Sept 2007
 * SVN: $Id: gazebo_ros_laser.h 329 2011-05-17 09:39:57Z ricardo $
 */

#ifndef GAZEBO_ROS_LASER_HH
#define GAZEBO_ROS_LASER_HH

#define USE_CBQ
#ifdef USE_CBQ
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#endif

#include <gazebo/Controller.hh>
#include <gazebo/Param.hh>

#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <sensor_msgs/LaserScan.h>

namespace gazebo
{
  class RaySensor;

/// @addtogroup gazebo_dynamic_plugins Gazebo ROS Dynamic Plugins
/// @{
/** \defgroup GazeboRosLaser ROS Laser Scanner Controller Plugin

  \brief ROS Laser Scanner Controller Plugin
  
  This controller gathers range data from a simulated ray sensor, publishes range data through
    sensor_msgs::LaserScan ROS topic.

  Example Usage:
  \verbatim
    <model:physical name="ray_model">
      <body:empty name="ray_body_name">
        <sensor:ray name="ray_sensor">
          <origin>0.0 0.0 0.0</origin>
          <rayCount>683</rayCount>
          <rangeCount>683</rangeCount>
          <laserCount>1</laserCount>
          <displayRays>false</displayRays>
          <minAngle>-45</minAngle>
          <maxAngle> 45</maxAngle>
          <minRange>0.05</minRange>
          <maxRange>10.0</maxRange>
          <updateRate>10.0</updateRate>
          <controller:gazebo_ros_laser name="ros_ray_sensor_controller" plugin="libgazebo_ros_laser.so">
            <gaussianNoise>0.005</gaussianNoise>
            <alwaysOn>true</alwaysOn>
            <updateRate>15.0</updateRate>
            <topicName>ray_scan</topicName>
            <frameName>ray_model</frameName>
            <interface:laser name="ros_ray_sensor_iface" />
          </controller:gazebo_ros_laser>
        </sensor:ray>
      </body:empty>
    </model:phyiscal>
  \endverbatim
 
\{
*/

/**
    \brief ROS laser scan controller.
           \li Starts a ROS node if none exists.
           \li Simulates a laser range sensor and publish sensor_msgs::LaserScan.msg over ROS.
           \li Example Usage:
  \verbatim
    <model:physical name="ray_model">
      <body:empty name="ray_body_name">
        <sensor:ray name="ray_sensor">
          <origin>0.0 0.0 0.0</origin>
          <rayCount>683</rayCount>
          <rangeCount>683</rangeCount>
          <laserCount>1</laserCount>
          <displayRays>false</displayRays>
          <minAngle>-45</minAngle>
          <maxAngle> 45</maxAngle>
          <minRange>0.05</minRange>
          <maxRange>10.0</maxRange>
          <updateRate>10.0</updateRate>
          <controller:gazebo_ros_laser name="ros_ray_sensor_controller" plugin="libgazebo_ros_laser.so">
            <gaussianNoise>0.005</gaussianNoise>
            <alwaysOn>true</alwaysOn>
            <updateRate>15.0</updateRate>
            <topicName>ray_scan</topicName>
            <frameName>ray_model</frameName>
            <interface:laser name="ros_ray_sensor_iface" />
          </controller:gazebo_ros_laser>
        </sensor:ray>
      </body:empty>
    </model:phyiscal>
  \endverbatim
           .
*/

class GazeboRosLaser : public Controller
{
  /// \brief Constructor
  /// \param parent The parent entity, must be a Model or a Sensor
  public: GazeboRosLaser(Entity *parent);

  /// \brief Destructor
  public: virtual ~GazeboRosLaser();

  /// \brief Load the controller
  /// \param node XML config node
  protected: virtual void LoadChild(XMLConfigNode *node);

  /// \brief Init the controller
  protected: virtual void InitChild();

  /// \brief Update the controller
  protected: virtual void UpdateChild();

  /// \brief Finalize the controller
  protected: virtual void FiniChild();

  /// \brief Put laser data to the ROS topic
  private: void PutLaserData();

  /// \brief Keep track of number of connctions
  private: int laserConnectCount;
  private: void LaserConnect();
  private: void LaserDisconnect();
  private: int deprecatedLaserConnectCount;
  private: void DeprecatedLaserConnect();
  private: void DeprecatedLaserDisconnect();

  /// \brief The parent sensor
  private: RaySensor *myParent;

  /// \brief pointer to ros node
  private: ros::NodeHandle* rosnode_;
  private: ros::Publisher pub_;
  private: ros::Publisher deprecated_pub_;

  /// \brief ros message
  private: sensor_msgs::LaserScan laserMsg;
 
  /// \brief topic name
  private: ParamT<std::string> *topicNameP;
  private: std::string topicName;
  private: ParamT<std::string> *deprecatedTopicNameP;
  private: std::string deprecatedTopicName;

  /// \brief frame transform name, should match link name
  private: ParamT<std::string> *frameNameP;
  private: std::string frameName;

  /// \brief Gaussian noise
  private: ParamT<double> *gaussianNoiseP;
  private: double gaussianNoise;

  /// \brief Gaussian noise generator
  private: double GaussianKernel(double mu,double sigma);

  /// \brief A mutex to lock access to fields that are used in message callbacks
  private: boost::mutex lock;

  /// \brief hack to mimic hokuyo intensity cutoff of 100
  private: ParamT<double> *hokuyoMinIntensityP;
  private: double hokuyoMinIntensity;

  /// \brief for setting ROS name space
  private: ParamT<std::string> *robotNamespaceP;
  private: std::string robotNamespace;

#ifdef USE_CBQ
  private: ros::CallbackQueue laser_queue_;
  private: void LaserQueueThread();
  private: boost::thread callback_queue_thread_;
#endif

};

/** \} */
/// @}

}

#endif

