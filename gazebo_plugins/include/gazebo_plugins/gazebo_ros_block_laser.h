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
 * Desc: ros laser controller.
 * Author: Nathan Koenig
 * Date: 01 Feb 2007
 * SVN: $Id: gazebo_ros_block_laser.h 329 2011-05-17 09:39:57Z ricardo $
 */

#ifndef GAZEBO_ROS_BLOCK_LASER_HH
#define GAZEBO_ROS_BLOCK_LASER_HH

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

#include <gazebo/Controller.hh>

#include <ros/ros.h>
#include "boost/thread/mutex.hpp"
#include <sensor_msgs/PointCloud.h>
#include <gazebo/Param.hh>

namespace gazebo
{
  class RaySensor;

/// @addtogroup gazebo_dynamic_plugins Gazebo ROS Dynamic Plugins
/// @{
/** \defgroup GazeboRosBlockLaser ROS Block Laser Scanner Controller Plugin

  \brief ROS Block Laser Scanner Controller Plugin
  
  This is a controller that gathers range data from a ray sensor, and returns results via publishing ROS topic for point clouds.

  Example Usage:
  \verbatim
    <model:physical name="ray_model">
      <body:empty name="ray_body_name">
        <sensor:ray name="ray_sensor">
          <rayCount>30</rayCount>
          <rangeCount>30</rangeCount>
          <laserCount>1</laserCount>
          
          <origin>0.0 0.0 0.05</origin>
          <displayRays>false</displayRays>
          
          <minAngle>-15</minAngle>
          <maxAngle> 15</maxAngle>
          
          <minRange>0.05</minRange>
          <maxRange>100.0</maxRange>
          <updateRate>10.0</updateRate>

          <verticalRayCount>30</verticalRayCount>
          <verticalRangeCount>30</verticalRangeCount>
          <verticalMinAngle>-20</verticalMinAngle>
          <verticalMaxAngle>  0</verticalMaxAngle>

          <controller:gazebo_ros_block_laser name="ray_block_controller" plugin="libgazebo_ros_block_laser.so">
            <gaussianNoise>0.005</gaussianNoise>
            <alwaysOn>true</alwaysOn>
            <updateRate>10.0</updateRate>
            <topicName>full_cloud</topicName>
            <frameName>ray_model</frameName>
            <interface:laser name="ray_block_iface" />
          </controller:gazebo_ros_block_laser>
        </sensor:ray>
      </body:empty>
    </model:phyiscal>
  \endverbatim
 
\{
*/

/**
 \brief ROS laser block simulation.
        \li Starts a ROS node if none exists.
        \li This controller simulates a block of laser range detections.
            Resulting point cloud (sensor_msgs::PointCloud.msg) is published as a ROS topic.
        \li Example Usage:
  \verbatim
    <model:physical name="ray_model">
      <body:empty name="ray_body_name">
        <sensor:ray name="ray_sensor">
          <rayCount>30</rayCount>
          <rangeCount>30</rangeCount>
          <laserCount>1</laserCount>
          
          <origin>0.0 0.0 0.05</origin>
          <displayRays>false</displayRays>
          
          <minAngle>-15</minAngle>
          <maxAngle> 15</maxAngle>
          
          <minRange>0.05</minRange>
          <maxRange>100.0</maxRange>
          <updateRate>10.0</updateRate>

          <verticalRayCount>30</verticalRayCount>
          <verticalRangeCount>30</verticalRangeCount>
          <verticalMinAngle>-20</verticalMinAngle>
          <verticalMaxAngle>  0</verticalMaxAngle>

          <controller:gazebo_ros_block_laser name="ray_block_controller" plugin="libgazebo_ros_block_laser.so">
            <gaussianNoise>0.005</gaussianNoise>
            <alwaysOn>true</alwaysOn>
            <updateRate>10.0</updateRate>
            <topicName>full_cloud</topicName>
            <frameName>ray_model</frameName>
            <interface:laser name="ray_block_iface" />
          </controller:gazebo_ros_block_laser>
        </sensor:ray>
      </body:empty>
    </model:phyiscal>
  \endverbatim
        .
*/

class GazeboRosBlockLaser : public Controller
{
  /// \brief Constructor
  /// \param parent The parent entity, must be a Model or a Sensor
  public: GazeboRosBlockLaser(Entity *parent);

  /// \brief Destructor
  public: virtual ~GazeboRosBlockLaser();

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

  /// \brief The parent sensor
  private: RaySensor *myParent;

  /// \brief pointer to ros node
  private: ros::NodeHandle* rosnode_;
  private: ros::Publisher pub_;

  /// \brief ros message
  private: sensor_msgs::PointCloud cloudMsg;
 
  /// \brief topic name
  private: ParamT<std::string> *topicNameP;
  private: std::string topicName;

  /// \brief frame transform name, should match link name
  /// \brief FIXME: extract link name directly?
  private: ParamT<std::string> *frameNameP;
  private: std::string frameName;

  /// \brief Gaussian noise
  private: ParamT<double> *gaussianNoiseP;
  private: double gaussianNoise;

  /// \brief Gaussian noise generator
  private: double GaussianKernel(double mu,double sigma);

  /// \brief A mutex to lock access to fields that are used in message callbacks
  private: boost::mutex lock;


  /// \brief for setting ROS name space
  private: ParamT<std::string> *robotNamespaceP;
  private: std::string robotNamespace;

  // Custom Callback Queue
  private: ros::CallbackQueue queue_;
  private: void QueueThread();
  private: boost::thread callback_queue_thread_;

};

/** \} */
/// @}

}

#endif

