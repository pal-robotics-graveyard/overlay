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
 * Desc: Actuator array controller for a Pr2 robot.
 * Author: Nathan Koenig
 * Date: 19 Sept 2007
 * SVN: $Id: gazebo_ros_step_world_state.h 329 2011-05-17 09:39:57Z ricardo $
 */
#ifndef GAZEBO_ROS_STEP_WORLD_STATE_HH
#define GAZEBO_ROS_STEP_WORLD_STATE_HH

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>

#include <gazebo/Controller.hh>
#include <gazebo/Body.hh>
#include <gazebo/Model.hh>
#include <gazebo/World.hh>
#include <gazebo/Simulator.hh>
#include <gazebo_plugins/WorldState.h>

#include <ros/ros.h>
#include <boost/thread.hpp>

#include <map>
#include <vector>
#include <string>

namespace gazebo
{
/// @addtogroup gazebo_dynamic_plugins Gazebo Dynamic Plugins
/// @{
/** \defgroup GazeboRosStepWorldState

  \brief A sample gazebo dynamic plugin
  
  This is a gazebo controller that does nothing

  Example Usage:
  \verbatim
    <model:physical name="robot_model1">

    </model:physical>
  \endverbatim
 
\{
*/

class GazeboRosStepWorldState : public Controller
{
  /// \brief Constructor
  /// \param parent The parent entity, must be a Model or a Sensor
  public: GazeboRosStepWorldState(Entity *parent);

  /// \brief Destructor
  public: virtual ~GazeboRosStepWorldState();

  /// \brief Load the controller
  /// \param node XML config node
  protected: virtual void LoadChild(XMLConfigNode *node);

  /// \brief Init the controller
  protected: virtual void InitChild();

  /// \brief Update the controller
  protected: virtual void UpdateChild();

  /// \brief Finalize the controller
  protected: virtual void FiniChild();

  /// \brief: Message for sending world state
  private: gazebo_plugins::WorldState worldStateMsg;

  /// \brief: Keep list of all models in the world
  private: const std::map<std::string,gazebo::Body*> *bodies;

  /// \brief: parent should be a model
  private: gazebo::Model* parent_model_;

  /// \brief: keep list of all models in the world
  private: std::vector<gazebo::Model*> models;

  /// \brief: keep list of all bodies in the world (across models)
  private: std::map<std::string,gazebo::Body*> all_bodies;

  /// \brief: ros node handle and publisher
  private: ros::NodeHandle* rosnode_;
  private: ros::Subscriber sub_;
  private: void WorldStateCallback(const gazebo_plugins::WorldStateConstPtr& worldStateMsg);

  /// \brief for setting ROS name space
  private: ParamT<std::string> *robotNamespaceP;
  private: std::string robotNamespace;

  /// \brief topic name
  private: ParamT<std::string> *topicNameP;
  private: std::string topicName;

  /// \brief frame transform name, should match link name
  private: ParamT<std::string> *frameNameP;
  private: std::string frameName;

  // Custom Callback Queue
  private: ros::CallbackQueue queue_;
  private: void QueueThread();
  private: boost::thread callback_queue_thread_;

};

/** \} */
/// @}

}
#endif

