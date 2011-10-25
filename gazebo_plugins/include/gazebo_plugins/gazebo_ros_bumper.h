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
 * Desc: Bumper Controller
 * Author: Nate Koenig mod by John Hsu
 * Date: 24 Sept 2008
 */
#ifndef GAZEBO_ROS_BUMPER_CONTROLLER_HH
#define GAZEBO_ROS_BUMPER_CONTROLLER_HH

#define USE_CBQ
#ifdef USE_CBQ
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#endif

#include <sys/time.h>

#include <gazebo/Controller.hh>
#include <gazebo/Entity.hh>
#include <gazebo/Param.hh>

// ros messages
#include <ros/ros.h>
#include "boost/thread/mutex.hpp"
#include <std_msgs/String.h>

#include <gazebo_plugins/ContactState.h>
#include <gazebo_plugins/ContactsState.h>

namespace gazebo
{
  class ContactSensor;

  /// \addtogroup gazebo_dynamic_plugins Gazebo ROS Dynamic Plugins
  /// \{
  /** \defgroup GazeboRosBumper Ros Bumper Plugin
  
    \brief A controller that returns bump contacts

  \verbatim
    <model:physical name="camera_model">
      <body:empty name="camera_body_name">
          <sensor:contact name="finger_tip_l_left_contact_sensor">
          <updateRate>15.0</updateRate>
          <geom>pr2_finger_tip_l_collision_geom</geom>
          <controller:gazebo_ros_bumper name="finger_tip_l_contact_controller" plugin="libgazebo_ros_bumper.so">
            <alwaysOn>true</alwaysOn>
            <updateRate>15.0</updateRate>
            <topicName>finger_tip_l_contact</topicName>
            <frameName>finger_tip_l_contact</frameName>
            <interface:bumper name="dummy_bumper_iface" />
          </controller:gazebo_ros_bumper>
        </sensor:contact>
      </body:empty>
    </model:phyiscal>
  \endverbatim

    \{
  */
  
  /// \brief A Bumper controller
  class GazeboRosBumper : public Controller
  {
    /// Constructor
      public: GazeboRosBumper(Entity *parent );
  
    /// Destructor
      public: virtual ~GazeboRosBumper();
  
    /// Load the controller
    /// \param node XML config node
    protected: virtual void LoadChild(XMLConfigNode *node);
  
    /// Init the controller
    protected: virtual void InitChild();
  
    /// Update the controller
    protected: virtual void UpdateChild();
  
    /// Finalize the controller
    protected: virtual void FiniChild();
  
    /// The parent Model
    private: ContactSensor *myParent;

    /// \brief Return information in this body (link) coordinate
    ///        if unavailable, use "/map" and global gazebo world frame
    private: Body *myFrame;

    /// \brief pointer to ros node
    private: ros::NodeHandle* rosnode_;
    private: ros::Publisher contact_pub_;

    /// \brief set topic name of broadcast
    private: ParamT<std::string> *bumperTopicNameP;
    private: std::string bumperTopicName;

    private: ParamT<std::string> *frameNameP;
    private: std::string frameName;

    /// \brief A mutex to lock access to fields that are used in message callbacks
    private: boost::mutex lock;

    /// \brief broadcast some string for now.
    private: gazebo_plugins::ContactsState contactsStateMsg;

    /// \brief for setting ROS name space
    private: ParamT<std::string> *robotNamespaceP;
    private: std::string robotNamespace;

    /// \brief Keep track of number of connctions
    private: int contactConnectCount;
    private: void ContactConnect();
    private: void ContactDisconnect();

#ifdef USE_CBQ
    private: ros::CallbackQueue contact_queue_;
    private: void ContactQueueThread();
    private: boost::thread callback_queue_thread_;
#endif

  };
  
  /** \} */
  /// \}

}

#endif

