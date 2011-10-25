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
 * Desc: 3D Applied Force Feedback Interface
 * Author: John Hsu
 * Date: 24 Sept 2008
 * SVN: $Id: gazebo_ros_f3d.h 329 2011-05-17 09:39:57Z ricardo $
 */
#ifndef GAZEBO_ROS_F3D_HH
#define GAZEBO_ROS_F3D_HH

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

#include <gazebo/Controller.hh>
#include <gazebo/Entity.hh>
#include <gazebo/Model.hh>
#include <gazebo/Body.hh>
#include <gazebo/Param.hh>

#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <geometry_msgs/WrenchStamped.h>

namespace gazebo
{

/// @addtogroup gazebo_dynamic_plugins Gazebo ROS Dynamic Plugins
/// @{
/** \defgroup GazeboRosF3D Applied Force State Plugin XML Reference and Example

  \brief FIXME: Applied Force Feedback controller.
  
  This is a controller that gathers applied force data from a Body and populates a libgazebo interfaace as well as publish a ROS geometry_msgs::WrenchStamped message (under topicName). This controller should only be used as a child of a Model.  Below is an example of the usage of this plugin.

  \verbatim
  <model:physical name="camera_model">
    <controller:gazebo_ros_f3d name="f3d_finger_tip_l_left_controller" plugin="libgazebo_ros_f3d.so">
        <alwaysOn>true</alwaysOn>
        <updateRate>100.0</updateRate>
        <bodyName>finger_tip_l_left</bodyName>
        <topicName>finger_tip_l_left_ground_truth</topicName>
        <frameName>map</frameName>
        <interface:position name="f3d_finger_tip_l_left_position" />
    </controller:gazebo_ros_f3d>
  </model:phyiscal>
  \endverbatim
 
\{
*/

/// \brief GazeboRosF3D controller
/// This is a controller that simulates a 6 dof force sensor
class GazeboRosF3D : public Controller
{
  /// \brief Constructor
  /// \param parent The parent entity must be a Model
  public: GazeboRosF3D(Entity *parent);

  /// \brief Destructor
  public: virtual ~GazeboRosF3D();

  /// \brief Load the controller
  /// \param node XML config node
  protected: virtual void LoadChild(XMLConfigNode *node);

  /// \brief Init the controller
  protected: virtual void InitChild();

  /// \brief Update the controller
  protected: virtual void UpdateChild();

  /// \brief Finalize the controller
  protected: virtual void FiniChild();

  /// \brief A link to the parent Model
  private: Model *myParent;

  /// \brief A pointer to the Gazebo Body
  private: Body *myBody;


  /// \brief A pointer to the ROS node.  A node will be instantiated if it does not exist.
  private: ros::NodeHandle* rosnode_;
  private: ros::Publisher pub_;

  /// \brief ROS WrenchStamped message
  private: geometry_msgs::WrenchStamped wrenchMsg;

  /// \brief store bodyname
  private: ParamT<std::string> *bodyNameP;
  private: std::string bodyName;

  /// \brief ROS WrenchStamped topic name
  private: ParamT<std::string> *topicNameP;
  private: std::string topicName;

  /// \brief ROS frame transform name to use in the image message header.
  ///        This should be simply map since the returned info is in Gazebo Global Frame.
  private: ParamT<std::string> *frameNameP;
  private: std::string frameName;

  /// \brief for setting ROS name space
  private: ParamT<std::string> *robotNamespaceP;
  private: std::string robotNamespace;

  /// \brief A mutex to lock access to fields that are used in message callbacks
  private: boost::mutex lock;

  /// \brief: keep track of number of connections
  private: int f3dConnectCount;
  private: void F3DConnect();
  private: void F3DDisconnect();

  // Custom Callback Queue
  private: ros::CallbackQueue queue_;
  private: void QueueThread();
  private: boost::thread callback_queue_thread_;

};

/** \} */
/// @}


}

#endif

