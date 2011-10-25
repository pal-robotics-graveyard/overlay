/*
 * Desc: A dynamic controller plugin that controls texture projection.
 * Author: Jared Duke
 * Date: 17 June 2010
 * SVN: $Id: gazebo_ros_projector.h 329 2011-05-17 09:39:57Z ricardo $
 */
#ifndef GAZEBO_ROS_PROJECTOR_HH
#define GAZEBO_ROS_PROJECTOR_HH

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>

#include <ros/ros.h>
#include "boost/thread/mutex.hpp"

#include <gazebo/Controller.hh>
#include <gazebo/Param.hh>
#include <gazebo/Body.hh>
#include <gazebo/Model.hh>

#include <std_msgs/String.h>
#include <std_msgs/Int32.h>

#include <OgrePrerequisites.h>
#include <OgreTexture.h>
#include <OgreFrameListener.h>

namespace Ogre
{
	class PlaneBoundedVolumeListSceneQuery;
	class Frustum;
	class Pass;
  class SceneNode;
}

namespace gazebo
{

	class Geom;
	
/// @addtogroup gazebo_dynamic_plugins Gazebo ROS Dynamic Plugins
/// @{
/** \defgroup GazeboRosProjector Plugin XML Reference and Example

  \brief Ros Texture Projector Controller.
  
  This is a controller that controls texture projection into the world from a given body.

  Example Usage:
  \verbatim
  <model:physical name="projector_model">
    <body:empty name="projector_body_name">
     ...
      <controller:gazebo_ros_projector_controller name="projector_controller" plugin="libgazebo_ros_projector.so">
        <alwaysOn>true</alwaysOn>
        <updateRate>15.0</updateRate>
        <textureName>stereo_projection_pattern_alpha.png</textureName>
        <filterTextureName>stereo_projection_pattern_filter.png</filterTextureName>        
        <textureTopicName>projector_controller/image</textureTopicName>
        <projectorTopicName>projector_controller/projector</projectorTopicName>        
        <fov>0.785398163</fov>
        <nearClipDist>0.1</nearClipDist>
        <farClipDist>10</farClipDist>
      </controller:gazebo_ros_projector_controller>

    </body:empty>
  </model:phyiscal>
  \endverbatim
 
\{
*/

class GazeboRosProjector : public Controller
{
  /// \brief Constructor
  /// \param parent The parent entity, must be a Model or a Sensor
  public: GazeboRosProjector(Entity *parent);

  /// \brief Destructor
  public: virtual ~GazeboRosProjector();

  /// \brief Load the controller
  /// \param node XML config node
  protected: virtual void LoadChild(XMLConfigNode *node);

  /// \brief Init the controller
  protected: virtual void InitChild();

  /// \brief Update the controller
  protected: virtual void UpdateChild();

  /// \brief Finalize the controller, unadvertise topics
  protected: virtual void FiniChild();

  /// \brief Callback when a texture is published
  private: void LoadImage(const std_msgs::String::ConstPtr& imageMsg);

  /// \brief Callbakc when a projector toggle is published
  private: void ToggleProjector(const std_msgs::Int32::ConstPtr& projectorMsg);	

  /// \brief Utility method for accessing the root Ogre object
  private: Ogre::Root *getRootP();
	
  /// \brief A pointer to the parent entity
  private: Model *myParent;
	//private: Entity *myParent;	

  /// \brief A pointer to the body entity
  private: Body *myBody;
	
  /// \brief A pointer to the ROS node.  A node will be instantiated if it does not exist.
  private: ros::NodeHandle* rosnode_;
  private: ros::Subscriber imageSubscriber_;
  private: ros::Subscriber projectorSubscriber_;	

	/// \brief A mutex to lock access to fields that are used in ROS message callbacks
  private: boost::mutex lock;

  /// \brief ROS Projector topic names
  /// \brief inputs
  private: ParamT<std::string> *textureTopicNameP;
  private: ParamT<std::string> *projectorTopicNameP;
  private: ParamT<std::string> *bodyNameP;
  private: ParamT<std::string> *textureNameP;
  private: ParamT<std::string> *filterTextureNameP;
  private: ParamT<double> *fovP;
  private: ParamT<double> *nearClipDistP;
  private: ParamT<double> *farClipDistP;

 /// \brief ROS texture topic name	
  private: std::string textureTopicName;

	/// \brief ROS projector topic name		
  private: std::string projectorTopicName;

	// \brief Projector parameters
  private: std::string bodyName;
  private: std::string textureName;
  private: std::string filterTextureName;		
  private: double fov;
  private: double nearClipDist;
  private: double farClipDist;
	
	/// \brief For setting ROS name space
  private: ParamT<std::string> *robotNamespaceP;
  private: std::string robotNamespace;

  // Custom Callback Queue
  private: ros::CallbackQueue queue_;
  private: void QueueThread();
  private: boost::thread callback_queue_thread_;

  private: std::string filterNodeName;

  private: 
	  class Projector : public Ogre::FrameListener
		{

		  public: Projector();
      public: virtual ~Projector();

			public: void init( Ogre::SceneNode *sceneNodePtr = NULL,
			                   Ogre::String textureName = "stereo_projection_pattern_alpha.png",
			                   Ogre::String filterTextureName = "stereo_projection_pattern_filter.png",			                   
			                   double nearDist = .5,
			                   double farDist = 10,
			                   double fov = 0.785398163,
                         std::string filterNodeName = "FilterNode" );

		  public: virtual bool frameStarted(const Ogre::FrameEvent &evt);
      public: virtual bool frameEnded(const Ogre::FrameEvent &evt);
		  public: virtual bool frameRenderingQueued(const Ogre::FrameEvent &evt);

   		public: void setEnabled( bool enabled );
   		public: void setUsingShaders( bool usingShaders );			
  		public: void setSceneNode( Ogre::SceneNode *sceneNodePtr );
  		public: void setTextureName( const Ogre::String& textureName );
  		public: void setFilterTextureName( const Ogre::String& textureName );			
  		public: void setFrustumClipDistance( double nearDist, double farDist );
  		public: void setFrustumFOV( double fovInRadians );
		                           
  		private: void addProjectorPassToVisibleMaterials();
  		private: void addProjectorPassToAllMaterials();
  		private: void addProjectorPassToMaterials(std::list<std::string>& matList);
  		private: void addProjectorPassToMaterial(std::string matName);
  		private: void removeProjectorPassFromMaterials();			
  		private: void removeProjectorPassFromMaterial(std::string matName);

  		private: Ogre::SceneManager* getSceneMgrP();
			
      private: bool isEnabled;
  		private: bool isInit;
  		private: bool isUsingShaders;

	  	private: Ogre::Frustum *projectorFrustum;
			private: Ogre::Frustum *projectorFilterFrustum;
  		private: Ogre::PlaneBoundedVolumeListSceneQuery *projectorQuery;
  		private: Ogre::SceneNode *projectorNode;
      private: Ogre::SceneNode *projectorFilterNode;

		  private: Ogre::String projectedTextureName;
  		private: Ogre::String projectedFilterTextureName;			

  		private: std::map<std::string,Ogre::Pass*> projectorTargets;
			
      private: std::string filterNodeName;
    };

  private: Projector projector_;

};

/** \} */
/// @}

}
#endif

