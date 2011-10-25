/*
 @mainpage
   Desc: GazeboRosTextureProjector plugin that controls texture projection
   Author: Jared Duke
   Date: 17 Jun 2010
   SVN info: $Id: gazebo_ros_projector.cpp 329 2011-05-17 09:39:57Z ricardo $
 @htmlinclude manifest.html
 @b GazeboRosTextureProjector plugin that controls texture projection
 */

#include <algorithm>
#include <assert.h>
#include <utility>
#include <sstream>

#include <gazebo_plugins/gazebo_ros_projector.h>

#include <gazebo/OgreAdaptor.hh>
#include <gazebo/OgreVisual.hh>
#include <gazebo/Sensor.hh>
#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/Simulator.hh>
#include <gazebo/gazebo.h>
#include <gazebo/World.hh>
#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>

#include "std_msgs/String.h"
#include "std_msgs/Int32.h"

#include <Ogre.h>
#include <OgreMath.h>
#include <OgreSceneNode.h>
#include <OgreFrustum.h>
#include <OgreSceneQuery.h>

using namespace gazebo;

typedef std::map<std::string,Ogre::Pass*> OgrePassMap;
typedef OgrePassMap::iterator OgrePassMapIterator;

GZ_REGISTER_DYNAMIC_CONTROLLER("gazebo_ros_projector", GazeboRosProjector);

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosProjector::GazeboRosProjector(Entity *parent)
    : Controller(parent)
{
	this->myParent = dynamic_cast<Model*>(this->parent);
  if (!this->myParent)
	  gzthrow("GazeboRosProjector controller requires a Model as its parent");

  Param::Begin(&this->parameters);
  this->robotNamespaceP = new ParamT<std::string>("robotNamespace", "/", 0);
  this->textureTopicNameP = new ParamT<std::string>("textureTopicName","", 1);
  this->projectorTopicNameP = new ParamT<std::string>("projectorTopicName","", 1);  
  this->bodyNameP = new ParamT<std::string>("bodyName","", 0);
  this->textureNameP = new ParamT<std::string>("textureName","", 0);
  this->filterTextureNameP = new ParamT<std::string>("filterTextureName","", 0);
  this->nearClipDistP = new ParamT<double>("nearClipDist", .1, 0);
  this->farClipDistP = new ParamT<double>("farClipDist", 15, 0);
  this->fovP = new ParamT<double>("fov", Ogre::Math::PI*.25, 0);
  Param::End();

  this->fov = Ogre::Math::PI*.25;
  this->nearClipDist = .25;
  this->farClipDist = 15;

  this->rosnode_ = NULL;

  std::ostringstream fn_stream;
  fn_stream << this->myParent->GetName() << "_FilterNode";
  this->filterNodeName = fn_stream.str();
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosProjector::~GazeboRosProjector()
{
  delete this->robotNamespaceP;
  delete this->rosnode_;

  delete this->textureTopicNameP;
  delete this->projectorTopicNameP;  
  delete this->bodyNameP;
  delete this->textureNameP;
  delete this->filterTextureNameP;
  delete this->fovP;
  delete this->nearClipDistP;
  delete this->farClipDistP;
  
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosProjector::LoadChild(XMLConfigNode *node)
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

  this->textureTopicNameP->Load(node);
  this->projectorTopicNameP->Load(node);
  this->bodyNameP->Load(node);
  this->textureNameP->Load(node);
  this->filterTextureNameP->Load(node);
  this->fovP->Load(node);
  this->nearClipDistP->Load(node);
  this->farClipDistP->Load(node);  

  this->textureTopicName = this->textureTopicNameP->GetValue();  
  this->projectorTopicName = this->projectorTopicNameP->GetValue();
  this->bodyName = this->bodyNameP->GetValue();    
  this->textureName = this->textureNameP->GetValue();
  this->filterTextureName = this->filterTextureNameP->GetValue();
  this->fov = this->fovP->GetValue();
  this->nearClipDist = this->nearClipDistP->GetValue();
  this->farClipDist = this->farClipDistP->GetValue();  

  // Verify that the body by bodyName exists
  this->myBody = dynamic_cast<Body*>(this->myParent->GetBody(this->bodyName));
  if (this->myBody == NULL)
  {
    ROS_WARN("gazebo_ros_projector plugin error: bodyName: %s does not exist\n",this->bodyName.c_str());
    return;
  }

  // Custom Callback Queue
  ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Int32>(
    this->projectorTopicName,1,
    boost::bind( &GazeboRosProjector::ToggleProjector,this,_1),
    ros::VoidPtr(), &this->queue_);
  this->projectorSubscriber_ = this->rosnode_->subscribe(so);

  ros::SubscribeOptions so2 = ros::SubscribeOptions::create<std_msgs::String>(
    this->textureTopicName,1,
    boost::bind( &GazeboRosProjector::LoadImage,this,_1),
    ros::VoidPtr(), &this->queue_);
  this->imageSubscriber_ = this->rosnode_->subscribe(so2);
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void GazeboRosProjector::InitChild()
{
  // Custom Callback Queue
  this->callback_queue_thread_ = boost::thread( boost::bind( &GazeboRosProjector::QueueThread,this ) );

  // Initialize the projector
  if (Simulator::Instance()->GetRenderEngineEnabled())
  {
    projector_.init( this->myBody->GetVisualNode()->GetSceneNode(),
                     this->textureName, this->filterTextureName,
                     this->nearClipDist, this->farClipDist, this->fov, this->filterNodeName );

    // Add the projector as an Ogre frame listener
    getRootP()->addFrameListener( &this->projector_ );
  }
}

////////////////////////////////////////////////////////////////////////////////
// Load a texture into the projector 
void GazeboRosProjector::LoadImage(const std_msgs::String::ConstPtr& imageMsg)
{
  if (Simulator::Instance()->GetRenderEngineEnabled())
  {
    this->lock.lock();
    this->projector_.setTextureName( imageMsg->data );
    this->lock.unlock();
  }
}

////////////////////////////////////////////////////////////////////////////////
// Toggle the activation of the projector
void GazeboRosProjector::ToggleProjector(const std_msgs::Int32::ConstPtr& projectorMsg)
{
  if (Simulator::Instance()->GetRenderEngineEnabled())
  {
    this->lock.lock();
    this->projector_.setEnabled( projectorMsg->data ? true : false );
    this->lock.unlock();
  }
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosProjector::UpdateChild()
{

}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void GazeboRosProjector::FiniChild()
{
  // Custom Callback Queue
  this->queue_.clear();
  this->queue_.disable();
  this->rosnode_->shutdown();
  this->callback_queue_thread_.join();

  // Ogre cleanup
  getRootP()->removeFrameListener( &this->projector_ );
}

Ogre::Root* GazeboRosProjector::getRootP()
{
	return Ogre::Root::getSingletonPtr();
}

////////////////////////////////////////////////////////////////////////////////
// Custom callback queue thread
void GazeboRosProjector::QueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->queue_.callAvailable(ros::WallDuration(timeout));
  }
}

////////////////////////////////////////////////////////////////////////////////
GazeboRosProjector::Projector::Projector()
{
	this->isEnabled = false;
	this->isInit = false;
	this->isUsingShaders = false;

	this->projectorNode = NULL;
	this->projectorFilterNode = NULL;
  this->projectorQuery = NULL;
  this->projectorFrustum = NULL;
  this->projectorFilterFrustum = NULL;  

  this->filterNodeName = "FilterNode";
}

////////////////////////////////////////////////////////////////////////////////
GazeboRosProjector::Projector::~Projector()
{
	removeProjectorPassFromMaterials();
	
	if( this->projectorNode )
	{
		this->projectorNode->detachObject( this->projectorFrustum );
		this->projectorFilterNode->detachObject( this->projectorFilterFrustum );
		this->projectorNode->removeAndDestroyChild(this->filterNodeName);	
	}

  delete this->projectorFrustum;
  delete this->projectorFilterFrustum;

	if( this->projectorQuery )
		getSceneMgrP()->destroyQuery( this->projectorQuery );
}

void GazeboRosProjector::Projector::init( Ogre::SceneNode *sceneNodePtr,
                                          Ogre::String textureName,
                                          Ogre::String filterTextureName,
                                          double nearDist,
                                          double farDist,
                                          double fov,
                                          std::string filterNodeName )
{
	if( this->isInit )
		return;

  this->filterNodeName = filterNodeName;

  this->projectorFrustum = new Ogre::Frustum();
  this->projectorFilterFrustum = new Ogre::Frustum();
  this->projectorFilterFrustum->setProjectionType(Ogre::PT_ORTHOGRAPHIC);
  
  this->projectorQuery = getSceneMgrP()->createPlaneBoundedVolumeQuery(Ogre::PlaneBoundedVolumeList());

  setSceneNode( sceneNodePtr );
	setTextureName( textureName );
	setFilterTextureName( filterTextureName );
	setFrustumClipDistance( nearDist, farDist );
	setFrustumFOV( fov );

  this->isInit = true;
}


////////////////////////////////////////////////////////////////////////////////  
bool GazeboRosProjector::Projector::frameStarted(const Ogre::FrameEvent &evt)
{
	if( !isInit )
		init( this->projectorNode );

  if( isEnabled && !projectedTextureName.empty() )
	{
		addProjectorPassToVisibleMaterials();
		//addProjectorPassToAllMaterials();
	}
  else
  {
	  removeProjectorPassFromMaterials();
  }

	return true;
}

////////////////////////////////////////////////////////////////////////////////
bool GazeboRosProjector::Projector::frameEnded(const Ogre::FrameEvent &evt)
{
	return true;
}

////////////////////////////////////////////////////////////////////////////////
bool GazeboRosProjector::Projector::frameRenderingQueued(const Ogre::FrameEvent &evt)
{
	return true;
}

////////////////////////////////////////////////////////////////////////////////
void GazeboRosProjector::Projector::setEnabled(bool enabled)
{
	this->isEnabled = enabled;
}

////////////////////////////////////////////////////////////////////////////////
void GazeboRosProjector::Projector::setUsingShaders(bool usingShaders)
{
	this->isUsingShaders = usingShaders;
}

////////////////////////////////////////////////////////////////////////////////
void GazeboRosProjector::Projector::setSceneNode( Ogre::SceneNode *sceneNodePtr )
{
	if( this->projectorNode )
	{
		this->projectorNode->detachObject( this->projectorFrustum );
		this->projectorFilterNode->detachObject( this->projectorFilterFrustum );
		this->projectorNode->removeAndDestroyChild(this->filterNodeName);
		this->projectorFilterNode = NULL;
	}

	this->projectorNode = sceneNodePtr;
	
	if( this->projectorNode )
	{
		this->projectorNode->attachObject( this->projectorFrustum );
		this->projectorFilterNode = sceneNodePtr->createChildSceneNode(this->filterNodeName);
		this->projectorFilterNode->attachObject( this->projectorFilterFrustum );
		this->projectorFilterNode->setOrientation( Ogre::Quaternion(Ogre::Degree(90), Ogre::Vector3::UNIT_Y) );
	}
}

////////////////////////////////////////////////////////////////////////////////
void GazeboRosProjector::Projector::setTextureName( const Ogre::String& textureName )
{
	this->projectedTextureName = textureName;
}

////////////////////////////////////////////////////////////////////////////////
void GazeboRosProjector::Projector::setFilterTextureName( const Ogre::String& textureName )
{
	this->projectedFilterTextureName = textureName;
}

////////////////////////////////////////////////////////////////////////////////
void GazeboRosProjector::Projector::setFrustumClipDistance( double nearDist, double farDist )
{
	this->projectorFrustum->setNearClipDistance( nearDist );
	this->projectorFilterFrustum->setNearClipDistance( nearDist );	
	this->projectorFrustum->setFarClipDistance( farDist );
	this->projectorFilterFrustum->setFarClipDistance( farDist );		
}

////////////////////////////////////////////////////////////////////////////////
void GazeboRosProjector::Projector::setFrustumFOV( double fovInRadians )
{
	this->projectorFrustum->setFOVy( Ogre::Radian( fovInRadians ) );
	this->projectorFilterFrustum->setFOVy( Ogre::Radian( fovInRadians ) );	
}

////////////////////////////////////////////////////////////////////////////////
Ogre::SceneManager* GazeboRosProjector::Projector::getSceneMgrP()
{
	return OgreAdaptor::Instance()->sceneMgr;
}

////////////////////////////////////////////////////////////////////////////////
void GazeboRosProjector::Projector::addProjectorPassToAllMaterials()
{
	using namespace Ogre;

	std::list<std::string> allMaterials;

	SceneManager::MovableObjectIterator it = getSceneMgrP()->getMovableObjectIterator("Entity");

	while( it.hasMoreElements() )
	{
		Ogre::Entity* entity = dynamic_cast<Ogre::Entity*>( it.getNext() );
		if(entity && entity->getName().find("ENTITY") != std::string::npos)
		
		for(unsigned int i = 0; i < entity->getNumSubEntities(); i++)
		{
			allMaterials.push_back( entity->getSubEntity( i )->getMaterialName() );
		}
	}
	
	addProjectorPassToMaterials( allMaterials );
}

////////////////////////////////////////////////////////////////////////////////
void GazeboRosProjector::Projector::addProjectorPassToVisibleMaterials()
{
	using namespace Ogre;

	PlaneBoundedVolumeList volumeList;
	volumeList.push_back( projectorFrustum->getPlaneBoundedVolume() );

	this->projectorQuery->setVolumes( volumeList );
	SceneQueryResult result = this->projectorQuery->execute();

	std::list<std::string> newVisibleMaterials;

	// Find all visible materials
	SceneQueryResultMovableList::iterator it;
  for(it = result.movables.begin(); it != result.movables.end(); ++it)
	{
		Ogre::Entity* entity = dynamic_cast<Ogre::Entity*>( *it );
		if(entity && entity->getName().find("ENTITY") != std::string::npos)
		{
			for(unsigned int i = 0; i < entity->getNumSubEntities(); i++)
			{
				//addProjectorPassToMaterial( entity->getSubEntity( i )->getMaterialName() );
				newVisibleMaterials.push_back( entity->getSubEntity( i )->getMaterialName() );
			}
		}
	}

  addProjectorPassToMaterials( newVisibleMaterials );
}

////////////////////////////////////////////////////////////////////////////////
void GazeboRosProjector::Projector::addProjectorPassToMaterials( std::list<std::string>& matList )
{
  matList.remove("");
  matList.unique();

  // Loop through all existing passes, removing those for materials not in the newlist,
  //   and skipping pass creation for those in the newlist that have already been created
  OgrePassMapIterator used = projectorTargets.begin();
  while( used != projectorTargets.end() )
  {
    std::list<std::string>::iterator visibleMaterial = std::find(matList.begin(),matList.end(),used->first);

    // Remove the pass if it applies to a material not in the new list
    if( visibleMaterial == matList.end() )
	  {
		  std::string invisibleMaterial = used->first;
		  ++used;
		  removeProjectorPassFromMaterial( invisibleMaterial );
	  }
    // Otherwise remove it from the list of passes to be added
    else
	  {                
		  matList.remove( used->first );
		  ++used;
	  }
  }

  // Add pass for new materials
  while( !matList.empty() )
  {
	  addProjectorPassToMaterial( matList.front() );
		matList.erase( matList.begin() );
  }  
}

////////////////////////////////////////////////////////////////////////////////
void GazeboRosProjector::Projector::addProjectorPassToMaterial( std::string matName )
{
	if( projectorTargets.find( matName ) != projectorTargets.end() )
	{
		ROS_DEBUG("Adding a Material that already exists.");
		return;
  }
	
	using namespace Ogre;
	
	MaterialPtr mat = (MaterialPtr)MaterialManager::getSingleton().getByName( matName );
	Pass *pass = mat->getTechnique( 0 )->createPass();

	if( isUsingShaders )
	{
		Matrix4 viewProj = projectorFrustum->getProjectionMatrix() * projectorFrustum->getViewMatrix();
		pass->setVertexProgram("GazeboWorlds/TexProjectionVP");
		//pass->setFragmentProgram("GazeboWorlds/TexProjectionFP");		
		GpuProgramParametersSharedPtr vsParams = pass->getVertexProgramParameters();
		GpuProgramParametersSharedPtr psParams = pass->getFragmentProgramParameters();		
		//vsParams->setNamedAutoConstant("worldViewProjMatrix", GpuProgramParameters::ACT_WORLDVIEWPROJ_MATRIX);
		//vsParams->setNamedAutoConstant("worldMatrix",GpuProgramParameters::ACT_WORLD_MATRIX);
		//vsParams->setNamedConstant("texViewProjMatrix", viewProj);
		vsParams->setNamedAutoConstant("worldMatrix",GpuProgramParameters::ACT_WORLD_MATRIX);
		vsParams->setNamedConstant("texProjMatrix", viewProj);
		//psParams->setNamedConstant("projMap", viewProj);		
		pass->setVertexProgramParameters(vsParams);
		//pass->setFragmentProgramParameters(psParams);		
	}

	TextureUnitState *texState = pass->createTextureUnitState( projectedTextureName );
  texState->setProjectiveTexturing(true, projectorFrustum);
  texState->setTextureAddressingMode(TextureUnitState::TAM_BORDER);
  texState->setTextureFiltering(TFO_ANISOTROPIC);
  texState->setTextureBorderColour(ColourValue(0.0, 0.0, 0.0, 0.0));  
  texState->setColourOperation(LBO_ALPHA_BLEND);

  texState = pass->createTextureUnitState( projectedFilterTextureName );
  texState->setProjectiveTexturing(true, projectorFilterFrustum);
  texState->setTextureAddressingMode(TextureUnitState::TAM_CLAMP);
  texState->setTextureFiltering(TFO_NONE);
        
  pass->setSceneBlending( SBT_TRANSPARENT_ALPHA );
  pass->setDepthBias( 1 );
	pass->setLightingEnabled( false );
  
  this->projectorTargets[ matName ] = pass;
}

////////////////////////////////////////////////////////////////////////////////
void GazeboRosProjector::Projector::removeProjectorPassFromMaterials()
{
	for( OgrePassMap::const_iterator it = this->projectorTargets.begin(); it != this->projectorTargets.end(); ++it )
	{
		it->second->getParent()->removePass( it->second->getIndex() );
	}
	this->projectorTargets.clear();
}

////////////////////////////////////////////////////////////////////////////////
void GazeboRosProjector::Projector::removeProjectorPassFromMaterial(std::string matName)
{
	this->projectorTargets[ matName ]->getParent()->removePass( this->projectorTargets[matName]->getIndex() );
  this->projectorTargets.erase( this->projectorTargets.find( matName ) );
}
	
