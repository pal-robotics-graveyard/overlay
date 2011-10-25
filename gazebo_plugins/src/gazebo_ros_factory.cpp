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
 * SVN info: $Id: gazebo_ros_factory.cpp 329 2011-05-17 09:39:57Z ricardo $
 */


#include <algorithm>
#include <assert.h>

#include <tinyxml/tinyxml.h>

#include <gazebo_plugins/gazebo_ros_factory.h>
#include <gazebo_tools/urdf2gazebo.h>

#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>
#include <gazebo/Simulator.hh>

#include <boost/thread/recursive_mutex.hpp>

using namespace gazebo;

GZ_REGISTER_DYNAMIC_CONTROLLER("gazebo_ros_factory", GazeboRosFactory);

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosFactory::GazeboRosFactory(Entity *parent)
    : Controller(parent)
{

  this->myParent = dynamic_cast<Model*>(this->parent);

  if (!this->myParent)
    gzthrow("GazeboRosFactory controller requires a Model as its parent");

  Param::Begin(&this->parameters);
  this->robotNamespaceP = new ParamT<std::string>("robotNamespace", "/", 0);
  this->spawnModelServiceNameP = new ParamT<std::string>("spawnModelServiceName","spawn_model", 0);
  this->deleteModelServiceNameP = new ParamT<std::string>("deleteModelServiceName","delete_model", 0);
  Param::End();
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosFactory::~GazeboRosFactory()
{
  delete this->robotNamespaceP;
  delete this->spawnModelServiceNameP;
  delete this->deleteModelServiceNameP;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosFactory::LoadChild(XMLConfigNode *node)
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

  this->spawnModelServiceNameP->Load(node);
  this->spawnModelServiceName = this->spawnModelServiceNameP->GetValue();
  // advertise spawn services on the custom queue
  ros::AdvertiseServiceOptions spawn_aso = ros::AdvertiseServiceOptions::create<gazebo_plugins::SpawnModel>(
      this->spawnModelServiceName,boost::bind( &GazeboRosFactory::spawnModel, this, _1, _2 ), ros::VoidPtr(), &this->factory_queue_);
  this->spawnModelService = this->rosnode_->advertiseService(spawn_aso);

  this->deleteModelServiceNameP->Load(node);
  this->deleteModelServiceName = this->deleteModelServiceNameP->GetValue();
  // advertise delete services on the custom queue
  ros::AdvertiseServiceOptions delete_aso = ros::AdvertiseServiceOptions::create<gazebo_plugins::DeleteModel>(
      this->deleteModelServiceName,boost::bind( &GazeboRosFactory::deleteModel, this, _1, _2 ), ros::VoidPtr(), &this->factory_queue_);
  this->deleteModelService = this->rosnode_->advertiseService(delete_aso);
}

////////////////////////////////////////////////////////////////////////////////
// utilites for checking incoming string
bool GazeboRosFactory::IsURDF(std::string robot_model)
{
  TiXmlDocument doc_in;
  doc_in.Parse(robot_model.c_str());
  if (doc_in.FirstChild("robot"))
    return true;
  else
    return false;
}

////////////////////////////////////////////////////////////////////////////////
// utilites for checking incoming string
bool GazeboRosFactory::IsGazeboModelXML(std::string robot_model)
{
  TiXmlDocument doc_in;
  doc_in.Parse(robot_model.c_str());
  if (doc_in.FirstChild("model:physical"))
    return true;
  else
    return false;
}
////////////////////////////////////////////////////////////////////////////////
// Service for deleting models in Gazebo
bool GazeboRosFactory::deleteModel(gazebo_plugins::DeleteModel::Request &req,
                             gazebo_plugins::DeleteModel::Response &res)
{
  if (!this->pushToDeleteQueue(req.model_name))
  {
    ROS_ERROR("Failed to push robot model to deletion queue iface");
    return 0;
  }

  // wait and verify that model is deleted
  while (gazebo::World::Instance()->GetEntityByName(req.model_name))
  {
    ROS_DEBUG("Waiting for model deletion (%s)",req.model_name.c_str());
    usleep(1000);
  }

  // set result
  res.success = true;
  res.status_message = std::string("successfully deleted robot");

  return 1;
}

////////////////////////////////////////////////////////////////////////////////
// Service for spawning models in Gazebo
bool GazeboRosFactory::spawnModel(gazebo_plugins::SpawnModel::Request &req,
                            gazebo_plugins::SpawnModel::Response &res)
{
  // check to see if model name already exist as a model
  std::string model_name = req.model.model_name;
  if (gazebo::World::Instance()->GetEntityByName(model_name))
  {
    ROS_ERROR("model name %s already exist.",model_name.c_str());
    return 0;
  }

  // get name space for the corresponding model plugins
  std::string robot_namespace = req.model.robot_namespace;

  // get model initial pose geometry_msgs::Pose initial_pose = req.model.initial_pose;

  // get incoming string containg either an URDF or a Gazebo Model XML
  // check type depending on the xml_type flag
  // grab from parameter server if necessary
  // convert to Gazebo Model XML if necessary
  std::string robot_model = req.model.robot_model; // incoming robot model string
  bool convert_urdf2gazebo = false;
  if (req.model.xml_type == req.model.URDF)
  {
    // incoming robot model string is a string containing URDF
    // perform simple XML check
    if (this->IsURDF(robot_model))
    {
      convert_urdf2gazebo = true;
    }
    else
    {
      ROS_ERROR("input xml type does not match xml_type in request: input URDF XML must begin with <robot>");
      return 0;
    }
  }
  else if (req.model.xml_type == req.model.GAZEBO_XML)
  {
    // incoming robot model string is a string containing a Gazebo Model XML
    // perform simple XML check
    if (this->IsGazeboModelXML(robot_model))
    {
      convert_urdf2gazebo = false;
    }
    else
    {
      ROS_ERROR(" input Gazebo Model XML must begin with <model:physical>\n");
      return 0;
    }

  }
  else if (req.model.xml_type == req.model.URDF_PARAM_NAME)
  {
    // incoming robot model string contains the parameter server name for the URDF
    // get the URDF off the parameter server
    std::string full_param_name;
    if (rosnode_->searchParam(robot_model,full_param_name))
      rosnode_->getParam(full_param_name.c_str(),robot_model);
    else
      rosnode_->getParam(robot_model,robot_model);
    // incoming robot model string is a string containing URDF
    convert_urdf2gazebo = true;

    if (robot_model.c_str()==NULL)
    {
      ROS_ERROR("Unable to load robot model from param server robot_description\n");  
      return 0;
    }
    else if (!this->IsURDF(robot_model)) // simple XML check
    {
      ROS_ERROR("input xml type does not match xml_type in request: input URDF PARAM XML must begin with <robot>");
      return 0;
    }
  }
  else if (req.model.xml_type == req.model.GAZEBO_XML_PARAM_NAME)
  {
    // incoming robot model string contains the parameter server name for the Gazebo Model XML
    // get the Gazebo Model XML off the parameter server
    std::string full_param_name;
    rosnode_->searchParam(robot_model,full_param_name);
    rosnode_->getParam(full_param_name.c_str(),robot_model);
    // incoming robot model string is a string containing a Gazebo Model XML
    convert_urdf2gazebo = false;

    if (robot_model.c_str()==NULL)
    {
      ROS_ERROR("Unable to load robot model from param server robot_description\n");  
      return 0;
    }
    else if (!this->IsGazeboModelXML(robot_model)) // simple XML check
    {
      ROS_ERROR("input Gazebo Model PARAM XML must begin with <model:physical>\n");
      return 0;
    }
  }
  else if (req.model.xml_type == req.model.URDF_FILE_NAME)
  {
    // incoming robot model string contains the filename for the URDF
    // get the URDF from file (or use resource retriever?)
    ROS_WARN("spawning from resource_retriever using a URDF filename is not supported right now");

    TiXmlDocument robot_model_xml(robot_model);
    robot_model_xml.LoadFile();
    // copy tixml to a string
    std::ostringstream stream;
    stream << robot_model_xml;
    robot_model = stream.str();
    convert_urdf2gazebo = true;

    if (robot_model.c_str()==NULL)
    {
      ROS_ERROR("Unable to load robot model from param server robot_description\n");  
      return 0;
    }
    else if (!this->IsURDF(robot_model))
    {
      ROS_ERROR("input xml type does not match xml_type in request: input URDF XML FILE must begin with <robot>");
      return 0;
    }
  }
  else if (req.model.xml_type == req.model.GAZEBO_XML_FILE_NAME)
  {
    // incoming robot model string contains the file name for the Gazebo Model XML
    // get the Gazebo Model XML from file (or use the resource retriever?)
    ROS_WARN("spawning from resource_retriever using a Gazebo Model XML filename is not supported right now");

    TiXmlDocument robot_model_xml(robot_model);
    robot_model_xml.LoadFile();
    // copy tixml to a string
    std::ostringstream stream;
    stream << robot_model_xml;
    robot_model = stream.str();
    convert_urdf2gazebo = false;

    if (robot_model.c_str()==NULL)
    {
      ROS_ERROR("Unable to load robot model from param server robot_description\n");  
      return 0;
    }
    else if (!this->IsGazeboModelXML(robot_model))
    {
      ROS_ERROR("input Gazebo Model XML FILE must begin with <model:physical>\n");
      return 0;
    }
  }
  else
  {
    ROS_ERROR("type of robot model to be spawned is incorrect, options are:(URDF,GAZEBO_XML,URDF_PARAM_NAME,GAZEBO_XML_PARAM_NAME)");
    return 0;
  }

  // get options for conversions
  // get initial xyz
  double initial_x = req.model.initial_pose.position.x;
  double initial_y = req.model.initial_pose.position.y;
  double initial_z = req.model.initial_pose.position.z;
  urdf::Vector3 initial_xyz(initial_x,initial_y,initial_z);
  // get initial roll pitch yaw (fixed frame transform)
  urdf::Rotation initial_q(req.model.initial_pose.orientation.x,req.model.initial_pose.orientation.y,req.model.initial_pose.orientation.z,req.model.initial_pose.orientation.w);
  double initial_rx,initial_ry,initial_rz;
  initial_q.getRPY(initial_rx,initial_ry,initial_rz);
  urdf::Vector3 initial_rpy(initial_rx*180/M_PI,initial_ry*180/M_PI,initial_rz*180/M_PI);
  // get flag on joint limits
  bool disable_urdf_joint_limits = req.model.disable_urdf_joint_limits;

  // do the conversion if necessary
  urdf2gazebo::URDF2Gazebo u2g;
  TiXmlDocument gazebo_model_xml; // resulting Gazebo Model XML to be sent to Factory Iface
  if (convert_urdf2gazebo)
  {
    //************************************/
    /*    convert urdf to gazebo model   */
    //************************************/
    TiXmlDocument robot_model_xml;
    robot_model_xml.Parse(robot_model.c_str());
    u2g.convert(robot_model_xml, gazebo_model_xml, disable_urdf_joint_limits, initial_xyz, initial_rpy, model_name, robot_namespace);
  }
  else
  {
    //************************************/
    /*    prepare gazebo model xml for   */
    /*    factory                        */
    //************************************/
    /// STRIP DECLARATION <? ... xml version="1.0" ... ?> from robot_model
    /// @todo: does tinyxml have functionality for this?
    /// @todo: should gazebo take care of the declaration?
    std::string open_bracket("<?");
    std::string close_bracket("?>");
    int pos1 = robot_model.find(open_bracket,0);
    int pos2 = robot_model.find(close_bracket,0);
    robot_model.replace(pos1,pos2-pos1+2,std::string(""));

    // put string in TiXmlDocument for manipulation
    gazebo_model_xml.Parse(robot_model.c_str());

    // optional model manipulations:
    //  * update initial pose
    //  * replace model name
    TiXmlElement* model;
    model = gazebo_model_xml.FirstChildElement("model:physical");
    if (model)
    {
      // replace initial pose of robot
      // find first instance of xyz and rpy, replace with initial pose
      TiXmlElement* xyz_key = model->FirstChildElement("xyz");
      if (xyz_key)
        model->RemoveChild(xyz_key);
      TiXmlElement* rpy_key = model->FirstChildElement("rpy");
      if (rpy_key)
        model->RemoveChild(rpy_key);

      xyz_key = new TiXmlElement("xyz");
      rpy_key = new TiXmlElement("rpy");

      std::ostringstream xyz_stream, rpy_stream;
      xyz_stream << initial_x << " " << initial_y << " " << initial_z;
      rpy_stream << initial_rx << " " << initial_ry << " " << initial_rz;

      TiXmlText* xyz_txt = new TiXmlText(xyz_stream.str());
      TiXmlText* rpy_txt = new TiXmlText(rpy_stream.str());

      xyz_key->LinkEndChild(xyz_txt);
      rpy_key->LinkEndChild(rpy_txt);

      model->LinkEndChild(xyz_key);
      model->LinkEndChild(rpy_key);


      // replace model name if one is specified by the user
      if (!model_name.empty())
      {
        model->RemoveAttribute("name");
        model->SetAttribute("name",model_name);
      }

    }
  }

  // push to factory iface
  std::ostringstream stream;
  stream << gazebo_model_xml;
  std::string gazebo_model_xml_string = stream.str();
  ROS_DEBUG("Gazebo Model XML\n\n%s\n\n ",gazebo_model_xml_string.c_str());

  if (!this->pushToFactory(gazebo_model_xml_string))
  {
    ROS_ERROR("Failed to push robot model to factory iface");
    return 0;
  }

  // wait and verify that model is spawned
  while (true)
  {
    boost::recursive_mutex::scoped_lock lock(*gazebo::Simulator::Instance()->GetMRMutex());
    if (gazebo::World::Instance()->GetEntityByName(model_name)) break;
    ROS_DEBUG("Waiting for spawning model (%s)",model_name.c_str());
    usleep(1000);
  }

  // set result
  res.success = true;
  res.status_message = std::string("successfully spawned robot");

  return 1;
}

////////////////////////////////////////////////////////////////////////////////
// Open Factory Iface and Push Model to Factory
bool GazeboRosFactory::pushToFactory(std::string gazebo_model_xml)
{
  //************************************/
  /*  Connect to Gazebo Iface Server   */
  //************************************/
  libgazebo::Client *client = new libgazebo::Client();
  libgazebo::FactoryIface *factoryIface = new libgazebo::FactoryIface();

  int serverId = 0;

  bool connected_to_server = false;
  /// Connect to the libgazebo server
  while (!connected_to_server)
  {
    try
    {
      ROS_DEBUG("spawn_gazebo_model waiting for gazebo factory, usually launched by 'roslaunch `rospack find gazebo_worlds`/launch/empty_world.launch'");
      client->ConnectWait(serverId, GZ_CLIENT_ID_USER_FIRST);
      connected_to_server = true;
    }
    catch (gazebo::GazeboError e)
    {
      ROS_ERROR("Gazebo error: Unable to connect\n %s\n",e.GetErrorStr().c_str());
      usleep(1000);
      connected_to_server = false;
    }
  }

  //************************************/
  /*    Open the Factory interface     */
  //************************************/
  try
  {
    factoryIface->Open(client, "default");
  }
  catch (gazebo::GazeboError e)
  {
    ROS_ERROR("Gazebo error: Unable to connect to the factory interface\n%s\n",e.GetErrorStr().c_str());
    return false;
  }

  //************************************/
  /*  Copy model to a string           */
  //************************************/
  std::ostringstream stream;
  stream << gazebo_model_xml;
  std::string gazebo_model_xml_string = stream.str();
  ROS_DEBUG("Gazebo Model XML\n\n%s\n\n ",gazebo_model_xml_string.c_str());

  bool writing_iface = true;
  while (writing_iface)
  {
    factoryIface->Lock(1);
    if (strcmp((char*)factoryIface->data->newModel,"")==0)
    {
      // don't overwrite data, only write if iface data is empty
      strcpy((char*)factoryIface->data->newModel, gazebo_model_xml_string.c_str());
      writing_iface = false;
    }
    factoryIface->Unlock();
  }
  return true;
}
////////////////////////////////////////////////////////////////////////////////
// Push model name to Iface for deletion
bool GazeboRosFactory::pushToDeleteQueue(std::string model_name)
{
  // connect to gazebo
  libgazebo::Client *client = new libgazebo::Client();
  libgazebo::FactoryIface *factoryIface = new libgazebo::FactoryIface();

  int serverId = 0;

  bool connected_to_server = false;
  /// Connect to the libgazebo server
  while (!connected_to_server)
  {
    try
    {
      client->ConnectWait(serverId, GZ_CLIENT_ID_USER_FIRST);
      connected_to_server = true;
    }
    catch (gazebo::GazeboError e)
    {
      ROS_ERROR("Gazebo error: Unable to connect\n %s\n",e.GetErrorStr().c_str());
      usleep(1000);
      connected_to_server = false;
    }
  }

  /// Open the Factory interface
  try
  {
    factoryIface->Open(client, "default");
  }
  catch (gazebo::GazeboError e)
  {
    ROS_ERROR("Gazebo error: Unable to connect to the factory interface\n%s\n",e.GetErrorStr().c_str());
    return -1;
  }

  bool writing_iface = true;
  while (writing_iface)
  {
    factoryIface->Lock(1);
    if (strcmp((char*)factoryIface->data->deleteEntity,"")==0)
    {
      ROS_INFO("Deleting Robot Model Name:%s in Gazebo\n",model_name.c_str());
      // don't overwrite data, only write if iface data is empty
      strcpy((char*)factoryIface->data->deleteEntity, model_name.c_str());
      writing_iface = false;
    }
    factoryIface->Unlock();
  }

  return true;

}
////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void GazeboRosFactory::InitChild()
{
  // start custom queue for factory
  this->callback_queue_thread_ = boost::thread( boost::bind( &GazeboRosFactory::FactoryQueueThread,this ) );
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosFactory::UpdateChild()
{
    /***************************************************************/
    /*                                                             */
    /*  this is called at every update simulation step             */
    /*                                                             */
    /***************************************************************/
}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void GazeboRosFactory::FiniChild()
{
  this->factory_queue_.disable();
  this->factory_queue_.clear();
  this->rosnode_->shutdown();
  this->callback_queue_thread_.join();
}

////////////////////////////////////////////////////////////////////////////////
// Put laser data to the interface
void GazeboRosFactory::FactoryQueueThread()
{
  ROS_DEBUG_STREAM("Callback thread id=" << boost::this_thread::get_id());

  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->factory_queue_.callAvailable(ros::WallDuration(timeout));
  }
}

