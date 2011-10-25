/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <cstdio>
#include <cstdlib>

#include <vector>
#include <string>
#include <sstream>

// Interface to Gazebo
#include <gazebo_plugins/DeleteModel.h>
#include <gazebo_plugins/SpawnModel.h>
#include <gazebo_plugins/SetPose.h>

#include <ros/ros.h>
#include <boost/program_options.hpp>

#include <gazebo_tools/urdf2gazebo.h>

void usage(const char *progname)
{
    printf("\nUsage: %s [options] <spawn|kill> <model_name>\n", progname);
    printf("  Example: %s --help\n",progname);
    printf("  Example: %s -p robot_description -x 10 spawn pr2\n",progname);
    printf("  Example: %s -f my_urdf_file.urdf -o gazebo_model.xml -x 10\n",progname);
    printf("  Example: %s kill pr2_model\n",progname);
}

/// \brief checks the xml document and set xml_type according to gazebo_polugins/GazeboModel.msg
///        returns 0 if fails to determine type
bool getXMLType(TiXmlDocument &xml_doc,uint32_t &xml_type)
{
      // check root xml element to determine if conversion is necessary
      if (xml_doc.FirstChild("robot"))
        xml_type = gazebo_plugins::GazeboModel::URDF;
      else if (xml_doc.FirstChild("model:physical"))
        xml_type = gazebo_plugins::GazeboModel::GAZEBO_XML;
      else
        return 0;

      return 1;
}
bool getXMLType(std::string xml_string,uint32_t &xml_type)
{
      TiXmlDocument xml_doc;
      xml_doc.Parse(xml_string.c_str());
      return getXMLType(xml_doc,xml_type);
}

void convertToGazeboXML(std::string xml_string,TiXmlDocument &gazebo_model_xml,bool enforce_limits,urdf::Vector3 initial_xyz,urdf::Vector3 initial_rpy,std::string model_name,std::string robot_namespace)
{
      // convert string to xml document
      TiXmlDocument xml_doc;
      xml_doc.Parse(xml_string.c_str());

      // do convertion / manipulate xml
      if (xml_doc.FirstChild("robot")) // is urdf, do conversion with urdf2gazebo
      {
        // convert urdf to gazebo model
        urdf2gazebo::URDF2Gazebo u2g;
        u2g.convert(xml_doc, gazebo_model_xml, enforce_limits, initial_xyz, initial_rpy, model_name, robot_namespace);
      }
      else if (xml_doc.FirstChild("model:physical")) // is gazeob XML, simply fixup the initial pose and name
      {
        /// STRIP DECLARATION <? ... xml version="1.0" ... ?> from xml_string
        /// @todo: does tinyxml have functionality for this?
        /// @todo: should gazebo take care of the declaration?
        std::string open_bracket("<?");
        std::string close_bracket("?>");
        int pos1 = xml_string.find(open_bracket,0);
        int pos2 = xml_string.find(close_bracket,0);
        xml_string.replace(pos1,pos2-pos1+2,std::string(""));

        // put string in TiXmlDocument for manipulation
        gazebo_model_xml.Parse(xml_string.c_str());

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
          xyz_stream << initial_xyz.x << " " << initial_xyz.y << " " << initial_xyz.z;
          rpy_stream << initial_rpy.x << " " << initial_rpy.y << " " << initial_rpy.z;

          TiXmlText* xyz_txt = new TiXmlText(xyz_stream.str());
          TiXmlText* rpy_txt = new TiXmlText(rpy_stream.str());

          xyz_key->LinkEndChild(xyz_txt);
          rpy_key->LinkEndChild(rpy_txt);

          model->LinkEndChild(xyz_key);
          model->LinkEndChild(rpy_key);


          // replace model name if one is specified by the user
          if (!model_name.empty())
          {
            // replace model name by the one specified by user
            model->RemoveAttribute("name");
            model->SetAttribute("name",model_name);
          }
          else
          {
            // set model_name to name specified in XML
            model_name = model->Attribute("name");
          }

        }

      }
      else
      {
            printf("Unable to determine XML type for writing to file\n");
            exit(3);
      }
}
namespace po = boost::program_options;

int main(int argc, char **argv)
{
    ros::init(argc,argv,"gazebo_model",ros::init_options::AnonymousName);

    // print usage
    if (argc < 2)
    {
        usage(argv[0]);
        exit(1);
    }

    // startup an node handle,
    ros::NodeHandle nh_("~");
    // establish plugin service connection
    ros::ServiceClient spawn_model_srv_ = nh_.serviceClient<gazebo_plugins::SpawnModel>("/spawn_model");
    ros::ServiceClient delete_model_srv_ = nh_.serviceClient<gazebo_plugins::DeleteModel>("/delete_model");
    

    // initialize variables
    double initial_x = 0;
    double initial_y = 0;
    double initial_z = 0;
    double initial_rx = 0;
    double initial_ry = 0;
    double initial_rz = 0;

    // parse options
    po::options_description v_desc("Allowed options");
    v_desc.add_options()
      ("help,h" , "produce this help message")
      ("param-name,p" , po::value<std::string>() , "Name of parameter on parameter server containing the model XML.")
      ("namespace,n" , po::value<std::string>() , "ROS namespace of the model spawned in simulation. If not specified, the namespace of this node is used.")
      ("input-filename,f" , po::value<std::string>() , "read input model from file, not from the parameter server, exclusive with -p option.")
      ("output-filename,o" , po::value<std::string>() , "write converted gazebo model xml to a file instead, model is not spawned in simulation.")
      ("skip-joint-limits,s" , "do not enforce joint limits during urdf-->gazebo conversion.")
      ("init-x,x" , po::value<double>() , "set initial x position of model, defaults to 0.")
      ("init-y,y" , po::value<double>() , "set initial y position of model, defaults to 0.")
      ("init-z,z" , po::value<double>() , "set initial z position of model, defaults to 0.")
      ("init-roll,R" , po::value<double>() , "set initial roll (rx) of model, defaults to 0.  application orders are r-p-y.")
      ("init-pitch,P" , po::value<double>() , "set initial pitch (ry) of model, defaults to 0.  application orders are r-p-y.")
      ("init-yaw,Y" , po::value<double>() , "set initial yaw (rz) of model, defaults to 0.  application orders are r-p-y.");

    po::options_description h_desc("Hidden options");
    h_desc.add_options()
      ("command" , po::value< std::vector<std::string> >(), "<spawn|kill>")
      ("model-name" , po::value< std::vector<std::string> >(), "overwrite name of the gazebo model in simulation. If not specified, the model name defaults to the name in urdf.");

    po::options_description desc("Allowed options");
    desc.add(v_desc).add(h_desc);

    po::positional_options_description p_desc;
    p_desc.add("command", 1);
    p_desc.add("model-name", 2);  // model-name must come after command

    po::variables_map vm;
    //po::store(po::parse_command_line(argc, argv, desc), vm);
    po::store(po::command_line_parser(argc, argv).options(desc).positional(p_desc).run(), vm);
    po::notify(vm);

    if (vm.count("help"))
    {
      usage(argv[0]);
      std::cout << v_desc << std::endl;
      exit(1);
    }

    // set flag to enforce joint limits, this is hardcoded to true because we do want a model with
    // joint limits enforced.
    bool enforce_limits = true;
    if (vm.count("skip-joint-limits"))
      enforce_limits = false;


    // if parameter contains urdf, convert from urdf to gazebo model
    std::string file_in;
    if (vm.count("input-filename"))
      file_in = vm["input-filename"].as<std::string>();
    else
      file_in.clear();

    std::string file_out;
    if (vm.count("output-filename"))
    {
      file_out = vm["output-filename"].as<std::string>();
    }

    std::string robot_namespace;
    if (vm.count("namespace"))
    {
      robot_namespace = vm["namespace"].as<std::string>();
    }
    else
    {
      robot_namespace = ros::this_node::getNamespace();
      ROS_DEBUG("NOTE: %s is called under namespace %s, this namespace is passed to gazebo so all Gazebo plugins (e.g.ros_time, gazebo_mechanism_control,etc.) will start up under this namespace.  User can overwrite by passing in argument -namespace, for example: %s /prf/robot_description -n /prf",argv[0],argv[0],robot_namespace.c_str());
    }
    ROS_DEBUG("namespace: %s",robot_namespace.c_str());


    if (vm.count("init-x"))
    {
      initial_x = vm["init-x"].as<double>();
      ROS_DEBUG("x: %f",initial_x);
    }
    if (vm.count("init-y"))
    {
      initial_y = vm["init-y"].as<double>();
      ROS_DEBUG("y: %f",initial_y);
    }
    if (vm.count("init-z"))
    {
      initial_z = vm["init-z"].as<double>();
      ROS_DEBUG("z: %f",initial_z);
    }
    if (vm.count("init-roll"))
    {
      initial_rx = vm["init-roll"].as<double>();
      ROS_DEBUG("init-roll: %f",initial_rx);
    }
    if (vm.count("init-pitch"))
    {
      initial_ry = vm["init-pitch"].as<double>();
      ROS_DEBUG("init-pitch: %f",initial_ry);
    }
    if (vm.count("init-yaw"))
    {
      initial_rz = vm["init-yaw"].as<double>();
      ROS_DEBUG("init-yaw: %f",initial_rz);
    }

    std::string command;
    if (vm.count("command"))
    {
      std::vector<std::string> str_vec = vm["command"].as<std::vector<std::string> >();
      if (str_vec.size() == 1)
        command = str_vec[0];
      else
        command.clear();
    }

    std::string param_name;
    if (vm.count("param-name"))
    {
      param_name = vm["param-name"].as<std::string>();
      if (!file_in.empty())
      {
        ROS_ERROR("Both param-name and input-filename are specified.  Only one input source is allowed.\n");  
        exit(2);
      }
    }
    else if (command == "spawn" && file_in.empty())
    {
      ROS_ERROR("Either param-name or input-filename must be specified for spawn to work.  Need one input source.\n");  
      exit(2);
    }


    // model name will over-ride any model names specified in the model URDF/XML
    std::string model_name;
    if (vm.count("model-name"))
    {
      std::vector<std::string> str_vec = vm["model-name"].as<std::vector<std::string> >();
      if (str_vec.size() == 1)
      {
        model_name = str_vec[0];
        // get rid of slahses in robot model name
        std::replace(model_name.begin(),model_name.end(),'/','_');
        ROS_DEBUG("model name will be: %s",model_name.c_str());
      }
      else
        model_name.clear();
    }


    // setup initial pose vectors
    urdf::Vector3 initial_xyz(initial_x,initial_y,initial_z);
    urdf::Vector3 initial_rpy(initial_rx,initial_ry,initial_rz);

    if (command == "kill" && !model_name.empty())
    {
      // delete model from simulation
      // wait for service
      delete_model_srv_.waitForExistence();

      gazebo_plugins::DeleteModel model_msg;
      model_msg.request.model_name = model_name;
      if (delete_model_srv_.call(model_msg))
        ROS_INFO("successfully deleted model %s",model_name.c_str());
      else
        ROS_INFO("could not delete model %s",model_name.c_str());
    }
    else
    {
      // connect to model spawning service
      spawn_model_srv_.waitForExistence();

      // construct gazebo model message
      gazebo_plugins::SpawnModel model_msg;

      /// Connect to the libgazebo server
      model_msg.request.model.model_name = model_name;


      /******************************/
      /* read input into xml_string */
      /******************************/
      if (file_in.empty()) // input filename is not specified, we'll search parameter name for robot xml
      {
        // load parameter server string for robot/model
        std::string xml_string;
        std::string full_param_name;
        nh_.searchParam(param_name,full_param_name);
        nh_.getParam(full_param_name.c_str(),xml_string);
        ROS_DEBUG("%s content\n%s\n", full_param_name.c_str(), xml_string.c_str());
        if (xml_string.c_str()==NULL)
        {
            ROS_ERROR("Unable to load robot model from param server robot_description\n");  
            exit(2);
        }
        else
        {
          model_msg.request.model.robot_model = xml_string;
        }
      }
      else // input filename is specified, parse the input file into xml doc and convert to string
      {
        // read urdf / gazebo model xml from file
        TiXmlDocument xml_in(file_in);
        xml_in.LoadFile();
        // copy tixml to a string
        std::ostringstream stream;
        stream << xml_in;
        model_msg.request.model.robot_model = stream.str();
      }

      getXMLType(model_msg.request.model.robot_model,model_msg.request.model.xml_type);


      if (command == "spawn" && file_out.empty()) // spawn model 
      {
        // call service to spawn model in simulation
        if (spawn_model_srv_.call(model_msg))
          ROS_INFO("successfull spawned model %s",model_name.c_str());
        else
          ROS_INFO("could not spawn model %s",model_name.c_str());
      }
      else if (!file_out.empty())
      {
        // do conversion locally and save to file
        TiXmlDocument gazebo_model_xml;
        convertToGazeboXML(model_msg.request.model.robot_model,gazebo_model_xml,enforce_limits,initial_xyz,initial_rpy,model_name,robot_namespace);
        // output filename is specified, save converted Gazebo Model XML to a file
        if (!gazebo_model_xml.SaveFile(file_out))
        {
            printf("Unable to save gazebo model in %s\n", file_out.c_str());  
            exit(3);
        }
      }
      else
      {
        ROS_ERROR("command must be either spawn, kill or use -o option to do file conversion.");
      }
    }

    return 0;
}

