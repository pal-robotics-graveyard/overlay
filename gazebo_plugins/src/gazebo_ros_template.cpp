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
 * SVN info: $Id: gazebo_ros_template.cpp 329 2011-05-17 09:39:57Z ricardo $
 */


#include <algorithm>
#include <assert.h>

#include <gazebo_plugins/gazebo_ros_template.h>

#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>

using namespace gazebo;

GZ_REGISTER_DYNAMIC_CONTROLLER("gazebo_ros_template", GazeboRosTemplate);

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosTemplate::GazeboRosTemplate(Entity *parent)
    : Controller(parent)
{

}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosTemplate::~GazeboRosTemplate()
{
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosTemplate::LoadChild(XMLConfigNode *node)
{

}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void GazeboRosTemplate::InitChild()
{
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosTemplate::UpdateChild()
{
    /***************************************************************/
    /*                                                             */
    /*  this is called at every update simulation step             */
    /*                                                             */
    /***************************************************************/
}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void GazeboRosTemplate::FiniChild()
{
}



