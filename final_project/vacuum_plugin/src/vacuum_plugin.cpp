#include "vacuum_plugin/vacuum_plugin.h"
#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/common/Plugin.hh>
#include <ignition/math/Pose3.hh>

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/sensors/sensors.hh>

// #include <ros/ros.h>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/String.h"

#include <iostream>
#include <string>

using namespace gazebo;
// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(vacuum_plugin)
/////////////////////////////////////////////////
vacuum_plugin::vacuum_plugin(){}

/////////////////////////////////////////////////
vacuum_plugin::~vacuum_plugin(){}

void vacuum_plugin::callback(const std_msgs::String::ConstPtr& msg)
{
  std::string cmd = msg->data.c_str();
  ROS_INFO("Comando recibido vacuum: [%s]", msg->data.c_str());
  if (obj!="vacio") {
    if (cmd=="suck"){Agarrar(obj);}
    if (cmd=="drop"){Soltar(obj);}
  }else{
    ROS_INFO("No hay objeto sobre el cual actuar");
  }
}

void vacuum_plugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  std::cout<<"caregado el plugin men"<<"\n";

  ignorados=_sdf->GetElement("ignore")->Get<std::string>();
  std::cout<<ignorados<<"\n";

  int argc = 0;
  char **argv = NULL;
  ros::init(argc, argv, "gazebo_client",ros::init_options::NoSigintHandler);
  ros::NodeHandle node_handle;
  pub = node_handle.advertise<std_msgs::String>("gazebo/gripper/gripped", 100);
  rosub = node_handle.subscribe<std_msgs::String>("vacuum_commander/cmd", 10, &vacuum_plugin::callback, this);

  this->model = _parent;
  obj="vacio";
  this->mundo = this->model->GetWorld();
  physics::PhysicsEnginePtr physics = this->mundo->Physics();

  this->fixedJoint = physics->CreateJoint("revolute");
  // link del robot
  this->palmLink = this->model->GetLink("link_7");
  // std::cout<<this->palmLink->GetName()<<"\n";

  // Create the node
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->model->GetWorld()->Name());
  // Create a topic name
  std::string topicName = "~/"+this->model->GetName()+"/link_7/detector/models";
  std::cout<<topicName<<"\n";
  // Subscribe to the topic, and register a callback
  this->sub = this->node->Subscribe(topicName,&vacuum_plugin::OnMsg, this);

}

void vacuum_plugin::OnMsg(ConstLogicalCameraImagePtr &_msg)
{
  if(_msg->model().size()>0){
    for (int i = 0; i <= _msg->model().size()-1; i++) {
      // std::cout<<_msg->model(i).name()<<"\n";
      if( ignorados.find(_msg->model(i).name()) == std::string::npos){
        obj=_msg->model(i).name();
        // std::cout<<"detectado "<<obj<<"\n";
        aviso.data = obj;
        pub.publish(aviso);
      }
    }
  }
}

void vacuum_plugin::Agarrar(std::string obja){
  std::cout<<"ay papaaa, agarrado "<<obja<<"\n";
  // objeto
  this->objeto = this->mundo->ModelByName(obja);
  // link del objeto
  this->objeto_link = this->objeto->GetLink();
  ignition::math::Pose3d var = this->objeto_link->WorldPose() - this->palmLink->WorldPose();
  std::cout<<var<<"\n";
  this->fixedJoint->Load(this->palmLink, this->objeto_link, var);
  this->fixedJoint->Init();
  this->fixedJoint->SetUpperLimit(0, 0);
  this->fixedJoint->SetLowerLimit(0, 0);

}

void vacuum_plugin::Soltar(std::string obja){
  std::cout<<"ay señor bendito, soltado "<<obja<<"\n";
  this->fixedJoint->Detach();
}
