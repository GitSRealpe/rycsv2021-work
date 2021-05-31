#ifndef vacuum_plugin_H
#define vacuum_plugin_H

#include <string>

#include <gazebo/gazebo.hh>
#include <ignition/transport/Node.hh>
#include "gazebo/common/Plugin.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/transport/Node.hh"
#include "std_msgs/String.h"
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"

namespace gazebo
{
  /// \brief An example plugin for a contact sensor.
  class vacuum_plugin : public ModelPlugin
  {
    /// \brief Constructor.
    public:
      vacuum_plugin();
      virtual ~vacuum_plugin();
      virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
      void callback(const std_msgs::String::ConstPtr& msg);

    private: transport::NodePtr node;
    private: transport::SubscriberPtr sub;

    private: physics::ModelPtr model;
    private: physics::JointPtr fixedJoint;
    private: physics::LinkPtr palmLink;
    private: physics::ModelPtr objeto;
    private: physics::LinkPtr objeto_link;
    private: physics::WorldPtr mundo;

    public: std::string obj;
    public: std::string obj2;
    public: std::string old_obj;
    public: std::string ignorados;
    public: bool drop;
    public: bool drop2;
    private: ros::Publisher pub;
    private: ros::Subscriber rosub;

    private: std_msgs::String aviso;

    private: void OnMsg(ConstLogicalCameraImagePtr &_msg);
    private: void Agarrar(std::string obja);
    private: void Soltar(std::string obja);
    // private: void callback(std_msgs::String::ConstPtr& msg);


  };
}
#endif
