#ifndef vacuum_plugin_H
#define vacuum_plugin_H

#include <string>

#include <gazebo/gazebo.hh>

#include <ignition/transport/Node.hh>
#include "gazebo/common/Plugin.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/transport/Node.hh"

namespace gazebo
{
  /// \brief An example plugin for a contact sensor.
  class vacuum_plugin : public ModelPlugin
  {
    /// \brief Constructor.
    public:
      vacuum_plugin();

    /// \brief Destructor.
      virtual ~vacuum_plugin();

      virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    private:
      // void OnUpdate();
      // void OnContainPluginMsg(const ignition::msgs::Boolean &_msg);
      //Pointer to the model
      physics::ModelPtr model;

      static void cb(ConstWorldStatisticsPtr &_msg);
      /// \brief A node used for transport
      private: transport::NodePtr node;
      /// \brief A subscriber to a named topic.
      private: transport::SubscriberPtr sub;
  };
}
#endif
