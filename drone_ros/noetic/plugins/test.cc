
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

namespace gazebo
{
class ModelPush : public ModelPlugin
{
public:
   void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
   {
      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&ModelPush::OnUpdate, this, _1));
   }

   // Called by the world update start event
public:
   void OnUpdate(const common::UpdateInfo & /*_info*/)
   {
      // Apply a small linear velocity to the model.
      ignition::math::Vector3<double> pos_CM = ignition::math::Vector3<double>(0, 0, 0); // centro de masas
      this->model->SetLinearVel(ignition::math::Vector3<double>(0.03, 0, 0));
   }

   // Pointer to the model
private:
   physics::ModelPtr model;

   // Pointer to the update event connection
private:
   event::ConnectionPtr updateConnection;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(ModelPush)
} // namespace gazebo