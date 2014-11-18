////////////////////////////////////////////////////////////////////////////////
//
// (C) Sam Hayhurst 2014
//
// Flipper class, derives from Box3D
//

#ifndef FLIPPER_INCLUDED
#define FLIPPER_INCLUDED

#include "Box3D.h"

namespace octet {
  namespace pinball {
    /// Flipper class derived from Box3D used to create pinball flippers in pinballWizzard app.
    /// contains functionality to apply a torque, axis and torque to be defined
    /// using bullet physics constraints.
    class Flipper : public Box3D {
    private:
      btVector3 flipTorque;

    public:
      /// default Flipper Constructor.
      Flipper() {
      }

      /// Default Flipper Destructor.
      ~Flipper() {
      }

      /// This is called to initialise the flipper.
      void init_flipper(mat4t model2world, vec3 box_size, material *box_material, vec3 torque, float mass) {
        flipTorque = get_btVector3(torque);
        init_box(model2world, box_size, box_material, mass);
        rigidbody->setActivationState(DISABLE_DEACTIVATION);    // disables the deactiveation state
      }

      /// This is called by the player to apply a torque to the Flipper object.
      void flip(){
        rigidbody->applyTorqueImpulse(flipTorque);
      }

      /// Adds the mesh and rigidbody of the box to the scene
      void add_to_scene(dynarray<scene_node*> &sceneNodes, ref<visual_scene> &appScene, btDiscreteDynamicsWorld &btWorld,
        dynarray<btRigidBody*> &rigidBodies, bool is_visible = true, bool make_child = true) {
        btWorld.addRigidBody(rigidbody);
        rigidBodies.push_back(rigidbody);
        sceneNodes.push_back(node);
        if (is_visible) {
          appScene->add_mesh_instance(new mesh_instance(node, meshBox, mat));
        }
        if (make_child) {
          appScene->add_child(node);
        }
      }


    };
  }
}

#endif