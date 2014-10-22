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

    /// Flipper class derived from box to hit phys boxes around the scene
    class Flipper : public Box3D {
    private:
      btVector3 flipTorque;

    public:
      Flipper() {
      }

      ~Flipper() {
      }

      /// This is called to initialise the flipper.
      void init_flipper(mat4t model2world, vec3 box_size, material *box_material, vec3 torque, float mass) {
        flipTorque = get_btVector3(torque);
        init_box(model2world, box_size, box_material, mass);
        rigidbody->setActivationState(DISABLE_DEACTIVATION);    // disables the deactiveation state
      }

      /// This is called by the player to Rotate the flipper.
      void flip(){
        rigidbody->applyTorqueImpulse(flipTorque);
        // printf("Flipper function has been activated");
      }
    };


  }
}

#endif