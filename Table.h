////////////////////////////////////////////////////////////////////////////////
//
// (C) Sam Hayhurst 2014
//
// Table class, used to construct the rigidbodies to fit the collada mesh of 
// the pinball table
//

#ifndef TABLE_INCLUDED
#define TABLE_INCLUDED

#include "Box3D.h"

namespace octet {
  namespace pinball {

    class table : public Box3D{
    private:
      int childCount;

    public:
      void init_table(mat4t model2world, vec3 size, material *material_box, float mass_box = 1.0f) {
        init_box(model2world, size, material_box, mass_box);
      }

      int getChildCount() {
        return childCount;
      }

    };
  }
}
#endif