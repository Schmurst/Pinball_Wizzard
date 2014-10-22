////////////////////////////////////////////////////////////////////////////////
//
// (C) Sam Hayhurst 2014
//
// Box3D class, derives from Object3D
//

#ifndef BOX3D_INCLUDED
#define BOX3D_INCLUDED

#include "Object3D.h"

namespace octet {
  namespace pinball {

    /// Box3D class, simple 3d box class, can be dynamic
    class Box3D : public Object3D {
    protected:
      vec3 size;
      mesh_box* meshBox;

    public:
      /// Box3d Constructor, used to initialise a dynamic box.
      Box3D()
      {}
      /// Box3D destructor
      ~Box3D(){
      }

      /// init function, mass defaults to 1.0 to ensure dynamic behavior within the scene
      void init_box(mat4t model2world, vec3 size, material *material_box, float mass_box = 1.0f) {
        init(model2world, material_box, mass_box);
        btCollisionShape *shape = new btBoxShape(get_btVector3(size));
        btVector3 inertialTensor;
        shape->calculateLocalInertia(mass, inertialTensor);
        rigidbody = new btRigidBody(mass, motionState, shape, inertialTensor);
        // init mesh_box and scene node
        meshBox = new mesh_box(size);
        node = new scene_node(modelToWorld, atom_);
      }

      /// Adds the mesh and rigidbody of the box to the scene
      void add_to_scene(dynarray<scene_node*> &sceneNodes, ref<visual_scene> appScene, btDiscreteDynamicsWorld &btWorld, dynarray<btRigidBody*> &rigidBodies, bool is_visible = true) {
        addToScene(sceneNodes, appScene, btWorld, rigidBodies);
        // below is inefficient, still making node and box and whatever
        if (is_visible) {
          appScene->add_mesh_instance(new mesh_instance(node, meshBox, mat));
        }
      }
    };


  }
}

#endif