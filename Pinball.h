////////////////////////////////////////////////////////////////////////////////
//
// (C) Sam Hayhurst 2014
//
// Pinball class, derives from Object3D
//

#ifndef PINBALL_INCLUDED
#define PINBALL_INCLUDED

#include "Object3D.h"

namespace octet {
  namespace pinball {

    /// Pinball class, a simple 3d sphere, dynamic
    class Pinball : public Object3D {
    private:
      float radii;
      mesh_sphere *meshSphere;

    public:
      /// Pinball Constructor
      Pinball()
      {}
      /// Pinball Destructor
      ~Pinball(){
      }

      /// init function, mass defaults to 1.0 to ensure dynamic behavior within the scene
      void init_sphere(mat4t model2world, float rad, material *sphere_material, float sphere_mass = 1.0f) {
        init(model2world, sphere_material, sphere_mass);
        btCollisionShape *shape = new btSphereShape(btScalar(rad));
        btVector3 inertialTensor;
        shape->calculateLocalInertia(mass, inertialTensor);
        rigidbody = new btRigidBody(mass, motionState, shape, inertialTensor);
        // init mesh_box and scene node
        meshSphere = new mesh_sphere(vec3(0), rad);
        node = new scene_node(modelToWorld, atom_);
      }

      /// Adds the mesh and rigidbody of the sphere to the scene
      void add_to_scene(dynarray<scene_node*> &sceneNodes, ref<visual_scene> &appScene, btDiscreteDynamicsWorld &btWorld,
                        dynarray<btRigidBody*> &rigidBodies, bool is_visible = true, bool make_child = true) {
        btWorld.addRigidBody(rigidbody);
        rigidBodies.push_back(rigidbody);
        sceneNodes.push_back(node);
        printf("Pinball added to the scene\n");
        appScene->add_mesh_instance(new mesh_instance(node, meshSphere, mat));
        if (make_child) {
          appScene->add_child(node);
        }
      }

      /// Moves Pinball to position within world
      void setPosition(vec3 pos) {
        node->access_nodeToParent().loadIdentity();
      }
    };


  }
}

#endif 
