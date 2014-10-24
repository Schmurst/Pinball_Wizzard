////////////////////////////////////////////////////////////////////////////////
//
// (C) Sam Hayhurst 2014
//
// Cylinder class, derived from 
//

#ifndef CYLINDER3D_INCLUDED
#define CYLINDER3D_INCLUDED

#include "Object3D.h"

namespace octet {
  namespace pinball {

    /// Cylinder3D class, a simple 3d cylinder, dynamic
    class Cylinder3D : public Object3D {
    private:
      float radii;
      float height;
      mesh_cylinder *meshCylinder;

    public:
      /// Cylinder3D Constructor
      Cylinder3D()
      {}

      /// Cylinder 4 arg constructor using modeltoworld
      Cylinder3D(mat4t model2world, float rad, float half_height, material *cylinder_material, float cylinder_mass = 1.0f) {
        init_cylinder(model2world, rad, half_height, cylinder_material, cylinder_mass);
      }

      /// Cylinder 4 arg constructor using ready made node
      Cylinder3D(scene_node *node, float rad, float half_height, material *cylinder_material, float cylinder_mass = 1.0f) {
        modelToWorld = node->access_nodeToParent();
        init_cylinder(modelToWorld, rad, half_height, cylinder_material, cylinder_mass);
      }

      /// Cylinder3D Destructor
      ~Cylinder3D(){
      }

      /// init function, mass defaults to 1.0 to ensure dynamic behavior within the scene
      void init_cylinder(mat4t model2world, float rad, float half_height, material *cylinder_material, float cylinder_mass = 1.0f) {
        init(model2world, cylinder_material, cylinder_mass);
        radii = rad;
        height = half_height;
        btCollisionShape *shape = new btCylinderShape(btVector3(radii, radii, height));
        btVector3 inertialTensor;
        shape->calculateLocalInertia(mass, inertialTensor);
        rigidbody = new btRigidBody(mass, motionState, shape, inertialTensor);
        // init mesh_box and scene node
        meshCylinder = new mesh_cylinder(zcylinder(vec3(), radii, height));
        node = new scene_node(modelToWorld, atom_);
      }

      /// Adds the mesh and rigidbody of the cylinder to the scene
      void add_to_scene(dynarray<scene_node*> &sceneNodes, ref<visual_scene> &appScene, btDiscreteDynamicsWorld &btWorld,
                        dynarray<btRigidBody*> &rigidBodies, bool is_visible = true, bool make_child = true) {
       btWorld.addRigidBody(rigidbody);
       rigidBodies.push_back(rigidbody);
       sceneNodes.push_back(node);
       printf("Cylinder3D added to scene\n");
       appScene->add_mesh_instance(new mesh_instance(node, meshCylinder, mat));
       if (make_child) {
         appScene->add_child(node);
        }
      }

      /// Moves Cylinder3D to position within world
      void setPosition(vec3 pos) {
        node->access_nodeToParent().loadIdentity();
      }

      /// returns the cylinder mesh
      mesh_cylinder* get_mesh() {
        return meshCylinder;
      }
    };
  }
}

#endif