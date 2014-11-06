////////////////////////////////////////////////////////////////////////////////
//
// (C) Sam Hayhurst 2014
//
// Cylinder class, derived from 
//

#ifndef LAMP_INCLUDED
#define LAMP_INCLUDED

#include "Cylinder3D.h"

namespace octet {
  namespace pinball {

    class Lamp : public Cylinder3D {
    private: 
      float score;
      float multiplier;
      float multiplierInc;
      int lightIndex;
      scene_node *light_node;
      light *point_light;

      mesh_instance *mesh_instance_;

      // colours for the materials
      static const int NUM_COLOURS = 5;
      enum {WHITE = 0, GREEN, BLUE, PURPLE, ORANGE};
      static const vec4 COLOURS[NUM_COLOURS];

    public:

      // deafult blank constructor
      Lamp(){
      }

      // default deconstructor
      ~Lamp() {
        }

      Lamp(scene_node *node, float rad, float half_height, material *cylinder_material, mesh *_mesh, float cylinder_mass = 1.0f) {
        colladaMesh = _mesh;
        modelToWorld = node->access_nodeToParent();
        init_cylinder(modelToWorld, rad, half_height, cylinder_material, cylinder_mass);
      }

      /// initialises default values for scoring and the attached light
      void init_lamp(ref<visual_scene> &app_scene) {
        // scoring defaults
        score = 10;
        multiplier = 1.0f;
        multiplierInc = 1.5f;

        // lighting defaults
        lightIndex = WHITE;
        light_node = new scene_node();
        light_node->translate(vec3(0.0f, 2.0f, 0.0));
        point_light = new light();
        point_light->set_kind(atom_spot);
        point_light->set_color(COLOURS[WHITE]);
        point_light->set_attenuation(0.0f, 0.1f, 0.0f);
        node->add_child(light_node);
        app_scene->add_light_instance(new light_instance(light_node, point_light));

        rigidbody->setUserPointer(this);
      }

      // upgrade the lamp; multiplyer and the light
      void upgrade(){
        printf("upgrade called index is: %i", lightIndex);
        if (lightIndex < NUM_COLOURS - 1){
          multiplier *= multiplierInc;
          mesh_instance_->set_material(new material(COLOURS[++lightIndex]));
        }
      }

      // gets the score
      float getHitScore() {
        printf("Score: %4.2f\n", score);
        printf("multiplier: %2.4f\n", multiplier);
        printf("Current colour index: %i", lightIndex);
        return score * multiplier;
      }

      /// Adds the mesh and rigidbody of the cylinder to the scene, overidden to maintain the mesh instance
      void add_to_scene(dynarray<scene_node*> &sceneNodes, ref<visual_scene> &appScene, btDiscreteDynamicsWorld &btWorld,
        dynarray<btRigidBody*> &rigidBodies, bool is_visible = true, bool make_child = true) override {
        btWorld.addRigidBody(rigidbody);
        rigidBodies.push_back(rigidbody);
        sceneNodes.push_back(node);
        printf("Cylinder3D added to scene\n");
        if (is_visible && colladaMesh == NULL) {
          appScene->add_mesh_instance(new mesh_instance(node, meshCylinder, mat));
        }
        else if (is_visible && colladaMesh != NULL) {
          mesh_instance_ = new mesh_instance(node, colladaMesh, mat);
          appScene->add_mesh_instance(mesh_instance_);
        }
        if (make_child) {
          appScene->add_child(node);
        }
      }

    };

      const vec4 Lamp::COLOURS[NUM_COLOURS] = {  // with buffers
      vec4(1.0f, 1.0f, 1.0f, 1.0f) * 0.5f,   // white
      vec4(0.0f, 1.0f, 0.0f, 1.0f) * 0.5f,   // green
      vec4(0.0f, 0.0f, 1.0f, 1.0f) * 0.5f,   // blue
      vec4(0.5f, 0.0f, 1.0f, 1.0f) * 0.5f,   // purple
      vec4(1.0f, 0.5f, 0.0f, 1.0f) * 0.5f }; // orange
  }


}

#endif
