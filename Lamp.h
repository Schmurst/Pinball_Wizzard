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
      float multiplierBase;
      float multiplierInc;
      scene_node *light_node;
      light *point_light;

      // colours for the lights
      enum {WHITE = 0, GREEN, BLUE, PURPLE, ORANGE};
      static const vec4 COLOURS[5];

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
        multiplierBase = 1.0f;
        multiplierInc = 1.5f;

        // lighting defaults
        light_node = new scene_node();
        light_node->translate(vec3(0.0f, 1.2f, 0.0));
        point_light = new light();
        point_light->set_kind(atom_point);
        point_light->set_color(COLOURS[WHITE]);
        node->add_child(light_node);
        app_scene->add_light_instance(new light_instance(light_node, point_light));
      }

    };

    const vec4  Lamp::COLOURS[5] = { 
      vec4(1.0f, 1.0f, 1.0f, 1.0f),   // white
      vec4(0.0f, 1.0f, 0.0f, 1.0f),   // green
      vec4(0.0f, 0.0f, 1.0f, 1.0f),   // blue
      vec4(0.5f, 0.0f, 1.0f, 1.0f),   // purple
      vec4(1.0f, 0.5f, 0.0f, 1.0f) }; // orange
  }


}

#endif
