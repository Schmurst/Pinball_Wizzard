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
        multiplier = 1.0f;
        multiplierInc = 1.5f;

        // lighting defaults
        lightIndex = WHITE;
        light_node = new scene_node();
        light_node->translate(vec3(0.0f, 0.8f, 0.0));
        point_light = new light();
        point_light->set_kind(atom_point);
        point_light->set_color(COLOURS[WHITE]);
        // does this even work?
        // may do with the shader, ie the lack of a shader
        //point_light->set_falloff(45.0f, 0.00001f);
        //point_light->set_near_far(0.01f, 1.0f);
        point_light->set_attenuation(0.0f, 0.5f, 0.0f);
        node->add_child(light_node);
        app_scene->add_light_instance(new light_instance(light_node, point_light));
      }

      // upgrade the lamp; multiplyer and the light
      void upgrade(){
        if (lightIndex < COLOURS->length())
        point_light->set_color(COLOURS[++lightIndex]);
        multiplier *= multiplierInc;
      }

      // 
      float getHitScore() {
        printf("Score: %4.2f\n", score);
        printf("multiplier: %2.4f\n", multiplier);
        printf("Current colour index: %i", lightIndex);
        return score * multiplier;
      }

    };

    const vec4 Lamp::COLOURS[5] = {  // with buffers
      vec4(1.0f, 1.0f, 1.0f, 1.0f) * 0.5f,   // white
      vec4(0.0f, 1.0f, 0.0f, 1.0f) * 0.5f,   // green
      vec4(0.0f, 0.0f, 1.0f, 1.0f) * 0.5f,   // blue
      vec4(0.5f, 0.0f, 1.0f, 1.0f) * 0.5f,   // purple
      vec4(1.0f, 0.5f, 0.0f, 1.0f) * 0.5f }; // orange
  }


}

#endif
