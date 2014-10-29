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
      btTransform trans;
      // following variables are used to impement randomness in the pinball drop
      random *seed;
      mat4t matrix;
      vec3 vec;
      ALuint bang;                // temp usage
      unsigned current_source;    // current sound source
      unsigned int sound_barrier_check;
      float previous_speed; // used to store the previous physics step's speed (actually speed squared)
      float speed_check;

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

        seed = new random();
        matrix.loadIdentity();

        // Sounds
        const ALuint num_sound_sources = 1;   // number of sound sources in scene, probably one?
        ALuint sources[num_sound_sources]; // what is this even for?
        bang = resource_dict::get_sound_handle(AL_FORMAT_MONO16, "assets/invaderers/bang.wav");
        alGenSources(num_sound_sources, sources);
        sound_barrier_check = 0;

        // collision values
        speed_check = 0.8f;
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
      void reset() {
        float x = seed->get(8.0f, 10.0f);
        vec = vec3(x, 1.0f, 1.0f);
        printf("Randomx: %f\n", x);
        trans = btTransform(get_btMatrix3x3(matrix), get_btVector3(vec));
        rigidbody->setWorldTransform(trans);
        rigidbody->setLinearVelocity(get_btVector3(vec3(0, 0, 0)));
        rigidbody->setAngularVelocity(get_btVector3(vec3(0, 0, 0)));
      }

      // play sound on barrier hit
      void hitBarrier() {
        alSourcei(0, AL_BUFFER, bang);
        alSourcePlay(0);
      }

      /// Sets the previous speed of the pinball, called everyphysics step
      void updateSpeed() {
        btVector3 velocity = rigidbody->getInterpolationLinearVelocity();
        previous_speed = velocity[0] * velocity[1] * velocity[2] * velocity[0] * velocity[1] * velocity[2];
      }

      /// detectes whether a significant impact has been detected
      bool isImpact() {
        float current_speed, acceleration;
        btVector3 velocity = rigidbody->getInterpolationLinearVelocity();
        current_speed = velocity[0] * velocity[1] * velocity[2] * velocity[0] * velocity[1] * velocity[2];
        
        acceleration = (current_speed - previous_speed) / current_speed;
        printf("pinball Current Speed: %f\n", current_speed);
        printf("pinball prev Speed: %f\n", previous_speed);
        printf("pinball Acceleration: %f\n", acceleration);
        return (acceleration >= speed_check || acceleration <= -speed_check) ? true : false;
      }

    };

  }
}

#endif 
