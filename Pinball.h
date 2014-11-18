////////////////////////////////////////////////////////////////////////////////
//
//  Sam Hayhurst 2014
//
// Pinball class, derives from Object3D
// Contains logic 

#ifndef PINBALL_INCLUDED
#define PINBALL_INCLUDED

#include "Object3D.h"

namespace octet {
  namespace pinball {
    /// Pinball class used to emulate a pinball for playing Pinball Wizzard.
    /// contains functionality to get and play sounds and set position.
    /// Lastly contains functions used to limit speed and detect whether a significant imapct has occured.
    class Pinball : public Object3D {
    private:
      float radii;
      mesh_sphere *meshSphere;
      btTransform trans;

      // following variables are used to impement randomness in the pinball drop
      random *seed;
      mat4t matrix;
      vec3 vec;

      // used to compare speeds and collsisions
      btScalar current_speed;
      btScalar previous_speed;
      float speed_check;
      btScalar maxSpeed;

      // sounds
      ALuint Pop;
      ALuint Bounce;
      ALuint Ding;
      ALuint Welcome;
      ALuint Drop;
      ALuint Flip;
      ALuint Randomise;
      unsigned current_source;    // current sound source
      unsigned int sound_barrier_check;
      ALuint num_sound_sources = 32;
      ALuint sources[32];    // 32 sound sources

      /// returns new sound source
      ALuint get_sound_source() {
        return sources[current_source++ % num_sound_sources];
      }

    public:
      /// Pinball Constructor.
      Pinball()
      {}
      /// Pinball Destructor.
      ~Pinball(){
      }

      /// initialise function, mass defaults to 1.0 to ensure dynamic behavior within the scene.
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
        rigidbody->setActivationState(DISABLE_DEACTIVATION);    // disables the deactiveation state

        // Sounds
        current_source = 0;
        Pop = resource_dict::get_sound_handle(AL_FORMAT_MONO16, "assets/Pinball_Wizzard/Pop.wav");
        Ding = resource_dict::get_sound_handle(AL_FORMAT_MONO16, "assets/Pinball_Wizzard/Ding.wav");
        Drop = resource_dict::get_sound_handle(AL_FORMAT_MONO16, "assets/Pinball_Wizzard/Drop.wav");
        Bounce = resource_dict::get_sound_handle(AL_FORMAT_MONO16, "assets/Pinball_Wizzard/Bounce.wav");
        Flip = resource_dict::get_sound_handle(AL_FORMAT_MONO16, "assets/Pinball_Wizzard/Flip.wav");
        Welcome = resource_dict::get_sound_handle(AL_FORMAT_MONO16, "assets/Pinball_Wizzard/Welcome.wav");
        Randomise = resource_dict::get_sound_handle(AL_FORMAT_MONO16, "assets/Pinball_Wizzard/randomise.wav");
        alGenSources(num_sound_sources, sources);
        sound_barrier_check = 0;

        // collision values
        speed_check = 0.8f;
        maxSpeed = 60.0f;
      }

      /// Adds the mesh and rigidbody of the sphere the btcollision world and the app_scene.
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

      /// Moves Pinball to randomised position within the world
      void reset() {
        float x = seed->get(3.5f, 7.5f);
        vec = vec3(x, 1.0f, 1.0f);
        printf("Randomx: %f\n", x);
        trans = btTransform(get_btMatrix3x3(matrix), get_btVector3(vec));
        rigidbody->setWorldTransform(trans);
        rigidbody->setLinearVelocity(get_btVector3(vec3(0, 0, 0)));
        rigidbody->setAngularVelocity(get_btVector3(vec3(0, 0, 0)));
      }

      /// play barrier hit sound
      void playSoundHitBarrier() {
        ALuint source = get_sound_source();
        alSourcei(source, AL_BUFFER, Pop);
        alSourcePlay(source);
      }

      /// play sound on bumper hit
      void playSoundHitBumper() {
        ALuint source = get_sound_source();
        alSourcei(source, AL_BUFFER, Ding);
        alSourcePlay(source);
      }

      /// play sound on Launcher hit
      void playSoundHitLauncher() {
        ALuint source = get_sound_source();
        alSourcei(source, AL_BUFFER, Bounce);
        alSourcePlay(source);
      }

      /// play sound on Launcher hit
      void playSoundRandomise() {
        ALuint source = get_sound_source();
        alSourcei(source, AL_BUFFER, Randomise);
        alSourcePlay(source);
      }

      /// play sound on Flipper hit
      void playSoundHitFlipper() {
        ALuint source = get_sound_source();
        alSourcei(source, AL_BUFFER, Flip);
        alSourcePlay(source);
      }

      /// play sound on ball drop
      void playSoundBallDrop() {
        ALuint source = get_sound_source();
        alSourcei(source, AL_BUFFER, Drop);
        alSourcePlay(source);
      }

      /// play sound on barrier hit
      void playSoundWelcome() {
        ALuint source = get_sound_source();
        alSourcei(source, AL_BUFFER, Welcome);
        alSourcePlay(source);
      }

      /// limits the speed of the Pinball, designed to be called every physics step.
      void limitSpeed() {
        btVector3 velocity = rigidbody->getLinearVelocity();
        btScalar speed = velocity.length();
        if (speed > maxSpeed) {
          velocity *= maxSpeed / speed;
          rigidbody->setLinearVelocity(velocity);
        }
      }

      /// sets a previous speed, used in conjunction with isImpact to determine whether an impact is significant.
      void updateSpeed() {
        btVector3 velocity = rigidbody->getInterpolationLinearVelocity();
        previous_speed = velocity.length();
      }

      /// detectes whether a significant impact has been detected, used in conjunction with updateSpeed.
      bool isImpact() {
        btVector3 velocity = rigidbody->getInterpolationLinearVelocity();
        current_speed = velocity.length();

        //printf("pinball Current Speed: %f\n", current_speed);
        //printf("pinball prev Speed: %f\n", previous_speed);

        float acceleration = abs(previous_speed - current_speed);
        return (acceleration >= speed_check) ? true : false;
      }

      /// Returns false if ball has dropped ie its y position has dropped below -15.
      bool isDropped() {
        float Ypos = rigidbody->getWorldTransform().getOrigin()[1];
        return (Ypos < -15.0) ? true : false;
      }

    };
  }
}

#endif 
