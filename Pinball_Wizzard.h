////////////////////////////////////////////////////////////////////////////////
//
// (C) Andy Thomason 2012-2014
//
// Modular Framework for OpenGLES2 rendering on multiple platforms.
//

#include "Object3D.h"
#include "Box3D.h"
#include "Pinball.h"
#include "Flipper.h"
#include "Table.h"

namespace octet {
  namespace pinball {

    /// Scene using bullet for physics effects. 
    class Pinball_Wizzard : public app {
      // scene for drawing box
      ref<visual_scene> app_scene;

      btDefaultCollisionConfiguration config;       /// setup for the world
      btCollisionDispatcher *dispatcher;            /// handler for collisions between objects
      btDbvtBroadphase *broadphase;                 /// handler for broadphase (rough) collision
      btSequentialImpulseConstraintSolver *solver;  /// handler to resolve collisions
      btDiscreteDynamicsWorld *world;               /// physics world, contains rigid bodies

      dynarray<btRigidBody*> rigid_bodies;
      dynarray<scene_node*> nodes;

      const float PI = 3.14159265f;

      // flipper & Pinball declaration is included here as they're common to all scopes/ functions below
      Flipper flipperR, flipperL;
      Pinball pinball;
      int flipDelayL = 0, flipDelayR = 0;
      int flipperCoolDown = 15; // frames between flips

    public:
      /// this is called when we construct the class before everything is initialised.
      Pinball_Wizzard(int argc, char **argv) : app(argc, argv) {
        dispatcher = new btCollisionDispatcher(&config);
        broadphase = new btDbvtBroadphase();
        solver = new btSequentialImpulseConstraintSolver();
        world = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, &config);
      }

      ~Pinball_Wizzard() {
        delete world;
        delete solver;
        delete broadphase;
        delete dispatcher;
      }

      /// this is called once OpenGL is initialized
      void app_init() {
        app_scene = new visual_scene();
        app_scene->create_default_camera_and_lights();
        app_scene->get_camera_instance(0)->get_node()->rotate(-20.0f, vec3(1.0, 0, 0));
        app_scene->get_camera_instance(0)->get_node()->translate(vec3(0, 10.0f, -8.5f));
        world->setGravity(btVector3(0, -9.81f, 0));

        // Import collada from file.
        resource_dict dict;
        collada_builder colladaBuilder;
        if (!colladaBuilder.load_xml("assets/PinballTable.dae")) {
          printf("failed to load the pinball table file");
          return;
        }
        // get meshes and their respective nodes from dictionary
        colladaBuilder.get_resources(dict);
        dynarray<resource*> collada_meshes;
        dict.find_all(collada_meshes, atom_mesh);
        printf("collada_meshes size: %i\n", collada_meshes.size());

        // stand in material for table
        material *temp_mat = new material(vec4(0.2f, 0.5f, 0.8f, 1.0f));

        // put the meshes and nodes in the scene... hopefully
        for (unsigned int i = 0; i < collada_meshes.size(); i++) {
          mesh *table_mesh = collada_meshes[i]->get_mesh();
          scene_node *table_node = dict.get_scene_node("TableNode");
          table_node->rotate(30.0f, vec3(1.0f, 0, 0));
          app_scene->add_child(table_node);
          app_scene->add_mesh_instance(new mesh_instance(table_node, table_mesh, temp_mat));
        }

        // Add a a fill light to the scene
        scene_node *light_node = new scene_node();
        light *light_fill = new light();
        light_fill->set_color(vec4(0.0f, 0.0f, 1.0f, 1.0f));
        light_fill->set_attenuation(1, 0, -1);
        light_node->rotate(-45, vec3(1, 0, 0));
        light_node->translate(vec3(20, 0, 20));
        app_scene->add_light_instance(new light_instance(light_node, light_fill));

        camera_instance *camera = app_scene->get_camera_instance(0);
        mat4t modelToWorld;

        ////////////////////////////////////////////////// table Rigidbody construction ///////////////////////////////////////////

        bool is_visible = true; // 1.0: visible for debug, 0.0f: invisible
        Box3D table, BarrierTop, BarrierL, BarrierR, BarrierBot;
        float tableWidth = 4.95f;
        float tableDepth = 0.5f;
        float tableLength = 10.0f;
        float barrierRestitution = 0.5f;
        float barrierThickness = 0.2f;
        material *barrier_mat = new material(vec4(0.1f, 0.8f, 0.1f, 1.0f));
        material *table_mat = new material(vec4(0, 1.0f, 0, 1.0f));

        // Table
        modelToWorld.loadIdentity();
        modelToWorld.translate(0.f, -0.6f, 0.0f);
        modelToWorld.rotateX(-30.0f);
        table.init_box(modelToWorld, vec3(tableWidth, tableDepth, tableLength), table_mat, 0.0f);    // mass = 0 -> static x: 1000 y: 100 z: 2000
        table.add_to_scene(nodes, app_scene, (*world), rigid_bodies, is_visible);

        // Barrier Left
        modelToWorld.translate(-tableWidth - tableDepth, tableDepth, 0);
        BarrierL.init_box(modelToWorld, vec3(tableDepth, tableDepth * 2, tableLength * 1.05f), barrier_mat, 0.0f);
        BarrierL.add_to_scene(nodes, app_scene, (*world), rigid_bodies, is_visible);
        BarrierL.getRigidBody()->setRestitution(barrierRestitution);


        //Barrier Right
        modelToWorld.translate(tableWidth * 2.0f + tableDepth * 2.0f, 0, 0);
        BarrierR.init_box(modelToWorld, vec3(tableDepth, tableDepth * 2, tableLength * 1.05f), barrier_mat, 0.0f);
        BarrierR.add_to_scene(nodes, app_scene, (*world), rigid_bodies, is_visible);
        BarrierR.getRigidBody()->setRestitution(barrierRestitution);

        // Barrier bottom
        BarrierBot.init_box(mat4t(), vec3(tableWidth, tableDepth, tableDepth), barrier_mat, 0.0f);
        BarrierBot.add_to_scene(nodes, app_scene, (*world), rigid_bodies, is_visible);
        BarrierBot.getRigidBody()->setRestitution(barrierRestitution);

        ////////////////////////////////////////////////// FLipper ///////////////////////////////////////////
        float torqueImpluse = 300.0f;
        float initialOffset = 10.0f;
        float halfheightFlipper = 0.2f;
        float halfwidthFlipper = 0.2f;
        float halflengthFlipper = 1.2f;
        float massFlipper = 8.0f;
        float flipperRestitution = 0.8f;
        material *flip_mat = new material(vec4(1.0f, 0, 0, 1.0f));

        btVector3 hingeOffsetR = btVector3(halflengthFlipper * 0.95f, 0, -halfheightFlipper);
        btVector3 hingeOffsetL = btVector3(-halflengthFlipper * 0.95f, 0, -halfheightFlipper);
        btVector3 tableOffsetR = btVector3(halflengthFlipper * 2.2f, 0.5f, 8.0f);
        btVector3 tableOffsetL = btVector3(-halflengthFlipper * 2.2f, 0.5f, 8.0f);
        vec3 sizeFlipper = vec3(halflengthFlipper, halfwidthFlipper, halfheightFlipper);

        // add right flipper to the scene
        modelToWorld.loadIdentity();
        modelToWorld.translate(initialOffset, initialOffset, initialOffset);  // to make the flipper fly in from off the screen
        modelToWorld.rotateX(-30.0f);
        flipperR.init_flipper(modelToWorld, sizeFlipper, flip_mat, vec3(0, 0, -1.0f) * torqueImpluse, massFlipper);
        flipperR.add_to_scene(nodes, app_scene, (*world), rigid_bodies);
        flipperR.getRigidBody()->setRestitution(flipperRestitution);

        // add left flipper to the scene
        modelToWorld.loadIdentity();
        modelToWorld.translate(-initialOffset, initialOffset, initialOffset);  // to make the flipper fly in from off the screen
        modelToWorld.rotateX(-30.0f);
        flipperL.init_flipper(modelToWorld, sizeFlipper, flip_mat, vec3(0, 0, 1.0f) * torqueImpluse, massFlipper); // x: 100 y: 20 z: 20
        flipperL.add_to_scene(nodes, app_scene, (*world), rigid_bodies);
        flipperL.getRigidBody()->setRestitution(flipperRestitution);

        // Add a constraint between flipper and table
        btHingeConstraint *hingeFlipperRight = new btHingeConstraint((*table.getRigidBody()), (*flipperR.getRigidBody()),
                                                                      tableOffsetR, hingeOffsetR,         // this are the hinge offset vectors
                                                                      btVector3(0, 1.0f, 0), btVector3(0, 0, 1.0f), false);
        btHingeConstraint *hingeFlipperLeft = new btHingeConstraint((*table.getRigidBody()), (*flipperL.getRigidBody()),
                                                                      tableOffsetL, hingeOffsetL,       // this are the hinge offset vectors
                                                                      btVector3(0, 1.0f, 0), btVector3(0, 0, 1.0f), false);

        // set angle limits on the flippers
        hingeFlipperLeft->setLimit(-PI * 0.2f, PI * 0.2f);
        hingeFlipperRight->setLimit(-PI * 0.2f, PI * 0.2f, 0.0f, 0.8f, 0.5f);

        // add constraints to world
        world->addConstraint(hingeFlipperLeft);
        world->addConstraint(hingeFlipperRight);

        ////////////////////////////////////////////////// Pinball ///////////////////////////////////////////
        // Add the pinball to the world
        material *sphere_mat = new material(vec4(1.0f, 0, 0.8f, 1.0f));
        float pinballRestitution = 1.0f;
        modelToWorld.loadIdentity();
        modelToWorld.translate(1.0f, 6.0f, 0.0f);
        pinball.init_sphere(modelToWorld, 0.2f, sphere_mat, 1.0f);
        pinball.add_to_scene(nodes, app_scene, *world, rigid_bodies);
        pinball.getRigidBody()->setRestitution(pinballRestitution);
      }

      /// this is called to draw the world
      void draw_world(int x, int y, int w, int h) {
        int vx, vy;
        vx = vy = 0;
        get_viewport_size(vx, vy);
        app_scene->begin_render(vx, vy);

        world->stepSimulation(1.0f / 60);
        for (unsigned i = 0; i != rigid_bodies.size(); ++i) {
          btRigidBody *rigid_body = rigid_bodies[i];
          btQuaternion btq = rigid_body->getOrientation();
          btVector3 pos = rigid_body->getCenterOfMassPosition();
          quat q(btq[0], btq[1], btq[2], btq[3]);
          mat4t modelToWorld = q;
          modelToWorld[3] = vec4(pos[0], pos[1], pos[2], 1);
          nodes[i]->access_nodeToParent() = modelToWorld;
        }

        // decrement flippers
        if (flipDelayL > 0) {
          flipDelayL--;
        }

        if (flipDelayR > 0) {
          flipDelayR--;
        }

        // Key handlers, when pushed will flip the flippers
        if (is_key_down('Z') && flipDelayR == 0) {
          flipperL.flip();
          flipDelayR = flipperCoolDown;
        }

        if (is_key_down('M') && flipDelayL == 0) {
          flipperR.flip();
          flipDelayL = flipperCoolDown;
        }

        // update matrices. assume 30 fps.
        app_scene->update(1.0f / 30);      // draw the scene
        app_scene->render((float)vx / vy);
      }
    };
  }
}
