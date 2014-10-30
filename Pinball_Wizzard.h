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
#include "Cylinder3D.h"
#include <Xinput.h>

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

      // debugs
      bool collada_debug = true;
      bool runtime_debug = true;

      // create an enum used to specify certain object types for collision logic
      enum obj_types { PINBALL = 0, FLIPPER = 1, TABLE = 2, BARRIER = 3, BUMPER = 4, FACE = 5 };

      // flipper & Pinball declaration is included here as they're common to all scopes/ functions below
      Flipper flipperR, flipperL;
      Pinball pinball;
      int flipDelayL = 0, flipDelayR = 0, pinballResetDelay = 0;
      int soundPopDelay = 0, speedUpdateDelay = 0, soundDingDelay = 0;
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
        app_scene->get_camera_instance(0)->get_node()->rotate(-22.0f, vec3(1.0, 0, 0));
        app_scene->get_camera_instance(0)->get_node()->translate(vec3(0, 10.0f, -9.5f));
        app_scene->get_camera_instance(0)->set_perspective(0, 70, 1, 0.1f, 1000.0f);
        world->setGravity(btVector3(0, -19.81f, 0));
        mat4t modelToWorld;

        // Add a a fill light to the scene and create camera instance
        scene_node *light_node = new scene_node();
        light *light_fill = new light();
        light_fill->set_color(vec4(0.0f, 0.0f, 1.0f, 1.0f));
        light_fill->set_attenuation(1, 0, -1);
        light_node->rotate(-45, vec3(1, 0, 0));
        light_node->translate(vec3(20, 0, 20));
        app_scene->add_light_instance(new light_instance(light_node, light_fill));

        ////////////////////////////////////////////////// Pinball ///////////////////////////////////////////
        // Add the pinball to the world
        material *sphere_mat = new material(new image("assets/Pinball_Wizzard/Eye.gif"));
        float pinballRestitution = 1.0f;
        modelToWorld.loadIdentity();
        modelToWorld.translate(0.7f, 6.0f, -2.0f);
        pinball.init_sphere(modelToWorld, 0.5f, sphere_mat, 1.0f);
        pinball.add_to_scene(nodes, app_scene, *world, rigid_bodies);
        pinball.getRigidBody()->setRestitution(pinballRestitution);
        pinball.getRigidBody()->setDamping(0.05f, 0.05f);
        pinball.getRigidBody()->setUserIndex(PINBALL);

        ////////////////////////////////////////////////// FLipper ///////////////////////////////////////////
        float torqueImpluse = 250.0f;
        float initialOffset = 10.0f;
        float halfheightFlipper = 0.4f;
        float halfwidthFlipper = 0.1f;
        float halflengthFlipper = 1.0f;
        float massFlipper = 8.0f;
        float flipperRestitution = 0.8f;
        material *flip_mat = new material(vec4(1.0f, 0, 0, 1.0f));

        btVector3 hingeOffsetR = btVector3(halflengthFlipper * 0.95f, 0, halfheightFlipper * -1.2f);
        btVector3 hingeOffsetL = btVector3(halflengthFlipper * -0.95f, 0, halfheightFlipper * -1.2f);
        btVector3 tableOffsetR = btVector3( 0.6f, -11.3f, 0.4f);
        btVector3 tableOffsetL = btVector3(-3.6f, -11.3f, 0.4f);
        vec3 sizeFlipper = vec3(halflengthFlipper, halfwidthFlipper, halfheightFlipper);

        // add right flipper to the scene
        modelToWorld.loadIdentity();
        modelToWorld.translate(initialOffset, initialOffset, initialOffset);  // to make the flipper fly in from off the screen
        modelToWorld.rotateX(-30.0f);
        flipperR.init_flipper(modelToWorld, sizeFlipper, flip_mat, vec3(0, 0, -1.0f) * torqueImpluse, massFlipper);
        flipperR.add_to_scene(nodes, app_scene, (*world), rigid_bodies);
        flipperR.getRigidBody()->setRestitution(flipperRestitution);
        flipperR.getRigidBody()->setUserIndex(FLIPPER);

        // add left flipper to the scene
        modelToWorld.loadIdentity();
        modelToWorld.translate(-initialOffset, initialOffset, initialOffset);  // to make the flipper fly in from off the screen
        modelToWorld.rotateX(-30.0f);
        flipperL.init_flipper(modelToWorld, sizeFlipper, flip_mat, vec3(0, 0, 1.0f) * torqueImpluse, massFlipper); // x: 100 y: 20 z: 20
        flipperL.add_to_scene(nodes, app_scene, (*world), rigid_bodies);
        flipperL.getRigidBody()->setRestitution(flipperRestitution);
        flipperL.getRigidBody()->setUserIndex(FLIPPER);

        ////////////////////////////////////////////////// Collada import ///////////////////////////////////////////
        // create dictionary & collada builder
        resource_dict dict;
        dynarray<resource*> collada_meshes;
        collada_builder colladaBuilder;
        if (!colladaBuilder.load_xml("assets/Pinball_Wizzard/PinballWizzardAssets.dae")) {
          printf("failed to load the pinball table file");
          return;
        }

        // get meshes and their respective nodes from dictionary
        colladaBuilder.get_resources(dict);
        dict.find_all(collada_meshes, atom_mesh);
        printf("collada_meshes size: %i\n", collada_meshes.size());

        // part list, taken from collada file, very important to keep uptodate
        dynarray <string> table_parts;
        table_parts.push_back("Table");       
        table_parts.push_back("BarrierLeft");   
        table_parts.push_back("BarrierRight");  
        table_parts.push_back("BarrierTop");    
        table_parts.push_back("Bumper001");     
        table_parts.push_back("Bumper002");     
        table_parts.push_back("Bumper003");     
        table_parts.push_back("Bumper004");     
        table_parts.push_back("Bumper005");     
        table_parts.push_back("Bumper006");     
        table_parts.push_back("BumperLeft");    
        table_parts.push_back("BumperRight");   
        table_parts.push_back("BumperEyeLeft"); 
        table_parts.push_back("BumperEyeRight");
        table_parts.push_back("EyeBrowLeft");   
        table_parts.push_back("EyeBrowRight");  
        table_parts.push_back("BarrierReflector");             
        table_parts.push_back("BumperLauncher");      
        table_parts.push_back("BumperMouth");   
        table_parts.push_back("Glass");         

        // Materials
        material *table_mat = new material(new image("assets/Pinball_Wizzard/nebula.gif"));
        material *barrier_mat = new material(vec4(0.8f, 0.5f, 0.2f, 1.0f));
        material *bumper_mat = new material(vec4(0.5f, 0.8f, 0.2f, 1.0f));
        material *wizzard_mat = new material(vec4(0.1f, 0.6f, 0.1f, 1.0f));
        material *error_mat = new material(vec4(1.0f, 0, 0, 1.0f));

        // put the meshes and nodes in the scene... hopefully
        scene_node *node_part;
        mesh *mesh_part;
        dynarray <Object3D*> table_boxes;

        // Ensure always the the table node is at origin in blender
        // make parent scene node (the table node from collada)
        // rotate the node
        // move it and (SCALE IN BLENDER ALWAYS)
        // instead of adding the barrier nodes to the directly to the scene
        // instead add them as childs of the parent node (table node)

        // tabe parent node
        modelToWorld.loadIdentity();
        scene_node *table_parent = new scene_node();
        table_parent->access_nodeToParent() = modelToWorld;
        table_parent->rotate(60.0f, vec3(1.0f, 0, 0));
        app_scene->add_child(table_parent);

        vec4 x = modelToWorld.x();
        vec4 y = modelToWorld.y();
        vec4 z = modelToWorld.z();
        printf("\nTABLE NODE\n");
        printf("      model to world matrix\n");
        printf("x: %f y: %f z: %f \n", x[0], x[1], x[2]);
        printf("x: %f y: %f z: %f \n", y[0], y[1], y[2]);
        printf("x: %f y: %f z: %f \n", z[0], z[1], z[2]);
        
        // now for the table parts
        for (unsigned int i = 0; i < table_parts.size(); i++) {
          // get the node and mesh of each object in table parts list
          node_part = dict.get_scene_node(table_parts[i]);
          table_parent->add_child(node_part);
          table_parts[i] += "-mesh";
          mesh_part = dict.get_mesh(table_parts[i]);

          // create axis_aligned bounding box
          aabb aabb_part = mesh_part->get_aabb();
          // initialise bt box shape, using centre + halfextents (absolute to avoid stange errors)
          vec3 size = (aabb_part.get_center().abs() + aabb_part.get_half_extent().abs());

          // the following check decides whether a collada mesh instance should be converted into a 
          // Box3D object or a Cylinder3D object depending on the collada mesh name

          if (table_parts[i].find("Table") != -1) {
            table_boxes.push_back(new Box3D(node_part, size, table_mat, 0.0f));
            table_boxes[i]->getRigidBody()->setUserIndex(TABLE);
          }
          else if (table_parts[i].find("Barrier") != -1) {
            table_boxes.push_back(new Box3D(node_part, size, barrier_mat, 0.0f));
            table_boxes[i]->getRigidBody()->setUserIndex(BARRIER);
          }
          else if (table_parts[i].find("Brow") != -1) {
            table_boxes.push_back(new Box3D(node_part, size, wizzard_mat, 0.0f));
            table_boxes[i]->getRigidBody()->setUserIndex(FACE);
          }
          else if (table_parts[i].find("Bumper") != -1) {
            float radii, height;
            radii = size[0];
            height = size[2];
            if (table_parts[i].find("Launcher") != -1) {
              table_boxes.push_back(new Box3D(node_part, size, error_mat, 0.0f));
              table_boxes[i]->getRigidBody()->setUserIndex(BARRIER);
            }
            else if (table_parts[i].find("Eye") != -1 || table_parts[i].find("Mouth") != -1) {
              table_boxes.push_back(new Cylinder3D(node_part, radii, height, wizzard_mat, 0.0f));
              table_boxes[i]->getRigidBody()->setUserIndex(FACE);
            }
            else {
              table_boxes.push_back(new Cylinder3D(node_part, radii, height, bumper_mat, 0.0f));
              table_boxes[i]->getRigidBody()->setUserIndex(BUMPER);
            }
            table_boxes[i]->setMesh(mesh_part);
          }
          else {
            printf("Collada mesh Object name not recognised, default Box3D loader used");
            table_boxes.push_back(new Box3D(node_part, size, error_mat, 0.0f));
            table_boxes[i]->getRigidBody()->setUserIndex(TABLE);
          }

          // for debug mode
          if (collada_debug) {
            printf("\n --------------------------------------------------------------------");
            printf("\nName of mesh: %s \n", table_parts[i]);
            printf("Half extents  x: %f y: %f z: %f \n", size[0], size[1], size[2]);
            printf("aabb center   x: %f y: %f z: %f \n", aabb_part.get_center()[0], aabb_part.get_center()[1], aabb_part.get_center()[2]);
            printf("aabb extents  x: %f y: %f z: %f \n", aabb_part.get_half_extent()[0], aabb_part.get_half_extent()[1], aabb_part.get_half_extent()[2]);
            printf("mesh extents  x: %f y: %f z: %f \n", mesh_part->get_size(0), mesh_part->get_size(1), mesh_part->get_size(2));
            vec3 pos = node_part->access_nodeToParent()[3].xyz();
            modelToWorld = node_part->access_nodeToParent();
            vec4 x = modelToWorld.x();
            vec4 y = modelToWorld.y();
            vec4 z = modelToWorld.z();
            printf("node position x: %f y: %f z: %f \n", pos[0], pos[1], pos[2]);
            printf("      model to world matrix\n");
            printf("x: %f y: %f z: %f \n", x[0], x[1], x[2]);
            printf("x: %f y: %f z: %f \n", y[0], y[1], y[2]);
            printf("x: %f y: %f z: %f \n", z[0], z[1], z[2]);
          }
        }

        // this code loops throught the boxes created by the collada file and allows rotation and transformation
        // its does this by manipulating the node then setting the rigidbodies orientation.
        // it is important to set the rigidbodies orientation as that is what is updated in the update function

        mat4t partToTable, tableToScene;
        tableToScene = table_parent->access_nodeToParent();
        btTransform tableTransform = btTransform(get_btMatrix3x3(tableToScene), get_btVector3(tableToScene[3].xyz()));

        for (unsigned int i = 0; i < table_boxes.size(); i++) {
          partToTable = table_boxes[i]->getNode()->access_nodeToParent();
          btVector3 pos = get_btVector3(partToTable[3].xyz());
          btMatrix3x3 matrix = get_btMatrix3x3(partToTable);
          btTransform partTransform = btTransform(matrix, pos);
          btRigidBody *rigidbody = table_boxes[i]->getRigidBody();
          rigidbody->setWorldTransform(tableTransform * partTransform);
          if (table_parts[i].find("Glass") == -1) {
            table_boxes[i]->add_to_scene(nodes, app_scene, (*world), rigid_bodies, true, false);
          }
          else {
            table_boxes[i]->add_to_scene(nodes, app_scene, (*world), rigid_bodies, false, false);
          }
        }
          
        // this code will loop throught the rigidbodies and set the right restitution for the parts
        for (unsigned int i = 0; i < table_parts.size(); i++) {
          if (table_parts[i].find("Barrier") != -1) {
            table_boxes[i]->getRigidBody()->setRestitution(0.8f);
          } 

          if (table_parts[i].find("Bumper") != -1) {
            table_boxes[i]->getRigidBody()->setRestitution(1.5f);
          }

          if (table_parts[i].find("Mouth") != -1 || table_parts[i].find("Launcher") != -1) {
            table_boxes[i]->getRigidBody()->setRestitution(5.0f);
          }

        }

        ///////////////////////////////////// Hinge Constraints ///////////////////////////////
        // Load the Hinge Constraints
        // get central table rigid body from rigidbodies dynarray
        btRigidBody *table = rigid_bodies[3];

        // Add a constraint between flipper and table
        btHingeConstraint *hingeFlipperRight = new btHingeConstraint( (*table), (*flipperR.getRigidBody()),
                                                                      tableOffsetR, hingeOffsetR,         // this are the hinge offset vectors
                                                                      btVector3(0, 0, 1.0f), btVector3(0, 0, 1.0f), false);
        btHingeConstraint *hingeFlipperLeft = new btHingeConstraint( (*table), (*flipperL.getRigidBody()),
                                                                      tableOffsetL, hingeOffsetL,       // this are the hinge offset vectors
                                                                      btVector3(0, 0, 1.0f), btVector3(0, 0, 1.0f), false);

        // set angle limits on the flippers
        hingeFlipperLeft->setLimit(-PI * 0.2f, PI * 0.2f);
        hingeFlipperRight->setLimit(-PI * 0.2f, PI * 0.2f, 0.0f, 0.8f, 0.5f);

        // add constraints to world
        world->addConstraint(hingeFlipperLeft);
        world->addConstraint(hingeFlipperRight);

        // reset the pinball
        pinball.reset();

        // add the skybox sphere to the world no rigidbody
        modelToWorld.loadIdentity();
        modelToWorld.rotateY90();
        material *skybox_mat = new material(new image("assets/Pinball_Wizzard/largeGalField.gif"));
        scene_node *skybox_node = new scene_node(modelToWorld, atom_);
        mesh_sphere *skybox_mesh = new mesh_sphere(vec3(0), 60.0f);
        nodes.push_back(skybox_node);
        app_scene->add_mesh_instance(new mesh_instance(skybox_node, skybox_mesh, skybox_mat));

        // check what user indexes are in the scene, duplicate check
        for (unsigned int i = 0; i < rigid_bodies.size(); i++) {
          printf("userpointer: %i\n", rigid_bodies[i]->getUserIndex());
        }

      }

      /// this is called to draw the world
      void draw_world(int x, int y, int w, int h) {
        int vx, vy;
        vx = vy = 0;
        get_viewport_size(vx, vy);
        app_scene->begin_render(vx, vy);

        if (speedUpdateDelay == 0) {
          pinball.updateSpeed();
          speedUpdateDelay += 5;
        }

        // collision handler
        int numManifolds = world->getDispatcher()->getNumManifolds();
        if (runtime_debug) printf("------new physics step--------\n");
        for (int i = 0; i<numManifolds; i++)
        {
          btPersistentManifold* contactManifold = world->getDispatcher()->getManifoldByIndexInternal(i);
          int objA = contactManifold->getBody0()->getUserIndex();
          int objB = contactManifold->getBody1()->getUserIndex();

          // check what has hit what
          if (objA == PINBALL || objB == PINBALL) {
            if (objA == FLIPPER || objB == FLIPPER) {
              if (runtime_debug) printf("The pinball has hit a FLIPPER\n");
            } 
            else if (objA == BUMPER || objB == BUMPER) {
              if (runtime_debug) printf("The pinball has hit a BUMPER\n");
              if (soundDingDelay == 0 && pinball.isImpact()) {
                pinball.playSoundHitBumper();
                soundDingDelay += 5;
              }
            }
            else if (objA == FACE || objB == FACE) {
              if (runtime_debug) printf("The pinball has hit the FACE\n");
            }
            else if (objA == BARRIER || objB == BARRIER) {
              if (runtime_debug) printf("The pinball has hit the FACE\n");
              if (soundPopDelay == 0 && pinball.isImpact()) {
                pinball.playSoundHitBarrier();
                soundPopDelay += 5;
              }
            }
          }
        }

        world->stepSimulation(1.0f / 30);

        pinball.limitSpeed();
        
        for (unsigned i = 0; i != rigid_bodies.size(); ++i) {
          btRigidBody *rigid_body = rigid_bodies[i];
          btQuaternion btq = rigid_body->getOrientation();
          btVector3 pos = rigid_body->getCenterOfMassPosition();
          quat q(btq[0], btq[1], btq[2], btq[3]);
          mat4t modelToWorld = q;
          modelToWorld[3] = vec4(pos[0], pos[1], pos[2], 1);
          nodes[i]->access_nodeToParent() = modelToWorld;
        }

        // decrement delays
        if (flipDelayL > 0) flipDelayL--;

        if (flipDelayR > 0) flipDelayR--;

        if (pinballResetDelay > 0) pinballResetDelay--;

        if (soundPopDelay > 0) soundPopDelay--;

        if (speedUpdateDelay > 0) speedUpdateDelay--;

        // Key handlers, when pushed will flip the flippers
        if (is_key_down('Z') && flipDelayR == 0) {
          flipperL.flip();
          flipDelayR = flipperCoolDown;
          pinball.playSoundHitFlipper();
        }

        if (is_key_down('M') && flipDelayL == 0) {
          flipperR.flip();
          flipDelayL = flipperCoolDown;
          pinball.playSoundHitFlipper();
        }

        if (is_key_down(' ') && pinballResetDelay == 0) {
          pinball.reset();
          pinballResetDelay = flipperCoolDown;
        }

        // update matrices. assume 30 fps.
        app_scene->update(1.0f / 30);      // draw the scene
        app_scene->render((float)vx / vy);
      }
    };
  }
}
