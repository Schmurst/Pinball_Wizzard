////////////////////////////////////////////////////////////////////////////////
//
// (C) Andy Thomason 2012-2014
//
// Modular Framework for OpenGLES2 rendering on multiple platforms.
//

/*
game logic:
-game comes on
-presented with controls
--user elects to start game (xbox)
---user reaches a certain score
----xpad controls are scrambled
---user drops ball
----user has balls left
-----user is allowed to reset pinball (xbox)
----user doesnt have any balls left
-----user is presented with score and logic goes to top
*/

#include "Object3D.h"
#include "Box3D.h"
#include "Pinball.h"
#include "Flipper.h"
#include "Cylinder3D.h"
#include "XboxController.h"
#include "Lamp.h"
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

      // Overlay and text drawing
      ref<text_overlay> overlay;
      ref<mesh_text> righttext;
      ref<mesh_text> lefttext;

      // Score and multiplier
      float multiplier = 1.0f;
      float score = 0.0f;
      float score_scroll = 20.0f;
      float score_lamp = 50.0f;
      float score_launch = 100.0f;
      int frame_count = 0; // to avoid first frame count addition

      // Gamepad state setup
      // button ids, order matters here
      XboxController pad;
      const static enum BUTTONIDS { A = 0, B, X, Y, DPad_Up, DPad_Down,
        DPad_Left, DPad_Right, L_Shoulder, R_Shoulder, L_Thumbstick,
        R_Thumbstick, Start, Back
      };

      // default controls to start the game
      int controlFlipperL = L_Shoulder;
      int controlFlipperR = R_Shoulder;
      int controlReset = Start;

      // debugs
      bool collada_debug = true;
      bool runtime_debug = false;

      // Game logic controls
      const int BALLS = 3;
      int currentBalls = BALLS;
      const enum GAMESTATE {INTRO = 0, PLAY, DROP, END};
      int currentGameState = INTRO;

      // create an enum used to specify certain object types for collision logic
      enum obj_types {
        PINBALL = 0, FLIPPER, TABLE, LAUNCHER, SCROLL,
        LAMP01, LAMP02, LAMP03, LAMP04, LAMP05, LAMP06, LAMP07, LAMPL, LAMPR
      };

      // flipper & Pinball declaration is included here as they're common to all scopes/ functions below
      Flipper flipperR, flipperL;
      Pinball pinball;
      int flipDelayL = 0, flipDelayR = 0;
      int soundPopDelay = 0, speedUpdateDelay = 0, soundDingDelay = 0, soundBounceDelay = 0;
      int flipperCoolDown = 15; // frames between flips
      dynarray<Lamp*> lamp_pointers;

      // skybox
      scene_node *skybox_node;

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

      /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// app_init
      ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /// this is called once OpenGL is initialized
      void app_init() {

        // initialise the scene
        app_scene = new visual_scene();

        // Add a a fill light to the scene
        scene_node *light_node = new scene_node();
        light *light_fill = new light();
        light_fill->set_color(vec4(0.0f, 0.0f, 1.0f, 1.0f));
        light_fill->set_attenuation(1, -6, -8);
        light_node->rotate(-45, vec3(1, 0, 0));
        light_node->translate(vec3(20, 0, 20));
        app_scene->add_light_instance(new light_instance(light_node, light_fill));

        // set up deafult camera
        app_scene->create_default_camera_and_lights();
        app_scene->get_camera_instance(0)->get_node()->rotate(-22.0f, vec3(1.0, 0, 0));
        app_scene->get_camera_instance(0)->get_node()->translate(vec3(0, 10.0f, -9.5f));
        app_scene->get_camera_instance(0)->set_perspective(0, 70, 1, 0.1f, 1000.0f);
        world->setGravity(btVector3(0, -30.0f, 0));
        mat4t modelToWorld;

        // Purloined from example_text.h
        // create the overlay
        overlay = new text_overlay();

        // get the font
        bitmap_font *font = overlay->get_default_font();

        // create a box containing text (in pixels)
        aabb Rtext_aabb(vec3(480.0, 250.0f, 0.0f), vec3(256, 128, 0));
        aabb Ltext_aabb(vec3(-100.0, -200.0f, 0.0f), vec3(512, 512, 0));
        righttext = new mesh_text(font, "sample text", &Rtext_aabb);
        lefttext = new mesh_text(font, "sample text", &Ltext_aabb);

        // add the mesh to the overlay.
        overlay->add_mesh_text(righttext);
        overlay->add_mesh_text(lefttext);

        ////////////////////////////////////////////////// Pinball ///////////////////////////////////////////
        // Add the pinball to the world
        material *sphere_mat = new material(new image("assets/Pinball_Wizzard/Eye.gif"));
        modelToWorld.loadIdentity();
        modelToWorld.translate(1000.0f, 6.0f, -2.0f);
        pinball.init_sphere(modelToWorld, 0.5f, sphere_mat, 1.0f);
        pinball.add_to_scene(nodes, app_scene, *world, rigid_bodies);
        pinball.getRigidBody()->setRestitution(1.0f);
        pinball.getRigidBody()->setDamping(0.05f, 0.05f);
        pinball.getRigidBody()->setUserIndex(PINBALL);

        ////////////////////////////////////////////////// FLipper ///////////////////////////////////////////
        float torqueImpluse = 500.0f;
        float initialOffset = 10.0f;
        float halfheightFlipper = 0.4f;
        float halfwidthFlipper = 0.2f;
        float halflengthFlipper = 1.4f;
        float massFlipper = 8.0f;
        float flipperRestitution = 1.0f;
        material *flip_mat = new material(vec4(1.0f, 0, 0, 1.0f));

        btVector3 hingeOffsetR = btVector3(halflengthFlipper * 0.95f, 0, halfheightFlipper * -1.2f);
        btVector3 hingeOffsetL = btVector3(halflengthFlipper * -0.95f, 0, halfheightFlipper * -1.2f);
        btVector3 tableOffsetR = btVector3(3.8f, -11.0f, 0.4f);
        btVector3 tableOffsetL = btVector3(-3.8f, -11.0f, 0.4f);
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
        dynarray<resource*> collada_materials;
        collada_builder colladaBuilder;
        if (!colladaBuilder.load_xml("assets/Pinball_Wizzard/PinballWizzardAssets.dae")) {
          printf("failed to load the pinball table file");
          return;
        }

        // get meshes and their respective nodes from dictionary
        colladaBuilder.get_resources(dict);
        dict.find_all(collada_meshes, atom_mesh);
        dict.find_all(collada_materials, atom_material);
        printf("collada_meshes size: %i\n", collada_meshes.size());
        printf("collada_materials size: %i\n", collada_materials.size());

        // part list, taken from collada file, very important to keep uptodate
        dynarray <string> table_parts;
        table_parts.push_back("Table");
        table_parts.push_back("Lamp001");
        table_parts.push_back("Lamp002");
        table_parts.push_back("Lamp003");
        table_parts.push_back("Lamp004");
        table_parts.push_back("Lamp005");
        table_parts.push_back("Lamp006");
        table_parts.push_back("Lamp007");
        table_parts.push_back("LauncherLeft");
        table_parts.push_back("LauncherRight");
        table_parts.push_back("Glass");
        table_parts.push_back("Scroll001");
        table_parts.push_back("Scroll002");
        table_parts.push_back("Scroll003");
        table_parts.push_back("Scroll004");
        table_parts.push_back("Scroll005");
        table_parts.push_back("ScrollLeft");
        table_parts.push_back("ScrollRight");
        table_parts.push_back("ScrollReflectorL");
        table_parts.push_back("ScrollReflectorR");
        table_parts.push_back("SkullL");
        table_parts.push_back("SkullR");
        table_parts.push_back("HatL");
        table_parts.push_back("HatR");


        // new texture shader that handles attenuation
        param_shader *atten_shader = new param_shader("shaders/attenuation_texture.vs", "shaders/attenuation_texture.fs");

        // Materials
        material *lamp_mat = new material(vec4(0.5f, 0.5f, 0.5f, 1.0f));
        material *table_mat = new material(new image("assets/Pinball_Wizzard/BookTexture.gif"), NULL, atten_shader, true);
        material *scroll_mat = new material(new image("assets/Pinball_Wizzard/scrollTexture.gif"), NULL, atten_shader, true);
        material *skull_mat = new material(new image("assets/Pinball_Wizzard/bone.gif"), NULL, atten_shader, true);
        material *hat_mat = new material(new image("assets/Pinball_Wizzard/GreenCloth.gif"), NULL, atten_shader, true);
        material *barrier_mat = new material(vec4(0.8f, 0.5f, 0.2f, 1.0f));
        material *bumper_mat = new material(vec4(0.5f, 0.8f, 0.2f, 1.0f));
        material *wizzard_mat = new material(vec4(0.2f, 0.6f, 0.1f, 1.0f));
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

        int numLamps = 0;

        // now for the table parts
        for (unsigned int i = 0; i < table_parts.size(); i++) {
          string temp;
          // get the node and mesh of each object in table parts list
          node_part = dict.get_scene_node(table_parts[i]);
          table_parent->add_child(node_part);
          temp = table_parts[i];
          temp += "-mesh";
          mesh_part = dict.get_mesh(temp);

          // if the above method finding the string fails attempt adding a # infront
          // this is done by collada when the mesh has a material attached to it

          if (mesh_part == nullptr) {
            if (temp.find("Lamp") != -1) {
              temp += "+Lamp-material";
              mesh_part = dict.get_mesh(temp);
            }
            else if (temp.find("Scroll") != -1) {
              temp += "+paper-material";
              mesh_part = dict.get_mesh(temp);
            }
            else if (temp.find("Hat") != -1) {
              temp += "+Hat-material";
              mesh_part = dict.get_mesh(temp);
            }
            else if (temp.find("Skull") != -1) {
              temp += "+Skull-material";
              mesh_part = dict.get_mesh(temp);
            }
          }

          // create axis_aligned bounding box
          aabb aabb_part = mesh_part->get_aabb();
          // initialise bt box shape, using centre + halfextents (absolute to avoid stange errors)
          vec3 size = (aabb_part.get_center().abs() + aabb_part.get_half_extent().abs());

          // the following check decides whether a collada mesh instance should be converted into a 
          // Box3D object or a Cylinder3D object depending on the collada mesh name

          if (table_parts[i].find("Table") != -1 || table_parts[i].find("Glass") != -1) {
            table_boxes.push_back(new Box3D(node_part, size, table_mat, 0.0f));
            table_boxes[i]->getRigidBody()->setUserIndex(TABLE);
          }
          else if (table_parts[i].find("Launcher") != -1) {
            table_boxes.push_back(new Box3D(node_part, size, error_mat, 0.0f));
            table_boxes[i]->getRigidBody()->setUserIndex(LAUNCHER);
          }
          else if (table_parts[i].find("Lamp") != -1) {
            float radii, height;
            radii = size[0];
            height = size[2];
            Lamp *lamp = new Lamp(node_part, radii, height, lamp_mat, mesh_part, 0.0f);
            lamp->init_lamp(app_scene);
            table_boxes.push_back(lamp);
            table_boxes[i]->getRigidBody()->setUserPointer(lamp);
            table_boxes[i]->getRigidBody()->setUserIndex(LAMP01 + numLamps++);
            table_boxes[i]->setMesh(mesh_part);
            lamp_pointers.push_back(lamp);
          }
          else if (table_parts[i].find("Scroll") != -1) {
            table_boxes.push_back(new Box3D(node_part, size, scroll_mat, 0.0f));
            table_boxes[i]->getRigidBody()->setUserIndex(SCROLL);
            table_boxes[i]->setMesh(mesh_part);
          }
          else if (table_parts[i].find("Skull") != -1 || table_parts[i].find("Hat") != -1) {
            float radii, height;
            radii = size[0];
            height = size[2];
            if (table_parts[i].find("Skull") != -1) {
              table_boxes.push_back(new Cylinder3D(node_part, radii, height, skull_mat, 0.0f));
            }
            else {
              table_boxes.push_back(new Cylinder3D(node_part, radii, height, hat_mat, 0.0f));
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
            printf("User index: %i\n", table_boxes[i]->getRigidBody()->getUserIndex());
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

          if (table_parts[i].find("Glass") != -1 || table_parts[i].find("Launcher") != -1) {
            table_boxes[i]->add_to_scene(nodes, app_scene, (*world), rigid_bodies, false, false);
          }
          else {
            table_boxes[i]->add_to_scene(nodes, app_scene, (*world), rigid_bodies, true, false);
          }
        }

        // this code will loop throught the rigidbodies and set the right restitution for the parts
        for (unsigned int i = 0; i < table_parts.size(); i++) {
          if (table_parts[i].find("Scroll") != -1) {
            table_boxes[i]->getRigidBody()->setRestitution(0.8f);
            if (table_parts[i].find("Left") != -1 || table_parts[i].find("Right") != -1) {
              table_boxes[i]->getRigidBody()->setRestitution(0.2f);
            }
          }
          else if (table_parts[i].find("Lamp") != -1) {
            table_boxes[i]->getRigidBody()->setRestitution(1.5f);
          }
          else if (table_parts[i].find("Launcher") != -1 || table_parts[i].find("Skull") != -1) {
            table_boxes[i]->getRigidBody()->setRestitution(3.0f);
          }
          else if (table_parts[i].find("table") != -1) {
            table_boxes[i]->getRigidBody()->setFriction(0.2f);
          }
        }

        ///////////////////////////////////// Hinge Constraints ///////////////////////////////
        // Load the Hinge Constraints
        // get central table rigid body from rigidbodies dynarray
        btRigidBody *table = rigid_bodies[3];

        // Add a constraint between flipper and table
        btHingeConstraint *hingeFlipperRight = new btHingeConstraint((*table), (*flipperR.getRigidBody()),
          tableOffsetR, hingeOffsetR,         // this are the hinge offset vectors
          btVector3(0, 0, 1.0f), btVector3(0, 0, 1.0f), false);
        btHingeConstraint *hingeFlipperLeft = new btHingeConstraint((*table), (*flipperL.getRigidBody()),
          tableOffsetL, hingeOffsetL,       // this are the hinge offset vectors
          btVector3(0, 0, 1.0f), btVector3(0, 0, 1.0f), false);

        // set angle limits on the flippers
        hingeFlipperLeft->setLimit(-PI * 0.2f, PI * 0.2f);
        hingeFlipperRight->setLimit(-PI * 0.2f, PI * 0.2f, 0.0f, 0.8f, 0.5f);

        // add constraints to world
        world->addConstraint(hingeFlipperLeft);
        world->addConstraint(hingeFlipperRight);

        //////////////////////////////// General setup /////////////////////////

        // add the skybox sphere to the world no rigidbody
        modelToWorld.loadIdentity();
        modelToWorld.rotateX90();
        material *skybox_mat = new material(new image("assets/Pinball_Wizzard/largeGalField.gif"));
        skybox_node = new scene_node(modelToWorld, atom_);
        mesh_sphere *skybox_mesh = new mesh_sphere(vec3(), 40.0f);
        nodes.push_back(skybox_node);
        app_scene->add_mesh_instance(new mesh_instance(skybox_node, skybox_mesh, skybox_mat));

        // check what user indexes are in the scene, duplicate check
        for (unsigned int i = 0; i < rigid_bodies.size(); i++) {
          printf("userpointer: %i\n", rigid_bodies[i]->getUserIndex());
        }

        ///////////////////////////////////// XBOX Pad ///////////////////////////////
        // create an xbox controller object
        printf((pad.isConnected()) ? "Pad is connected\n" : "Pad is NOT connected");
      }

      /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// draw_world
      ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      void draw_world(int x, int y, int w, int h) {
        int vx, vy;
        vx = vy = 0;
        get_viewport_size(vx, vy);
        app_scene->begin_render(vx, vy);

        // used in collision detection to 'delay' the previous speed by 5 frames
        // this is because a collision often to always lasts more than 3 frames,
        // over which a large change in speed is changed incrementally              (is this true?)
        if (speedUpdateDelay == 0) {
          pinball.updateSpeed();
          speedUpdateDelay += 5;
        }

        frame_count++;
        // detect frame count and use this to increment score multiplier
        if (frame_count % 600 == 0){
          multiplier += 0.1f;
        }
        ///////////////////////////////////// Xbox Pad ///////////////////////////////
        // update the pad state
        pad.Update();
        

        ///////////////////////////////////// Game Logic ///////////////////////////////

        if (pinball.isDropped() && currentGameState == PLAY) {
          currentBalls--;
          pinball.playSoundBallDrop();
          // detect here for game loss
          if (currentBalls == 0) {
            currentGameState = END;
            printf("Game Over\n");
          }
          else {
            currentGameState = DROP;
            printf("You have %i Balls remaining\n", currentBalls);
          }
        }

        ///////////////////////////////////// Collisions ///////////////////////////////

        int numManifolds = world->getDispatcher()->getNumManifolds();
        if (runtime_debug) printf("------new physics step--------\n");
        for (int i = 0; i < numManifolds; i++)
        {
          btPersistentManifold* contactManifold = world->getDispatcher()->getManifoldByIndexInternal(i);
          int objA = contactManifold->getBody0()->getUserIndex();
          int objB = contactManifold->getBody1()->getUserIndex();


          // check what has hit what and then play sounds
          if (objA == PINBALL || objB == PINBALL) {
            if (objA == FLIPPER || objB == FLIPPER) {
              if (runtime_debug) printf("The pinball has hit a FLIPPER\n");
            }
            else if ((objA >= LAMP01 && objA <= LAMPR) || (objB >= LAMP01 && objB <= LAMPR)) {
              if (runtime_debug) printf("The pinball has hit a LAMP\n");
              if (runtime_debug) printf("The LAMP index is: %i\n", objB);
              if (soundDingDelay == 0 && pinball.isImpact()) {
                pinball.playSoundHitBumper();
                soundDingDelay += 10;
                if (objA >= LAMP01 && objA <= LAMPR) {
                  if (runtime_debug) printf("objA is a lamp\n");
                }
                else if (objB >= LAMP01 && objB <= LAMPR) {
                  if (runtime_debug) printf("objB is a lamp\n");
                  Lamp *plamp = lamp_pointers[objB - LAMP01];
                  plamp->upgrade();
                  score += plamp->getHitScore();
                }
              }
            }

            else if (objA == LAUNCHER || objB == LAUNCHER) {
              if (runtime_debug) printf("The pinball has hit the Launcher\n");
              if (soundBounceDelay == 0 && pinball.isImpact()) {
                pinball.playSoundHitLauncher();
                soundBounceDelay += 15;
                score += score_launch * multiplier;
              }
            }
            else if (objA == SCROLL || objB == SCROLL) {
              if (runtime_debug) printf("The pinball has hit the BARRIER\n");
              if (soundPopDelay == 0 && pinball.isImpact()) {
                pinball.playSoundHitBarrier();
                soundPopDelay += 15;
                score += score_scroll * multiplier;
              }
            }
          }
        }

        world->stepSimulation(1.0f / 5);
        // limit the speed of the pinball
        pinball.limitSpeed();

        ///////////////////////////////////// update mesh positions to RBs ///////////////////////////////
        for (unsigned i = 0; i != rigid_bodies.size(); ++i) {
          btRigidBody *rigid_body = rigid_bodies[i];
          btQuaternion btq = rigid_body->getOrientation();
          btVector3 pos = rigid_body->getCenterOfMassPosition();
          quat q(btq[0], btq[1], btq[2], btq[3]);
          mat4t modelToWorld = q;
          modelToWorld[3] = vec4(pos[0], pos[1], pos[2], 1);
          nodes[i]->access_nodeToParent() = modelToWorld;
        }

        ///////////////////////////////////// delay handling ///////////////////////////////
        if (flipDelayL > 0) flipDelayL--;

        if (flipDelayR > 0) flipDelayR--;

        if (soundPopDelay > 0) soundPopDelay--;

        if (speedUpdateDelay > 0) speedUpdateDelay--;

        if (soundDingDelay > 0) soundDingDelay--;

        if (soundBounceDelay > 0) soundBounceDelay--;

        ///////////////////////////////////// button handlers ///////////////////////////////
        if (pad.isButtonPressed(controlFlipperR) && flipDelayR == 0) {
          flipperR.flip();
          flipDelayR = flipperCoolDown;
          pinball.playSoundHitFlipper();
        }

        if (pad.isButtonPressed(controlFlipperL) && flipDelayL == 0) {
          flipperL.flip();
          flipDelayL = flipperCoolDown;
          pinball.playSoundHitFlipper();
        }

        if (pad.isButtonPressed(controlReset) && (currentGameState == DROP || currentGameState == INTRO || currentGameState == END)) {
          switch (currentGameState){
          case INTRO:
            pinball.playSoundWelcome();
            currentGameState = PLAY;
            pinball.reset();
            return;
          case END:
            currentGameState = INTRO;
            // pinball play sound
            for (unsigned int i = 0; i < lamp_pointers.size(); ++i){
              lamp_pointers[i]->resetMultipier();
            }
            score = 0.0f;
            multiplier = 0.0f;
            currentBalls = BALLS;
            return;
          case DROP:
            currentGameState = PLAY;
            pinball.reset();
            return;
          default:
            return;
          }
        }

        // update matrices. assume 30 fps.
        app_scene->update(1.0f / 30);      // draw the scene
        app_scene->render((float)vx / vy);

        // rotate the skybox (skysphere)
        skybox_node->rotate(0.25f / 10, vec3(0, 1, 0));

        ///////////////////////////////////// Draw the UI ///////////////////////////////
        // clear and update text
        if (currentGameState == PLAY || currentGameState == DROP || currentGameState == END) {
          righttext->clear();
          righttext->format("SCORE: %8.2f\n" "MULITPLIER: %4.2f\n" "Balls Left: %i\n", score, multiplier, currentBalls);
        }
        else {
          righttext->clear();
        }

        if (currentGameState == END || currentGameState == INTRO) {
          lefttext->clear();
          lefttext->format("CONTROLS\n" "left Flipper: Lbumper\n" "right Flipper: Rbumper\n" "reset: start\n");
        }
        else {
          lefttext->clear();
        }
        // convert it to a mesh.
        righttext->update();
        lefttext->update();

        // draw the text overlay
        overlay->render(vx, vy);
      }
    };
  }
}


