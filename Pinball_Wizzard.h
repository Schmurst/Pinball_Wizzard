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

        bool collada_debug = true;
        app_scene = new visual_scene();
        app_scene->create_default_camera_and_lights();
        app_scene->get_camera_instance(0)->get_node()->rotate(-20.0f, vec3(1.0, 0, 0));
        app_scene->get_camera_instance(0)->get_node()->translate(vec3(0, 10.0f, -18.5f));
        world->setGravity(btVector3(0, -9.81f, 0));
        mat4t modelToWorld;

        // Add a a fill light to the scene and create camera instance
        scene_node *light_node = new scene_node();
        light *light_fill = new light();
        light_fill->set_color(vec4(0.0f, 0.0f, 1.0f, 1.0f));
        light_fill->set_attenuation(1, 0, -1);
        light_node->rotate(-45, vec3(1, 0, 0));
        light_node->translate(vec3(20, 0, 20));
        app_scene->add_light_instance(new light_instance(light_node, light_fill));

        ////////////////////////////////////////////////// Collada import ///////////////////////////////////////////
        // create dictionary & collada builder
        resource_dict dict;
        dynarray<resource*> collada_meshes;
        collada_builder colladaBuilder;
        if (!colladaBuilder.load_xml("assets/PinballWizzardAssets.dae")) {
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
        table_parts.push_back("BarrierBase");

        // temporary material for table
        material *temp_mat = new material(vec4(0.2f, 0.5f, 0.8f, 1.0f));

        // put the meshes and nodes in the scene... hopefully
        scene_node *node_part;
        mesh *mesh_part;
        dynarray <Box3D*> table_boxes;

        // Ensure always the the table node is at origin in blender
        // make parent scene node (the table node from collada)
        // rotate the node
        // move it and (SCALE IN BLANDER FOO)
        // instead of adding the barrier nodes to the directly to the scene
        // instead add them as childs of the parent node (table node)

        // tabe parent node
        modelToWorld.loadIdentity();
        scene_node *table_parent = new scene_node();
        table_parent->access_nodeToParent() = modelToWorld;
        table_parent->rotate(90.0f, vec3(1.0f, 0, 0));
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
          table_boxes.push_back(new Box3D(node_part, size, temp_mat, 0.0f));
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
          table_boxes[i]->add_to_scene(nodes, app_scene, (*world), rigid_bodies, true, false);
        }

        ////////////////////////////////////////////////// table Rigidbody construction ///////////////////////////////////////////
        bool is_visible = false; // 1.0: visible for debug, 0.0f: invisible
        Box3D table, BarrierTop, BarrierL, BarrierR;
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

        ////////////////////////////////////////////////// FLipper ///////////////////////////////////////////
        float torqueImpluse = 300.0f;
        float initialOffset = 10.0f;
        float halfheightFlipper = 0.4f;
        float halfwidthFlipper = 0.1f;
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
        modelToWorld.translate(0.7f, 6.0f, -2.0f);
        pinball.init_sphere(modelToWorld, 0.4f, sphere_mat, 1.0f);
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
