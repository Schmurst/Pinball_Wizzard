////////////////////////////////////////////////////////////////////////////////
//
// (C) Andy Thomason 2012-2014
//
// Modular Framework for OpenGLES2 rendering on multiple platforms.
//
namespace octet {

  /// Object3D class, a base class from which more advanced shapes are derived
  class Object3D {
  protected:
    mat4t modelToWorld;
    material *mat;
    scene_node *node;
    btScalar mass;
    btMotionState *motionState;
    btRigidBody *rigidbody;

  public:
    /// Empty constructor
    Object3D() {
    }

    /// Destructor
    ~Object3D() {
    }

    /// initialise the Object3D
    void init(mat4t_in model2World, material *mat_object, btScalar mass_object) {
      // assign private data
      modelToWorld = model2World;
      mat = mat_object;
      mass = mass_object;
      // Get the scale and rotation elements from the model to world matrix
      btMatrix3x3 scaleRotMatrix(get_btMatrix3x3(modelToWorld));
      // get and store the translation elements from the model to world matrix
      btVector3 transVec(get_btVector3(modelToWorld[3].xyz()));
      // create default motion state, init dynamic elements
      btTransform transform(scaleRotMatrix, transVec);
      motionState = new btDefaultMotionState(transform);
    }

    /// This function is called to place the meshes and rigidbodies in the
    void addToScene(dynarray<scene_node*> &sceneNodes, ref<visual_scene> &appScene, btDiscreteDynamicsWorld &btWorld, dynarray<btRigidBody*> &rigidBodies) {
      btWorld.addRigidBody(rigidbody);
      rigidBodies.push_back(rigidbody);
      sceneNodes.push_back(node);
      appScene->add_child(node);
    }

    /// returns rigidbody
    btRigidBody* getRigidBody() {
      return rigidbody;
    }
  };

  /// Box3D class, simple 3d box class, can be dynamic
  class Box3D : public Object3D {
  protected:
    vec3 size;
    mesh_box* meshBox;

  public:
    /// Box3d Constructor, used to initialise a dynamic box.
    Box3D()
    {}
    /// Box3D destructor
    ~Box3D(){
    }

    /// init function, mass defaults to 1.0 to ensure dynamic behavior within the scene
    void init_box(mat4t model2world, vec3 size, material *material_box, float mass_box = 1.0f) {
      init(model2world, material_box, mass_box);
      btCollisionShape *shape = new btBoxShape(get_btVector3(size));
      btVector3 inertialTensor;
      shape->calculateLocalInertia(mass, inertialTensor);
      rigidbody = new btRigidBody(mass, motionState, shape, inertialTensor);
      // init mesh_box and scene node
      meshBox = new mesh_box(size);
      node = new scene_node(modelToWorld, atom_);
    }

    /// Adds the mesh and rigidbody of the box to the scene
    void add_to_scene(dynarray<scene_node*> &sceneNodes, ref<visual_scene> appScene, btDiscreteDynamicsWorld &btWorld, dynarray<btRigidBody*> &rigidBodies) {
      addToScene(sceneNodes, appScene, btWorld, rigidBodies);
      appScene->add_mesh_instance(new mesh_instance(node, meshBox, mat));
    }
  };

  /// triangular prism class, derived from simple Object3D class
  class TriPrism3D : public Object3D {
  protected:
    float base;
    float height;
    float depth;
    float angle;
    mesh_points* mesh;

  public:
    /// triprism constructor
    TriPrism3D() {
      }

    /// triprism destructor
    ~TriPrism3D() {
    }



  };

  /// Pinball class, a simple 3d sphere, dynamic
  class Pinball : public Object3D {
  private:
    float radii;
    mesh_sphere *meshSphere;

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
    }

    /// Adds the mesh and rigidbody of the sphere to the scene
    void add_to_scene(dynarray<scene_node*> &sceneNodes, ref<visual_scene> appScene, btDiscreteDynamicsWorld &btWorld, dynarray<btRigidBody*> &rigidBodies) {
      addToScene(sceneNodes, appScene, btWorld, rigidBodies);
      appScene->add_mesh_instance(new mesh_instance(node, meshSphere, mat));
    }

    /// Moves Pinball to position within world
    void setPosition(vec3 pos) {
      node->access_nodeToParent().loadIdentity();
    }
  };

  /// Flipper class derived from box to hit phys boxes around the scene
  class Flipper : public Box3D {
  private:
    btVector3 flipTorque;

  public:
    Flipper() {
    }

    ~Flipper() {
    }

    /// This is called to initialise the flipper.
    void init_flipper(mat4t model2world, vec3 box_size, material *box_material, vec3 torque, float mass) {
      flipTorque = get_btVector3(torque);
      init_box(model2world, box_size, box_material, mass);
      rigidbody->setActivationState(DISABLE_DEACTIVATION);    // disables the deactiveation state
    }

    /// This is called by the player to Rotate the flipper.
    void flip(){
      rigidbody->applyTorqueImpulse(flipTorque);
      // printf("Flipper function has been activated");
    }
  };

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
		  app_scene->get_camera_instance(0)->get_node()->translate(vec3(0, 1.0f, -5.5f));
      world->setGravity(btVector3(0, -9.81f, 0));

      scene_node *light_node = new scene_node();
      light *light_fill = new light();
      light_fill->set_color(vec4(0.0f, 0.0f, 1.0f, 1.0f));
      light_fill->set_attenuation(1, 0, -1);
      light_node->rotate(-45, vec3(1, 0, 0));
      light_node->translate(vec3(20, 0, 20));
      app_scene->add_light_instance(new light_instance(light_node, light_fill));

      camera_instance *camera = app_scene->get_camera_instance(0);
		  mat4t modelToWorld; 
      material *table_mat = new material(vec4(0, 1.0f, 0, 1.0f));

      // Table Construction
      Box3D table, tableTop, tableR, tableL;
      float tableWidth = 5.0f;
      float tableDepth = 0.5f; // used
      float tableLength = 10.0f;
      material *table_buffer = new material(vec4(0.1f, 0.8f, 0.1f, 1.0f));

      // Table base
      modelToWorld.loadIdentity();
      modelToWorld.translate(0.0f, 1.0f, 1.0f);
      modelToWorld.rotateX(-30.0f);
      table.init_box(modelToWorld, vec3(tableWidth, tableDepth, tableLength), table_mat, 0.0f);    // mass = 0 -> static x: 1000 y: 100 z: 2000
      table.add_to_scene(nodes, app_scene, (*world), rigid_bodies);

      // Table Left
      modelToWorld.translate(-tableWidth - tableDepth, tableDepth, 0);
      tableL.init_box(modelToWorld, vec3(tableDepth, tableDepth * 2.0f, tableLength * 1.05f), table_buffer, 0.0f);
      tableL.add_to_scene(nodes, app_scene, (*world), rigid_bodies);

      //table Right
      modelToWorld.translate(tableWidth * 2.0f + tableDepth * 2.0f, 0, 0);
      tableR.init_box(modelToWorld, vec3(tableDepth, tableDepth * 2.0f, tableLength * 1.05f), table_buffer, 0.0f);
      tableR.add_to_scene(nodes, app_scene, (*world), rigid_bodies);

      // FLipper
      float torqueImpluse = 300.0f;
      float initialOffset = 10.0f;
      float halfheightFlipper = 0.2f;
      float halfwidthFlipper = 0.2f;
      float halflengthFlipper = 1.2f;
      float massFlipper = 8.0f;
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

      // add left flipper to the scene
      modelToWorld.loadIdentity();
      modelToWorld.translate(-initialOffset, initialOffset, initialOffset);  // to make the flipper fly in from off the screen
      modelToWorld.rotateX(-30.0f); 
      flipperL.init_flipper(modelToWorld, sizeFlipper, flip_mat, vec3(0, 0, 1.0f) * torqueImpluse, massFlipper); // x: 100 y: 20 z: 20
      flipperL.add_to_scene(nodes, app_scene, (*world), rigid_bodies);

      // Add a constraint between flipper and table
      btHingeConstraint *hingeFlipperRight = new btHingeConstraint((*table.getRigidBody()), (*flipperR.getRigidBody()),
                                                              tableOffsetR, hingeOffsetR,         // this are the hinge offset vectors
                                                              btVector3(0, 1.0f, 0), btVector3(0, 0, 1.0f), false);
      btHingeConstraint *hingeFlipperLeft = new btHingeConstraint((*table.getRigidBody()), (*flipperL.getRigidBody()),
                                                              tableOffsetL, hingeOffsetL,       // this are the hinge offset vectors
                                                              btVector3(0, 1.0f, 0), btVector3(0, 0, 1.0f), false);
      // btGeneric6DofConstraint *ConstraintTableL = new btGeneric6DofConstraint()

      // set angle limits on the flippers
      hingeFlipperLeft->setLimit(-PI * 0.2f, PI * 0.2f);
      hingeFlipperRight->setLimit(-PI * 0.2f, PI * 0.2f, 0.0f, 0.8f, 0.5f);

      // add constraints to world
      world->addConstraint(hingeFlipperLeft);
      world->addConstraint(hingeFlipperRight);

      // Add the pinball to the world
      material *sphere_mat = new material(vec4(1.0f, 0, 0.8f, 1.0f));
      modelToWorld.loadIdentity();
      modelToWorld.translate(1.0f, 6.0f, 0.0f);
      pinball.init_sphere(modelToWorld, 0.2f, sphere_mat, 1.0f);
      pinball.add_to_scene(nodes, app_scene, *world, rigid_bodies);
	}

    /// this is called to draw the world
    void draw_world(int x, int y, int w, int h) {
      int vx, vy;
      vx = vy = 0;
      get_viewport_size(vx, vy);
      app_scene->begin_render(vx, vy);

      world->stepSimulation(1.0f/60);
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
      app_scene->update(1.0f/30);      // draw the scene
      app_scene->render((float)vx / vy);
    }
  };
}
