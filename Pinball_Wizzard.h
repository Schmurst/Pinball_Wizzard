////////////////////////////////////////////////////////////////////////////////
//
// (C) Andy Thomason 2012-2014
//
// Modular Framework for OpenGLES2 rendering on multiple platforms.
//
namespace octet {

  /// Box3D class, simple 3d box class, can be dynamic
  class Box3D {
  protected:
    mat4t modelToWorld;
    vec3 size;
    material *mat;
    bool is_dynamic;
    btRigidBody *rigidbody;
    scene_node *node;
    mesh_box *meshBox;

  public:
    /// Box3d Constructor, used to initialise a dynamic box.
    Box3D()
    {}
    /// Box3D destructor
    ~Box3D(){
    }

    /// init function
    void init(mat4t model2world, vec3 box_size, material *box_material, float box_mass = 0.0f) {
      // assign private data
      modelToWorld = model2world;
      size = box_size;
      mat = box_material;
      // Get the scale and rotation elements from the model to world matrix
      btMatrix3x3 scaleRotMatrix(get_btMatrix3x3(modelToWorld));
      // get and store the translation elements from the model to world matrix
      btVector3 transVec(get_btVector3(modelToWorld[3].xyz()));
      // create a collision shape out of the size input
      btCollisionShape *shape = new btBoxShape(get_btVector3(size)); 
      // create default motion state, init dynamic elements
      btTransform transform(scaleRotMatrix, transVec);
      btDefaultMotionState *motionState = new btDefaultMotionState(transform);
      btScalar mass = box_mass;
      btVector3 inertialTensor;
      shape->calculateLocalInertia(mass, inertialTensor);
      // construct rigidbody
      rigidbody = new btRigidBody(mass, motionState, shape, inertialTensor);
      // init mesh_box and scene node
      meshBox = new mesh_box(size);
      node = new scene_node(modelToWorld, atom_);
    }

    /// called to add a Box3D to a scene.
    void addToScene(dynarray<scene_node*> &sceneNodes, ref<visual_scene> appScene, btDiscreteDynamicsWorld &btWorld, dynarray<btRigidBody*> &rigidBodies) {
      btWorld.addRigidBody(rigidbody);
      rigidBodies.push_back(rigidbody);
      sceneNodes.push_back(node);
      appScene->add_child(node);
      appScene->add_mesh_instance(new mesh_instance(node, meshBox, mat));
    }

    /// returns rigidbody
    btRigidBody* getRigidBody() {
      return rigidbody;
    }

    /// This is called to set a hinge constraints on the rigid body.
    void SetConstraintHinge(float rotX, float rotY, float rotZ) {
      rigidbody->setAngularFactor(btVector3(rotX, rotY, rotZ));
    }

    /// This is called to set spacial constraints on a box
    void SetSpaceConstraint(float transX, float transY, float transZ) {
      rigidbody->setLinearFactor(btVector3(transX, transY, transZ));
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
      init(model2world, box_size, box_material, mass);
    }

    /// This is called by the player to Rotate the flipper.
    void flip(){
      rigidbody->applyTorque(flipTorque);
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

    // camera_instance *camera = app_scene->get_camera_instance(0);

    // flipper instatiate is included here such that it is common to all scopes/ functions below
    Flipper flipperR, flipperL;

    void add_box(mat4t_in modelToWorld, vec3_in size, material *mat, bool is_dynamic=true) {

      btMatrix3x3 matrix(get_btMatrix3x3(modelToWorld));    // creates a new 3x3 matrix from model to world
      btVector3 pos(get_btVector3(modelToWorld[3].xyz()));  // creates position vector from model to world

      btCollisionShape *shape = new btBoxShape(get_btVector3(size));  // btcollsionshape out of a bt box

      btTransform transform(matrix, pos); // creates a transform matrix

      btDefaultMotionState *motionState = new btDefaultMotionState(transform);  // to be understood
      btScalar mass = is_dynamic ? 1.0f : 0.0f;
      btVector3 inertiaTensor;
   
      shape->calculateLocalInertia(mass, inertiaTensor);
    
      btRigidBody * rigid_body = new btRigidBody(mass, motionState, shape, inertiaTensor);
      world->addRigidBody(rigid_body);    
      rigid_bodies.push_back(rigid_body); 

      mesh_box *box = new mesh_box(size);
      scene_node *node = new scene_node(modelToWorld, atom_);
      nodes.push_back(node);

      app_scene->add_child(node);
      app_scene->add_mesh_instance(new mesh_instance(node, box, mat));
    }

    void add_sphere(mat4t_in modelToWorld, float radii, material *mat, bool is_dynamic = true) {

      btMatrix3x3 matrix(get_btMatrix3x3(modelToWorld));    // creates a new 3x3 matrix from model to world
      btVector3 pos(get_btVector3(modelToWorld[3].xyz()));  // creates position vector from model to world

      btCollisionShape *shape = new btSphereShape(btScalar(radii));

      btTransform transform(matrix, pos); // creates a transform matrix

      btDefaultMotionState *motionState = new btDefaultMotionState(transform);  // to be understood
      btScalar mass = is_dynamic ? 1.0f : 0.0f;
      btVector3 inertiaTensor;

      shape->calculateLocalInertia(mass, inertiaTensor);

      btRigidBody * rigid_body = new btRigidBody(mass, motionState, shape, inertiaTensor);
      world->addRigidBody(rigid_body);
      rigid_bodies.push_back(rigid_body);

      mesh_sphere *ball = new mesh_sphere(vec3(0, 0, 0), radii);
      scene_node *node = new scene_node(modelToWorld, atom_);
      nodes.push_back(node);

      app_scene->add_child(node);
      app_scene->add_mesh_instance(new mesh_instance(node, ball, mat));
    }

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
		  app_scene->get_camera_instance(0)->get_node()->translate(vec3(0, 8.0f, 0));
      world->setGravity(btVector3(0, -9.81f, 0));

      camera_instance *camera = app_scene->get_camera_instance(0);
      //camera->get_node()->rotate(45.0f, vec3(0, 1.0f, 0));
      //camera->get_node()->rotate(-15.0f, vec3(1.0f, 0, 0));
      //camera->get_node()->translate(vec3(24.0f, 0, 0));

		  mat4t modelToWorld;
		  material *floor_mat = new material(vec4(0, 1, 1, 1));

		  // add the ground (as a static object)
		  add_box(modelToWorld, vec3(200.0f, 0.5f, 200.0f), floor_mat, false);

		  // add the boxes (as dynamic objects)
		  modelToWorld.translate(-4.5f, 10.0f, 0);
		  material *mat = new material(vec4(0, 1, 1, 1));
      for (int i = 0; i != 20; ++i) {
        modelToWorld.translate(3, 0, 0);
        modelToWorld.rotateZ(360 / 20);
        add_sphere(modelToWorld, 0.5f, mat);
      }

      // add table to the scene
      material *box_mat = new material(vec4(1.0f, 0, 0, 1.0f));
      material *table_mat = new material(vec4(0, 1.0f, 0, 1.0f));
      Box3D table;                                                     // table is the base of the pinball table
      modelToWorld.loadIdentity();
      modelToWorld.translate(0.0f, 4.0f, 4.0f);
      modelToWorld.rotateX(-30.0f);
      table.init(modelToWorld, vec3(8.0f, 0.5f, 12.0f), table_mat);                                             // x:8m y:0.5m z:12m
      table.addToScene(nodes, app_scene, (*world), rigid_bodies);

      // add right flipper to the scene
      modelToWorld.loadIdentity();
      modelToWorld.translate(17.0f, 17.0f, 17.0f);  // to make the flipper fly in from off the screen
      modelToWorld.rotateX(-30.0f);
      flipperR.init_flipper(modelToWorld, vec3(2.5f, 0.25f, 0.5f), box_mat, vec3(0, 0, -1.0f) * 3000.0f, 5.0f);    // x: 1.5m y: 0.25f, z: 0.5m
      flipperR.addToScene(nodes, app_scene, (*world), rigid_bodies);

      // add left flipper to the scene
      modelToWorld.loadIdentity();
      modelToWorld.translate(-17.0f, 17.0f, 17.0f);  // to make the flipper fly in from off the screen
      modelToWorld.rotateX(-30.0f);
      flipperL.init_flipper(modelToWorld, vec3(2.5f, 0.25f, 0.5f), box_mat, vec3(0, 0, 1.0f) * 3000.0f, 5.0f);    // x: 1.5m y: 0.25f, z: 0.5m
      flipperL.addToScene(nodes, app_scene, (*world), rigid_bodies);

      // Add a constraint between flipper and table
      btHingeConstraint *hingeFlipperLeft = new btHingeConstraint((*table.getRigidBody()), (*flipperR.getRigidBody()),
                                                              btVector3(5.0f, 1.2f, 4.0f), btVector3(1.3f, -0.125f, 0), // this are the hinge offset vectors
                                                              btVector3(0, 1.0f, 0), btVector3(0, 0, 1.0f), false);
      btHingeConstraint *hingeFlipperRight = new btHingeConstraint((*table.getRigidBody()), (*flipperL.getRigidBody()),
                                                              btVector3(-5.0f, 1.2f, 4.0f), btVector3(-1.3f, -0.125f, 0), // this are the hinge offset vectors
                                                              btVector3(0, 1.0f, 0), btVector3(0, 0, 1.0f), false);
      // hingeFlipperLeft->setMotorTarget(0.0f, 0.333333f);

      // add constraints to world
      world->addConstraint(hingeFlipperLeft);
      world->addConstraint(hingeFlipperRight);
	}

    /// this is called to draw the world
    void draw_world(int x, int y, int w, int h) {
      int vx = 0, vy = 0;
      get_viewport_size(vx, vy);
      app_scene->begin_render(vx, vy);

      world->stepSimulation(1.0f/30);
      for (unsigned i = 0; i != rigid_bodies.size(); ++i) {
        btRigidBody *rigid_body = rigid_bodies[i];
        btQuaternion btq = rigid_body->getOrientation();
        btVector3 pos = rigid_body->getCenterOfMassPosition();
        quat q(btq[0], btq[1], btq[2], btq[3]);
        mat4t modelToWorld = q;
        modelToWorld[3] = vec4(pos[0], pos[1], pos[2], 1);
        nodes[i]->access_nodeToParent() = modelToWorld;
      }

      // Key handlers, when pushed will flip the flippers
      if (is_key_down('z') || is_key_down('Z')) {
        flipperL.flip();
      } 

      if (is_key_down('m') || is_key_down('M')) {
        flipperR.flip();
      }
     
      // update matrices. assume 30 fps.
      app_scene->update(1.0f/30);      // draw the scene
      app_scene->render((float)vx / vy);
    }
  };
}
