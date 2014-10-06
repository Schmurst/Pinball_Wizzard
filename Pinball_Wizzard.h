////////////////////////////////////////////////////////////////////////////////
//
// (C) Andy Thomason 2012-2014
//
// Modular Framework for OpenGLES2 rendering on multiple platforms.
//
namespace octet {
  /*
  /// Box3D Class implementation used to derive other such objects.
  class Box3D {
    mat4t modelToWorld;
    scene_node *node;
    float halfWidth;  // x
    float halfHeight; // y
    float halfDepth;  // z
    vec4  colour;

  public:

    Box3D() {
      node = 0;
    }

    ~Box3D() {
      delete node;
    }

    /// This is called to initialised a box3D in a position with a colour.
    void init(vec3 box_location, vec4 box_colour, vec3 box_size) {
      // First colour the 3D box.
      colour = box_colour;
      // now set the half dimentions
      halfHeight = 0.5f * box_size.y();
      halfWidth = 0.5f * box_size.x();
      halfDepth = 0.5f * box_size.z();
      // init the matrix to convert local (model) space to world space
      modelToWorld.loadIdentity();
      modelToWorld.translate(box_location.x(), box_location.y(), box_location.z());
      // create a node to hold out box
      if (node == 0)  node = new scene_node(modelToWorld, atom_);
      modelToWorld.loadIdentity();
    }

    /// This function is used after init to load the Box3D inti the current scene.
    void LoadToScene(ref<visual_scene> scene) {
      // first the materials
      material *box_material = new material(colour);
      // now the box
      mesh_box *box_mesh = new mesh_box();
      mat4t boxLocation;
      boxLocation.loadIdentity();
      boxLocation.translate(halfWidth, halfHeight, halfDepth);
      scene->add_mesh_instance(new mesh_instance(node, box_mesh, box_material));
    }

  };
  */

  /// Box3D class, simple 3d box class, can be dynamic
  class Box3D {
  private:
    mat4t modelToWorld;
    vec3 size;
    vec3 position;
    material *mat;
    bool is_dynamic;

  public:
    Box3D(mat4t model2world, vec3 box_size, material *box_material, bool is_box_dynamic = true) {
      // assign private data
      modelToWorld = model2world;
      size = box_size;
      mat = box_material;
      is_dynamic = is_box_dynamic;
      // Get the scale and rotation elements from the model to world matrix
      btMatrix3x3 matrix(get_btMatrix3x3(modelToWorld));
      // get and store the translation elements from the model to world matrix
      btVector3 trans_vec(get_btVector3(modelToWorld[3].xyz()));

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

	/// This is called to move the Flippers.
	void move_flipper() {
	  printf("Flipper function has been activated");
	}

    /// this is called once OpenGL is initialized
	void app_init() {
		app_scene = new visual_scene();
		app_scene->create_default_camera_and_lights();
		app_scene->get_camera_instance(0)->get_node()->translate(vec3(0, 5, 0));

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
      add_box(modelToWorld, vec3(0.5f), mat);
    }
    /*
    // add a box3D object
    Box3D boxee;
    boxee.init(vec3(0, 0, 0), vec4(1, 0, 0, 1), vec3(1, 1, 1));
    // boxee.LoadToScene(app_scene);		
    */

		// add the flippers
		modelToWorld.translate(5.0f, -1.0f, 0);
		material *mat_hitter = new material(vec4(1, 0, 0, 1));
		add_box(modelToWorld, vec3(2.5f, 0.5f, 0.5f), mat_hitter, false);
		modelToWorld.translate(-10.0f, 0, 0);
		add_box(modelToWorld, vec3(2.5f, 0.5f, 0.5f), mat_hitter, false);
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

	    // detect whether flipper should be moved
	    if (is_key_down('A'))  {
		    move_flipper();
	    }
	    else if (is_key_down('D')) {
		    move_flipper();
	    }

      // update matrices. assume 30 fps.
      app_scene->update(1.0f/30);

      // draw the scene
      app_scene->render((float)vx / vy);
    }
  };
}
