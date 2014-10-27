////////////////////////////////////////////////////////////////////////////////
//
// (C) Sam Hayhurst 2014
//
// Object3D base class
//

#ifndef OBJECT3D_INCLUDED
#define OBJECT3D_INCLUDED

namespace octet {
  namespace pinball {

    /// Object3D class, a base class from which more advanced shapes are derived
    class Object3D {
    protected:
      mat4t modelToWorld;
      material *mat;
      scene_node *node;
      btScalar mass;
      btMotionState *motionState;
      btRigidBody *rigidbody;
      mesh *colladaMesh;  // only to be used when loading meshes from collada

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
        colladaMesh = nullptr;
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

      /// pure virtual function used make class abstract
      virtual void add_to_scene(dynarray<scene_node*> &sceneNodes, ref<visual_scene> &appScene, btDiscreteDynamicsWorld &btWorld,
                                dynarray<btRigidBody*> &rigidBodies, bool is_visible = true, bool make_child = true) = 0;

      /// returns rigidbody
      btRigidBody* getRigidBody() {
        return rigidbody;
      }

      /// returns the node
      scene_node* getNode() {
        return node;
      }

      /// virtual function to allow setting the mesh
      virtual void setMesh(mesh *mesh) {
        colladaMesh = mesh;
      }

    };
  }
}

#endif