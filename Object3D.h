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

      /// returns the node
      scene_node* getNode() {
        return node;
      }
    };


  }
}

#endif