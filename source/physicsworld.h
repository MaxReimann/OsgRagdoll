#pragma once
// STD
#include <vector>
#include <set>
#include <iterator>
#include <algorithm>
#include <array>
#include <mutex>
#include <memory> //shared_ptr
//bullet
#include <btBulletDynamicsCommon.h>
#include "LinearMath/btHashMap.h"
#include "gldebugdrawer.h"

namespace ragdoll
{


/*! The PhysicsWorld provides a level of abstraction over the btDynamicsWorld of Bullet. Primarily, it is responsible for stepping the world, adding/removing collision bodies and detecting collisions between them.*/
	class PhysicsWorld
	{
	public:
		PhysicsWorld(bool useDebugView = false);
		virtual ~PhysicsWorld();

		void initializeWorld();
		void stepSimulation(const long double currentTime);

		void addRigidBodies(const std::vector<std::shared_ptr<btRigidBody>>& bodies, const short group = 0, const short mask = 0);
		void addRigidBody(btRigidBody *body, const short group = 0, const short mask = 0);
		void removeRigidBodies(const std::vector<std::shared_ptr<btRigidBody>>& bodies);
		void removeRigidBody(btRigidBody* body);

		void addCollisionObject(btCollisionObject* obj);
		void removeCollisionObject(btCollisionObject* obj);

		// debugview
		GLDebugDrawer* m_debug;

		
		btDiscreteDynamicsWorld* getDiscreteWorld()
		{
			return m_world;
		};


	private:
		btDiscreteDynamicsWorld*			m_world;
		btSequentialImpulseConstraintSolver*m_solver;
		btDefaultCollisionConfiguration*	m_collisionConfiguration;
		btCollisionDispatcher*				m_dispatcher;
		btBroadphaseInterface*				m_broadphase;

		// steping variables
		long double m_lastSimulationTime;

		bool m_useDebugView;
	};
}