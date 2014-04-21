#include "PhysicsWorld.h"
// STD
#include <array>
// bullet
#include <btBulletDynamicsCommon.h>
#include "LinearMath/btHashMap.h"

#include "ragdollmodel.h"

#include <ctime>

using namespace ragdoll;

const btVector3 DEFAULT_GRAVITY(0, 0, -180);


PhysicsWorld::PhysicsWorld(bool useDebugView) :
m_lastSimulationTime(0),
m_useDebugView(useDebugView)
{
	initializeWorld();

	if (m_useDebugView)
	{
		m_debug = new GLDebugDrawer();
		m_debug->setDebugMode(btIDebugDraw::DBG_MAX_DEBUG_DRAW_MODE);
		m_world->setDebugDrawer(m_debug);
	}
}

PhysicsWorld::~PhysicsWorld()
{
	delete m_solver;
	delete m_collisionConfiguration;
	delete m_dispatcher;
	delete m_broadphase;

}

void PhysicsWorld::initializeWorld()
{
	// can be used to used to filter manually potential collision partners
	m_broadphase = new btDbvtBroadphase();

	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);

	// potential bottleneck
	m_solver = new btSequentialImpulseConstraintSolver;

	m_world = new btDiscreteDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration);

	m_world->setGravity(DEFAULT_GRAVITY);
	
	//using callbacks is the preffered way to handle collision events, 
	//the bullet wiki promises, that are no invalid pointers when using this callback
	//m_world->setInternalTickCallback(tickCallback, static_cast<void *>(this));
}

void PhysicsWorld::addRigidBodies(const std::vector<std::shared_ptr<btRigidBody>>& bodies, const short group/*=0*/, const short mask/*=0*/)
{
	for (auto body : bodies)
	{
		m_world->addRigidBody(body.get(), group, mask);
	}
}

void PhysicsWorld::addRigidBody(btRigidBody *body, const short group /*=0*/, const short mask /*=0*/)
{
	m_world->addRigidBody(body, group, mask);
}

void PhysicsWorld::removeRigidBodies(const std::vector<std::shared_ptr<btRigidBody>>& bodies)
{
	for (auto body : bodies) {
		m_world->removeRigidBody(body.get());
	}
}


void PhysicsWorld::removeRigidBody(btRigidBody* body)
{
	m_world->removeRigidBody(body);
}

void PhysicsWorld::removeCollisionObject(btCollisionObject* obj)
{
	m_world->removeCollisionObject(obj);
}

void PhysicsWorld::addCollisionObject(btCollisionObject* obj)
{
	m_world->addCollisionObject(obj);
}

void PhysicsWorld::stepSimulation(long double currentTime)
{
	float timeSinceLastSimulation = currentTime - m_lastSimulationTime;
	m_lastSimulationTime = currentTime;

	// careful:
	// including this debug-printout will significantly reduce
	// framerate & flow of the game

	if (m_useDebugView)
	{
		m_debug->BeginDraw();
	}

	// mind the following constraint:
	// timeStep < maxSubSteps * fixedTimeStep
	// where the parameters are given as follows:
	// stepSimulation(timeStep, maxSubSteps, fixedTimeStep)
	m_world->stepSimulation(timeSinceLastSimulation/1000.f, 8);

	if (m_useDebugView)
	{
		m_world->debugDrawWorld();
		m_debug->EndDraw();
	}

}
