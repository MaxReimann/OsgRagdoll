#include <osg/MatrixTransform>
#include <osg/PositionAttitudeTransform>
#include <osg/Geometry>
#include <osg/Geode>

#include <osgUtil/Optimizer>

#include <osgGA/TrackballManipulator>

#include <osgViewer/Viewer>
//bullet
#include "btBulletDynamicsCommon.h"

#include "ragdollmodel.h"
#include "physicsworld.h"
#include <chrono>
#include "osg/ShapeDrawable"
#include "conversionutils.h"

typedef std::chrono::high_resolution_clock highres_clock;
typedef highres_clock::time_point time_point;
typedef std::chrono::duration<long double, std::milli> milli;

#define USE_DEBUGVIEW 1


using namespace ragdoll;


void addFloor(btVector3 position, btVector3 extent, PhysicsWorld *world, osg::Node *parentNode)
{
	btQuaternion rot90 = btQuaternion();
	rot90.setRotation(btVector3(0.0, 0.0, 1.0), Conversion::degreeToRadians(90.0));
	btBoxShape *wallShape = new btBoxShape(extent/2.0);
	btTransform trans(rot90, position);

	btCollisionObject* fixedGround = new btCollisionObject();
	fixedGround->setCollisionShape(wallShape);
	fixedGround->setWorldTransform(trans);
	world->addCollisionObject(fixedGround);



	osg::ref_ptr<osg::ShapeDrawable> boxDrawable
		= new osg::ShapeDrawable(new osg::Box(osg::Vec3(0, 0, 0), extent.x(), extent.y(), extent.z()));


	osg::ref_ptr<osg::Geode> boxGeode = new osg::Geode();
	boxGeode->addDrawable(boxDrawable);

	// place objects in world space
	osg::Matrixd initialTransform;
	initialTransform.makeTranslate(Conversion::btToOSGVec3(position));

	osg::ref_ptr<osg::MatrixTransform> matrixTransform = new osg::MatrixTransform(initialTransform);
	matrixTransform->addChild(boxGeode);
	static_cast<osg::Group*>(parentNode)->addChild(matrixTransform);
}


int main(int argc, char* argv[])
{
	btVector3 startPosition(0.0, 0.0, 5.0);

	PhysicsWorld *physics = new PhysicsWorld(USE_DEBUGVIEW);
	RagdollView *ragdollView = new RagdollView();
	RagdollModel *ragdollModel = new RagdollModel(physics->getDiscreteWorld(), ragdollView, startPosition);
	

	// tilt the scene so the default eye position is looking down on the model.
	osg::MatrixTransform* sceneNode = new osg::MatrixTransform;
	//sceneNode->setMatrix(osg::Matrix::rotate(osg::inDegrees(30.0f), 1.0f, 0.0f, 0.0f));
	sceneNode->addChild(ragdollView->getRootNode());
	addFloor(btVector3(0.0, 0.0, 0.0), btVector3(500.0, 500.0, 1.0), physics, sceneNode);
	if (USE_DEBUGVIEW)
		sceneNode->addChild(physics->m_debug->getSceneGraph());


	osgViewer::Viewer viewer;
	viewer.setSceneData(sceneNode);
	osgGA::TrackballManipulator *manip = new osgGA::TrackballManipulator();
	manip->setHomePosition(
		osg::Vec3(40.0,0.0,10.0), // homeEye
		osg::Vec3f(), // homeCenter
		osg::Z_AXIS, // up
		false
		);
	viewer.setCameraManipulator(manip);

	viewer.setUpViewInWindow(100, 100, 800, 600);

	auto m_t0 = highres_clock::now();

	
	while (!viewer.done())
	{
		//using c++11 highres chrono timer
		auto m_t1 = highres_clock::now();
		const auto delta = m_t1 - m_t0;
		auto elapsed = milli(delta).count();

		physics->stepSimulation(elapsed);
		viewer.frame();
	}

	return 0;
}

