#include "modelbone.h"
#include "ragdollview.h"
#include "gldebugdrawer.h"


using namespace ragdoll;



char *ragdoll::bodyPartNames[BODYPART_COUNT] = { "BODYPART_PELVIS", "BODYPART_SPINE", "BODYPART_HEAD", "BODYPART_LEFT_UPPER_LEG",
"BODYPART_LEFT_LOWER_LEG", "BODYPART_RIGHT_UPPER_LEG", "BODYPART_RIGHT_LOWER_LEG",
"BODYPART_LEFT_UPPER_ARM", "BODYPART_LEFT_LOWER_ARM", "BODYPART_RIGHT_UPPER_ARM", "BODYPART_RIGHT_LOWER_ARM" };


ModelBone::ModelBone(BODYPART bodyType, btTransform origin, btScalar radius, btScalar height, float mass, RagdollView *ragdollView, btDynamicsWorld* ownerWorld, ModelBone *parent) :
m_bodyType(bodyType), m_worldTransform(origin), m_radius(radius), m_height(height),
m_ragdollView(ragdollView), m_ownerWorld(ownerWorld), m_parent(parent)
{
	btScalar ssm = SHAPE_SIZE_MULTIPLIER;
	m_worldTransform.setOrigin(m_worldTransform.getOrigin() * ssm);
	m_shape = new btCapsuleShape(radius * ssm, height * ssm);

	btScalar _mass = mass; //0.0 for static
	bool isDynamic = (_mass != 0.f);

	btVector3 localInertia(0, 0, 0);
	if (isDynamic)
		m_shape->calculateLocalInertia(_mass, localInertia);


	BoneMotionState* myMotionState = new BoneMotionState(this);
	m_motionState = myMotionState;

	if (parent == nullptr)
	{
		m_viewBone = m_ragdollView->createBone(bodyPartNames[bodyType], Conversion::asOsgMatrix(m_worldTransform), ragdollView->getSkelRoot(), myMotionState);
	}
	else
	{

		m_viewBone = m_ragdollView->createBone(bodyPartNames[bodyType], Conversion::asOsgMatrix(localBoneTransform(m_worldTransform)), parent->m_viewBone, myMotionState);
	}

	// draw bone name in debugview
	static_cast<GLDebugDrawer*>(m_ownerWorld->getDebugDrawer())->setTextSize(200.0);
	static_cast<GLDebugDrawer*>(m_ownerWorld->getDebugDrawer())->draw3dText(origin.getOrigin(), bodyPartNames[m_bodyType]);




	btRigidBody::btRigidBodyConstructionInfo rbInfo(_mass, myMotionState, m_shape, localInertia);
	btRigidBody* body = new btRigidBody(rbInfo);
	m_rigidBody = body;


	m_ownerWorld->addRigidBody(body);
}

btTransform ModelBone::localBoneTransform(btTransform worldTransform)
{
	if (m_parent != nullptr)
	{
		btVector3 parentToChild = worldTransform.getOrigin() - m_parent->m_worldTransform.getOrigin();
		btQuaternion parentToChildQuat = m_parent->m_worldTransform.getRotation() * worldTransform.getRotation().inverse();


		return btTransform(parentToChildQuat, parentToChild);
	}
	else
		return worldTransform;
}
