#include "ragdollmodel.h"
#include "ragdollview.h"

#include "gldebugdrawer.h"

using namespace ragdoll;



RagdollModel::RagdollModel(btDynamicsWorld* ownerWorld, RagdollView *ragdollView, const btVector3& positionOffset)
		: m_ownerWorld (ownerWorld)
	{
		float ssm = SHAPE_SIZE_MULTIPLIER;
		// Setup the geometry
		m_ragdollView = ragdollView;

		m_bones[BODYPART_PELVIS] = new ModelBone(BODYPART_PELVIS,
			btTransform(btQuaternion(), btVector3(0.0, 1.0, 0.0) + positionOffset),
			btScalar(0.15), btScalar(0.2), btScalar(1.0),
			m_ragdollView, m_ownerWorld);

		m_bones[BODYPART_SPINE] = new ModelBone(BODYPART_SPINE,
			btTransform(btQuaternion(), btVector3(0.0, 1.2, 0.0) + positionOffset),
			btScalar(0.15), btScalar(0.28), btScalar(1.0),
			m_ragdollView, m_ownerWorld, 
			m_bones[BODYPART_PELVIS]);
		
		m_bones[BODYPART_HEAD] = new ModelBone(BODYPART_HEAD,
			btTransform(btQuaternion(), btVector3(0.0, 1.6, 0.0) + positionOffset),
			btScalar(0.1), btScalar(0.05), btScalar(1.0),
			m_ragdollView, m_ownerWorld,
			m_bones[BODYPART_SPINE]);

		m_bones[BODYPART_LEFT_UPPER_LEG] = new ModelBone(BODYPART_LEFT_UPPER_LEG,
			btTransform(btQuaternion(), btVector3(-0.18, 0.65, 0.0) + positionOffset),
			btScalar(0.07), btScalar(0.45), btScalar(1.0),
			m_ragdollView, m_ownerWorld,
			m_bones[BODYPART_PELVIS]);
		
		m_bones[BODYPART_LEFT_LOWER_LEG] = new ModelBone(BODYPART_LEFT_LOWER_LEG,
			btTransform(btQuaternion(), btVector3(-0.18, 0.2, 0.0) + positionOffset),
			btScalar(0.05), btScalar(0.37), btScalar(1.0),
			m_ragdollView, m_ownerWorld,
			m_bones[BODYPART_LEFT_UPPER_LEG]);

		m_bones[BODYPART_RIGHT_UPPER_LEG] = new ModelBone(BODYPART_RIGHT_UPPER_LEG,
			btTransform(btQuaternion(), btVector3(0.18, 0.65, 0.0) + positionOffset),
			btScalar(0.07), btScalar(0.45), btScalar(1.0),
			m_ragdollView, m_ownerWorld,
			m_bones[BODYPART_PELVIS]);
		
		m_bones[BODYPART_RIGHT_LOWER_LEG] = new ModelBone(BODYPART_RIGHT_LOWER_LEG,
			btTransform(btQuaternion(), btVector3(0.18, 0.2, 0.0) + positionOffset),
			btScalar(0.05), btScalar(0.37), btScalar(1.0),
			m_ragdollView, m_ownerWorld,
			m_bones[BODYPART_RIGHT_UPPER_LEG]);

		m_bones[BODYPART_LEFT_UPPER_ARM] = new ModelBone(BODYPART_LEFT_UPPER_ARM,
			btTransform(btQuaternion(0, 0, PI_2), btVector3(-0.35, 1.45, 0.0) + positionOffset),
			btScalar(0.05), btScalar(0.33), btScalar(1.0),
			m_ragdollView, m_ownerWorld,
			m_bones[BODYPART_SPINE]);

		m_bones[BODYPART_LEFT_LOWER_ARM] = new ModelBone(BODYPART_LEFT_LOWER_ARM,
			btTransform(btQuaternion(0, 0, PI_2), btVector3(-0.7, 1.45, 0.0) + positionOffset),
			btScalar(0.04), btScalar(0.25), btScalar(1.0),
			m_ragdollView, m_ownerWorld,
			m_bones[BODYPART_LEFT_UPPER_ARM]);

		m_bones[BODYPART_RIGHT_UPPER_ARM] = new ModelBone(BODYPART_RIGHT_UPPER_ARM,
			btTransform(btQuaternion(0, 0, -PI_2), btVector3(0.35, 1.45, 0.0) + positionOffset),
			btScalar(0.05), btScalar(0.33), btScalar(1.0),
			m_ragdollView, m_ownerWorld,
			m_bones[BODYPART_SPINE]);

		m_bones[BODYPART_RIGHT_LOWER_ARM] = new ModelBone(BODYPART_RIGHT_LOWER_ARM,
			btTransform(btQuaternion(0, 0, -PI_2), btVector3(0.7, 1.45, 0.0) + positionOffset),
			btScalar(0.04), btScalar(0.25), btScalar(1.0),
			m_ragdollView, m_ownerWorld,
			m_bones[BODYPART_RIGHT_UPPER_ARM]);


		// Setup some damping on the m_bodies
		for (int i = 0; i < BODYPART_COUNT; ++i)
		{
			
			m_bones[i]->m_rigidBody->setDamping(0.05, 0.85);
			m_bones[i]->m_rigidBody->setDeactivationTime(0.8);
			m_bones[i]->m_rigidBody->setSleepingThresholds(1.6, 2.5);
		}

		// Now setup the constraints
		btHingeConstraint* hingeC;
		btConeTwistConstraint* coneC;

		btTransform localA, localB;

		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0, PI_2, 0); localA.setOrigin(btVector3(btScalar(0.), btScalar(0.15), btScalar(0.)) * ssm);
		localB.getBasis().setEulerZYX(0, PI_2, 0); localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.15), btScalar(0.)) * ssm);
		hingeC = new btHingeConstraint(*m_bones[BODYPART_PELVIS]->m_rigidBody, *m_bones[BODYPART_SPINE]->m_rigidBody, localA, localB);
		hingeC->setLimit(btScalar(-PI_4), btScalar(PI_2));
		m_joints[JOINT_PELVIS_SPINE] = hingeC;
		hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

		m_ownerWorld->addConstraint(m_joints[JOINT_PELVIS_SPINE], true);


		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0, 0, PI_2); localA.setOrigin(btVector3(btScalar(0.), btScalar(0.30), btScalar(0.)) * ssm);
		localB.getBasis().setEulerZYX(0, 0, PI_2); localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.14), btScalar(0.)) * ssm);
		coneC = new btConeTwistConstraint(*m_bones[BODYPART_SPINE]->m_rigidBody, *m_bones[BODYPART_HEAD]->m_rigidBody, localA, localB);
		coneC->setLimit(PI_4, PI_4, PI_2);
		m_joints[JOINT_SPINE_HEAD] = coneC;
		coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

		m_ownerWorld->addConstraint(m_joints[JOINT_SPINE_HEAD], true);


		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,0,-PI_4*5); localA.setOrigin(btVector3(btScalar(-0.18), btScalar(-0.10), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,0,-PI_4*5); localB.setOrigin(btVector3(btScalar(0.), btScalar(0.225), btScalar(0.)));
		coneC = new btConeTwistConstraint(*m_bones[BODYPART_PELVIS]->m_rigidBody, *m_bones[BODYPART_LEFT_UPPER_LEG]->m_rigidBody, localA, localB);
		coneC->setLimit(PI_4, PI_4, 0);
		m_joints[JOINT_LEFT_HIP] = coneC;
		coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

		m_ownerWorld->addConstraint(m_joints[JOINT_LEFT_HIP], true);

		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0, PI_2, 0); localA.setOrigin(btVector3(btScalar(0.), btScalar(-0.225), btScalar(0.)) * ssm);
		localB.getBasis().setEulerZYX(0, PI_2, 0); localB.setOrigin(btVector3(btScalar(0.), btScalar(0.185), btScalar(0.)) * ssm);
		hingeC = new btHingeConstraint(*m_bones[BODYPART_LEFT_UPPER_LEG]->m_rigidBody, *m_bones[BODYPART_LEFT_LOWER_LEG]->m_rigidBody, localA, localB);
		hingeC->setLimit(btScalar(0), btScalar(PI_2));
		m_joints[JOINT_LEFT_KNEE] = hingeC;
		hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

		m_ownerWorld->addConstraint(m_joints[JOINT_LEFT_KNEE], true);


		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0, 0, PI_4); localA.setOrigin(btVector3(btScalar(0.18), btScalar(-0.10), btScalar(0.)) * ssm);
		localB.getBasis().setEulerZYX(0, 0, PI_4); localB.setOrigin(btVector3(btScalar(0.), btScalar(0.225), btScalar(0.)) * ssm);
		coneC = new btConeTwistConstraint(*m_bones[BODYPART_PELVIS]->m_rigidBody, *m_bones[BODYPART_RIGHT_UPPER_LEG]->m_rigidBody, localA, localB);
		coneC->setLimit(PI_4, PI_4, 0);
		m_joints[JOINT_RIGHT_HIP] = coneC;
		coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

		m_ownerWorld->addConstraint(m_joints[JOINT_RIGHT_HIP], true);

		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0, PI_2, 0); localA.setOrigin(btVector3(btScalar(0.), btScalar(-0.225), btScalar(0.)) * ssm);
		localB.getBasis().setEulerZYX(0, PI_2, 0); localB.setOrigin(btVector3(btScalar(0.), btScalar(0.185), btScalar(0.)) * ssm);
		hingeC = new btHingeConstraint(*m_bones[BODYPART_RIGHT_UPPER_LEG]->m_rigidBody, *m_bones[BODYPART_RIGHT_LOWER_LEG]->m_rigidBody, localA, localB);
		hingeC->setLimit(btScalar(0), btScalar(PI_2));
		m_joints[JOINT_RIGHT_KNEE] = hingeC;
		hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

		m_ownerWorld->addConstraint(m_joints[JOINT_RIGHT_KNEE], true);


		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0, 0, PI); localA.setOrigin(btVector3(btScalar(-0.2), btScalar(0.15), btScalar(0.)) * ssm);
		localB.getBasis().setEulerZYX(0, 0, PI_2); localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.18), btScalar(0.)) * ssm);
		coneC = new btConeTwistConstraint(*m_bones[BODYPART_SPINE]->m_rigidBody, *m_bones[BODYPART_LEFT_UPPER_ARM]->m_rigidBody, localA, localB);
		coneC->setLimit(PI_2, PI_2, 0);
		coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

		m_joints[JOINT_LEFT_SHOULDER] = coneC;
		m_ownerWorld->addConstraint(m_joints[JOINT_LEFT_SHOULDER], true);

		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0, PI_2, 0); localA.setOrigin(btVector3(btScalar(0.), btScalar(0.18), btScalar(0.)) * ssm);
		localB.getBasis().setEulerZYX(0, PI_2, 0); localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.14), btScalar(0.)) * ssm);
		hingeC = new btHingeConstraint(*m_bones[BODYPART_LEFT_UPPER_ARM]->m_rigidBody, *m_bones[BODYPART_LEFT_LOWER_ARM]->m_rigidBody, localA, localB);
//		hingeC->setLimit(btScalar(-M_PI_2), btScalar(0));
		hingeC->setLimit(btScalar(0), btScalar(PI_2));
		m_joints[JOINT_LEFT_ELBOW] = hingeC;
		hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

		m_ownerWorld->addConstraint(m_joints[JOINT_LEFT_ELBOW], true);



		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0, 0, 0); localA.setOrigin(btVector3(btScalar(0.2), btScalar(0.15), btScalar(0.)) * ssm);
		localB.getBasis().setEulerZYX(0, 0, PI_2); localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.18), btScalar(0.)) * ssm);
		coneC = new btConeTwistConstraint(*m_bones[BODYPART_SPINE]->m_rigidBody, *m_bones[BODYPART_RIGHT_UPPER_ARM]->m_rigidBody, localA, localB);
		coneC->setLimit(PI_2, PI_2, 0);
		m_joints[JOINT_RIGHT_SHOULDER] = coneC;
		coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

		m_ownerWorld->addConstraint(m_joints[JOINT_RIGHT_SHOULDER], true);

		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0, PI_2, 0); localA.setOrigin(btVector3(btScalar(0.), btScalar(0.18), btScalar(0.)) * ssm);
		localB.getBasis().setEulerZYX(0, PI_2, 0); localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.14), btScalar(0.)) * ssm);
		hingeC = new btHingeConstraint(*m_bones[BODYPART_RIGHT_UPPER_ARM]->m_rigidBody, *m_bones[BODYPART_RIGHT_LOWER_ARM]->m_rigidBody, localA, localB);
//		hingeC->setLimit(btScalar(-M_PI_2), btScalar(0));
		hingeC->setLimit(btScalar(0), btScalar(PI_2));
		m_joints[JOINT_RIGHT_ELBOW] = hingeC;
		hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

		m_ownerWorld->addConstraint(m_joints[JOINT_RIGHT_ELBOW], true);
	}



