#pragma once
#include <osgAnimation/Bone>
#include "btBulletDynamicsCommon.h"
#include "constants.h"
#include "conversionutils.h"


namespace ragdoll
{

	class ModelBone
	{
	public:
		BODYPART m_bodyType;
		btCollisionShape *m_shape = nullptr;
		btRigidBody *m_rigidBody = nullptr;
		btMotionState *m_motionState = nullptr;

		ModelBone *m_parent = nullptr;
		osgAnimation::Bone *m_viewBone = nullptr;

		ModelBone(BODYPART bodyType, btTransform origin, btScalar radius, btScalar height, float mass, RagdollView *ragdollView, btDynamicsWorld* ownerWorld, ModelBone *parent = nullptr);
		btTransform localBoneTransform(btTransform worldTransform);

		btTransform m_worldTransform;
		btScalar m_radius;
		btScalar m_height;
		btDynamicsWorld* m_ownerWorld;
		RagdollView *m_ragdollView;

	};


	class BoneMotionState : public btMotionState
	{
	public:
		BoneMotionState(ModelBone *bone) :
			m_bone(bone)
		{
			m_positionTransform = m_bone->m_worldTransform;
		}

		virtual ~BoneMotionState() {}



		virtual void getWorldTransform(btTransform &worldTrans) const {
			worldTrans = m_positionTransform;
		}

		virtual void setWorldTransform(const btTransform &worldTrans) {
			if (nullptr == m_bone->m_viewBone)
				return; // silently return before we set a node

			m_bone->m_worldTransform = worldTrans;
			m_positionTransform = worldTrans;// localTransform;
		}


		btTransform getLocalTransform() const {
			btTransform localTransform = m_bone->localBoneTransform(m_positionTransform);
			return localTransform;
		}

	protected:
		ModelBone *m_bone;
		btTransform m_positionTransform;
	};

}