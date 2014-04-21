#pragma once

#include "btBulletDynamicsCommon.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "conversionutils.h"
#include "modelbone.h"
#include "ragdollview.h"

namespace ragdoll
{

	class RagdollModel
	{

	public:
		RagdollModel(btDynamicsWorld* ownerWorld, RagdollView *ragdollView, const btVector3& positionOffset);

		btDynamicsWorld *getOwnerWorld()
		{
			return m_ownerWorld;
		}

	protected:

		btDynamicsWorld* m_ownerWorld;
		ModelBone *m_bones[BODYPART_COUNT];
		btTypedConstraint* m_joints[JOINT_COUNT];

		RagdollView *m_ragdollView;
	};
}


