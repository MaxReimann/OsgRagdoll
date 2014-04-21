#pragma once
// OSG
#include <osg/Geometry>
#include <osg/Geode>
#include <osg/MatrixTransform>

#include <osgAnimation/Bone>
#include <osgAnimation/Skeleton>
#include <osgAnimation/UpdateBone>
#include <osgAnimation/StackedTranslateElement>
#include <osgAnimation/StackedQuaternionElement>
#include <osgAnimation/BasicAnimationManager>
#include <osg/LineWidth>

#include "btBulletDynamicsCommon.h"
#include "modelbone.h"

namespace ragdoll{

	class RagdollView
	{
	public:
		RagdollView();
		void remove();
		void updateBonePositions();
		osg::ref_ptr<osg::PositionAttitudeTransform> createBodyPart(btTransform transform, osg::Node* parent);
		osgAnimation::Bone* createBone(const char* name, const osg::Matrix& transform, osg::Group* parent, BoneMotionState *motionState);

		osg::ref_ptr<osgAnimation::Skeleton> getSkelRoot()
		{
			return skelroot;
		}

		osg::ref_ptr<osg::Group> getRootNode()
		{
			return m_node;
		}

	private:
		void setTexture(osg::ref_ptr<osg::StateSet> stateset, std::string filePath, int unit);
		osgAnimation::Bone* createEndBone(const char* name, const osg::Matrix& transform, osg::Group* parent);
		osgAnimation::Channel* createChannel(const char* name, const osg::Vec3& axis, float rad);
		osg::Geode* createBoneShape(const osg::Vec3& trans, const osg::Vec4& color);
		osg::ref_ptr<osg::MatrixTransform> m_matrixTransform;
		osg::ref_ptr<osgAnimation::Skeleton> skelroot;
		osg::ref_ptr < osg::Group> m_node;
	};

}