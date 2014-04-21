#pragma once
#include "btBulletDynamicsCommon.h"
#include <osg/PositionAttitudeTransform>

#define PI 3.1415926535897932384626433832795L

namespace ragdoll{

	class Conversion
	{
	public:

		static osg::Vec3 btToOSGVec3(btVector3 v)
		{
			return osg::Vec3(v.x(), v.y(), v.z());
		}

		static osg::Quat btToOSGQuat(btQuaternion q){
			return osg::Quat(q.x(), q.y(), q.z(), q.w());
		}


		static osg::PositionAttitudeTransform *transformToPAT(btTransform transform)
		{
			osg::PositionAttitudeTransform *pat = new osg::PositionAttitudeTransform();
			osg::Vec3 pos = btToOSGVec3(transform.getOrigin());
			osg::Quat rot = osg::Quat(transform.getRotation().getAngle(), btToOSGVec3(transform.getRotation().getAxis()));
			pat->setPosition(pos);
			pat->setAttitude(rot);
			return pat;
		}

		static void  updateWithTransform(btTransform transform, osg::PositionAttitudeTransform *pat)
		{
			osg::Vec3 pos = btToOSGVec3(transform.getOrigin());
			osg::Quat rot = osg::Quat(transform.getRotation().getAngle(), btToOSGVec3(transform.getRotation().getAxis()));
			pat->setPosition(pos);
			pat->setAttitude(rot);
		}

		static osg::Matrix asOsgMatrix(const btTransform& t)
		{
			osg::Matrix m;
			m.setRotate(btToOSGQuat(t.getRotation()));
			m = m.translate(btToOSGVec3(t.getOrigin()));
			return m;
		}

		static double degreeToRadians(double degree)
		{
			return PI / 180.0 * degree;
		}
	};

}