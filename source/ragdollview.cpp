// OSG
#include <osg/ShapeDrawable>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osg/Texture2D>
#include "osg/MatrixTransform"
#include <osg/PositionAttitudeTransform>
#include <osg/Geode>
#include <osg/ShapeDrawable>
#include "osg/Math"

#include "ragdollview.h"
#include "conversionutils.h"

using namespace ragdoll;

class RagdollBoneUpdater : public osgAnimation::UpdateBone
{
public:
	RagdollBoneUpdater(BoneMotionState* motionState, const std::string& name) : m_motionState(motionState), UpdateBone(name)
	{

	}

	/** Callback method called by the NodeVisitor when visiting a node.*/
	void operator()(osg::Node* node, osg::NodeVisitor* nv)
	{
		if (nv && nv->getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR)
		{
			osgAnimation::Bone* b = dynamic_cast<osgAnimation::Bone*>(node);
			if (!b)
			{
				OSG_WARN << "Warning: UpdateBone set on non-Bone object." << std::endl;
				return;
			}

			// here we would prefer to have a flag inside transform stack in order to avoid update and a dirty state in matrixTransform if it's not require.
			//const osg::Matrix& matrix = _transforms.getMatrix();
			//b->setMatrix(matrix);
			if (b->getName() == "BODYPART_LEFT_LOWER_LEG")
				int d = 0;

			btTransform transform = m_motionState->getLocalTransform();
			const osg::Matrix& matrix = Conversion::asOsgMatrix(transform);
			b->setMatrix(matrix);

			osgAnimation::Bone* parent = b->getBoneParent();
			if (parent)
				b->setMatrixInSkeletonSpace(b->getMatrixInBoneSpace() * parent->getMatrixInSkeletonSpace());/**///osg::Matrix::translate(matrix.getTrans())
			//b->setInvBindMatrixInSkeletonSpace(matrix);
			else
				b->setMatrixInSkeletonSpace(b->getMatrixInBoneSpace());
		}
		traverse(node, nv);
	}
private:
	BoneMotionState *m_motionState;
};



RagdollView::RagdollView()
{

	m_node = new osg::Group();

	skelroot = new osgAnimation::Skeleton;
	skelroot->setDefaultUpdateCallback();
	m_node->addChild(skelroot.get());

}

void RagdollView::setTexture(osg::ref_ptr<osg::StateSet> stateset, std::string filePath, int unit)
{

	osg::Image* image = osgDB::readImageFile(filePath);
	if (!image)
		std::cout << " File \"" << filePath << "\" not found." << std::endl;
	else
	{
		osg::Texture2D* texture = new osg::Texture2D;
		texture->setImage(image);
		texture->setResizeNonPowerOfTwoHint(false);
		stateset->setTextureAttributeAndModes(unit, texture, osg::StateAttribute::ON);
	}
}




osg::ref_ptr<osg::PositionAttitudeTransform>  RagdollView::createBodyPart(btTransform transform, osg::Node* parent)
{
	osg::ref_ptr<osg::PositionAttitudeTransform> pat = Conversion::transformToPAT(transform);

	osg::Cylinder *cylinder = new osg::Cylinder(osg::Vec3(0.0, 0.0, 0.0), 1.0, 1.0);
	osg::ref_ptr<osg::ShapeDrawable> cylinderDrawable = new osg::ShapeDrawable(cylinder);

	osg::ref_ptr<osg::Geode> cylinderGeode = new osg::Geode();
	cylinderGeode->addDrawable(cylinderDrawable);
	osg::Matrixd turnTransform;

	osg::Quat rotationQuatY(osg::DegreesToRadians(90.0f), osg::Y_AXIS);
	osg::Quat rotationQuatX(osg::DegreesToRadians(90.0f), osg::Z_AXIS);
	turnTransform.makeRotate(rotationQuatY);
	turnTransform *= turnTransform.rotate(rotationQuatX);
	osg::MatrixTransform* matrixTransform = new osg::MatrixTransform(turnTransform);

	matrixTransform->addChild(cylinderGeode);
	pat->addChild(matrixTransform);

	m_node->addChild(pat);
	return pat;
}






osg::Geode* RagdollView::createBoneShape(const osg::Vec3& trans, const osg::Vec4& color)
{
	osg::ref_ptr<osg::Vec3Array> va = new osg::Vec3Array;
	va->push_back(osg::Vec3()); va->push_back(trans);

	osg::ref_ptr<osg::Vec4Array> ca = new osg::Vec4Array;
	ca->push_back(color);

	osg::ref_ptr<osg::Geometry> line = new osg::Geometry;
	line->setVertexArray(va.get());
	line->setColorArray(ca.get());
	line->setColorBinding(osg::Geometry::BIND_OVERALL);
	line->addPrimitiveSet(new osg::DrawArrays(GL_LINES, 0, 2));

	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	geode->addDrawable(line.get());
	geode->getOrCreateStateSet()->setAttributeAndModes(new osg::LineWidth(15.0f));
	geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
	return geode.release();
}

osgAnimation::Bone* RagdollView::createBone(const char* name, const osg::Matrix& transform, osg::Group* parent, BoneMotionState *motionState)
{
	osg::ref_ptr<osgAnimation::Bone> bone = new osgAnimation::Bone;
	parent->insertChild(0, bone.get());
	parent->addChild(createBoneShape(transform.getTrans(), osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f)));

	osg::ref_ptr<RagdollBoneUpdater> updater = new RagdollBoneUpdater(motionState, name);
	//updater->getStackedTransforms().push_back(new osgAnimation::StackedTranslateElement("translate", transform.getTrans()));
	//updater->getStackedTransforms().push_back(new osgAnimation::StackedQuaternionElement("quaternion",transform.getRotate()));

	bone->setUpdateCallback(updater.get());
	bone->setMatrixInSkeletonSpace(transform * bone->getMatrixInSkeletonSpace()); //osg::Matrix::translate(transform.getTrans())
	bone->setName(name);
	return bone.get();
}


osgAnimation::Channel* RagdollView::createChannel(const char* name, const osg::Vec3& axis, float rad)
{
	osg::ref_ptr<osgAnimation::QuatSphericalLinearChannel> ch = new osgAnimation::QuatSphericalLinearChannel;
	ch->setName("quaternion");
	ch->setTargetName(name);

	osgAnimation::QuatKeyframeContainer* kfs = ch->getOrCreateSampler()->getOrCreateKeyframeContainer();
	kfs->push_back(osgAnimation::QuatKeyframe(0.0, osg::Quat(0.0, axis)));
	kfs->push_back(osgAnimation::QuatKeyframe(8.0, osg::Quat(rad, axis)));
	return ch.release();
}
