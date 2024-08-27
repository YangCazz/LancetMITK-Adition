#include "RobotJoint.h"

RobotJoint::RobotJoint(JointPartDescription jointPartDescription/*, RobotJoint* ParentJoint = nullptr*/)
{
	//m_AxesActor = vtkSmartPointer<vtkAx>::New();
	m_Joint2Link = vtkSmartPointer<vtkMatrix4x4>::New();
	m_Base2Link = vtkSmartPointer<vtkMatrix4x4>::New();

	m_AxesActor = PKARenderHelper::GenerateAxesActor();
	m_JointPartDescription = jointPartDescription;
	Name = m_JointPartDescription.Name;
	this->Path = m_JointPartDescription.FileName;

}

vtkSmartPointer<vtkAxesActor> RobotJoint::GetJointAxesActor()
{
	return m_AxesActor;
}

vtkSmartPointer<vtkMatrix4x4> RobotJoint::GetBaseToLink()
{
	return m_Base2Link;
}

vtkSmartPointer<vtkMatrix4x4> RobotJoint::GetJointToLink()
{
	return m_Joint2Link;
}

void RobotJoint::UpdateJointToLink(vtkMatrix4x4* matrix)
{
	m_Joint2Link->DeepCopy(matrix);
}

void RobotJoint::UpdateJointToLink(double* matrix)
{
	std::string str = "UpdateJointToLink " + m_JointPartDescription.Name;
	PrintDataHelper::CoutMatrix(str.c_str(), matrix);
	m_Joint2Link->DeepCopy(matrix);
}

void RobotJoint::UpdateBaseToLink(vtkMatrix4x4* matrix)
{
	std::string str = "UpdateBaseToLink " + m_JointPartDescription.Name;
	PrintDataHelper::CoutMatrix(str.c_str(), matrix);
	m_Base2Link->DeepCopy(matrix);
}

double* RobotJoint::GetJointDH()
{
	return m_JointPartDescription.DH;
}
