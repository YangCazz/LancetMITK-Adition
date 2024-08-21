#include "RobotBase.h"

RobotBase::RobotBase(JointPartDescription description)
{
	axesActor = PKARenderHelper::GenerateAxesActor();
	m_JointPartDescription = description;
}

vtkSmartPointer<vtkAxesActor> RobotBase::GetBaseAxesActor()
{
	return vtkSmartPointer<vtkAxesActor>();
}

JointPartDescription RobotBase::GetDescription()
{
	return m_JointPartDescription;
}
