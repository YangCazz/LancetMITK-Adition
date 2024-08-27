#pragma once
#include <vtkSmartPointer.h>
#include <vtkMatrix4x4.h>
#include <vtkAxesActor.h>
#include "JointPartDescription.h"
#include "PKARenderHelper.h"
class RobotBase
{
public:
	RobotBase(JointPartDescription description);
	vtkSmartPointer<vtkAxesActor> GetBaseAxesActor();
	JointPartDescription GetDescription();
private:
	vtkSmartPointer<vtkAxesActor> axesActor;
	JointPartDescription m_JointPartDescription;
};

