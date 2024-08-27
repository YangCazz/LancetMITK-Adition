#pragma once
#include <vtkAxesActor.h>
#include <vtkSmartPointer.h>
#include <vtkMatrix4x4.h>
#include <mitkDataStorage.h>
#include "PKARenderHelper.h"
#include "JointPartDescription.h"
#include "PrintDataHelper.h"
class RobotJoint
{
public:
	RobotJoint(JointPartDescription jointPartDescription);
	vtkSmartPointer<vtkAxesActor> GetJointAxesActor();
	vtkSmartPointer<vtkMatrix4x4> GetBaseToLink();
	vtkSmartPointer<vtkMatrix4x4> GetJointToLink();
	void UpdateJointToLink(vtkMatrix4x4* matrix);
	void UpdateJointToLink(double* matrix);
	void UpdateBaseToLink(vtkMatrix4x4* matrix);
	double* GetJointDH();
public:
	std::string Name;
	std::string Path;
private:
	vtkSmartPointer<vtkAxesActor> m_AxesActor;
	vtkSmartPointer<vtkMatrix4x4> m_Joint2Link;
	vtkSmartPointer<vtkMatrix4x4> m_Base2Link;
	JointPartDescription m_JointPartDescription;
};

