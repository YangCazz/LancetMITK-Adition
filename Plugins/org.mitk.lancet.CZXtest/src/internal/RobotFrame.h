#pragma once
#include <vtkAxesActor.h>
#include <vtkSmartPointer.h>
#include <mitkDataStorage.h>
#include "PKARenderHelper.h"
#include "RobotBase.h"
#include "RobotJoint.h"
#include "CalculationHelper.h"
#include "PrintDataHelper.h"
#include <mitkILinkedRenderWindowPart.h>
class RobotFrame
{
public:
	RobotFrame(mitk::DataStorage* dataStorage, std::vector<RobotJoint*> robotJoints,RobotBase* base);
	void AddRobot();
	void AddAxis(mitk::IRenderWindowPart* renderWindowPart);
	void UpdateJoints(int jointId, double aPosition);
	void UpdateJoints(double* aPosition);
	void DisplayRobot();
	void HideRobot();
	void DispalyAxes();
	void HideAxes();
private:
	void UpdateJointToLink(int jointID, double position);
	void Update(int id = 0);
	void UpdateBaseToLinkTool(int id);
	void UpdateForRender(int startId);
private:
	std::vector<RobotJoint*> m_RobotJoints;
	mitk::DataStorage* m_dataStorage;
	RobotJoint* m_EndJoint = nullptr;
	RobotBase* m_Base;
};