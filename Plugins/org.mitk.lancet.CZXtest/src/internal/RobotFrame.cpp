#include "RobotFrame.h"
using namespace lancetAlgorithm;
RobotFrame::RobotFrame(mitk::DataStorage* dataStorage, std::vector<RobotJoint*> robotJoints, RobotBase* base)
{
	m_dataStorage = dataStorage;
	m_RobotJoints = robotJoints;
	int size = robotJoints.size();
	m_EndJoint = robotJoints[size - 1];
	m_Base = base;
}

void RobotFrame::AddRobot()
{
	//PKARenderHelper::LoadSingleMitkFile(m_dataStorage, m_Base->GetDescription().FileName, m_Base->GetDescription().Name);
	for (auto joint : m_RobotJoints)
	{
		std::cout << "JointPath: "<<joint->Path << std::endl;
		std::cout << "JointName: " << joint->Name << std::endl;
		PKARenderHelper::LoadSingleMitkFile(m_dataStorage, joint->Path, joint->Name);
	}
}

void RobotFrame::AddAxis(mitk::IRenderWindowPart* renderWindowPart)
{
	for (auto joint : m_RobotJoints)
	{
		PKARenderHelper::AddActor(renderWindowPart, joint->GetJointAxesActor());
	}
}

void RobotFrame::UpdateJoints(int jointId, double aPosition)
{
	UpdateJointToLink(jointId, aPosition);
	Update(jointId);
}

void RobotFrame::UpdateJoints(double* aPosition)
{
	int size = m_RobotJoints.size();
	for (int i = 0; i < size; ++i)
	{
		UpdateJointToLink(i, aPosition[i]);
	}
	Update();
}

void RobotFrame::DisplayRobot()
{
	for (auto joint : m_RobotJoints)
	{
		auto node = m_dataStorage->GetNamedNode(joint->Name);
		node->SetVisibility(true);
	}
}

void RobotFrame::HideRobot()
{
	for (auto joint : m_RobotJoints)
	{
		auto node = m_dataStorage->GetNamedNode(joint->Name);
		node->SetVisibility(false);
	}
}

void RobotFrame::DispalyAxes()
{
	for (auto joint : m_RobotJoints)
	{
		joint->GetJointAxesActor()->SetVisibility(true);
	}
}

void RobotFrame::HideAxes()
{
	for (auto joint : m_RobotJoints)
	{
		joint->GetJointAxesActor()->SetVisibility(false);
	}
}

void RobotFrame::UpdateJointToLink(int jointID, double position)
{
	auto joint = m_RobotJoints[jointID];
	double d = joint->GetJointDH()[0];
	double a = joint->GetJointDH()[1];
	double alpha = joint->GetJointDH()[2];
	std::cout << "alpha:" << alpha << std::endl;
	double theta = joint->GetJointDH()[3] + CalculationHelper::FromDegreeToRadian(position);
	std::cout << "alpha: " << theta << std::endl;
	vtkSmartPointer<vtkMatrix4x4> m = vtkSmartPointer<vtkMatrix4x4>::New();
	double joint2Link[16] =
	{
		std::cos(theta), -std::sin(theta) * std::cos(alpha),   std::sin(theta) * std::sin(alpha),   a * std::cos(theta),
		std::sin(theta),  std::cos(theta) * std::cos(alpha),    -std::cos(theta) * std::sin(alpha),  a * std::sin(theta),
				 0,       std::sin(alpha),                       std::cos(alpha),    d,
				0,  0,  0,  1
	};
	//PrintDataHelper::CoutMatrix("UpdateJointToLink", joint2Link);
	joint->UpdateJointToLink(joint2Link);
}

void RobotFrame::Update(int id)
{
	UpdateBaseToLinkTool(id);
	UpdateForRender(id);
}

void RobotFrame::UpdateBaseToLinkTool(int id)
{
	for (int jointi = id; jointi < m_RobotJoints.size(); ++jointi)
	{
		if (jointi == 0)
		{
			m_RobotJoints[jointi]->UpdateBaseToLink(m_RobotJoints[jointi]->GetJointToLink());
		}
		else
		{
			vtkSmartPointer<vtkMatrix4x4> base2Link = vtkSmartPointer<vtkMatrix4x4>::New();
			vtkMatrix4x4::Multiply4x4(m_RobotJoints[jointi - 1]->GetBaseToLink(), m_RobotJoints[jointi]->GetJointToLink(), base2Link);
			m_RobotJoints[jointi]->UpdateBaseToLink(base2Link);
		}
	}

	//update Tools
}

void RobotFrame::UpdateForRender(int startId)
{
	for (int jointID = startId; jointID<m_RobotJoints.size(); ++jointID)
	{
		auto joint = m_RobotJoints[jointID];
		auto jointGeometry = m_dataStorage->GetNamedNode(joint->Name)->GetData()->GetGeometry();
		jointGeometry->SetIndexToWorldTransformByVtkMatrix(joint->GetBaseToLink());
		joint->GetJointAxesActor()->SetUserMatrix(joint->GetBaseToLink());
	}
	//updatae Tools
}
