/*============================================================================
Robot-assisted Surgical Navigation Extension based on MITK
Copyright (c) Hangzhou LancetRobotics,CHINA
All rights reserved.
============================================================================*/

// Blueberry
#include <berryISelectionService.h>
#include <berryIWorkbenchWindow.h>

// Qt
#include <QMessageBox>

// mitk image
// #include <mitkCLUtil.h>
#include <mitkImage.h>
#include <vtkAppendPolyData.h>
#include <vtkCardinalSpline.h>
#include <vtkSplineFilter.h>
#include <vtkProbeFilter.h>
#include <vtkImageSlabReslice.h>
#include <vtkImageAppend.h>

#include "mitkSurfaceToImageFilter.h"
#include <ep/include/vtk-9.1/vtkTransformFilter.h>
#include <vtkCleanPolyData.h>
#include <vtkClipPolyData.h>
#include <vtkDataSetMapper.h>
#include <vtkImageCast.h>
#include <vtkImplicitPolyDataDistance.h>
#include <vtkPointData.h>

#include "mitkApplyTransformMatrixOperation.h"
#include "mitkInteractionConst.h"
#include "mitkNodePredicateAnd.h"
#include "mitkNodePredicateDataType.h"
#include "mitkNodePredicateNot.h"
#include "mitkNodePredicateOr.h"
#include "mitkNodePredicateProperty.h"
#include "mitkPointSet.h"
#include "mitkRotationOperation.h"
#include "mitkSurface.h"
#include "mitkSurfaceToImageFilter.h"
#include "QmitkSingleNodeSelectionWidget.h"
#include "QmitkDataStorageTreeModel.h"
#include "QmitkDataStorageTreeModelInternalItem.h"
#include "QmitkRenderWindow.h"
#include "surfaceregistraion.h"
#include <vtkWindowLevelLookupTable.h>

#include "HTONDI.h"

/*===============================================================
HTONDI_MoveData.cpp
----------------------------------------------------------------
== ��ǰ�滮�����ƶ�
== ���е��������ƶ�
===============================================================*/


void HTONDI::OnSelectionChanged(berry::IWorkbenchPart::Pointer source, const QList<mitk::DataNode::Pointer>& nodes)
{
	// ѡ����Ҫ�ƶ�������
	// ���û��ڽ�����ѡ��ͬ�����ݽڵ�ʱ�����³�Ա����
	std::string nodeName;
	// iterate all selected objects, adjust warning visibility

	m_Controls.textBrowser_Action->append("User Click.");
	foreach(mitk::DataNode::Pointer node, nodes)
	{
		if (node.IsNull())
		{
			return;
		}
		node->GetName(nodeName);

		if (node != nullptr)
		{
			current = nodeName;
			m_currentSelectedNode = node;
			m_baseDataToMove = node->GetData();
			if (m_currentSelectedNode == nullptr) {
				std::cerr << "Error: No node selected." << std::endl;
				return;
			}
			else {
				// ��ѡ�е���������������
				m_Controls.textBrowser_Action->append("Object Selected: " + QString::fromStdString(nodeName));
				//QModelIndex parentIndex = m_NodetreeModel->GetIndex(m_currentSelectedNode);

				//int parentRowCount = m_NodetreeModel->rowCount(parentIndex);
				//int parentColumnCount = m_NodetreeModel->columnCount(parentIndex);

				//cout<< parentRowCount << " "<< parentColumnCount <<endl;
			}
		}
	}
}


void HTONDI::Rotate(const double center[3], double direction[3], double counterclockwiseDegree, mitk::BaseData* data)
{
	// Apply Rotate to Object
	/*
	Input��
	 1. ��ת����
	 2. ��ת�᷽��
	 3. ��ת�Ƕ�
	 4. ��Ҫ��ת������
	*/
	if (data)
	{
		// ��׼��
		double normalizedDirection[3];
		double directionLength = sqrt((pow(direction[0], 2) + pow(direction[1], 2) + pow(direction[2], 2)));
		normalizedDirection[0] = direction[0] / directionLength;
		normalizedDirection[1] = direction[1] / directionLength;
		normalizedDirection[2] = direction[2] / directionLength;

		// ��ֵ��ת���� �� ��ת�᷽��
		mitk::Point3D rotateCenter{ center };
		mitk::Vector3D rotateAxis{ normalizedDirection };

		// �����˶�ʵ��, counterclockwiseDegree ��ʾ��ת�Ƕ�
		auto* doOp = new mitk::RotationOperation(mitk::OpROTATE, rotateCenter, rotateAxis, counterclockwiseDegree);

		// Ӧ���˶�ʵ��
		data->GetGeometry()->ExecuteOperation(doOp);

		// ɾ���˶�ʵ��
		delete doOp;

		if (current == "1st cut plane")
		{
			// ��ȡˮƽ�ع���ķ�����
			auto cutPlaneSource01 = dynamic_cast<mitk::Surface*>(GetDataStorage()->GetNamedNode("1st cut plane")->GetData());
			auto cutPlanePolyData01 = cutPlaneSource01->GetVtkPolyData();
			double centerPlane01[3], normalPlane01[3];

			// ȡ�õ�һ�ع��浱ǰ�ķ�������ƽ������
			GetPlaneNormal(cutPlanePolyData01, normalPlane01, centerPlane01);
			QString tmpNormal, tmpCenter;
			tmpNormal = "normal:" + QString::number(normalPlane01[0]) + ", "
				+ QString::number(normalPlane01[1]) + ", "
				+ QString::number(normalPlane01[2]) + ", ";
			tmpCenter = "center:" + QString::number(centerPlane01[0]) + ", "
				+ QString::number(centerPlane01[1]) + ", "
				+ QString::number(centerPlane01[2]) + ", ";
			m_Controls.textBrowser_Action->append(tmpNormal);
			m_Controls.textBrowser_Action->append(tmpCenter);
		}

		// ���´���
		data->Update();
		mitk::RenderingManager::GetInstance()->RequestUpdateAll();
	}
}


void HTONDI::Translate(double direction[3], double length, mitk::BaseData* data)
{
	// Apply Translate to Object
	/*
	Input��
	 1. ƽ�Ʒ���
	 2. ƽ�ƾ���
	 3. ��Ҫ�ƶ�����
	*/
	if (data != nullptr)
	{
		// ��׼��
		double directionLength = sqrt((pow(direction[0], 2) + pow(direction[1], 2) + pow(direction[2], 2)));

		// ��ֵƽ������
		mitk::Point3D movementVector;
		movementVector[0] = length * direction[0] / directionLength;
		movementVector[1] = length * direction[1] / directionLength;
		movementVector[2] = length * direction[2] / directionLength;

		// �����˶�ʵ��
		auto* doOp = new mitk::PointOperation(mitk::OpMOVE, 0, movementVector, 0);

		// Ӧ���˶�ʵ��
		data->GetGeometry()->ExecuteOperation(doOp);

		// ɾ���˶�ʵ��
		delete doOp;

		if (current == "1st cut plane")
		{
			//// ��ȡˮƽ�ع���ķ�����
			//auto cutPlaneSource01 = dynamic_cast<mitk::Surface*>(GetDataStorage()->GetNamedNode("1st cut plane")->GetData());
			//auto cutPlanePolyData01 = cutPlaneSource01->GetVtkPolyData();
			//double centerPlane01[3], normalPlane01[3];

			//// ȡ�õ�һ�ع��浱ǰ�ķ�������ƽ������
			//GetPlaneNormal(cutPlanePolyData01, normalPlane01, centerPlane01);
			//QString tmpNormal, tmpCenter;
			//tmpNormal = "normal:" + QString::number(normalPlane01[0]) + ", "
			//	+ QString::number(normalPlane01[1]) + ", "
			//	+ QString::number(normalPlane01[2]) + ", ";
			//tmpCenter = "center:" + QString::number(centerPlane01[0]) + ", "
			//	+ QString::number(centerPlane01[1]) + ", "
			//	+ QString::number(centerPlane01[2]) + ", ";
			//m_Controls.textBrowser_Action->append(tmpNormal);
			//m_Controls.textBrowser_Action->append(tmpCenter);
			Eigen::Vector3d normal = ExtractNormalFromPlane(current);
			QString tmpNormal = "normal:" + QString::number(normal[0]) + ", "
				+ QString::number(normal[1]) + ", "
				+ QString::number(normal[2]) + ", ";
			m_Controls.textBrowser_Action->append(tmpNormal);
			
		}

		// ���´���
		data->Update();
		mitk::RenderingManager::GetInstance()->RequestUpdateAll();
	}
}


void HTONDI::TranslateMinusX()
{	
	if (current == "") {
		m_Controls.textBrowser_Action->append("Plesase Select Any Object first.");
		return;
	}
	// �� -x �����ƶ�
	double direction[3]{ -1,0,0 };

	// �����ƶ�
	Translate(direction, m_Controls.lineEdit_intuitiveValue->text().toDouble(), GetDataStorage()->GetNamedNode(current)->GetData());
	if (current == "tibiaSurface") 
	{
		Translate(direction, m_Controls.lineEdit_intuitiveValue->text().toDouble(), GetDataStorage()->GetNamedNode("tibiaLandmarkPointSet")->GetData());
		Translate(direction, m_Controls.lineEdit_intuitiveValue->text().toDouble(), GetDataStorage()->GetNamedNode("tibiaICPPointSet")->GetData());
		OnShowMachineLineClicked();
	}
	if (current == "1st cut plane")
	{
		// ƽ���ϵ�
		Translate(direction, m_Controls.lineEdit_intuitiveValue->text().toDouble(), GetDataStorage()->GetNamedNode("pointSetInPlaneCutPlane1")->GetData());
		// Ȼ�󣬼���ʵʱ�Ľع���
		if (GetIntersectionLine())
		{
			GetDistancefromTibia();
		}
	}
	if (current == "SteelPlate")
	{
		// ͬʱ�ƶ������
		Translate(direction, m_Controls.lineEdit_intuitiveValue->text().toDouble(), GetDataStorage()->GetNamedNode("SteelPlatePointSet")->GetData());
	}

}


void HTONDI::TranslateMinusY()
{
	if (current == "") {
		m_Controls.textBrowser_Action->append("Plesase Select Any Object first.");
		return;
	}
	
	// �� -y �����ƶ�
	double direction[3]{ 0,-1,0 };

	// �����ƶ�
	Translate(direction, m_Controls.lineEdit_intuitiveValue->text().toDouble(), GetDataStorage()->GetNamedNode(current)->GetData());
	if (current == "tibiaSurface")
	{
		Translate(direction, m_Controls.lineEdit_intuitiveValue->text().toDouble(), GetDataStorage()->GetNamedNode("tibiaLandmarkPointSet")->GetData());
		Translate(direction, m_Controls.lineEdit_intuitiveValue->text().toDouble(), GetDataStorage()->GetNamedNode("tibiaICPPointSet")->GetData());
		OnShowMachineLineClicked();
	}
	if (current == "1st cut plane")
	{
		// ƽ���ϵ�
		Translate(direction, m_Controls.lineEdit_intuitiveValue->text().toDouble(), GetDataStorage()->GetNamedNode("pointSetInPlaneCutPlane1")->GetData());
		// Ȼ�󣬼���ʵʱ�Ľع���
		if (GetIntersectionLine())
		{
			GetDistancefromTibia();
		}
	}
	if (current == "SteelPlate")
	{
		// ͬʱ�ƶ������
		Translate(direction, m_Controls.lineEdit_intuitiveValue->text().toDouble(), GetDataStorage()->GetNamedNode("SteelPlatePointSet")->GetData());


	}
}


void HTONDI::TranslateMinusZ()
{
	if (current == "") {
		m_Controls.textBrowser_Action->append("Plesase Select Any Object first.");
		return;
	}
	// �� -z �����ƶ�
	double direction[3]{ 0,0,-1 };

	// �����ƶ�
	Translate(direction, m_Controls.lineEdit_intuitiveValue->text().toDouble(), GetDataStorage()->GetNamedNode(current)->GetData());
	if (current == "tibiaSurface")
	{
		Translate(direction, m_Controls.lineEdit_intuitiveValue->text().toDouble(), GetDataStorage()->GetNamedNode("tibiaLandmarkPointSet")->GetData());
		Translate(direction, m_Controls.lineEdit_intuitiveValue->text().toDouble(), GetDataStorage()->GetNamedNode("tibiaICPPointSet")->GetData());
		OnShowMachineLineClicked();
	}
	if (current == "1st cut plane")
	{
		// ƽ���ϵ�
		Translate(direction, m_Controls.lineEdit_intuitiveValue->text().toDouble(), GetDataStorage()->GetNamedNode("pointSetInPlaneCutPlane1")->GetData());
		// Ȼ�󣬼���ʵʱ�Ľع���
		if (GetIntersectionLine())
		{
			GetDistancefromTibia();
		}
	}
	if (current == "SteelPlate")
	{
		// ͬʱ�ƶ������
		Translate(direction, m_Controls.lineEdit_intuitiveValue->text().toDouble(), GetDataStorage()->GetNamedNode("SteelPlatePointSet")->GetData());


	}
}


void HTONDI::TranslatePlusX()
{
	if (current == "") {
		m_Controls.textBrowser_Action->append("Plesase Select Any Object first.");
		return;
	}
	// �� +x �����ƶ�
	// �ƶ�����
	double direction[3]{ 1,0,0 };

	// �����ƶ�
	Translate(direction, m_Controls.lineEdit_intuitiveValue->text().toDouble(), GetDataStorage()->GetNamedNode(current)->GetData());
	if (current == "tibiaSurface")
	{
		Translate(direction, m_Controls.lineEdit_intuitiveValue->text().toDouble(), GetDataStorage()->GetNamedNode("tibiaLandmarkPointSet")->GetData());
		Translate(direction, m_Controls.lineEdit_intuitiveValue->text().toDouble(), GetDataStorage()->GetNamedNode("tibiaICPPointSet")->GetData());
		OnShowMachineLineClicked();
	}
	if (current == "1st cut plane")
	{
		// ƽ���ϵ�
		Translate(direction, m_Controls.lineEdit_intuitiveValue->text().toDouble(), GetDataStorage()->GetNamedNode("pointSetInPlaneCutPlane1")->GetData());
		// Ȼ�󣬼���ʵʱ�Ľع���
		if (GetIntersectionLine())
		{
			GetDistancefromTibia();
		}
	}
	if (current == "SteelPlate")
	{
		// ͬʱ�ƶ������
		Translate(direction, m_Controls.lineEdit_intuitiveValue->text().toDouble(), GetDataStorage()->GetNamedNode("SteelPlatePointSet")->GetData());


	}
}


void HTONDI::TranslatePlusY()
{
	if (current == "") {
		m_Controls.textBrowser_Action->append("Plesase Select Any Object first.");
		return;
	}
	// �� +y �����ƶ�
	double direction[3]{ 0,1,0 };

	// �����ƶ�
	Translate(direction, m_Controls.lineEdit_intuitiveValue->text().toDouble(), GetDataStorage()->GetNamedNode(current)->GetData());
	if (current == "tibiaSurface")
	{
		Translate(direction, m_Controls.lineEdit_intuitiveValue->text().toDouble(), GetDataStorage()->GetNamedNode("tibiaLandmarkPointSet")->GetData());
		Translate(direction, m_Controls.lineEdit_intuitiveValue->text().toDouble(), GetDataStorage()->GetNamedNode("tibiaICPPointSet")->GetData());
		OnShowMachineLineClicked();
	}
	if (current == "1st cut plane")
	{
		// ƽ���ϵ�
		Translate(direction, m_Controls.lineEdit_intuitiveValue->text().toDouble(), GetDataStorage()->GetNamedNode("pointSetInPlaneCutPlane1")->GetData());
		// Ȼ�󣬼���ʵʱ�Ľع���
		if (GetIntersectionLine())
		{
			GetDistancefromTibia();
		}
	}
	if (current == "SteelPlate")
	{
		// ͬʱ�ƶ������
		Translate(direction, m_Controls.lineEdit_intuitiveValue->text().toDouble(), GetDataStorage()->GetNamedNode("SteelPlatePointSet")->GetData());


	}
}


void HTONDI::TranslatePlusZ()
{
	if (current == "") {
		m_Controls.textBrowser_Action->append("Plesase Select Any Object first.");
		return;
	}
	// �� +z �����ƶ�
	double direction[3]{ 0,0,1 };

	// �����ƶ�
	Translate(direction, m_Controls.lineEdit_intuitiveValue->text().toDouble(), GetDataStorage()->GetNamedNode(current)->GetData());
	if (current == "tibiaSurface")
	{
		Translate(direction, m_Controls.lineEdit_intuitiveValue->text().toDouble(), GetDataStorage()->GetNamedNode("tibiaLandmarkPointSet")->GetData());
		Translate(direction, m_Controls.lineEdit_intuitiveValue->text().toDouble(), GetDataStorage()->GetNamedNode("tibiaICPPointSet")->GetData());
		OnShowMachineLineClicked();
	}
	if (current == "1st cut plane")
	{
		// ƽ���ϵ�
		Translate(direction, m_Controls.lineEdit_intuitiveValue->text().toDouble(), GetDataStorage()->GetNamedNode("pointSetInPlaneCutPlane1")->GetData());
		// Ȼ�󣬼���ʵʱ�Ľع���
		if (GetIntersectionLine())
		{
			GetDistancefromTibia();
		}
	}
	if (current == "SteelPlate")
	{
		// ͬʱ�ƶ������
		Translate(direction, m_Controls.lineEdit_intuitiveValue->text().toDouble(), GetDataStorage()->GetNamedNode("SteelPlatePointSet")->GetData());
	}
}


void HTONDI::RotatePlusX()
{
	if (current == "") {
		m_Controls.textBrowser_Action->append("Plesase Select Any Object first.");
		return;
	}
	// ���������� +X ��ķ�����ת
	double direction[3]{ 1,0,0 };
	double angle = m_Controls.lineEdit_intuitiveValue->text().toDouble();

	// Ӧ����ת
	Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(), 
		direction, 
		angle, 
		GetDataStorage()->GetNamedNode(current)->GetData());
	if(current == "tibiaSurface")
	{
		// ��ת�ֹ��ϵ�
		Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
			direction,
			angle,
			GetDataStorage()->GetNamedNode("tibiaLandmarkPointSet")->GetData());
		Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
			direction,
			angle,
			GetDataStorage()->GetNamedNode("tibiaICPPointSet")->GetData());
		OnShowMachineLineClicked();
	}
	if (current == "1st cut plane")
	{
		// ƽ���ϵ�
		Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
			direction,
			angle,
			GetDataStorage()->GetNamedNode("pointSetInPlaneCutPlane1")->GetData());
		// Ȼ�󣬼���ʵʱ�Ľع���
		if (GetIntersectionLine())
		{
			GetDistancefromTibia();
		}
	}
	if (current == "SteelPlate")
	{
		// ͬʱ�ƶ������
		Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
			direction,
			angle,
			GetDataStorage()->GetNamedNode("SteelPlatePointSet")->GetData());
	}
	
}


void HTONDI::RotatePlusY()
{
	if (current == "") {
		m_Controls.textBrowser_Action->append("Plesase Select Any Object first.");
		return;
	}
	// ���������� +Y ��ķ�����ת
	double direction[3]{ 0,1,0 };
	double angle = m_Controls.lineEdit_intuitiveValue->text().toDouble();
	// Ӧ����ת
	Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
		direction,
		angle,
		GetDataStorage()->GetNamedNode(current)->GetData());
	if (current == "tibiaSurface")
	{
		// ��ת�ֹ��ϵ�
		Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
			direction,
			angle,
			GetDataStorage()->GetNamedNode("tibiaLandmarkPointSet")->GetData());
		Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
			direction,
			angle,
			GetDataStorage()->GetNamedNode("tibiaICPPointSet")->GetData());
		OnShowMachineLineClicked();
	}
	if (current == "1st cut plane")
	{
		// ƽ���ϵ�
		Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
			direction,
			angle,
			GetDataStorage()->GetNamedNode("pointSetInPlaneCutPlane1")->GetData());
		// Ȼ�󣬼���ʵʱ�Ľع���
		if (GetIntersectionLine())
		{
			GetDistancefromTibia();
		}
	}
	if (current == "SteelPlate")
	{
		// ͬʱ�ƶ������
		Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
			direction,
			angle,
			GetDataStorage()->GetNamedNode("SteelPlatePointSet")->GetData());
	}
}


void HTONDI::RotatePlusZ()
{
	if (current == "") {
		m_Controls.textBrowser_Action->append("Plesase Select Any Object first.");
		return;
	}
	// ���������� +Z ��ķ�����ת
	double direction[3]{ 0,0,1 };
	double angle = m_Controls.lineEdit_intuitiveValue->text().toDouble();
	// Ӧ����ת
	Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
		direction,
		angle,
		GetDataStorage()->GetNamedNode(current)->GetData());
	if (current == "tibiaSurface")
	{
		// ��ת�ֹ��ϵ�
		Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
			direction,
			angle,
			GetDataStorage()->GetNamedNode("tibiaLandmarkPointSet")->GetData());
		Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
			direction,
			angle,
			GetDataStorage()->GetNamedNode("tibiaICPPointSet")->GetData());
		OnShowMachineLineClicked();
	}
	if (current == "1st cut plane")
	{
		// ƽ���ϵ�
		Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
			direction,
			angle,
			GetDataStorage()->GetNamedNode("pointSetInPlaneCutPlane1")->GetData());
		// Ȼ�󣬼���ʵʱ�Ľع���
		if (GetIntersectionLine())
		{
			GetDistancefromTibia();
		}
	}
	if (current == "SteelPlate")
	{
		// ͬʱ�ƶ������
		Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
			direction,
			angle,
			GetDataStorage()->GetNamedNode("SteelPlatePointSet")->GetData());
	}
}


void HTONDI::RotateMinusX()
{
	if (current == "") {
		m_Controls.textBrowser_Action->append("Plesase Select Any Object first.");
		return;
	}
	// ���������� -X ��ķ�����ת
	double direction[3]{ 1,0,0 };
	double angle = -m_Controls.lineEdit_intuitiveValue->text().toDouble();
	// Ӧ����ת
	Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
		direction,
		angle,
		GetDataStorage()->GetNamedNode(current)->GetData());
	if (current == "tibiaSurface")
	{
		// ��ת�ֹ��ϵ�
		Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
			direction,
			angle,
			GetDataStorage()->GetNamedNode("tibiaLandmarkPointSet")->GetData());
		Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
			direction,
			angle,
			GetDataStorage()->GetNamedNode("tibiaICPPointSet")->GetData());
		OnShowMachineLineClicked();
	}
	if (current == "1st cut plane")
	{
		// ƽ���ϵ�
		Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
			direction,
			angle,
			GetDataStorage()->GetNamedNode("pointSetInPlaneCutPlane1")->GetData());
		// Ȼ�󣬼���ʵʱ�Ľع���
		if (GetIntersectionLine())
		{
			GetDistancefromTibia();
		}
	}
	if (current == "SteelPlate")
	{
		// ͬʱ�ƶ������
		Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
			direction,
			angle,
			GetDataStorage()->GetNamedNode("SteelPlatePointSet")->GetData());
	}
}


void HTONDI::RotateMinusY()
{
	if (current == "") {
		m_Controls.textBrowser_Action->append("Plesase Select Any Object first.");
		return;
	}
	// ���������� -Y ��ķ�����ת
	double direction[3]{ 0,1,0 };
	double angle = -m_Controls.lineEdit_intuitiveValue->text().toDouble();
	// Ӧ����ת
	Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
		direction,
		angle,
		GetDataStorage()->GetNamedNode(current)->GetData());
	if (current == "tibiaSurface")
	{
		// ��ת�ֹ��ϵ�
		Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
			direction,
			angle,
			GetDataStorage()->GetNamedNode("tibiaLandmarkPointSet")->GetData());
		Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
			direction,
			angle,
			GetDataStorage()->GetNamedNode("tibiaICPPointSet")->GetData());
		OnShowMachineLineClicked();
	}
	if (current == "1st cut plane")
	{
		// �ڽ��и������Եĸ�����ʱ����Ҫȷ�����Դ���
		// ȷ������ => ����������Ҫ�� "�ɹ�ͷ����" �� "�׹ؽ�����"
		// ƽ���ϵ�
		Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
			direction,
			angle,
			GetDataStorage()->GetNamedNode("pointSetInPlaneCutPlane1")->GetData());
		// Ȼ�󣬼���ʵʱ�Ľع���
		if (GetIntersectionLine())
		{
			GetDistancefromTibia();
		}
	}
	if (current == "SteelPlate")
	{
		// ͬʱ�ƶ������
		Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
			direction,
			angle,
			GetDataStorage()->GetNamedNode("SteelPlatePointSet")->GetData());
	}
}


void HTONDI::RotateMinusZ()
{
	if (current == "") {
		m_Controls.textBrowser_Action->append("Plesase Select Any Object first.");
		return;
	}
	// ���������� -Z ��ķ�����ת
	double direction[3]{ 0,0,1 };
	double angle = -m_Controls.lineEdit_intuitiveValue->text().toDouble();
	// Ӧ����ת
	Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
		direction,
		angle,
		GetDataStorage()->GetNamedNode(current)->GetData());
	if (current == "tibiaSurface")
	{
		// ��ת�ֹ��ϵ�
		Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
			direction,
			angle,
			GetDataStorage()->GetNamedNode("tibiaLandmarkPointSet")->GetData());
		Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
			direction,
			angle,
			GetDataStorage()->GetNamedNode("tibiaICPPointSet")->GetData());
		OnShowMachineLineClicked();
	}
	if (current == "1st cut plane")
	{
		// ƽ���ϵ�
		Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
			direction,
			angle,
			GetDataStorage()->GetNamedNode("pointSetInPlaneCutPlane1")->GetData());
		// Ȼ�󣬼���ʵʱ�Ľع���
		if (GetIntersectionLine())
		{
			GetDistancefromTibia();
		}
	}
	if (current == "SteelPlate")
	{
		// ͬʱ�ƶ������
		Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
			direction,
			angle,
			GetDataStorage()->GetNamedNode("SteelPlatePointSet")->GetData());
	}
}

void HTONDI::RotateMinus()
{
	// �Խع�Զ�˽�����ת���ƺ�ҳ��������ʱ��
	auto distalTibiaSurface = GetDataStorage()->GetNamedNode("distal tibiaSurface");
	if (distalTibiaSurface == nullptr) {
		m_Controls.textBrowser_Action->append("Please Cut Tibia First.");
		return;
	}
	double vx = mitkPointSet1->GetPoint(0)[0] - mitkPointSet1->GetPoint(1)[0];
	double vy = mitkPointSet1->GetPoint(0)[1] - mitkPointSet1->GetPoint(1)[1];
	double vz = mitkPointSet1->GetPoint(0)[2] - mitkPointSet1->GetPoint(1)[2];
	double length = sqrt(pow(vx, 2) + pow(vy, 2) + pow(vz, 2));
	double direction_cutPlane[3]{ vx / length,vy / length,vz / length };
	double angle = m_Controls.LineEdit_transAngle->text().toDouble();

	// �����������ȣ���Ʋ�ͬ����ת��
	// 0:����. 1������
	double center[3];
	if (judgModel_flag == 1) {
		center[0] = mitkPointSet1->GetPoint(3)[0];
		center[1] = mitkPointSet1->GetPoint(3)[1];
		center[2] = mitkPointSet1->GetPoint(3)[2];
	}
	else if(judgModel_flag == 0) {
		center[0] = mitkPointSet1->GetPoint(0)[0];
		center[1] = mitkPointSet1->GetPoint(0)[1];
		center[2] = mitkPointSet1->GetPoint(0)[2];
	}
	

	// Ӧ����ת
	Rotate(center,direction_cutPlane,angle,distalTibiaSurface->GetData());
	// ͬʱ�ƶ��ֹ��ϵ�Զ�˵㣨�����µ����͵��ĸ��㣩
	// ȡ��Զ�˵������㣬���������㼯
	auto tibiaLandmarkNode = GetDataStorage()->GetNamedNode("tibiaLandmarkPointSet");
	if (tibiaLandmarkNode == nullptr) 
	{
		m_Controls.textBrowser_Action->append("tibiaLandmarkPointSet Not Found!");
		return;
	}
	
	// ����һ���µĵ㼯�����òɼ����ĵ�
	mitk::PointSet::Pointer landmarkPointSet = dynamic_cast<mitk::PointSet*>(tibiaLandmarkNode->GetData());
	mitk::PointSet::Pointer tibiaDistalPoints = mitk::PointSet::New();
	tibiaDistalPoints->InsertPoint(0, landmarkPointSet->GetPoint(2));
	tibiaDistalPoints->InsertPoint(1, landmarkPointSet->GetPoint(3));
	// ����һ���µ����ݽڵ㲢���õ㼯
	// ����Ѿ�������ֱ�Ӹ�������
	auto tibiaDistal = GetDataStorage()->GetNamedNode("tibiaDistalPointSet");
	if (tibiaDistal != nullptr)
	{
		m_Controls.textBrowser_Action->append("Refresh tibiaDistalPointSet");
		// Ӧ����ת
		Rotate(center, direction_cutPlane, angle, tibiaDistal->GetData());
		// ���´���
		mitk::RenderingManager::GetInstance()->RequestUpdateAll();

	}
	else
	{
		// �����½�һ������
		m_Controls.textBrowser_Action->append("Create new PointSet");
		mitk::DataNode::Pointer tibiaDistalPointSet = mitk::DataNode::New();
		tibiaDistalPointSet->SetData(tibiaDistalPoints);
		tibiaDistalPointSet->SetName("tibiaDistalPointSet");
		// ���ýڵ���ɫΪ��ɫ
		tibiaDistalPointSet->SetColor(0.0, 0.0, 1.0); // RGB: Blue
		tibiaDistalPointSet->SetFloatProperty("pointsize", 5);
		GetDataStorage()->Add(tibiaDistalPointSet);
		// Ӧ����ת
		Rotate(center, direction_cutPlane, angle, tibiaDistalPointSet->GetData());
	}
	// Ȼ�󣬼���һ���µ�����
	OnShowMachineLine02Clicked();
	
	// ���´���
	mitk::RenderingManager::GetInstance()->RequestUpdateAll();
}


void HTONDI::RotatePlus()
{
	// ������Ҫ�б��ҳ����������
	// ���ȣ�
	// �Խع�Զ�˽�����ת���ƺ�ҳ������˳ʱ��
	auto distalTibiaSurface = GetDataStorage()->GetNamedNode("distal tibiaSurface");
	if (distalTibiaSurface == nullptr) {
		m_Controls.textBrowser_Action->append("Please Cut Tibia First.");
		return;
	}
	double vx = mitkPointSet1->GetPoint(0)[0] - mitkPointSet1->GetPoint(1)[0];
	double vy = mitkPointSet1->GetPoint(0)[1] - mitkPointSet1->GetPoint(1)[1];
	double vz = mitkPointSet1->GetPoint(0)[2] - mitkPointSet1->GetPoint(1)[2];
	double length = sqrt(pow(vx, 2) + pow(vy, 2) + pow(vz, 2));
	double direction_cutPlane[3]{ vx / length,vy / length,vz / length };
	double angle = -m_Controls.LineEdit_transAngle->text().toDouble();

	// �����������ȣ���Ʋ�ͬ����ת��
	// 0:����. 1������
	double center[3];
	if (judgModel_flag == 1) {
		center[0] = mitkPointSet1->GetPoint(3)[0];
		center[1] = mitkPointSet1->GetPoint(3)[1];
		center[2] = mitkPointSet1->GetPoint(3)[2];
	}
	else if (judgModel_flag == 0) {
		center[0] = mitkPointSet1->GetPoint(0)[0];
		center[1] = mitkPointSet1->GetPoint(0)[1];
		center[2] = mitkPointSet1->GetPoint(0)[2];
	}

	// Ӧ����ת
	Rotate(center, direction_cutPlane, angle, distalTibiaSurface->GetData());
	// ͬʱ�ƶ��ֹ��ϵ�Զ�˵㣨�����µ����͵��ĸ��㣩
	// ȡ��Զ�˵������㣬���������㼯
	auto tibiaLandmarkNode = GetDataStorage()->GetNamedNode("tibiaLandmarkPointSet");
	if (tibiaLandmarkNode == nullptr)
	{
		m_Controls.textBrowser_Action->append("tibiaLandmarkPointSet Not Found!");
		return;
	}

	// ����һ���µĵ㼯�����òɼ����ĵ�
	mitk::PointSet::Pointer landmarkPointSet = dynamic_cast<mitk::PointSet*>(tibiaLandmarkNode->GetData());
	mitk::PointSet::Pointer tibiaDistalPoints = mitk::PointSet::New();
	tibiaDistalPoints->InsertPoint(0, landmarkPointSet->GetPoint(2));
	tibiaDistalPoints->InsertPoint(1, landmarkPointSet->GetPoint(3));
	// ����һ���µ����ݽڵ㲢���õ㼯
	// ����Ѿ�������ֱ�Ӹ�������
	auto tibiaDistal = GetDataStorage()->GetNamedNode("tibiaDistalPointSet");
	if (tibiaDistal != nullptr)
	{
		m_Controls.textBrowser_Action->append("Refresh tibiaDistalPointSet");
		// Ӧ����ת
		Rotate(center, direction_cutPlane, angle, tibiaDistal->GetData());
		// ���´���
		mitk::RenderingManager::GetInstance()->RequestUpdateAll();
	}
	else
	{
		// �����½�һ������
		m_Controls.textBrowser_Action->append("Create new PointSet");
		mitk::DataNode::Pointer tibiaDistalPointSet = mitk::DataNode::New();
		tibiaDistalPointSet->SetData(tibiaDistalPoints);
		tibiaDistalPointSet->SetName("tibiaDistalPointSet");
		// ���ýڵ���ɫΪ��ɫ
		tibiaDistalPointSet->SetColor(0.0, 0.0, 1.0); // RGB: Blue
		tibiaDistalPointSet->SetFloatProperty("pointsize", 5);
		GetDataStorage()->Add(tibiaDistalPointSet);
		// Ӧ����ת
		Rotate(center, direction_cutPlane, angle, tibiaDistalPointSet->GetData());
	}
	// Ȼ�󣬼���һ���µ�����
	OnShowMachineLine02Clicked();

	// ���´���
	mitk::RenderingManager::GetInstance()->RequestUpdateAll();
}


void HTONDI::TranslateMinusX2()
{
	if (current == "") {
		m_Controls.textBrowser_Action->append("Plesase Select Any Object first.");
		return;
	}
	// �� -x �����ƶ�
	double direction[3]{ -1,0,0 };

	// �����ƶ�
	Translate(direction, m_Controls.lineEdit_intuitiveValue_2->text().toDouble(), GetDataStorage()->GetNamedNode(current)->GetData());

	if (current == "Saw")
	{
		Translate(direction, m_Controls.lineEdit_intuitiveValue_2->text().toDouble(), GetDataStorage()->GetNamedNode("SawLandMarkPointSet")->GetData());
		if (m_cutType == 1)
		{
			// ��Ҫͬʱ�ƶ� ƽ�� ƽ���ϵĵ� �Լ������
			Translate(direction, m_Controls.lineEdit_intuitiveValue_2->text().toDouble(), GetDataStorage()->GetNamedNode("StateCutPlane01")->GetData());
			Translate(direction, m_Controls.lineEdit_intuitiveValue_2->text().toDouble(), GetDataStorage()->GetNamedNode("pointSetInRealPlaneAxial")->GetData());
			CaculateCutPlaneMiss01();
		}
		if (m_cutType == 2)
		{
			// ��Ҫͬʱ�ƶ� ƽ�� ƽ���ϵĵ� �Լ������
			Translate(direction, m_Controls.lineEdit_intuitiveValue_2->text().toDouble(), GetDataStorage()->GetNamedNode("StateCutPlane02")->GetData());
			Translate(direction, m_Controls.lineEdit_intuitiveValue_2->text().toDouble(), GetDataStorage()->GetNamedNode("pointSetInRealPlaneSag")->GetData());
			CaculateCutPlaneMiss02();
		}
	}

}


void HTONDI::TranslateMinusY2()
{
	if (current == "") {
		m_Controls.textBrowser_Action->append("Plesase Select Any Object first.");
		return;
	}

	// �� -y �����ƶ�
	double direction[3]{ 0,-1,0 };

	// �����ƶ�
	Translate(direction, m_Controls.lineEdit_intuitiveValue_2->text().toDouble(), GetDataStorage()->GetNamedNode(current)->GetData());

	if (current == "Saw")
	{
		Translate(direction, m_Controls.lineEdit_intuitiveValue_2->text().toDouble(), GetDataStorage()->GetNamedNode("SawLandMarkPointSet")->GetData());
		if (m_cutType == 1)
		{
			// ��Ҫͬʱ�ƶ� ƽ�� ƽ���ϵĵ� �Լ������
			Translate(direction, m_Controls.lineEdit_intuitiveValue_2->text().toDouble(), GetDataStorage()->GetNamedNode("StateCutPlane01")->GetData());
			Translate(direction, m_Controls.lineEdit_intuitiveValue_2->text().toDouble(), GetDataStorage()->GetNamedNode("pointSetInRealPlaneAxial")->GetData());
			CaculateCutPlaneMiss01();
		}
		if (m_cutType == 2)
		{
			// ��Ҫͬʱ�ƶ� ƽ�� ƽ���ϵĵ� �Լ������
			Translate(direction, m_Controls.lineEdit_intuitiveValue_2->text().toDouble(), GetDataStorage()->GetNamedNode("StateCutPlane02")->GetData());
			Translate(direction, m_Controls.lineEdit_intuitiveValue_2->text().toDouble(), GetDataStorage()->GetNamedNode("pointSetInRealPlaneSag")->GetData());
			CaculateCutPlaneMiss02();
		}
	}
}


void HTONDI::TranslateMinusZ2()
{
	if (current == "") {
		m_Controls.textBrowser_Action->append("Plesase Select Any Object first.");
		return;
	}
	// �� -z �����ƶ�
	double direction[3]{ 0,0,-1 };

	// �����ƶ�
	Translate(direction, m_Controls.lineEdit_intuitiveValue_2->text().toDouble(), GetDataStorage()->GetNamedNode(current)->GetData());

	if (current == "Saw")
	{
		Translate(direction, m_Controls.lineEdit_intuitiveValue_2->text().toDouble(), GetDataStorage()->GetNamedNode("SawLandMarkPointSet")->GetData());
		if (m_cutType == 1)
		{
			// ��Ҫͬʱ�ƶ� ƽ�� ƽ���ϵĵ� �Լ������
			Translate(direction, m_Controls.lineEdit_intuitiveValue_2->text().toDouble(), GetDataStorage()->GetNamedNode("StateCutPlane01")->GetData());
			Translate(direction, m_Controls.lineEdit_intuitiveValue_2->text().toDouble(), GetDataStorage()->GetNamedNode("pointSetInRealPlaneAxial")->GetData());
			CaculateCutPlaneMiss01();
		}
		if (m_cutType == 2)
		{
			// ��Ҫͬʱ�ƶ� ƽ�� ƽ���ϵĵ� �Լ������
			Translate(direction, m_Controls.lineEdit_intuitiveValue_2->text().toDouble(), GetDataStorage()->GetNamedNode("StateCutPlane02")->GetData());
			Translate(direction, m_Controls.lineEdit_intuitiveValue_2->text().toDouble(), GetDataStorage()->GetNamedNode("pointSetInRealPlaneSag")->GetData());
			CaculateCutPlaneMiss02();
		}
	}
}


void HTONDI::TranslatePlusX2()
{
	if (current == "") {
		m_Controls.textBrowser_Action->append("Plesase Select Any Object first.");
		return;
	}
	// �� +x �����ƶ�
	// �ƶ�����
	double direction[3]{ 1,0,0 };

	// �����ƶ�
	Translate(direction, m_Controls.lineEdit_intuitiveValue_2->text().toDouble(), GetDataStorage()->GetNamedNode(current)->GetData());

	if (current == "Saw")
	{
		Translate(direction, m_Controls.lineEdit_intuitiveValue_2->text().toDouble(), GetDataStorage()->GetNamedNode("SawLandMarkPointSet")->GetData());
		if (m_cutType == 1)
		{
			// ��Ҫͬʱ�ƶ� ƽ�� ƽ���ϵĵ� �Լ������
			Translate(direction, m_Controls.lineEdit_intuitiveValue_2->text().toDouble(), GetDataStorage()->GetNamedNode("StateCutPlane01")->GetData());
			Translate(direction, m_Controls.lineEdit_intuitiveValue_2->text().toDouble(), GetDataStorage()->GetNamedNode("pointSetInRealPlaneAxial")->GetData());
			CaculateCutPlaneMiss01();
		}
		if (m_cutType == 2)
		{
			// ��Ҫͬʱ�ƶ� ƽ�� ƽ���ϵĵ� �Լ������
			Translate(direction, m_Controls.lineEdit_intuitiveValue_2->text().toDouble(), GetDataStorage()->GetNamedNode("StateCutPlane02")->GetData());
			Translate(direction, m_Controls.lineEdit_intuitiveValue_2->text().toDouble(), GetDataStorage()->GetNamedNode("pointSetInRealPlaneSag")->GetData());
			CaculateCutPlaneMiss02();
		}
	}
}


void HTONDI::TranslatePlusY2()
{
	if (current == "") {
		m_Controls.textBrowser_Action->append("Plesase Select Any Object first.");
		return;
	}
	// �� +y �����ƶ�
	double direction[3]{ 0,1,0 };

	// �����ƶ�
	Translate(direction, m_Controls.lineEdit_intuitiveValue_2->text().toDouble(), GetDataStorage()->GetNamedNode(current)->GetData());

	if (current == "Saw")
	{
		Translate(direction, m_Controls.lineEdit_intuitiveValue_2->text().toDouble(), GetDataStorage()->GetNamedNode("SawLandMarkPointSet")->GetData());
		if (m_cutType == 1)
		{
			// ��Ҫͬʱ�ƶ� ƽ�� ƽ���ϵĵ� �Լ������
			Translate(direction, m_Controls.lineEdit_intuitiveValue_2->text().toDouble(), GetDataStorage()->GetNamedNode("StateCutPlane01")->GetData());
			Translate(direction, m_Controls.lineEdit_intuitiveValue_2->text().toDouble(), GetDataStorage()->GetNamedNode("pointSetInRealPlaneAxial")->GetData());
			CaculateCutPlaneMiss01();
		}
		if (m_cutType == 2)
		{
			// ��Ҫͬʱ�ƶ� ƽ�� ƽ���ϵĵ� �Լ������
			Translate(direction, m_Controls.lineEdit_intuitiveValue_2->text().toDouble(), GetDataStorage()->GetNamedNode("StateCutPlane02")->GetData());
			Translate(direction, m_Controls.lineEdit_intuitiveValue_2->text().toDouble(), GetDataStorage()->GetNamedNode("pointSetInRealPlaneSag")->GetData());
			CaculateCutPlaneMiss02();
		}
	}
}


void HTONDI::TranslatePlusZ2()
{
	if (current == "") {
		m_Controls.textBrowser_Action->append("Plesase Select Any Object first.");
		return;
	}
	// �� +z �����ƶ�
	double direction[3]{ 0,0,1 };

	// �����ƶ�
	Translate(direction, m_Controls.lineEdit_intuitiveValue_2->text().toDouble(), GetDataStorage()->GetNamedNode(current)->GetData());

	if (current == "Saw")
	{
		Translate(direction, m_Controls.lineEdit_intuitiveValue_2->text().toDouble(), GetDataStorage()->GetNamedNode("SawLandMarkPointSet")->GetData());
		if (m_cutType == 1)
		{
			// ��Ҫͬʱ�ƶ� ƽ�� ƽ���ϵĵ� �Լ������
			Translate(direction, m_Controls.lineEdit_intuitiveValue_2->text().toDouble(), GetDataStorage()->GetNamedNode("StateCutPlane01")->GetData());
			Translate(direction, m_Controls.lineEdit_intuitiveValue_2->text().toDouble(), GetDataStorage()->GetNamedNode("pointSetInRealPlaneAxial")->GetData());
			CaculateCutPlaneMiss01();
		}
		if (m_cutType == 2)
		{
			// ��Ҫͬʱ�ƶ� ƽ�� ƽ���ϵĵ� �Լ������
			Translate(direction, m_Controls.lineEdit_intuitiveValue_2->text().toDouble(), GetDataStorage()->GetNamedNode("StateCutPlane02")->GetData());
			Translate(direction, m_Controls.lineEdit_intuitiveValue_2->text().toDouble(), GetDataStorage()->GetNamedNode("pointSetInRealPlaneSag")->GetData());
			CaculateCutPlaneMiss02();
		}
	}
}


void HTONDI::RotatePlusX2()
{
	if (current == "") {
		m_Controls.textBrowser_Action->append("Plesase Select Any Object first.");
		return;
	}
	// ���������� +X ��ķ�����ת
	double direction[3]{ 1,0,0 };
	double angle = m_Controls.lineEdit_intuitiveValue_2->text().toDouble();

	// Ӧ����ת
	Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
		direction,
		angle,
		GetDataStorage()->GetNamedNode(current)->GetData());
	if (current == "Saw")
	{
		Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
			direction,
			angle,
			GetDataStorage()->GetNamedNode("SawLandMarkPointSet")->GetData());
		if (m_cutType == 1)
		{
			// ��ʾΪˮƽ�ع�
			cout << "test 02" << endl;
			Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
				direction,
				angle,
				GetDataStorage()->GetNamedNode("StateCutPlane01")->GetData());
			Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
				direction,
				angle,
				GetDataStorage()->GetNamedNode("pointSetInRealPlaneAxial")->GetData());
			cout << "test 03" << endl;
			CaculateCutPlaneMiss01();
		}
		if (m_cutType == 2)
		{
			// ��ʾΪˮƽ�ع�
			cout << "test 02" << endl;
			Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
				direction,
				angle,
				GetDataStorage()->GetNamedNode("StateCutPlane02")->GetData());
			Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
				direction,
				angle,
				GetDataStorage()->GetNamedNode("pointSetInRealPlaneSag")->GetData());
			cout << "test 03" << endl;
			CaculateCutPlaneMiss02();
		}
	}
}


void HTONDI::RotatePlusY2()
{
	if (current == "") {
		m_Controls.textBrowser_Action->append("Plesase Select Any Object first.");
		return;
	}
	// ���������� +Y ��ķ�����ת
	double direction[3]{ 0,1,0 };
	double angle = m_Controls.lineEdit_intuitiveValue_2->text().toDouble();

	// Ӧ����ת
	cout << "test 01" << endl;
	Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
		direction,
		angle,
		GetDataStorage()->GetNamedNode(current)->GetData());
	if (current == "Saw")
	{
		Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
			direction,
			angle,
			GetDataStorage()->GetNamedNode("SawLandMarkPointSet")->GetData());
		if (m_cutType == 1)
		{
			// ��ʾΪˮƽ�ع�
			cout << "test 02" << endl;
			Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
				direction,
				angle,
				GetDataStorage()->GetNamedNode("StateCutPlane01")->GetData());
			Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
				direction,
				angle,
				GetDataStorage()->GetNamedNode("pointSetInRealPlaneAxial")->GetData());
			cout << "test 03" << endl;
			CaculateCutPlaneMiss01();
		}
		if (m_cutType == 2)
		{
			// ��ʾΪˮƽ�ع�
			cout << "test 02" << endl;
			Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
				direction,
				angle,
				GetDataStorage()->GetNamedNode("StateCutPlane02")->GetData());
			Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
				direction,
				angle,
				GetDataStorage()->GetNamedNode("pointSetInRealPlaneSag")->GetData());
			cout << "test 03" << endl;
			CaculateCutPlaneMiss02();
		}
	}
}


void HTONDI::RotatePlusZ2()
{
	if (current == "") {
		m_Controls.textBrowser_Action->append("Plesase Select Any Object first.");
		return;
	}
	// ���������� +Z ��ķ�����ת
	double direction[3]{ 0,0,1 };
	double angle = m_Controls.lineEdit_intuitiveValue_2->text().toDouble();

	// Ӧ����ת
	Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
		direction,
		angle,
		GetDataStorage()->GetNamedNode(current)->GetData());
	if (current == "Saw")
	{
		Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
			direction,
			angle,
			GetDataStorage()->GetNamedNode("SawLandMarkPointSet")->GetData());
		if (m_cutType == 1)
		{
			// ��ʾΪˮƽ�ع�
			cout << "test 02" << endl;
			Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
				direction,
				angle,
				GetDataStorage()->GetNamedNode("StateCutPlane01")->GetData());
			Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
				direction,
				angle,
				GetDataStorage()->GetNamedNode("pointSetInRealPlaneAxial")->GetData());
			cout << "test 03" << endl;
			CaculateCutPlaneMiss01();
		}
		if (m_cutType == 2)
		{
			// ��ʾΪˮƽ�ع�
			cout << "test 02" << endl;
			Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
				direction,
				angle,
				GetDataStorage()->GetNamedNode("StateCutPlane02")->GetData());
			Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
				direction,
				angle,
				GetDataStorage()->GetNamedNode("pointSetInRealPlaneSag")->GetData());
			cout << "test 03" << endl;
			CaculateCutPlaneMiss02();
		}
	}
}


void HTONDI::RotateMinusX2()
{
	if (current == "") {
		m_Controls.textBrowser_Action->append("Plesase Select Any Object first.");
		return;
	}
	// ���������� -X ��ķ�����ת
	double direction[3]{ 1,0,0 };
	double angle = -m_Controls.lineEdit_intuitiveValue_2->text().toDouble();

	// Ӧ����ת
	Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
		direction,
		angle,
		GetDataStorage()->GetNamedNode(current)->GetData());
	if (current == "Saw")
	{
		Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
			direction,
			angle,
			GetDataStorage()->GetNamedNode("SawLandMarkPointSet")->GetData());
		if (m_cutType == 1)
		{
			// ��ʾΪˮƽ�ع�
			cout << "test 02" << endl;
			Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
				direction,
				angle,
				GetDataStorage()->GetNamedNode("StateCutPlane01")->GetData());
			Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
				direction,
				angle,
				GetDataStorage()->GetNamedNode("pointSetInRealPlaneAxial")->GetData());
			cout << "test 03" << endl;
			CaculateCutPlaneMiss01();
		}
		if (m_cutType == 2)
		{
			// ��ʾΪˮƽ�ع�
			cout << "test 02" << endl;
			Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
				direction,
				angle,
				GetDataStorage()->GetNamedNode("StateCutPlane02")->GetData());
			Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
				direction,
				angle,
				GetDataStorage()->GetNamedNode("pointSetInRealPlaneSag")->GetData());
			cout << "test 03" << endl;
			CaculateCutPlaneMiss02();
		}
	}
}


void HTONDI::RotateMinusY2()
{
	if (current == "") {
		m_Controls.textBrowser_Action->append("Plesase Select Any Object first.");
		return;
	}
	// ���������� -Y ��ķ�����ת
	double direction[3]{ 0,1,0 };
	double angle = -m_Controls.lineEdit_intuitiveValue_2->text().toDouble();

	// Ӧ����ת
	Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
		direction,
		angle,
		GetDataStorage()->GetNamedNode(current)->GetData());
	if (current == "Saw")
	{
		Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
			direction,
			angle,
			GetDataStorage()->GetNamedNode("SawLandMarkPointSet")->GetData());
		if (m_cutType == 1)
		{
			// ��ʾΪˮƽ�ع�
			cout << "test 02" << endl;
			Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
				direction,
				angle,
				GetDataStorage()->GetNamedNode("StateCutPlane01")->GetData());
			Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
				direction,
				angle,
				GetDataStorage()->GetNamedNode("pointSetInRealPlaneAxial")->GetData());
			cout << "test 03" << endl;
			CaculateCutPlaneMiss01();
		}
		if (m_cutType == 2)
		{
			// ��ʾΪˮƽ�ع�
			cout << "test 02" << endl;
			Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
				direction,
				angle,
				GetDataStorage()->GetNamedNode("StateCutPlane02")->GetData());
			Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
				direction,
				angle,
				GetDataStorage()->GetNamedNode("pointSetInRealPlaneSag")->GetData());
			cout << "test 03" << endl;
			CaculateCutPlaneMiss02();
		}
	}
}


void HTONDI::RotateMinusZ2()
{
	if (current == "") {
		m_Controls.textBrowser_Action->append("Plesase Select Any Object first.");
		return;
	}
	// ���������� -Z ��ķ�����ת
	double direction[3]{ 0,0,1 };
	double angle = -m_Controls.lineEdit_intuitiveValue_2->text().toDouble();

	// Ӧ����ת
	Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
		direction,
		angle,
		GetDataStorage()->GetNamedNode(current)->GetData());
	if (current == "Saw")
	{
		Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
			direction,
			angle,
			GetDataStorage()->GetNamedNode("SawLandMarkPointSet")->GetData());
		if (m_cutType == 1)
		{
			// ��ʾΪˮƽ�ع�
			cout << "test 02" << endl;
			Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
				direction,
				angle,
				GetDataStorage()->GetNamedNode("StateCutPlane01")->GetData());
			Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
				direction,
				angle,
				GetDataStorage()->GetNamedNode("pointSetInRealPlaneAxial")->GetData());
			cout << "test 03" << endl;
			CaculateCutPlaneMiss01();
		}
		if (m_cutType == 2)
		{
			// ��ʾΪˮƽ�ع�
			cout << "test 02" << endl;
			Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
				direction,
				angle,
				GetDataStorage()->GetNamedNode("StateCutPlane02")->GetData());
			Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
				direction,
				angle,
				GetDataStorage()->GetNamedNode("pointSetInRealPlaneSag")->GetData());
			cout << "test 03" << endl;
			CaculateCutPlaneMiss02();
		}
	}
}