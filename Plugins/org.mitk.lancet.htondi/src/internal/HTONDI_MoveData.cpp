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
== 术前规划数据移动
== 术中导航数据移动
===============================================================*/


void HTONDI::OnSelectionChanged(berry::IWorkbenchPart::Pointer source, const QList<mitk::DataNode::Pointer>& nodes)
{
	// 选中需要移动的物体
	// 当用户在界面上选择不同的数据节点时，更新成员变量
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
				// 将选中的内容输出到面板上
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
	Input：
	 1. 旋转中心
	 2. 旋转轴方向
	 3. 旋转角度
	 4. 需要旋转的物体
	*/
	if (data)
	{
		// 标准化
		double normalizedDirection[3];
		double directionLength = sqrt((pow(direction[0], 2) + pow(direction[1], 2) + pow(direction[2], 2)));
		normalizedDirection[0] = direction[0] / directionLength;
		normalizedDirection[1] = direction[1] / directionLength;
		normalizedDirection[2] = direction[2] / directionLength;

		// 赋值旋转中心 和 旋转轴方向
		mitk::Point3D rotateCenter{ center };
		mitk::Vector3D rotateAxis{ normalizedDirection };

		// 构造运动实例, counterclockwiseDegree 表示旋转角度
		auto* doOp = new mitk::RotationOperation(mitk::OpROTATE, rotateCenter, rotateAxis, counterclockwiseDegree);

		// 应用运动实例
		data->GetGeometry()->ExecuteOperation(doOp);

		// 删除运动实例
		delete doOp;

		if (current == "1st cut plane")
		{
			// 提取水平截骨面的法向量
			auto cutPlaneSource01 = dynamic_cast<mitk::Surface*>(GetDataStorage()->GetNamedNode("1st cut plane")->GetData());
			auto cutPlanePolyData01 = cutPlaneSource01->GetVtkPolyData();
			double centerPlane01[3], normalPlane01[3];

			// 取得第一截骨面当前的法向量和平面中心
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

		// 更新窗口
		data->Update();
		mitk::RenderingManager::GetInstance()->RequestUpdateAll();
	}
}


void HTONDI::Translate(double direction[3], double length, mitk::BaseData* data)
{
	// Apply Translate to Object
	/*
	Input：
	 1. 平移方向
	 2. 平移距离
	 3. 需要移动物体
	*/
	if (data != nullptr)
	{
		// 标准化
		double directionLength = sqrt((pow(direction[0], 2) + pow(direction[1], 2) + pow(direction[2], 2)));

		// 赋值平移向量
		mitk::Point3D movementVector;
		movementVector[0] = length * direction[0] / directionLength;
		movementVector[1] = length * direction[1] / directionLength;
		movementVector[2] = length * direction[2] / directionLength;

		// 构造运动实例
		auto* doOp = new mitk::PointOperation(mitk::OpMOVE, 0, movementVector, 0);

		// 应用运动实例
		data->GetGeometry()->ExecuteOperation(doOp);

		// 删除运动实例
		delete doOp;

		if (current == "1st cut plane")
		{
			//// 提取水平截骨面的法向量
			//auto cutPlaneSource01 = dynamic_cast<mitk::Surface*>(GetDataStorage()->GetNamedNode("1st cut plane")->GetData());
			//auto cutPlanePolyData01 = cutPlaneSource01->GetVtkPolyData();
			//double centerPlane01[3], normalPlane01[3];

			//// 取得第一截骨面当前的法向量和平面中心
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

		// 更新窗口
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
	// 向 -x 方向移动
	double direction[3]{ -1,0,0 };

	// 采用移动
	Translate(direction, m_Controls.lineEdit_intuitiveValue->text().toDouble(), GetDataStorage()->GetNamedNode(current)->GetData());
	if (current == "tibiaSurface") 
	{
		Translate(direction, m_Controls.lineEdit_intuitiveValue->text().toDouble(), GetDataStorage()->GetNamedNode("tibiaLandmarkPointSet")->GetData());
		Translate(direction, m_Controls.lineEdit_intuitiveValue->text().toDouble(), GetDataStorage()->GetNamedNode("tibiaICPPointSet")->GetData());
		OnShowMachineLineClicked();
	}
	if (current == "1st cut plane")
	{
		// 平面上点
		Translate(direction, m_Controls.lineEdit_intuitiveValue->text().toDouble(), GetDataStorage()->GetNamedNode("pointSetInPlaneCutPlane1")->GetData());
		// 然后，计算实时的截骨线
		if (GetIntersectionLine())
		{
			GetDistancefromTibia();
		}
	}
	if (current == "SteelPlate")
	{
		// 同时移动表面点
		Translate(direction, m_Controls.lineEdit_intuitiveValue->text().toDouble(), GetDataStorage()->GetNamedNode("SteelPlatePointSet")->GetData());
	}

}


void HTONDI::TranslateMinusY()
{
	if (current == "") {
		m_Controls.textBrowser_Action->append("Plesase Select Any Object first.");
		return;
	}
	
	// 向 -y 方向移动
	double direction[3]{ 0,-1,0 };

	// 采用移动
	Translate(direction, m_Controls.lineEdit_intuitiveValue->text().toDouble(), GetDataStorage()->GetNamedNode(current)->GetData());
	if (current == "tibiaSurface")
	{
		Translate(direction, m_Controls.lineEdit_intuitiveValue->text().toDouble(), GetDataStorage()->GetNamedNode("tibiaLandmarkPointSet")->GetData());
		Translate(direction, m_Controls.lineEdit_intuitiveValue->text().toDouble(), GetDataStorage()->GetNamedNode("tibiaICPPointSet")->GetData());
		OnShowMachineLineClicked();
	}
	if (current == "1st cut plane")
	{
		// 平面上点
		Translate(direction, m_Controls.lineEdit_intuitiveValue->text().toDouble(), GetDataStorage()->GetNamedNode("pointSetInPlaneCutPlane1")->GetData());
		// 然后，计算实时的截骨线
		if (GetIntersectionLine())
		{
			GetDistancefromTibia();
		}
	}
	if (current == "SteelPlate")
	{
		// 同时移动表面点
		Translate(direction, m_Controls.lineEdit_intuitiveValue->text().toDouble(), GetDataStorage()->GetNamedNode("SteelPlatePointSet")->GetData());


	}
}


void HTONDI::TranslateMinusZ()
{
	if (current == "") {
		m_Controls.textBrowser_Action->append("Plesase Select Any Object first.");
		return;
	}
	// 向 -z 方向移动
	double direction[3]{ 0,0,-1 };

	// 采用移动
	Translate(direction, m_Controls.lineEdit_intuitiveValue->text().toDouble(), GetDataStorage()->GetNamedNode(current)->GetData());
	if (current == "tibiaSurface")
	{
		Translate(direction, m_Controls.lineEdit_intuitiveValue->text().toDouble(), GetDataStorage()->GetNamedNode("tibiaLandmarkPointSet")->GetData());
		Translate(direction, m_Controls.lineEdit_intuitiveValue->text().toDouble(), GetDataStorage()->GetNamedNode("tibiaICPPointSet")->GetData());
		OnShowMachineLineClicked();
	}
	if (current == "1st cut plane")
	{
		// 平面上点
		Translate(direction, m_Controls.lineEdit_intuitiveValue->text().toDouble(), GetDataStorage()->GetNamedNode("pointSetInPlaneCutPlane1")->GetData());
		// 然后，计算实时的截骨线
		if (GetIntersectionLine())
		{
			GetDistancefromTibia();
		}
	}
	if (current == "SteelPlate")
	{
		// 同时移动表面点
		Translate(direction, m_Controls.lineEdit_intuitiveValue->text().toDouble(), GetDataStorage()->GetNamedNode("SteelPlatePointSet")->GetData());


	}
}


void HTONDI::TranslatePlusX()
{
	if (current == "") {
		m_Controls.textBrowser_Action->append("Plesase Select Any Object first.");
		return;
	}
	// 向 +x 方向移动
	// 移动方向
	double direction[3]{ 1,0,0 };

	// 采用移动
	Translate(direction, m_Controls.lineEdit_intuitiveValue->text().toDouble(), GetDataStorage()->GetNamedNode(current)->GetData());
	if (current == "tibiaSurface")
	{
		Translate(direction, m_Controls.lineEdit_intuitiveValue->text().toDouble(), GetDataStorage()->GetNamedNode("tibiaLandmarkPointSet")->GetData());
		Translate(direction, m_Controls.lineEdit_intuitiveValue->text().toDouble(), GetDataStorage()->GetNamedNode("tibiaICPPointSet")->GetData());
		OnShowMachineLineClicked();
	}
	if (current == "1st cut plane")
	{
		// 平面上点
		Translate(direction, m_Controls.lineEdit_intuitiveValue->text().toDouble(), GetDataStorage()->GetNamedNode("pointSetInPlaneCutPlane1")->GetData());
		// 然后，计算实时的截骨线
		if (GetIntersectionLine())
		{
			GetDistancefromTibia();
		}
	}
	if (current == "SteelPlate")
	{
		// 同时移动表面点
		Translate(direction, m_Controls.lineEdit_intuitiveValue->text().toDouble(), GetDataStorage()->GetNamedNode("SteelPlatePointSet")->GetData());


	}
}


void HTONDI::TranslatePlusY()
{
	if (current == "") {
		m_Controls.textBrowser_Action->append("Plesase Select Any Object first.");
		return;
	}
	// 向 +y 方向移动
	double direction[3]{ 0,1,0 };

	// 采用移动
	Translate(direction, m_Controls.lineEdit_intuitiveValue->text().toDouble(), GetDataStorage()->GetNamedNode(current)->GetData());
	if (current == "tibiaSurface")
	{
		Translate(direction, m_Controls.lineEdit_intuitiveValue->text().toDouble(), GetDataStorage()->GetNamedNode("tibiaLandmarkPointSet")->GetData());
		Translate(direction, m_Controls.lineEdit_intuitiveValue->text().toDouble(), GetDataStorage()->GetNamedNode("tibiaICPPointSet")->GetData());
		OnShowMachineLineClicked();
	}
	if (current == "1st cut plane")
	{
		// 平面上点
		Translate(direction, m_Controls.lineEdit_intuitiveValue->text().toDouble(), GetDataStorage()->GetNamedNode("pointSetInPlaneCutPlane1")->GetData());
		// 然后，计算实时的截骨线
		if (GetIntersectionLine())
		{
			GetDistancefromTibia();
		}
	}
	if (current == "SteelPlate")
	{
		// 同时移动表面点
		Translate(direction, m_Controls.lineEdit_intuitiveValue->text().toDouble(), GetDataStorage()->GetNamedNode("SteelPlatePointSet")->GetData());


	}
}


void HTONDI::TranslatePlusZ()
{
	if (current == "") {
		m_Controls.textBrowser_Action->append("Plesase Select Any Object first.");
		return;
	}
	// 向 +z 方向移动
	double direction[3]{ 0,0,1 };

	// 采用移动
	Translate(direction, m_Controls.lineEdit_intuitiveValue->text().toDouble(), GetDataStorage()->GetNamedNode(current)->GetData());
	if (current == "tibiaSurface")
	{
		Translate(direction, m_Controls.lineEdit_intuitiveValue->text().toDouble(), GetDataStorage()->GetNamedNode("tibiaLandmarkPointSet")->GetData());
		Translate(direction, m_Controls.lineEdit_intuitiveValue->text().toDouble(), GetDataStorage()->GetNamedNode("tibiaICPPointSet")->GetData());
		OnShowMachineLineClicked();
	}
	if (current == "1st cut plane")
	{
		// 平面上点
		Translate(direction, m_Controls.lineEdit_intuitiveValue->text().toDouble(), GetDataStorage()->GetNamedNode("pointSetInPlaneCutPlane1")->GetData());
		// 然后，计算实时的截骨线
		if (GetIntersectionLine())
		{
			GetDistancefromTibia();
		}
	}
	if (current == "SteelPlate")
	{
		// 同时移动表面点
		Translate(direction, m_Controls.lineEdit_intuitiveValue->text().toDouble(), GetDataStorage()->GetNamedNode("SteelPlatePointSet")->GetData());
	}
}


void HTONDI::RotatePlusX()
{
	if (current == "") {
		m_Controls.textBrowser_Action->append("Plesase Select Any Object first.");
		return;
	}
	// 向物体中心 +X 轴的方向旋转
	double direction[3]{ 1,0,0 };
	double angle = m_Controls.lineEdit_intuitiveValue->text().toDouble();

	// 应用旋转
	Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(), 
		direction, 
		angle, 
		GetDataStorage()->GetNamedNode(current)->GetData());
	if(current == "tibiaSurface")
	{
		// 旋转胫骨上点
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
		// 平面上点
		Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
			direction,
			angle,
			GetDataStorage()->GetNamedNode("pointSetInPlaneCutPlane1")->GetData());
		// 然后，计算实时的截骨线
		if (GetIntersectionLine())
		{
			GetDistancefromTibia();
		}
	}
	if (current == "SteelPlate")
	{
		// 同时移动表面点
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
	// 向物体中心 +Y 轴的方向旋转
	double direction[3]{ 0,1,0 };
	double angle = m_Controls.lineEdit_intuitiveValue->text().toDouble();
	// 应用旋转
	Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
		direction,
		angle,
		GetDataStorage()->GetNamedNode(current)->GetData());
	if (current == "tibiaSurface")
	{
		// 旋转胫骨上点
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
		// 平面上点
		Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
			direction,
			angle,
			GetDataStorage()->GetNamedNode("pointSetInPlaneCutPlane1")->GetData());
		// 然后，计算实时的截骨线
		if (GetIntersectionLine())
		{
			GetDistancefromTibia();
		}
	}
	if (current == "SteelPlate")
	{
		// 同时移动表面点
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
	// 向物体中心 +Z 轴的方向旋转
	double direction[3]{ 0,0,1 };
	double angle = m_Controls.lineEdit_intuitiveValue->text().toDouble();
	// 应用旋转
	Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
		direction,
		angle,
		GetDataStorage()->GetNamedNode(current)->GetData());
	if (current == "tibiaSurface")
	{
		// 旋转胫骨上点
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
		// 平面上点
		Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
			direction,
			angle,
			GetDataStorage()->GetNamedNode("pointSetInPlaneCutPlane1")->GetData());
		// 然后，计算实时的截骨线
		if (GetIntersectionLine())
		{
			GetDistancefromTibia();
		}
	}
	if (current == "SteelPlate")
	{
		// 同时移动表面点
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
	// 向物体中心 -X 轴的方向旋转
	double direction[3]{ 1,0,0 };
	double angle = -m_Controls.lineEdit_intuitiveValue->text().toDouble();
	// 应用旋转
	Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
		direction,
		angle,
		GetDataStorage()->GetNamedNode(current)->GetData());
	if (current == "tibiaSurface")
	{
		// 旋转胫骨上点
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
		// 平面上点
		Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
			direction,
			angle,
			GetDataStorage()->GetNamedNode("pointSetInPlaneCutPlane1")->GetData());
		// 然后，计算实时的截骨线
		if (GetIntersectionLine())
		{
			GetDistancefromTibia();
		}
	}
	if (current == "SteelPlate")
	{
		// 同时移动表面点
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
	// 向物体中心 -Y 轴的方向旋转
	double direction[3]{ 0,1,0 };
	double angle = -m_Controls.lineEdit_intuitiveValue->text().toDouble();
	// 应用旋转
	Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
		direction,
		angle,
		GetDataStorage()->GetNamedNode(current)->GetData());
	if (current == "tibiaSurface")
	{
		// 旋转胫骨上点
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
		// 在进行附着属性的更迭的时候，需要确保属性存在
		// 确定属性 => 计算力线需要的 "股骨头中心" 和 "踝关节中心"
		// 平面上点
		Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
			direction,
			angle,
			GetDataStorage()->GetNamedNode("pointSetInPlaneCutPlane1")->GetData());
		// 然后，计算实时的截骨线
		if (GetIntersectionLine())
		{
			GetDistancefromTibia();
		}
	}
	if (current == "SteelPlate")
	{
		// 同时移动表面点
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
	// 向物体中心 -Z 轴的方向旋转
	double direction[3]{ 0,0,1 };
	double angle = -m_Controls.lineEdit_intuitiveValue->text().toDouble();
	// 应用旋转
	Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
		direction,
		angle,
		GetDataStorage()->GetNamedNode(current)->GetData());
	if (current == "tibiaSurface")
	{
		// 旋转胫骨上点
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
		// 平面上点
		Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
			direction,
			angle,
			GetDataStorage()->GetNamedNode("pointSetInPlaneCutPlane1")->GetData());
		// 然后，计算实时的截骨线
		if (GetIntersectionLine())
		{
			GetDistancefromTibia();
		}
	}
	if (current == "SteelPlate")
	{
		// 同时移动表面点
		Rotate(GetDataStorage()->GetNamedNode(current)->GetData()->GetGeometry()->GetCenter().GetDataPointer(),
			direction,
			angle,
			GetDataStorage()->GetNamedNode("SteelPlatePointSet")->GetData());
	}
}

void HTONDI::RotateMinus()
{
	// 对截骨远端进行旋转，绕合页点轴向逆时针
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

	// 对于左腿右腿，设计不同的旋转轴
	// 0:右腿. 1：左腿
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
	

	// 应用旋转
	Rotate(center,direction_cutPlane,angle,distalTibiaSurface->GetData());
	// 同时移动胫骨上的远端点（仅更新第三和第四个点）
	// 取出远端的两个点，单独构建点集
	auto tibiaLandmarkNode = GetDataStorage()->GetNamedNode("tibiaLandmarkPointSet");
	if (tibiaLandmarkNode == nullptr) 
	{
		m_Controls.textBrowser_Action->append("tibiaLandmarkPointSet Not Found!");
		return;
	}
	
	// 创建一个新的点集并设置采集到的点
	mitk::PointSet::Pointer landmarkPointSet = dynamic_cast<mitk::PointSet*>(tibiaLandmarkNode->GetData());
	mitk::PointSet::Pointer tibiaDistalPoints = mitk::PointSet::New();
	tibiaDistalPoints->InsertPoint(0, landmarkPointSet->GetPoint(2));
	tibiaDistalPoints->InsertPoint(1, landmarkPointSet->GetPoint(3));
	// 创建一个新的数据节点并设置点集
	// 如果已经创建则直接更新数据
	auto tibiaDistal = GetDataStorage()->GetNamedNode("tibiaDistalPointSet");
	if (tibiaDistal != nullptr)
	{
		m_Controls.textBrowser_Action->append("Refresh tibiaDistalPointSet");
		// 应用旋转
		Rotate(center, direction_cutPlane, angle, tibiaDistal->GetData());
		// 更新窗口
		mitk::RenderingManager::GetInstance()->RequestUpdateAll();

	}
	else
	{
		// 否则，新建一个对象
		m_Controls.textBrowser_Action->append("Create new PointSet");
		mitk::DataNode::Pointer tibiaDistalPointSet = mitk::DataNode::New();
		tibiaDistalPointSet->SetData(tibiaDistalPoints);
		tibiaDistalPointSet->SetName("tibiaDistalPointSet");
		// 设置节点颜色为红色
		tibiaDistalPointSet->SetColor(0.0, 0.0, 1.0); // RGB: Blue
		tibiaDistalPointSet->SetFloatProperty("pointsize", 5);
		GetDataStorage()->Add(tibiaDistalPointSet);
		// 应用旋转
		Rotate(center, direction_cutPlane, angle, tibiaDistalPointSet->GetData());
	}
	// 然后，计算一条新的力线
	OnShowMachineLine02Clicked();
	
	// 更新窗口
	mitk::RenderingManager::GetInstance()->RequestUpdateAll();
}


void HTONDI::RotatePlus()
{
	// 首先需要判别合页点是哪两个
	// 左腿：
	// 对截骨远端进行旋转，绕合页点轴向顺时针
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

	// 对于左腿右腿，设计不同的旋转轴
	// 0:右腿. 1：左腿
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

	// 应用旋转
	Rotate(center, direction_cutPlane, angle, distalTibiaSurface->GetData());
	// 同时移动胫骨上的远端点（仅更新第三和第四个点）
	// 取出远端的两个点，单独构建点集
	auto tibiaLandmarkNode = GetDataStorage()->GetNamedNode("tibiaLandmarkPointSet");
	if (tibiaLandmarkNode == nullptr)
	{
		m_Controls.textBrowser_Action->append("tibiaLandmarkPointSet Not Found!");
		return;
	}

	// 创建一个新的点集并设置采集到的点
	mitk::PointSet::Pointer landmarkPointSet = dynamic_cast<mitk::PointSet*>(tibiaLandmarkNode->GetData());
	mitk::PointSet::Pointer tibiaDistalPoints = mitk::PointSet::New();
	tibiaDistalPoints->InsertPoint(0, landmarkPointSet->GetPoint(2));
	tibiaDistalPoints->InsertPoint(1, landmarkPointSet->GetPoint(3));
	// 创建一个新的数据节点并设置点集
	// 如果已经创建则直接更新数据
	auto tibiaDistal = GetDataStorage()->GetNamedNode("tibiaDistalPointSet");
	if (tibiaDistal != nullptr)
	{
		m_Controls.textBrowser_Action->append("Refresh tibiaDistalPointSet");
		// 应用旋转
		Rotate(center, direction_cutPlane, angle, tibiaDistal->GetData());
		// 更新窗口
		mitk::RenderingManager::GetInstance()->RequestUpdateAll();
	}
	else
	{
		// 否则，新建一个对象
		m_Controls.textBrowser_Action->append("Create new PointSet");
		mitk::DataNode::Pointer tibiaDistalPointSet = mitk::DataNode::New();
		tibiaDistalPointSet->SetData(tibiaDistalPoints);
		tibiaDistalPointSet->SetName("tibiaDistalPointSet");
		// 设置节点颜色为红色
		tibiaDistalPointSet->SetColor(0.0, 0.0, 1.0); // RGB: Blue
		tibiaDistalPointSet->SetFloatProperty("pointsize", 5);
		GetDataStorage()->Add(tibiaDistalPointSet);
		// 应用旋转
		Rotate(center, direction_cutPlane, angle, tibiaDistalPointSet->GetData());
	}
	// 然后，计算一条新的力线
	OnShowMachineLine02Clicked();

	// 更新窗口
	mitk::RenderingManager::GetInstance()->RequestUpdateAll();
}


void HTONDI::TranslateMinusX2()
{
	if (current == "") {
		m_Controls.textBrowser_Action->append("Plesase Select Any Object first.");
		return;
	}
	// 向 -x 方向移动
	double direction[3]{ -1,0,0 };

	// 采用移动
	Translate(direction, m_Controls.lineEdit_intuitiveValue_2->text().toDouble(), GetDataStorage()->GetNamedNode(current)->GetData());

	if (current == "Saw")
	{
		Translate(direction, m_Controls.lineEdit_intuitiveValue_2->text().toDouble(), GetDataStorage()->GetNamedNode("SawLandMarkPointSet")->GetData());
		if (m_cutType == 1)
		{
			// 需要同时移动 平面 平面上的点 以及表面点
			Translate(direction, m_Controls.lineEdit_intuitiveValue_2->text().toDouble(), GetDataStorage()->GetNamedNode("StateCutPlane01")->GetData());
			Translate(direction, m_Controls.lineEdit_intuitiveValue_2->text().toDouble(), GetDataStorage()->GetNamedNode("pointSetInRealPlaneAxial")->GetData());
			CaculateCutPlaneMiss01();
		}
		if (m_cutType == 2)
		{
			// 需要同时移动 平面 平面上的点 以及表面点
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

	// 向 -y 方向移动
	double direction[3]{ 0,-1,0 };

	// 采用移动
	Translate(direction, m_Controls.lineEdit_intuitiveValue_2->text().toDouble(), GetDataStorage()->GetNamedNode(current)->GetData());

	if (current == "Saw")
	{
		Translate(direction, m_Controls.lineEdit_intuitiveValue_2->text().toDouble(), GetDataStorage()->GetNamedNode("SawLandMarkPointSet")->GetData());
		if (m_cutType == 1)
		{
			// 需要同时移动 平面 平面上的点 以及表面点
			Translate(direction, m_Controls.lineEdit_intuitiveValue_2->text().toDouble(), GetDataStorage()->GetNamedNode("StateCutPlane01")->GetData());
			Translate(direction, m_Controls.lineEdit_intuitiveValue_2->text().toDouble(), GetDataStorage()->GetNamedNode("pointSetInRealPlaneAxial")->GetData());
			CaculateCutPlaneMiss01();
		}
		if (m_cutType == 2)
		{
			// 需要同时移动 平面 平面上的点 以及表面点
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
	// 向 -z 方向移动
	double direction[3]{ 0,0,-1 };

	// 采用移动
	Translate(direction, m_Controls.lineEdit_intuitiveValue_2->text().toDouble(), GetDataStorage()->GetNamedNode(current)->GetData());

	if (current == "Saw")
	{
		Translate(direction, m_Controls.lineEdit_intuitiveValue_2->text().toDouble(), GetDataStorage()->GetNamedNode("SawLandMarkPointSet")->GetData());
		if (m_cutType == 1)
		{
			// 需要同时移动 平面 平面上的点 以及表面点
			Translate(direction, m_Controls.lineEdit_intuitiveValue_2->text().toDouble(), GetDataStorage()->GetNamedNode("StateCutPlane01")->GetData());
			Translate(direction, m_Controls.lineEdit_intuitiveValue_2->text().toDouble(), GetDataStorage()->GetNamedNode("pointSetInRealPlaneAxial")->GetData());
			CaculateCutPlaneMiss01();
		}
		if (m_cutType == 2)
		{
			// 需要同时移动 平面 平面上的点 以及表面点
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
	// 向 +x 方向移动
	// 移动方向
	double direction[3]{ 1,0,0 };

	// 采用移动
	Translate(direction, m_Controls.lineEdit_intuitiveValue_2->text().toDouble(), GetDataStorage()->GetNamedNode(current)->GetData());

	if (current == "Saw")
	{
		Translate(direction, m_Controls.lineEdit_intuitiveValue_2->text().toDouble(), GetDataStorage()->GetNamedNode("SawLandMarkPointSet")->GetData());
		if (m_cutType == 1)
		{
			// 需要同时移动 平面 平面上的点 以及表面点
			Translate(direction, m_Controls.lineEdit_intuitiveValue_2->text().toDouble(), GetDataStorage()->GetNamedNode("StateCutPlane01")->GetData());
			Translate(direction, m_Controls.lineEdit_intuitiveValue_2->text().toDouble(), GetDataStorage()->GetNamedNode("pointSetInRealPlaneAxial")->GetData());
			CaculateCutPlaneMiss01();
		}
		if (m_cutType == 2)
		{
			// 需要同时移动 平面 平面上的点 以及表面点
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
	// 向 +y 方向移动
	double direction[3]{ 0,1,0 };

	// 采用移动
	Translate(direction, m_Controls.lineEdit_intuitiveValue_2->text().toDouble(), GetDataStorage()->GetNamedNode(current)->GetData());

	if (current == "Saw")
	{
		Translate(direction, m_Controls.lineEdit_intuitiveValue_2->text().toDouble(), GetDataStorage()->GetNamedNode("SawLandMarkPointSet")->GetData());
		if (m_cutType == 1)
		{
			// 需要同时移动 平面 平面上的点 以及表面点
			Translate(direction, m_Controls.lineEdit_intuitiveValue_2->text().toDouble(), GetDataStorage()->GetNamedNode("StateCutPlane01")->GetData());
			Translate(direction, m_Controls.lineEdit_intuitiveValue_2->text().toDouble(), GetDataStorage()->GetNamedNode("pointSetInRealPlaneAxial")->GetData());
			CaculateCutPlaneMiss01();
		}
		if (m_cutType == 2)
		{
			// 需要同时移动 平面 平面上的点 以及表面点
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
	// 向 +z 方向移动
	double direction[3]{ 0,0,1 };

	// 采用移动
	Translate(direction, m_Controls.lineEdit_intuitiveValue_2->text().toDouble(), GetDataStorage()->GetNamedNode(current)->GetData());

	if (current == "Saw")
	{
		Translate(direction, m_Controls.lineEdit_intuitiveValue_2->text().toDouble(), GetDataStorage()->GetNamedNode("SawLandMarkPointSet")->GetData());
		if (m_cutType == 1)
		{
			// 需要同时移动 平面 平面上的点 以及表面点
			Translate(direction, m_Controls.lineEdit_intuitiveValue_2->text().toDouble(), GetDataStorage()->GetNamedNode("StateCutPlane01")->GetData());
			Translate(direction, m_Controls.lineEdit_intuitiveValue_2->text().toDouble(), GetDataStorage()->GetNamedNode("pointSetInRealPlaneAxial")->GetData());
			CaculateCutPlaneMiss01();
		}
		if (m_cutType == 2)
		{
			// 需要同时移动 平面 平面上的点 以及表面点
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
	// 向物体中心 +X 轴的方向旋转
	double direction[3]{ 1,0,0 };
	double angle = m_Controls.lineEdit_intuitiveValue_2->text().toDouble();

	// 应用旋转
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
			// 表示为水平截骨
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
			// 表示为水平截骨
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
	// 向物体中心 +Y 轴的方向旋转
	double direction[3]{ 0,1,0 };
	double angle = m_Controls.lineEdit_intuitiveValue_2->text().toDouble();

	// 应用旋转
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
			// 表示为水平截骨
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
			// 表示为水平截骨
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
	// 向物体中心 +Z 轴的方向旋转
	double direction[3]{ 0,0,1 };
	double angle = m_Controls.lineEdit_intuitiveValue_2->text().toDouble();

	// 应用旋转
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
			// 表示为水平截骨
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
			// 表示为水平截骨
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
	// 向物体中心 -X 轴的方向旋转
	double direction[3]{ 1,0,0 };
	double angle = -m_Controls.lineEdit_intuitiveValue_2->text().toDouble();

	// 应用旋转
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
			// 表示为水平截骨
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
			// 表示为水平截骨
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
	// 向物体中心 -Y 轴的方向旋转
	double direction[3]{ 0,1,0 };
	double angle = -m_Controls.lineEdit_intuitiveValue_2->text().toDouble();

	// 应用旋转
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
			// 表示为水平截骨
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
			// 表示为水平截骨
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
	// 向物体中心 -Z 轴的方向旋转
	double direction[3]{ 0,0,1 };
	double angle = -m_Controls.lineEdit_intuitiveValue_2->text().toDouble();

	// 应用旋转
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
			// 表示为水平截骨
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
			// 表示为水平截骨
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