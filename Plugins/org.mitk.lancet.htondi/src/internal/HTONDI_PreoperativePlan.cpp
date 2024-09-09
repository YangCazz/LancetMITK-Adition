/*============================================================================
Robot-assisted Surgical Navigation Extension based on MITK
Copyright (c) Hangzhou LancetRobotics,CHINA
All rights reserved.
============================================================================*/

// Blueberry
#include <berryISelectionService.h>
#include <berryIWorkbenchWindow.h>

// Qmitk
#include "HTONDI.h"

// Qt
#include <QTimer>
#include <QMessageBox>
#include <QtWidgets/QFileDialog>
#include <QtCore\qmath.h>

// mitk
#include <ep/include/vtk-9.1/vtkTransformFilter.h>
#include <mitkPointSet.h>
#include <mitkRemeshing.h>
#include "mitkVtkInterpolationProperty.h"

// vtk
#include <vtkPolyDataPlaneClipper.h>
#include <vtkFillHolesFilter.h>
#include <vtkAppendPolyData.h>
#include <vtkCleanPolyData.h>
#include <vtkClipPolyData.h>
#include "vtkOBBTree.h"
#include <vtkPlaneSource.h>
#include <vtkCutter.h>

/*========================= 术前规划 ==================================
HTONDI_PreoperativePlan.cpp
----------------------------------------------------------------
== 1.校验数据准备
== 截骨面规划
== 下肢力线规划
== 撑开角度规划
== 钢板规划
== 截骨效应
=====================================================================*/



// 1.校验数据准备
bool HTONDI::OnCheckBaseDataClicked()
{
	// 校验调试过程的数据完备性
	m_Controls.textBrowser_Action->append("Action_01: Check Base data.");
	auto femurLandmark = GetDataStorage()->GetNamedNode("femurLandmarkPointSet");
	auto tibiaLandmark = GetDataStorage()->GetNamedNode("tibiaLandmarkPointSet");
	auto femurHeadPoints = GetDataStorage()->GetNamedNode("femoralHead");
	auto femur = GetDataStorage()->GetNamedNode("femurSurface");
	auto tibia = GetDataStorage()->GetNamedNode("tibiaSurface");
	auto steelPlate = GetDataStorage()->GetNamedNode("SteelPlate");
	auto steelPlatePoints = GetDataStorage()->GetNamedNode("SteelPlatePointSet");
	auto Saw = GetDataStorage()->GetNamedNode("Saw");
	auto SawPoints = GetDataStorage()->GetNamedNode("SawLandMarkPointSet");
	auto Drill = GetDataStorage()->GetNamedNode("Drill");
	auto DrillPoints = GetDataStorage()->GetNamedNode("DrillLandMarkPointSet");
	auto LinkPin = GetDataStorage()->GetNamedNode("LinkPin");
	auto LinkPinLandmark = GetDataStorage()->GetNamedNode("LinkPinLandMarkPointSet");
	auto Probe = GetDataStorage()->GetNamedNode("Probe");
	auto ProbeLandmark = GetDataStorage()->GetNamedNode("ProbeLandMarkPointSet");

	// 确认手术模型的模式
	if (m_Controls.checkBox_LeftlimbHTO->isChecked())
	{
		judgModel_flag = 1;
		m_Controls.textBrowser_Action->append("Set action mode as Left Limb!");
	}
	else if (m_Controls.checkBox_RightlimbHTO->isChecked())
	{
		judgModel_flag = 0;
		m_Controls.textBrowser_Action->append("Set action mode as Right Limb!");
	}

	if (femurLandmark)
	{
		m_Controls.checkBox_point02->setChecked(true);
		m_Controls.checkBox_point03->setChecked(true);
		m_Controls.textBrowser_Action->append("load femurLandmarkPointSet.");
		m_Controls.textBrowser_Action->append("load femurLateralMalleolusPoint.");
		m_Controls.textBrowser_Action->append("load femurMedialMalleolusPoint.");

		SetModelColor(femurLandmark, 1.0f, 0.0f, 0.0f);
		SetNodeSize(femurLandmark, 10.0f);


	}
	if (tibiaLandmark)
	{
		m_Controls.checkBox_point04->setChecked(true);
		m_Controls.checkBox_point05->setChecked(true);
		m_Controls.checkBox_point06->setChecked(true);
		m_Controls.checkBox_point07->setChecked(true);
		m_Controls.textBrowser_Action->append("load tibiaLandmarkPointSet.");
		m_Controls.textBrowser_Action->append("load tibiaProximalLateralPoint.");
		m_Controls.textBrowser_Action->append("load tibiaProximalMedialPoint.");
		m_Controls.textBrowser_Action->append("load tibiaDistalLateralPoint.");
		m_Controls.textBrowser_Action->append("load tibiaDistalMedialPoint.");

		SetModelColor(tibiaLandmark, 1.0f, 0.0f, 0.0f);
		SetNodeSize(tibiaLandmark, 10.0f);

		// 装载tibia的landmark点，这里装载了四个点
		// 胫骨近端外侧点 胫骨近端内侧点 胫骨远端外踝点 胫骨远端内踝点
		cout << "test 02" << endl;
		// femur_image->SetLandmarks(dynamic_cast<mitk::PointSet*>(tibiaLandmark->GetData()));
	}
	if (femurHeadPoints)
	{
		m_Controls.textBrowser_Action->append("load femurHeadPoints.");
		m_Controls.checkBox_point01->setChecked(true);

	}
	if (femur)
	{
		m_Controls.textBrowser_Action->append("load femurSurface.");
		m_Controls.checkBox_point08->setChecked(true);

	}
	if (tibia)
	{
		m_Controls.textBrowser_Action->append("load tibiaSurface.");
		m_Controls.checkBox_point09->setChecked(true);

	}

	// 工具类设置为不可见即可
	if (steelPlate)
	{
		m_Controls.textBrowser_Action->append("load SteelPlate.");
		m_Controls.checkBox_point11->setChecked(true);
		steelPlate->SetVisibility(false);
		// 记录下当前按钢板的初始位置==物体原点坐标
		m_steelPosition = steelPlate->GetData()->GetGeometry()->GetOrigin();

	}
	if (Saw)
	{
		m_Controls.textBrowser_Action->append("load Saw.");
		m_Controls.checkBox_point12->setChecked(true);
		Saw->SetVisibility(false);

	}
	if (Drill)
	{
		m_Controls.textBrowser_Action->append("load Drill.");
		m_Controls.checkBox_point13->setChecked(true);
		Drill->SetVisibility(false);

	}

	// 工具标记点，设置landmark点
	if (steelPlatePoints)
	{
		m_Controls.textBrowser_Action->append("load SteelPlatePointSet.");
		m_Controls.checkBox_point14->setChecked(true);
		steelPlatePoints->SetVisibility(false);
	}
	if (SawPoints)
	{
		m_Controls.textBrowser_Action->append("load SawPoints.");
		m_Controls.checkBox_point15->setChecked(true);
		SawPoints->SetVisibility(false);
	
	}
	if (DrillPoints)
	{
		m_Controls.textBrowser_Action->append("load DrillPointSet.");
		m_Controls.checkBox_point16->setChecked(true);
		DrillPoints->SetVisibility(false);

	}
	if (LinkPin)
	{
		m_Controls.textBrowser_Action->append("load LinkPin.");
		m_Controls.checkBox_point17->setChecked(true);
		LinkPin->SetVisibility(false);

	}
	if (LinkPinLandmark)
	{
		m_Controls.textBrowser_Action->append("load LinkPinLandMarkPointSet.");
		m_Controls.checkBox_point17->setChecked(true);
		LinkPinLandmark->SetVisibility(false);

	}
	if (Probe)
	{
		m_Controls.textBrowser_Action->append("load Probe.");
		m_Controls.checkBox_point18->setChecked(true);
		Probe->SetVisibility(false);

	}
	if (ProbeLandmark)
	{
		m_Controls.textBrowser_Action->append("load ProbeLandmark.");
		m_Controls.checkBox_point19->setChecked(true);
		ProbeLandmark->SetVisibility(false);

	}

	GetDataStorage()->Modified();
	mitk::RenderingManager::GetInstance()->RequestUpdateAll();

	return true;
}

// 截骨面规划
bool HTONDI::OnGenerateCutPlaneClicked()
{
	/* 生成截骨面
	1. 水平截骨面为初始位置
	2. 上升截骨面在水平面规划之后生成

	多次生成时，删除已经生成的平面后再重新生成，或者直接更新数据
	*/
	// 生成截骨平面
	m_Controls.textBrowser_Action->append("Action: Generate Cut Plane.");

	// 1. 水平截骨
	if (m_Controls.cutAxialTibet_radioButton->isChecked())
	{
		if (CreateOneCutPlane())
		{
			// 首先删除同名的平面
			auto cutPlane = GetDataStorage()->GetNamedNode("1st cut plane");
			if (cutPlane) {
				GetDataStorage()->Remove(cutPlane);
			}
			// 创建平面后，设置截骨平面为 绿色 0.5透明度
			auto cutSurfaceNode = GetDataStorage()->GetNamedNode("tibia cut plane");
			cutSurfaceNode->SetColor(0, 1, 0);
			cutSurfaceNode->SetOpacity(0.5);
			cutSurfaceNode->SetName("1st cut plane");
			return true;
		}
	}

	// 2. 上升截骨
	if (m_Controls.cutSagTibet_radioButton->isChecked())
	{
		// 删除同名平面
		auto cutPlane_2 = GetDataStorage()->GetNamedNode("2nd cut plane");
		if (cutPlane_2) {
			GetDataStorage()->Remove(cutPlane_2);
		}


		// 基于当前水平截骨面的规划来生成上升截骨面
		auto cutPlaneNode = GetDataStorage()->GetNamedNode("1st cut plane");
		auto mitkCutPlane_0 = dynamic_cast<mitk::Surface*>(cutPlaneNode->GetData());
		auto tmpVtkSurface_initial = mitkCutPlane_0->GetVtkPolyData();

		vtkNew<vtkTransform> cutPlaneTransform;
		cutPlaneTransform->SetMatrix(mitkCutPlane_0->GetGeometry()->GetVtkMatrix());

		vtkNew<vtkTransformFilter> cutPlaneTransformFilter;
		cutPlaneTransformFilter->SetTransform(cutPlaneTransform);
		cutPlaneTransformFilter->SetInputData(tmpVtkSurface_initial);
		cutPlaneTransformFilter->Update();

		// 创建新平面
		vtkNew<vtkPolyData> tmpVtkSurface;
		tmpVtkSurface->DeepCopy(cutPlaneTransformFilter->GetPolyDataOutput());

		auto cutSurface1 = mitk::Surface::New();
		cutSurface1->SetVtkPolyData(tmpVtkSurface);

		auto planeNode1 = mitk::DataNode::New();
		planeNode1->SetData(cutSurface1);
		planeNode1->SetName("2 cut plane");


		GetDataStorage()->Add(planeNode1);
		auto cutSurfaceNode2 = GetDataStorage()->GetNamedNode("2 cut plane");
		cutSurfaceNode2->SetColor(1, 0, 0);
		cutSurfaceNode2->SetOpacity(0.5);
		cutSurfaceNode2->SetName("2nd cut plane");

		// 进行旋转计算
		double vx, vy, vz, length;
		double angle;
		// 0:右腿. 1：左腿
		// 计算旋转轴单位向量
		if (judgModel_flag == 0) {
			cout << "Left Limb" << endl;
			// 对右腿进行旋转，应该是点0->点1
			double vx_1 = mitkPointSet1->GetPoint(2)[0] - mitkPointSet1->GetPoint(1)[0];
			double vy_1 = mitkPointSet1->GetPoint(2)[1] - mitkPointSet1->GetPoint(1)[1];
			double vz_1 = mitkPointSet1->GetPoint(2)[2] - mitkPointSet1->GetPoint(1)[2];
			double length_1 = sqrt(pow(vx_1, 2) + pow(vy_1, 2) + pow(vz_1, 2));
			vx = vx_1;
			vy = vy_1;
			vz = vz_1;
			length = length_1;
			// 右腿: B->A, 顺时针110°
			angle = 110.0;
		}
		else if (judgModel_flag == 1) {
			cout << "Right Limb" << endl;
			// 计算右腿旋转分量
			// 截骨面对称变化，原来的截骨面是用的
			double vx_2 = mitkPointSet1->GetPoint(1)[0] - mitkPointSet1->GetPoint(2)[0];
			double vy_2 = mitkPointSet1->GetPoint(1)[1] - mitkPointSet1->GetPoint(2)[1];
			double vz_2 = mitkPointSet1->GetPoint(1)[2] - mitkPointSet1->GetPoint(2)[2];
			double length_2 = sqrt(pow(vx_2, 2) + pow(vy_2, 2) + pow(vz_2, 2));
			vx = vx_2;
			vy = vy_2;
			vz = vz_2;
			length = length_2;
			// 左腿: A->B, 逆时针110°
			angle = -110.0;
		}

		// 旋转分量标准化
		double direction[3]{ vx / length, vy / length, vz / length };

		// 计算截骨线中心点
		double center[3];
		center[0] = (mitkPointSet1->GetPoint(1)[0] + mitkPointSet1->GetPoint(2)[0]) / 2.0;
		center[1] = (mitkPointSet1->GetPoint(1)[1] + mitkPointSet1->GetPoint(2)[1]) / 2.0;
		center[2] = (mitkPointSet1->GetPoint(1)[2] + mitkPointSet1->GetPoint(2)[2]) / 2.0;

		// 对截骨面进行旋转
		auto tibiaNode = GetDataStorage()->GetNamedNode("2nd cut plane");
		Rotate(center, direction, angle, tibiaNode->GetData());
	}

	// 最后， 将几个标志点进行展示
	// 通知数据存储需要更新其内部状态，以便在需要时正确处理数据节点的更改
	// 可以删除
	GetDataStorage()->Modified();

	mitk::RenderingManager::GetInstance()->RequestUpdateAll();
	return true;

}

bool HTONDI::OnCutTibetClicked()
{
	/* 进行截骨计算
	1. 单平面截骨
	2. 双平面截骨

	*/

	// Surface Segementation
	m_Controls.textBrowser_Action->append("Action: Generate Cutting Surface.");

	// 进行截骨计算 
	auto AxialPlane = GetDataStorage()->GetNamedNode("1st cut plane");
	auto SagPlane = GetDataStorage()->GetNamedNode("2nd cut plane");

	// 进行截骨
	if (AxialPlane != nullptr && SagPlane == nullptr)
	{
		CutTibiaWithOnePlane();
	}
	else if (AxialPlane != nullptr && SagPlane != nullptr) {
		CutTibiaWithTwoPlanes();
	}
	else {
		m_Controls.textBrowser_Action->append("Please Modify Cut first!");
		return false;
	}
	// 
	auto proximalTibiaSurface = GetDataStorage()->GetNamedNode("proximal tibiaSurface");
	auto distalTibiaSurface = GetDataStorage()->GetNamedNode("distal tibiaSurface");
	// 记录下这个初始位置，用于后面重置操作
	m_distalTibiaPosition = distalTibiaSurface->GetData()->GetGeometry()->GetOrigin();
	SetModelOpacity(proximalTibiaSurface, 0.5f);
	SetModelOpacity(distalTibiaSurface, 0.5f);
	// 截骨后马上更新
	GetDistancefromTibia();
	return true;
}

bool HTONDI::OnResetCutClicked()
{
	// 重置截骨操作
	m_Controls.textBrowser_Action->append("Action: Reset Cut Plane.");
	// 如果某个对象不存在，调用Remove方法不会产生错误，因为在尝试移除不存在的对象时，函数会简单地忽略该操作
	GetDataStorage()->Remove(GetDataStorage()->GetNamedNode("1st cut plane"));
	GetDataStorage()->Remove(GetDataStorage()->GetNamedNode("2nd cut plane"));
	GetDataStorage()->Remove(GetDataStorage()->GetNamedNode("distal tibiaSurface"));
	GetDataStorage()->Remove(GetDataStorage()->GetNamedNode("proximal tibiaSurface"));
	GetDataStorage()->Remove(GetDataStorage()->GetNamedNode("pointSetInPlaneCutPlane1"));
	GetDataStorage()->Remove(GetDataStorage()->GetNamedNode("planeAndTibiaIntersectionPoint"));

	GetDataStorage()->Remove(GetDataStorage()->GetNamedNode("legForceLineNew"));
	GetDataStorage()->Remove(GetDataStorage()->GetNamedNode("ankleCenterPoint"));
	GetDataStorage()->Remove(GetDataStorage()->GetNamedNode("tibiaDistalPointSet"));

	// 恢复胫骨的可见性
	mitk::DataNode::Pointer tibiaSurface = GetDataStorage()->GetNamedNode("tibiaSurface");
	if (tibiaSurface.IsNotNull())
	{
		// 切换下肢力线节点的可见性
		// bool currentVisibility = tibiaSurface->IsVisible(nullptr, "visible", true);
		tibiaSurface->SetVisibility(true);
	}
	return true;
}

bool HTONDI::CreateOneCutPlane()
{
	// 创建截骨平面
	auto dataNode_tibiaSurface = GetDataStorage()->GetNamedNode("tibiaSurface");

	// 检测库中是否存在需要的数据
	if (dataNode_tibiaSurface == nullptr)
	{
		m_Controls.textBrowser_Action->append("tibiaSurface is missing");
		return false;
	}

	// 获取胫骨表面的数据
	auto tibiaSurface = dynamic_cast<mitk::Surface*>(dataNode_tibiaSurface->GetData());
	// 获取胫骨表面的PolyData，用于几何计算
	auto initialTibiaPolyData = tibiaSurface->GetVtkPolyData();

	// 构建几何变换方法实例
	vtkNew<vtkTransform> tibiaTransform;
	tibiaTransform->SetMatrix(tibiaSurface->GetGeometry()->GetVtkMatrix());

	// 构建变换后的输出实例
	vtkNew<vtkTransformFilter> tmpFilter;
	tmpFilter->SetTransform(tibiaTransform);
	tmpFilter->SetInputData(initialTibiaPolyData);
	tmpFilter->Update();

	// 取出变换后的数据实例
	// 创建复制是为了确保对原始数据的变换操作不会影响到原始数据本身
	vtkNew<vtkPolyData> tibiaPolyData;
	tibiaPolyData->DeepCopy(tmpFilter->GetPolyDataOutput());

	// 创建包围盒实例
	vtkNew<vtkOBBTree> obbTree;
	obbTree->SetDataSet(tibiaPolyData);
	obbTree->SetMaxLevel(2);
	obbTree->BuildLocator();

	// corner OBB的角点坐标位置
	// max OBB最长边-向量
	// mid OBB次长边-向量
	// min OBB最短边-向量
	//获取到包围多边形数据集的OBB的所有关键信息，包括它的位置、大小和方向
	double corner[3], max[3], mid[3], min[3], size[3];
	obbTree->ComputeOBB(tibiaPolyData, corner, max, mid, min, size);


	//=====================OBB可视化==============================
	if (OBB) {
		// 提取数据
		vtkNew<vtkPolyData> obbPolydata;
		obbTree->GenerateRepresentation(0, obbPolydata);
		// 创建临时物体
		auto tmpNode = mitk::DataNode::New();
		auto tmpSurface = mitk::Surface::New();
		tmpSurface->SetVtkPolyData(obbPolydata);
		tmpNode->SetData(tmpSurface);
		tmpNode->SetOpacity(0.2);
		tmpNode->SetName("OBB");
		// 库更新，可视化
		GetDataStorage()->Add(tmpNode);
	}
	//=====================OBB可视化==============================


	// 创建一个初始的截骨平面，并定义位置和方向
	auto cutPlaneSource = vtkSmartPointer<vtkPlaneSource>::New();
	// 平面的原点坐标
	cutPlaneSource->SetOrigin(0, 0, 0);
	// 设定可视化截骨平面的大小，70*70
	cutPlaneSource->SetPoint1(0, 70, 0);
	cutPlaneSource->SetPoint2(70, 0, 0);
	//设置法向量
	// cutPlaneSource->SetNormal(max);
	cutPlaneSource->SetNormal(0, 0, 1);//设置法向量

	// 定义截骨末端合页点的初始坐标
	mitk::Point3D point0;//水平面左边中点
	mitk::Point3D point1;//水平面左下角
	mitk::Point3D point2;//水平面右下角
	mitk::Point3D point3;//水平面右边中点
	mitk::Point3D planeCenterPoint;//水平面中心点

	// 确定最优截骨位置

	// 存储截骨得到的两个部分
	// 截骨面 01 - 0.85
	vtkNew<vtkPolyData> largerSubpart_0;
	vtkNew<vtkPolyData> smallerSubpart_0;
	//新的截骨平面中心点，通过对现有几何数据加权平均得到
	double origin_0[3]
	{
		corner[0] + 0.5 * (1.7 * max[0] + mid[0] + min[0]),
		corner[1] + 0.5 * (1.7 * max[1] + mid[1] + min[1]),
		corner[2] + 0.5 * (1.7 * max[2] + mid[2] + min[2])
	};

	// 截骨面 02 - 0.15
	vtkNew<vtkPolyData> largerSubpart_1;
	vtkNew<vtkPolyData> smallerSubpart_1;
	double origin_1[3]
	{
		corner[0] + 0.5 * (0.3 * max[0] + mid[0] + min[0]),
		corner[1] + 0.5 * (0.3 * max[1] + mid[1] + min[1]),
		corner[2] + 0.5 * (0.3 * max[2] + mid[2] + min[2])
	};


	//执行截骨操作
	CutPolyDataWithPlane(tibiaPolyData, largerSubpart_0, smallerSubpart_0, origin_0, max);
	CutPolyDataWithPlane(tibiaPolyData, largerSubpart_1, smallerSubpart_1, origin_1, max);

	double origin_final[3];
	if (smallerSubpart_0->GetNumberOfCells() >= smallerSubpart_1->GetNumberOfCells())
	{
		// 选择最佳的法向量即可
		origin_final[0] = origin_0[0];
		origin_final[1] = origin_0[1];
		origin_final[2] = origin_0[2];
	}
	else
	{
		origin_final[0] = origin_1[0];
		origin_final[1] = origin_1[1];
		origin_final[2] = origin_1[2];
	}

	// 将初始构建的截骨平面移动到最优的位置上来
	cutPlaneSource->SetCenter(origin_final);
	/*
	 ----------------
	 |              |
	 |              |
	 1       5      4
	 |              |
	 |              |
	 2--------------3
	*/

	// 点 1 - 末端中点 
	point0[0] = origin_final[0] - 35;
	point0[1] = origin_final[1];
	point0[2] = origin_final[2];
	// 点 2 - 左下角
	point1[0] = origin_final[0] - 35;
	point1[1] = origin_final[1] - 35;
	point1[2] = origin_final[2];
	// 点 3 - 右下角
	point2[0] = origin_final[0] + 35;
	point2[1] = origin_final[1] - 35;
	point2[2] = origin_final[2];
	// 点 4 - 末端中点 
	point3[0] = origin_final[0] + 35;
	point3[1] = origin_final[1];
	point3[2] = origin_final[2];
	// 点 5 - 设置原点
	planeCenterPoint[0] = origin_final[0];
	planeCenterPoint[1] = origin_final[1];
	planeCenterPoint[2] = origin_final[2];

	// 将坐标添加到mitk::PointSet中
	// 用于上升截骨面的旋转计算
	mitkPointSet1->InsertPoint(0, point0);//第一截骨面末端中点标记
	mitkPointSet1->InsertPoint(1, point1);//平面左上角
	mitkPointSet1->InsertPoint(2, point2);//平面左下角
	mitkPointSet1->InsertPoint(3, point3);//平面右边中点
	mitkPointSet1->InsertPoint(4, planeCenterPoint);//平面原点

	// 删除上次生成的点
	auto tmpNodes = GetDataStorage()->GetNamedNode("pointSetInPlaneCutPlane1");
	if (tmpNodes) {
		GetDataStorage()->Remove(tmpNodes);
	}

	mitk::DataNode::Pointer pointSetInPlaneCutPlane = mitk::DataNode::New();
	pointSetInPlaneCutPlane->SetName("pointSetInPlaneCutPlane1");
	// 红色，大小 5.0
	pointSetInPlaneCutPlane->SetColor(1.0, 0.0, 0.0);
	pointSetInPlaneCutPlane->SetData(mitkPointSet1);
	pointSetInPlaneCutPlane->SetFloatProperty("pointsize", 5.0);
	GetDataStorage()->Add(pointSetInPlaneCutPlane);

	// 图像更新
	cutPlaneSource->Update();

	// 获取截骨平面的数据
	auto cutSurface = mitk::Surface::New();
	cutSurface->SetVtkPolyData(cutPlaneSource->GetOutput());

	// 创建截骨平面DataNode对象
	auto planeNode = mitk::DataNode::New();
	planeNode->SetData(cutSurface);
	planeNode->SetName("tibia cut plane");

	// 将DataNode添加到数据存储中
	GetDataStorage()->Add(planeNode);

	return true;
}

bool HTONDI::CutTibiaWithOnePlane()
{
	// 单平面截骨函数
	m_Controls.textBrowser_Action->append("Cutting with one plane");
	auto cutPlaneNode = GetDataStorage()->GetNamedNode("1st cut plane");
	auto tibiaSurfaceNode = GetDataStorage()->GetNamedNode("tibiaSurface");

	// 检查是否存在所需要的数据
	if (cutPlaneNode == nullptr)
	{
		m_Controls.textBrowser_Action->append("'tibia cut plane' is not ready");
		return false;
	}
	if (tibiaSurfaceNode == nullptr)
	{
		m_Controls.textBrowser_Action->append("'tibiaSurface' is not ready");
		return false;
	}

	// 获取 截骨平面 和 原始骨平面 的数据
	// 所有的原始数据
	auto mitkCutSurface = dynamic_cast<mitk::Surface*>(cutPlaneNode->GetData());
	auto tibiaMitkSurface = dynamic_cast<mitk::Surface*>(tibiaSurfaceNode->GetData());
	// 所有的PolyData
	auto tmpVtkSurface_initial = mitkCutSurface->GetVtkPolyData();
	auto tibiaVtkSurface_initial = tibiaMitkSurface->GetVtkPolyData();

	// 截骨面
	//=======================================================================
	// 构建转化方法实例
	vtkNew<vtkTransform> cutPlaneTransform;
	cutPlaneTransform->SetMatrix(mitkCutSurface->GetGeometry()->GetVtkMatrix());
	// 构建转化实例
	vtkNew<vtkTransformFilter> cutPlaneTransformFilter;
	cutPlaneTransformFilter->SetTransform(cutPlaneTransform);
	cutPlaneTransformFilter->SetInputData(tmpVtkSurface_initial);
	cutPlaneTransformFilter->Update();
	// 数据深拷贝
	vtkNew<vtkPolyData> tmpVtkSurface;
	tmpVtkSurface->DeepCopy(cutPlaneTransformFilter->GetPolyDataOutput());
	//=======================================================================
	// 胫骨表面
	//=======================================================================
	// 构建转化方法实例
	vtkNew<vtkTransform> tibiaTransform;
	tibiaTransform->SetMatrix(tibiaMitkSurface->GetGeometry()->GetVtkMatrix());
	// 构建转化实例
	vtkNew<vtkTransformFilter> tibiaTransformFilter;
	tibiaTransformFilter->SetTransform(tibiaTransform);
	tibiaTransformFilter->SetInputData(tibiaVtkSurface_initial);
	tibiaTransformFilter->Update();
	// 数据深拷贝
	vtkNew<vtkPolyData> tibiaVtkSurface;
	tibiaVtkSurface->DeepCopy(tibiaTransformFilter->GetPolyDataOutput());
	//=======================================================================

	// 从给定的平面数据中提取法线向量和中心点坐标
	double cutPlaneCenter[3];
	double surfaceNormal[3];

	// 获取数据
	GetPlaneProperty(tmpVtkSurface, surfaceNormal, cutPlaneCenter);

	// 进行图像分割
	vtkNew<vtkPolyData> proximalTibiaSurface;
	vtkNew<vtkPolyData> distalTibiaSurface;

	CutPolyDataWithPlane(tibiaVtkSurface, distalTibiaSurface, proximalTibiaSurface, cutPlaneCenter, surfaceNormal);

	// 获取两个部分的PloyData
	auto mitkProximalSurface = mitk::Surface::New();
	auto mitkDistalSurface = mitk::Surface::New();
	mitkProximalSurface->SetVtkPolyData(proximalTibiaSurface);
	mitkDistalSurface->SetVtkPolyData(distalTibiaSurface);

	// 表面粗糙化处理，用于加速实时渲染
	auto proximal_remehsed = mitk::Remeshing::Decimate(mitkProximalSurface, 1, true, true);
	auto distal_remehsed = mitk::Remeshing::Decimate(mitkDistalSurface, 1, true, true);

	// 填补空洞-近端
	vtkNew<vtkFillHolesFilter> holeFiller0;
	holeFiller0->SetInputData(proximal_remehsed->GetVtkPolyData());
	holeFiller0->SetHoleSize(500);
	holeFiller0->Update();
	// 获取填补后的表面数据
	vtkNew<vtkPolyData> proximalTibia;
	proximalTibia->DeepCopy(holeFiller0->GetOutput());
	auto proximalSurface = mitk::Surface::New();
	proximalSurface->SetVtkPolyData(proximalTibia);

	// 填补空洞-远端
	vtkNew<vtkFillHolesFilter> holeFiller1;
	holeFiller1->SetInputData(distal_remehsed->GetVtkPolyData());
	holeFiller1->SetHoleSize(500);
	holeFiller1->Update();
	// 获取填补后的表面数据
	vtkNew<vtkPolyData> distalTibia;
	distalTibia->DeepCopy(holeFiller1->GetOutput());
	auto distalSurface = mitk::Surface::New();
	distalSurface->SetVtkPolyData(distalTibia);

	// 远端
	// 检查是否存在
	auto checktmp01 = GetDataStorage()->GetNamedNode("proximal tibiaSurface");
	if (checktmp01)
	{
		GetDataStorage()->Remove(GetDataStorage()->GetNamedNode("proximal tibiaSurface"));
	}
	// 新建
	auto tmpNode0 = mitk::DataNode::New();
	tmpNode0->SetData(proximalSurface);
	tmpNode0->SetName("proximal tibiaSurface");
	// 进行光滑平面渲染
	mitk::VtkInterpolationProperty::Pointer interpolationProp0;
	tmpNode0->GetProperty(interpolationProp0, "material.interpolation");
	interpolationProp0->SetInterpolationToFlat();
	tmpNode0->SetProperty("material.interpolation", interpolationProp0);

	// 近端
	// 检查是否存在
	auto checktmp02 = GetDataStorage()->GetNamedNode("distal tibiaSurface");
	if (checktmp02)
	{
		GetDataStorage()->Remove(GetDataStorage()->GetNamedNode("distal tibiaSurface"));
	}
	auto tmpNode1 = mitk::DataNode::New();
	tmpNode1->SetData(distalSurface);
	tmpNode1->SetName("distal tibiaSurface");
	// 进行光滑平面渲染
	mitk::VtkInterpolationProperty::Pointer interpolationProp1;
	tmpNode1->GetProperty(interpolationProp1, "material.interpolation");
	interpolationProp1->SetInterpolationToFlat();
	tmpNode1->SetProperty("material.interpolation", interpolationProp1);

	// 设置原有的胫骨为不可见
	mitk::DataNode::Pointer tibiaSurface = GetDataStorage()->GetNamedNode("tibiaSurface");
	if (tibiaSurface.IsNotNull())
	{
		// 切换下肢力线节点的可见性
		bool currentVisibility = tibiaSurface->IsVisible(nullptr, "visible", true);
		tibiaSurface->SetVisibility(!currentVisibility);
	}

	GetDataStorage()->Add(tmpNode0);
	GetDataStorage()->Add(tmpNode1);

	return true;
}

bool HTONDI::CutTibiaWithTwoPlanes()
{
	m_Controls.textBrowser_Action->append("Cutting with two plane");
	// 同时使用两个平面进行截骨
	auto cutplane_0 = GetDataStorage()->GetNamedNode("1st cut plane");
	auto cutplane_1 = GetDataStorage()->GetNamedNode("2nd cut plane");
	auto tibiaNode = GetDataStorage()->GetNamedNode("tibiaSurface");

	// 检查是否存在所需要的数据
	if (cutplane_0 == nullptr || cutplane_1 == nullptr)
	{
		m_Controls.textBrowser_Action->append("'1st cut plane' or '2nd cut plane' is not ready");
		return false;
	}
	if (tibiaNode == nullptr)
	{
		m_Controls.textBrowser_Action->append("'tibiaSurface' is not ready");
		return false;
	}

	auto mitkCutPlane_0 = dynamic_cast<mitk::Surface*>(cutplane_0->GetData());
	auto mitkCutPlane_1 = dynamic_cast<mitk::Surface*>(cutplane_1->GetData());
	auto mitkTibia = dynamic_cast<mitk::Surface*>(tibiaNode->GetData());

	vtkNew<vtkPolyData> vtkCutPlane_0;
	vtkNew<vtkPolyData> vtkCutPlane_1;
	vtkNew<vtkPolyData> vtkTibia;

	// 截骨面- Axial
	vtkNew<vtkTransform> cutPlaneTransform_0;
	cutPlaneTransform_0->SetMatrix(mitkCutPlane_0->GetGeometry()->GetVtkMatrix());
	vtkNew<vtkTransformFilter> cutPlaneTransformFilter_0;
	cutPlaneTransformFilter_0->SetTransform(cutPlaneTransform_0);
	cutPlaneTransformFilter_0->SetInputData(mitkCutPlane_0->GetVtkPolyData());
	cutPlaneTransformFilter_0->Update();
	vtkCutPlane_0->DeepCopy(cutPlaneTransformFilter_0->GetPolyDataOutput());

	// 截骨面- Sag
	vtkNew<vtkTransform> cutPlaneTransform_1;
	cutPlaneTransform_1->SetMatrix(mitkCutPlane_1->GetGeometry()->GetVtkMatrix());
	vtkNew<vtkTransformFilter> cutPlaneTransformFilter_1;
	cutPlaneTransformFilter_1->SetTransform(cutPlaneTransform_1);
	cutPlaneTransformFilter_1->SetInputData(mitkCutPlane_1->GetVtkPolyData());
	cutPlaneTransformFilter_1->Update();
	vtkCutPlane_1->DeepCopy(cutPlaneTransformFilter_1->GetPolyDataOutput());

	// 骨表面
	vtkNew<vtkTransform> tibiaTransform;
	tibiaTransform->SetMatrix(mitkTibia->GetGeometry()->GetVtkMatrix());
	vtkNew<vtkTransformFilter> tibiaTransformFilter;
	tibiaTransformFilter->SetTransform(tibiaTransform);
	tibiaTransformFilter->SetInputData(mitkTibia->GetVtkPolyData());
	tibiaTransformFilter->Update();
	vtkTibia->DeepCopy(tibiaTransformFilter->GetPolyDataOutput());

	// 获取两个截骨面的法向量和平面中心
	double cutPlaneCenter_0[3];
	double cutPlaneNormal_0[3];
	double cutPlaneCenter_1[3];
	double cutPlaneNormal_1[3];
	GetPlaneProperty(vtkCutPlane_0, cutPlaneNormal_0, cutPlaneCenter_0);
	GetPlaneProperty(vtkCutPlane_1, cutPlaneNormal_1, cutPlaneCenter_1);

	vtkNew<vtkPolyData> largetPart;
	vtkNew<vtkPolyData> tmpMiddlePart;
	vtkNew<vtkPolyData> middlePart;
	vtkNew<vtkPolyData> smallPart;

	// Part01: 先使用Axial截骨，达到 A-大-小 两个部分
	// Part02: 取出 A-小，再用Sag截骨，得到 S-大-小 两个部分
	// Part03: 合并 S-小 和 A-大，最后库中为 A-大 + S-大

	// Part01:
	CutPolyDataWithPlane(vtkTibia, largetPart, tmpMiddlePart, cutPlaneCenter_0, cutPlaneNormal_0);
	// Part02:
	CutPolyDataWithPlane(tmpMiddlePart, middlePart, smallPart, cutPlaneCenter_1, cutPlaneNormal_1);

	// Part03
	vtkSmartPointer<vtkAppendPolyData> appendFilter =
		vtkSmartPointer<vtkAppendPolyData>::New();
	vtkSmartPointer<vtkCleanPolyData> cleanFilter =
		vtkSmartPointer<vtkCleanPolyData>::New();

	// 合并 A-大 和 S-小
	appendFilter->AddInputData(largetPart);
	appendFilter->AddInputData(smallPart);
	appendFilter->Update();

	cleanFilter->SetInputData(appendFilter->GetOutput());
	cleanFilter->Update();

	auto proximalSurface = mitk::Surface::New();
	auto distalSurface = mitk::Surface::New();

	proximalSurface->SetVtkPolyData(middlePart);
	distalSurface->SetVtkPolyData(cleanFilter->GetOutput());

	//auto proximal_remehsed = mitk::Remeshing::Decimate(proximalSurface, 1, true, true);
	//auto distal_remehsed = mitk::Remeshing::Decimate(distalSurface, 1, true, true);

	// 检查是否存在
	auto checktmp01 = GetDataStorage()->GetNamedNode("proximal tibiaSurface");
	if (checktmp01)
	{
		GetDataStorage()->Remove(GetDataStorage()->GetNamedNode("proximal tibiaSurface"));
	}
	// 检查是否存在
	auto checktmp02 = GetDataStorage()->GetNamedNode("distal tibiaSurface");
	if (checktmp02)
	{
		GetDataStorage()->Remove(GetDataStorage()->GetNamedNode("distal tibiaSurface"));
	}
	// 新建

	auto proximalNode = mitk::DataNode::New();
	auto distalNode = mitk::DataNode::New();

	proximalNode->SetName("proximal tibiaSurface");
	proximalNode->SetData(proximalSurface);
	proximalNode->SetColor(0, 1, 0);

	distalNode->SetName("distal tibiaSurface");
	distalNode->SetData(distalSurface);
	//distalNode->SetColor(0,0,1);

	GetDataStorage()->Add(distalNode);
	GetDataStorage()->Add(proximalNode);

	// 设置原有的胫骨为不可见
	mitk::DataNode::Pointer tibiaSurface = GetDataStorage()->GetNamedNode("tibiaSurface");
	if (tibiaSurface.IsNotNull())
	{
		// 切换下肢力线节点的可见性
		tibiaSurface->SetVisibility(false);
	}

	GetDataStorage()->Modified();
	mitk::RenderingManager::GetInstance()->RequestUpdateAll();

	return true;
}

bool HTONDI::GetPlaneProperty(vtkSmartPointer<vtkPolyData> plane, double normal[3], double center[3])
{
	/* 计算平面的法向量
	
	采取平面上的三个点，计算向量叉乘
	*/

	auto tmpCenter = plane->GetCenter();

	center[0] = *tmpCenter;
	center[1] = *(tmpCenter + 1);
	center[2] = *(tmpCenter + 2);

	// Obtain the normal of the mitkSurface
	double p0[3]; double p1[3]; double p2[3];

	plane->GetCell(0)->GetPoints()->GetPoint(0, p0);
	plane->GetCell(0)->GetPoints()->GetPoint(1, p1);
	plane->GetCell(0)->GetPoints()->GetPoint(2, p2);

	Eigen::Vector3d a(*p0, *(p0 + 1), *(p0 + 2));
	Eigen::Vector3d b(*p1, *(p1 + 1), *(p1 + 2));
	Eigen::Vector3d c(*p2, *(p2 + 1), *(p2 + 2));

	Eigen::Vector3d tmpVector0 = b - a;
	Eigen::Vector3d tmpVector1 = c - a;

	Eigen::Vector3d normalVector = tmpVector0.cross(tmpVector1);
	// 将法向量归一化为单位向量
	normalVector.normalize();

	normal[0] = normalVector[0];
	normal[1] = normalVector[1];
	normal[2] = normalVector[2];

	return true;
}





bool HTONDI::CutPolyDataWithPlane(vtkSmartPointer<vtkPolyData> dataToCut,
	vtkSmartPointer<vtkPolyData> largerSubPart,
	vtkSmartPointer<vtkPolyData> smallerSubPart,
	double planeOrigin[3], double planeNormal[3])
{
	/* 进行截骨计算
	输入原数据、切分后的大、小数据存储体、截骨面中心、截骨面法向量
	对输入的体数据进行截取，返回切分后的数据，分解为大分量和小分量

	*/

	// 创建一个vtkPlane对象，设置平面的法向量和原点
	vtkNew<vtkPlane> implicitPlane;
	implicitPlane->SetNormal(planeNormal);
	implicitPlane->SetOrigin(planeOrigin);

	// 创建一个vtkClipPolyData对象，设置输入数据并根据给定平面进行切割
	vtkNew<vtkClipPolyData> clipper;
	clipper->SetInputData(dataToCut);
	clipper->GenerateClippedOutputOn();
	clipper->SetClipFunction(implicitPlane);
	clipper->Update();

	// 复制切割后的数据到新的vtkPolyData对象中
	vtkNew<vtkPolyData> tibiaPart_0;
	tibiaPart_0->DeepCopy(clipper->GetClippedOutput());
	// 获取分量的多边形数量
	int cellNum_0 = tibiaPart_0->GetNumberOfCells();

	// 更新clipper并获取另一部分切割数据
	clipper->Update();
	vtkNew<vtkPolyData> tibiaPart_1;
	tibiaPart_1->DeepCopy(clipper->GetOutput());
	// 获取分量的多边形数量
	int cellNum_1 = tibiaPart_1->GetNumberOfCells();

	// 使用vtkFillHolesFilter填补可能存在的空洞
	// 对 tibiaPart_0 进行相同的空洞填补操作
	vtkNew<vtkFillHolesFilter> holeFiller0;
	holeFiller0->SetInputData(tibiaPart_0);
	holeFiller0->SetHoleSize(500);
	holeFiller0->Update();
	vtkNew<vtkPolyData> tibia_filled_0;
	tibia_filled_0->DeepCopy(holeFiller0->GetOutput());

	// 对 tibiaPart_1 进行相同的空洞填补操作
	vtkNew<vtkFillHolesFilter> holeFiller1;
	holeFiller1->SetInputData(tibiaPart_1);
	holeFiller1->SetHoleSize(500);
	holeFiller1->Update();
	vtkNew<vtkPolyData> tibia_filled_1;
	tibia_filled_1->DeepCopy(holeFiller1->GetOutput());

	// 根据多边形数量确定哪部分是大的分量，哪部分是小分量，并将数据复制到对应的对象中
	if (cellNum_1 >= cellNum_0)
	{
		largerSubPart->DeepCopy(tibia_filled_1);
		smallerSubPart->DeepCopy(tibia_filled_0);
	}
	else
	{
		largerSubPart->DeepCopy(tibia_filled_0);
		smallerSubPart->DeepCopy(tibia_filled_1);
	}

	return true;
}

// 钢板规划
bool HTONDI::OnShowSteelClick()
{
	/* 显示/隐藏 钢板模型 */


	m_Controls.textBrowser_Action->append("Action: Show SteelPlate.");
	auto steelPlate = GetDataStorage()->GetNamedNode("SteelPlate");
	auto steelPlatePoint = GetDataStorage()->GetNamedNode("SteelPlatePointSet");
	if (steelPlate && steelPlatePoint)
	{
		// 获取状态
		bool currentVisibility = steelPlate->IsVisible(nullptr, "visible", true);
		// 展示钢板和点集合
		steelPlate->SetVisibility(!currentVisibility);
		steelPlatePoint->SetVisibility(!currentVisibility);
	}
	else 
	{
		m_Controls.textBrowser_Action->append("SteelPlate or SteelPlatePoint Not Found!");
	}
	GetDataStorage()->Modified();
	mitk::RenderingManager::GetInstance()->RequestUpdateAll();
	return true;
}

bool HTONDI::OnResetSteelClick()
{
	/* 重置钢板模型 */


	m_Controls.textBrowser_Action->append("Action: Reset SteelPlate.");
	auto steelPlate = GetDataStorage()->GetNamedNode("SteelPlate");
	if (steelPlate)
	{
		steelPlate->SetVisibility(true);
		steelPlate->GetData()->GetGeometry()->SetOrigin(m_steelPosition);
		// 将其上的节点也一起迁移
		auto steelPlatePoints = GetDataStorage()->GetNamedNode("SteelPlatePointSet");
		steelPlatePoints->GetData()->GetGeometry()->SetOrigin(m_steelPosition);
	}
	else
	{
		m_Controls.textBrowser_Action->append("SteelPlate model Not Found!");
	}
	GetDataStorage()->Modified();
	mitk::RenderingManager::GetInstance()->RequestUpdateAll();
	return true;
}

bool HTONDI::OnSetFinalPosClick()
{
	/* 确定钢板的最后位置，并标记钢钉位置 */

	m_Controls.textBrowser_Action->append("Action: Set Final SteelPlate Pos.");
	
	// 标记钢钉规划点
	// 已知三个目标点的顺序为 中 -> 左 -> 右
	mitk::DataNode::Pointer steelPlatePoints = GetDataStorage()->GetNamedNode("SteelPlatePointSet");
	if (steelPlatePoints)
	{
		// 创建一个点集，用于记录所有计算出的钻孔中心位置
		mitk::PointSet::Pointer holeCenters = mitk::PointSet::New();
		// 中 -> 左 -> 右
		auto point1 = dynamic_cast<mitk::PointSet*>(steelPlatePoints->GetData())->GetPoint(0);
		auto point2 = dynamic_cast<mitk::PointSet*>(steelPlatePoints->GetData())->GetPoint(1);
		auto point3 = dynamic_cast<mitk::PointSet*>(steelPlatePoints->GetData())->GetPoint(2);
		holeCenters->InsertPoint(0, point1);
		holeCenters->InsertPoint(1, point2);
		holeCenters->InsertPoint(2, point3);

		// 进行可视化
		auto holeCentersNode = GetDataStorage()->GetNamedNode("SteelPlateHolePointSet");
		if (holeCentersNode != nullptr)
		{	
			// 有数据则直接更新
			m_Controls.textBrowser_Action->append("Change SteelPlateHole Pos");
			holeCentersNode->SetData(holeCenters);
		}
		else
		{
			// 没有点则创建点
			m_Controls.textBrowser_Action->append("Init SteelPlateHole Pos");
			mitk::DataNode::Pointer holeCentersNode = mitk::DataNode::New();
			holeCentersNode->SetData(holeCenters);
			holeCentersNode->SetName("SteelPlateHolePointSet");
			holeCentersNode->SetColor(0.0, 0.0, 1.0);
			holeCentersNode->SetProperty("pointsize", mitk::FloatProperty::New(5.0));
			GetDataStorage()->Add(holeCentersNode);
		}
	}
	else
	{
		m_Controls.textBrowser_Action->append("SteelPlatePointSet Not Found!");
	}
	// 将标记点进行不可见化
	if (steelPlatePoints)
	{
		steelPlatePoints->SetVisibility(false);
	}
	else
	{
		m_Controls.textBrowser_Action->append("SteelPlatePointSet model Not Found!");
	}
	GetDataStorage()->Modified();
	mitk::RenderingManager::GetInstance()->RequestUpdateAll();
	return true;
}

// 撑开角度规划
bool HTONDI::OnCaculateStrechAngleClicked()
{
	/* 计划手术撑开角度并应用远端旋转位移
	// 将力线矫正到一个合适的位置，也就是将力线穿过胫骨平台的中点
	// 本质上，将胫骨远端部分旋转到矫正后的力线的延长线上即可
	*/

	m_Controls.textBrowser_Action->append("Action: Caculate the Angel.");

	// 获取股骨中心点、胫骨平台中点
	cout << "test 01" << endl;
	mitk::DataNode::Pointer hipCenter = GetDataStorage()->GetNamedNode("hipCenterPoint");
	auto hipCenterPointSet = dynamic_cast<mitk::PointSet*>(hipCenter->GetData());
	// 进行格式转化
	mitk::Point3D hipCenterPoint = hipCenterPointSet->GetPoint(0);

	// 获取胫骨踝点集
	mitk::DataNode::Pointer tibiaLandmarkNode = GetDataStorage()->GetNamedNode("tibiaLandmarkPointSet");
	auto tibiaLandmarkPointSet = dynamic_cast<mitk::PointSet*>(tibiaLandmarkNode->GetData());

	// 获取第一和第二个点
	// 胫骨近端外侧点: tibiaProximalLateralPoint
	// 胫骨近端内侧点: tibiaProximalMedialPoint
	mitk::Point3D tibiaProximalLateralPoint = tibiaLandmarkPointSet->GetPoint(1);
	mitk::Point3D tibiaProximalMedialPoint = tibiaLandmarkPointSet->GetPoint(2);

	// 计算胫骨近端(胫骨平台中心)的中心点
	mitk::Point3D tibiaProximalCenterPoint;
	tibiaProximalCenterPoint[0] = (tibiaProximalLateralPoint[0] + tibiaProximalMedialPoint[0]) / 2;
	tibiaProximalCenterPoint[1] = (tibiaProximalLateralPoint[1] + tibiaProximalMedialPoint[1]) / 2;
	tibiaProximalCenterPoint[2] = (tibiaProximalLateralPoint[2] + tibiaProximalMedialPoint[2]) / 2;

	cout << "test 02" << endl;
	// 计算目标力线的延长线
	// 首先，取得线的方向
	mitk::Vector3D newLegForceDirection = tibiaProximalCenterPoint - hipCenterPoint;
	// 计算向量的长度
	double length = sqrt(pow(newLegForceDirection[0], 2) +
		pow(newLegForceDirection[1], 2) +
		pow(newLegForceDirection[2], 2));
	// 归一化向量
	if (length != 0) {
		newLegForceDirection[0] /= length;
		newLegForceDirection[1] /= length;
		newLegForceDirection[2] /= length;
	}
	// 根据力线长度计算目标点的坐标
	// 来自【胡】的方法 => 假设力线长度不变 => 去计算力线旋转到目标位置的点位置
	// 根据力线长度计算目标点的坐标
	mitk::Vector3D transPos;
	transPos[0] = newLegForceDirection[0] * Line_length + hipCenterPoint[0];
	transPos[1] = newLegForceDirection[1] * Line_length + hipCenterPoint[1];
	transPos[2] = newLegForceDirection[2] * Line_length + hipCenterPoint[2];
	std::cout << "目标延长线上的点C坐标为: (" << transPos[0] << ", " << transPos[1] << ", " << transPos[2] << ")" << endl;

	// 0:右腿. 1：左腿
	if (judgModel_flag == 1) {
		// 左腿
		cout << "left Limb" << endl;
		std::cout << "合页点为: (" << mitkPointSet1->GetPoint(3)[0] << ", " << mitkPointSet1->GetPoint(3)[1] << ", " << mitkPointSet1->GetPoint(3)[2] << ")" << endl;
		auto tmp = GetDataStorage()->GetNamedNode("legForceLine");
		if (tmp == nullptr)
		{
			m_Controls.textBrowser_Action->append("legForceLine Not Found.");
			return false;
		}
		auto linePointSet = dynamic_cast<mitk::PointSet*>(GetDataStorage()->GetNamedNode("legForceLine")->GetData());
		//计算当前力线的向量；
		double vx_1 = linePointSet->GetPoint(1)[0] - mitkPointSet1->GetPoint(3)[0];//当前力线终点-合页点
		double vy_1 = linePointSet->GetPoint(1)[1] - mitkPointSet1->GetPoint(3)[1];
		double vz_1 = linePointSet->GetPoint(1)[2] - mitkPointSet1->GetPoint(3)[2];
		double length_1 = sqrt(pow(vx_1, 2) + pow(vy_1, 2) + pow(vz_1, 2));
		double ux_1 = vx_1 / length_1;
		double uy_1 = vy_1 / length_1;
		double uz_1 = vz_1 / length_1;
		double direction1[3]{ ux_1,uy_1,uz_1 };
		//计算理想力线的向量
		double vx_2 = transPos[0] - mitkPointSet1->GetPoint(3)[0];//理想力线终点-合页点
		double vy_2 = transPos[1] - mitkPointSet1->GetPoint(3)[1];
		double vz_2 = transPos[2] - mitkPointSet1->GetPoint(3)[2];
		double length_2 = sqrt(pow(vx_2, 2) + pow(vy_2, 2) + pow(vz_2, 2));
		double ux_2 = vx_2 / length_2;
		double uy_2 = vy_2 / length_2;
		double uz_2 = vz_2 / length_2;
		double direction2[3]{ ux_2,uy_2,uz_2 };
		// 计算两个方向向量之间的夹角
		double dotProduct = direction1[0] * direction2[0] + direction1[1] * direction2[1] + direction1[2] * direction2[2]; // 点积
		double angleInRadians = acos(dotProduct); // 使用反余弦函数得到角度的弧度值
		double angleInDegrees = round(angleInRadians * (180.0 / M_PI) * 10) / 10; // 保留一位小数

		m_Controls.LineEdit_angle->setText(QString::number(angleInDegrees));
		std::cout << "The angle between the two directions is: " << angleInDegrees << " degrees." << std::endl;
		m_Controls.LineEdit_transAngle->setText(QString::number(angleInDegrees));

		CaculateStrechHeigh();
		// 这里的旋转会按照LinEdit中的数值来进行，数值在前面计算夹角的时候就被更新了
		RotatePlus();
		
	}
	else if (judgModel_flag == 0)
	{
		// 当前模型为右腿
		cout << "Right Limb" << endl;
		std::cout << "合页点为: (" << mitkPointSet1->GetPoint(0)[0] << ", " << mitkPointSet1->GetPoint(0)[1] << ", " << mitkPointSet1->GetPoint(0)[2] << ")" << endl;
		auto tmp = GetDataStorage()->GetNamedNode("legForceLine");
		if (tmp == nullptr)
		{
			m_Controls.textBrowser_Action->append("legForceLine Not Found.");
			return false;
		}
		auto linePointSet = dynamic_cast<mitk::PointSet*>(GetDataStorage()->GetNamedNode("legForceLine")->GetData());
		//计算当前力线的向量；
		double vx_1 = linePointSet->GetPoint(1)[0] - mitkPointSet1->GetPoint(0)[0];//当前力线终点-合页点
		double vy_1 = linePointSet->GetPoint(1)[1] - mitkPointSet1->GetPoint(0)[1];
		double vz_1 = linePointSet->GetPoint(1)[2] - mitkPointSet1->GetPoint(0)[2];
		double length_1 = sqrt(pow(vx_1, 2) + pow(vy_1, 2) + pow(vz_1, 2));
		double ux_1 = vx_1 / length_1;
		double uy_1 = vy_1 / length_1;
		double uz_1 = vz_1 / length_1;
		double direction1[3]{ ux_1,uy_1,uz_1 };
		//计算理想力线的向量
		double vx_2 = transPos[0] - mitkPointSet1->GetPoint(0)[0];//理想力线终点-合页点
		double vy_2 = transPos[1] - mitkPointSet1->GetPoint(0)[1];
		double vz_2 = transPos[2] - mitkPointSet1->GetPoint(0)[2];
		double length_2 = sqrt(pow(vx_2, 2) + pow(vy_2, 2) + pow(vz_2, 2));
		double ux_2 = vx_2 / length_2;
		double uy_2 = vy_2 / length_2;
		double uz_2 = vz_2 / length_2;
		double direction2[3]{ ux_2,uy_2,uz_2 };
		// 计算两个方向向量之间的夹角
		double dotProduct = direction1[0] * direction2[0] + direction1[1] * direction2[1] + direction1[2] * direction2[2]; // 点积
		double angleInRadians = acos(dotProduct); // 使用反余弦函数得到角度的弧度值
		double angleInDegrees = round(angleInRadians * (180.0 / M_PI) * 10) / 10; // 保留一位小数
		m_Controls.LineEdit_angle->setText(QString::number(angleInDegrees));
		std::cout << "The angle between the two directions is: " << angleInDegrees << " degrees." << std::endl;
		m_Controls.LineEdit_transAngle->setText(QString::number(angleInDegrees));

		CaculateStrechHeigh();
		// 这里的旋转会按照LinEdit中的数值来进行，数值在前面计算夹角的时候就被更新了
		RotateMinus();
	}
	return true;
}

bool HTONDI::OnResetAngleClicked()
{
	/* 重置撑开角度规划
		删除规划结果
		重新进行截骨生成
	*/
	m_Controls.textBrowser_Action->append("Action: ResetAngle.");

	GetDataStorage()->Remove(GetDataStorage()->GetNamedNode("legForceLineNew"));
	GetDataStorage()->Remove(GetDataStorage()->GetNamedNode("ankleCenterPointNew"));
	GetDataStorage()->Remove(GetDataStorage()->GetNamedNode("tibiaDistalPointSet"));
	GetDataStorage()->Remove(GetDataStorage()->GetNamedNode("distal tibiaSurface"));
	GetDataStorage()->Remove(GetDataStorage()->GetNamedNode("proximal tibiaSurface"));

	// 清除所有后重新生成截骨面
	OnCutTibetClicked();
	return true;
}

// 下肢力线规划
bool HTONDI::OnShowMachineLineClicked()
{
	/* 计算并显示下肢力线
	// 选中 股骨头中心 和 踝关节中心
	// ===================================
	// 股骨头中心点 由计算拟合得出
	// 踝关节中心由 内外踝 计算中点得出
	// ===================================
	*/
	
	m_Controls.textBrowser_Action->append("Action: Show The Line of Force.");

	// 直接从库中加载股骨头中心点
	cout << "test 01" << endl;
	mitk::DataNode::Pointer hipCenterNode = GetDataStorage()->GetNamedNode("hipCenterPoint");
	auto hipCenterPoint = dynamic_cast<mitk::PointSet*>(hipCenterNode->GetData())->GetPoint(0);
	cout << "test 02" << endl;
	// 获取胫骨踝点集
	mitk::DataNode::Pointer tibiaLandmarkNode = GetDataStorage()->GetNamedNode("tibiaLandmarkPointSet");
	auto tibiaLandmarkPointSet = dynamic_cast<mitk::PointSet*>(tibiaLandmarkNode->GetData());
	cout << "test 03" << endl;
	// 获取第三和第四个点
	mitk::Point3D anklePoint1 = tibiaLandmarkPointSet->GetPoint(2);
	mitk::Point3D anklePoint2 = tibiaLandmarkPointSet->GetPoint(3);
	cout << "test 04" << endl;
	// 计算踝关节中心点
	mitk::Point3D ankleCenterPoint;
	ankleCenterPoint[0] = (anklePoint1[0] + anklePoint2[0]) / 2;
	ankleCenterPoint[1] = (anklePoint1[1] + anklePoint2[1]) / 2;
	ankleCenterPoint[2] = (anklePoint1[2] + anklePoint2[2]) / 2;
	cout << "test 05" << endl;
	// 可视化踝关节中心点
	// 在图像上绘制踝关节中心点

	// 首先删除原有的
	auto ankleCenter = GetDataStorage()->GetNamedNode("ankleCenterPoint");
	if (ankleCenter != nullptr) {
		cout << "test 06" << endl;
		GetDataStorage()->Remove(ankleCenter);
	}

	cout << "test 07" << endl;
	mitk::PointSet::Pointer ankleCenterPointSet = mitk::PointSet::New();
	ankleCenterPointSet->InsertPoint(0, ankleCenterPoint);
	mitk::DataNode::Pointer ankleCenterNode = mitk::DataNode::New();
	ankleCenterNode->SetData(ankleCenterPointSet);
	ankleCenterNode->SetName("ankleCenterPoint");
	ankleCenterNode->SetProperty("color", mitk::ColorProperty::New(1.0, 0.0, 0.0)); // 设置颜色为红色
	ankleCenterNode->SetProperty("pointsize", mitk::FloatProperty::New(5.0)); // 设置点的大小为5
	GetDataStorage()->Add(ankleCenterNode);

	cout << "test 08" << endl;
	// 计算下肢力线方向
	mitk::Vector3D legForceDirection = ankleCenterPoint - hipCenterPoint;

	cout << "test 09" << endl;
	// 创建新的点集
	mitk::PointSet::Pointer legForceLine = mitk::PointSet::New();
	legForceLine->SetPoint(0, hipCenterPoint);
	legForceLine->SetPoint(1, ankleCenterPoint);

	cout << "test 10" << endl;
	// 查询是否存在名为"LegForceLine"的节点
	mitk::DataNode::Pointer existingLegForceLineNode = GetDataStorage()->GetNamedNode("legForceLine");
	cout << "test 11" << endl;
	// 设置下肢骨骼透明度
	auto tibiaSurface = GetDataStorage()->GetNamedNode("tibiaSurface");
	SetModelOpacity(tibiaSurface, 0.5f);
	cout << "test 12" << endl;
	if (existingLegForceLineNode.IsNotNull())
	{
		// 如果节点已存在，更新其数据
		existingLegForceLineNode->SetData(legForceLine);
	}
	else
	{
		// 如果节点不存在，创建新的节点
		mitk::DataNode::Pointer legForceLineNode = mitk::DataNode::New();
		legForceLineNode->SetData(legForceLine);
		legForceLineNode->SetName("legForceLine");

		// 可以在此处添加其他属性设置，例如显示属性
		legForceLineNode->SetBoolProperty("show contour", true);
		legForceLineNode->SetFloatProperty("contoursize", 1.0);
		legForceLineNode->SetVisibility(true);
		// 将新节点添加到数据存储中
		GetDataStorage()->Add(legForceLineNode);
	}
	cout << "test 13" << endl;
	// 计算并更新力线占比值
	updateProportation();
	cout << "test 14" << endl;
	// 更新图像
	GetDataStorage()->Modified();
	mitk::RenderingManager::GetInstance()->RequestUpdateAll();
	return true;
}

bool HTONDI::OnShowMachineLine02Clicked()
{
	/* 计算并显示规划的下肢力线
	// 选中 股骨头中心 和 踝关节中心新位置
	// ===================================
	// 股骨头中心点 由计算拟合得出
	// 踝关节中心由 内外踝 计算中点得出
	// ===================================
	*/
	
	m_Controls.textBrowser_Action->append("Action: Show The pro Line of Force.");

	// 直接从库中加载股骨头中心点
	cout << "test 01" << endl;
	mitk::DataNode::Pointer hipCenterNode = GetDataStorage()->GetNamedNode("hipCenterPoint");
	auto hipCenterPoint = dynamic_cast<mitk::PointSet*>(hipCenterNode->GetData())->GetPoint(0);
	cout << "test 02" << endl;
	// 获取胫骨踝点集
	mitk::DataNode::Pointer tibiaLandmarkNode = GetDataStorage()->GetNamedNode("tibiaDistalPointSet");
	auto tibiaLandmarkPointSet = dynamic_cast<mitk::PointSet*>(tibiaLandmarkNode->GetData());
	cout << "test 03" << endl;
	// 获取第三和第四个点
	mitk::Point3D anklePoint1 = tibiaLandmarkPointSet->GetPoint(0);
	mitk::Point3D anklePoint2 = tibiaLandmarkPointSet->GetPoint(1);
	cout << "test 04" << endl;
	// 计算踝关节中心点
	mitk::Point3D ankleCenterPoint;
	ankleCenterPoint[0] = (anklePoint1[0] + anklePoint2[0]) / 2;
	ankleCenterPoint[1] = (anklePoint1[1] + anklePoint2[1]) / 2;
	ankleCenterPoint[2] = (anklePoint1[2] + anklePoint2[2]) / 2;
	cout << "test 05" << endl;
	// 可视化踝关节中心点
	// 在图像上绘制踝关节中心点

	// 首先删除原有的
	auto ankleCenter = GetDataStorage()->GetNamedNode("ankleCenterPointNew");
	if (ankleCenter != nullptr) {
		cout << "test 06" << endl;
		GetDataStorage()->Remove(ankleCenter);
	}

	cout << "test 07" << endl;
	mitk::PointSet::Pointer ankleCenterPointSet = mitk::PointSet::New();
	ankleCenterPointSet->InsertPoint(0, ankleCenterPoint);
	mitk::DataNode::Pointer ankleCenterNode = mitk::DataNode::New();
	ankleCenterNode->SetData(ankleCenterPointSet);
	ankleCenterNode->SetName("ankleCenterPointNew");
	ankleCenterNode->SetProperty("color", mitk::ColorProperty::New(0.0, 0.0, 1.0)); // 设置颜色为红色
	ankleCenterNode->SetProperty("pointsize", mitk::FloatProperty::New(5.0)); // 设置点的大小为5
	GetDataStorage()->Add(ankleCenterNode);

	cout << "test 08" << endl;
	// 计算下肢力线方向
	mitk::Vector3D legForceDirection = ankleCenterPoint - hipCenterPoint;

	cout << "test 09" << endl;
	// 创建新的点集
	mitk::PointSet::Pointer legForceLine = mitk::PointSet::New();
	legForceLine->SetPoint(0, hipCenterPoint);
	legForceLine->SetPoint(1, ankleCenterPoint);
	legForceLine->SetProperty("color", mitk::ColorProperty::New(0.0, 0.0, 1.0));

	cout << "test 10" << endl;
	// 查询是否存在名为"LegForceLine"的节点
	mitk::DataNode::Pointer existingLegForceLineNode = GetDataStorage()->GetNamedNode("legForceLineNew");
	cout << "test 11" << endl;
	// 设置下肢骨骼透明度
	auto tibiaSurface = GetDataStorage()->GetNamedNode("tibiaSurface");
	SetModelOpacity(tibiaSurface, 0.5f);
	cout << "test 12" << endl;
	if (existingLegForceLineNode.IsNotNull())
	{
		// 如果节点已存在，更新其数据
		existingLegForceLineNode->SetData(legForceLine);
	}
	else
	{
		// 如果节点不存在，创建新的节点
		mitk::DataNode::Pointer legForceLineNode = mitk::DataNode::New();
		legForceLineNode->SetData(legForceLine);
		legForceLineNode->SetName("legForceLineNew");

		// 可以在此处添加其他属性设置，例如显示属性
		legForceLineNode->SetBoolProperty("show contour", true);
		legForceLineNode->SetFloatProperty("contoursize", 1.0);

		legForceLineNode->SetVisibility(true);
		// 将新节点添加到数据存储中
		GetDataStorage()->Add(legForceLineNode);
	}
	cout << "test 13" << endl;
	// 计算并更新力线占比值
	updateProportation02();
	cout << "test 14" << endl;
	// 更新图像
	GetDataStorage()->Modified();
	mitk::RenderingManager::GetInstance()->RequestUpdateAll();
	return true;
}

bool HTONDI::OnUnshowMachineLineClicked()
{
	/* 显示/隐藏 下肢力线 */

	// 隐藏下肢力线
	m_Controls.textBrowser_Action->append("Action: Unshow The Line of Force.");
	// 从数据存储中获取下肢力线节点
	mitk::DataNode::Pointer legForceLineNode = GetDataStorage()->GetNamedNode("legForceLine");
	if (legForceLineNode.IsNotNull())
	{
		// 切换下肢力线节点的可见性
		bool currentVisibility = legForceLineNode->IsVisible(nullptr, "visible", true);
		legForceLineNode->SetVisibility(!currentVisibility);
	}

	GetDataStorage()->Modified();
	mitk::RenderingManager::GetInstance()->RequestUpdateAll();
	return true;
}

bool HTONDI::OnUnshowMachineLine02Clicked()
{
	// 隐藏下肢力线
	m_Controls.textBrowser_Action->append("Action: Unshow The pro Line of Force.");
	// 从数据存储中获取下肢力线节点
	mitk::DataNode::Pointer legForceLineNode = GetDataStorage()->GetNamedNode("legForceLineNew");
	if (legForceLineNode.IsNotNull())
	{
		// 切换下肢力线节点的可见性
		bool currentVisibility = legForceLineNode->IsVisible(nullptr, "visible", true);
		legForceLineNode->SetVisibility(!currentVisibility);
	}

	GetDataStorage()->Modified();
	mitk::RenderingManager::GetInstance()->RequestUpdateAll();
	return true;
}

void HTONDI::updateProportation()
{
	// 计算力线在胫骨平台占比
	cout << "test 01" << endl;
	m_Controls.textBrowser_Action->append("Show Force Line Rate");
	// 直接从库中加载 股骨中心点 和 踝关节中心点
	mitk::DataNode::Pointer ankleCenter = GetDataStorage()->GetNamedNode("ankleCenterPoint");
	auto ankleCenterPoint = dynamic_cast<mitk::PointSet*>(ankleCenter->GetData());
	cout << "test 02" << endl;
	mitk::DataNode::Pointer hipCenter = GetDataStorage()->GetNamedNode("hipCenterPoint");
	auto hipCenterPoint = dynamic_cast<mitk::PointSet*>(hipCenter->GetData());
	cout << "test 03" << endl;
	// 取得力线的两端点
	auto ForcePoint1 = ankleCenterPoint->GetPoint(0);
	auto ForcePoint2 = hipCenterPoint->GetPoint(0);
	cout << "test 04" << endl;
	// 从胫骨踝点集中取得胫骨平台两端点
	mitk::DataNode::Pointer tibiaLandmarkNode = GetDataStorage()->GetNamedNode("tibiaLandmarkPointSet");
	auto tibiaLandmarkPointSet = dynamic_cast<mitk::PointSet*>(tibiaLandmarkNode->GetData());
	cout << "test 05" << endl;
	// 获取胫骨近端 外侧点 + 内侧点
	auto Point_0 = tibiaLandmarkPointSet->GetPoint(0);
	auto Point_1 = tibiaLandmarkPointSet->GetPoint(1);

	// 计算胫骨平台的宽度 = abs(x1 - x2) 或者 开放根
	double tibia_length = sqrt(pow(Point_0[0] - Point_1[0], 2));

	// ============================
	//力线的长度
	Line_length = sqrt(pow(ForcePoint1[0] - ForcePoint2[0], 2)
		+ pow(ForcePoint1[1] - ForcePoint2[1], 2)
		+ pow(ForcePoint1[2] - ForcePoint2[2], 2));
	// ============================

	// 计算水平线段, 高度取胫骨近端内外侧两点的平均高度
	double coronalZ = (Point_0[2] + Point_1[2]) / 2;

	// XoZ 平面 (x, z)
	// 创建一个二维平面上的水平线段, 线段的起点和终点分别是在冠状面上投影的两个点
	Eigen::Vector2d line1ProjStart = { Point_0[0], coronalZ };
	Eigen::Vector2d line1ProjEnd = { Point_1[0], coronalZ };

	// 计算倾斜线段
	Eigen::Vector2d line2ProjStart = { ForcePoint1[0], ForcePoint1[2] };
	Eigen::Vector2d line2ProjEnd = { ForcePoint2[0], ForcePoint2[2] };

	cout << "test 06" << endl;
	// 使用VTK数学库计算二维空间内两线段的交点
	Eigen::Vector2d intersection;
	if (LineLineIntersection(intersection, line1ProjStart, line1ProjEnd, line2ProjStart, line2ProjEnd)) {
		cout << "test 07" << endl;
		// 计算交点在水平线段上的位置
		double propatation = sqrt(pow(intersection[0] - Point_0[0], 2)) / sqrt(pow(Point_1[0] - Point_0[0], 2));
		cout << "propatation = " << propatation << endl;
		propatation = ceil(propatation * 100);
		m_Controls.textBrowser_Action->append(QString::number(propatation) + "%");
		m_Controls.showForceLine_textEdit->setText(QString::number(propatation) + "%");
	}
	else {
		cout << "test 08" << endl;
		std::cout << "The lines do not intersect in the coronal plane." << std::endl;
		m_Controls.textBrowser_Action->append(QString::number(0) + "%");
		m_Controls.showForceLine_textEdit->setText(QString::number(0) + "%");
	}
}

void HTONDI::updateProportation02()
{
	// 计算新力线在胫骨平台占比
	cout << "test 01" << endl;
	m_Controls.textBrowser_Action->append("Show Force Line Rate");
	// 直接从库中加载 股骨中心点 和 踝关节中心点
	mitk::DataNode::Pointer ankleCenter = GetDataStorage()->GetNamedNode("ankleCenterPointNew");
	auto ankleCenterPoint = dynamic_cast<mitk::PointSet*>(ankleCenter->GetData());
	cout << "test 02" << endl;
	mitk::DataNode::Pointer hipCenter = GetDataStorage()->GetNamedNode("hipCenterPoint");
	auto hipCenterPoint = dynamic_cast<mitk::PointSet*>(hipCenter->GetData());
	cout << "test 03" << endl;
	// 取得力线的两端点
	auto ForcePoint1 = ankleCenterPoint->GetPoint(0);
	auto ForcePoint2 = hipCenterPoint->GetPoint(0);
	cout << "test 04" << endl;
	// 从胫骨踝点集中取得胫骨平台两端点
	mitk::DataNode::Pointer tibiaLandmarkNode = GetDataStorage()->GetNamedNode("tibiaLandmarkPointSet");
	auto tibiaLandmarkPointSet = dynamic_cast<mitk::PointSet*>(tibiaLandmarkNode->GetData());
	cout << "test 05" << endl;
	// 获取胫骨近端 外侧点 + 内侧点
	auto Point_0 = tibiaLandmarkPointSet->GetPoint(0);
	auto Point_1 = tibiaLandmarkPointSet->GetPoint(1);

	// 计算胫骨平台的宽度 = abs(x1 - x2) 或者 开放根
	double tibia_length = sqrt(pow(Point_0[0] - Point_1[0], 2));

	// 计算水平线段, 高度取胫骨近端内外侧两点的平均高度
	double coronalZ = (Point_0[2] + Point_1[2]) / 2;

	// XoZ 平面 (x, z)
	// 创建一个二维平面上的水平线段, 线段的起点和终点分别是在冠状面上投影的两个点
	Eigen::Vector2d line1ProjStart = { Point_0[0], coronalZ };
	Eigen::Vector2d line1ProjEnd = { Point_1[0], coronalZ };

	// 计算倾斜线段
	Eigen::Vector2d line2ProjStart = { ForcePoint1[0], ForcePoint1[2] };
	Eigen::Vector2d line2ProjEnd = { ForcePoint2[0], ForcePoint2[2] };

	cout << "test 06" << endl;
	// 使用VTK数学库计算二维空间内两线段的交点
	Eigen::Vector2d intersection;
	if (LineLineIntersection(intersection, line1ProjStart, line1ProjEnd, line2ProjStart, line2ProjEnd)) {
		cout << "test 07" << endl;
		// 计算交点在水平线段上的位置
		double propatation = sqrt(pow(intersection[0] - Point_0[0], 2)) / sqrt(pow(Point_1[0] - Point_0[0], 2));
		cout << "propatation = " << propatation << endl;
		propatation = ceil(propatation * 100);
		m_Controls.textBrowser_Action->append(QString::number(propatation) + "%");
		m_Controls.showForceLine02_textEdit->setText(QString::number(propatation) + "%");
	}
	else {
		cout << "test 08" << endl;
		std::cout << "The lines do not intersect in the coronal plane." << std::endl;
		m_Controls.textBrowser_Action->append(QString::number(0) + "%");
		m_Controls.showForceLine02_textEdit->setText(QString::number(0) + "%");
	}
}

bool HTONDI::LineLineIntersection(Eigen::Vector2d& intersection, Eigen::Vector2d p1, Eigen::Vector2d p2, Eigen::Vector2d q1, Eigen::Vector2d q2)
{
	// 计算空间中两个线段的交点
	// 输入：两个线段的起点和终点 p1, p2, q1, q2
	// 输出：交点 intersection

	// 计算向量
	Eigen::Vector2d r = p2 - p1;
	Eigen::Vector2d s = q2 - q1;

	// 计算叉积
	double rxs = r.x() * s.y() - r.y() * s.x();
	double qpxr = (q1 - p1).x() * r.y() - (q1 - p1).y() * r.x();

	// 检查是否平行
	if (rxs == 0 && qpxr == 0) {
		// 共线
		return false;
	}
	if (rxs == 0 && qpxr != 0) {
		// 平行且不共线
		return false;
	}

	// 计算参数 t 和 u
	double t = ((q1 - p1).x() * s.y() - (q1 - p1).y() * s.x()) / rxs;
	double u = ((q1 - p1).x() * r.y() - (q1 - p1).y() * r.x()) / rxs;

	// 检查交点是否在线段上
	if (rxs != 0 && t >= 0 && t <= 1 && u >= 0 && u <= 1) {
		intersection = p1 + t * r;
		return true;
	}

	// 没有交点
	return false;
}

void HTONDI::CaculateStrechHeigh()
{
	// 将角度转换为弧度
	std::cout << "angleInDegrees: " << angleInDegrees << std::endl;
	double angle_rad = angleInDegrees * M_PI / 180.0;
	std::cout << "angle_rad: " << angle_rad << std::endl;
	double height = round(sin(angle_rad) * depth);
	m_Controls.LineEdit_height->setText(QString::number(height));
}

// 截骨效应
bool HTONDI::GetIntersectionLine()
{
	/* 实时获取截骨线 
		原理: 利用截骨平面和骨表面进行切割计算，得到一组点云的交集
	
	*/
	auto cutPlaneNode = GetDataStorage()->GetNamedNode("1st cut plane");
	auto tibiaSurfaceNode = GetDataStorage()->GetNamedNode("tibiaSurface");
	if (cutPlaneNode == nullptr)
	{
		m_Controls.textBrowser_Action->append("'tibia cut plane' is not ready");
		return false;
	}
	if (tibiaSurfaceNode == nullptr)
	{
		m_Controls.textBrowser_Action->append("'tibiaSurface' is not ready");
		return false;
	}
	auto mitkCutSurface = dynamic_cast<mitk::Surface*>(cutPlaneNode->GetData());

	auto tibiaMitkSurface = dynamic_cast<mitk::Surface*>(tibiaSurfaceNode->GetData());

	auto tmpVtkSurface_initial = mitkCutSurface->GetVtkPolyData();

	auto tibiaVtkSurface_initial = tibiaMitkSurface->GetVtkPolyData();

	// 用于复制出一个新的图像，而不对原始数据产生不好的影像
	// 复制截骨平面
	vtkNew<vtkTransform> cutPlaneTransform;
	cutPlaneTransform->SetMatrix(mitkCutSurface->GetGeometry()->GetVtkMatrix());
	vtkNew<vtkTransformFilter> cutPlaneTransformFilter;
	cutPlaneTransformFilter->SetTransform(cutPlaneTransform);
	cutPlaneTransformFilter->SetInputData(tmpVtkSurface_initial);
	cutPlaneTransformFilter->Update();

	// 得到了这新的数据
	vtkNew<vtkPolyData> tmpVtkSurface;
	tmpVtkSurface->DeepCopy(cutPlaneTransformFilter->GetPolyDataOutput());

	// 复制骨表面
	vtkNew<vtkTransform> tibiaTransform;
	tibiaTransform->SetMatrix(tibiaMitkSurface->GetGeometry()->GetVtkMatrix());
	vtkNew<vtkTransformFilter> tibiaTransformFilter;
	tibiaTransformFilter->SetTransform(tibiaTransform);
	tibiaTransformFilter->SetInputData(tibiaVtkSurface_initial);
	tibiaTransformFilter->Update();

	vtkNew<vtkPolyData> tibiaVtkSurface;
	tibiaVtkSurface->DeepCopy(tibiaTransformFilter->GetPolyDataOutput());

	// 计算截骨平面的当前法向量和平面中心
	double surfaceNormal[3];
	double cutPlaneCenter[3];

	GetPlaneProperty(tmpVtkSurface, surfaceNormal, cutPlaneCenter);


	// 使用获取的法向量和原点创建一个vtkPlane对象
	vtkSmartPointer<vtkPlane> cutPlane = vtkSmartPointer<vtkPlane>::New();
	cutPlane->SetNormal(surfaceNormal);
	cutPlane->SetOrigin(cutPlaneCenter);

	// 然后将这个cutPlane设置给vtkCutter
	vtkSmartPointer<vtkCutter> cutter_plane = vtkSmartPointer<vtkCutter>::New();
	cutter_plane->SetCutFunction(cutPlane);
	//设置输入数据
	cutter_plane->SetInputData(tibiaVtkSurface); // tibiaModelPolyData 是表示胫骨的vtkPolyData对象
	//更新获取交线
	cutter_plane->Update();
	vtkSmartPointer<vtkPolyData> intersectionLine = cutter_plane->GetOutput();
	TraverseIntersectionLines(intersectionLine);//获取切割平面与tibiaPolyData的交线以及特定点

	return true;
}

void HTONDI::TraverseIntersectionLines(vtkSmartPointer<vtkPolyData> intersectionLine)
{
	// 获取截骨线最左最右侧点

	if (!planeAndTibiaIntersectionPoint)
	{
		planeAndTibiaIntersectionPoint = mitk::PointSet::New();
	}
	planeAndTibiaIntersectionPoint->Clear();

	// 获取点集
	vtkSmartPointer<vtkPoints> points = intersectionLine->GetPoints();

	// 获取点的数量
	int numberOfPoints = points->GetNumberOfPoints();
	double minX = std::numeric_limits<double>::max();
	int minXIndex = -1;
	double maxX = -std::numeric_limits<double>::max(); // 初始化为小于可能的最小x值
	int maxXIndex = -1;

	std::cout << "Intersection Line Points:" << std::endl;
	for (int i = 0; i < numberOfPoints; ++i)
	{
		double point[3];
		points->GetPoint(i, point); // 获取第i个点的坐标

		std::cout << "Point " << i << ": (" << point[0] << ", " << point[1] << ", " << point[2] << ")" << std::endl;
		//检查当前点的x坐标是否小于已知的最小x值
		if (point[0] < minX)
		{
			minX = point[0]; // 更新最小x值
			minXIndex = i;   // 记录最小x值对应的点的索引
		}
		if (point[0] > maxX)
		{
			maxX = point[0];
			maxXIndex = i;
		}
	}
	if (minXIndex != -1 && maxXIndex != -1)
	{
		std::cout << "The point with the smallest x-coordinate is Point " << minXIndex << ": ("
			<< minX << ", " << points->GetPoint(minXIndex)[1] << ", " << points->GetPoint(minXIndex)[2] << ")" << std::endl;

		std::cout << "The point with the largest x-coordinate is Point " << maxXIndex << ": ("
			<< maxX << ", " << points->GetPoint(maxXIndex)[1] << ", " << points->GetPoint(maxXIndex)[2] << ")" << std::endl;
	}
	else
	{
		std::cout << "No points were processed." << std::endl;
	}

	////存储找到的截骨面与胫骨模型的交线上的最左侧点和入口点；
	minPoint[0] = points->GetPoint(minXIndex)[0];
	minPoint[1] = points->GetPoint(minXIndex)[1];
	minPoint[2] = points->GetPoint(minXIndex)[2];

	maxPoint[0] = points->GetPoint(maxXIndex)[0];
	maxPoint[1] = points->GetPoint(maxXIndex)[1];
	maxPoint[2] = points->GetPoint(maxXIndex)[2];


	planeAndTibiaIntersectionPoint->InsertPoint(0, minPoint);
	planeAndTibiaIntersectionPoint->InsertPoint(1, maxPoint);

	// 删除之前的测试节点
	auto tmpCutLine = GetDataStorage()->GetNamedNode("planeAndTibiaIntersectionPoint");
	if (tmpCutLine != nullptr) {
		GetDataStorage()->Remove(tmpCutLine);
	}
	mitk::DataNode::Pointer cutLine = mitk::DataNode::New();
	cutLine->SetName("planeAndTibiaIntersectionPoint");
	cutLine->SetColor(1.0, 0.0, 0.0);                                  // Set color to red
	cutLine->SetData(planeAndTibiaIntersectionPoint);                                // Add poinset into datanode
	cutLine->SetProperty("pointsize", mitk::FloatProperty::New(10.0)); // Change the point size
	mitk::DataStorage::Pointer dataStorage = this->GetDataStorage();
	dataStorage->Add(cutLine);
	dataStorage->Modified();

}

void HTONDI::GetDistancefromTibia()
{
	//获取平面位置到距离胫骨内外侧平台的距离以及保留外侧合页
	auto tmpPointSet2 = dynamic_cast<mitk::PointSet*>(GetDataStorage()->GetNamedNode("tibiaLandmarkPointSet")->GetData());
	auto Point_0 = tmpPointSet2->GetPoint(0);
	auto Point_1 = tmpPointSet2->GetPoint(1);

	// 0:右腿. 1：左腿
	if (judgModel_flag == 0)
	{
		cout << "left Limb" << endl;
		distance1 = Point_0[2] - maxPoint[2];//截骨面末端距外侧平台
		distance2 = sqrt(pow((Point_1[0] - minPoint[0]), 2) +
			pow((Point_1[1] - minPoint[1]), 2) +
			pow((Point_1[2] - minPoint[2]), 2));//截骨面入口距内侧平台
		distance3 = sqrt(pow((maxPoint[0] - mitkPointSet1->GetPoint(3)[0]), 2) +
			pow((maxPoint[1] - mitkPointSet1->GetPoint(3)[1]), 2) +
			pow((maxPoint[2] - mitkPointSet1->GetPoint(3)[2]), 2));//保留外侧合页
		if (mitkPointSet1->GetPoint(3)[0] > maxPoint[0]) {
			depth = abs(maxPoint[0] - minPoint[0]);//入骨深度
		}
		else {
			depth = abs(mitkPointSet1->GetPoint(3)[0] - minPoint[0]);//入骨深度
		}

		std::cout << "mitkPointSet1： " << ": (" << mitkPointSet1->GetPoint(3)[0] << ", " << mitkPointSet1->GetPoint(3)[1] << ", " << mitkPointSet1->GetPoint(3)[2] << ")" << std::endl;
		m_Controls.LineEdit_distance1->setText(QString::number(distance1));
		m_Controls.LineEdit_distance2->setText(QString::number(distance2));
		m_Controls.LineEdit_hy->setText(QString::number(distance3));
		m_Controls.LineEdit_depth->setText(QString::number(depth));
	}
	else {
		// 0:右腿. 1：左腿
		if (judgModel_flag == 1)
		{
			cout << "Right Limb" << endl;
			distance1 = Point_0[2] - minPoint[2];//截骨面末端距外侧平台
			distance2 = sqrt(pow((Point_1[0] - maxPoint[0]), 2) +
				pow((Point_1[1] - maxPoint[1]), 2) +
				pow((Point_1[2] - maxPoint[2]), 2));//截骨面入口距内侧平台
			distance3 = sqrt(pow((minPoint[0] - mitkPointSet1->GetPoint(0)[0]), 2) +
				pow((minPoint[1] - mitkPointSet1->GetPoint(0)[1]), 2) +
				pow((minPoint[2] - mitkPointSet1->GetPoint(0)[2]), 2));//保留外侧合页
			if (mitkPointSet1->GetPoint(0)[0] < minPoint[0]) {
				depth = abs(minPoint[0] - maxPoint[0]);//截骨面入骨深度
			}
			else {
				depth = abs(mitkPointSet1->GetPoint(0)[0] - maxPoint[0]);//截骨面入骨深度
			}
			std::cout << "mitkPointSet1： " << ": (" << mitkPointSet1->GetPoint(0)[0] << ", " << mitkPointSet1->GetPoint(0)[1] << ", " << mitkPointSet1->GetPoint(0)[2] << ")" << std::endl;
			m_Controls.LineEdit_distance1->setText(QString::number(distance1));
			m_Controls.LineEdit_distance2->setText(QString::number(distance2));
			m_Controls.LineEdit_hy->setText(QString::number(distance3));
			m_Controls.LineEdit_depth->setText(QString::number(depth));
		}
	}
}



