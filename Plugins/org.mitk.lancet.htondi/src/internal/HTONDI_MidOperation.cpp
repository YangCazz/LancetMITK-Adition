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
#include <QtCore\qmath.h>

// vtk
#include <vtkPlane.h>
#include <vtkPlaneSource.h>

/*=========================术中导航==============================
HTONDI_MidOperation.cpp
----------------------------------------------------------------
== 截骨导航
== 钻孔导航
== 术中力线验证
===============================================================*/






// 术中导航
void HTONDI::trackingObjectPos()
{
	/* 追踪并返回导航物体在相机视角下的转化矩阵 
		1. 股骨 FemurRF - m_MetrixCameraToFemurRF
		2. 胫骨 TibiaRF - m_MetrixCameraToTibiaRF
		3. 探针 ProbeRF - m_MetrixCameraToProbeRF
		4. 摆锯 SawRF   - m_MetrixCameraToSawRF
		5. 磨钻 DrillRF - m_MetrixCameraToDrillRF
	*/
	// 开始进行配准点采集
	// Pos_Tip = T_Camera2ProbeRF * pos_TipOnProbeRF
	auto femurIndex = m_VegaToolStorage->GetToolIndexByName("FemurRF");
	auto tibiaRFIndex = m_VegaToolStorage->GetToolIndexByName("TibiaRF");
	auto probeIndex = m_VegaToolStorage->GetToolIndexByName("ProbeRF");
	auto sawIndex = m_VegaToolStorage->GetToolIndexByName("SawRF");
	auto drillRFIndex = m_VegaToolStorage->GetToolIndexByName("DrillRF");

	// 获取探针针尖位置
	auto boneFemur = m_VegaToolStorage->GetToolByName("FemurRF");
	auto boneTibia = m_VegaToolStorage->GetToolByName("TibiaRF");
	auto toolProbe = m_VegaToolStorage->GetToolByName("ProbeRF");
	auto toolSaw = m_VegaToolStorage->GetToolByName("SawRF");
	auto toolDrill = m_VegaToolStorage->GetToolByName("DrillRF");

	// 取出探针针尖点在相机下的位置
	// 4x4齐次变换矩阵
	Eigen::Matrix4d T_Camera2FemurRF = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d T_Camera2TibiaRF = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d T_Camera2ProbeRF = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d T_Camera2SawRF = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d T_Camera2DrillRF = Eigen::Matrix4d::Identity();

	// 计算当前标定点位置
	Eigen::Vector4d CurrentFemurCheckPointOnImage;
	Eigen::Vector4d CurrentTibiaCheckPointOnImage;
	Eigen::Vector4d CurrentTipOnImage;
	Eigen::Vector4d CurrentSawPointsOnImage;
	Eigen::Vector4d CurrentDrillPointsOnImage;

	// 1. FemurRF
	// 股骨的存在主要计算截骨面夹角
	if (femurIndex != -1)
	{
		m_Controls.textBrowser_Action->append("Record Current FemurRF Pos:");
		mitk::NavigationData::Pointer pos_FemurRFOnCamera = m_VegaSource->GetOutput(probeIndex);
		if (pos_FemurRFOnCamera)
		{
			// 	构建T_Camera2TibiaRF
			Eigen::Vector3d position(
				pos_FemurRFOnCamera->GetPosition()[0], pos_FemurRFOnCamera->GetPosition()[1], pos_FemurRFOnCamera->GetPosition()[2]);
			Eigen::Quaterniond orientation(
				pos_FemurRFOnCamera->GetOrientation().r(), pos_FemurRFOnCamera->GetOrientation().x(),
				pos_FemurRFOnCamera->GetOrientation().y(), pos_FemurRFOnCamera->GetOrientation().z()
			);

			// 将3x3的旋转矩阵设置到4x4矩阵的左上角
			T_Camera2FemurRF.block<3, 3>(0, 0) = orientation.toRotationMatrix();
			// 将3x1的平移向量设置到4x4矩阵的右上角
			T_Camera2FemurRF.block<3, 1>(0, 3) = position;
		}

		// 计算Femur标定点在Camera下的坐标值
		// pos_FemurCheckPointOnCamera = T_Camera2FemurRF * pos_FemurCheckPointOnFemurRF
		Eigen::Vector4d CurrentFemurCheckPointOnCamera;
		CurrentFemurCheckPointOnCamera = T_Camera2FemurRF * m_FemurCheckPointOnFemurRF;
		m_CurrentFemurCheckPointOnCamera = CurrentFemurCheckPointOnCamera;
		m_Controls.textBrowser_Action->append("Added CurrentFemurCheckPointOnCamera: ("
			     + QString::number(m_CurrentFemurCheckPointOnCamera[0]) +
			", " + QString::number(m_CurrentFemurCheckPointOnCamera[1]) +
			", " + QString::number(m_CurrentFemurCheckPointOnCamera[2]) +
			", " + QString::number(m_CurrentFemurCheckPointOnCamera[3]) + ")"
		);

		// 计算Femur标定点在Image下的坐标
		// Pos_FemurCheckPointOnImage = T_FemurRF2M.inverse() * T_Camera2FemurRF.inverse() * pos_FemurCheckPointOnCamera
		CurrentFemurCheckPointOnImage = m_Metrix4dFemurRFToImage.inverse() * T_Camera2FemurRF.inverse() * m_CurrentFemurCheckPointOnCamera;
		m_CurrentFemurCheckPointOnImage = CurrentFemurCheckPointOnImage;
		m_Controls.textBrowser_Action->append("Added CurrentFemurCheckPointOnImage: ("
			     + QString::number(m_CurrentFemurCheckPointOnImage[0]) +
			", " + QString::number(m_CurrentFemurCheckPointOnImage[1]) +
			", " + QString::number(m_CurrentFemurCheckPointOnImage[2]) +
			", " + QString::number(m_CurrentFemurCheckPointOnImage[3]) + ")"
		);

		// 进行图像更新计算


		// 应用图像更新转化



	}
	

	// 2. TibiaRF
	// 胫骨
	if (femurIndex != -1)
	{
		m_Controls.textBrowser_Action->append("Record Current TibiaRF Pos:");
		mitk::NavigationData::Pointer pos_TibiaRFOnCamera = m_VegaSource->GetOutput(probeIndex);
		if (pos_TibiaRFOnCamera)
		{
			// 	构建T_Camera2TibiaRF
			Eigen::Vector3d position(
				pos_TibiaRFOnCamera->GetPosition()[0], pos_TibiaRFOnCamera->GetPosition()[1], pos_TibiaRFOnCamera->GetPosition()[2]);
			Eigen::Quaterniond orientation(
				pos_TibiaRFOnCamera->GetOrientation().r(), pos_TibiaRFOnCamera->GetOrientation().x(),
				pos_TibiaRFOnCamera->GetOrientation().y(), pos_TibiaRFOnCamera->GetOrientation().z()
			);

			// 将3x3的旋转矩阵设置到4x4矩阵的左上角
			T_Camera2TibiaRF.block<3, 3>(0, 0) = orientation.toRotationMatrix();
			// 将3x1的平移向量设置到4x4矩阵的右上角
			T_Camera2TibiaRF.block<3, 1>(0, 3) = position;
		}

		// 计算Tibia标定点在Camera下的坐标值
		// Pos_TibiaCheckPoint = T_Camera2TibiaRF * pos_TibiaCheckPointOnTibiaRF
		Eigen::Vector4d CurrentFemurCheckPointOnCamera;
		CurrentFemurCheckPointOnCamera = T_Camera2TibiaRF * m_TibiaCheckPointOnTibiaRF;
		m_CurrentTibiaCheckPointOnCamera = CurrentFemurCheckPointOnCamera;
		m_Controls.textBrowser_Action->append("Added CurrentTibiaCheckPoint: ("
			     + QString::number(m_CurrentTibiaCheckPointOnCamera[0]) +
			", " + QString::number(m_CurrentTibiaCheckPointOnCamera[1]) +
			", " + QString::number(m_CurrentTibiaCheckPointOnCamera[2]) +
			", " + QString::number(m_CurrentTibiaCheckPointOnCamera[3]) + ")"
		);

		// 计算Tibia标定点在Image下的坐标
		// Pos_TibiaCheckPointOnImage = T_TibiaRF2M.inverse() * T_Camera2TibiaRF.inverse() * pos_TibiaCheckPointOnCamera
		CurrentTibiaCheckPointOnImage = m_Metrix4dTibiaRFToImage.inverse() * T_Camera2TibiaRF.inverse() * m_CurrentTibiaCheckPointOnCamera;
		m_CurrentTibiaCheckPointOnImage = CurrentTibiaCheckPointOnImage;
		m_Controls.textBrowser_Action->append("Added CurrentFemurCheckPointOnImage: ("
			     + QString::number(m_CurrentTibiaCheckPointOnImage[0]) +
			", " + QString::number(m_CurrentTibiaCheckPointOnImage[1]) +
			", " + QString::number(m_CurrentTibiaCheckPointOnImage[2]) +
			", " + QString::number(m_CurrentTibiaCheckPointOnImage[3]) + ")"
		);

		// 进行图像更新计算


		// 应用图像更新转化



	}
	

	// 3. ProbeRF
	// 探针RF用于识别探针位置
	if (femurIndex != -1)
	{
		m_Controls.textBrowser_Action->append("Record Current ProbeRF Pos:");
		// 获取探针针尖位置
		mitk::Point3D pos_TipOnProbeRF = toolProbe->GetToolTipPosition();
		// 转化为4d
		Eigen::Vector4d pos_TipOnProbeRF4d(pos_TipOnProbeRF[0], pos_TipOnProbeRF[1], pos_TipOnProbeRF[2], 1.0);
		mitk::NavigationData::Pointer pos_ProbeRFOnCamera = m_VegaSource->GetOutput(probeIndex);
		if (pos_ProbeRFOnCamera)
		{
			// 	构建T_Camera2TibiaRF
			Eigen::Vector3d position(
				pos_ProbeRFOnCamera->GetPosition()[0], pos_ProbeRFOnCamera->GetPosition()[1], pos_ProbeRFOnCamera->GetPosition()[2]);
			Eigen::Quaterniond orientation(
				pos_ProbeRFOnCamera->GetOrientation().r(), pos_ProbeRFOnCamera->GetOrientation().x(),
				pos_ProbeRFOnCamera->GetOrientation().y(), pos_ProbeRFOnCamera->GetOrientation().z()
			);

			// 将3x3的旋转矩阵设置到4x4矩阵的左上角
			T_Camera2ProbeRF.block<3, 3>(0, 0) = orientation.toRotationMatrix();
			// 将3x1的平移向量设置到4x4矩阵的右上角
			T_Camera2ProbeRF.block<3, 1>(0, 3) = position;
		}
		cout << "pos_TipOnProbeRF: (" << pos_TipOnProbeRF[0] << " , " << pos_TipOnProbeRF[1] << " , " << pos_TipOnProbeRF[2] << ")" << endl;
		
		// 计算探针尖端在Camera下的坐标值
		// Pos_Tip = T_Camera2ProbeRF * pos_TipOnProbeRF
		Eigen::Vector4d pos_TipOnCamera = T_Camera2ProbeRF * pos_TipOnProbeRF4d;

		cout << "pos_TipOnCamera: (" << pos_TipOnCamera[0] << " , " << pos_TipOnCamera[1] << " , " << pos_TipOnCamera[2] << ")" << endl;

		// 计算ProbeTip在Camera下的坐标值
		// Pos_Tip = T_Camera2ProbeRF * pos_TipOnProbeRF
		pos_TipOnCamera = T_Camera2ProbeRF * pos_TipOnProbeRF4d;
		m_CurrentTipOnCamera = pos_TipOnCamera;
		m_Controls.textBrowser_Action->append("Added m_CurrentTipOnCamera: ("
			+ QString::number(m_CurrentTipOnCamera[0]) +
			", " + QString::number(m_CurrentTipOnCamera[1]) +
			", " + QString::number(m_CurrentTipOnCamera[2]) +
			", " + QString::number(m_CurrentTipOnCamera[3]) + ")"
		);

		// 计算ProbeTip在Image下的坐标
		// Pos_TipOnImage = T_TibiaRF2M.inverse() * T_Camera2TibiaRF.inverse() * pos_TibiaCheckPointOnCamera
		CurrentTipOnImage = m_Metrix4dTibiaRFToImage.inverse() * T_Camera2TibiaRF.inverse() * m_CurrentTipOnCamera;
		m_CurrentTipOnImage = CurrentTipOnImage;
		m_Controls.textBrowser_Action->append("Added CurrentTipOnImage: ("
			     + QString::number(m_CurrentTipOnImage[0]) +
			", " + QString::number(m_CurrentTipOnImage[1]) +
			", " + QString::number(m_CurrentTipOnImage[2]) +
			", " + QString::number(m_CurrentTipOnImage[3]) + ")"
		);

		// 进行图像更新计算， 利用上个位置和当前位置来进行更替
		// T_last2recent = N_pointsOnM_recent * N_pointsOnM_last.inverse()
		// 对于探针，则直接改变探针位置
		auto probeRf = GetDataStorage()->GetNamedNode("ProbeRF");
		auto probe = GetDataStorage()->GetNamedNode("Probe");
		mitk::Point3D pos_tip;
		pos_tip[0] = CurrentTipOnImage[0];
		pos_tip[1] = CurrentTipOnImage[1];
		pos_tip[2] = CurrentTipOnImage[2];

		// 应用图像更新转化
		//probeRf->GetData()->GetGeometry()->SetOrigin(pos_tip);
		probe->GetData()->GetGeometry()->SetOrigin(pos_tip);
	}


	// 4. SawRF
	// 用于截骨导航
	if (femurIndex != -1)
	{
		m_Controls.textBrowser_Action->append("Record Current SawRF Pos:");
		mitk::NavigationData::Pointer pos_SawRFOnCamera = m_VegaSource->GetOutput(probeIndex);
		if (pos_SawRFOnCamera)
		{
			// 	构建T_Camera2TibiaRF
			Eigen::Vector3d position(
				pos_SawRFOnCamera->GetPosition()[0], pos_SawRFOnCamera->GetPosition()[1], pos_SawRFOnCamera->GetPosition()[2]);
			Eigen::Quaterniond orientation(
				pos_SawRFOnCamera->GetOrientation().r(), pos_SawRFOnCamera->GetOrientation().x(),
				pos_SawRFOnCamera->GetOrientation().y(), pos_SawRFOnCamera->GetOrientation().z()
			);

			// 将3x3的旋转矩阵设置到4x4矩阵的左上角
			T_Camera2SawRF.block<3, 3>(0, 0) = orientation.toRotationMatrix();
			// 将3x1的平移向量设置到4x4矩阵的右上角
			T_Camera2SawRF.block<3, 1>(0, 3) = position;
		}

		// 计算Saw标定点在Camera下的坐标值
		// Pos_SawPoints = T_Camera2SawRF * pos_SawPointsOnTibiaRF
		for (int i = 0; i < m_SawPointsOnSawRF.size(); i++) {
			Eigen::Vector4d SawPoint_tmp = T_Camera2SawRF * m_SawPointsOnSawRF[i];
			// CurrentSawPoints.push_back(SawPoint);
			m_Controls.textBrowser_Action->append("Added CurrentSawPoint: ("
				+ QString::number(SawPoint_tmp[0]) +
				", " + QString::number(SawPoint_tmp[1]) +
				", " + QString::number(SawPoint_tmp[2]) +
				", " + QString::number(SawPoint_tmp[3]) + ")"
			);
			// 记录当前在相机下的位置
			m_CurrentSawPointsOnCamera.push_back(SawPoint_tmp);
			// 计算并记录在影像空间的位置
			Eigen::Vector4d SawPoint = m_Metrix4dTibiaRFToImage.inverse() * T_Camera2TibiaRF.inverse() * SawPoint_tmp;
			m_CurrentSawPointsOnImage.push_back(SawPoint);
		}
		
		// 进行图像更新计算， 利用上个位置和当前位置来进行更替
		// T_last2recent = N_pointsOnM_recent * N_pointsOnM_last.inverse()
		// 对于摆锯，去计算当前图像留存点到投影点的位置




		// 应用图像更新转化

		// 已知 A B C三点在自己坐标系下的坐标
		// D F 为摆锯边缘点，根据向量计算方程
		// AD = aAB + bAC => a = 403/628, b = -437/628
		// AF = cAB + dAC => c = 733/628, d = -131/628



	}
	

	// 5. DrillRF
	// 用于磨钻导航
	if (femurIndex != -1)
	{
		m_Controls.textBrowser_Action->append("Record Current DrillRF Pos:");
		mitk::NavigationData::Pointer pos_DrillRFOnCamera = m_VegaSource->GetOutput(probeIndex);
		if (pos_DrillRFOnCamera)
		{
			// 	构建T_Camera2TibiaRF
			Eigen::Vector3d position(
				pos_DrillRFOnCamera->GetPosition()[0], pos_DrillRFOnCamera->GetPosition()[1], pos_DrillRFOnCamera->GetPosition()[2]);
			Eigen::Quaterniond orientation(
				pos_DrillRFOnCamera->GetOrientation().r(), pos_DrillRFOnCamera->GetOrientation().x(),
				pos_DrillRFOnCamera->GetOrientation().y(), pos_DrillRFOnCamera->GetOrientation().z()
			);

			// 将3x3的旋转矩阵设置到4x4矩阵的左上角
			T_Camera2DrillRF.block<3, 3>(0, 0) = orientation.toRotationMatrix();
			// 将3x1的平移向量设置到4x4矩阵的右上角
			T_Camera2DrillRF.block<3, 1>(0, 3) = position;
		}

		// 计算Drill标定点在Camera下的坐标值
		// Pos_DrillPoints = T_Camera2DrillRF * pos_DrillPointsOnDrillRF
		for (int i = 0; i < m_DrillPointsOnDrillRF.size(); i++) {
			Eigen::Vector4d DrillPoint_tmp = T_Camera2DrillRF * m_DrillPointsOnDrillRF[i];
			// CurrentSawPoints.push_back(SawPoint);
			m_Controls.textBrowser_Action->append("Added CurrentDrillPoint: ("
				     + QString::number(DrillPoint_tmp[0]) +
				", " + QString::number(DrillPoint_tmp[1]) +
				", " + QString::number(DrillPoint_tmp[2]) +
				", " + QString::number(DrillPoint_tmp[3]) + ")"
			);

			// 记录当前在相机下的位置
			m_CurrentSawPointsOnCamera.push_back(DrillPoint_tmp);
			// 计算并记录在影像空间的位置
			Eigen::Vector4d DrillPoint = m_Metrix4dTibiaRFToImage.inverse() * T_Camera2TibiaRF.inverse() * DrillPoint_tmp;
			m_CurrentSawPointsOnImage.push_back(DrillPoint);

		}
		
		// 进行图像更新计算


		// 应用图像更新转化



	}
}



void HTONDI::GenerateRealTimeBoneSurface()
{
	/* 生成实时的截骨面
	1. 找到器械上的标定点，计算法向量和前端点位置，生成截骨面
	2. 计算实时面和规划面的夹角
	*/

	// 取出摆锯上的几个标定点位 => 计算摆锯的法向量
	mitk::Point3D PlanePoint1, PlanePoint2, PlanePoint3;
	
	if (saw_image) {
		auto SawPointSet = saw_image->GetLandmarks();
		// 由远及近
		PlanePoint1 = SawPointSet->GetPoint(0);
		PlanePoint2 = SawPointSet->GetPoint(1);
		PlanePoint3 = SawPointSet->GetPoint(2);
	}
	else
	{
		m_Controls.textBrowser_Action->append("saw_image Not init, stop record real time saw!");
		m_timer_saw->stop();
		return;
	}
	
	// 计算平面的法向量，根据摆锯上的三个标定点位计算平面的法向量
	Eigen::Vector3d normalVector;
	Eigen::Vector3d vector1 = Eigen::Vector3d(PlanePoint2[0] - PlanePoint1[0], PlanePoint2[1] - PlanePoint1[1], PlanePoint2[2] - PlanePoint1[2]);
	Eigen::Vector3d vector2 = Eigen::Vector3d(PlanePoint3[0] - PlanePoint1[0], PlanePoint3[1] - PlanePoint1[1], PlanePoint3[2] - PlanePoint1[2]);
	normalVector = vector1.cross(vector2).normalized();
	
	// 法向量
	double normal[3]
	{
		normalVector[0],
		normalVector[1],
		normalVector[2]
	};

	// 计算平面的中心点为 几何中心
	double origin[3]
	{
		(PlanePoint1[0] + PlanePoint1[0] + PlanePoint1[0]) / 3,
		(PlanePoint1[1] + PlanePoint1[1] + PlanePoint1[1]) / 3,
		(PlanePoint1[2] + PlanePoint1[2] + PlanePoint1[2]) / 3
	};

	// 创建一个初始的截骨平面，并定义位置和方向
	auto realTimeSawPlaneSource = vtkSmartPointer<vtkPlaneSource>::New();
	// 生成初始平面
	realTimeSawPlaneSource->SetOrigin(0, 0, 0);

	// 设定可视化截骨平面的大小，70*70
	realTimeSawPlaneSource->SetPoint1(0, 70, 0);
	realTimeSawPlaneSource->SetPoint2(70, 0, 0);
	realTimeSawPlaneSource->SetNormal(normal);//设置法向量

	// 将初始构建的截骨平面移动到计算出来的位置上来
	realTimeSawPlaneSource->SetCenter(origin);

	// 添加入库
	auto realTimeSawPlane = mitk::Surface::New();
	realTimeSawPlane->SetVtkPolyData(realTimeSawPlaneSource->GetOutput());

	// 创建实时平面
	auto tmpPlane = GetDataStorage()->GetNamedNode("realTimeCutPlane");
	if (tmpPlane)
	{
		tmpPlane->SetData(realTimeSawPlane);
	}
	else 
	{
		// 创建截骨平面DataNode对象
		auto planeNode = mitk::DataNode::New();
		planeNode->SetData(realTimeSawPlane);
		planeNode->SetColor(1.0, 0.0, 0.0);
		planeNode->SetOpacity(0.5);
		planeNode->SetName("realTimeCutPlane");
		GetDataStorage()->Add(planeNode);
	}
	
	// 计算锯片最前端的位置，计算截骨深度

	// 计算两个平面的夹角
	auto cutPlaneSource01 = GetDataStorage()->GetNamedNode("1st cut plane")->GetData();
}

void HTONDI::CalculateRealTimeCutAngle()
{
	/* 计算截骨面的夹角 */

}


bool HTONDI::OnStartAxialGuideClicked()
{
	/* 开始水平截骨导航
	1. 生成实时截骨面
	2. 计算 实时截骨面 与 规划截骨面 的夹角

	开始水平截骨导航时，需要计算截骨面的法向量和前端位置
	那么，也就需要知道实时的摆锯最前端的截骨面点以及对应的截骨面法向量
	*/

	m_Controls.textBrowser_Action->append("Action: Start Axial Cut Guide.");

	// 1. 显示规划的截骨面 + 显示实时截骨面位置
	auto preCutPlane01 = GetDataStorage()->GetNamedNode("1st cut plane");
	auto realTimeSaw = GetDataStorage()->GetNamedNode("Saw");

	// 检测数据存在，然后打开截骨导航
	if (preCutPlane01 && realTimeSaw)
	{
		preCutPlane01->SetVisibility(true);
		realTimeSaw->SetVisibility(true);
	}
	else
	{
		m_Controls.textBrowser_Action->append("Cut plane or Saw model Not Found!");
		return false;
	}

	// 对Saw生成实时截骨平面
	if (m_timer_saw == nullptr)
	{
		m_timer_saw = new QTimer(this);
	}
	m_Controls.textBrowser_Action->append("Generate Real time Cut plane");
	connect(m_timer_saw, SIGNAL(timeout()), this, SLOT(GenerateRealTimeBoneSurface()));
	m_timer_saw->start(100);

	return true;
}

bool HTONDI::OnStartAxialCutClicked()
{
	/* 启动水平截骨
	1. 改变摆锯截骨状态
	2. 计算实时 截骨深度
	3. 进行截骨保护
	*/

	// 开始水平截骨，启动摆锯，启动截骨导航
	m_Controls.textBrowser_Action->append("Action: Check Drill Node.");

	return true;
}

bool HTONDI::OnStartStateAxialCutClicked()
{
	/* 静态截骨模拟，生成静态平面
		1. 获取静态模型表面标定点
		2. 生成初始的静态模型代表的截骨面
		3. 生成截骨平面初始点, 然后应用到当前相对与初始的法向量
		4. 计算当前截骨面夹角

	采用物体控制来模拟截骨过程
	*/
	m_Controls.textBrowser_Action->append("Action: Generate State Axial Cut Plane.");

	// 设置截骨方法的状态
	m_cutType = 1;

	// 1. 获取当前静态模型表面数据和标定点的当前位置
	mitk::Point3D PlanePoint1, PlanePoint2, PlanePoint3;
	auto SawPoints = GetDataStorage()->GetNamedNode("SawLandMarkPointSet");
	auto Saw = GetDataStorage()->GetNamedNode("Saw");

	// 由远及近
	if (SawPoints) {
		// 由远及近，取出三个点
		auto SawPointSet = dynamic_cast<mitk::PointSet*>(SawPoints->GetData());
		PlanePoint1 = SawPointSet->GetPoint(0);
		PlanePoint2 = SawPointSet->GetPoint(1);
		PlanePoint3 = SawPointSet->GetPoint(2);
		// 由远及近，取出三个点
		SawPoints->SetVisibility(true);
	}
	else
	{
		m_Controls.textBrowser_Action->append("SawLandMarkPointSet Not Found.");
		return false;
	}
	if (Saw) {
		// 由远及近，取出三个点
		Saw->SetVisibility(true);
	}
	else
	{
		m_Controls.textBrowser_Action->append("Saw model Not Found.");
		return false;
	}

	// 计算当前摆锯平面的 法向量 normalSaw ，根据摆锯上的三个标定点位计算平面的法向量
	Eigen::Vector3d normalVector;
	Eigen::Vector3d vector1 = Eigen::Vector3d(PlanePoint2[0] - PlanePoint1[0], PlanePoint2[1] - PlanePoint1[1], PlanePoint2[2] - PlanePoint1[2]);
	Eigen::Vector3d vector2 = Eigen::Vector3d(PlanePoint3[0] - PlanePoint1[0], PlanePoint3[1] - PlanePoint1[1], PlanePoint3[2] - PlanePoint1[2]);
	normalVector = vector1.cross(vector2).normalized();

	// 法向量
	double normalSaw[3]
	{
		normalVector[0],
		normalVector[1],
		normalVector[2]
	};

	// 计算平面的中心点为 几何中心, 将其位置移动到摆锯最前端
	double originSaw[3]
	{
		(PlanePoint1[0] + PlanePoint2[0] + PlanePoint3[0]) / 3,
		(PlanePoint1[1] + PlanePoint2[1] + PlanePoint3[1]) / 3,
		(PlanePoint1[2] + PlanePoint2[2] + PlanePoint3[2]) / 3
	};


	// 2. 生成摆锯截骨平面
	// 创建一个初始的截骨平面，并定义位置和方向
	auto realTimeSawPlaneSource = vtkSmartPointer<vtkPlaneSource>::New();

	// 生成初始平面
	//realTimeSawPlaneSource->SetOrigin(0, 0, 0);
	
	// 设定可视化截骨平面的大小，70*70
	realTimeSawPlaneSource->SetPoint1(0, 70, 0);
	realTimeSawPlaneSource->SetPoint2(70, 0, 0);


	// 将初始构建的截骨平面移动到计算出来的位置上来
	realTimeSawPlaneSource->SetCenter(originSaw);
	realTimeSawPlaneSource->SetNormal(normalSaw);

	// 图像更新
	realTimeSawPlaneSource->Update();

	// 添加入库
	auto realTimeSawPlane = mitk::Surface::New();
	realTimeSawPlane->SetVtkPolyData(realTimeSawPlaneSource->GetOutput());

	// 创建实时平面
	auto tmpPlane = GetDataStorage()->GetNamedNode("StateCutPlane01");
	if (tmpPlane)
	{
		tmpPlane->SetData(realTimeSawPlane);
		m_Controls.textBrowser_Action->append("Plane updated.");
	}
	else
	{
		// 创建截骨平面DataNode对象
		auto planeNode = mitk::DataNode::New();
		planeNode->SetData(realTimeSawPlane);
		planeNode->SetColor(1.0, 1.0, 0.0);
		planeNode->SetOpacity(0.5);
		planeNode->SetName("StateCutPlane01");

		// 添加到库
		GetDataStorage()->Add(planeNode);
		m_Controls.textBrowser_Action->append("Plane created.");
	}

	// 计算锯片最前端的位置，计算截骨深度

	// 3. 生成截骨面最前端点，同术前规划方法

	// 可视化节点
	auto tmpNodes = GetDataStorage()->GetNamedNode("pointSetInRealPlaneAxial");
	if (tmpNodes) {
		//GetDataStorage()->Remove(tmpNodes);
		/* 如果不是第一次生成摆锯平面
			1. 取出当前的原点和法向量
			2. 依据存储的相对位置来计算实际位置
			3. 更新节点数据
		*/
		mitk::Point3D point0;
		mitk::Point3D point1;
		mitk::Point3D point2;
		mitk::Point3D point3;
		mitk::Point3D point4;
		mitk::Point3D planeCenterPoint;//水平面中心点
		// 使用 CalculateActualPoints 函数计算实际位置
		std::vector<mitk::Point3D> actualPoints = CalculateActualPoints(m_PointsOnSaw, normalVector, Eigen::Vector3d(originSaw[0], originSaw[1], originSaw[2]));
		mitkPointSetRealTime->SetPoint(0, point0);//第一截骨面末端中点标记
		mitkPointSetRealTime->SetPoint(1, point1);
		mitkPointSetRealTime->SetPoint(2, point2);
		mitkPointSetRealTime->SetPoint(3, point3);
		mitkPointSetRealTime->SetPoint(4, point4);
		// 更新数据
		tmpNodes->SetData(mitkPointSetRealTime);
	}
	else 
	{
		/* 如果是第一次生成摆锯的平面
			1. 初始化为水平位置(依据模型初始位置)
			2. 计算并存储平面关键点在摆锯平面下的位置
			3. 然后进行可视化更新

		注：这里默认初始加载的摆锯是水平
		*/

		/* 1. 水平面位置初始化 */
		// 定义截骨末端合页点的初始坐标
		mitk::Point3D point0;
		mitk::Point3D point1;
		mitk::Point3D point2;
		mitk::Point3D point3;
		mitk::Point3D point4;
		mitk::Point3D planeCenterPoint;//水平面中心点

		// 点 1 - 前端中点
		point0[0] = originSaw[0] - 35;
		point0[1] = originSaw[1];
		point0[2] = originSaw[2];
		// 点 2 - 前端左侧
		point1[0] = originSaw[0] - 35;
		point1[1] = originSaw[1] - 35;
		point1[2] = originSaw[2];
		// 点 3 - 前端右侧
		point2[0] = originSaw[0] - 35;
		point2[1] = originSaw[1] + 35;
		point2[2] = originSaw[2];
		// 点 4 - 末端左侧
		point3[0] = originSaw[0] + 35;
		point3[1] = originSaw[1] - 35;
		point3[2] = originSaw[2];
		// 点 5 - 末端中点 
		point4[0] = originSaw[0] + 35;
		point4[1] = originSaw[1];
		point4[2] = originSaw[2];
		// 点 6 - 设置原点
		planeCenterPoint[0] = originSaw[0];
		planeCenterPoint[1] = originSaw[1];
		planeCenterPoint[2] = originSaw[2];

		// 计算这些点相对于摆锯坐标系的位置
		m_PointsOnSaw = CalculateRelativePoints({ point0, point1, point2, point3, point4 }, 
			normalVector, Eigen::Vector3d(originSaw[0], originSaw[1], originSaw[2]));

		// 将坐标添加到mitk::PointSet中
		// 用于上升截骨面的旋转计算
		mitkPointSetRealTime->InsertPoint(0, point0);//第一截骨面末端中点标记
		mitkPointSetRealTime->InsertPoint(1, point1);
		mitkPointSetRealTime->InsertPoint(2, point2);
		mitkPointSetRealTime->InsertPoint(3, point3);
		mitkPointSetRealTime->InsertPoint(4, point4);
		mitkPointSetRealTime->InsertPoint(5, planeCenterPoint);//平面原点
		// 否则创建新的平面
		mitk::DataNode::Pointer pointSetInPlaneCutPlane = mitk::DataNode::New();
		pointSetInPlaneCutPlane->SetName("pointSetInRealPlaneAxial");
		// 红色，大小 5.0
		pointSetInPlaneCutPlane->SetColor(1.0, 0.0, 0.0);
		pointSetInPlaneCutPlane->SetData(mitkPointSetRealTime);
		pointSetInPlaneCutPlane->SetFloatProperty("pointsize", 5.0);
		GetDataStorage()->Add(pointSetInPlaneCutPlane);
	}
	


	// 4. 计算两个平面的夹角

	// 提取水平截骨面的法向量
	auto cutPlaneSource01 = dynamic_cast<mitk::Surface*>(GetDataStorage()->GetNamedNode("1st cut plane")->GetData());
	auto cutPlanePolyData01 = cutPlaneSource01->GetVtkPolyData();

	Eigen::Vector3d normalPlane01;
	normalPlane01 = ExtractNormalFromPlane("1st cut plane");

	// 输出法向量情况
	cout << "normalPlane: " << " " << normalPlane01[0] << " " << normalPlane01[1] << " " << normalPlane01[2] << endl;
	cout << "normalSaw  : " << " " << normalSaw[0] << " " << normalSaw[1] << " " << normalSaw[2] << endl;

	//cout << "CenterPlane: " << " " << centerPlane01[0] << " " << centerPlane01[1] << " " << centerPlane01[2] << endl;
	cout << "CenterSaw  : " << " " << originSaw[0] << " " << originSaw[1] << " " << originSaw[2] << endl;

	// 计算两个方向向量之间的夹角
	double dotProduct = normalPlane01[0] * normalSaw[0] + normalPlane01[1] * normalSaw[1] + normalPlane01[2] * normalSaw[2]; // 点积

	// 计算向量长度
	double magnitude1 = sqrt(normalPlane01[0] * normalPlane01[0] + normalPlane01[1] * normalPlane01[1] + normalPlane01[2] * normalPlane01[2]);
	double magnitude2 = sqrt(normalSaw[0] * normalSaw[0] + normalSaw[1] * normalSaw[1] + normalSaw[2] * normalSaw[2]);

	// 计算夹角的余弦值
	double cosAngle = dotProduct / (magnitude1 * magnitude2);

	// 使用反余弦函数计算夹角（弧度）
	double angleInRadians = acos(cosAngle);

	// 将弧度转换为度数，保留1位小数
	double angleInDegrees = round(angleInRadians * (180.0 / M_PI) * 10) / 10;
	if (angleInDegrees > 90)
	{
		angleInDegrees = 180.0 - angleInDegrees;
	}

	// 输出夹角到实际的位置
	m_Controls.textBrowser_AxialCut->append("Real time Angle Miss: " + QString::number(angleInDegrees));

	GetDataStorage()->Modified();
	mitk::RenderingManager::GetInstance()->RequestUpdateAll();
	return true;
}

bool HTONDI::OnCheckStateCutClicked()
{
	/* 确定水平截骨面位置	*/
	m_Controls.textBrowser_Action->append("Action: Check State Cut Plane.");
	m_cutType = -1;
	return true;
}


bool HTONDI::OnStartSagGuideClicked()
{
	/* 确定水平截骨面位置	*/
	m_Controls.textBrowser_Action->append("Action: Check State Cut Plane.");
	m_cutType = -1;
	return true;
}

bool HTONDI::OnStartStateSagCutClicked()
{
	/* 静态截骨模拟，生成静态平面
		1. 获取静态模型表面标定点
		2. 生成初始的静态模型代表的截骨面
		3. 生成截骨平面初始点
		4. 计算当前截骨面夹角

	采用物体控制来模拟截骨过程
	*/

	m_Controls.textBrowser_Action->append("Action: Generate State Sag Cut Plane.");

	// 设置截骨方法的状态
	m_cutType = 2;

	// 1. 获取静态模型表面的标定点
	mitk::Point3D PlanePoint1, PlanePoint2, PlanePoint3;
	auto SawPoints = GetDataStorage()->GetNamedNode("SawLandMarkPointSet");
	auto Saw = GetDataStorage()->GetNamedNode("Saw");

	// 由远及近
	if (SawPoints) {
		// 由远及近，取出三个点
		auto SawPointSet = dynamic_cast<mitk::PointSet*>(SawPoints->GetData());
		PlanePoint1 = SawPointSet->GetPoint(0);
		PlanePoint2 = SawPointSet->GetPoint(1);
		PlanePoint3 = SawPointSet->GetPoint(2);
		// 由远及近，取出三个点
		SawPoints->SetVisibility(true);
	}
	else
	{
		m_Controls.textBrowser_Action->append("SawLandMarkPointSet Not Found.");
		return false;
	}
	if (Saw) {
		// 由远及近，取出三个点
		Saw->SetVisibility(true);
	}
	else
	{
		m_Controls.textBrowser_Action->append("Saw model Not Found.");
		return false;
	}

	// 计算摆锯平面的 法向量 normalSaw ，根据摆锯上的三个标定点位计算平面的法向量
	Eigen::Vector3d normalVector;
	Eigen::Vector3d vector1 = Eigen::Vector3d(PlanePoint2[0] - PlanePoint1[0], PlanePoint2[1] - PlanePoint1[1], PlanePoint2[2] - PlanePoint1[2]);
	Eigen::Vector3d vector2 = Eigen::Vector3d(PlanePoint3[0] - PlanePoint1[0], PlanePoint3[1] - PlanePoint1[1], PlanePoint3[2] - PlanePoint1[2]);
	normalVector = vector1.cross(vector2).normalized();

	// 法向量
	double normalSaw[3]
	{
		normalVector[0],
		normalVector[1],
		normalVector[2]
	};

	// 计算平面的中心点为 几何中心, 将其位置移动到摆锯最前端
	double originSaw[3]
	{
		(PlanePoint1[0] + PlanePoint1[0] + PlanePoint1[0]) / 3 + 25,
		(PlanePoint1[1] + PlanePoint1[1] + PlanePoint1[1]) / 3,
		(PlanePoint1[2] + PlanePoint1[2] + PlanePoint1[2]) / 3
	};


	// 2. 生成摆锯截骨平面
	// 创建一个初始的截骨平面，并定义位置和方向
	auto realTimeSawPlaneSource = vtkSmartPointer<vtkPlaneSource>::New();

	// 生成初始平面
	realTimeSawPlaneSource->SetOrigin(0, 0, 0);

	// 设定可视化截骨平面的大小，70*70
	realTimeSawPlaneSource->SetPoint1(0, 70, 0);
	realTimeSawPlaneSource->SetPoint2(70, 0, 0);
	realTimeSawPlaneSource->SetNormal(0, 0, 1);//设置法向量

	// 将初始构建的截骨平面移动到计算出来的位置上来
	realTimeSawPlaneSource->SetCenter(originSaw);
	realTimeSawPlaneSource->SetNormal(normalSaw);
	

	// 图像更新
	realTimeSawPlaneSource->Update();

	// 添加入库
	auto realTimeSawPlane = mitk::Surface::New();
	realTimeSawPlane->SetVtkPolyData(realTimeSawPlaneSource->GetOutput());

	// 创建实时平面
	auto tmpPlane = GetDataStorage()->GetNamedNode("StateCutPlane02");
	if (tmpPlane)
	{
		tmpPlane->SetData(realTimeSawPlane);
		m_Controls.textBrowser_Action->append("Plane updated.");
	}
	else
	{
		// 创建截骨平面DataNode对象
		auto planeNode = mitk::DataNode::New();
		planeNode->SetData(realTimeSawPlane);
		planeNode->SetColor(1.0, 1.0, 0.0);
		planeNode->SetOpacity(0.5);
		planeNode->SetName("StateCutPlane02");

		// 添加到库
		GetDataStorage()->Add(planeNode);
		m_Controls.textBrowser_Action->append("Plane created.");
	}

	// 计算锯片最前端的位置，计算截骨深度

	// 3. 生成截骨面最前端点，同术前规划方法

	// 定义截骨末端合页点的初始坐标
	mitk::Point3D point0;
	mitk::Point3D point1;
	mitk::Point3D point2;
	mitk::Point3D point3;
	mitk::Point3D point4;
	mitk::Point3D planeCenterPoint;//水平面中心点

	// 点 1 - 前端中点
	point0[0] = originSaw[0] - 35;
	point0[1] = originSaw[1];
	point0[2] = originSaw[2];
	// 点 2 - 前端左侧
	point1[0] = originSaw[0] - 35;
	point1[1] = originSaw[1] - 35;
	point1[2] = originSaw[2];
	// 点 3 - 前端右侧
	point2[0] = originSaw[0] - 35;
	point2[1] = originSaw[1] + 35;
	point2[2] = originSaw[2];
	// 点 4 - 末端左侧
	point3[0] = originSaw[0] + 35;
	point3[1] = originSaw[1] - 35;
	point3[2] = originSaw[2];
	// 点 5 - 末端中点 
	point4[0] = originSaw[0] + 35;
	point4[1] = originSaw[1];
	point4[2] = originSaw[2];

	// 点 6 - 设置原点
	planeCenterPoint[0] = originSaw[0];
	planeCenterPoint[1] = originSaw[1];
	planeCenterPoint[2] = originSaw[2];


	// 将坐标添加到mitk::PointSet中
	// 用于上升截骨面的旋转计算
	mitkPointSetRealTime->InsertPoint(0, point0);//第一截骨面末端中点标记
	mitkPointSetRealTime->InsertPoint(1, point1);
	mitkPointSetRealTime->InsertPoint(2, point2);
	mitkPointSetRealTime->InsertPoint(3, point3);
	mitkPointSetRealTime->InsertPoint(4, point4);
	mitkPointSetRealTime->InsertPoint(4, planeCenterPoint);//平面原点

	// 可视化节点
	auto tmpNodes = GetDataStorage()->GetNamedNode("pointSetInRealPlaneSag");
	if (tmpNodes) {
		//GetDataStorage()->Remove(tmpNodes);
		//// 如果已经创建则更新其信息
		tmpNodes->SetData(mitkPointSetRealTime);
	}
	else
	{
		// 否则创建新的平面
		mitk::DataNode::Pointer pointSetInPlaneCutPlane = mitk::DataNode::New();
		pointSetInPlaneCutPlane->SetName("pointSetInRealPlaneSag");
		// 红色，大小 5.0
		pointSetInPlaneCutPlane->SetColor(1.0, 0.0, 0.0);
		pointSetInPlaneCutPlane->SetData(mitkPointSetRealTime);
		pointSetInPlaneCutPlane->SetFloatProperty("pointsize", 5.0);
		GetDataStorage()->Add(pointSetInPlaneCutPlane);
	}



	// 4. 计算两个平面的夹角

	// 提取水平截骨面的法向量
	auto cutPlaneSource01 = dynamic_cast<mitk::Surface*>(GetDataStorage()->GetNamedNode("1st cut plane")->GetData());
	auto cutPlanePolyData01 = cutPlaneSource01->GetVtkPolyData();

	Eigen::Vector3d normalPlane01;
	normalPlane01 = ExtractNormalFromPlane("1st cut plane");

	// 输出法向量情况
	cout << "normalPlane: " << " " << normalPlane01[0] << " " << normalPlane01[1] << " " << normalPlane01[2] << endl;
	cout << "normalSaw  : " << " " << normalSaw[0] << " " << normalSaw[1] << " " << normalSaw[2] << endl;

	//cout << "CenterPlane: " << " " << centerPlane01[0] << " " << centerPlane01[1] << " " << centerPlane01[2] << endl;
	cout << "CenterSaw  : " << " " << originSaw[0] << " " << originSaw[1] << " " << originSaw[2] << endl;

	// 计算两个方向向量之间的夹角
	double dotProduct = normalPlane01[0] * normalSaw[0] + normalPlane01[1] * normalSaw[1] + normalPlane01[2] * normalSaw[2]; // 点积

	// 计算向量长度
	double magnitude1 = sqrt(normalPlane01[0] * normalPlane01[0] + normalPlane01[1] * normalPlane01[1] + normalPlane01[2] * normalPlane01[2]);
	double magnitude2 = sqrt(normalSaw[0] * normalSaw[0] + normalSaw[1] * normalSaw[1] + normalSaw[2] * normalSaw[2]);

	// 计算夹角的余弦值
	double cosAngle = dotProduct / (magnitude1 * magnitude2);

	// 使用反余弦函数计算夹角（弧度）
	double angleInRadians = acos(cosAngle);

	// 将弧度转换为度数，保留1位小数
	double angleInDegrees = round(angleInRadians * (180.0 / M_PI) * 10) / 10;
	if (angleInDegrees > 90)
	{
		angleInDegrees = 180.0 - angleInDegrees;
	}

	// 输出夹角到实际的位置
	m_Controls.textBrowser_SagCut->append("Real time Angle Miss: " + QString::number(angleInDegrees));

	GetDataStorage()->Modified();
	mitk::RenderingManager::GetInstance()->RequestUpdateAll();
	return true;


	return true;
}

bool HTONDI::OnStartSagCutClicked()
{
	/* 启动上升截骨
	1. 改变摆锯截骨状态
	2. 计算实时 截骨深度
	3. 进行截骨保护
	*/

	m_Controls.textBrowser_Action->append("Action: Check Drill Node.");

	return true;
}

bool HTONDI::OnStartDrillGuideClicked()
{
	/* 开始钻孔导航
	1. 生成实时磨钻位置
	2. 计算 实时钻孔位置 和 规划钻孔位置 差距
	*/

	m_Controls.textBrowser_Action->append("Action: Check Drill Node.");

	return true;
}

bool HTONDI::OnStartDrillHoleClicked()
{
	/* 启动钻孔
	1. 改变磨钻器械状态
	2. 计算实时 钻孔深度
	3. 进行钻孔保护
	*/

	m_Controls.textBrowser_Action->append("Action: Check Drill Node.");

	return true;
}

bool HTONDI::OnStartStateDrillHoleClicked()
{
	/* 启动钻孔
	1. 改变磨钻器械状态
	2. 计算实时 钻孔深度
	3. 进行钻孔保护
	*/

	m_Controls.textBrowser_Action->append("Action: Check Drill Node.");

	return true;
}



// 术后验证
bool HTONDI::OnShowResClicked()
{
	// 开始钻孔
	m_Controls.textBrowser_Action->append("Action: Check Drill Node.");

	return true;
}


bool HTONDI::OnUnshowResClicked()
{
	// 开始钻孔
	m_Controls.textBrowser_Action->append("Action: Check Drill Node.");

	return true;
}


bool HTONDI::OnCaculateErrorClicked()
{
	// 开始钻孔
	m_Controls.textBrowser_Action->append("Action: Check Drill Node.");

	return true;
}