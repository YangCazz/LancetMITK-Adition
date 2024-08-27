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

// mitk
#include <mitkAffineTransform3D.h>
#include <mitkMatrixConvert.h>
#include "mitkTrackingTool.h"

// lancet
#include <lancetRobotTrackingTool.h>


/*========================= 术中注册 ==================================
HTONDI_MidPlan.cpp
--01--------------------------------------------------------------
== 探针旋转标定
== 股骨/胫骨标记验证
== 装载器械配准点
== 摆锯标定
== 磨钻标定
== 标记点验证
--02--------------------------------------------------------------
== 装载骨骼配准点
== 股骨头中心计算
== 股骨粗配准
== 胫骨粗配准
== 胫骨精配准
== 标记点捕捉
=====================================================================*/

bool HTONDI::OnSelectPointerClicked()
{
	/* 选中探针工具 
	   探针工具名必须为:ProbeRF
	*/
	m_Controls.textBrowser_Action->append("Action: Select Pointer.");
	// 获取工具存储中的探针工具
	auto toolProbe = m_VegaToolStorage->GetToolByName("ProbeRF");
	if (toolProbe.IsNull())
	{
		MITK_ERROR << "ProbeRF tool not found.";
		return false;
	}

	// 赋值 m_ToolToCalibrate
	m_ToolToCalibrate = toolProbe;
	MITK_INFO << "Tool selected: " << m_ToolToCalibrate->GetToolName();
	return true;
}


bool HTONDI::OnCalibratePointerClicked()
{
	/* 探针标定过程
	1. 旋转采集探针转化矩阵信息 100处
	2. 球面拟合标定
	3. 输出标定误差

	探针工具名必须为:ProbeRF
	*/

	m_Controls.textBrowser_Action->append("Action: Collect Probe Pos.");

	// 开始采集接下来一段时间内的探针位置信息
	m_ProbePositions.clear();
	m_Transformations.clear();
	m_Controls.textBrowser_Action->append("Start collect probe data, please move slowly...");

	cout << "test 01" << endl;
	// 启动定时器，定期采集数据
	if (m_ProbeDataCollectionTimer == nullptr)
	{
		cout << "test 01-01" << endl;
		m_ProbeDataCollectionTimer = new QTimer(this);
		cout << "test 01-02" << endl;
		connect(m_ProbeDataCollectionTimer, &QTimer::timeout, this, &HTONDI::CollectProbeData);
	}
	cout << "test 03" << endl;
	m_ProbeDataCollectionTimer->start(100); // 每100ms采集一次数据

	cout << "test 02" << endl;
	// 等待数据收集完成
	QEventLoop loop;
	connect(m_ProbeDataCollectionTimer, &QTimer::timeout, &loop, [&]() {
		if (m_ProbePositions.size() >= number)
		{
			loop.quit();
		}
		});
	loop.exec();

	cout << "test 03" << endl;
	// 停止定时器
	m_ProbeDataCollectionTimer->stop();

	// 计算探针针尖位置
	if (!CalculateProbeTip())
	{
		m_Controls.textBrowser_Action->append("ERROR: Failed to calculate probe tip.");
		return false;
	}

	cout << "test 04" << endl;
	// 注册探针位置
	RegisterProbePosition();

	cout << "test 05" << endl;
	// 计算并显示标定误差
	double calibrationError = CalculateCalibrationError();
	m_Controls.textBrowser_Action->append("Calibration Error (RMS): " + QString::number(calibrationError));

	cout << "test 06" << endl;
	// 可视化采集到的点和球心
	// OnVisualizeCollectedPoints();

	return true;
}

bool HTONDI::CollectProbeData()
{
	/* 探针信息采集函数
	1. 每100ms采集一次当前探针位置
	2. 存储探针位置到全局变量

	探针工具名必须为:ProbeRF
	*/

	// 获取当前探针追踪器的位置
	cout << "test-1 01" << endl;
	auto probeIndex = m_VegaToolStorage->GetToolIndexByName("ProbeRF");
	if (probeIndex == -1)
	{
		m_Controls.textBrowser_Action->append("ERROR: ProbeRF tool not found.");
		return false;
	}

	cout << "test-1 02" << endl;
	mitk::NavigationData::Pointer nd_ndiToProbe = m_VegaSource->GetOutput(probeIndex);
	if (nd_ndiToProbe)
	{
		cout << "test-1 03" << endl;
		Eigen::Vector3d position(nd_ndiToProbe->GetPosition()[0], nd_ndiToProbe->GetPosition()[1], nd_ndiToProbe->GetPosition()[2]);

		Eigen::Quaterniond orientation(nd_ndiToProbe->GetOrientation().r(), nd_ndiToProbe->GetOrientation().x(), nd_ndiToProbe->GetOrientation().y(), nd_ndiToProbe->GetOrientation().z());


		cout << "test-1 04" << endl;
		// 构建4x4齐次变换矩阵
		Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
		// 将3x3的旋转矩阵设置到4x4矩阵的左上角
		transformation.block<3, 3>(0, 0) = orientation.toRotationMatrix();
		// 将3x1的平移向量设置到4x4矩阵的右上角
		transformation.block<3, 1>(0, 3) = position;

		cout << "test-1 05" << endl;
		m_ProbePositions.push_back(position);
		m_Transformations.push_back(transformation);
		m_Controls.textBrowser_PointerCalibration->append("Get probe pos: " + QString::number(m_ProbePositions.size()) + "/" + QString::number(number));
		m_Controls.textBrowser_ProbeState->append("Probe Pos " + QString::number(m_ProbePositions.size()) + "= (" +
			QString::number(position[0]) + ", " +
			QString::number(position[1]) + ", " +
			QString::number(position[2]) + ")");
	}
	if (m_ProbePositions.size() >= number)
	{
		cout << "test-2 01" << endl;
		m_Controls.textBrowser_PointerCalibration->append("Collect done");
		m_ProbeDataCollectionTimer->stop();
	}

	return true;
}

bool HTONDI::CalculateProbeTip()
{
	/* 球面拟合计算探针在探针RF下的位置
	1. 多组球面拟合，求球心均值，作为探针位置

	探针工具名必须为:ProbeRF
	*/
	m_Controls.textBrowser_Action->append("Action: Calculate Probe Tip.");

	// 将点分成20个一组
	//int groupSize = 100;
	int numGroups = m_ProbePositions.size() / groupSize;
	cout << "numGroups: " << numGroups << endl;
	std::vector<Eigen::Vector3d> centers;

	for (int i = 0; i < numGroups; ++i) {
		std::vector<Eigen::Vector3d> subsetPoints(m_ProbePositions.begin() + i * groupSize, m_ProbePositions.begin() + (i + 1) * groupSize);
		auto [center, _] = FitSphere(subsetPoints);
		centers.push_back(center);
	}

	// 计算球心的均值
	Eigen::Vector3d averageCenter(0, 0, 0);
	for (const auto& center : centers) {
		averageCenter += center;
	}
	averageCenter /= centers.size();
	m_Center = averageCenter;

	// 计算探针尖端在F_probe下的坐标值
	std::vector<Eigen::Vector3d> probeTips;
	for (const auto& T_cameraToProbe : m_Transformations)
	{
		Eigen::Vector4d p_homogeneous(averageCenter[0], averageCenter[1], averageCenter[2], 1.0);
		Eigen::Vector4d s_homogeneous = T_cameraToProbe.inverse() * p_homogeneous;
		Eigen::Vector3d s = s_homogeneous.head<3>();
		probeTips.push_back(s);
	}

	// 计算所有探针尖端坐标值的平均值
	Eigen::Vector3d averageTip(0, 0, 0);
	for (const auto& tip : probeTips)
	{
		averageTip += tip;
	}
	averageTip /= probeTips.size();
	m_ProbeTip = averageTip;

	m_Controls.textBrowser_Action->append("Probe tip calculated!");
	m_Controls.textBrowser_Action->append("Pos: (" +
		QString::number(m_Center(0)) + ", " +
		QString::number(m_Center(1)) + ", " +
		QString::number(m_Center(2)) + ")");

	m_Controls.textBrowser_ProbeState->append("Probe tip Pos: (" +
		QString::number(m_Center(0)) + ", " +
		QString::number(m_Center(1)) + ", " +
		QString::number(m_Center(2)) + ")");

	return true;
}


void HTONDI::RegisterProbePosition()
{
	/* 注册探针在探针RF下的位置 */

	if (m_ToolToCalibrate.IsNull())
	{
		MITK_ERROR << "No tool to calibrate!";
	}

	// 将 Eigen::Vector3d 转换为 mitk::Point3D
	mitk::Point3D probeTip;
	probeTip[0] = m_ProbeTip[0];
	probeTip[1] = m_ProbeTip[1];
	probeTip[2] = m_ProbeTip[2];

	// 更新工具存储中的工具信息
	m_ToolToCalibrate->SetToolTipPosition(probeTip);

	// 打印调试信息
	MITK_INFO << "Registered probe position: " << m_ProbeTip;
	MITK_INFO << "Tool tip position: " << m_ToolToCalibrate->GetToolTipPosition();

	//m_ToolToCalibrate->GetTrackingError();

}


double HTONDI::CalculateCalibrationError()
{
	/* 计算探针标定的误差 
	  对100个姿态中每个姿态计算实际的探针位置，
	  计算所有位置的探针尖端位置到拟合球心的位置
	*/
	if (m_ProbePositions.empty())
	{
		MITK_ERROR << "No probe positions recorded!";
		return -1.0;
	}

	// 计算探针针尖在双目相机坐标系下的位置
	std::vector<Eigen::Vector3d> probeTipsInCamera;
	for (const auto& T_cameraToProbe : m_Transformations)
	{
		// 将探针针尖位置转换为齐次坐标
		Eigen::Vector4d s_homogeneous(m_ProbeTip[0], m_ProbeTip[1], m_ProbeTip[2], 1.0);

		// 计算探针针尖在双目相机坐标系下的位置
		Eigen::Vector4d p_homogeneous = T_cameraToProbe * s_homogeneous;

		// 提取前3个元素，得到探针针尖在双目相机坐标系下的3D坐标
		Eigen::Vector3d p = p_homogeneous.head<3>();

		// 存储结果
		probeTipsInCamera.push_back(p);
	}

	// 计算每个采集到的位置和最终计算得到的球心位置之间的误差
	double sumSquaredError = 0.0;
	for (size_t i = 0; i < m_ProbePositions.size(); ++i)
	{
		double error = (m_Center - probeTipsInCamera[i]).norm();
		//cout << "Probe Tip in Camera: " << probeTipsInCamera[i].x() << ", " << probeTipsInCamera[i].y() << ", " << probeTipsInCamera[i].z() << endl;
		//cout << "current error " << i <<": "<< error << endl;
		sumSquaredError += error * error;
	}

	// 计算均方根误差（RMS）
	double rmsError = std::sqrt(sumSquaredError / m_ProbePositions.size());
	return rmsError;
}

// 从库中装载标定点信息
bool HTONDI::OnLoadDeviceLandmarkClicked()
{
	/* 从库中装载摆锯和磨钻的标定点信息
	1. 装载摆锯 + 摆锯标定点
	2. 装载磨钻 + 磨钻标定点

	====== 更新0823 ======
	不需要再注册表面，直接用于展示物体模型与数据注册
	*/
	m_Controls.textBrowser_Action->append("Action: Load Device Landmark Nodes.");

	saw_image = lancet::NavigationObject::New();
	drill_image = lancet::NavigationObject::New();

	// 摆锯Saw
	auto SawPoints = GetDataStorage()->GetNamedNode("SawLandMarkPointSet");
	if (SawPoints)
	{
		m_Controls.textBrowser_Action->append("load SawPoints.");
		SawPoints->SetVisibility(true);

		// 设置 摆锯 导航目标的标定点
		saw_image->SetLandmarks(dynamic_cast<mitk::PointSet*>(SawPoints->GetData()));

		// 将物体的参考阵列设计为标定点的坐标系
		//saw_image->SetReferencFrameName(SawPoints->GetName());
	}

	// 磨钻Drill
	auto DrillPoints = GetDataStorage()->GetNamedNode("DrillLandMarkPointSet");
	if (DrillPoints)
	{
		m_Controls.textBrowser_Action->append("load DrillPointSet.");
		DrillPoints->SetVisibility(true);

		// 设置 磨钻 导航目标的标定点
		drill_image->SetLandmarks(dynamic_cast<mitk::PointSet*>(DrillPoints->GetData()));

		// 将物体的参考阵列设计为标定点的坐标系
		//drill_image->SetReferencFrameName(DrillPoints->GetName());
	}

	// 可视化绑定
	auto Saw = GetDataStorage()->GetNamedNode("Saw");
	auto Drill = GetDataStorage()->GetNamedNode("Drill");

	if (Saw)
	{
		m_Controls.textBrowser_Action->append("load Saw.");
		Saw->SetVisibility(true);

		// 设置摆锯导航目标
		saw_image->SetDataNode(Saw);
	}
	if (Drill)
	{
		m_Controls.textBrowser_Action->append("load Drill.");
		Drill->SetVisibility(true);

		// 设置磨钻导航目标
		drill_image->SetDataNode(Drill);
	}
	// 然后隐藏所有其他无关模型？

	cout << "test load model" << endl;

	GetDataStorage()->Modified();
	mitk::RenderingManager::GetInstance()->RequestUpdateAll();
	return true;
}

bool HTONDI::OnLoadBonePointsClicked()
{
	/* 从库中装载股骨和胫骨的配准点信息，必须在连接视觉设备之后才能执行
	1. 装载股骨粗配准点
	2. 装载胫骨粗配准点
	3. 装载胫骨精配准点

	=======已检验=======
	*/
	m_Controls.textBrowser_Action->append("Action: Load Bone Landmark Nodes.");

	cout << "test 01" << endl;
	auto femurLandmark = GetDataStorage()->GetNamedNode("femurLandmarkPointSet");
	cout << "test 02" << endl;
	auto tibiaLandmark = GetDataStorage()->GetNamedNode("tibiaLandmarkPointSet");
	cout << "test 03" << endl;
	auto tibiaICP = GetDataStorage()->GetNamedNode("tibiaICPPointSet");
	cout << "test 04" << endl;
	auto hipCenter = GetDataStorage()->GetNamedNode("hipCenterPoint");
	
	// 需要先初始化
	femur_image = lancet::NavigationObject::New();
	tibia_image = lancet::NavigationObject::New();

	// 装载femur的landmark点，这里只装载了两个点，需要额外计算股骨头中心并装载
	// 股骨外踝点 股骨内踝点
	if (femurLandmark)
	{
		cout << "test 05" << endl;
		femur_image->SetDataNode(GetDataStorage()->GetNamedNode("femurSurface"));
		cout << "test 05 02" << endl;
		femur_image->SetLandmarks(dynamic_cast<mitk::PointSet*>(femurLandmark->GetData()));
	}
	if (tibiaLandmark)
	{
		cout << "test 06" << endl;
		tibia_image->SetDataNode(GetDataStorage()->GetNamedNode("tibiaSurface"));
		tibia_image->SetLandmarks(dynamic_cast<mitk::PointSet*>(tibiaLandmark->GetData()));
	}
	if (tibiaICP)
	{
		cout << "test 07" << endl;
		tibia_image->SetDataNode(GetDataStorage()->GetNamedNode("tibiaSurface"));
		tibia_image->SetIcpPoints(dynamic_cast<mitk::PointSet*>(tibiaICP->GetData()));
	}
	if (hipCenter)
	{
		cout << "test 07" << endl;
		auto landmark = femur_image->GetLandmarks();
		
		// 取出数据
		mitk::DataNode::Pointer hipCenter = GetDataStorage()->GetNamedNode("hipCenterPoint");
		auto hipCenterPointSet = dynamic_cast<mitk::PointSet*>(hipCenter->GetData());
		mitk::Point3D hipCenterPoint = hipCenterPointSet->GetPoint(0);

		// 插入landmark
		landmark->InsertPoint(hipCenterPoint);
	}

	GetDataStorage()->Modified();
	mitk::RenderingManager::GetInstance()->RequestUpdateAll();
	return true;
}


bool HTONDI::OnFemurCheckClicked()
{
	/* 标记点注册-股骨标记点
	1. 探针采点
	2. 注册标记点在股骨RF下的相对位置 m_TibiaCheckPointOnTibiaRF

	====== 更新0823 ======
	标记点验证的目的是确保参考RF刚性连接
	*/

	// 注册股骨上验证点的位置
	// 检测股骨RF，然后使用探针采集股骨上标记点位置
	m_Controls.textBrowser_Action->append("Action: Register Tibia Node.");

	auto probeIndex = m_VegaToolStorage->GetToolIndexByName("ProbeRF");
	auto femurRFIndex = m_VegaToolStorage->GetToolIndexByName("FemurRF");
	if (probeIndex == -1 || femurRFIndex == -1)
	{
		m_Controls.textBrowser_Action->append("There is no 'ProbeRF' or 'FemurRF' in the toolStorage!");
		return false;
	}

	// 获取探针针尖位置
	auto toolProbe = m_VegaToolStorage->GetToolByName("ProbeRF");
	mitk::Point3D pos_TipOnProbeRF = toolProbe->GetToolTipPosition();

	// 转化为4d
	Eigen::Vector4d pos_TipOnProbeRF4d(pos_TipOnProbeRF[0], pos_TipOnProbeRF[1], pos_TipOnProbeRF[2], 1.0);

	// 取出探针针尖点在相机下的位置
	// 4x4齐次变换矩阵
	// T_FemurRF2ProbeRF = T_Camera2FemurRF.inverse() * T_Camera2ProbeRF
	Eigen::Matrix4d T_Camera2FemurRF = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d T_Camera2ProbeRF = Eigen::Matrix4d::Identity();

	mitk::NavigationData::Pointer pos_ProbeRFOnCamera = m_VegaSource->GetOutput(probeIndex);
	if (pos_ProbeRFOnCamera)
	{
		// 	构建T_FemurRF2Camera
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

		// 打印信息
		m_Controls.textBrowser_Action->append("Added pos_ProbeRFOnCamera: "
			+ QString::number(position[0]) +
			"/ " + QString::number(position[1]) +
			"/ " + QString::number(position[2])
		);

		// 计算探针Tip在相机下的位置
		Eigen::Vector4d pos_TipOnCamera4d = T_Camera2ProbeRF * pos_TipOnProbeRF4d;

		m_Controls.textBrowser_Action->append("Added pos_TipOnCamera: "
			+ QString::number(pos_TipOnCamera4d[0]) +
			"/ " + QString::number(pos_TipOnCamera4d[1]) +
			"/ " + QString::number(pos_TipOnCamera4d[2])
		);
	}

	mitk::NavigationData::Pointer pos_FemurRFOnCamera = m_VegaSource->GetOutput(probeIndex);
	if (pos_FemurRFOnCamera)
	{
		// 	构建T_FemurRF2Camera
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

	// 计算探针尖端在FemurRF下的坐标值
	// Pos = T_Camera2FemurRF.inverse() * T_Camera2ProbeRF * pos_TipOnProbeRF
	Eigen::Vector4d pos_TipOnFemurRF4d = T_Camera2FemurRF.inverse() * T_Camera2ProbeRF * pos_TipOnProbeRF4d;
	// 完成注册
	m_FemurCheckPointOnFemurRF = pos_TipOnFemurRF4d;

	m_Controls.textBrowser_Action->append("Added m_FemurCheckPointOnFemurRF: ("
		     + QString::number(pos_TipOnFemurRF4d[0]) +
		", " + QString::number(pos_TipOnFemurRF4d[1]) +
		", " + QString::number(pos_TipOnFemurRF4d[2]) + ")"
	);
	return true;
}


bool HTONDI::OnTibiaCheckClicked()
{
	/* 标记点注册-胫骨标记点
	1. 探针采点
	2. 注册标记点在股骨RF下的相对位置 m_TibiaCheckPointOnTibiaRF

	====== 更新0823 ======
	标记点验证的目的是确保参考RF刚性连接
	*/

	// 注册股骨上验证点的位置
	// 检测股骨RF，然后使用探针采集股骨上标记点位置
	m_Controls.textBrowser_Action->append("Action: Register Tibia Node.");

	auto probeIndex = m_VegaToolStorage->GetToolIndexByName("ProbeRF");
	auto tibiaRFIndex = m_VegaToolStorage->GetToolIndexByName("TibiaRF");
	if (probeIndex == -1 || tibiaRFIndex == -1)
	{
		m_Controls.textBrowser_Action->append("There is no 'ProbeRF' or 'TibiaRF' in the toolStorage!");
		return false;
	}

	// 获取探针针尖位置
	auto toolProbe = m_VegaToolStorage->GetToolByName("ProbeRF");
	mitk::Point3D pos_TipOnProbeRF = toolProbe->GetToolTipPosition();

	// 转化为4d
	Eigen::Vector4d pos_TipOnProbeRF4d(pos_TipOnProbeRF[0], pos_TipOnProbeRF[1], pos_TipOnProbeRF[2], 1.0);

	// 取出探针针尖点在相机下的位置
	// 4x4齐次变换矩阵
	// T_TibiaRF2ProbeRF = T_Camera2TibiaRF.inverse() * T_Camera2ProbeRF
	Eigen::Matrix4d T_Camera2TibiaRF = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d T_Camera2ProbeRF = Eigen::Matrix4d::Identity();

	mitk::NavigationData::Pointer pos_ProbeRFOnCamera = m_VegaSource->GetOutput(probeIndex);
	if (pos_ProbeRFOnCamera)
	{
		// 	构建T_Camera2ProbeRF
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

		// 打印信息
		m_Controls.textBrowser_Action->append("Added pos_ProbeRFOnCamera: "
			+ QString::number(position[0]) +
			"/ " + QString::number(position[1]) +
			"/ " + QString::number(position[2])
		);

		// 计算探针Tip在相机下的位置
		Eigen::Vector4d pos_TipOnCamera4d = T_Camera2ProbeRF * pos_TipOnProbeRF4d;

		m_Controls.textBrowser_Action->append("Added pos_TipOnCamera: "
			+ QString::number(pos_TipOnCamera4d[0]) +
			"/ " + QString::number(pos_TipOnCamera4d[1]) +
			"/ " + QString::number(pos_TipOnCamera4d[2])
		);
	}

	mitk::NavigationData::Pointer pos_TibiaRFOnCamera = m_VegaSource->GetOutput(probeIndex);
	if (pos_TibiaRFOnCamera)
	{
		// 	构建T_Camera2Tibia
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
	
	// 计算探针尖端在FemurRF下的坐标值
	// Pos = T_Camera2TibiaRF.inverse() * T_Camera2ProbeRF * pos_TipOnProbeRF
	Eigen::Vector4d pos_TipOnTibiaRF4d = T_Camera2TibiaRF.inverse() * T_Camera2ProbeRF * pos_TipOnProbeRF4d;
	// 完成注册
	m_TibiaCheckPointOnTibiaRF = pos_TipOnTibiaRF4d;

	m_Controls.textBrowser_Action->append("Added m_TibiaCheckPointOnTibiaRF: ("
		     + QString::number(pos_TipOnTibiaRF4d[0]) +
		", " + QString::number(pos_TipOnTibiaRF4d[1]) +
		", " + QString::number(pos_TipOnTibiaRF4d[2]) + ")"
	);
	return true;
}

bool HTONDI::OnCalibrateSawClicked()
{
	/* 器械标定-摆锯标定
	1. 记录探针针尖位置，探针针尖的位置也就代表了摆锯标定点的位置，
	2. 记录摆锯标定点位置[由外向内] 3个点
	3. 计算m_SawPointsOnSawRF = T_CameraToTool.inverse() * m_SawPointsOnCamera

	====== 修改0821 ======
	这里只计算并记录得到摆锯标定点的位置m_SawPointsOnSawRF = Metrix[4,3]

	====== 修改0823 ======
	  修改计算误差的模式


	====== 修改0826 ======
	*/

	m_Controls.textBrowser_Action->append("Action: Calibrate Saw Node.");

	cout << "test 01" << endl;

	// 1. 记录探针针尖位置，探针针尖的位置也就代表了摆锯标定点的位置
	// 获取探针和摆锯信息
	auto probeIndex = m_VegaToolStorage->GetToolIndexByName("ProbeRF");
	auto sawRfIndex = m_VegaToolStorage->GetToolIndexByName("SawRF");
	if (probeIndex == -1 || sawRfIndex == -1)
	{
		m_Controls.textBrowser_Action->append("There is no 'Probe' or 'SawRf' in the toolStorage!");
		return false;
	}

	// 依据存储数据的数量，获取需要的探针尖端点的位置信息
	if (m_SawPointsOnSawRF.size() < 3)
	{
		m_Controls.textBrowser_Action->append("Get " + QString::number(m_SawPointsOnSawRF.size()) + "-th Saw Point");

		// 获取 ProbeRF 在 NDI 中的位置和方向数据
		mitk::NavigationData::Pointer nd_ndiToProbe = m_VegaSource->GetOutput(probeIndex);

		// 获取 SawRF 在 NDI 中的位置和方向数据
		mitk::NavigationData::Pointer nd_ndiToObjectRf = m_VegaSource->GetOutput(sawRfIndex);

		// 将 ProbeTipOnCameraRF 转化为 ProbeTipOnSawRF
		// 也就是将标定点的 SawPointsOnCameraRF 转化为 SawPointsOnSawRF
		mitk::NavigationData::Pointer nd_rfToProbe = GetNavigationDataInRef(nd_ndiToProbe, nd_ndiToObjectRf);

		// 提取转化后的标定点信息
		mitk::Point3D probeTipPointUnderRf = nd_rfToProbe->GetPosition();

		// 存储到全局位置 m_SawPointsOnSawRF [X, Y, Z, 1.0]
		// 共计 3 个
		Eigen::Vector4d CurrentSawPointsOnSawRF =
		{
			probeTipPointUnderRf[0],
			probeTipPointUnderRf[1],
			probeTipPointUnderRf[2],
			1.0
		};

		// 将计算得到的探针尖端位置添加到 点集 中
		m_SawPointsOnSawRF.push_back(CurrentSawPointsOnSawRF);


		// 打印信息
		m_Controls.textBrowser_Action->append("Added CurrentSawPointsOnSawRF: "
			+ QString::number(CurrentSawPointsOnSawRF[0]) +
			"/ " + QString::number(CurrentSawPointsOnSawRF[1]) +
			"/ " + QString::number(CurrentSawPointsOnSawRF[2])
		);
	}
	else if (m_SawPointsOnSawRF.size() == 3)
	{
		m_Controls.textBrowser_Action->append("Get Enough Saw Points.");

		// 计算标定距离: 1-2 1-3 2-3
		double caculateDistance[3] = {
			(m_SawPointsOnSawRF[0] - m_SawPointsOnSawRF[1]).norm(),
			(m_SawPointsOnSawRF[0] - m_SawPointsOnSawRF[2]).norm(),
			(m_SawPointsOnSawRF[1] - m_SawPointsOnSawRF[2]).norm(),
		};

		// 计算误差
		double SawError = sqrt(
			  pow((caculateDistance[0] - m_SawDistance[0]), 2)
			+ pow((caculateDistance[1] - m_SawDistance[1]), 2)
			+ pow((caculateDistance[2] - m_SawDistance[2]), 2)) / 3;

		m_Controls.textBrowser_CalibrateRes->append("Current Saw Calibrate ERROR: " + QString::number(SawError));
	}

	// 其它情况
	return false;
}

bool HTONDI::OnCalibrateDrillClicked()
{
	/* 器械标定-磨钻标定
	1. 记录探针针尖位置，探针针尖的位置也就代表了磨钻标定点的位置，
	2. 记录摆锯标定点位置[由外向内] 3 个点
	3. 计算m_DrillPointsOnDrillRF = T_CameraToTool.inverse() * m_DrillPointsOnCamera

	====== 修改0821 ======
	这里只计算并记录得到摆锯标定点的位置m_DrillPointsOnDrillRF = Metrix[4,3]

	====== 注释0823 ======
	需要添加磨钻的误差计算方法，待确认
	*/

	m_Controls.textBrowser_Action->append("Action: Calibrate Drill Node.");


	// 1. 记录探针针尖位置，探针针尖的位置也就代表了摆锯标定点的位置
	// 获取探针和摆锯信息
	auto probeIndex = m_VegaToolStorage->GetToolIndexByName("ProbeRF");
	auto DrillRfIndex = m_VegaToolStorage->GetToolIndexByName("DrillRF");
	if (probeIndex == -1 || DrillRfIndex == -1)
	{
		m_Controls.textBrowser_Action->append("There is no 'Probe' or 'DrillRF' in the toolStorage!");
		return false;
	}

	// 依据存储数据的数量，获取需要的探针尖端点的位置信息
	if (m_DrillPointsOnDrillRF.size() < 3)
	{
		m_Controls.textBrowser_Action->append("Get " + QString::number(m_DrillPointsOnDrillRF.size()) + "-th Drill Point");

		// 获取 ProbeRF 在 NDI 中的位置和方向数据
		mitk::NavigationData::Pointer nd_ndiToProbe = m_VegaSource->GetOutput(probeIndex);

		// 获取 SawRF 在 NDI 中的位置和方向数据
		mitk::NavigationData::Pointer nd_ndiToObjectRf = m_VegaSource->GetOutput(DrillRfIndex);

		// 将 ProbeTipOnCameraRF 转化为 ProbeTipOnSawRF
		// 也就是将标定点的 SawPointsOnCameraRF 转化为 SawPointsOnSawRF
		mitk::NavigationData::Pointer nd_rfToProbe = GetNavigationDataInRef(nd_ndiToProbe, nd_ndiToObjectRf);

		// 提取转化后的标定点信息
		mitk::Point3D probeTipPointUnderRf = nd_rfToProbe->GetPosition();

		// 存储到全局位置 m_SawPointsOnSawRF [X, Y, Z, 1.0]
		// 共计 3 个
		Eigen::Vector4d CurrentDrillPointsOnSawRF =
		{
			probeTipPointUnderRf[0],
			probeTipPointUnderRf[1],
			probeTipPointUnderRf[2],
			1.0
		};

		// 将计算得到的探针尖端位置添加到 点集 中
		m_DrillPointsOnDrillRF.push_back(CurrentDrillPointsOnSawRF);

		// 打印信息
		m_Controls.textBrowser_Action->append("Add CurrentDrillPointsOnSawRF: "
			+ QString::number(CurrentDrillPointsOnSawRF[0]) +
			"/ " + QString::number(CurrentDrillPointsOnSawRF[1]) +
			"/ " + QString::number(CurrentDrillPointsOnSawRF[2])
		);
	}
	else if (m_DrillPointsOnDrillRF.size() == 3)
	{
		m_Controls.textBrowser_Action->append("Get Enough Drill Points.");
		// ============== 需要添加误差计算方法 ================
		double DrillError = 0.0;
		// ==================================-================
		m_Controls.textBrowser_CalibrateRes->append("Current Saw Calibrate ERROR: " + QString::number(DrillError));
	}

	// 其它情况
	return false;
}
bool HTONDI::OnSawVisualizeClicked()
{
	/* 摆锯可视化 */
	m_Controls.textBrowser_Action->append("Action: Visualize Saw.");
	auto saw = GetDataStorage()->GetNamedNode("Saw");
	auto sawPoints = GetDataStorage()->GetNamedNode("SawLandMarkPointSet");
	if (saw && sawPoints)
	{
		// 获取状态
		bool currentVisibility = saw->IsVisible(nullptr, "visible", true);
		// 展示摆锯和点集合
		saw->SetVisibility(!currentVisibility);
		sawPoints->SetVisibility(!currentVisibility);
	}
	else
	{
		m_Controls.textBrowser_Action->append("Saw or SawLandMarkPointSet Not Found!");
	}
	GetDataStorage()->Modified();
	mitk::RenderingManager::GetInstance()->RequestUpdateAll();
	return true;
}


bool HTONDI::OnDrillVisualizeClicked()
{
	/* 磨钻可视化 */
	m_Controls.textBrowser_Action->append("Action: Visualize Drill.");
	auto drill = GetDataStorage()->GetNamedNode("Drill");
	auto drillPoints = GetDataStorage()->GetNamedNode("DrillLandMarkPointSet");
	if (drill && drillPoints)
	{
		// 获取状态
		bool currentVisibility = drill->IsVisible(nullptr, "visible", true);
		// 展示钢板和点集合
		drill->SetVisibility(!currentVisibility);
		drillPoints->SetVisibility(!currentVisibility);
	}
	else
	{
		m_Controls.textBrowser_Action->append("Drill or DrillLandMarkPointSet Not Found!");
	}
	GetDataStorage()->Modified();
	mitk::RenderingManager::GetInstance()->RequestUpdateAll();
	return true;
}

bool HTONDI::OnresetSawCaliClicked()
{
	/* 重置摆锯标定 */
	m_Controls.textBrowser_Action->append("Action: Reset Cali Saw.");
	m_SawPointsOnSawRF.clear();

	return true;
}

bool HTONDI::OnresetDrillCaliClicked()
{
	/* 重置摆锯标定 */
	m_Controls.textBrowser_Action->append("Action: Reset Cali Drill.");
	m_DrillPointsOnDrillRF.clear();

	return true;
}

// 粗配准流程
// 股骨粗配准
bool HTONDI::OnGetFemurLandmarkClicked()
{
	/* 股骨的三点粗配准点采集 = 股骨外踝点 股骨内踝点 + 股骨头中心
	1. 粗配准点采集，这里采集前两个点，股骨头中心采用拟合的方法获得
	
	需要探针及股骨的RF: ProbeRF FemurRF

	====== 修改0823 ======
	股骨实际上不需要配准，直接去计算几个标记点在股骨坐标系下的位置即可
	这样的话，这里的代码应该和股骨标记点注册是一样的
	*/

	m_Controls.textBrowser_Action->append("Action: Get Femur Landmark Nodes.");

	// 取出已经得到的Target配准点集
	auto pointSet_probeLandmark = femur_image->GetLandmarks_probe();
	auto pointSet_landmark = femur_image->GetLandmarks();

	
	// 股骨粗配准点的采集只采2个点
	auto tmpNum = femur_image->GetLandmarks_probe()->GetSize();
	if (tmpNum >= 2)
	{
		m_Controls.textBrowser_Action->append("'Femur' landmark point done, please caculate hipcenter!");
		return false;
	}

	// 开始选择标定点
	// 股骨外踝点 -> 股骨内踝点
	// Pos = T_Camera2TibiaRF.inverse() * T_Camera2ProbeRF * pos_TipOnProbeRF
	auto probeIndex = m_VegaToolStorage->GetToolIndexByName("ProbeRF");
	auto tibiaRFIndex = m_VegaToolStorage->GetToolIndexByName("FemurRF");
	if (probeIndex == -1 || tibiaRFIndex == -1)
	{
		m_Controls.textBrowser_Action->append("There is no 'ProbeRF' or 'FemurRF' in the toolStorage!");
		return false;
	}

	// 获取探针针尖位置
	auto toolProbe = m_VegaToolStorage->GetToolByName("ProbeRF");
	mitk::Point3D pos_TipOnProbeRF = toolProbe->GetToolTipPosition();

	// 转化为4d
	Eigen::Vector4d pos_TipOnProbeRF4d(pos_TipOnProbeRF[0], pos_TipOnProbeRF[1], pos_TipOnProbeRF[2], 1.0);

	// 取出探针针尖点在相机下的位置
	// 4x4齐次变换矩阵
	// T_FemurRF2ProbeRF = T_Camera2FemurRF.inverse() * T_Camera2ProbeRF
	Eigen::Matrix4d T_Camera2FemurRF = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d T_Camera2ProbeRF = Eigen::Matrix4d::Identity();

	mitk::NavigationData::Pointer pos_ProbeRFOnCamera = m_VegaSource->GetOutput(probeIndex);
	if (pos_ProbeRFOnCamera)
	{
		// 	构建T_FemurRF2Camera
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

	mitk::NavigationData::Pointer pos_FemurRFOnCamera = m_VegaSource->GetOutput(probeIndex);
	if (pos_FemurRFOnCamera)
	{
		// 	构建T_FemurRF2Camera
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

	// 计算探针尖端在FemurRF下的坐标值
	// Pos = T_Camera2FemurRF.inverse() * T_Camera2ProbeRF * pos_TipOnProbeRF
	Eigen::Vector4d pos_TipOnFemurRF4d = T_Camera2FemurRF.inverse() * T_Camera2ProbeRF * pos_TipOnProbeRF4d;
	// 添加标记点数据
	mitk::Point3D CurrentFemurLandmarkPoint = (pos_TipOnFemurRF4d[0], pos_TipOnFemurRF4d[1], pos_TipOnFemurRF4d[2]);
	pointSet_landmark->InsertPoint(CurrentFemurLandmarkPoint);

	m_Controls.textBrowser_Action->append("Added CurrentFemurLandmarkPointOnFemurRF: ("
		+ QString::number(CurrentFemurLandmarkPoint[0]) +
		", " + QString::number(CurrentFemurLandmarkPoint[1]) +
		", " + QString::number(CurrentFemurLandmarkPoint[2]) + ")"
	);
	return true;
}

// ============================股骨粗配准点采集=====================================
// 股骨外踝点=>股骨内踝点=>股骨头中心(已经计算得到) 

// 晃动大腿获得股骨头中心
// 采集晃动过程中的 100 个点来拟合球面
bool HTONDI::OnCollectBonePosClicked()
{
	/* 欢动大腿获得股骨头中心
	1. 采集大腿晃动过程的RF姿态
	2. 球面拟合，获得球心/股骨头中心
	*/

	m_Controls.textBrowser_Action->append("Action: Collect Femora Pos.");

	cout << "test 01" << endl;
	// 开始采集接下来一段时间内的股骨位置信息
	m_FemurPositions.clear();
	m_Controls.textBrowser_Action->append("Start collect femora data, please move slowly...");

	cout << "test 02" << endl;
	// 启动定时器，定期采集数据
	if (m_FemurDataCollectionTimer == nullptr)
	{
		cout << "test 03" << endl;
		m_FemurDataCollectionTimer = new QTimer(this);
		connect(m_FemurDataCollectionTimer, &QTimer::timeout, this, &HTONDI::CollectFemurData);
	}
	m_FemurDataCollectionTimer->start(100); // 每100ms采集一次数据

	return true;
}

bool HTONDI::CollectFemurData()
{
	/* 获取当前股骨追踪器的位置
	1. 采集100个股骨姿态

	股骨参考阵列命名必须为：FemurRF
	*/
	cout << "test 04-01" << endl;
	auto femurIndex = m_VegaToolStorage->GetToolIndexByName("FemurRF");
	if (femurIndex == -1)
	{
		m_Controls.textBrowser_Action->append("ERROR: FemurRF tool not found.");
		return false;
	}
	cout << "test 04-02" << endl;
	mitk::NavigationData::Pointer nd_ndiToFemur = m_VegaSource->GetOutput(femurIndex);
	if (nd_ndiToFemur)
	{
		Eigen::Vector3d position(nd_ndiToFemur->GetPosition()[0], nd_ndiToFemur->GetPosition()[1], nd_ndiToFemur->GetPosition()[2]);
		m_FemurPositions.push_back(position);
		m_Controls.textBrowser_BoneCenter->append("Get femora pos: " + QString::number(m_FemurPositions.size()) + "/100");
	}
	if (m_FemurPositions.size() >= 100)
	{
		m_Controls.textBrowser_BoneCenter->append("Collect done, get 100 pos data.");
		m_FemurDataCollectionTimer->stop();
	}

	return true;
}


bool HTONDI::OnCaculateBoneCenterClicked()
{
	/*
	拟合计算出晃动的股骨头中心
	*/

	m_Controls.textBrowser_Action->append("Action: Caculate FemoralHead Center.");

	if (m_FemurPositions.size() >= 100) // 确保有足够的数据点
	{
		CalculateHipCenter();
	}
	else
	{
		m_Controls.textBrowser_Action->append("Femora data < " + number);
	}

	return true;
}

// 拟合函数
bool HTONDI::CalculateHipCenter()
{
	/* 球面拟合
	1. 100个姿态，每10个一组拟合球心
	2. 输出球心均值为股骨头中心
	*/
	m_Controls.textBrowser_Action->append("Action: Collect Femora Pos.");

	// 将点分成10个一组
	int groupSize = 10;
	int numGroups = m_FemurPositions.size() / groupSize;
	std::vector<Eigen::Vector3d> centers;

	for (int i = 0; i < numGroups; ++i) {
		std::vector<Eigen::Vector3d> subsetPoints(m_FemurPositions.begin() + i * groupSize, m_FemurPositions.begin() + (i + 1) * groupSize);
		auto [center, _] = FitSphere(subsetPoints);
		centers.push_back(center);
	}

	// 计算球心的均值
	Eigen::Vector3d averageCenter(0, 0, 0);
	for (const auto& center : centers) {
		averageCenter += center;
	}
	averageCenter /= centers.size();
	m_HipCenter = averageCenter;

	m_Controls.textBrowser_FemurRes->append("Femora center get!");
	m_Controls.textBrowser_FemurRes->append("Pos: (" +
		QString::number(m_HipCenter(0)) + ", " +
		QString::number(m_HipCenter(1)) + ", " +
		QString::number(m_HipCenter(2)) + ")");

	// 保存股骨头中心的位置

	return true;
}



bool HTONDI::OnCaculateFemurLandmarkClicked()
{
	/* 股骨的三点粗配准计算及应用
	1. 集成股骨粗配准点
	2. 配准计算
	3. 保存股骨的配准矩阵

	股骨RF必须为FemurRf
	
	====== 修改0822 ======
	不需要再绑定物体表面

	====== 修改0823 ======
	仅完成配准计算，不再进行应用

	实际上，胫骨的粗配准和精配准得到的配准矩阵就可以直接将股骨表面点进行投影了
	但是春立这边缺要求进行配准，那只好保存股骨的配准矩阵
	*/

	// 将计算得到的 股骨头中心 也添加到 landmark 列表 
	m_Controls.textBrowser_Action->append("Action: Caculate Femur Landmark.");

	// 将股骨头中心点添加进来
	// 取出已经得到的Target配准点集
	auto pointSet_probeLandmark = femur_image->GetLandmarks_probe();

	// 将计算得到的探针尖端位置添加到 点集 中
	mitk::Point3D tmpHipCenter;
	tmpHipCenter[0] = m_HipCenter[0];
	tmpHipCenter[1] = m_HipCenter[1];
	tmpHipCenter[2] = m_HipCenter[2];
	pointSet_probeLandmark->SetPoint(2, tmpHipCenter);

	// 取得粗配准点集 和 当前已经取得的点
	int femurRF_SurfaceSrcNum = femur_image->GetLandmarks()->GetSize();
	int femurRF_SurfaceTargetNum = femur_image->GetLandmarks_probe()->GetSize();

	// 判别采集进度
	if (femurRF_SurfaceSrcNum == femurRF_SurfaceTargetNum)
	{
		m_Controls.textBrowser_Action->append("--- Enough Femur landmarks have been collected ----");
	}
	else
	{
		m_Controls.textBrowser_Action->append("--- Not Enough Femur landmarks have been collected ----");
		cout << "femurRF_SurfaceSrcNum: " << femurRF_SurfaceSrcNum << "femurRF_SurfaceTargetNum: " << femurRF_SurfaceTargetNum << endl;
		return false;
	}

	// 更新股骨从实际空间到图像空间的变换矩阵 m_
	femur_image->UpdateObjectToRfMatrix();

	// 打印当前配准计算的结果
	// lanmark
	m_Controls.textBrowser_FemurRes->append("Avg landmark error:" + QString::number(femur_image->GetlandmarkRegis_avgError()));
	m_Controls.textBrowser_FemurRes->append("Max landmark error:" + QString::number(femur_image->GetlandmarkRegis_maxError()));

	// 计算转化矩阵 m_MetrixFemurRFToImage
	// 记录转化矩阵
	m_MetrixFemurRFToImage = mitk::AffineTransform3D::New();

	// 将导航图像的对象到参考框架的变换矩阵从VTK格式转换为ITK格式，并存储
	mitk::TransferVtkMatrixToItkTransform(femur_image->GetT_Object2ReferenceFrame(), m_MetrixFemurRFToImage.GetPointer());

	// 将配准矩阵应用到 FemurRF 工具中
	m_VegaToolStorage->GetToolByName("FemurRF")->SetToolRegistrationMatrix(m_MetrixFemurRFToImage);

	// 提取矩阵
	// 获取仿射变换矩阵
	mitk::AffineTransform3D::MatrixType matrix = m_MetrixFemurRFToImage->GetMatrix();


	// 将 mitk::AffineTransform3D 的矩阵转换为 Eigen::Matrix4d
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			m_Metrix4dFemurRFToImage(i, j) = matrix(i, j);
		}
	}

	return true;
}


bool HTONDI::OnGetTibiaLandmarkClicked()
{
	/* 胫骨表面粗配准点采集：胫骨近端外侧点 胫骨近端内侧点 胫骨远端外踝点 胫骨远端内踝点
	1. 按照顺序进行粗配准点的采集
	2. 判别添加点的数量

	需要与术前规划特征点采集的方法一致
	====== 修正0826 ======
	修正点集识别
	*/
	m_Controls.textBrowser_Action->append("Action: Get Tibia Landmark.");

	// 获取股骨的表面数据
	auto TibiaRF_Surface = GetDataStorage()->GetNamedNode("tibiaSurface");

	if (TibiaRF_Surface == nullptr)
	{
		m_Controls.textBrowser_Action->append("ERROR: Tibia model Not Found!");
		return false;
	}

	// 取出已经得到的Target配准点集
	auto pointSet_landmark = tibia_image->GetLandmarks();
	auto pointSet_probeLandmark = tibia_image->GetLandmarks_probe();


	if (pointSet_probeLandmark->GetSize() >= pointSet_landmark->GetSize())
	{
		m_Controls.textBrowser_Action->append("Tibia landmark points is enough!");
		return false;
	}


	// 开始进行配准点采集
	// Pos_Tip = T_Camera2ProbeRF * pos_TipOnProbeRF
	auto probeIndex = m_VegaToolStorage->GetToolIndexByName("ProbeRF");
	auto tibiaRFIndex = m_VegaToolStorage->GetToolIndexByName("TibiaRF");
	if (probeIndex == -1 || tibiaRFIndex == -1)
	{
		m_Controls.textBrowser_Action->append("There is no 'ProbeRF' or 'TibiaRF' in the toolStorage!");
		return false;
	}

	// 获取探针针尖位置
	auto toolProbe = m_VegaToolStorage->GetToolByName("ProbeRF");
	mitk::Point3D pos_TipOnProbeRF = toolProbe->GetToolTipPosition();
	cout << "pos_TipOnProbeRF: (" << pos_TipOnProbeRF[0] << " , " << pos_TipOnProbeRF[1] << " , " << pos_TipOnProbeRF[2] << ")" << endl;

	// 转化为4d
	Eigen::Vector4d pos_TipOnProbeRF4d(pos_TipOnProbeRF[0], pos_TipOnProbeRF[1], pos_TipOnProbeRF[2], 1.0);

	// 取出探针针尖点在相机下的位置
	// 4x4齐次变换矩阵
	Eigen::Matrix4d T_Camera2ProbeRF = Eigen::Matrix4d::Identity();

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

	// 计算探针尖端在FemurRF下的坐标值
	// Pos_Tip = T_Camera2ProbeRF * pos_TipOnProbeRF
	Eigen::Vector4d pos_TipOnCamera = T_Camera2ProbeRF * pos_TipOnProbeRF4d;

	cout << "pos_TipOnCamera: (" << pos_TipOnCamera[0] << " , " << pos_TipOnCamera[1] << " , " << pos_TipOnCamera[2] << ")" << endl;

	// 添加标记点数据
	mitk::Point3D CurrentTibiaLandmarkPoint;
	CurrentTibiaLandmarkPoint[0] = pos_TipOnCamera[0];
	CurrentTibiaLandmarkPoint[1] = pos_TipOnCamera[1];
	CurrentTibiaLandmarkPoint[2] = pos_TipOnCamera[2];

	pointSet_probeLandmark->InsertPoint(CurrentTibiaLandmarkPoint);

	m_Controls.textBrowser_Action->append("Added CurrentTibiaLandmarkPoint: ("
		     + QString::number(CurrentTibiaLandmarkPoint[0]) +
		", " + QString::number(CurrentTibiaLandmarkPoint[1]) +
		", " + QString::number(CurrentTibiaLandmarkPoint[2]) + ")"
	);
	return true;
}

bool HTONDI::OnCaculateTibiaLandmarlClicked()
{	
	/* 胫骨的四点粗配准计算及应用
	1. 粗配准计算
	2. 粗配准应用

	====已验证0813====
	*/
	m_Controls.textBrowser_Action->append("Action: Caculate Tibia Landmark.");

	// 取出已经得到的Target配准点集
	auto pointSet_probeLandmark = tibia_image->GetLandmarks_probe();

	// 取得粗配准点集 和 当前已经取得的点
	int tibiaRF_SurfaceSrcNum = tibia_image->GetLandmarks()->GetSize();
	int tibiaRF_SurfaceTargetNum = tibia_image->GetLandmarks_probe()->GetSize();

	// 判别采集进度
	if (tibiaRF_SurfaceSrcNum == tibiaRF_SurfaceTargetNum)
	{
		m_Controls.textBrowser_Action->append("--- Enough tibia landmarks have been collected ----");
	}
	else
	{
		m_Controls.textBrowser_Action->append("--- Not Enough tibia landmarks have been collected ----");
		return true;
	}

	// 开始配准计算
	// 创建一个新的静态图像表面配准过滤器实例
	m_surfaceRegistrationStaticImageFilter = lancet::ApplySurfaceRegistratioinStaticImageFilter::New();

	// 将过滤器连接到Vega跟踪数据源
	m_surfaceRegistrationStaticImageFilter->ConnectTo(m_VegaSource);

	// 创建一个新的仿射变换矩阵，用于存储配准结果
	m_MetrixTibiaRFToImage = mitk::AffineTransform3D::New();

	// 更新导航图像的对象到参考框架的变换矩阵
	tibia_image->UpdateObjectToRfMatrix();

	// 打印当前配准计算的结果
	// lanmark
	m_Controls.textBrowser_TibieRes->append("Avg landmark error:" + QString::number(tibia_image->GetlandmarkRegis_avgError()));
	m_Controls.textBrowser_TibieRes->append("Max landmark error:" + QString::number(tibia_image->GetlandmarkRegis_maxError()));

	// 将导航图像的对象到参考框架的变换矩阵从VTK格式转换为ITK格式，并存储
	mitk::TransferVtkMatrixToItkTransform(tibia_image->GetT_Object2ReferenceFrame(), m_MetrixTibiaRFToImage.GetPointer());

	// 影像配准矩阵随着骨骼参考阵列在不断变化 
	// T_C2M = T_C2TibiaRF * T_TibiaRF2M

	// 将配准矩阵应用到 BoneRF 工具中
	m_VegaToolStorage->GetToolByName("TibiaRF")->SetToolRegistrationMatrix(m_MetrixTibiaRFToImage);

	// 获取仿射变换矩阵
	mitk::AffineTransform3D::MatrixType matrix = m_MetrixTibiaRFToImage->GetMatrix();

	// 将 mitk::AffineTransform3D 的矩阵转换为 Eigen::Matrix4d
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			m_Metrix4dTibiaRFToImage(i, j) = matrix(i, j);
		}
	}

	// 将配准矩阵设置到静态图像表面配准过滤器中
	m_surfaceRegistrationStaticImageFilter->SetRegistrationMatrix(m_VegaToolStorage->GetToolByName("TibiaRF")->GetToolRegistrationMatrix());

	// 设置参考框架的导航数据
	m_surfaceRegistrationStaticImageFilter->SetNavigationDataOfRF(m_VegaSource->GetOutput("TibiaRF"));

	// 停止Vega可视化定时器
	m_VegaVisualizeTimer->stop();

	// 将Vega可视化器连接到静态图像表面配准过滤器
	m_VegaVisualizer->ConnectTo(m_surfaceRegistrationStaticImageFilter);

	// 重新启动Vega可视化定时器，显示配准计算的结果
	m_VegaVisualizeTimer->start();

	return true;
}

bool HTONDI::OnGetTibieICPClicked()
{
	/* 胫骨表面精配准点采集
	1. 表面采点

	=========已验证0814===========
	*/
	m_Controls.textBrowser_Action->append("Action: Get Tibie ICP.");

	// 首先检查胫骨是否已经初始化
	if (tibia_image == nullptr)
	{
		m_Controls.textBrowser_Action->append("Please setup the tibia_image first!");
		return false;
	}

	// 获取ICP(迭代最近点)点集
	auto pointSet_probeIcp = tibia_image->GetIcpPoints_probe();

	// 获取探针和参考物体(骨骼)的索引
	auto probeIndex = m_VegaToolStorage->GetToolIndexByName("ProbeRF");
	auto objectRfIndex = m_VegaToolStorage->GetToolIndexByName("TibiaRF");

	// 检查工具是否存在
	if (probeIndex == -1 || objectRfIndex == -1)
	{
		m_Controls.textBrowser_Action->append("There is no 'Probe' or 'TibiaRF' in the toolStorage!");
	}

	// 获取探针和参考物体在NDI坐标系中的导航数据
	mitk::NavigationData::Pointer nd_ndiToProbe = m_VegaSource->GetOutput(probeIndex);
	mitk::NavigationData::Pointer nd_ndiToObjectRf = m_VegaSource->GetOutput(objectRfIndex);

	// 将探针的导航数据转换到参考物体的坐标系中
	mitk::NavigationData::Pointer nd_rfToProbe = GetNavigationDataInRef(nd_ndiToProbe, nd_ndiToObjectRf);

	// 获取探针尖端在参考物体坐标系中的位置
	mitk::Point3D probeTipPointUnderRf = nd_rfToProbe->GetPosition();

	// 将这个点添加到ICP点集中
	pointSet_probeIcp->InsertPoint(probeTipPointUnderRf);

	// 在用户界面上显示添加的点的坐标
	m_Controls.textBrowser_TibieRes->append("Added icp point: " + QString::number(probeTipPointUnderRf[0]) +
		"/ " + QString::number(probeTipPointUnderRf[1]) + "/ " + QString::number(probeTipPointUnderRf[2]));

	return true;
}
bool HTONDI::OnCaculateTibiaICPClicked()
{
	/* 胫骨的精配准计算及配准融合应用
	1. 精配准计算
	2. 精配准与粗配准的融合应用
	3. 绑定物体表面

	====已验证0814====
	*/
	m_Controls.textBrowser_Action->append("Action: Caculate Tibia ICP.");

	// 创建一个新的静态图像表面配准过滤器实例
	m_surfaceRegistrationStaticImageFilter = lancet::ApplySurfaceRegistratioinStaticImageFilter::New();
	// 将过滤器连接到Vega跟踪数据源
	m_surfaceRegistrationStaticImageFilter->ConnectTo(m_VegaSource);
	// 创建一个新的仿射变换矩阵，用于存储配准结果
	m_imageRegistrationMatrix = mitk::AffineTransform3D::New();
	// 更新导航图像的对象到参考框架的变换矩阵
	tibia_image->UpdateObjectToRfMatrix();

	// 打印当前配准计算的结果
	// lanmark
	m_Controls.textBrowser_TibieRes->append("Avg landmark error:" + QString::number(tibia_image->GetlandmarkRegis_avgError()));
	m_Controls.textBrowser_TibieRes->append("Max landmark error:" + QString::number(tibia_image->GetlandmarkRegis_maxError()));
	// icp
	m_Controls.textBrowser_TibieRes->append("Avg ICP error:" + QString::number(tibia_image->GetIcpRegis_avgError()));
	m_Controls.textBrowser_TibieRes->append("Max ICP error:" + QString::number(tibia_image->GetIcpRegis_maxError()));

	// 将导航图像的对象到参考框架的变换矩阵从VTK格式转换为ITK格式，并存储
	mitk::TransferVtkMatrixToItkTransform(tibia_image->GetT_Object2ReferenceFrame(), m_imageRegistrationMatrix.GetPointer());
	// 将配准矩阵应用到 BoneRF 工具中
	m_VegaToolStorage->GetToolByName("TibiaRF")->SetToolRegistrationMatrix(m_imageRegistrationMatrix);
	// 将配准矩阵设置到静态图像表面配准过滤器中
	m_surfaceRegistrationStaticImageFilter->SetRegistrationMatrix(m_VegaToolStorage->GetToolByName("TibiaRF")->GetToolRegistrationMatrix());
	// 设置参考框架的导航数据
	m_surfaceRegistrationStaticImageFilter->SetNavigationDataOfRF(m_VegaSource->GetOutput("TibiaRF"));
	// 停止Vega可视化定时器
	m_VegaVisualizeTimer->stop();
	// 将Vega可视化器连接到静态图像表面配准过滤器
	m_VegaVisualizer->ConnectTo(m_surfaceRegistrationStaticImageFilter);
	// 重新启动Vega可视化定时器，显示配准计算的结果
	m_VegaVisualizeTimer->start();

	// 保存配准矩阵到 m_PreviousImageRegistrationMatrix
	// m_PreviousImageRegistrationMatrix = m_imageRegistrationMatrix;

	// 绑定物体表面
	// 获取当前胫骨RF的位置
	auto TibiaRF = m_VegaToolStorage->GetToolByName("TibiaRF");

	// 获取胫骨的表面数据
	auto TibiaRF_Surface = GetDataStorage()->GetNamedNode("tibiaSurface");

	if (TibiaRF.IsNull())
	{
		m_Controls.textBrowser_Action->append("ERROR: TibiaRF not found, try reload .IGTToolStorage");
		return false;
	}

	if (TibiaRF_Surface == nullptr)
	{
		m_Controls.textBrowser_Action->append("ERROR: Tibia model Not Found!");
		return false;
	}

	// 绑定物体表面
	TibiaRF->SetDataNode(TibiaRF_Surface);
	TibiaRF_Surface->SetVisibility(true);

	return true;
}