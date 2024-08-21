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
		saw_image->SetReferencFrameName(SawPoints->GetName());
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
		drill_image->SetReferencFrameName(DrillPoints->GetName());
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
	/* 标记点验证-验证股骨标记点
	1. 探针采点
	2. 计算误差：将采集到的点与实际规划的点进行误差计算比较
	*/

	// 验证股骨点
	// 检测股骨RF，然后使用探针采集股骨上标记点位置，与事前注册的位置进行误差计算
	m_Controls.textBrowser_Action->append("Action: Check Femur Node.");

	// 获取探针针尖标记位置
	auto probeRF = m_VegaToolStorage->GetToolByName("ProbeRF");
	if (probeRF.IsNull())
	{
		m_Controls.textBrowser_Action->append("ERROR: ProbeRF tool not found.");
		return false;
	}

	// 获取针尖位置
	mitk::Point3D TipProbe = probeRF->GetToolTipPosition();


	// 获取当前股骨RF的位置
	auto femurRF = m_VegaToolStorage->GetToolByName("FemurRF");
	if (femurRF.IsNull())
	{
		m_Controls.textBrowser_Action->append("ERROR: FemurRF not found, try reload .IGTToolStorage");
		return false;
	}

	// 获取工具
	m_femurRFNode = femurRF;

	// 获取验证点 VerifyPoint
	mitk::Point3D valid_point;

	// 好像这个VerifyPoint更像是机器人验证点？
	valid_point = femurRF->GetVerifyPoint();

	// 计算误差
	double vx = TipProbe[0] - valid_point[0];
	double vy = TipProbe[1] - valid_point[1];
	double vz = TipProbe[2] - valid_point[2];
	double FemurError = sqrt(pow(vx, 2) + pow(vy, 2) + pow(vz, 2));

	// 输出结果
	m_Controls.textBrowser_CheckProbe->append("Probe to FemurPoint ERROR: " + QString::number(FemurError));

	return true;
}


bool HTONDI::OnTibiaCheckClicked()
{
	/* 标记点验证-验证胫骨标记点
	1. 探针采点
	2. 计算误差：将采集到的点与实际规划的点进行误差计算比较
	*/

	// 检测胫骨RF，然后使用探针采集胫骨上标记点位置，与事前注册的位置进行误差计算
	m_Controls.textBrowser_Action->append("Action: Check Tibia Node.");

	// 获取探针针尖标记位置
	auto probeRF = m_VegaToolStorage->GetToolByName("ProbeRF");
	if (probeRF.IsNull())
	{
		m_Controls.textBrowser_Action->append("ERROR: ProbeRF tool not found.");
		return false;
	}

	// 获取针尖位置
	mitk::Point3D TipProbe = probeRF->GetToolTipPosition();

	// 获取当前股骨RF的位置
	auto tibiaRF = m_VegaToolStorage->GetToolByName("TibiaRF");
	if (tibiaRF.IsNull())
	{
		m_Controls.textBrowser_Action->append("ERROR: TibiaRF not found, try reload .IGTToolStorage");
		return false;
	}

	// 获取工具
	m_tibiaRFNode = tibiaRF;

	// 获取验证点 VerifyPoint
	mitk::Point3D valid_point;

	// 好像这个VerifyPoint更像是机器人验证点？
	valid_point = m_tibiaRFNode->GetVerifyPoint();

	// 计算误差
	double vx = TipProbe[0] - valid_point[0];
	double vy = TipProbe[1] - valid_point[1];
	double vz = TipProbe[2] - valid_point[2];
	double FemurError = sqrt(pow(vx, 2) + pow(vy, 2) + pow(vz, 2));

	// 输出结果
	m_Controls.textBrowser_CheckProbe->append("Probe to TibiaPoint ERROR: " + QString::number(FemurError));

	return true;
}


bool HTONDI::OnCalibrateSawClicked()
{
	/* 器械标定-摆锯标定
	1. 记录探针针尖位置，注册摆锯标定点位置
	2. 配准摆锯表面位置，计算T_saw下的摆锯标定点的坐标
	3. 应用配准结果
	4. 绑定物体表面图像

	====已验证0813====
	*/

	m_Controls.textBrowser_Action->append("Action: Calibrate Saw Node.");
	
	cout << "test 01" << endl;

	// 获取当前摆锯RF的位置
	auto sawRF = m_VegaToolStorage->GetToolByName("SawRF");

	// 获取摆锯的表面数据
	auto sawRF_Surface = GetDataStorage()->GetNamedNode("Saw");

	cout << "test 02" << endl;
	if (sawRF.IsNull())
	{
		m_Controls.textBrowser_Action->append("ERROR: sawRF not found, try reload .IGTToolStorage");
		return false;
	}

	if (sawRF_Surface == nullptr)
	{
		m_Controls.textBrowser_Action->append("ERROR: Saw model Not Found!");
		return false;
	}

	// 获取工具
	m_saw = sawRF;

	cout << "test 03" << endl;
	// 开始标定点采集和标定过程
	// 取得粗配准点集 和 当前已经取得的点

	// 如果配准点的数量已经足够，则开始进行配准计算
	int sawRF_SurfaceSrcNum = saw_image->GetLandmarks()->GetSize();
	int sawRF_SurfaceTargetNum = saw_image->GetLandmarks_probe()->GetSize();

	if (sawRF_SurfaceSrcNum == sawRF_SurfaceTargetNum)
	{
		cout << "test 06" << endl;
		m_Controls.textBrowser_Action->append("--- Enough Saw landmarks have been collected ----");
		// 开始配准计算

		// 创建一个新的静态图像表面配准过滤器实例
		m_surfaceRegistrationStaticImageFilter = lancet::ApplySurfaceRegistratioinStaticImageFilter::New();

		cout << "test 07" << endl;
		// 将过滤器连接到Vega跟踪数据源
		m_surfaceRegistrationStaticImageFilter->ConnectTo(m_VegaSource);

		// 创建一个新的仿射变换矩阵，用于存储配准结果
		m_imageRegistrationMatrix = mitk::AffineTransform3D::New();

		cout << "test 08" << endl;
		// 更新导航图像的对象到参考框架的变换矩阵
		saw_image->UpdateObjectToRfMatrix();

		cout << "test 09" << endl;
		// 打印当前配准计算的结果
		// lanmark
		m_Controls.textBrowser_CalibrateRes->append("Avg landmark error:" + QString::number(saw_image->GetlandmarkRegis_avgError()));
		m_Controls.textBrowser_CalibrateRes->append("Max landmark error:" + QString::number(saw_image->GetlandmarkRegis_maxError()));

		cout << "Avg landmark error:" << saw_image->GetlandmarkRegis_avgError() << endl;
		cout << "Max landmark error:" << saw_image->GetlandmarkRegis_maxError() << endl;

		cout << "test 10" << endl;
		// 将导航图像的对象到参考框架的变换矩阵从VTK格式转换为ITK格式，并存储
		mitk::TransferVtkMatrixToItkTransform(saw_image->GetT_Object2ReferenceFrame(), m_imageRegistrationMatrix.GetPointer());

		cout << "test 11" << endl;
		// 将配准矩阵应用到 SawRF 工具中
		m_VegaToolStorage->GetToolByName("SawRF")->SetToolRegistrationMatrix(m_imageRegistrationMatrix);

		cout << "test 12" << endl;
		// 将配准矩阵设置到静态图像表面配准过滤器中
		//m_surfaceRegistrationStaticImageFilter->SetRegistrationMatrix(m_VegaToolStorage->GetToolByName("SawRF")->GetToolRegistrationMatrix());

		cout << "test 13" << endl;
		// 设置参考框架的导航数据
		/*
			如果这时候有第三个RF，需要测试将它们在第三个RF下进行相对运动
		*/
		//m_surfaceRegistrationStaticImageFilter->SetNavigationDataOfRF(m_VegaSource->GetOutput("SawRF"));

		cout << "test 14" << endl;
		// 停止Vega可视化定时器
		m_VegaVisualizeTimer->stop();

		cout << "test 15" << endl;
		// 将Vega可视化器连接到静态图像表面配准过滤器
		m_VegaVisualizer->ConnectTo(m_surfaceRegistrationStaticImageFilter);

		cout << "test 16" << endl;
		// 重新启动Vega可视化定时器，显示配准计算的结果
		m_VegaVisualizeTimer->start();

		cout << "test 17" << endl;
		// 4. 绑定物体表面
		sawRF->SetDataNode(sawRF_Surface);
		sawRF_Surface->SetVisibility(true);

		return true;
	}

	// 倘若采点不足够，则继续采点
	cout << "test 04" << endl;
	// 取出已经得到的Target配准点集
	auto pointSet_probeLandmark = saw_image->GetLandmarks_probe();
	auto pointSet_landmark = saw_image->GetLandmarks();

	// 开始选择标定点
	// 由远及近开始标定

	// 检测工具是否有效
	auto probeIndex = m_VegaToolStorage->GetToolIndexByName("ProbeRF");
	auto sawRfIndex = m_VegaToolStorage->GetToolIndexByName("SawRF");
	if (probeIndex == -1 || sawRfIndex == -1)
	{
		m_Controls.textBrowser_Action->append("There is no 'Probe' or 'SawRf' in the toolStorage!");
	}

	cout << "test 05" << endl;
	// 获取探针在 NDI 跟踪系统坐标系中的位置和方向数据
	mitk::NavigationData::Pointer nd_ndiToProbe = m_VegaSource->GetOutput(probeIndex);
	
	// 获取摆锯在NDI坐标系中的位置和方向数据
	mitk::NavigationData::Pointer nd_ndiToObjectRf = m_VegaSource->GetOutput(sawRfIndex);
	
	// 将探针的位置从NDI坐标系转换到摆锯的坐标系中
	mitk::NavigationData::Pointer nd_rfToProbe = GetNavigationDataInRef(nd_ndiToProbe, nd_ndiToObjectRf);
	
	// 从转换后的导航数据中提取探针尖端的位置, 这个位置现在是相对于参考物体坐标系的
	mitk::Point3D probeTipPointUnderRf = nd_rfToProbe->GetPosition();
	
	// 将计算得到的探针尖端位置添加到 点集 中
	pointSet_probeLandmark->InsertPoint(probeTipPointUnderRf);
	
	// 打印信息
	m_Controls.textBrowser_Action->append("Added saw landmark: " + QString::number(probeTipPointUnderRf[0]) +
		"/ " + QString::number(probeTipPointUnderRf[1]) + "/ " + QString::number(probeTipPointUnderRf[2]));

	return true;
}

bool HTONDI::OnCalibrateDrillClicked()
{
	/* 磨钻标定
	1. 绑定物体表面图像
	2. 记录探针针尖位置，注册磨钻标定点位置
	3. 配准摆锯表面位置，计算T_drill下的摆锯标定点的坐标
	4. 应用配准结果

	====已验证0813====
	*/

	m_Controls.textBrowser_Action->append("Action: Calibrate drill Node.");

	// 获取当前摆锯RF的位置
	auto drillRF = m_VegaToolStorage->GetToolByName("DrillRF");

	// 获取摆锯的表面数据
	auto drillRF_Surface = GetDataStorage()->GetNamedNode("Drill");

	if (drillRF.IsNull())
	{
		m_Controls.textBrowser_Action->append("ERROR: sawRF not found, try reload .IGTToolStorage");
		return false;
	}

	if (drillRF_Surface == nullptr)
	{
		m_Controls.textBrowser_Action->append("ERROR: Saw model Not Found!");
		return false;
	}

	// 获取工具
	m_drill = drillRF;

	// 开始标定点采集和标定过程
	// 取得粗配准点集 和 当前已经取得的点
	int drillRF_SurfaceSrcNum = drill_image->GetLandmarks()->GetSize();
	int drillRF_SurfaceTargetNum = drill_image->GetLandmarks_probe()->GetSize();

	if (drillRF_SurfaceSrcNum == drillRF_SurfaceTargetNum)
	{
		m_Controls.textBrowser_Action->append("--- Enough Drill landmarks have been collected ----");
		// 开始配准计算
		// 创建一个新的静态图像表面配准过滤器实例
		m_surfaceRegistrationStaticImageFilter = lancet::ApplySurfaceRegistratioinStaticImageFilter::New();

		// 将过滤器连接到Vega跟踪数据源
		m_surfaceRegistrationStaticImageFilter->ConnectTo(m_VegaSource);

		// 创建一个新的仿射变换矩阵，用于存储配准结果
		m_imageRegistrationMatrix = mitk::AffineTransform3D::New();

		// 更新导航图像的对象到参考框架的变换矩阵
		drill_image->UpdateObjectToRfMatrix();

		// 打印当前配准计算的结果
		// lanmark
		m_Controls.textBrowser_CalibrateRes->append("Avg landmark error:" + QString::number(drill_image->GetlandmarkRegis_avgError()));
		m_Controls.textBrowser_CalibrateRes->append("Max landmark error:" + QString::number(drill_image->GetlandmarkRegis_maxError()));

		// 将导航图像的对象到参考框架的变换矩阵从VTK格式转换为ITK格式，并存储
		mitk::TransferVtkMatrixToItkTransform(drill_image->GetT_Object2ReferenceFrame(), m_imageRegistrationMatrix.GetPointer());

		// 将配准矩阵应用到 DrillRF 工具中
		m_VegaToolStorage->GetToolByName("DrillRF")->SetToolRegistrationMatrix(m_imageRegistrationMatrix);

		// 将配准矩阵设置到静态图像表面配准过滤器中
		m_surfaceRegistrationStaticImageFilter->SetRegistrationMatrix(m_VegaToolStorage->GetToolByName("DrillRF")->GetToolRegistrationMatrix());

		// 设置参考框架的导航数据
		m_surfaceRegistrationStaticImageFilter->SetNavigationDataOfRF(m_VegaSource->GetOutput("DrillRF"));

		// 停止Vega可视化定时器
		m_VegaVisualizeTimer->stop();

		// 将Vega可视化器连接到静态图像表面配准过滤器
		m_VegaVisualizer->ConnectTo(m_surfaceRegistrationStaticImageFilter);

		// 重新启动Vega可视化定时器，显示配准计算的结果
		m_VegaVisualizeTimer->start();

		// 绑定物体表面
		drillRF->SetDataNode(drillRF_Surface);
		drillRF_Surface->SetVisibility(true);

		return true;
	}

	// 没有采点足够的时候，继续采点
	// 取出已经得到的Target配准点集
	auto pointSet_probeLandmark = drill_image->GetLandmarks_probe();
	auto pointSet_landmark = drill_image->GetLandmarks();

	// 开始选择标定点
	// 由远及近开始标定

	// 检测工具是否有效
	auto probeIndex = m_VegaToolStorage->GetToolIndexByName("ProbeRF");
	auto drillRfIndex = m_VegaToolStorage->GetToolIndexByName("DrillRF");
	if (probeIndex == -1 || drillRfIndex == -1)
	{
		m_Controls.textBrowser_Action->append("There is no 'Probe' or 'SawRf' in the toolStorage!");
	}
	// 获取探针在 NDI 跟踪系统坐标系中的位置和方向数据
	mitk::NavigationData::Pointer nd_ndiToProbe = m_VegaSource->GetOutput(probeIndex);

	// 获取磨钻在NDI坐标系中的位置和方向数据
	mitk::NavigationData::Pointer nd_ndiToObjectRf = m_VegaSource->GetOutput(drillRfIndex);

	// 将探针的位置从NDI坐标系转换到摆锯的坐标系中
	mitk::NavigationData::Pointer nd_rfToProbe = GetNavigationDataInRef(nd_ndiToProbe, nd_ndiToObjectRf);

	// 从转换后的导航数据中提取探针尖端的位置, 这个位置现在是相对于参考物体坐标系的
	mitk::Point3D probeTipPointUnderRf = nd_rfToProbe->GetPosition();

	// 将计算得到的探针尖端位置添加到 点集 中
	pointSet_probeLandmark->InsertPoint(probeTipPointUnderRf);

	// 打印信息
	m_Controls.textBrowser_Action->append("Added drill landmark: " + QString::number(probeTipPointUnderRf[0]) +
		"/ " + QString::number(probeTipPointUnderRf[1]) + "/ " + QString::number(probeTipPointUnderRf[2]));

	return true;
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
		// 展示钢板和点集合
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


// 粗配准流程
bool HTONDI::OnGetFemurLandmarkClicked()
{
	/* 股骨的三点粗配准点采集 = 股骨外踝点 股骨内踝点 采集
	1. 粗配准点采集，这里采集前两个点，股骨头中心采用拟合的方法获得

	需要探针及股骨的RF: ProbeRF FemurRF
	*/
	m_Controls.textBrowser_Action->append("Action: Get Femur Landmark Nodes.");

	cout << "test 01" << endl;
	// 获取股骨的表面数据
	auto FemurRF_Surface = GetDataStorage()->GetNamedNode("femurSurface");

	if (FemurRF_Surface == nullptr)
	{
		m_Controls.textBrowser_Action->append("ERROR: Femur model Not Found!");
		return false;
	}
	cout << "test 02" << endl;
	// 取出已经得到的Target配准点集
	auto pointSet_probeLandmark = femur_image->GetLandmarks_probe();
	auto pointSet_landmark = femur_image->GetLandmarks();


	// 测试用例
	// 162.517/ 379.001/ 147.178
	// 141.182/ 416.836/ 153.199
	if (true)
	{
		// 创建一个新的 Point3D 对象来存储测试用点的数据
		mitk::Point3D testPoint1;
		testPoint1[0] = 162.517;
		testPoint1[1] = 379.001;
		testPoint1[2] = 147.178;
		// 创建另一个 Point3D 对象来存储第二个测试用点的数据
		mitk::Point3D testPoint2;
		testPoint2[0] = 141.182;
		testPoint2[1] = 416.836;
		testPoint2[2] = 153.199;
		pointSet_probeLandmark->InsertPoint(testPoint1);
		pointSet_probeLandmark->InsertPoint(testPoint2);
	}


	// 开始选择标定点
	// 股骨外踝点 -> 股骨内踝点

	cout << "test 03" << endl;
	// 检测工具是否有效
	auto probeIndex = m_VegaToolStorage->GetToolIndexByName("ProbeRF");
	auto femurRfIndex = m_VegaToolStorage->GetToolIndexByName("FemurRF");

	if (probeIndex == -1 || femurRfIndex == -1)
	{
		m_Controls.textBrowser_Action->append("There is no 'Probe' or 'Femur' in the toolStorage!");
	}

	// 股骨粗配准点的采集只采2个点
	auto tmpNum = femur_image->GetLandmarks_probe()->GetSize();
	if (tmpNum >= 2)
	{
		m_Controls.textBrowser_Action->append("'Femur' landmark point done, please caculate hipcenter!");
		return false;
	}

	cout << "test 04" << endl;
	// 获取探针在 NDI 跟踪系统坐标系中的位置和方向数据
	mitk::NavigationData::Pointer nd_ndiToProbe = m_VegaSource->GetOutput(probeIndex);

	// 获取摆锯在NDI坐标系中的位置和方向数据
	mitk::NavigationData::Pointer nd_ndiToObjectRf = m_VegaSource->GetOutput(femurRfIndex);

	// 将探针的位置从NDI坐标系转换到摆锯的坐标系中
	mitk::NavigationData::Pointer nd_rfToProbe = GetNavigationDataInRef(nd_ndiToProbe, nd_ndiToObjectRf);

	// 从转换后的导航数据中提取探针尖端的位置, 这个位置现在是相对于参考物体坐标系的
	mitk::Point3D probeTipPointUnderRf = nd_rfToProbe->GetPosition();

	// 将计算得到的探针尖端位置添加到 点集 中
	pointSet_probeLandmark->InsertPoint(probeTipPointUnderRf);

	// 打印信息
	m_Controls.textBrowser_FemurRes->append("Added femur landmark: " + QString::number(probeTipPointUnderRf[0]) +
		"/ " + QString::number(probeTipPointUnderRf[1]) + "/ " + QString::number(probeTipPointUnderRf[2]));

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
	1. 配准计算
	2. 应用配准矩阵
	3. 绑定物体表面图像 

	股骨RF必须为FemurRf
	====已验证0813====
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
	pointSet_probeLandmark->InsertPoint(tmpHipCenter);

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

	// 开始配准计算
	// 创建一个新的静态图像表面配准过滤器实例
	m_surfaceRegistrationStaticImageFilter = lancet::ApplySurfaceRegistratioinStaticImageFilter::New();

	// 将过滤器连接到Vega跟踪数据源
	m_surfaceRegistrationStaticImageFilter->ConnectTo(m_VegaSource);

	// 创建一个新的仿射变换矩阵，用于存储配准结果
	m_imageRegistrationMatrix = mitk::AffineTransform3D::New();

	// 更新导航图像的对象到参考框架的变换矩阵
	femur_image->UpdateObjectToRfMatrix();

	// 打印当前配准计算的结果
	// lanmark
	m_Controls.textBrowser_FemurRes->append("Avg landmark error:" + QString::number(femur_image->GetlandmarkRegis_avgError()));
	m_Controls.textBrowser_FemurRes->append("Max landmark error:" + QString::number(femur_image->GetlandmarkRegis_maxError()));

	// 将导航图像的对象到参考框架的变换矩阵从VTK格式转换为ITK格式，并存储
	mitk::TransferVtkMatrixToItkTransform(femur_image->GetT_Object2ReferenceFrame(), m_imageRegistrationMatrix.GetPointer());

	// 将配准矩阵应用到 FemurRF 工具中
	m_VegaToolStorage->GetToolByName("FemurRF")->SetToolRegistrationMatrix(m_imageRegistrationMatrix);

	// 将配准矩阵设置到静态图像表面配准过滤器中
	m_surfaceRegistrationStaticImageFilter->SetRegistrationMatrix(m_VegaToolStorage->GetToolByName("FemurRF")->GetToolRegistrationMatrix());

	// 设置参考框架的导航数据
	m_surfaceRegistrationStaticImageFilter->SetNavigationDataOfRF(m_VegaSource->GetOutput("FemurRF"));

	// 停止Vega可视化定时器
	m_VegaVisualizeTimer->stop();

	// 将Vega可视化器连接到静态图像表面配准过滤器
	m_VegaVisualizer->ConnectTo(m_surfaceRegistrationStaticImageFilter);

	// 重新启动Vega可视化定时器，显示配准计算的结果
	m_VegaVisualizeTimer->start();

	// 绑定图像表面
	// 获取当前摆锯RF的位置
	auto FemurRF = m_VegaToolStorage->GetToolByName("FemurRF");

	// 获取摆锯的表面数据
	auto FemurRF_Surface = GetDataStorage()->GetNamedNode("femurSurface");

	if (FemurRF.IsNull())
	{
		m_Controls.textBrowser_Action->append("ERROR: femurRF not found, try reload .IGTToolStorage");
		return false;
	}

	if (FemurRF_Surface == nullptr)
	{
		m_Controls.textBrowser_Action->append("ERROR: Femur model Not Found!");
		return false;
	}

	// 绑定物体表面
	FemurRF->SetDataNode(FemurRF_Surface);
	FemurRF_Surface->SetVisibility(true);

	return true;
}


bool HTONDI::OnGetTibiaLandmarkClicked()
{
	/* 胫骨表面粗配准点采集：胫骨近端外侧点 胫骨近端内侧点 胫骨远端外踝点 胫骨远端内踝点
	1. 粗配准点采集
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
	auto pointSet_probeLandmark = tibia_image->GetLandmarks_probe();
	auto pointSet_landmark = tibia_image->GetLandmarks();

	// 开始选择标定点
	// 股骨外踝点 -> 股骨内踝点

	// 检测工具是否有效
	auto probeIndex = m_VegaToolStorage->GetToolIndexByName("ProbeRF");
	auto tibiaRfIndex = m_VegaToolStorage->GetToolIndexByName("TibiaRF");

	if (probeIndex == -1 || tibiaRfIndex == -1)
	{
		m_Controls.textBrowser_Action->append("There is no 'Probe' or 'Femur' in the toolStorage!");
	}
	// 获取探针在 NDI 跟踪系统坐标系中的位置和方向数据
	mitk::NavigationData::Pointer nd_ndiToProbe = m_VegaSource->GetOutput(probeIndex);

	// 获取摆锯在NDI坐标系中的位置和方向数据
	mitk::NavigationData::Pointer nd_ndiToObjectRf = m_VegaSource->GetOutput(tibiaRfIndex);

	// 将探针的位置从NDI坐标系转换到摆锯的坐标系中
	mitk::NavigationData::Pointer nd_rfToProbe = GetNavigationDataInRef(nd_ndiToProbe, nd_ndiToObjectRf);

	// 从转换后的导航数据中提取探针尖端的位置, 这个位置现在是相对于参考物体坐标系的
	mitk::Point3D probeTipPointUnderRf = nd_rfToProbe->GetPosition();

	// 将计算得到的探针尖端位置添加到 点集 中
	pointSet_probeLandmark->InsertPoint(probeTipPointUnderRf);

	// 打印信息
	m_Controls.textBrowser_TibieRes->append("Added tibia landmark src: " + QString::number(probeTipPointUnderRf[0]) +
		"/ " + QString::number(probeTipPointUnderRf[1]) + "/ " + QString::number(probeTipPointUnderRf[2]));

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
	m_imageRegistrationMatrix = mitk::AffineTransform3D::New();

	// 更新导航图像的对象到参考框架的变换矩阵
	tibia_image->UpdateObjectToRfMatrix();

	// 打印当前配准计算的结果
	// lanmark
	m_Controls.textBrowser_TibieRes->append("Avg landmark error:" + QString::number(tibia_image->GetlandmarkRegis_avgError()));
	m_Controls.textBrowser_TibieRes->append("Max landmark error:" + QString::number(tibia_image->GetlandmarkRegis_maxError()));

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
		m_Controls.textBrowser_Action->append("There is no 'Probe' or 'ObjectRf' in the toolStorage!");
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
		m_Controls.textBrowser_Action->append("ERROR: Femur model Not Found!");
		return false;
	}

	// 绑定物体表面
	TibiaRF->SetDataNode(TibiaRF_Surface);
	TibiaRF_Surface->SetVisibility(true);

	return true;
}


