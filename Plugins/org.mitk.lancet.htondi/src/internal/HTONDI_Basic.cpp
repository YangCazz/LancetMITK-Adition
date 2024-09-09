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
#include <QMessageBox>
#include <QWidget>
#include <QTimer>
#include <QtCore\qmath.h>

// mitk image
#include <mitkImage.h>
#include <mitkAffineTransform3D.h>
#include <mitkMatrixConvert.h>
#include <vtkTransformFilter.h>


// vtk
#include <vtkPolyDataNormals.h>
#include <vtkDataArray.h>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkIdList.h>
#include <vtkMath.h>


//igt
#include <lancetVegaTrackingDevice.h>
//#include <kukaRobotDevice.h>
#include <lancetApplyDeviceRegistratioinFilter.h>
#include <mitkNavigationDataToPointSetFilter.h>
#include <lancetPathPoint.h>
#include <vtkQuaternion.h>

#include "lancetTrackingDeviceSourceConfigurator.h"
#include "mitkNavigationToolStorageDeserializer.h"
#include <QtWidgets\qfiledialog.h>

#include "mitkIGTIOException.h"
#include "mitkNavigationToolStorageSerializer.h"
#include "QmitkIGTCommonHelper.h"
#include "lancetTreeCoords.h"

//// 链接到对应的头文件
//#include "HTONDI_Basic.h"
//
//// 使用统一的命名空间
//using namespace lancetHTO;


/*===============================================================
HTONDI_Basic.cpp
----------------------------------------------------------------
== NDI connect and real time Toolkits visualization
== Probe Tip Registration
===============================================================*/
// 抓捕意外只在高频操作中





bool HTONDI::OnConnectNDIClicked()
{	
	/* NDI相机连接
		1. 设备连接
		2. 更新实时设备追踪状态
	*/

	m_Controls.textBrowser_Action->append("Action: NDI Connection.");

	// 1. 设备连接

	// 读取工具配置文件
	QString filename = QFileDialog::getOpenFileName(nullptr, tr("Open Tool Storage"), "/",
		tr("Tool Storage Files (*.IGTToolStorage)"));
	if (filename.isNull()) return false;

	// 反序列化工具存储， 加载工具的配置信息
	std::string errorMessage = "";
	mitk::NavigationToolStorageDeserializer::Pointer myDeserializer = mitk::NavigationToolStorageDeserializer::New(
		GetDataStorage());
	m_VegaToolStorage = myDeserializer->Deserialize(filename.toStdString());
	m_VegaToolStorage->SetName(filename.toStdString());

	// 创建NDI Vega跟踪设备
	MITK_INFO << "Vega tracking";
	lancet::NDIVegaTrackingDevice::Pointer vegaTrackingDevice = lancet::NDIVegaTrackingDevice::New();

	// 使用工厂类创建跟踪设备源
	lancet::TrackingDeviceSourceConfiguratorLancet::Pointer vegaSourceFactory =
		lancet::TrackingDeviceSourceConfiguratorLancet::New(m_VegaToolStorage, vegaTrackingDevice);
	

	// 2. 更新实时设备追踪状态

	// 设置可视化器
	m_VegaSource = vegaSourceFactory->CreateTrackingDeviceSource(m_VegaVisualizer);
	// 设置工具元数据集合
	m_VegaSource->SetToolMetaDataCollection(m_VegaToolStorage);

	// 连接并开始跟踪
	m_VegaSource->Connect();
	m_VegaSource->StartTracking();

	// 设置定时器来更新可视化
	if (m_VegaVisualizeTimer == nullptr)
	{
		m_VegaVisualizeTimer = new QTimer(this);
	}
	connect(m_VegaVisualizeTimer, SIGNAL(timeout()), this, SLOT(OnVegaVisualizeTimer()));
	connect(m_VegaVisualizeTimer, SIGNAL(timeout()), this, SLOT(UpdateToolStatusWidget()));
	
	// 可视化
	ShowToolStatus_Vega();

	// 启动定时器
	m_VegaVisualizeTimer->start(100);

	// 更新视图
	auto geo = this->GetDataStorage()->ComputeBoundingGeometry3D(this->GetDataStorage()->GetAll());
	mitk::RenderingManager::GetInstance()->InitializeViews(geo);

	m_Controls.textBrowser_Action->append("NDI Connected !!!");

	return true;
}

bool HTONDI::OnVegaVisualizeTimer()
{	
	/* 更新虚拟设备的可视化 */
	
	if (m_VegaVisualizer.IsNotNull())
	{
		m_VegaVisualizer->Update();
		this->RequestRenderWindowUpdate();
	}

	// Update the global variables which hold the camera data
	auto probeRFindex = m_VegaToolStorage->GetToolIndexByName("ProbeRF");
	auto femurRFindex = m_VegaToolStorage->GetToolIndexByName("FemurRF");
	auto tibiaRFindex = m_VegaToolStorage->GetToolIndexByName("TibiaRF");
	auto sawRFindex = m_VegaToolStorage->GetToolIndexByName("SawRF");
	auto drillRFindex = m_VegaToolStorage->GetToolIndexByName("DrillRF");
	auto calibratorRFindex = m_VegaToolStorage->GetToolIndexByName("CalibratorRF");

	mitk::NavigationData::Pointer nd_cameraToProbeRF = m_VegaSource->GetOutput(probeRFindex);
	mitk::NavigationData::Pointer nd_cameraToFemurRF = m_VegaSource->GetOutput(femurRFindex);
	mitk::NavigationData::Pointer nd_cameraToTibiaRF = m_VegaSource->GetOutput(tibiaRFindex);
	mitk::NavigationData::Pointer nd_cameraToSawRF = m_VegaSource->GetOutput(sawRFindex);
	mitk::NavigationData::Pointer nd_cameraToDrillRF = m_VegaSource->GetOutput(drillRFindex);
	mitk::NavigationData::Pointer nd_cameraToCalibratorRF = m_VegaSource->GetOutput(calibratorRFindex);

	bool m_Stat_cameraToProbeRF = nd_cameraToProbeRF->IsDataValid();
	bool m_Stat_cameraToFemurRF = nd_cameraToFemurRF->IsDataValid();
	bool m_Stat_cameraToTibiaRF = nd_cameraToTibiaRF->IsDataValid();
	bool m_Stat_cameraToSawRF = nd_cameraToSawRF->IsDataValid();
	bool m_Stat_cameraToDrillRF = nd_cameraToDrillRF->IsDataValid();
	bool m_Stat_cameraToCalibratorRF = nd_cameraToCalibratorRF->IsDataValid();

	if (m_Stat_cameraToProbeRF)
	{
		auto tmpMatrix = vtkSmartPointer<vtkMatrix4x4>::New();
		mitk::TransferItkTransformToVtkMatrix(nd_cameraToProbeRF->GetAffineTransform3D().GetPointer(), tmpMatrix);
		memcpy_s(m_T_cameraToProbeRF, sizeof(double) * 16, tmpMatrix->GetData(), sizeof(double) * 16);
		
		m_Controls.textBrowser_ProbeRF->clear();
		for (int i = 0; i < 16; i += 4) {
			m_Controls.textBrowser_ProbeRF->append(
				QString::number(m_T_cameraToProbeRF[i], 'f', 1) + "  " + 
				QString::number(m_T_cameraToProbeRF[i + 1], 'f', 1) + "  " + 
				QString::number(m_T_cameraToProbeRF[i + 2], 'f', 1) + "  " + 
				QString::number(m_T_cameraToProbeRF[i + 3], 'f', 1));
		}
	}
	else
	{
		m_Controls.textBrowser_ProbeRF->setText("None");
	}

	if (m_Stat_cameraToFemurRF)
	{
		auto tmpMatrix = vtkSmartPointer<vtkMatrix4x4>::New();
		mitk::TransferItkTransformToVtkMatrix(nd_cameraToFemurRF->GetAffineTransform3D().GetPointer(), tmpMatrix);
		memcpy_s(m_T_cameraToFemurRF, sizeof(double) * 16, tmpMatrix->GetData(), sizeof(double) * 16);

		m_Controls.textBrowser_FemurRF->clear();
		for (int i = 0; i < 16; i += 4) {
			m_Controls.textBrowser_FemurRF->append(
				QString::number(m_T_cameraToFemurRF[i], 'f', 1) + "  " +
				QString::number(m_T_cameraToFemurRF[i + 1], 'f', 1) + "  " +
				QString::number(m_T_cameraToFemurRF[i + 2], 'f', 1) + "  " +
				QString::number(m_T_cameraToFemurRF[i + 3], 'f', 1));
		}
	}
	else
	{
		m_Controls.textBrowser_FemurRF->setText("None");
	}

	if (m_Stat_cameraToTibiaRF)
	{
		auto tmpMatrix = vtkSmartPointer<vtkMatrix4x4>::New();
		mitk::TransferItkTransformToVtkMatrix(nd_cameraToTibiaRF->GetAffineTransform3D().GetPointer(), tmpMatrix);
		memcpy_s(m_T_cameraToTibiaRF, sizeof(double) * 16, tmpMatrix->GetData(), sizeof(double) * 16);

		m_Controls.textBrowser_TibiaRF->clear();
		for (int i = 0; i < 16; i += 4) {
			m_Controls.textBrowser_TibiaRF->append(
				QString::number(m_T_cameraToTibiaRF[i], 'f', 1) + "  " +
				QString::number(m_T_cameraToTibiaRF[i + 1], 'f', 1) + "  " +
				QString::number(m_T_cameraToTibiaRF[i + 2], 'f', 1) + "  " +
				QString::number(m_T_cameraToTibiaRF[i + 3], 'f', 1));
		}
	}
	else
	{
		m_Controls.textBrowser_TibiaRF->setText("None");
	}

	if (m_Stat_cameraToSawRF)
	{
		auto tmpMatrix = vtkSmartPointer<vtkMatrix4x4>::New();
		mitk::TransferItkTransformToVtkMatrix(nd_cameraToSawRF->GetAffineTransform3D().GetPointer(), tmpMatrix);
		memcpy_s(m_T_cameraToSawRF, sizeof(double) * 16, tmpMatrix->GetData(), sizeof(double) * 16);

		m_Controls.textBrowser_SawRF->clear();
		for (int i = 0; i < 16; i += 4) {
			m_Controls.textBrowser_SawRF->append(
				QString::number(m_T_cameraToSawRF[i], 'f', 1) + "  " +
				QString::number(m_T_cameraToSawRF[i + 1], 'f', 1) + "  " +
				QString::number(m_T_cameraToSawRF[i + 2], 'f', 1) + "  " +
				QString::number(m_T_cameraToSawRF[i + 3], 'f', 1));
		}
	}
	else
	{
		m_Controls.textBrowser_SawRF->setText("None");
	}

	if (m_Stat_cameraToDrillRF)
	{
		auto tmpMatrix = vtkSmartPointer<vtkMatrix4x4>::New();
		mitk::TransferItkTransformToVtkMatrix(nd_cameraToDrillRF->GetAffineTransform3D().GetPointer(), tmpMatrix);
		memcpy_s(m_T_cameraToDrillRF, sizeof(double) * 16, tmpMatrix->GetData(), sizeof(double) * 16);

		m_Controls.textBrowser_DrillRF->clear();
		for (int i = 0; i < 16; i += 4) {
			m_Controls.textBrowser_DrillRF->append(
				QString::number(m_T_cameraToDrillRF[i], 'f', 1) + "  " +
				QString::number(m_T_cameraToDrillRF[i + 1], 'f', 1) + "  " +
				QString::number(m_T_cameraToDrillRF[i + 2], 'f', 1) + "  " +
				QString::number(m_T_cameraToDrillRF[i + 3], 'f', 1));
		}
	}
	else
	{
		m_Controls.textBrowser_DrillRF->setText("None");
	}

	if (m_Stat_cameraToCalibratorRF)
	{
		auto tmpMatrix = vtkSmartPointer<vtkMatrix4x4>::New();
		mitk::TransferItkTransformToVtkMatrix(nd_cameraToCalibratorRF->GetAffineTransform3D().GetPointer(), tmpMatrix);
		memcpy_s(m_T_cameraToCalibratorRF, sizeof(double) * 16, tmpMatrix->GetData(), sizeof(double) * 16);

		m_Controls.textBrowser_CalibratorRF->clear();
		for (int i = 0; i < 16; i += 4) {
			m_Controls.textBrowser_CalibratorRF->append(
				QString::number(m_T_cameraToCalibratorRF[i], 'f', 1) + "  " +
				QString::number(m_T_cameraToCalibratorRF[i + 1], 'f', 1) + "  " +
				QString::number(m_T_cameraToCalibratorRF[i + 2], 'f', 1) + "  " +
				QString::number(m_T_cameraToCalibratorRF[i + 3], 'f', 1));
		}
	}
	else
	{
		m_Controls.textBrowser_CalibratorRF->setText("None");
	}

	return true;
}

bool HTONDI::UpdateToolStatusWidget()
{	
	/* 只更新Vega（NDI）工具的状态显示 */
	

	m_Controls.showVegaTool_StatusWidget->Refresh();
	m_Controls.showVegaTool_StatusWidget02->Refresh();
	// 如果Vega源未连接，函数直接返回
	if (m_VegaSource == nullptr)
	{
		m_Controls.textBrowser_Action->append("ERROR: NDI Connection.");
		return false;

	}
	return true;
}

bool HTONDI::ShowToolStatus_Vega()
{
	/* 展示NDI Vega设备的状态 */

	m_VegaNavigationData.clear();
	for (std::size_t i = 0; i < m_VegaSource->GetNumberOfOutputs(); i++)
	{
		m_VegaNavigationData.push_back(m_VegaSource->GetOutput(i));
	}
	// Demo最下方状态栏
	m_Controls.showVegaTool_StatusWidget->RemoveStatusLabels();
	m_Controls.showVegaTool_StatusWidget->SetShowPositions(true);
	m_Controls.showVegaTool_StatusWidget->SetTextAlignment(Qt::AlignLeft);
	m_Controls.showVegaTool_StatusWidget->SetNavigationDatas(&m_VegaNavigationData);
	m_Controls.showVegaTool_StatusWidget->ShowStatusLabels();

	// 设备连接界面状态栏
	m_Controls.showVegaTool_StatusWidget02->RemoveStatusLabels();
	m_Controls.showVegaTool_StatusWidget02->SetShowPositions(true);
	m_Controls.showVegaTool_StatusWidget02->SetTextAlignment(Qt::AlignLeft);
	m_Controls.showVegaTool_StatusWidget02->SetNavigationDatas(&m_VegaNavigationData);
	m_Controls.showVegaTool_StatusWidget02->ShowStatusLabels();
	return true;
}

// 拟合平面
std::pair<Eigen::Vector3d, Eigen::Vector3d> HTONDI::FitPlane3D(const std::vector<Eigen::Vector3d>& points)
{
	// 获取点集的大小
	int n = points.size();

	// 对于方程 AX = b
	// 创建 
	// n x 3 的矩阵 A 
	// n x 1 的向量 b
	Eigen::MatrixXd A(n, 3);
	Eigen::VectorXd b(n);

	// 填充矩阵 A 和向量 b
	// 假设平面方程为 ax + by + cz + d = 0，这里将常数项设为 1
	for (int i = 0; i < n; ++i) {
		const Eigen::Vector3d& p = points[i];
		A(i, 0) = p.x(); // 将点的 x 坐标放入矩阵 A 的第一列
		A(i, 1) = p.y(); // 将点的 y 坐标放入矩阵 A 的第二列
		A(i, 2) = p.z(); // 将点的 z 坐标放入矩阵 A 的第三列
		b(i) = 1;        // 设置常数 1
	}

	// 使用最小二乘法求解线性方程 Ax = b，其中 x 是平面方程的系数
	Eigen::Vector3d x = A.colPivHouseholderQr().solve(b);

	// 构造平面的法向量(a, b, c)
	Eigen::Vector3d normal(x[0], x[1], x[2]);

	// 归一化法向量
	double norm = normal.norm();
	if (norm > 0) {
		normal /= norm;
	}

	// 返回拟合平面上的一个点和平面的法向量
	return { x, normal };
}

// 使用最小二乘法拟合三维空间中的三个点到一个圆
std::pair<Eigen::Vector3d, double> HTONDI::FitCircle3D(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& p3) {
	Eigen::Vector3d center;
	double radius;

	// Calculate the vectors between the points
	Eigen::Vector3d v1 = p2 - p1;
	Eigen::Vector3d v2 = p3 - p1;

	// Calculate the normal to the plane defined by the points
	Eigen::Vector3d normal = v1.cross(v2).normalized();

	// Calculate the center of the circle
	center = p1 + 0.5 * v1 + ((v1.cross(v2)).cross(v1)).normalized() * (v1.norm() * v1.norm() * v2.dot(v1)) / (2 * v1.cross(v2).squaredNorm());

	// Calculate the radius of the circle
	radius = (p1 - center).norm();

	return std::make_pair(center, radius);
}



//bool HTONDI::GetPlaneNormal(vtkSmartPointer<vtkPolyData> plane, double normal[3], double center[3])
//{
//	/* 计算平面的法向量
//
//	采取平面上的三个点，计算向量叉乘
//	*/
//
//	auto tmpCenter = plane->GetCenter();
//
//	center[0] = *tmpCenter;
//	center[1] = *(tmpCenter + 1);
//	center[2] = *(tmpCenter + 2);
//
//	// Obtain the normal of the mitkSurface
//	double p0[3]; double p1[3]; double p2[3];
//
//	plane->GetCell(0)->GetPoints()->GetPoint(0, p0);
//	plane->GetCell(0)->GetPoints()->GetPoint(1, p1);
//	plane->GetCell(0)->GetPoints()->GetPoint(2, p2);
//
//	// 可视化
//	mitk::PointSet::Pointer tmpMitkPointSet = mitk::PointSet::New();
//	tmpMitkPointSet->InsertPoint(0, p0);
//	tmpMitkPointSet->InsertPoint(1, p1);
//	tmpMitkPointSet->InsertPoint(2, p2);
//
//	// 否则创建新的平面
//	auto tmpNodes = GetDataStorage()->GetNamedNode("TmpPoint");
//	if (tmpNodes) {
//		GetDataStorage()->Remove(tmpNodes);
//		//// 如果已经创建则更新其信息
//		//tmpNodes->SetData(mitkPointSetRealTime);
//	}
//	mitk::DataNode::Pointer pointSetInPlaneCutPlane = mitk::DataNode::New();
//	pointSetInPlaneCutPlane->SetName("TmpPoint");
//	// 红色，大小 5.0
//	pointSetInPlaneCutPlane->SetColor(0.0, 0.0, 1.0);
//	pointSetInPlaneCutPlane->SetData(tmpMitkPointSet);
//	pointSetInPlaneCutPlane->SetFloatProperty("pointsize", 5.0);
//	GetDataStorage()->Add(pointSetInPlaneCutPlane);
//
//	Eigen::Vector3d a(*p0, *(p0 + 1), *(p0 + 2));
//	Eigen::Vector3d b(*p1, *(p1 + 1), *(p1 + 2));
//	Eigen::Vector3d c(*p2, *(p2 + 1), *(p2 + 2));
//
//	Eigen::Vector3d tmpVector0 = b - a;
//	Eigen::Vector3d tmpVector1 = c - a;
//
//	Eigen::Vector3d normalVector = tmpVector0.cross(tmpVector1);
//	// 将法向量归一化为单位向量
//	normalVector.normalize();
//
//	normal[0] = normalVector[0];
//	normal[1] = normalVector[1];
//	normal[2] = normalVector[2];
//
//	mitk::RenderingManager::GetInstance()->RequestUpdateAll();
//	return true;
//}
//
//
//// Function to extract three points from a surface representing a plane
///*
//Eigen::Vector3d HTONDI::ExtractNormalFromPlane(const std::string& planeName)
//{
//	/* 计算平面的法向量
//		提取平面上的三个点，计算法向量
//	*/
//	auto PlaneSource = dynamic_cast<mitk::Surface*>(GetDataStorage()->GetNamedNode(planeName)->GetData());
//	mitk::Point3D p1, p2, p3;
//	Eigen::Vector3d normal;
//
//	if (PlaneSource)
//	{
//		// 需要先应用转化器才能正常提取数据
//		auto surfacePoints = PlaneSource->GetVtkPolyData();
//		// 创建转化器
//		vtkNew<vtkTransform> cutPlaneTransform;
//		cutPlaneTransform->SetMatrix(PlaneSource->GetGeometry()->GetVtkMatrix());
//		// 应用转化器
//		vtkNew<vtkTransformFilter> cutPlaneTransformFilter;
//		cutPlaneTransformFilter->SetTransform(cutPlaneTransform);
//		cutPlaneTransformFilter->SetInputData(surfacePoints);
//		cutPlaneTransformFilter->Update();
//		// 数据拷贝
//		vtkNew<vtkPolyData> tmpVtkSurface;
//		tmpVtkSurface->DeepCopy(cutPlaneTransformFilter->GetPolyDataOutput());
//
//		// 数据采集
//		if (tmpVtkSurface->GetNumberOfPoints() >= 3)
//		{
//			p1 = surfacePoints->GetPoint(1);
//			p2 = surfacePoints->GetPoint(2);
//			p3 = surfacePoints->GetPoint(3);
//
//			Eigen::Vector3d a(p1[0], p1[1], p1[2]);
//			Eigen::Vector3d b(p2[0], p2[1], p2[2]);
//			Eigen::Vector3d c(p3[0], p3[1], p3[2]);
//
//			Eigen::Vector3d tmpVector0 = b - a;
//			Eigen::Vector3d tmpVector1 = c - a;
//
//			Eigen::Vector3d normalVector = tmpVector0.cross(tmpVector1);
//			// 将法向量归一化为单位向量
//			normalVector.normalize();
//			normal = normalVector;
//
//
//			// 可视化
//			mitk::PointSet::Pointer tmpMitkPointSet = mitk::PointSet::New();
//			tmpMitkPointSet->InsertPoint(0, p1);
//			tmpMitkPointSet->InsertPoint(1, p2);
//			tmpMitkPointSet->InsertPoint(2, p3);
//
//			// 否则创建新的平面
//			auto tmpNodes = GetDataStorage()->GetNamedNode("TmpPoint");
//			if (tmpNodes) {
//				GetDataStorage()->Remove(tmpNodes);
//				//// 如果已经创建则更新其信息
//				//tmpNodes->SetData(mitkPointSetRealTime);
//			}
//			mitk::DataNode::Pointer pointSetInPlaneCutPlane = mitk::DataNode::New();
//			pointSetInPlaneCutPlane->SetName("TmpPoint");
//			// 红色，大小 5.0
//			pointSetInPlaneCutPlane->SetColor(0.0, 0.0, 1.0);
//			pointSetInPlaneCutPlane->SetData(tmpMitkPointSet);
//			pointSetInPlaneCutPlane->SetFloatProperty("pointsize", 5.0);
//			GetDataStorage()->Add(pointSetInPlaneCutPlane);
//		}
//		else
//		{
//			m_Controls.textBrowser_Action->append("Node < 3");
//		}
//	}
//	else
//	{
//		m_Controls.textBrowser_Action->append("plane is not ready");
//	}
//
//	return normal;
//}


bool HTONDI::GetPlaneNormal(vtkSmartPointer<vtkPolyData> plane, double normal[3], double center[3])
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

	// 可视化
	mitk::PointSet::Pointer tmpMitkPointSet = mitk::PointSet::New();
	tmpMitkPointSet->InsertPoint(0, p0);
	tmpMitkPointSet->InsertPoint(1, p1);
	tmpMitkPointSet->InsertPoint(2, p2);

	// 否则创建新的平面
	auto tmpNodes = GetDataStorage()->GetNamedNode("TmpPoint");
	if (tmpNodes) {
		GetDataStorage()->Remove(tmpNodes);
		//// 如果已经创建则更新其信息
		//tmpNodes->SetData(mitkPointSetRealTime);
	}
	mitk::DataNode::Pointer pointSetInPlaneCutPlane = mitk::DataNode::New();
	pointSetInPlaneCutPlane->SetName("TmpPoint");
	// 红色，大小 5.0
	pointSetInPlaneCutPlane->SetColor(0.0, 0.0, 1.0);
	pointSetInPlaneCutPlane->SetData(tmpMitkPointSet);
	pointSetInPlaneCutPlane->SetFloatProperty("pointsize", 5.0);
	GetDataStorage()->Add(pointSetInPlaneCutPlane);

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

	mitk::RenderingManager::GetInstance()->RequestUpdateAll();
	return true;
}


// Function to extract three points from a surface representing a plane
Eigen::Vector3d HTONDI::ExtractNormalFromPlane(const std::string& planeName)
{
	/* 计算平面的法向量
		提取平面上的三个点，计算法向量
	*/

	// 基于当前水平截骨面的规划来生成上升截骨面
	cout << "test 02-01" << endl;
	auto cutPlaneNode = GetDataStorage()->GetNamedNode(planeName);
	
	mitk::Point3D p1, p2, p3;
	Eigen::Vector3d normal;

	if (cutPlaneNode)
	{
		cout << "test 02-02" << endl;
		auto mitkCutPlane_0 = dynamic_cast<mitk::Surface*>(cutPlaneNode->GetData());
		cout << "test 02-02-01" << endl;
		auto tmpVtkSurface_initial = mitkCutPlane_0->GetVtkPolyData();

		cout << "test 02-03" << endl;
		vtkNew<vtkTransform> cutPlaneTransform;
		cutPlaneTransform->SetMatrix(mitkCutPlane_0->GetGeometry()->GetVtkMatrix());

		cout << "test 02-04" << endl;
		vtkNew<vtkTransformFilter> cutPlaneTransformFilter;
		cutPlaneTransformFilter->SetTransform(cutPlaneTransform);
		cutPlaneTransformFilter->SetInputData(tmpVtkSurface_initial);
		cutPlaneTransformFilter->Update();

		cout << "test 02-05" << endl;
		// 创建新平面
		vtkNew<vtkPolyData> tmpVtkSurface;
		tmpVtkSurface->DeepCopy(cutPlaneTransformFilter->GetPolyDataOutput());

		cout << "test 02-06" << endl;
		// 数据采集
		if (tmpVtkSurface->GetNumberOfPoints() >= 3)
		{
			p1 = tmpVtkSurface->GetPoint(1);
			p2 = tmpVtkSurface->GetPoint(2);
			p3 = tmpVtkSurface->GetPoint(3);

			Eigen::Vector3d a(p1[0], p1[1], p1[2]);
			Eigen::Vector3d b(p2[0], p2[1], p2[2]);
			Eigen::Vector3d c(p3[0], p3[1], p3[2]);

			Eigen::Vector3d tmpVector0 = b - a;
			Eigen::Vector3d tmpVector1 = c - a;

			Eigen::Vector3d normalVector = tmpVector0.cross(tmpVector1);
			// 将法向量归一化为单位向量
			normalVector.normalize();
			normal = normalVector;

			/*
			// 可视化
			mitk::PointSet::Pointer tmpMitkPointSet = mitk::PointSet::New();
			tmpMitkPointSet->InsertPoint(0, p1);
			tmpMitkPointSet->InsertPoint(1, p2);
			tmpMitkPointSet->InsertPoint(2, p3);

			// 否则创建新的平面
			auto tmpNodes = GetDataStorage()->GetNamedNode("TmpPoint");
			if (tmpNodes) {
				GetDataStorage()->Remove(tmpNodes);
				//// 如果已经创建则更新其信息
				//tmpNodes->SetData(mitkPointSetRealTime);
			}
			mitk::DataNode::Pointer pointSetInPlaneCutPlane = mitk::DataNode::New();
			pointSetInPlaneCutPlane->SetName("TmpPoint");
			// 红色，大小 5.0
			pointSetInPlaneCutPlane->SetColor(0.0, 0.0, 1.0);
			pointSetInPlaneCutPlane->SetData(tmpMitkPointSet);
			pointSetInPlaneCutPlane->SetFloatProperty("pointsize", 5.0);
			GetDataStorage()->Add(pointSetInPlaneCutPlane);
			*/
			cout << "test 02-07" << endl;
			//GetDataStorage()->Modified();
			//mitk::RenderingManager::GetInstance()->RequestUpdateAll();
		}
	}
	return normal;
}

void HTONDI::CaculateCutPlaneMiss01()
{
	/* 计算水平截骨夹角误差 
		当前截骨面和规划水平截骨面的夹角
	*/
	
	Eigen::Vector3d normalPlane01, normalSaw;
	cout << "test 01-01" << endl;
	normalPlane01 = ExtractNormalFromPlane("1st cut plane");
	cout << "test 01-02" << endl;
	normalSaw = ExtractNormalFromPlane("StateCutPlane01");
	cout << "test 01-03" << endl;
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
	cout << "test 01-04" << endl;
	// 输出夹角到实际的位置
	m_Controls.textBrowser_AxialCut->append("Real time Angle Miss: " + QString::number(angleInDegrees));
}

void HTONDI::CaculateCutPlaneMiss02()
{
	/* 计算上升截骨夹角误差 
		当前截骨面和水平截骨面的夹角
	*/
	Eigen::Vector3d normalPlane01, normalSaw;
	cout << "test 01-01" << endl;
	normalPlane01 = ExtractNormalFromPlane("1st cut plane");
	cout << "test 01-02" << endl;
	normalSaw = ExtractNormalFromPlane("StateCutPlane02");
	cout << "test 01-03" << endl;
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

	cout << "test 01-04" << endl;
	// 输出夹角到实际的位置
	m_Controls.textBrowser_SagCut->append("Real time Angle Miss: " + QString::number(angleInDegrees));
}

// 计算平面上的若干点相对于法向量和原点确定的坐标系下的坐标位置
std::vector<mitk::Point3D> HTONDI::CalculateRelativePoints(const std::vector<mitk::Point3D>& points, const Eigen::Vector3d& normalVector, const Eigen::Vector3d& origin)
{
	std::vector<mitk::Point3D> relativePoints;
	for (const auto& point : points) {
		Eigen::Vector3d relativePosition = Eigen::Vector3d(point[0], point[1], point[2]) - origin;
		double distance = relativePosition.dot(normalVector);
		Eigen::Vector3d relativePoint = Eigen::Vector3d(point[0], point[1], point[2]) - distance * normalVector;
		mitk::Point3D newPoint;
		newPoint[0] = relativePoint.x();
		newPoint[1] = relativePoint.y();
		newPoint[2] = relativePoint.z();
		relativePoints.push_back(newPoint);
	}
	return relativePoints;
}

// 计算实际位置
std::vector<mitk::Point3D> HTONDI::CalculateActualPoints(const std::vector<mitk::Point3D>& relativePoints, const Eigen::Vector3d& normalVector, const Eigen::Vector3d& origin)
{
	std::vector<mitk::Point3D> actualPoints;
	for (const auto& point : relativePoints) {
		Eigen::Vector3d relativePosition = Eigen::Vector3d(point[0], point[1], point[2]);
		Eigen::Vector3d actualPosition = origin + relativePosition + (normalVector.dot(relativePosition)) * normalVector;
		mitk::Point3D newPoint;
		newPoint[0] = actualPosition.x();
		newPoint[1] = actualPosition.y();
		newPoint[2] = actualPosition.z();
		actualPoints.push_back(newPoint);
	}
	return actualPoints;
}

double HTONDI::DistancePointToLine(double pointA[3], double pointB[3], double pointC[3])
{
	// 计算直线AB的向量
	double vectorAB[3] = { pointB[0] - pointA[0], pointB[1] - pointA[1], pointB[2] - pointA[2] };

	// 计算AC向量
	double vectorAC[3] = { pointC[0] - pointA[0], pointC[1] - pointA[1], pointC[2] - pointA[2] };

	// 计算投影向量
	double projection = (vectorAC[0] * vectorAB[0] + vectorAC[1] * vectorAB[1] + vectorAC[2] * vectorAB[2]) /
		(vectorAB[0] * vectorAB[0] + vectorAB[1] * vectorAB[1] + vectorAB[2] * vectorAB[2]);

	// 计算垂直向量
	double vectorAP[3] = { vectorAC[0] - projection * vectorAB[0],
						  vectorAC[1] - projection * vectorAB[1],
						  vectorAC[2] - projection * vectorAB[2] };

	// 计算距离
	double distance = sqrt(vectorAP[0] * vectorAP[0] + vectorAP[1] * vectorAP[1] + vectorAP[2] * vectorAP[2]);

	return distance;
}