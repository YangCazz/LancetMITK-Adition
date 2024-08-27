/*============================================================================
Robot-assisted Surgical Navigation Extension based on MITK
Copyright (c) Hangzhou LancetRobotics,CHINA
All rights reserved.
============================================================================*/

// .h head files here
// Blueberry
#include <berryISelectionService.h>
#include <berryIWorkbenchWindow.h>

// Qmitk
#include "HTONDI.h"
#include <QmitkAbstractView.h>
#include <QmitkDataStorageTreeModel.h>
#include <QmitkSingleNodeSelectionWidget.h>
#include <QtWidgets/QFileDialog>

// Qt
#include <QTimer>
#include <QMessageBox>
#include <QtWidgets/QFileDialog>

// VTK
#include <vtkPointLocator.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkMath.h>
#include <vtkImplicitPolyDataDistance.h>


// mitk
#include <mitkImage.h>
#include <mitkAffineTransform3D.h>
#include <mitkMatrixConvert.h>
#include <mitkNodePredicateAnd.h>
#include <mitkNodePredicateDataType.h>
#include <mitkNodePredicateNot.h>
#include <mitkNodePredicateOr.h>
#include <mitkNodePredicateProperty.h>
#include <mitkRenderingManager.h>
#include <mitkRotationOperation.h>
#include <mitkOperation.h>
#include <mitkTrackingDevice.h>
#include <mitkNavigationData.h>
#include <mitkPointOperation.h>
#include <mitkInteractionConst.h>
#include "mitkVirtualTrackingDevice.h"
#include "mitkVirtualTrackingTool.h"

// lancet
#include "lancetNavigationObjectVisualizationFilter.h"
#include "lancetApplyDeviceRegistratioinFilter.h"
#include "lancetApplySurfaceRegistratioinFilter.h"
#include "lancetApplySurfaceRegistratioinStaticImageFilter.h"
#include "lancetTreeCoords.h"
#include "lancetPathPoint.h"
#include "mitkTrackingDeviceSource.h"
#include "robotRegistration.h"



/*===============================================================
HTONDI_ImageRegistration.cpp
----------------------------------------------------------------
== ImageRegistration with NDI
== mitk object init
== Landmark 
== ICP
== Accuracy Test
== Bone Center Caculation
== Reuse Last Registration Result
===============================================================*/


void HTONDI::InitSurfaceSelector(QmitkSingleNodeSelectionWidget* widget)
{
	// surface data Load
	// data storage
	widget->SetDataStorage(GetDataStorage());
	// set nodes to show
	widget->SetNodePredicate(mitk::NodePredicateAnd::New(
		mitk::TNodePredicateDataType<mitk::Surface>::New(),
		mitk::NodePredicateNot::New(mitk::NodePredicateOr::New(mitk::NodePredicateProperty::New("helper object"),
			mitk::NodePredicateProperty::New("hidden object")))));

	// user can do nothing
	widget->SetSelectionIsOptional(true);
	widget->SetAutoSelectNewNodes(true);
	// Info set
	widget->SetEmptyInfo(QString("Please select a surface"));
	// pop Info
	widget->SetPopUpTitel(QString("Select surface"));
}

void HTONDI::InitPointSetSelector(QmitkSingleNodeSelectionWidget* widget)
{	
	// load node data
	widget->SetDataStorage(GetDataStorage());
	widget->SetNodePredicate(mitk::NodePredicateAnd::New(
		mitk::TNodePredicateDataType<mitk::PointSet>::New(),
		mitk::NodePredicateNot::New(mitk::NodePredicateOr::New(mitk::NodePredicateProperty::New("helper object"),
			mitk::NodePredicateProperty::New("hidden object")))));

	widget->SetSelectionIsOptional(true);
	widget->SetAutoSelectNewNodes(true);
	widget->SetEmptyInfo(QString("Please select a point set"));
	widget->SetPopUpTitel(QString("Select point set"));
}

bool HTONDI::OnSetNavigateClicked()
{	
	// 加载影像导航数据
	// The surface node should have no offset, i.e., should have an identity matrix!
	m_Controls.textBrowser_Action->append("Action: Set Navigation.");

	// Load surface and landmark nodes
	auto surfaceNode = m_Controls.setSTL_mitkSlecectButton->GetSelectedNode();
	auto landmarkSrcNode = m_Controls.setLandMark_mitkSlecectButton->GetSelectedNode();

	SetModelColor(landmarkSrcNode, 0.0f, 0.3f, 1.0f);
	SetNodeSize(landmarkSrcNode, 3.0f); // set node size
	SetModelOpacity(landmarkSrcNode, 0.5f); // set node opacity

	// check if is NONE
	if (surfaceNode == nullptr || landmarkSrcNode == nullptr)
	{
		m_Controls.textBrowser_Action->append("Get Navigated Image ERROR: Source surface or source landmarks is not ready!");
		return false;
	}

	navigatedImage = lancet::NavigationObject::New();

	// Get the GetGeometry Matrix of Surface => matrix
	auto matrix = dynamic_cast<mitk::Surface*>(surfaceNode->GetData())->GetGeometry()->GetVtkMatrix();

	// check if it is Identity
	// NOT ===> Set as Identity, Warning!!
	if (matrix->IsIdentity() == false)
	{
		vtkNew<vtkMatrix4x4> identityMatrix;
		identityMatrix->Identity();
		dynamic_cast<mitk::Surface*>(surfaceNode->GetData())->GetGeometry()->SetIndexToWorldTransformByVtkMatrix(identityMatrix);

		m_Controls.textBrowser_Action->append("Warning: the initial surface has a non-identity offset matrix; the matrix has been reset to identity!");
	}

	// Load navigated Image Nodes, like Mesh
	navigatedImage->SetDataNode(surfaceNode);
	// Set the feature Nodes of the Image
	navigatedImage->SetLandmarks(dynamic_cast<mitk::PointSet*>(landmarkSrcNode->GetData()));
	// Set Image flash as Name => "xxx"
	navigatedImage->SetReferencFrameName(surfaceNode->GetName());
	// Show func logs
	m_Controls.textBrowser_Action->append("--- navigatedImage has been set up ---");

	return true;
}

// model params set
void HTONDI::SetModelColor(mitk::DataNode::Pointer node, float r, float g, float b)
{
	node->SetColor(r, g, b);
}

void HTONDI::SetNodeSize(mitk::DataNode::Pointer node, float size)
{
	node->SetFloatProperty("pointsize", size);
}

void HTONDI::SetModelOpacity(mitk::DataNode::Pointer node, float opacity)
{
	node->SetOpacity(opacity);
}

bool HTONDI::OnShowNodeClicked()
{
	// show naviagate node on the surface
	m_Controls.textBrowser_Action->append("Action: Show Navigation Node.");
	// 计算标定点的标定误差
	// 从 navigatedImage 中提取表面节点
	auto surfacePointSet = GetDataStorage()->GetNamedNode("tibiaSurface");
	if (surfacePointSet == nullptr)
	{
		m_Controls.textBrowser_Action->append("Get Tibia ERROR, use whole Data.");
		auto surfacePointSet = navigatedImage->GetDataNode();
	}
	// 构建拟合曲面
	auto tibiaSurface = dynamic_cast<mitk::Surface*>(surfacePointSet->GetData());
	// 获取表面的几何信息
	auto polyData = tibiaSurface->GetVtkPolyData();
	// 创建一个点定位器，用于查找最近点
	vtkSmartPointer<vtkPointLocator> pointLocator = vtkSmartPointer<vtkPointLocator>::New();
	pointLocator->SetDataSet(polyData);
	pointLocator->BuildLocator();

	// 从 navigatedImage 中提取标记点信息
	auto landmarkPointSet = navigatedImage->GetLandmarks();
	if (landmarkPointSet)
	{
		m_Controls.textBrowser_Action->append("Landmark Points and Errors:");

		double totalError = 0.0;
		int pointCount = 0;

		for (auto it = landmarkPointSet->Begin(); it != landmarkPointSet->End(); ++it)
		{
			auto point = it->Value();

			// 查找最近的表面点
			vtkIdType closestPointId = pointLocator->FindClosestPoint(point.GetDataPointer());
			double closestPoint[3];
			polyData->GetPoint(closestPointId, closestPoint);

			// 计算距离
			double distance = sqrt(vtkMath::Distance2BetweenPoints(point.GetDataPointer(), closestPoint));

			// 输出点信息和误差
			QString pointInfo = QString("Point: %1 - (%2, %3, %4), Error: %5")
				.arg(it->Index() + 1)
				.arg(point[0])
				.arg(point[1])
				.arg(point[2])
				.arg(distance);
			m_Controls.textBrowser_Action->append(pointInfo);

			totalError += distance;
			++pointCount;
		}

		if (pointCount > 0)
		{
			double averageError = totalError / pointCount;
			m_Controls.textBrowser_Action->append(QString("Average Error: %1").arg(averageError));
		}
	}
	return true;
}

bool HTONDI::OnReuseMatrixClicked()
{
	// Reuse the Registration Matrix last
	m_Controls.textBrowser_Action->append("Action: Apply Last Registration Matrix.");

	// 检查是否有之前的配准矩阵
	if (m_PreviousImageRegistrationMatrix.IsNull())
	{
		m_Controls.textBrowser_Action->append("No previous registration available");
		return false;
	}

	// 创建一个新的静态图像表面配准过滤器实例
	m_surfaceRegistrationStaticImageFilter = lancet::ApplySurfaceRegistratioinStaticImageFilter::New();

	// 将过滤器连接到Vega跟踪数据源
	m_surfaceRegistrationStaticImageFilter->ConnectTo(m_VegaSource);

	// 将之前保存的配准矩阵设置到过滤器中
	m_surfaceRegistrationStaticImageFilter->SetRegistrationMatrix(m_PreviousImageRegistrationMatrix);

	// 设置参考框架的导航数据
	m_surfaceRegistrationStaticImageFilter->SetNavigationDataOfRF(m_VegaSource->GetOutput("BoneRF"));

	// 停止Vega可视化定时器
	m_VegaVisualizeTimer->stop();

	// 将Vega可视化器连接到静态图像表面配准过滤器
	m_VegaVisualizer->ConnectTo(m_surfaceRegistrationStaticImageFilter);

	// 重新启动Vega可视化定时器
	m_VegaVisualizeTimer->start();

	return true;
}

bool HTONDI::OnMoveTestClicked()
{
	// 运动测试--探针物体
	m_Controls.textBrowser_Action->append("Action: Move Test.");

	try {
		// 删除之前的测试节点
		auto previousTestNode = GetDataStorage()->GetNamedNode("MoveTestNode");
		if (previousTestNode) {
			GetDataStorage()->Remove(previousTestNode);
		}

		// 创建一个新的点集并设置初始位置
		mitk::PointSet::Pointer testPointSet = mitk::PointSet::New();
		mitk::Point3D initialPoint;
		initialPoint[0] = 0.0;
		initialPoint[1] = 0.0;
		initialPoint[2] = 0.0;
		testPointSet->InsertPoint(0, initialPoint);

		// 创建一个新的数据节点并设置点集
		mitk::DataNode::Pointer testPointNode = mitk::DataNode::New();
		testPointNode->SetData(testPointSet);
		testPointNode->SetName("MoveTestNode");

		// 设置节点颜色为红色
		testPointNode->SetColor(1.0, 0.0, 0.0); // RGB: Red
		testPointNode->SetFloatProperty("pointsize", 10);

		// 将数据节点添加到数据存储中
		GetDataStorage()->Add(testPointNode);

		// 启动定时器以移动节点
		QTimer* timer = new QTimer(this);
		connect(timer, &QTimer::timeout, this, [this, testPointSet]() {
			static double t = 0.0;
			t += 0.1;

			// 更新节点位置
			mitk::Point3D newPoint;
			newPoint[0] = 10.0 * std::cos(t); // X 方向的移动路径
			newPoint[1] = 10.0 * std::sin(t); // Y 方向的移动路径
			newPoint[2] = 0.0; // Z 方向保持不变
			testPointSet->SetPoint(0, newPoint);

			// 刷新视图
			mitk::RenderingManager::GetInstance()->RequestUpdateAll();
			});
		timer->start(100); // 每100毫秒更新一次

	}
	catch (const std::exception & e) {
		m_Controls.textBrowser_Action->append(QString("Error: %1").arg(e.what()));
	}
	return true;
}

bool HTONDI::OnMoveBoneClicked()
{
	// 运动测试-胫骨骨表面旋转
	m_Controls.textBrowser_Action->append("Action: Rotate Tibia.");

	try {
		// 获取股骨和胫骨的模型节点
		auto femurNode = GetDataStorage()->GetNamedNode("femurSurface");
		auto tibiaNode = GetDataStorage()->GetNamedNode("tibiaSurface");
		if (!femurNode || !tibiaNode) {
			throw std::runtime_error("Could not find femur or tibia node.");
		}

		// 确定胫骨和股骨的连接点
		// 从数据中读取中心点的位置
		auto tibiaCenterNode = GetDataStorage()->GetNamedNode("tibiaRotateCenter");
		if (!tibiaCenterNode) {
			throw std::runtime_error("Could not find tibia rotate center node.");
		}
		auto tibiaCenterPointSet = dynamic_cast<mitk::PointSet*>(tibiaCenterNode->GetData());
		if (!tibiaCenterPointSet || tibiaCenterPointSet->GetSize() == 0) {
			throw std::runtime_error("Could not get point set data from node or point set is empty.");
		}
		mitk::Point3D centerPoint = tibiaCenterPointSet->GetPoint(0);
		// 赋值
		double connectionPoint[3] = { centerPoint[0], centerPoint[1], centerPoint[2] }; // X, Y, Z 方向的连接点坐标


		// 启动定时器以旋转胫骨
		QTimer* timer = new QTimer(this);
		connect(timer, &QTimer::timeout, this, [this, tibiaNode, connectionPoint]() {
			static double t = 0.0;
			t += 0.1;

			// 计算旋转轴和角度
			double rotationAxis[3] = { 0.0, 0.0, 1.0 }; // Z 轴旋转
			double rotationDegree = 5.0 * std::sin(t); // 旋转角度

			// 使用 Rotate 方法旋转胫骨
			Rotate(connectionPoint, rotationAxis, rotationDegree, tibiaNode->GetData());
			});
		timer->start(100); // 每100毫秒更新一次

	}
	catch (const std::exception & e) {
		m_Controls.textBrowser_Action->append(QString("Error: %1").arg(e.what()));
		return false;
	}

	return true;
}

std::pair<Eigen::Vector3d, double> HTONDI::FitSphere(const std::vector<Eigen::Vector3d>& points)
{
	// 获取股骨头中心
	// 表面选点 + 球面中心拟合
	// 拟合球体，返回球心和半径
	int n = points.size();
	Eigen::MatrixXd A(n, 4);
	Eigen::VectorXd b(n);

	for (int i = 0; i < n; ++i) {
		const Eigen::Vector3d& p = points[i];
		A(i, 0) = 2 * p.x();
		A(i, 1) = 2 * p.y();
		A(i, 2) = 2 * p.z();
		A(i, 3) = 1;
		b(i) = p.squaredNorm();
	}

	Eigen::Vector4d x = A.colPivHouseholderQr().solve(b);
	Eigen::Vector3d center = x.head<3>();
	double radius = std::sqrt(x[3] + center.squaredNorm());

	return { center, radius };
}

bool HTONDI::OnFemurCenterClicked()
{
	// 计算股骨头的中心
	m_Controls.textBrowser_Action->append("Action: Get Femur Center.");

	try {
		// 从库中读取股骨头表面手动标记的点
		auto surfacePointSetNode = GetDataStorage()->GetNamedNode("femoralHead");
		if (!surfacePointSetNode) {
			throw std::runtime_error("Error: Could not find femoral head point set node.");
		}
		auto surfacePointSet = dynamic_cast<mitk::PointSet*>(surfacePointSetNode->GetData());
		if (!surfacePointSet) {
			throw std::runtime_error("Error: Could not get point set data from node.");
		}

		std::vector<Eigen::Vector3d> femoralHeadPoints;
		for (auto it = surfacePointSet->Begin(); it != surfacePointSet->End(); ++it) {
			auto point = it->Value();
			femoralHeadPoints.emplace_back(point[0], point[1], point[2]);
		}

		// 拟合球体并计算球心
		std::vector<Eigen::Vector3d> centers;
		int numPoints = femoralHeadPoints.size();
		for (int i = 0; i < numPoints; ++i) {
			std::vector<Eigen::Vector3d> subsetPoints;
			for (int j = 0; j < numPoints; ++j) {
				if (j != i) {
					subsetPoints.push_back(femoralHeadPoints[j]);
				}
			}
			auto [center, _] = FitSphere(subsetPoints);
			centers.push_back(center);
		}

		// 求所有球心的平均值
		if (centers.empty()) {
			throw std::runtime_error("Error: No valid groups of points to fit spheres.");
		}

		Eigen::Vector3d averageCenter(0, 0, 0);
		for (const auto& center : centers) {
			averageCenter += center;
		}
		averageCenter /= centers.size();

		// 输出股骨头中心
		m_Controls.textBrowser_Action->append(QString("Femur Head Center: (%1, %2, %3)").arg(averageCenter.x()).arg(averageCenter.y()).arg(averageCenter.z()));
		// 删除之前的中心点节点
		auto previousCenterNode = GetDataStorage()->GetNamedNode("hipCenterPoint");
		if (previousCenterNode) {
			GetDataStorage()->Remove(previousCenterNode);
		}

		// 创建一个新的点集并添加中心点
		mitk::PointSet::Pointer centerPointSet = mitk::PointSet::New();
		mitk::Point3D centerPoint;
		centerPoint[0] = averageCenter.x();
		centerPoint[1] = averageCenter.y();
		centerPoint[2] = averageCenter.z();
		centerPointSet->InsertPoint(0, centerPoint);
		
		// 同时向landmark中插入股骨头中心点
		// 取出已经得到的Target配准点集
		// auto pointSet_Landmark = femur_image->GetLandmarks();
		// pointSet_Landmark->InsertPoint(centerPoint);

		// 创建一个新的数据节点并设置点集
		mitk::DataNode::Pointer centerPointNode = mitk::DataNode::New();
		centerPointNode->SetData(centerPointSet);
		centerPointNode->SetName("hipCenterPoint");

		// 将数据节点添加到数据存储中
		GetDataStorage()->Add(centerPointNode);

		// 设置模型的基本参数，股骨透明度、节点大小等
		auto femurSurface = GetDataStorage()->GetNamedNode("femurSurface");
		SetModelColor(centerPointNode, 1.0f, 0.0f, 0.0f);
		SetNodeSize(centerPointNode, 10.0f);
		SetModelOpacity(femurSurface, 0.5f);

		// 计算得到股骨头中心后在面板显示
		m_Controls.checkBox_point10->setChecked(true);

		// 显示landmark点集中的数据量
		// m_Controls.textBrowser_Action->append("femurSurface landmarks: " + QString::number(femur_image->GetLandmarks()->GetSize()));
	}
	catch (const std::exception & e) {
		m_Controls.textBrowser_Action->append(QString("Error: %1").arg(e.what()));
		return false;
	}
	return true;
}

mitk::NavigationData::Pointer HTONDI::GetNavigationDataInRef(mitk::NavigationData::Pointer nd,
	mitk::NavigationData::Pointer nd_ref)
{
	// 计算得到导航物件在RF下的坐标
	mitk::NavigationData::Pointer res = mitk::NavigationData::New();
	res->Graft(nd);
	res->Compose(nd_ref->GetInverse());
	return res;
}

bool HTONDI::OnGetLandMarkPointClicked()
{
	// 取得粗配准点
	m_Controls.textBrowser_Action->append("Action: Collect LandMark Points.");
	if (navigatedImage == nullptr)
	{
		m_Controls.textBrowser_Action->append("Please setup the navigationObject first!");
		return false;
	}

	// 取得粗配准点集 和 当前已经取得的点
	int landmark_surfaceNum = navigatedImage->GetLandmarks()->GetSize();
	int landmark_rmNum = navigatedImage->GetLandmarks_probe()->GetSize();

	if (landmark_surfaceNum == landmark_rmNum)
	{
		m_Controls.textBrowser_Action->append("--- Enough landmarks have been collected ----");
		return true;
	}

	// 取出已经得到的Target配准点集
	auto pointSet_probeLandmark = navigatedImage->GetLandmarks_probe();
	auto pointSet_landmark = navigatedImage->GetLandmarks();
	
	
	
	// 第一次点击的时候，只做引导
	mitk::PointSet::Pointer collectedPoints = mitk::PointSet::New();
	if (firstnode)
	{	
		m_Controls.textBrowser_Action->append("Please collect node pos.");
		firstnode = false;
		// 对接下来需要采集的节点染色为红色，将已经采集好的节点染色为绿色
		// 首先，取出当前需要采集信息的节点
		mitk::PointSet::Pointer current = mitk::PointSet::New();
		currentpoint = pointSet_landmark->GetPoint(landmark_rmNum);
		current->InsertPoint(0, currentpoint);

		// 然后，将这个点可视化并设置颜色为红色
		// 创建一个新的数据节点并设置点集
		mitk::DataNode::Pointer currentPointNode = mitk::DataNode::New();
		currentPointNode->SetData(current);
		currentPointNode->SetName("CurrentLandmarkPoint");

		// 设置节点颜色为红色
		currentPointNode->SetColor(1.0, 0.0, 0.0); // RGB: Red
		currentPointNode->SetFloatProperty("pointsize", 3);

		// 将数据节点添加到数据存储中
		GetDataStorage()->Add(currentPointNode);

		// 初始化一个空对象
		collectedPointsNode->SetData(collectedPoints);
		collectedPointsNode->SetName("CollectedLandmarkPoints");
		collectedPointsNode->SetColor(0.0, 1.0, 0.0); // 已采集好的节点染色为绿色
		collectedPointsNode->SetFloatProperty("pointsize", 3.0f);
		GetDataStorage()->Add(collectedPointsNode);

		return true;
	}

	// 第二次点击开始采集信息
	// 取得探针和骨骼参考阵列在工具列表的下标
	auto probeIndex = m_VegaToolStorage->GetToolIndexByName("ProbeRF");
	auto objectRfIndex = m_VegaToolStorage->GetToolIndexByName("BoneRF");
	if (probeIndex == -1 || objectRfIndex == -1)
	{
		m_Controls.textBrowser_Action->append("There is no 'Probe' or 'ObjectRf' in the toolStorage!");
	}


	// 获取探针在 NDI 跟踪系统坐标系中的位置和方向数据
	mitk::NavigationData::Pointer nd_ndiToProbe = m_VegaSource->GetOutput(probeIndex);

	// 获取参考物体（可能是骨骼或其他固定参考物）在NDI坐标系中的位置和方向数据
	mitk::NavigationData::Pointer nd_ndiToObjectRf = m_VegaSource->GetOutput(objectRfIndex);

	// 将探针的位置从NDI坐标系转换到参考物体的坐标系中
	mitk::NavigationData::Pointer nd_rfToProbe = GetNavigationDataInRef(nd_ndiToProbe, nd_ndiToObjectRf);

	// 从转换后的导航数据中提取探针尖端的位置, 这个位置现在是相对于参考物体坐标系的
	mitk::Point3D probeTipPointUnderRf = nd_rfToProbe->GetPosition();

	// 将计算得到的探针尖端位置添加到 点集 中
	pointSet_probeLandmark->InsertPoint(probeTipPointUnderRf);
	
	// 将收集到的current点，放入到已经采集的点对象中
	collectedPoints->InsertPoint(landmark_rmNum, currentpoint);

	// 首先，取出当前需要采集信息的节点
	mitk::PointSet::Pointer current = mitk::PointSet::New();
	currentpoint = pointSet_landmark->GetPoint(landmark_rmNum);
	current->InsertPoint(0, currentpoint);

	// 然后，将这个点可视化并设置颜色为红色
	// 首先删除已经库中已经有的CurrentLandmarkPoint对象
	auto existingCurrentPointNode = GetDataStorage()->GetNamedNode("CurrentLandmarkPoint");
	if (existingCurrentPointNode)
	{
		GetDataStorage()->Remove(existingCurrentPointNode);
	}

	// 创建一个新的数据节点并设置点集
	mitk::DataNode::Pointer currentPointNode = mitk::DataNode::New();
	currentPointNode->SetData(current);
	currentPointNode->SetName("CurrentLandmarkPoint");

	// 设置节点颜色为红色
	currentPointNode->SetColor(1.0, 0.0, 0.0); // RGB: Red
	currentPointNode->SetFloatProperty("pointsize", 3);

	// 将数据节点添加到数据存储中
	GetDataStorage()->Add(currentPointNode);

	m_Controls.textBrowser_Action->append("Added landmark: " + QString::number(probeTipPointUnderRf[0]) +
		"/ " + QString::number(probeTipPointUnderRf[1]) + "/ " + QString::number(probeTipPointUnderRf[2]));

	return true;
}

bool HTONDI::OnGetICPPointClicked()
{
	// ICP Image Registration
	// 同 LandMark 方法
	m_Controls.textBrowser_Action->append("Action: Collect ICP Points.");
	
	// 检查navigatedImage是否已经初始化
	if (navigatedImage == nullptr)
	{
		m_Controls.textBrowser_Action->append("Please setup the navigationObject first!");
		return false;
	}

	// 获取ICP(迭代最近点)点集
	auto pointSet_probeIcp = navigatedImage->GetIcpPoints_probe();

	// 获取探针和参考物体(骨骼)的索引
	auto probeIndex = m_VegaToolStorage->GetToolIndexByName("ProbeRF");
	auto objectRfIndex = m_VegaToolStorage->GetToolIndexByName("BoneRF");

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
	m_Controls.textBrowser_Action->append("Added icp point: " + QString::number(probeTipPointUnderRf[0]) +
		"/ " + QString::number(probeTipPointUnderRf[1]) + "/ " + QString::number(probeTipPointUnderRf[2]));

	return true;
}

bool HTONDI::OnShowCollectionClicked()
{
	// Show the collected Nodes
	m_Controls.textBrowser_Action->append("Action: Show Collected Points.");

	// 获取收集到的节点信息
	auto CollectedLandmarkNodes = navigatedImage->GetLandmarks_probe();
	auto CollectedIcpNodes = navigatedImage->GetIcpPoints_probe();
	QString LandmarkNodes = "Landmark Points:\n";
	QString ICPNodes = "ICP Points:\n";;

	// 显示landmark点
	for (auto it = CollectedLandmarkNodes->Begin(); it != CollectedLandmarkNodes->End(); ++it)
	{
		auto point = it->Value();
		QString pointStr = QString("Point %1: (%2, %3, %4)")
			.arg(it->Index() + 1)
			.arg(point[0])
			.arg(point[1])
			.arg(point[2]);
		LandmarkNodes.append(pointStr);
	}
	// 显示icp点
	for (auto it = CollectedIcpNodes->Begin(); it != CollectedIcpNodes->End(); ++it)
	{
		auto point = it->Value();
		QString pointStr = QString("Point %1: (%2, %3, %4)")
			.arg(it->Index() + 1)
			.arg(point[0])
			.arg(point[1])
			.arg(point[2]);
		ICPNodes.append(pointStr);
	}
	m_Controls.textBrowser_Action->append(LandmarkNodes);
	m_Controls.textBrowser_Action->append(ICPNodes);
	return true;
}

bool HTONDI::OnCaculateRegistrationClicked()
{
	// Caculate Image Registration Matrix
	m_Controls.textBrowser_Action->append("Action: Caculate Registration.");
	// 进行图像配准，将结果应用到当前影像

	// 创建一个新的静态图像表面配准过滤器实例
	m_surfaceRegistrationStaticImageFilter = lancet::ApplySurfaceRegistratioinStaticImageFilter::New();
	// 将过滤器连接到Vega跟踪数据源
	m_surfaceRegistrationStaticImageFilter->ConnectTo(m_VegaSource);
	// 创建一个新的仿射变换矩阵，用于存储配准结果
	m_imageRegistrationMatrix = mitk::AffineTransform3D::New();
	// 更新导航图像的对象到参考框架的变换矩阵
	navigatedImage->UpdateObjectToRfMatrix();

	// 打印当前配准计算的结果
	// lanmark
	m_Controls.textBrowser_Action->append("Avg landmark error:" + QString::number(navigatedImage->GetlandmarkRegis_avgError()));
	m_Controls.textBrowser_Action->append("Max landmark error:" + QString::number(navigatedImage->GetlandmarkRegis_maxError()));
	// icp
	m_Controls.textBrowser_Action->append("Avg ICP error:" + QString::number(navigatedImage->GetIcpRegis_avgError()));
	m_Controls.textBrowser_Action->append("Max ICP error:" + QString::number(navigatedImage->GetIcpRegis_maxError()));

	// 将导航图像的对象到参考框架的变换矩阵从VTK格式转换为ITK格式，并存储
	mitk::TransferVtkMatrixToItkTransform(navigatedImage->GetT_Object2ReferenceFrame(), m_imageRegistrationMatrix.GetPointer());
	// 将配准矩阵应用到 BoneRF 工具中
	m_VegaToolStorage->GetToolByName("BoneRF")->SetToolRegistrationMatrix(m_imageRegistrationMatrix);
	// 将配准矩阵设置到静态图像表面配准过滤器中
	m_surfaceRegistrationStaticImageFilter->SetRegistrationMatrix(m_VegaToolStorage->GetToolByName("BoneRF")->GetToolRegistrationMatrix());
	// 设置参考框架的导航数据
	m_surfaceRegistrationStaticImageFilter->SetNavigationDataOfRF(m_VegaSource->GetOutput("BoneRF"));
	// 停止Vega可视化定时器
	m_VegaVisualizeTimer->stop();
	// 将Vega可视化器连接到静态图像表面配准过滤器
	m_VegaVisualizer->ConnectTo(m_surfaceRegistrationStaticImageFilter);
	// 重新启动Vega可视化定时器，显示配准计算的结果
	m_VegaVisualizeTimer->start();

	// 保存配准矩阵到 m_PreviousImageRegistrationMatrix
	m_PreviousImageRegistrationMatrix = m_imageRegistrationMatrix;

	/*
	这段代码的主要逻辑是执行静态图像的表面配准。它通过创建和配置配准过滤器，更新导航图像的变换矩阵，
	计算和显示配准误差，并将配准结果应用到 BoneRF 工具和过滤器中。
	最后，它重新启动Vega可视化定时器，以便在用户界面上显示配准结果。
	*/

	return true;
}

bool HTONDI::OnCheckAccuracyPointsClicked()
{
	// Check Points set
	m_Controls.textBrowser_Action->append("Action: Set Accuracy Test Points.");
	auto femoralHeadNode = GetDataStorage()->GetNamedNode("femoralHead");
	SetModelColor(femoralHeadNode, 0.0f, 0.3f, 0.9f);
	SetNodeSize(femoralHeadNode, 5.0f);
	SetModelOpacity(femoralHeadNode, 0.0f);
	return true;
}


bool HTONDI::OnCheckNode2NodeClicked()
{
	// Point to point Loss
	m_Controls.textBrowser_Action->append("Action: Check Accuracy Node2Node.");
	
	// 获取BoneRF工具的配准矩阵
	auto imageRfRegistrationMatrix = m_VegaToolStorage->GetToolByName("BoneRF")->GetToolRegistrationMatrix();

	// 创建一个新的VTK 4x4矩阵，用于存储配准矩阵
	vtkNew<vtkMatrix4x4> vtkImageRfRegistrationMatrix; // image to RF matrix

	// 将ITK格式的配准矩阵转换为VTK格式
	mitk::TransferItkTransformToVtkMatrix(imageRfRegistrationMatrix.GetPointer(), vtkImageRfRegistrationMatrix);

	// 检查配准矩阵是否为单位矩阵（即没有进行配准）
	if (vtkImageRfRegistrationMatrix->IsIdentity())
	{
		m_Controls.textBrowser_Action->append("No image registration available");
		return false;
	}

	// 获取选中的点集
	auto tmpPointSet = dynamic_cast<mitk::PointSet*>(m_Controls.selectCheckNode_mitkSlecectButton->GetSelectedNode()->GetData());
	auto imageCheckPoint = tmpPointSet->GetPoint(0);

	// 获取探针在NDI坐标系中的导航数据
	mitk::NavigationData::Pointer nd_ndiToProbe = m_VegaSource->GetOutput("ProbeRF");

	// 获取参考物体（BoneRF）在NDI坐标系中的导航数据
	mitk::NavigationData::Pointer nd_ndiToObjectRf = m_VegaSource->GetOutput("BoneRF");

	// 将探针的导航数据转换到参考物体的坐标系中
	mitk::NavigationData::Pointer nd_rfToProbe = GetNavigationDataInRef(nd_ndiToProbe, nd_ndiToObjectRf);

	// 获取探针尖端在参考物体坐标系中的位置
	mitk::Point3D probeTipPointUnderRf = nd_rfToProbe->GetPosition();

	// 创建一个新的VTK 4x4矩阵，用于存储探针尖端的位置
	vtkNew<vtkMatrix4x4> tmpMatrix;
	tmpMatrix->Identity();
	tmpMatrix->SetElement(0, 3, probeTipPointUnderRf[0]);
	tmpMatrix->SetElement(1, 3, probeTipPointUnderRf[1]);
	tmpMatrix->SetElement(2, 3, probeTipPointUnderRf[2]);

	// 创建一个新的VTK变换对象，并设置其矩阵
	vtkNew<vtkTransform> tmpTransform;
	tmpTransform->SetMatrix(tmpMatrix);
	tmpTransform->PostMultiply();
	tmpTransform->Concatenate(vtkImageRfRegistrationMatrix);
	tmpTransform->Update();

	// 获取变换后的矩阵
	auto transMatrix = tmpTransform->GetMatrix();

	// 提取探针尖端在影像坐标系中的位置
	double probeTipInImage[3]
	{
		transMatrix->GetElement(0,3),
		transMatrix->GetElement(1,3),
		transMatrix->GetElement(2,3),
	};

	// 计算探针尖端与影像检查点之间的距离
	double distance = sqrt(pow(probeTipInImage[0] - imageCheckPoint[0], 2) +
		pow(probeTipInImage[1] - imageCheckPoint[1], 2) +
		pow(probeTipInImage[2] - imageCheckPoint[2], 2));

	// 在用户界面上显示探针尖端与影像检查点之间的距离
	m_Controls.textBrowser_Action->append("Distance to the checkpoint:" + QString::number(distance));

	return true;
}

bool HTONDI::OnCheckNode2SurfaceClicked()
{
	// Point to surface Loss
	m_Controls.textBrowser_Action->append("Action: Check Accuracy Node2Surface.");

	// 获取BoneRF工具的配准矩阵
	auto imageRfRegistrationMatrix = m_VegaToolStorage->GetToolByName("BoneRF")->GetToolRegistrationMatrix();

	// 创建一个新的VTK 4x4矩阵，用于存储配准矩阵
	vtkNew<vtkMatrix4x4> vtkImageRfRegistrationMatrix; // image to RF matrix

	// 将ITK格式的配准矩阵转换为VTK格式
	mitk::TransferItkTransformToVtkMatrix(imageRfRegistrationMatrix.GetPointer(), vtkImageRfRegistrationMatrix);

	// 检查配准矩阵是否为单位矩阵（即没有进行配准）
	if (vtkImageRfRegistrationMatrix->IsIdentity())
	{
		m_Controls.textBrowser_Action->append("No image registration available");
		return false;
	}

	// 获取探针在NDI坐标系中的导航数据
	mitk::NavigationData::Pointer nd_ndiToProbe = m_VegaSource->GetOutput("ProbeRF");

	// 获取参考物体（BoneRF）在NDI坐标系中的导航数据
	mitk::NavigationData::Pointer nd_ndiToObjectRf = m_VegaSource->GetOutput("BoneRF");

	// 将探针的导航数据转换到参考物体的坐标系中
	mitk::NavigationData::Pointer nd_rfToProbe = GetNavigationDataInRef(nd_ndiToProbe, nd_ndiToObjectRf);

	// 获取探针尖端在参考物体坐标系中的位置
	mitk::Point3D probeTipPointUnderRf = nd_rfToProbe->GetPosition();

	// 创建一个新的VTK 4x4矩阵，用于存储探针尖端的位置
	vtkNew<vtkMatrix4x4> tmpMatrix;
	tmpMatrix->Identity();
	tmpMatrix->SetElement(0, 3, probeTipPointUnderRf[0]);
	tmpMatrix->SetElement(1, 3, probeTipPointUnderRf[1]);
	tmpMatrix->SetElement(2, 3, probeTipPointUnderRf[2]);

	// 创建一个新的VTK变换对象，并设置其矩阵
	vtkNew<vtkTransform> tmpTransform;
	tmpTransform->SetMatrix(tmpMatrix);
	tmpTransform->PostMultiply();
	tmpTransform->Concatenate(vtkImageRfRegistrationMatrix);
	tmpTransform->Update();

	// 获取变换后的矩阵
	auto transMatrix = tmpTransform->GetMatrix();

	// 提取探针尖端在影像坐标系中的位置
	double probeTipInImage[3]
	{
		transMatrix->GetElement(0,3),
		transMatrix->GetElement(1,3),
		transMatrix->GetElement(2,3),
	};

	// 获取表面数据，也就是原先加载的图像表面
	auto surfacePolyData = dynamic_cast<mitk::Surface*>(m_Controls.setSTL_mitkSlecectButton->GetSelectedNode()->GetData())->GetVtkPolyData();

	// 创建一个隐式多边形数据距离对象，用于计算点到表面的距离
	vtkNew<vtkImplicitPolyDataDistance> implicitPolyDataDistance;
	implicitPolyDataDistance->SetInput(surfacePolyData);

	// 计算探针尖端到表面的距离
	double currentError = implicitPolyDataDistance->EvaluateFunction(probeTipInImage);

	// 在用户界面上显示探针尖端到表面的距离
	m_Controls.textBrowser_Action->append("Distance to the surface: " + QString::number(currentError));

	return true;
}





bool HTONDI::OnLinkSurfaceClicked()
{
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
	// 4. 绑定物体表面
	sawRF->SetDataNode(sawRF_Surface);
	sawRF_Surface->SetVisibility(true);


	return true;
}

bool HTONDI::OnCollectPointerPosClicked()
{
	// Collect Pointer Pos
	m_Controls.textBrowser_Action->append("Action: Collect Pointer Pos.");
	// 开始采集接下来一段时间内的探针位置信息
	m_ProbePositions.clear();
	m_Transformations.clear();
	m_Controls.textBrowser_Action->append("Start collect probe data, please move slowly...");

	// 启动定时器，定期采集数据
	if (!m_ProbeDataCollectionTimer)
	{
		m_ProbeDataCollectionTimer = new QTimer(this);
		connect(m_ProbeDataCollectionTimer, &QTimer::timeout, this, &HTONDI::CollectProbeData);
	}
	m_ProbeDataCollectionTimer->start(100); // 每100ms采集一次数据

	return true;
}


bool HTONDI::OnVisualizeCollectedPoints()
{
    // Action
    m_Controls.textBrowser_Action->append("Action: Visualize Collected Points.");

    try {
        // 删除之前的测试节点
        auto previousCollectedPointsNode = GetDataStorage()->GetNamedNode("CollectedPointsNode");
        if (previousCollectedPointsNode) {
            GetDataStorage()->Remove(previousCollectedPointsNode);
        }

        auto previousCenterPointNode = GetDataStorage()->GetNamedNode("CenterPointNode");
        if (previousCenterPointNode) {
            GetDataStorage()->Remove(previousCenterPointNode);
        }

        auto previousProbeTipsNode = GetDataStorage()->GetNamedNode("ProbeTipsNode");
        if (previousProbeTipsNode) {
            GetDataStorage()->Remove(previousProbeTipsNode);
        }

        // 创建一个新的点集并设置采集到的点
        mitk::PointSet::Pointer pointSet = mitk::PointSet::New();
        for (size_t i = 0; i < m_ProbePositions.size(); ++i) {
            mitk::Point3D point;
            point[0] = m_ProbePositions[i][0] - m_Center[0];
            point[1] = m_ProbePositions[i][1] - m_Center[1];
            point[2] = m_ProbePositions[i][2] - m_Center[2];
            pointSet->InsertPoint(i, point);
        }

        // 添加球心点
        mitk::Point3D centerPoint;
        centerPoint[0] = 0.0;
        centerPoint[1] = 0.0;
        centerPoint[2] = 0.0;
        pointSet->InsertPoint(m_ProbePositions.size(), centerPoint);

        // 创建一个新的数据节点并设置点集
        mitk::DataNode::Pointer pointNode = mitk::DataNode::New();
        pointNode->SetData(pointSet);
        pointNode->SetName("CollectedPointsNode");
        // 设置节点颜色为红色
        pointNode->SetColor(1.0, 0.0, 0.0); // RGB: Red
        pointNode->SetFloatProperty("pointsize", 10);

        // 创建一个新的数据节点用于球心
        mitk::DataNode::Pointer centerNode = mitk::DataNode::New();
        mitk::PointSet::Pointer centerPointSet = mitk::PointSet::New();
        centerPointSet->InsertPoint(0, centerPoint);
        centerNode->SetData(centerPointSet);
        centerNode->SetName("CenterPointNode");
        centerNode->SetColor(0.0, 1.0, 0.0); // RGB: Green
        centerNode->SetFloatProperty("pointsize", 10);

        // 将数据节点添加到数据存储中
        GetDataStorage()->Add(pointNode);
        GetDataStorage()->Add(centerNode);

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

        // 创建一个新的点集并设置探针针尖在双目相机坐标系下的位置
        mitk::PointSet::Pointer probeTipPointSet = mitk::PointSet::New();
        for (size_t i = 0; i < probeTipsInCamera.size(); ++i) {
            mitk::Point3D point;
            point[0] = probeTipsInCamera[i][0];
            point[1] = probeTipsInCamera[i][1];
            point[2] = probeTipsInCamera[i][2];
            probeTipPointSet->InsertPoint(i, point);
        }

        // 创建一个新的数据节点并设置点集
        mitk::DataNode::Pointer probeTipPointNode = mitk::DataNode::New();
        probeTipPointNode->SetData(probeTipPointSet);
        probeTipPointNode->SetName("ProbeTipsNode");
        // 设置节点颜色为蓝色
        probeTipPointNode->SetColor(0.0, 0.0, 1.0); // RGB: Blue
        probeTipPointNode->SetFloatProperty("pointsize", 10);

        // 将数据节点添加到数据存储中
        GetDataStorage()->Add(probeTipPointNode);

        // 刷新视图
        mitk::RenderingManager::GetInstance()->RequestUpdateAll();
    }
    catch (const std::exception & e) {
        m_Controls.textBrowser_Action->append(QString("Error: %1").arg(e.what()));
    }
    return true;
}