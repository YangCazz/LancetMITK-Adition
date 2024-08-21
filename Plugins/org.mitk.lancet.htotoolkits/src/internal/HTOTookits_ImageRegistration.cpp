/*============================================================================
Robot-assisted Surgical Navigation Extension based on MITK
Copyright (c) Hangzhou LancetRobotics,CHINA
All rights reserved.
============================================================================*/

// Blueberry
#include <berryISelectionService.h>
#include <berryIWorkbenchWindow.h>

// Qmitk
#include "HTOTookits.h"

// Qt
#include <QMessageBox>
#include <QtWidgets/QFileDialog>

// VTK
#include <vtkPointLocator.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkMath.h>
#include <vtkImplicitPolyDataDistance.h>

// MITK
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

// Qmitk
#include <QmitkDataStorageTreeModel.h>
#include <QmitkSingleNodeSelectionWidget.h>

// IGT
#include <lancetVegaTrackingDevice.h>
#include <kukaRobotDevice.h>
#include <lancetApplyDeviceRegistratioinFilter.h>
#include <lancetTrackingDeviceSourceConfigurator.h>
#include <mitkNavigationToolStorageDeserializer.h>

// Lancet
#include "lancetTreeCoords.h"

// surface data
void HTOTookits::InitSurfaceSelector(QmitkSingleNodeSelectionWidget* widget)
{
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

// node data
void HTOTookits::InitPointSetSelector(QmitkSingleNodeSelectionWidget* widget)
{
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

// set navigation STL
bool HTOTookits::OnSetNavigateClicked()
{
	// The surface node should have no offset, i.e., should have an identity matrix!
	// Action
	m_Controls.textBrowser_Action->append("Action: Set Navigation.");

	// Load surface and landmark nodes
	auto surfaceNode = m_Controls.setSTL_mitkSlecectButton->GetSelectedNode();
	auto landmarkSrcNode = m_Controls.setLandMark_mitkSlecectButton->GetSelectedNode();

	SetModelColor(landmarkSrcNode, 0.0f, 0.3f, 1.0f);
	SetNodeSize(landmarkSrcNode, 3.0f); // 设置节点大小
	SetModelOpacity(landmarkSrcNode, 0.5f); // 设置透明度为50%

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

bool HTOTookits::OnShowNodeClicked()
{	
	// Action
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

//
bool HTOTookits::OnReuseMatrixClicked()
{
	// Action
	m_Controls.textBrowser_Action->append("Action: Apply Last Registration Matrix.");
	return true;
}

// 获取股骨头中心
// 表面选点 + 球面中心拟合
// 拟合球体，返回球心和半径
std::pair<Eigen::Vector3d, double> HTOTookits::FitSphere(const std::vector<Eigen::Vector3d>& points)
{
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
	Eigen::Vector3d center(x[0], x[1], x[2]);
	double radius = std::sqrt(x[3] + center.squaredNorm());

	return { center, radius };
}

bool HTOTookits::OnFemurCenterClicked()
{
	// Action
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
		auto previousCenterNode = GetDataStorage()->GetNamedNode("FemurHeadCenter");
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

		// 创建一个新的数据节点并设置点集
		mitk::DataNode::Pointer centerPointNode = mitk::DataNode::New();
		centerPointNode->SetData(centerPointSet);
		centerPointNode->SetName("FemurHeadCenter");

		// 将数据节点添加到数据存储中
		GetDataStorage()->Add(centerPointNode);

		// 设置模型的基本参数，股骨透明度、节点大小等
		auto femurSurface = GetDataStorage()->GetNamedNode("femurSurface");
		SetModelColor(centerPointNode, 1.0f, 0.0f, 0.0f);
		SetNodeSize(centerPointNode, 10.0f);
		SetModelOpacity(femurSurface, 0.5f);

	}
	catch (const std::exception & e) {
		m_Controls.textBrowser_Action->append(QString("Error: %1").arg(e.what()));
		return false;
	}
	return true;
}



//
bool HTOTookits::OnGetLandMarkPointClicked()
{
	// Action
	m_Controls.textBrowser_Action->append("Action: Collect LandMark Points.");
	return true;
}

//
bool HTOTookits::OnGetICPPointClicked()
{
	// Action
	m_Controls.textBrowser_Action->append("Action: Collect ICP Points.");
	return true;
}

//
bool HTOTookits::OnShowCollectionClicked()
{
	// Action
	m_Controls.textBrowser_Action->append("Action: Show Collected Points.");
	return true;
}

//
bool HTOTookits::OnTestRegistrationClicked()
{
	// Action
	m_Controls.textBrowser_Action->append("Action: Caculate Registration TEST.");
	return true;
}

//
bool HTOTookits::OnCaculateRegistrationClicked()
{
	// Action
	m_Controls.textBrowser_Action->append("Action: Caculate Registration.");
	return true;
}

//
bool HTOTookits::OnApplyRegistration()
{
	// Action
	m_Controls.textBrowser_Action->append("Action: Apply Recent Registration.");
	return true;
}

bool HTOTookits::OnCheckAccuracyPointsClicked()
{
	// Action
	m_Controls.textBrowser_Action->append("Action: Set Accuracy Test Points.");
	auto femoralHeadNode = GetDataStorage()->GetNamedNode("femoralHead");
	SetModelColor(femoralHeadNode, 0.0f, 0.3f, 0.9f);
	SetNodeSize(femoralHeadNode, 5.0f);
	SetModelOpacity(femoralHeadNode, 0.0f);
	return true;
}

//
bool HTOTookits::OnCheckNode2NodeClicked()
{
	// Action
	m_Controls.textBrowser_Action->append("Action: Check Accuracy Node2Node.");
	return true;
}

//
bool HTOTookits::OnCheckNode2SurfaceClicked()
{
	// Action
	m_Controls.textBrowser_Action->append("Action: Check Accuracy Node2Surface.");
	return true;
}

// 设置模型参数
void HTOTookits::SetModelColor(mitk::DataNode::Pointer node, float r, float g, float b)
{
	node->SetColor(r, g, b);
}

void HTOTookits::SetNodeSize(mitk::DataNode::Pointer node, float size)
{
	node->SetFloatProperty("pointsize", size);
}

void HTOTookits::SetModelOpacity(mitk::DataNode::Pointer node, float opacity)
{
	node->SetOpacity(opacity);
}


// 运动检测--探针物体
bool HTOTookits::OnMoveTestClicked()
{
	// Action
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



bool HTOTookits::OnMoveBoneClicked()
{
	// Action
	m_Controls.textBrowser_Action->append("Action: Rotate Tibia.");

	try {
		// 获取股骨和胫骨的模型节点
		auto femurNode = GetDataStorage()->GetNamedNode("femurSurface");
		auto tibiaNode = GetDataStorage()->GetNamedNode("tibiaSurface");
		if (!femurNode || !tibiaNode) {
			throw std::runtime_error("Error: Could not find femur or tibia node.");
		}

		// 确定胫骨和股骨的连接点
        // 从数据中读取中心点的位置
        auto tibiaCenterNode = GetDataStorage()->GetNamedNode("tibiaRotateCenter");
        if (!tibiaCenterNode) {
            throw std::runtime_error("Error: Could not find tibia rotate center node.");
        }
        auto tibiaCenterPointSet = dynamic_cast<mitk::PointSet*>(tibiaCenterNode->GetData());
        if (!tibiaCenterPointSet || tibiaCenterPointSet->GetSize() == 0) {
            throw std::runtime_error("Error: Could not get point set data from node or point set is empty.");
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

void HTOTookits::Translate(double direction[3], double length, mitk::BaseData* data)
{
	//if (data)
	//{
	//	mitk::Vector3D movementVector;
	//	movementVector[0] = direction[0] * length;
	//	movementVector[1] = direction[1] * length;
	//	movementVector[2] = direction[2] * length;

	//	auto* doOp = new mitk::PointOperation(mitk::OpMOVE, 0, movementVector, 0);
	//	data->GetGeometry()->ExecuteOperation(doOp);
	//	delete doOp;

	//	mitk::RenderingManager::GetInstance()->RequestUpdateAll();
	//}
}

void HTOTookits::Rotate(const double center[3], double direction[3], double counterclockwiseDegree, mitk::BaseData* data)
{
	if (data)
	{
		mitk::Point3D rotationCenter;
		rotationCenter[0] = center[0];
		rotationCenter[1] = center[1];
		rotationCenter[2] = center[2];

		mitk::Vector3D rotationAxis;
		rotationAxis[0] = direction[0];
		rotationAxis[1] = direction[1];
		rotationAxis[2] = direction[2];

		auto* doOp = new mitk::RotationOperation(mitk::OpROTATE, rotationCenter, rotationAxis, counterclockwiseDegree);
		data->GetGeometry()->ExecuteOperation(doOp);
		delete doOp;

		mitk::RenderingManager::GetInstance()->RequestUpdateAll();
	}
}