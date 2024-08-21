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
	SetNodeSize(landmarkSrcNode, 3.0f); // ���ýڵ��С
	SetModelOpacity(landmarkSrcNode, 0.5f); // ����͸����Ϊ50%

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
	// ����궨��ı궨���
	// �� navigatedImage ����ȡ����ڵ�
	auto surfacePointSet = GetDataStorage()->GetNamedNode("tibiaSurface");
	if (surfacePointSet == nullptr)
	{
		m_Controls.textBrowser_Action->append("Get Tibia ERROR, use whole Data.");
		auto surfacePointSet = navigatedImage->GetDataNode();
	}
	// �����������
	auto tibiaSurface = dynamic_cast<mitk::Surface*>(surfacePointSet->GetData());
	// ��ȡ����ļ�����Ϣ
	auto polyData = tibiaSurface->GetVtkPolyData();
	// ����һ���㶨λ�������ڲ��������
	vtkSmartPointer<vtkPointLocator> pointLocator = vtkSmartPointer<vtkPointLocator>::New();
	pointLocator->SetDataSet(polyData);
	pointLocator->BuildLocator();

	// �� navigatedImage ����ȡ��ǵ���Ϣ
	auto landmarkPointSet = navigatedImage->GetLandmarks();
	if (landmarkPointSet)
	{
		m_Controls.textBrowser_Action->append("Landmark Points and Errors:");

		double totalError = 0.0;
		int pointCount = 0;

		for (auto it = landmarkPointSet->Begin(); it != landmarkPointSet->End(); ++it)
		{
			auto point = it->Value();

			// ��������ı����
			vtkIdType closestPointId = pointLocator->FindClosestPoint(point.GetDataPointer());
			double closestPoint[3];
			polyData->GetPoint(closestPointId, closestPoint);

			// �������
			double distance = sqrt(vtkMath::Distance2BetweenPoints(point.GetDataPointer(), closestPoint));

			// �������Ϣ�����
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

// ��ȡ�ɹ�ͷ����
// ����ѡ�� + �����������
// ������壬�������ĺͰ뾶
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
		// �ӿ��ж�ȡ�ɹ�ͷ�����ֶ���ǵĵ�
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

		// ������岢��������
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

		// ���������ĵ�ƽ��ֵ
		if (centers.empty()) {
			throw std::runtime_error("Error: No valid groups of points to fit spheres.");
		}

		Eigen::Vector3d averageCenter(0, 0, 0);
		for (const auto& center : centers) {
			averageCenter += center;
		}
		averageCenter /= centers.size();

		// ����ɹ�ͷ����
		m_Controls.textBrowser_Action->append(QString("Femur Head Center: (%1, %2, %3)").arg(averageCenter.x()).arg(averageCenter.y()).arg(averageCenter.z()));
		// ɾ��֮ǰ�����ĵ�ڵ�
		auto previousCenterNode = GetDataStorage()->GetNamedNode("FemurHeadCenter");
		if (previousCenterNode) {
			GetDataStorage()->Remove(previousCenterNode);
		}

		// ����һ���µĵ㼯��������ĵ�
		mitk::PointSet::Pointer centerPointSet = mitk::PointSet::New();
		mitk::Point3D centerPoint;
		centerPoint[0] = averageCenter.x();
		centerPoint[1] = averageCenter.y();
		centerPoint[2] = averageCenter.z();
		centerPointSet->InsertPoint(0, centerPoint);

		// ����һ���µ����ݽڵ㲢���õ㼯
		mitk::DataNode::Pointer centerPointNode = mitk::DataNode::New();
		centerPointNode->SetData(centerPointSet);
		centerPointNode->SetName("FemurHeadCenter");

		// �����ݽڵ���ӵ����ݴ洢��
		GetDataStorage()->Add(centerPointNode);

		// ����ģ�͵Ļ����������ɹ�͸���ȡ��ڵ��С��
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

// ����ģ�Ͳ���
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


// �˶����--̽������
bool HTOTookits::OnMoveTestClicked()
{
	// Action
	m_Controls.textBrowser_Action->append("Action: Move Test.");

	try {
		// ɾ��֮ǰ�Ĳ��Խڵ�
		auto previousTestNode = GetDataStorage()->GetNamedNode("MoveTestNode");
		if (previousTestNode) {
			GetDataStorage()->Remove(previousTestNode);
		}

		// ����һ���µĵ㼯�����ó�ʼλ��
		mitk::PointSet::Pointer testPointSet = mitk::PointSet::New();
		mitk::Point3D initialPoint;
		initialPoint[0] = 0.0;
		initialPoint[1] = 0.0;
		initialPoint[2] = 0.0;
		testPointSet->InsertPoint(0, initialPoint);

		// ����һ���µ����ݽڵ㲢���õ㼯
		mitk::DataNode::Pointer testPointNode = mitk::DataNode::New();
		testPointNode->SetData(testPointSet);
		testPointNode->SetName("MoveTestNode");

		// ���ýڵ���ɫΪ��ɫ
		testPointNode->SetColor(1.0, 0.0, 0.0); // RGB: Red
		testPointNode->SetFloatProperty("pointsize", 10);

		// �����ݽڵ���ӵ����ݴ洢��
		GetDataStorage()->Add(testPointNode);

		// ������ʱ�����ƶ��ڵ�
		QTimer* timer = new QTimer(this);
		connect(timer, &QTimer::timeout, this, [this, testPointSet]() {
			static double t = 0.0;
			t += 0.1;

			// ���½ڵ�λ��
			mitk::Point3D newPoint;
			newPoint[0] = 10.0 * std::cos(t); // X ������ƶ�·��
			newPoint[1] = 10.0 * std::sin(t); // Y ������ƶ�·��
			newPoint[2] = 0.0; // Z ���򱣳ֲ���
			testPointSet->SetPoint(0, newPoint);

			// ˢ����ͼ
			mitk::RenderingManager::GetInstance()->RequestUpdateAll();
			});
		timer->start(100); // ÿ100�������һ��

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
		// ��ȡ�ɹǺ��ֹǵ�ģ�ͽڵ�
		auto femurNode = GetDataStorage()->GetNamedNode("femurSurface");
		auto tibiaNode = GetDataStorage()->GetNamedNode("tibiaSurface");
		if (!femurNode || !tibiaNode) {
			throw std::runtime_error("Error: Could not find femur or tibia node.");
		}

		// ȷ���ֹǺ͹ɹǵ����ӵ�
        // �������ж�ȡ���ĵ��λ��
        auto tibiaCenterNode = GetDataStorage()->GetNamedNode("tibiaRotateCenter");
        if (!tibiaCenterNode) {
            throw std::runtime_error("Error: Could not find tibia rotate center node.");
        }
        auto tibiaCenterPointSet = dynamic_cast<mitk::PointSet*>(tibiaCenterNode->GetData());
        if (!tibiaCenterPointSet || tibiaCenterPointSet->GetSize() == 0) {
            throw std::runtime_error("Error: Could not get point set data from node or point set is empty.");
        }
        mitk::Point3D centerPoint = tibiaCenterPointSet->GetPoint(0);
		// ��ֵ
        double connectionPoint[3] = { centerPoint[0], centerPoint[1], centerPoint[2] }; // X, Y, Z ��������ӵ�����


		// ������ʱ������ת�ֹ�
		QTimer* timer = new QTimer(this);
		connect(timer, &QTimer::timeout, this, [this, tibiaNode, connectionPoint]() {
			static double t = 0.0;
			t += 0.1;

			// ������ת��ͽǶ�
			double rotationAxis[3] = { 0.0, 0.0, 1.0 }; // Z ����ת
			double rotationDegree = 5.0 * std::sin(t); // ��ת�Ƕ�

			// ʹ�� Rotate ������ת�ֹ�
			Rotate(connectionPoint, rotationAxis, rotationDegree, tibiaNode->GetData());
			});
		timer->start(100); // ÿ100�������һ��

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