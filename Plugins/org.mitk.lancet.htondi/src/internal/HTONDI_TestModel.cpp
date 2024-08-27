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
	// ����Ӱ�񵼺�����
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

bool HTONDI::OnReuseMatrixClicked()
{
	// Reuse the Registration Matrix last
	m_Controls.textBrowser_Action->append("Action: Apply Last Registration Matrix.");

	// ����Ƿ���֮ǰ����׼����
	if (m_PreviousImageRegistrationMatrix.IsNull())
	{
		m_Controls.textBrowser_Action->append("No previous registration available");
		return false;
	}

	// ����һ���µľ�̬ͼ�������׼������ʵ��
	m_surfaceRegistrationStaticImageFilter = lancet::ApplySurfaceRegistratioinStaticImageFilter::New();

	// �����������ӵ�Vega��������Դ
	m_surfaceRegistrationStaticImageFilter->ConnectTo(m_VegaSource);

	// ��֮ǰ�������׼�������õ���������
	m_surfaceRegistrationStaticImageFilter->SetRegistrationMatrix(m_PreviousImageRegistrationMatrix);

	// ���òο���ܵĵ�������
	m_surfaceRegistrationStaticImageFilter->SetNavigationDataOfRF(m_VegaSource->GetOutput("BoneRF"));

	// ֹͣVega���ӻ���ʱ��
	m_VegaVisualizeTimer->stop();

	// ��Vega���ӻ������ӵ���̬ͼ�������׼������
	m_VegaVisualizer->ConnectTo(m_surfaceRegistrationStaticImageFilter);

	// ��������Vega���ӻ���ʱ��
	m_VegaVisualizeTimer->start();

	return true;
}

bool HTONDI::OnMoveTestClicked()
{
	// �˶�����--̽������
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

bool HTONDI::OnMoveBoneClicked()
{
	// �˶�����-�ֹǹǱ�����ת
	m_Controls.textBrowser_Action->append("Action: Rotate Tibia.");

	try {
		// ��ȡ�ɹǺ��ֹǵ�ģ�ͽڵ�
		auto femurNode = GetDataStorage()->GetNamedNode("femurSurface");
		auto tibiaNode = GetDataStorage()->GetNamedNode("tibiaSurface");
		if (!femurNode || !tibiaNode) {
			throw std::runtime_error("Could not find femur or tibia node.");
		}

		// ȷ���ֹǺ͹ɹǵ����ӵ�
		// �������ж�ȡ���ĵ��λ��
		auto tibiaCenterNode = GetDataStorage()->GetNamedNode("tibiaRotateCenter");
		if (!tibiaCenterNode) {
			throw std::runtime_error("Could not find tibia rotate center node.");
		}
		auto tibiaCenterPointSet = dynamic_cast<mitk::PointSet*>(tibiaCenterNode->GetData());
		if (!tibiaCenterPointSet || tibiaCenterPointSet->GetSize() == 0) {
			throw std::runtime_error("Could not get point set data from node or point set is empty.");
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

std::pair<Eigen::Vector3d, double> HTONDI::FitSphere(const std::vector<Eigen::Vector3d>& points)
{
	// ��ȡ�ɹ�ͷ����
	// ����ѡ�� + �����������
	// ������壬�������ĺͰ뾶
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
	// ����ɹ�ͷ������
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
		auto previousCenterNode = GetDataStorage()->GetNamedNode("hipCenterPoint");
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
		
		// ͬʱ��landmark�в���ɹ�ͷ���ĵ�
		// ȡ���Ѿ��õ���Target��׼�㼯
		// auto pointSet_Landmark = femur_image->GetLandmarks();
		// pointSet_Landmark->InsertPoint(centerPoint);

		// ����һ���µ����ݽڵ㲢���õ㼯
		mitk::DataNode::Pointer centerPointNode = mitk::DataNode::New();
		centerPointNode->SetData(centerPointSet);
		centerPointNode->SetName("hipCenterPoint");

		// �����ݽڵ���ӵ����ݴ洢��
		GetDataStorage()->Add(centerPointNode);

		// ����ģ�͵Ļ����������ɹ�͸���ȡ��ڵ��С��
		auto femurSurface = GetDataStorage()->GetNamedNode("femurSurface");
		SetModelColor(centerPointNode, 1.0f, 0.0f, 0.0f);
		SetNodeSize(centerPointNode, 10.0f);
		SetModelOpacity(femurSurface, 0.5f);

		// ����õ��ɹ�ͷ���ĺ��������ʾ
		m_Controls.checkBox_point10->setChecked(true);

		// ��ʾlandmark�㼯�е�������
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
	// ����õ����������RF�µ�����
	mitk::NavigationData::Pointer res = mitk::NavigationData::New();
	res->Graft(nd);
	res->Compose(nd_ref->GetInverse());
	return res;
}

bool HTONDI::OnGetLandMarkPointClicked()
{
	// ȡ�ô���׼��
	m_Controls.textBrowser_Action->append("Action: Collect LandMark Points.");
	if (navigatedImage == nullptr)
	{
		m_Controls.textBrowser_Action->append("Please setup the navigationObject first!");
		return false;
	}

	// ȡ�ô���׼�㼯 �� ��ǰ�Ѿ�ȡ�õĵ�
	int landmark_surfaceNum = navigatedImage->GetLandmarks()->GetSize();
	int landmark_rmNum = navigatedImage->GetLandmarks_probe()->GetSize();

	if (landmark_surfaceNum == landmark_rmNum)
	{
		m_Controls.textBrowser_Action->append("--- Enough landmarks have been collected ----");
		return true;
	}

	// ȡ���Ѿ��õ���Target��׼�㼯
	auto pointSet_probeLandmark = navigatedImage->GetLandmarks_probe();
	auto pointSet_landmark = navigatedImage->GetLandmarks();
	
	
	
	// ��һ�ε����ʱ��ֻ������
	mitk::PointSet::Pointer collectedPoints = mitk::PointSet::New();
	if (firstnode)
	{	
		m_Controls.textBrowser_Action->append("Please collect node pos.");
		firstnode = false;
		// �Խ�������Ҫ�ɼ��Ľڵ�ȾɫΪ��ɫ�����Ѿ��ɼ��õĽڵ�ȾɫΪ��ɫ
		// ���ȣ�ȡ����ǰ��Ҫ�ɼ���Ϣ�Ľڵ�
		mitk::PointSet::Pointer current = mitk::PointSet::New();
		currentpoint = pointSet_landmark->GetPoint(landmark_rmNum);
		current->InsertPoint(0, currentpoint);

		// Ȼ�󣬽��������ӻ���������ɫΪ��ɫ
		// ����һ���µ����ݽڵ㲢���õ㼯
		mitk::DataNode::Pointer currentPointNode = mitk::DataNode::New();
		currentPointNode->SetData(current);
		currentPointNode->SetName("CurrentLandmarkPoint");

		// ���ýڵ���ɫΪ��ɫ
		currentPointNode->SetColor(1.0, 0.0, 0.0); // RGB: Red
		currentPointNode->SetFloatProperty("pointsize", 3);

		// �����ݽڵ���ӵ����ݴ洢��
		GetDataStorage()->Add(currentPointNode);

		// ��ʼ��һ���ն���
		collectedPointsNode->SetData(collectedPoints);
		collectedPointsNode->SetName("CollectedLandmarkPoints");
		collectedPointsNode->SetColor(0.0, 1.0, 0.0); // �Ѳɼ��õĽڵ�ȾɫΪ��ɫ
		collectedPointsNode->SetFloatProperty("pointsize", 3.0f);
		GetDataStorage()->Add(collectedPointsNode);

		return true;
	}

	// �ڶ��ε����ʼ�ɼ���Ϣ
	// ȡ��̽��͹����ο������ڹ����б���±�
	auto probeIndex = m_VegaToolStorage->GetToolIndexByName("ProbeRF");
	auto objectRfIndex = m_VegaToolStorage->GetToolIndexByName("BoneRF");
	if (probeIndex == -1 || objectRfIndex == -1)
	{
		m_Controls.textBrowser_Action->append("There is no 'Probe' or 'ObjectRf' in the toolStorage!");
	}


	// ��ȡ̽���� NDI ����ϵͳ����ϵ�е�λ�úͷ�������
	mitk::NavigationData::Pointer nd_ndiToProbe = m_VegaSource->GetOutput(probeIndex);

	// ��ȡ�ο����壨�����ǹ����������̶��ο����NDI����ϵ�е�λ�úͷ�������
	mitk::NavigationData::Pointer nd_ndiToObjectRf = m_VegaSource->GetOutput(objectRfIndex);

	// ��̽���λ�ô�NDI����ϵת�����ο����������ϵ��
	mitk::NavigationData::Pointer nd_rfToProbe = GetNavigationDataInRef(nd_ndiToProbe, nd_ndiToObjectRf);

	// ��ת����ĵ�����������ȡ̽���˵�λ��, ���λ������������ڲο���������ϵ��
	mitk::Point3D probeTipPointUnderRf = nd_rfToProbe->GetPosition();

	// ������õ���̽����λ����ӵ� �㼯 ��
	pointSet_probeLandmark->InsertPoint(probeTipPointUnderRf);
	
	// ���ռ�����current�㣬���뵽�Ѿ��ɼ��ĵ������
	collectedPoints->InsertPoint(landmark_rmNum, currentpoint);

	// ���ȣ�ȡ����ǰ��Ҫ�ɼ���Ϣ�Ľڵ�
	mitk::PointSet::Pointer current = mitk::PointSet::New();
	currentpoint = pointSet_landmark->GetPoint(landmark_rmNum);
	current->InsertPoint(0, currentpoint);

	// Ȼ�󣬽��������ӻ���������ɫΪ��ɫ
	// ����ɾ���Ѿ������Ѿ��е�CurrentLandmarkPoint����
	auto existingCurrentPointNode = GetDataStorage()->GetNamedNode("CurrentLandmarkPoint");
	if (existingCurrentPointNode)
	{
		GetDataStorage()->Remove(existingCurrentPointNode);
	}

	// ����һ���µ����ݽڵ㲢���õ㼯
	mitk::DataNode::Pointer currentPointNode = mitk::DataNode::New();
	currentPointNode->SetData(current);
	currentPointNode->SetName("CurrentLandmarkPoint");

	// ���ýڵ���ɫΪ��ɫ
	currentPointNode->SetColor(1.0, 0.0, 0.0); // RGB: Red
	currentPointNode->SetFloatProperty("pointsize", 3);

	// �����ݽڵ���ӵ����ݴ洢��
	GetDataStorage()->Add(currentPointNode);

	m_Controls.textBrowser_Action->append("Added landmark: " + QString::number(probeTipPointUnderRf[0]) +
		"/ " + QString::number(probeTipPointUnderRf[1]) + "/ " + QString::number(probeTipPointUnderRf[2]));

	return true;
}

bool HTONDI::OnGetICPPointClicked()
{
	// ICP Image Registration
	// ͬ LandMark ����
	m_Controls.textBrowser_Action->append("Action: Collect ICP Points.");
	
	// ���navigatedImage�Ƿ��Ѿ���ʼ��
	if (navigatedImage == nullptr)
	{
		m_Controls.textBrowser_Action->append("Please setup the navigationObject first!");
		return false;
	}

	// ��ȡICP(���������)�㼯
	auto pointSet_probeIcp = navigatedImage->GetIcpPoints_probe();

	// ��ȡ̽��Ͳο�����(����)������
	auto probeIndex = m_VegaToolStorage->GetToolIndexByName("ProbeRF");
	auto objectRfIndex = m_VegaToolStorage->GetToolIndexByName("BoneRF");

	// ��鹤���Ƿ����
	if (probeIndex == -1 || objectRfIndex == -1)
	{
		m_Controls.textBrowser_Action->append("There is no 'Probe' or 'ObjectRf' in the toolStorage!");
	}

	// ��ȡ̽��Ͳο�������NDI����ϵ�еĵ�������
	mitk::NavigationData::Pointer nd_ndiToProbe = m_VegaSource->GetOutput(probeIndex);
	mitk::NavigationData::Pointer nd_ndiToObjectRf = m_VegaSource->GetOutput(objectRfIndex);

	// ��̽��ĵ�������ת�����ο����������ϵ��
	mitk::NavigationData::Pointer nd_rfToProbe = GetNavigationDataInRef(nd_ndiToProbe, nd_ndiToObjectRf);

	// ��ȡ̽�����ڲο���������ϵ�е�λ��
	mitk::Point3D probeTipPointUnderRf = nd_rfToProbe->GetPosition();

	// ���������ӵ�ICP�㼯��
	pointSet_probeIcp->InsertPoint(probeTipPointUnderRf);

	// ���û���������ʾ��ӵĵ������
	m_Controls.textBrowser_Action->append("Added icp point: " + QString::number(probeTipPointUnderRf[0]) +
		"/ " + QString::number(probeTipPointUnderRf[1]) + "/ " + QString::number(probeTipPointUnderRf[2]));

	return true;
}

bool HTONDI::OnShowCollectionClicked()
{
	// Show the collected Nodes
	m_Controls.textBrowser_Action->append("Action: Show Collected Points.");

	// ��ȡ�ռ����Ľڵ���Ϣ
	auto CollectedLandmarkNodes = navigatedImage->GetLandmarks_probe();
	auto CollectedIcpNodes = navigatedImage->GetIcpPoints_probe();
	QString LandmarkNodes = "Landmark Points:\n";
	QString ICPNodes = "ICP Points:\n";;

	// ��ʾlandmark��
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
	// ��ʾicp��
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
	// ����ͼ����׼�������Ӧ�õ���ǰӰ��

	// ����һ���µľ�̬ͼ�������׼������ʵ��
	m_surfaceRegistrationStaticImageFilter = lancet::ApplySurfaceRegistratioinStaticImageFilter::New();
	// �����������ӵ�Vega��������Դ
	m_surfaceRegistrationStaticImageFilter->ConnectTo(m_VegaSource);
	// ����һ���µķ���任�������ڴ洢��׼���
	m_imageRegistrationMatrix = mitk::AffineTransform3D::New();
	// ���µ���ͼ��Ķ��󵽲ο���ܵı任����
	navigatedImage->UpdateObjectToRfMatrix();

	// ��ӡ��ǰ��׼����Ľ��
	// lanmark
	m_Controls.textBrowser_Action->append("Avg landmark error:" + QString::number(navigatedImage->GetlandmarkRegis_avgError()));
	m_Controls.textBrowser_Action->append("Max landmark error:" + QString::number(navigatedImage->GetlandmarkRegis_maxError()));
	// icp
	m_Controls.textBrowser_Action->append("Avg ICP error:" + QString::number(navigatedImage->GetIcpRegis_avgError()));
	m_Controls.textBrowser_Action->append("Max ICP error:" + QString::number(navigatedImage->GetIcpRegis_maxError()));

	// ������ͼ��Ķ��󵽲ο���ܵı任�����VTK��ʽת��ΪITK��ʽ�����洢
	mitk::TransferVtkMatrixToItkTransform(navigatedImage->GetT_Object2ReferenceFrame(), m_imageRegistrationMatrix.GetPointer());
	// ����׼����Ӧ�õ� BoneRF ������
	m_VegaToolStorage->GetToolByName("BoneRF")->SetToolRegistrationMatrix(m_imageRegistrationMatrix);
	// ����׼�������õ���̬ͼ�������׼��������
	m_surfaceRegistrationStaticImageFilter->SetRegistrationMatrix(m_VegaToolStorage->GetToolByName("BoneRF")->GetToolRegistrationMatrix());
	// ���òο���ܵĵ�������
	m_surfaceRegistrationStaticImageFilter->SetNavigationDataOfRF(m_VegaSource->GetOutput("BoneRF"));
	// ֹͣVega���ӻ���ʱ��
	m_VegaVisualizeTimer->stop();
	// ��Vega���ӻ������ӵ���̬ͼ�������׼������
	m_VegaVisualizer->ConnectTo(m_surfaceRegistrationStaticImageFilter);
	// ��������Vega���ӻ���ʱ������ʾ��׼����Ľ��
	m_VegaVisualizeTimer->start();

	// ������׼���� m_PreviousImageRegistrationMatrix
	m_PreviousImageRegistrationMatrix = m_imageRegistrationMatrix;

	/*
	��δ������Ҫ�߼���ִ�о�̬ͼ��ı�����׼����ͨ��������������׼�����������µ���ͼ��ı任����
	�������ʾ��׼��������׼���Ӧ�õ� BoneRF ���ߺ͹������С�
	�������������Vega���ӻ���ʱ�����Ա����û���������ʾ��׼�����
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
	
	// ��ȡBoneRF���ߵ���׼����
	auto imageRfRegistrationMatrix = m_VegaToolStorage->GetToolByName("BoneRF")->GetToolRegistrationMatrix();

	// ����һ���µ�VTK 4x4�������ڴ洢��׼����
	vtkNew<vtkMatrix4x4> vtkImageRfRegistrationMatrix; // image to RF matrix

	// ��ITK��ʽ����׼����ת��ΪVTK��ʽ
	mitk::TransferItkTransformToVtkMatrix(imageRfRegistrationMatrix.GetPointer(), vtkImageRfRegistrationMatrix);

	// �����׼�����Ƿ�Ϊ��λ���󣨼�û�н�����׼��
	if (vtkImageRfRegistrationMatrix->IsIdentity())
	{
		m_Controls.textBrowser_Action->append("No image registration available");
		return false;
	}

	// ��ȡѡ�еĵ㼯
	auto tmpPointSet = dynamic_cast<mitk::PointSet*>(m_Controls.selectCheckNode_mitkSlecectButton->GetSelectedNode()->GetData());
	auto imageCheckPoint = tmpPointSet->GetPoint(0);

	// ��ȡ̽����NDI����ϵ�еĵ�������
	mitk::NavigationData::Pointer nd_ndiToProbe = m_VegaSource->GetOutput("ProbeRF");

	// ��ȡ�ο����壨BoneRF����NDI����ϵ�еĵ�������
	mitk::NavigationData::Pointer nd_ndiToObjectRf = m_VegaSource->GetOutput("BoneRF");

	// ��̽��ĵ�������ת�����ο����������ϵ��
	mitk::NavigationData::Pointer nd_rfToProbe = GetNavigationDataInRef(nd_ndiToProbe, nd_ndiToObjectRf);

	// ��ȡ̽�����ڲο���������ϵ�е�λ��
	mitk::Point3D probeTipPointUnderRf = nd_rfToProbe->GetPosition();

	// ����һ���µ�VTK 4x4�������ڴ洢̽���˵�λ��
	vtkNew<vtkMatrix4x4> tmpMatrix;
	tmpMatrix->Identity();
	tmpMatrix->SetElement(0, 3, probeTipPointUnderRf[0]);
	tmpMatrix->SetElement(1, 3, probeTipPointUnderRf[1]);
	tmpMatrix->SetElement(2, 3, probeTipPointUnderRf[2]);

	// ����һ���µ�VTK�任���󣬲����������
	vtkNew<vtkTransform> tmpTransform;
	tmpTransform->SetMatrix(tmpMatrix);
	tmpTransform->PostMultiply();
	tmpTransform->Concatenate(vtkImageRfRegistrationMatrix);
	tmpTransform->Update();

	// ��ȡ�任��ľ���
	auto transMatrix = tmpTransform->GetMatrix();

	// ��ȡ̽������Ӱ������ϵ�е�λ��
	double probeTipInImage[3]
	{
		transMatrix->GetElement(0,3),
		transMatrix->GetElement(1,3),
		transMatrix->GetElement(2,3),
	};

	// ����̽������Ӱ�����֮��ľ���
	double distance = sqrt(pow(probeTipInImage[0] - imageCheckPoint[0], 2) +
		pow(probeTipInImage[1] - imageCheckPoint[1], 2) +
		pow(probeTipInImage[2] - imageCheckPoint[2], 2));

	// ���û���������ʾ̽������Ӱ�����֮��ľ���
	m_Controls.textBrowser_Action->append("Distance to the checkpoint:" + QString::number(distance));

	return true;
}

bool HTONDI::OnCheckNode2SurfaceClicked()
{
	// Point to surface Loss
	m_Controls.textBrowser_Action->append("Action: Check Accuracy Node2Surface.");

	// ��ȡBoneRF���ߵ���׼����
	auto imageRfRegistrationMatrix = m_VegaToolStorage->GetToolByName("BoneRF")->GetToolRegistrationMatrix();

	// ����һ���µ�VTK 4x4�������ڴ洢��׼����
	vtkNew<vtkMatrix4x4> vtkImageRfRegistrationMatrix; // image to RF matrix

	// ��ITK��ʽ����׼����ת��ΪVTK��ʽ
	mitk::TransferItkTransformToVtkMatrix(imageRfRegistrationMatrix.GetPointer(), vtkImageRfRegistrationMatrix);

	// �����׼�����Ƿ�Ϊ��λ���󣨼�û�н�����׼��
	if (vtkImageRfRegistrationMatrix->IsIdentity())
	{
		m_Controls.textBrowser_Action->append("No image registration available");
		return false;
	}

	// ��ȡ̽����NDI����ϵ�еĵ�������
	mitk::NavigationData::Pointer nd_ndiToProbe = m_VegaSource->GetOutput("ProbeRF");

	// ��ȡ�ο����壨BoneRF����NDI����ϵ�еĵ�������
	mitk::NavigationData::Pointer nd_ndiToObjectRf = m_VegaSource->GetOutput("BoneRF");

	// ��̽��ĵ�������ת�����ο����������ϵ��
	mitk::NavigationData::Pointer nd_rfToProbe = GetNavigationDataInRef(nd_ndiToProbe, nd_ndiToObjectRf);

	// ��ȡ̽�����ڲο���������ϵ�е�λ��
	mitk::Point3D probeTipPointUnderRf = nd_rfToProbe->GetPosition();

	// ����һ���µ�VTK 4x4�������ڴ洢̽���˵�λ��
	vtkNew<vtkMatrix4x4> tmpMatrix;
	tmpMatrix->Identity();
	tmpMatrix->SetElement(0, 3, probeTipPointUnderRf[0]);
	tmpMatrix->SetElement(1, 3, probeTipPointUnderRf[1]);
	tmpMatrix->SetElement(2, 3, probeTipPointUnderRf[2]);

	// ����һ���µ�VTK�任���󣬲����������
	vtkNew<vtkTransform> tmpTransform;
	tmpTransform->SetMatrix(tmpMatrix);
	tmpTransform->PostMultiply();
	tmpTransform->Concatenate(vtkImageRfRegistrationMatrix);
	tmpTransform->Update();

	// ��ȡ�任��ľ���
	auto transMatrix = tmpTransform->GetMatrix();

	// ��ȡ̽������Ӱ������ϵ�е�λ��
	double probeTipInImage[3]
	{
		transMatrix->GetElement(0,3),
		transMatrix->GetElement(1,3),
		transMatrix->GetElement(2,3),
	};

	// ��ȡ�������ݣ�Ҳ����ԭ�ȼ��ص�ͼ�����
	auto surfacePolyData = dynamic_cast<mitk::Surface*>(m_Controls.setSTL_mitkSlecectButton->GetSelectedNode()->GetData())->GetVtkPolyData();

	// ����һ����ʽ��������ݾ���������ڼ���㵽����ľ���
	vtkNew<vtkImplicitPolyDataDistance> implicitPolyDataDistance;
	implicitPolyDataDistance->SetInput(surfacePolyData);

	// ����̽���˵�����ľ���
	double currentError = implicitPolyDataDistance->EvaluateFunction(probeTipInImage);

	// ���û���������ʾ̽���˵�����ľ���
	m_Controls.textBrowser_Action->append("Distance to the surface: " + QString::number(currentError));

	return true;
}





bool HTONDI::OnLinkSurfaceClicked()
{
	// ��ȡ��ǰ�ھ�RF��λ��
	auto sawRF = m_VegaToolStorage->GetToolByName("SawRF");

	// ��ȡ�ھ�ı�������
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
	// 4. ���������
	sawRF->SetDataNode(sawRF_Surface);
	sawRF_Surface->SetVisibility(true);


	return true;
}

bool HTONDI::OnCollectPointerPosClicked()
{
	// Collect Pointer Pos
	m_Controls.textBrowser_Action->append("Action: Collect Pointer Pos.");
	// ��ʼ�ɼ�������һ��ʱ���ڵ�̽��λ����Ϣ
	m_ProbePositions.clear();
	m_Transformations.clear();
	m_Controls.textBrowser_Action->append("Start collect probe data, please move slowly...");

	// ������ʱ�������ڲɼ�����
	if (!m_ProbeDataCollectionTimer)
	{
		m_ProbeDataCollectionTimer = new QTimer(this);
		connect(m_ProbeDataCollectionTimer, &QTimer::timeout, this, &HTONDI::CollectProbeData);
	}
	m_ProbeDataCollectionTimer->start(100); // ÿ100ms�ɼ�һ������

	return true;
}


bool HTONDI::OnVisualizeCollectedPoints()
{
    // Action
    m_Controls.textBrowser_Action->append("Action: Visualize Collected Points.");

    try {
        // ɾ��֮ǰ�Ĳ��Խڵ�
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

        // ����һ���µĵ㼯�����òɼ����ĵ�
        mitk::PointSet::Pointer pointSet = mitk::PointSet::New();
        for (size_t i = 0; i < m_ProbePositions.size(); ++i) {
            mitk::Point3D point;
            point[0] = m_ProbePositions[i][0] - m_Center[0];
            point[1] = m_ProbePositions[i][1] - m_Center[1];
            point[2] = m_ProbePositions[i][2] - m_Center[2];
            pointSet->InsertPoint(i, point);
        }

        // ������ĵ�
        mitk::Point3D centerPoint;
        centerPoint[0] = 0.0;
        centerPoint[1] = 0.0;
        centerPoint[2] = 0.0;
        pointSet->InsertPoint(m_ProbePositions.size(), centerPoint);

        // ����һ���µ����ݽڵ㲢���õ㼯
        mitk::DataNode::Pointer pointNode = mitk::DataNode::New();
        pointNode->SetData(pointSet);
        pointNode->SetName("CollectedPointsNode");
        // ���ýڵ���ɫΪ��ɫ
        pointNode->SetColor(1.0, 0.0, 0.0); // RGB: Red
        pointNode->SetFloatProperty("pointsize", 10);

        // ����һ���µ����ݽڵ���������
        mitk::DataNode::Pointer centerNode = mitk::DataNode::New();
        mitk::PointSet::Pointer centerPointSet = mitk::PointSet::New();
        centerPointSet->InsertPoint(0, centerPoint);
        centerNode->SetData(centerPointSet);
        centerNode->SetName("CenterPointNode");
        centerNode->SetColor(0.0, 1.0, 0.0); // RGB: Green
        centerNode->SetFloatProperty("pointsize", 10);

        // �����ݽڵ���ӵ����ݴ洢��
        GetDataStorage()->Add(pointNode);
        GetDataStorage()->Add(centerNode);

        // ����̽�������˫Ŀ�������ϵ�µ�λ��
        std::vector<Eigen::Vector3d> probeTipsInCamera;
        for (const auto& T_cameraToProbe : m_Transformations)
        {
            // ��̽�����λ��ת��Ϊ�������
            Eigen::Vector4d s_homogeneous(m_ProbeTip[0], m_ProbeTip[1], m_ProbeTip[2], 1.0);

            // ����̽�������˫Ŀ�������ϵ�µ�λ��
            Eigen::Vector4d p_homogeneous = T_cameraToProbe * s_homogeneous;

            // ��ȡǰ3��Ԫ�أ��õ�̽�������˫Ŀ�������ϵ�µ�3D����
            Eigen::Vector3d p = p_homogeneous.head<3>();

            // �洢���
            probeTipsInCamera.push_back(p);
        }

        // ����һ���µĵ㼯������̽�������˫Ŀ�������ϵ�µ�λ��
        mitk::PointSet::Pointer probeTipPointSet = mitk::PointSet::New();
        for (size_t i = 0; i < probeTipsInCamera.size(); ++i) {
            mitk::Point3D point;
            point[0] = probeTipsInCamera[i][0];
            point[1] = probeTipsInCamera[i][1];
            point[2] = probeTipsInCamera[i][2];
            probeTipPointSet->InsertPoint(i, point);
        }

        // ����һ���µ����ݽڵ㲢���õ㼯
        mitk::DataNode::Pointer probeTipPointNode = mitk::DataNode::New();
        probeTipPointNode->SetData(probeTipPointSet);
        probeTipPointNode->SetName("ProbeTipsNode");
        // ���ýڵ���ɫΪ��ɫ
        probeTipPointNode->SetColor(0.0, 0.0, 1.0); // RGB: Blue
        probeTipPointNode->SetFloatProperty("pointsize", 10);

        // �����ݽڵ���ӵ����ݴ洢��
        GetDataStorage()->Add(probeTipPointNode);

        // ˢ����ͼ
        mitk::RenderingManager::GetInstance()->RequestUpdateAll();
    }
    catch (const std::exception & e) {
        m_Controls.textBrowser_Action->append(QString("Error: %1").arg(e.what()));
    }
    return true;
}