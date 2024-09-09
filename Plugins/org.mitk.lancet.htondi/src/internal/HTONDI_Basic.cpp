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

//// ���ӵ���Ӧ��ͷ�ļ�
//#include "HTONDI_Basic.h"
//
//// ʹ��ͳһ�������ռ�
//using namespace lancetHTO;


/*===============================================================
HTONDI_Basic.cpp
----------------------------------------------------------------
== NDI connect and real time Toolkits visualization
== Probe Tip Registration
===============================================================*/
// ץ������ֻ�ڸ�Ƶ������





bool HTONDI::OnConnectNDIClicked()
{	
	/* NDI�������
		1. �豸����
		2. ����ʵʱ�豸׷��״̬
	*/

	m_Controls.textBrowser_Action->append("Action: NDI Connection.");

	// 1. �豸����

	// ��ȡ���������ļ�
	QString filename = QFileDialog::getOpenFileName(nullptr, tr("Open Tool Storage"), "/",
		tr("Tool Storage Files (*.IGTToolStorage)"));
	if (filename.isNull()) return false;

	// �����л����ߴ洢�� ���ع��ߵ�������Ϣ
	std::string errorMessage = "";
	mitk::NavigationToolStorageDeserializer::Pointer myDeserializer = mitk::NavigationToolStorageDeserializer::New(
		GetDataStorage());
	m_VegaToolStorage = myDeserializer->Deserialize(filename.toStdString());
	m_VegaToolStorage->SetName(filename.toStdString());

	// ����NDI Vega�����豸
	MITK_INFO << "Vega tracking";
	lancet::NDIVegaTrackingDevice::Pointer vegaTrackingDevice = lancet::NDIVegaTrackingDevice::New();

	// ʹ�ù����ഴ�������豸Դ
	lancet::TrackingDeviceSourceConfiguratorLancet::Pointer vegaSourceFactory =
		lancet::TrackingDeviceSourceConfiguratorLancet::New(m_VegaToolStorage, vegaTrackingDevice);
	

	// 2. ����ʵʱ�豸׷��״̬

	// ���ÿ��ӻ���
	m_VegaSource = vegaSourceFactory->CreateTrackingDeviceSource(m_VegaVisualizer);
	// ���ù���Ԫ���ݼ���
	m_VegaSource->SetToolMetaDataCollection(m_VegaToolStorage);

	// ���Ӳ���ʼ����
	m_VegaSource->Connect();
	m_VegaSource->StartTracking();

	// ���ö�ʱ�������¿��ӻ�
	if (m_VegaVisualizeTimer == nullptr)
	{
		m_VegaVisualizeTimer = new QTimer(this);
	}
	connect(m_VegaVisualizeTimer, SIGNAL(timeout()), this, SLOT(OnVegaVisualizeTimer()));
	connect(m_VegaVisualizeTimer, SIGNAL(timeout()), this, SLOT(UpdateToolStatusWidget()));
	
	// ���ӻ�
	ShowToolStatus_Vega();

	// ������ʱ��
	m_VegaVisualizeTimer->start(100);

	// ������ͼ
	auto geo = this->GetDataStorage()->ComputeBoundingGeometry3D(this->GetDataStorage()->GetAll());
	mitk::RenderingManager::GetInstance()->InitializeViews(geo);

	m_Controls.textBrowser_Action->append("NDI Connected !!!");

	return true;
}

bool HTONDI::OnVegaVisualizeTimer()
{	
	/* ���������豸�Ŀ��ӻ� */
	
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
	/* ֻ����Vega��NDI�����ߵ�״̬��ʾ */
	

	m_Controls.showVegaTool_StatusWidget->Refresh();
	m_Controls.showVegaTool_StatusWidget02->Refresh();
	// ���VegaԴδ���ӣ�����ֱ�ӷ���
	if (m_VegaSource == nullptr)
	{
		m_Controls.textBrowser_Action->append("ERROR: NDI Connection.");
		return false;

	}
	return true;
}

bool HTONDI::ShowToolStatus_Vega()
{
	/* չʾNDI Vega�豸��״̬ */

	m_VegaNavigationData.clear();
	for (std::size_t i = 0; i < m_VegaSource->GetNumberOfOutputs(); i++)
	{
		m_VegaNavigationData.push_back(m_VegaSource->GetOutput(i));
	}
	// Demo���·�״̬��
	m_Controls.showVegaTool_StatusWidget->RemoveStatusLabels();
	m_Controls.showVegaTool_StatusWidget->SetShowPositions(true);
	m_Controls.showVegaTool_StatusWidget->SetTextAlignment(Qt::AlignLeft);
	m_Controls.showVegaTool_StatusWidget->SetNavigationDatas(&m_VegaNavigationData);
	m_Controls.showVegaTool_StatusWidget->ShowStatusLabels();

	// �豸���ӽ���״̬��
	m_Controls.showVegaTool_StatusWidget02->RemoveStatusLabels();
	m_Controls.showVegaTool_StatusWidget02->SetShowPositions(true);
	m_Controls.showVegaTool_StatusWidget02->SetTextAlignment(Qt::AlignLeft);
	m_Controls.showVegaTool_StatusWidget02->SetNavigationDatas(&m_VegaNavigationData);
	m_Controls.showVegaTool_StatusWidget02->ShowStatusLabels();
	return true;
}

// ���ƽ��
std::pair<Eigen::Vector3d, Eigen::Vector3d> HTONDI::FitPlane3D(const std::vector<Eigen::Vector3d>& points)
{
	// ��ȡ�㼯�Ĵ�С
	int n = points.size();

	// ���ڷ��� AX = b
	// ���� 
	// n x 3 �ľ��� A 
	// n x 1 ������ b
	Eigen::MatrixXd A(n, 3);
	Eigen::VectorXd b(n);

	// ������ A ������ b
	// ����ƽ�淽��Ϊ ax + by + cz + d = 0�����ｫ��������Ϊ 1
	for (int i = 0; i < n; ++i) {
		const Eigen::Vector3d& p = points[i];
		A(i, 0) = p.x(); // ����� x ���������� A �ĵ�һ��
		A(i, 1) = p.y(); // ����� y ���������� A �ĵڶ���
		A(i, 2) = p.z(); // ����� z ���������� A �ĵ�����
		b(i) = 1;        // ���ó��� 1
	}

	// ʹ����С���˷�������Է��� Ax = b������ x ��ƽ�淽�̵�ϵ��
	Eigen::Vector3d x = A.colPivHouseholderQr().solve(b);

	// ����ƽ��ķ�����(a, b, c)
	Eigen::Vector3d normal(x[0], x[1], x[2]);

	// ��һ��������
	double norm = normal.norm();
	if (norm > 0) {
		normal /= norm;
	}

	// �������ƽ���ϵ�һ�����ƽ��ķ�����
	return { x, normal };
}

// ʹ����С���˷������ά�ռ��е������㵽һ��Բ
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
//	/* ����ƽ��ķ�����
//
//	��ȡƽ���ϵ������㣬�����������
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
//	// ���ӻ�
//	mitk::PointSet::Pointer tmpMitkPointSet = mitk::PointSet::New();
//	tmpMitkPointSet->InsertPoint(0, p0);
//	tmpMitkPointSet->InsertPoint(1, p1);
//	tmpMitkPointSet->InsertPoint(2, p2);
//
//	// ���򴴽��µ�ƽ��
//	auto tmpNodes = GetDataStorage()->GetNamedNode("TmpPoint");
//	if (tmpNodes) {
//		GetDataStorage()->Remove(tmpNodes);
//		//// ����Ѿ��������������Ϣ
//		//tmpNodes->SetData(mitkPointSetRealTime);
//	}
//	mitk::DataNode::Pointer pointSetInPlaneCutPlane = mitk::DataNode::New();
//	pointSetInPlaneCutPlane->SetName("TmpPoint");
//	// ��ɫ����С 5.0
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
//	// ����������һ��Ϊ��λ����
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
//	/* ����ƽ��ķ�����
//		��ȡƽ���ϵ������㣬���㷨����
//	*/
//	auto PlaneSource = dynamic_cast<mitk::Surface*>(GetDataStorage()->GetNamedNode(planeName)->GetData());
//	mitk::Point3D p1, p2, p3;
//	Eigen::Vector3d normal;
//
//	if (PlaneSource)
//	{
//		// ��Ҫ��Ӧ��ת��������������ȡ����
//		auto surfacePoints = PlaneSource->GetVtkPolyData();
//		// ����ת����
//		vtkNew<vtkTransform> cutPlaneTransform;
//		cutPlaneTransform->SetMatrix(PlaneSource->GetGeometry()->GetVtkMatrix());
//		// Ӧ��ת����
//		vtkNew<vtkTransformFilter> cutPlaneTransformFilter;
//		cutPlaneTransformFilter->SetTransform(cutPlaneTransform);
//		cutPlaneTransformFilter->SetInputData(surfacePoints);
//		cutPlaneTransformFilter->Update();
//		// ���ݿ���
//		vtkNew<vtkPolyData> tmpVtkSurface;
//		tmpVtkSurface->DeepCopy(cutPlaneTransformFilter->GetPolyDataOutput());
//
//		// ���ݲɼ�
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
//			// ����������һ��Ϊ��λ����
//			normalVector.normalize();
//			normal = normalVector;
//
//
//			// ���ӻ�
//			mitk::PointSet::Pointer tmpMitkPointSet = mitk::PointSet::New();
//			tmpMitkPointSet->InsertPoint(0, p1);
//			tmpMitkPointSet->InsertPoint(1, p2);
//			tmpMitkPointSet->InsertPoint(2, p3);
//
//			// ���򴴽��µ�ƽ��
//			auto tmpNodes = GetDataStorage()->GetNamedNode("TmpPoint");
//			if (tmpNodes) {
//				GetDataStorage()->Remove(tmpNodes);
//				//// ����Ѿ��������������Ϣ
//				//tmpNodes->SetData(mitkPointSetRealTime);
//			}
//			mitk::DataNode::Pointer pointSetInPlaneCutPlane = mitk::DataNode::New();
//			pointSetInPlaneCutPlane->SetName("TmpPoint");
//			// ��ɫ����С 5.0
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
	/* ����ƽ��ķ�����

	��ȡƽ���ϵ������㣬�����������
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

	// ���ӻ�
	mitk::PointSet::Pointer tmpMitkPointSet = mitk::PointSet::New();
	tmpMitkPointSet->InsertPoint(0, p0);
	tmpMitkPointSet->InsertPoint(1, p1);
	tmpMitkPointSet->InsertPoint(2, p2);

	// ���򴴽��µ�ƽ��
	auto tmpNodes = GetDataStorage()->GetNamedNode("TmpPoint");
	if (tmpNodes) {
		GetDataStorage()->Remove(tmpNodes);
		//// ����Ѿ��������������Ϣ
		//tmpNodes->SetData(mitkPointSetRealTime);
	}
	mitk::DataNode::Pointer pointSetInPlaneCutPlane = mitk::DataNode::New();
	pointSetInPlaneCutPlane->SetName("TmpPoint");
	// ��ɫ����С 5.0
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
	// ����������һ��Ϊ��λ����
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
	/* ����ƽ��ķ�����
		��ȡƽ���ϵ������㣬���㷨����
	*/

	// ���ڵ�ǰˮƽ�ع���Ĺ滮�����������ع���
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
		// ������ƽ��
		vtkNew<vtkPolyData> tmpVtkSurface;
		tmpVtkSurface->DeepCopy(cutPlaneTransformFilter->GetPolyDataOutput());

		cout << "test 02-06" << endl;
		// ���ݲɼ�
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
			// ����������һ��Ϊ��λ����
			normalVector.normalize();
			normal = normalVector;

			/*
			// ���ӻ�
			mitk::PointSet::Pointer tmpMitkPointSet = mitk::PointSet::New();
			tmpMitkPointSet->InsertPoint(0, p1);
			tmpMitkPointSet->InsertPoint(1, p2);
			tmpMitkPointSet->InsertPoint(2, p3);

			// ���򴴽��µ�ƽ��
			auto tmpNodes = GetDataStorage()->GetNamedNode("TmpPoint");
			if (tmpNodes) {
				GetDataStorage()->Remove(tmpNodes);
				//// ����Ѿ��������������Ϣ
				//tmpNodes->SetData(mitkPointSetRealTime);
			}
			mitk::DataNode::Pointer pointSetInPlaneCutPlane = mitk::DataNode::New();
			pointSetInPlaneCutPlane->SetName("TmpPoint");
			// ��ɫ����С 5.0
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
	/* ����ˮƽ�عǼн���� 
		��ǰ�ع���͹滮ˮƽ�ع���ļн�
	*/
	
	Eigen::Vector3d normalPlane01, normalSaw;
	cout << "test 01-01" << endl;
	normalPlane01 = ExtractNormalFromPlane("1st cut plane");
	cout << "test 01-02" << endl;
	normalSaw = ExtractNormalFromPlane("StateCutPlane01");
	cout << "test 01-03" << endl;
	// ����������������֮��ļн�
	double dotProduct = normalPlane01[0] * normalSaw[0] + normalPlane01[1] * normalSaw[1] + normalPlane01[2] * normalSaw[2]; // ���

	// ������������
	double magnitude1 = sqrt(normalPlane01[0] * normalPlane01[0] + normalPlane01[1] * normalPlane01[1] + normalPlane01[2] * normalPlane01[2]);
	double magnitude2 = sqrt(normalSaw[0] * normalSaw[0] + normalSaw[1] * normalSaw[1] + normalSaw[2] * normalSaw[2]);

	// ����нǵ�����ֵ
	double cosAngle = dotProduct / (magnitude1 * magnitude2);

	// ʹ�÷����Һ�������нǣ����ȣ�
	double angleInRadians = acos(cosAngle);

	// ������ת��Ϊ����������1λС��
	double angleInDegrees = round(angleInRadians * (180.0 / M_PI) * 10) / 10;
	if (angleInDegrees > 90)
	{
		angleInDegrees = 180.0 - angleInDegrees;
	}
	cout << "test 01-04" << endl;
	// ����нǵ�ʵ�ʵ�λ��
	m_Controls.textBrowser_AxialCut->append("Real time Angle Miss: " + QString::number(angleInDegrees));
}

void HTONDI::CaculateCutPlaneMiss02()
{
	/* ���������عǼн���� 
		��ǰ�ع����ˮƽ�ع���ļн�
	*/
	Eigen::Vector3d normalPlane01, normalSaw;
	cout << "test 01-01" << endl;
	normalPlane01 = ExtractNormalFromPlane("1st cut plane");
	cout << "test 01-02" << endl;
	normalSaw = ExtractNormalFromPlane("StateCutPlane02");
	cout << "test 01-03" << endl;
	// ����������������֮��ļн�
	double dotProduct = normalPlane01[0] * normalSaw[0] + normalPlane01[1] * normalSaw[1] + normalPlane01[2] * normalSaw[2]; // ���

	// ������������
	double magnitude1 = sqrt(normalPlane01[0] * normalPlane01[0] + normalPlane01[1] * normalPlane01[1] + normalPlane01[2] * normalPlane01[2]);
	double magnitude2 = sqrt(normalSaw[0] * normalSaw[0] + normalSaw[1] * normalSaw[1] + normalSaw[2] * normalSaw[2]);

	// ����нǵ�����ֵ
	double cosAngle = dotProduct / (magnitude1 * magnitude2);

	// ʹ�÷����Һ�������нǣ����ȣ�
	double angleInRadians = acos(cosAngle);

	// ������ת��Ϊ����������1λС��
	double angleInDegrees = round(angleInRadians * (180.0 / M_PI) * 10) / 10;

	cout << "test 01-04" << endl;
	// ����нǵ�ʵ�ʵ�λ��
	m_Controls.textBrowser_SagCut->append("Real time Angle Miss: " + QString::number(angleInDegrees));
}

// ����ƽ���ϵ����ɵ�����ڷ�������ԭ��ȷ��������ϵ�µ�����λ��
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

// ����ʵ��λ��
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
	// ����ֱ��AB������
	double vectorAB[3] = { pointB[0] - pointA[0], pointB[1] - pointA[1], pointB[2] - pointA[2] };

	// ����AC����
	double vectorAC[3] = { pointC[0] - pointA[0], pointC[1] - pointA[1], pointC[2] - pointA[2] };

	// ����ͶӰ����
	double projection = (vectorAC[0] * vectorAB[0] + vectorAC[1] * vectorAB[1] + vectorAC[2] * vectorAB[2]) /
		(vectorAB[0] * vectorAB[0] + vectorAB[1] * vectorAB[1] + vectorAB[2] * vectorAB[2]);

	// ���㴹ֱ����
	double vectorAP[3] = { vectorAC[0] - projection * vectorAB[0],
						  vectorAC[1] - projection * vectorAB[1],
						  vectorAC[2] - projection * vectorAB[2] };

	// �������
	double distance = sqrt(vectorAP[0] * vectorAP[0] + vectorAP[1] * vectorAP[1] + vectorAP[2] * vectorAP[2]);

	return distance;
}