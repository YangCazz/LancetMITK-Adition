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

/*=========================���е���==============================
HTONDI_MidOperation.cpp
----------------------------------------------------------------
== �عǵ���
== ��׵���
== ����������֤
===============================================================*/






// ���е���
void HTONDI::trackingObjectPos()
{
	/* ׷�ٲ����ص�������������ӽ��µ�ת������ 
		1. �ɹ� FemurRF - m_MetrixCameraToFemurRF
		2. �ֹ� TibiaRF - m_MetrixCameraToTibiaRF
		3. ̽�� ProbeRF - m_MetrixCameraToProbeRF
		4. �ھ� SawRF   - m_MetrixCameraToSawRF
		5. ĥ�� DrillRF - m_MetrixCameraToDrillRF
	*/
	// ��ʼ������׼��ɼ�
	// Pos_Tip = T_Camera2ProbeRF * pos_TipOnProbeRF
	auto femurIndex = m_VegaToolStorage->GetToolIndexByName("FemurRF");
	auto tibiaRFIndex = m_VegaToolStorage->GetToolIndexByName("TibiaRF");
	auto probeIndex = m_VegaToolStorage->GetToolIndexByName("ProbeRF");
	auto sawIndex = m_VegaToolStorage->GetToolIndexByName("SawRF");
	auto drillRFIndex = m_VegaToolStorage->GetToolIndexByName("DrillRF");

	// ��ȡ̽�����λ��
	auto boneFemur = m_VegaToolStorage->GetToolByName("FemurRF");
	auto boneTibia = m_VegaToolStorage->GetToolByName("TibiaRF");
	auto toolProbe = m_VegaToolStorage->GetToolByName("ProbeRF");
	auto toolSaw = m_VegaToolStorage->GetToolByName("SawRF");
	auto toolDrill = m_VegaToolStorage->GetToolByName("DrillRF");

	// ȡ��̽������������µ�λ��
	// 4x4��α任����
	Eigen::Matrix4d T_Camera2FemurRF = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d T_Camera2TibiaRF = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d T_Camera2ProbeRF = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d T_Camera2SawRF = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d T_Camera2DrillRF = Eigen::Matrix4d::Identity();

	// ���㵱ǰ�궨��λ��
	Eigen::Vector4d CurrentFemurCheckPointOnImage;
	Eigen::Vector4d CurrentTibiaCheckPointOnImage;
	Eigen::Vector4d CurrentTipOnImage;
	Eigen::Vector4d CurrentSawPointsOnImage;
	Eigen::Vector4d CurrentDrillPointsOnImage;

	// 1. FemurRF
	// �ɹǵĴ�����Ҫ����ع���н�
	if (femurIndex != -1)
	{
		m_Controls.textBrowser_Action->append("Record Current FemurRF Pos:");
		mitk::NavigationData::Pointer pos_FemurRFOnCamera = m_VegaSource->GetOutput(probeIndex);
		if (pos_FemurRFOnCamera)
		{
			// 	����T_Camera2TibiaRF
			Eigen::Vector3d position(
				pos_FemurRFOnCamera->GetPosition()[0], pos_FemurRFOnCamera->GetPosition()[1], pos_FemurRFOnCamera->GetPosition()[2]);
			Eigen::Quaterniond orientation(
				pos_FemurRFOnCamera->GetOrientation().r(), pos_FemurRFOnCamera->GetOrientation().x(),
				pos_FemurRFOnCamera->GetOrientation().y(), pos_FemurRFOnCamera->GetOrientation().z()
			);

			// ��3x3����ת�������õ�4x4��������Ͻ�
			T_Camera2FemurRF.block<3, 3>(0, 0) = orientation.toRotationMatrix();
			// ��3x1��ƽ���������õ�4x4��������Ͻ�
			T_Camera2FemurRF.block<3, 1>(0, 3) = position;
		}

		// ����Femur�궨����Camera�µ�����ֵ
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

		// ����Femur�궨����Image�µ�����
		// Pos_FemurCheckPointOnImage = T_FemurRF2M.inverse() * T_Camera2FemurRF.inverse() * pos_FemurCheckPointOnCamera
		CurrentFemurCheckPointOnImage = m_Metrix4dFemurRFToImage.inverse() * T_Camera2FemurRF.inverse() * m_CurrentFemurCheckPointOnCamera;
		m_CurrentFemurCheckPointOnImage = CurrentFemurCheckPointOnImage;
		m_Controls.textBrowser_Action->append("Added CurrentFemurCheckPointOnImage: ("
			     + QString::number(m_CurrentFemurCheckPointOnImage[0]) +
			", " + QString::number(m_CurrentFemurCheckPointOnImage[1]) +
			", " + QString::number(m_CurrentFemurCheckPointOnImage[2]) +
			", " + QString::number(m_CurrentFemurCheckPointOnImage[3]) + ")"
		);

		// ����ͼ����¼���


		// Ӧ��ͼ�����ת��



	}
	

	// 2. TibiaRF
	// �ֹ�
	if (femurIndex != -1)
	{
		m_Controls.textBrowser_Action->append("Record Current TibiaRF Pos:");
		mitk::NavigationData::Pointer pos_TibiaRFOnCamera = m_VegaSource->GetOutput(probeIndex);
		if (pos_TibiaRFOnCamera)
		{
			// 	����T_Camera2TibiaRF
			Eigen::Vector3d position(
				pos_TibiaRFOnCamera->GetPosition()[0], pos_TibiaRFOnCamera->GetPosition()[1], pos_TibiaRFOnCamera->GetPosition()[2]);
			Eigen::Quaterniond orientation(
				pos_TibiaRFOnCamera->GetOrientation().r(), pos_TibiaRFOnCamera->GetOrientation().x(),
				pos_TibiaRFOnCamera->GetOrientation().y(), pos_TibiaRFOnCamera->GetOrientation().z()
			);

			// ��3x3����ת�������õ�4x4��������Ͻ�
			T_Camera2TibiaRF.block<3, 3>(0, 0) = orientation.toRotationMatrix();
			// ��3x1��ƽ���������õ�4x4��������Ͻ�
			T_Camera2TibiaRF.block<3, 1>(0, 3) = position;
		}

		// ����Tibia�궨����Camera�µ�����ֵ
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

		// ����Tibia�궨����Image�µ�����
		// Pos_TibiaCheckPointOnImage = T_TibiaRF2M.inverse() * T_Camera2TibiaRF.inverse() * pos_TibiaCheckPointOnCamera
		CurrentTibiaCheckPointOnImage = m_Metrix4dTibiaRFToImage.inverse() * T_Camera2TibiaRF.inverse() * m_CurrentTibiaCheckPointOnCamera;
		m_CurrentTibiaCheckPointOnImage = CurrentTibiaCheckPointOnImage;
		m_Controls.textBrowser_Action->append("Added CurrentFemurCheckPointOnImage: ("
			     + QString::number(m_CurrentTibiaCheckPointOnImage[0]) +
			", " + QString::number(m_CurrentTibiaCheckPointOnImage[1]) +
			", " + QString::number(m_CurrentTibiaCheckPointOnImage[2]) +
			", " + QString::number(m_CurrentTibiaCheckPointOnImage[3]) + ")"
		);

		// ����ͼ����¼���


		// Ӧ��ͼ�����ת��



	}
	

	// 3. ProbeRF
	// ̽��RF����ʶ��̽��λ��
	if (femurIndex != -1)
	{
		m_Controls.textBrowser_Action->append("Record Current ProbeRF Pos:");
		// ��ȡ̽�����λ��
		mitk::Point3D pos_TipOnProbeRF = toolProbe->GetToolTipPosition();
		// ת��Ϊ4d
		Eigen::Vector4d pos_TipOnProbeRF4d(pos_TipOnProbeRF[0], pos_TipOnProbeRF[1], pos_TipOnProbeRF[2], 1.0);
		mitk::NavigationData::Pointer pos_ProbeRFOnCamera = m_VegaSource->GetOutput(probeIndex);
		if (pos_ProbeRFOnCamera)
		{
			// 	����T_Camera2TibiaRF
			Eigen::Vector3d position(
				pos_ProbeRFOnCamera->GetPosition()[0], pos_ProbeRFOnCamera->GetPosition()[1], pos_ProbeRFOnCamera->GetPosition()[2]);
			Eigen::Quaterniond orientation(
				pos_ProbeRFOnCamera->GetOrientation().r(), pos_ProbeRFOnCamera->GetOrientation().x(),
				pos_ProbeRFOnCamera->GetOrientation().y(), pos_ProbeRFOnCamera->GetOrientation().z()
			);

			// ��3x3����ת�������õ�4x4��������Ͻ�
			T_Camera2ProbeRF.block<3, 3>(0, 0) = orientation.toRotationMatrix();
			// ��3x1��ƽ���������õ�4x4��������Ͻ�
			T_Camera2ProbeRF.block<3, 1>(0, 3) = position;
		}
		cout << "pos_TipOnProbeRF: (" << pos_TipOnProbeRF[0] << " , " << pos_TipOnProbeRF[1] << " , " << pos_TipOnProbeRF[2] << ")" << endl;
		
		// ����̽������Camera�µ�����ֵ
		// Pos_Tip = T_Camera2ProbeRF * pos_TipOnProbeRF
		Eigen::Vector4d pos_TipOnCamera = T_Camera2ProbeRF * pos_TipOnProbeRF4d;

		cout << "pos_TipOnCamera: (" << pos_TipOnCamera[0] << " , " << pos_TipOnCamera[1] << " , " << pos_TipOnCamera[2] << ")" << endl;

		// ����ProbeTip��Camera�µ�����ֵ
		// Pos_Tip = T_Camera2ProbeRF * pos_TipOnProbeRF
		pos_TipOnCamera = T_Camera2ProbeRF * pos_TipOnProbeRF4d;
		m_CurrentTipOnCamera = pos_TipOnCamera;
		m_Controls.textBrowser_Action->append("Added m_CurrentTipOnCamera: ("
			+ QString::number(m_CurrentTipOnCamera[0]) +
			", " + QString::number(m_CurrentTipOnCamera[1]) +
			", " + QString::number(m_CurrentTipOnCamera[2]) +
			", " + QString::number(m_CurrentTipOnCamera[3]) + ")"
		);

		// ����ProbeTip��Image�µ�����
		// Pos_TipOnImage = T_TibiaRF2M.inverse() * T_Camera2TibiaRF.inverse() * pos_TibiaCheckPointOnCamera
		CurrentTipOnImage = m_Metrix4dTibiaRFToImage.inverse() * T_Camera2TibiaRF.inverse() * m_CurrentTipOnCamera;
		m_CurrentTipOnImage = CurrentTipOnImage;
		m_Controls.textBrowser_Action->append("Added CurrentTipOnImage: ("
			     + QString::number(m_CurrentTipOnImage[0]) +
			", " + QString::number(m_CurrentTipOnImage[1]) +
			", " + QString::number(m_CurrentTipOnImage[2]) +
			", " + QString::number(m_CurrentTipOnImage[3]) + ")"
		);

		// ����ͼ����¼��㣬 �����ϸ�λ�ú͵�ǰλ�������и���
		// T_last2recent = N_pointsOnM_recent * N_pointsOnM_last.inverse()
		// ����̽�룬��ֱ�Ӹı�̽��λ��
		auto probeRf = GetDataStorage()->GetNamedNode("ProbeRF");
		auto probe = GetDataStorage()->GetNamedNode("Probe");
		mitk::Point3D pos_tip;
		pos_tip[0] = CurrentTipOnImage[0];
		pos_tip[1] = CurrentTipOnImage[1];
		pos_tip[2] = CurrentTipOnImage[2];

		// Ӧ��ͼ�����ת��
		//probeRf->GetData()->GetGeometry()->SetOrigin(pos_tip);
		probe->GetData()->GetGeometry()->SetOrigin(pos_tip);
	}


	// 4. SawRF
	// ���ڽعǵ���
	if (femurIndex != -1)
	{
		m_Controls.textBrowser_Action->append("Record Current SawRF Pos:");
		mitk::NavigationData::Pointer pos_SawRFOnCamera = m_VegaSource->GetOutput(probeIndex);
		if (pos_SawRFOnCamera)
		{
			// 	����T_Camera2TibiaRF
			Eigen::Vector3d position(
				pos_SawRFOnCamera->GetPosition()[0], pos_SawRFOnCamera->GetPosition()[1], pos_SawRFOnCamera->GetPosition()[2]);
			Eigen::Quaterniond orientation(
				pos_SawRFOnCamera->GetOrientation().r(), pos_SawRFOnCamera->GetOrientation().x(),
				pos_SawRFOnCamera->GetOrientation().y(), pos_SawRFOnCamera->GetOrientation().z()
			);

			// ��3x3����ת�������õ�4x4��������Ͻ�
			T_Camera2SawRF.block<3, 3>(0, 0) = orientation.toRotationMatrix();
			// ��3x1��ƽ���������õ�4x4��������Ͻ�
			T_Camera2SawRF.block<3, 1>(0, 3) = position;
		}

		// ����Saw�궨����Camera�µ�����ֵ
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
			// ��¼��ǰ������µ�λ��
			m_CurrentSawPointsOnCamera.push_back(SawPoint_tmp);
			// ���㲢��¼��Ӱ��ռ��λ��
			Eigen::Vector4d SawPoint = m_Metrix4dTibiaRFToImage.inverse() * T_Camera2TibiaRF.inverse() * SawPoint_tmp;
			m_CurrentSawPointsOnImage.push_back(SawPoint);
		}
		
		// ����ͼ����¼��㣬 �����ϸ�λ�ú͵�ǰλ�������и���
		// T_last2recent = N_pointsOnM_recent * N_pointsOnM_last.inverse()
		// ���ڰھ⣬ȥ���㵱ǰͼ������㵽ͶӰ���λ��




		// Ӧ��ͼ�����ת��

		// ��֪ A B C�������Լ�����ϵ�µ�����
		// D F Ϊ�ھ��Ե�㣬�����������㷽��
		// AD = aAB + bAC => a = 403/628, b = -437/628
		// AF = cAB + dAC => c = 733/628, d = -131/628



	}
	

	// 5. DrillRF
	// ����ĥ�굼��
	if (femurIndex != -1)
	{
		m_Controls.textBrowser_Action->append("Record Current DrillRF Pos:");
		mitk::NavigationData::Pointer pos_DrillRFOnCamera = m_VegaSource->GetOutput(probeIndex);
		if (pos_DrillRFOnCamera)
		{
			// 	����T_Camera2TibiaRF
			Eigen::Vector3d position(
				pos_DrillRFOnCamera->GetPosition()[0], pos_DrillRFOnCamera->GetPosition()[1], pos_DrillRFOnCamera->GetPosition()[2]);
			Eigen::Quaterniond orientation(
				pos_DrillRFOnCamera->GetOrientation().r(), pos_DrillRFOnCamera->GetOrientation().x(),
				pos_DrillRFOnCamera->GetOrientation().y(), pos_DrillRFOnCamera->GetOrientation().z()
			);

			// ��3x3����ת�������õ�4x4��������Ͻ�
			T_Camera2DrillRF.block<3, 3>(0, 0) = orientation.toRotationMatrix();
			// ��3x1��ƽ���������õ�4x4��������Ͻ�
			T_Camera2DrillRF.block<3, 1>(0, 3) = position;
		}

		// ����Drill�궨����Camera�µ�����ֵ
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

			// ��¼��ǰ������µ�λ��
			m_CurrentSawPointsOnCamera.push_back(DrillPoint_tmp);
			// ���㲢��¼��Ӱ��ռ��λ��
			Eigen::Vector4d DrillPoint = m_Metrix4dTibiaRFToImage.inverse() * T_Camera2TibiaRF.inverse() * DrillPoint_tmp;
			m_CurrentSawPointsOnImage.push_back(DrillPoint);

		}
		
		// ����ͼ����¼���


		// Ӧ��ͼ�����ת��



	}
}



void HTONDI::GenerateRealTimeBoneSurface()
{
	/* ����ʵʱ�Ľع���
	1. �ҵ���е�ϵı궨�㣬���㷨������ǰ�˵�λ�ã����ɽع���
	2. ����ʵʱ��͹滮��ļн�
	*/

	// ȡ���ھ��ϵļ����궨��λ => ����ھ�ķ�����
	mitk::Point3D PlanePoint1, PlanePoint2, PlanePoint3;
	
	if (saw_image) {
		auto SawPointSet = saw_image->GetLandmarks();
		// ��Զ����
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
	
	// ����ƽ��ķ����������ݰھ��ϵ������궨��λ����ƽ��ķ�����
	Eigen::Vector3d normalVector;
	Eigen::Vector3d vector1 = Eigen::Vector3d(PlanePoint2[0] - PlanePoint1[0], PlanePoint2[1] - PlanePoint1[1], PlanePoint2[2] - PlanePoint1[2]);
	Eigen::Vector3d vector2 = Eigen::Vector3d(PlanePoint3[0] - PlanePoint1[0], PlanePoint3[1] - PlanePoint1[1], PlanePoint3[2] - PlanePoint1[2]);
	normalVector = vector1.cross(vector2).normalized();
	
	// ������
	double normal[3]
	{
		normalVector[0],
		normalVector[1],
		normalVector[2]
	};

	// ����ƽ������ĵ�Ϊ ��������
	double origin[3]
	{
		(PlanePoint1[0] + PlanePoint1[0] + PlanePoint1[0]) / 3,
		(PlanePoint1[1] + PlanePoint1[1] + PlanePoint1[1]) / 3,
		(PlanePoint1[2] + PlanePoint1[2] + PlanePoint1[2]) / 3
	};

	// ����һ����ʼ�Ľع�ƽ�棬������λ�úͷ���
	auto realTimeSawPlaneSource = vtkSmartPointer<vtkPlaneSource>::New();
	// ���ɳ�ʼƽ��
	realTimeSawPlaneSource->SetOrigin(0, 0, 0);

	// �趨���ӻ��ع�ƽ��Ĵ�С��70*70
	realTimeSawPlaneSource->SetPoint1(0, 70, 0);
	realTimeSawPlaneSource->SetPoint2(70, 0, 0);
	realTimeSawPlaneSource->SetNormal(normal);//���÷�����

	// ����ʼ�����Ľع�ƽ���ƶ������������λ������
	realTimeSawPlaneSource->SetCenter(origin);

	// ������
	auto realTimeSawPlane = mitk::Surface::New();
	realTimeSawPlane->SetVtkPolyData(realTimeSawPlaneSource->GetOutput());

	// ����ʵʱƽ��
	auto tmpPlane = GetDataStorage()->GetNamedNode("realTimeCutPlane");
	if (tmpPlane)
	{
		tmpPlane->SetData(realTimeSawPlane);
	}
	else 
	{
		// �����ع�ƽ��DataNode����
		auto planeNode = mitk::DataNode::New();
		planeNode->SetData(realTimeSawPlane);
		planeNode->SetColor(1.0, 0.0, 0.0);
		planeNode->SetOpacity(0.5);
		planeNode->SetName("realTimeCutPlane");
		GetDataStorage()->Add(planeNode);
	}
	
	// �����Ƭ��ǰ�˵�λ�ã�����ع����

	// ��������ƽ��ļн�
	auto cutPlaneSource01 = GetDataStorage()->GetNamedNode("1st cut plane")->GetData();
}

void HTONDI::CalculateRealTimeCutAngle()
{
	/* ����ع���ļн� */

}


bool HTONDI::OnStartAxialGuideClicked()
{
	/* ��ʼˮƽ�عǵ���
	1. ����ʵʱ�ع���
	2. ���� ʵʱ�ع��� �� �滮�ع��� �ļн�

	��ʼˮƽ�عǵ���ʱ����Ҫ����ع���ķ�������ǰ��λ��
	��ô��Ҳ����Ҫ֪��ʵʱ�İھ���ǰ�˵Ľع�����Լ���Ӧ�Ľع��淨����
	*/

	m_Controls.textBrowser_Action->append("Action: Start Axial Cut Guide.");

	// 1. ��ʾ�滮�Ľع��� + ��ʾʵʱ�ع���λ��
	auto preCutPlane01 = GetDataStorage()->GetNamedNode("1st cut plane");
	auto realTimeSaw = GetDataStorage()->GetNamedNode("Saw");

	// ������ݴ��ڣ�Ȼ��򿪽عǵ���
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

	// ��Saw����ʵʱ�ع�ƽ��
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
	/* ����ˮƽ�ع�
	1. �ı�ھ�ع�״̬
	2. ����ʵʱ �ع����
	3. ���нعǱ���
	*/

	// ��ʼˮƽ�عǣ������ھ⣬�����عǵ���
	m_Controls.textBrowser_Action->append("Action: Check Drill Node.");

	return true;
}

bool HTONDI::OnStartStateAxialCutClicked()
{
	/* ��̬�ع�ģ�⣬���ɾ�̬ƽ��
		1. ��ȡ��̬ģ�ͱ���궨��
		2. ���ɳ�ʼ�ľ�̬ģ�ʹ���Ľع���
		3. ���ɽع�ƽ���ʼ��, Ȼ��Ӧ�õ���ǰ������ʼ�ķ�����
		4. ���㵱ǰ�ع���н�

	�������������ģ��عǹ���
	*/
	m_Controls.textBrowser_Action->append("Action: Generate State Axial Cut Plane.");

	// ���ýعǷ�����״̬
	m_cutType = 1;

	// 1. ��ȡ��ǰ��̬ģ�ͱ������ݺͱ궨��ĵ�ǰλ��
	mitk::Point3D PlanePoint1, PlanePoint2, PlanePoint3;
	auto SawPoints = GetDataStorage()->GetNamedNode("SawLandMarkPointSet");
	auto Saw = GetDataStorage()->GetNamedNode("Saw");

	// ��Զ����
	if (SawPoints) {
		// ��Զ������ȡ��������
		auto SawPointSet = dynamic_cast<mitk::PointSet*>(SawPoints->GetData());
		PlanePoint1 = SawPointSet->GetPoint(0);
		PlanePoint2 = SawPointSet->GetPoint(1);
		PlanePoint3 = SawPointSet->GetPoint(2);
		// ��Զ������ȡ��������
		SawPoints->SetVisibility(true);
	}
	else
	{
		m_Controls.textBrowser_Action->append("SawLandMarkPointSet Not Found.");
		return false;
	}
	if (Saw) {
		// ��Զ������ȡ��������
		Saw->SetVisibility(true);
	}
	else
	{
		m_Controls.textBrowser_Action->append("Saw model Not Found.");
		return false;
	}

	// ���㵱ǰ�ھ�ƽ��� ������ normalSaw �����ݰھ��ϵ������궨��λ����ƽ��ķ�����
	Eigen::Vector3d normalVector;
	Eigen::Vector3d vector1 = Eigen::Vector3d(PlanePoint2[0] - PlanePoint1[0], PlanePoint2[1] - PlanePoint1[1], PlanePoint2[2] - PlanePoint1[2]);
	Eigen::Vector3d vector2 = Eigen::Vector3d(PlanePoint3[0] - PlanePoint1[0], PlanePoint3[1] - PlanePoint1[1], PlanePoint3[2] - PlanePoint1[2]);
	normalVector = vector1.cross(vector2).normalized();

	// ������
	double normalSaw[3]
	{
		normalVector[0],
		normalVector[1],
		normalVector[2]
	};

	// ����ƽ������ĵ�Ϊ ��������, ����λ���ƶ����ھ���ǰ��
	double originSaw[3]
	{
		(PlanePoint1[0] + PlanePoint2[0] + PlanePoint3[0]) / 3,
		(PlanePoint1[1] + PlanePoint2[1] + PlanePoint3[1]) / 3,
		(PlanePoint1[2] + PlanePoint2[2] + PlanePoint3[2]) / 3
	};


	// 2. ���ɰھ�ع�ƽ��
	// ����һ����ʼ�Ľع�ƽ�棬������λ�úͷ���
	auto realTimeSawPlaneSource = vtkSmartPointer<vtkPlaneSource>::New();

	// ���ɳ�ʼƽ��
	//realTimeSawPlaneSource->SetOrigin(0, 0, 0);
	
	// �趨���ӻ��ع�ƽ��Ĵ�С��70*70
	realTimeSawPlaneSource->SetPoint1(0, 70, 0);
	realTimeSawPlaneSource->SetPoint2(70, 0, 0);


	// ����ʼ�����Ľع�ƽ���ƶ������������λ������
	realTimeSawPlaneSource->SetCenter(originSaw);
	realTimeSawPlaneSource->SetNormal(normalSaw);

	// ͼ�����
	realTimeSawPlaneSource->Update();

	// ������
	auto realTimeSawPlane = mitk::Surface::New();
	realTimeSawPlane->SetVtkPolyData(realTimeSawPlaneSource->GetOutput());

	// ����ʵʱƽ��
	auto tmpPlane = GetDataStorage()->GetNamedNode("StateCutPlane01");
	if (tmpPlane)
	{
		tmpPlane->SetData(realTimeSawPlane);
		m_Controls.textBrowser_Action->append("Plane updated.");
	}
	else
	{
		// �����ع�ƽ��DataNode����
		auto planeNode = mitk::DataNode::New();
		planeNode->SetData(realTimeSawPlane);
		planeNode->SetColor(1.0, 1.0, 0.0);
		planeNode->SetOpacity(0.5);
		planeNode->SetName("StateCutPlane01");

		// ��ӵ���
		GetDataStorage()->Add(planeNode);
		m_Controls.textBrowser_Action->append("Plane created.");
	}

	// �����Ƭ��ǰ�˵�λ�ã�����ع����

	// 3. ���ɽع�����ǰ�˵㣬ͬ��ǰ�滮����

	// ���ӻ��ڵ�
	auto tmpNodes = GetDataStorage()->GetNamedNode("pointSetInRealPlaneAxial");
	if (tmpNodes) {
		//GetDataStorage()->Remove(tmpNodes);
		/* ������ǵ�һ�����ɰھ�ƽ��
			1. ȡ����ǰ��ԭ��ͷ�����
			2. ���ݴ洢�����λ��������ʵ��λ��
			3. ���½ڵ�����
		*/
		mitk::Point3D point0;
		mitk::Point3D point1;
		mitk::Point3D point2;
		mitk::Point3D point3;
		mitk::Point3D point4;
		mitk::Point3D planeCenterPoint;//ˮƽ�����ĵ�
		// ʹ�� CalculateActualPoints ��������ʵ��λ��
		std::vector<mitk::Point3D> actualPoints = CalculateActualPoints(m_PointsOnSaw, normalVector, Eigen::Vector3d(originSaw[0], originSaw[1], originSaw[2]));
		mitkPointSetRealTime->SetPoint(0, point0);//��һ�ع���ĩ���е���
		mitkPointSetRealTime->SetPoint(1, point1);
		mitkPointSetRealTime->SetPoint(2, point2);
		mitkPointSetRealTime->SetPoint(3, point3);
		mitkPointSetRealTime->SetPoint(4, point4);
		// ��������
		tmpNodes->SetData(mitkPointSetRealTime);
	}
	else 
	{
		/* ����ǵ�һ�����ɰھ��ƽ��
			1. ��ʼ��Ϊˮƽλ��(����ģ�ͳ�ʼλ��)
			2. ���㲢�洢ƽ��ؼ����ڰھ�ƽ���µ�λ��
			3. Ȼ����п��ӻ�����

		ע������Ĭ�ϳ�ʼ���صİھ���ˮƽ
		*/

		/* 1. ˮƽ��λ�ó�ʼ�� */
		// ����ع�ĩ�˺�ҳ��ĳ�ʼ����
		mitk::Point3D point0;
		mitk::Point3D point1;
		mitk::Point3D point2;
		mitk::Point3D point3;
		mitk::Point3D point4;
		mitk::Point3D planeCenterPoint;//ˮƽ�����ĵ�

		// �� 1 - ǰ���е�
		point0[0] = originSaw[0] - 35;
		point0[1] = originSaw[1];
		point0[2] = originSaw[2];
		// �� 2 - ǰ�����
		point1[0] = originSaw[0] - 35;
		point1[1] = originSaw[1] - 35;
		point1[2] = originSaw[2];
		// �� 3 - ǰ���Ҳ�
		point2[0] = originSaw[0] - 35;
		point2[1] = originSaw[1] + 35;
		point2[2] = originSaw[2];
		// �� 4 - ĩ�����
		point3[0] = originSaw[0] + 35;
		point3[1] = originSaw[1] - 35;
		point3[2] = originSaw[2];
		// �� 5 - ĩ���е� 
		point4[0] = originSaw[0] + 35;
		point4[1] = originSaw[1];
		point4[2] = originSaw[2];
		// �� 6 - ����ԭ��
		planeCenterPoint[0] = originSaw[0];
		planeCenterPoint[1] = originSaw[1];
		planeCenterPoint[2] = originSaw[2];

		// ������Щ������ڰھ�����ϵ��λ��
		m_PointsOnSaw = CalculateRelativePoints({ point0, point1, point2, point3, point4 }, 
			normalVector, Eigen::Vector3d(originSaw[0], originSaw[1], originSaw[2]));

		// ��������ӵ�mitk::PointSet��
		// ���������ع������ת����
		mitkPointSetRealTime->InsertPoint(0, point0);//��һ�ع���ĩ���е���
		mitkPointSetRealTime->InsertPoint(1, point1);
		mitkPointSetRealTime->InsertPoint(2, point2);
		mitkPointSetRealTime->InsertPoint(3, point3);
		mitkPointSetRealTime->InsertPoint(4, point4);
		mitkPointSetRealTime->InsertPoint(5, planeCenterPoint);//ƽ��ԭ��
		// ���򴴽��µ�ƽ��
		mitk::DataNode::Pointer pointSetInPlaneCutPlane = mitk::DataNode::New();
		pointSetInPlaneCutPlane->SetName("pointSetInRealPlaneAxial");
		// ��ɫ����С 5.0
		pointSetInPlaneCutPlane->SetColor(1.0, 0.0, 0.0);
		pointSetInPlaneCutPlane->SetData(mitkPointSetRealTime);
		pointSetInPlaneCutPlane->SetFloatProperty("pointsize", 5.0);
		GetDataStorage()->Add(pointSetInPlaneCutPlane);
	}
	


	// 4. ��������ƽ��ļн�

	// ��ȡˮƽ�ع���ķ�����
	auto cutPlaneSource01 = dynamic_cast<mitk::Surface*>(GetDataStorage()->GetNamedNode("1st cut plane")->GetData());
	auto cutPlanePolyData01 = cutPlaneSource01->GetVtkPolyData();

	Eigen::Vector3d normalPlane01;
	normalPlane01 = ExtractNormalFromPlane("1st cut plane");

	// ������������
	cout << "normalPlane: " << " " << normalPlane01[0] << " " << normalPlane01[1] << " " << normalPlane01[2] << endl;
	cout << "normalSaw  : " << " " << normalSaw[0] << " " << normalSaw[1] << " " << normalSaw[2] << endl;

	//cout << "CenterPlane: " << " " << centerPlane01[0] << " " << centerPlane01[1] << " " << centerPlane01[2] << endl;
	cout << "CenterSaw  : " << " " << originSaw[0] << " " << originSaw[1] << " " << originSaw[2] << endl;

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

	// ����нǵ�ʵ�ʵ�λ��
	m_Controls.textBrowser_AxialCut->append("Real time Angle Miss: " + QString::number(angleInDegrees));

	GetDataStorage()->Modified();
	mitk::RenderingManager::GetInstance()->RequestUpdateAll();
	return true;
}

bool HTONDI::OnCheckStateCutClicked()
{
	/* ȷ��ˮƽ�ع���λ��	*/
	m_Controls.textBrowser_Action->append("Action: Check State Cut Plane.");
	m_cutType = -1;
	return true;
}


bool HTONDI::OnStartSagGuideClicked()
{
	/* ȷ��ˮƽ�ع���λ��	*/
	m_Controls.textBrowser_Action->append("Action: Check State Cut Plane.");
	m_cutType = -1;
	return true;
}

bool HTONDI::OnStartStateSagCutClicked()
{
	/* ��̬�ع�ģ�⣬���ɾ�̬ƽ��
		1. ��ȡ��̬ģ�ͱ���궨��
		2. ���ɳ�ʼ�ľ�̬ģ�ʹ���Ľع���
		3. ���ɽع�ƽ���ʼ��
		4. ���㵱ǰ�ع���н�

	�������������ģ��عǹ���
	*/

	m_Controls.textBrowser_Action->append("Action: Generate State Sag Cut Plane.");

	// ���ýعǷ�����״̬
	m_cutType = 2;

	// 1. ��ȡ��̬ģ�ͱ���ı궨��
	mitk::Point3D PlanePoint1, PlanePoint2, PlanePoint3;
	auto SawPoints = GetDataStorage()->GetNamedNode("SawLandMarkPointSet");
	auto Saw = GetDataStorage()->GetNamedNode("Saw");

	// ��Զ����
	if (SawPoints) {
		// ��Զ������ȡ��������
		auto SawPointSet = dynamic_cast<mitk::PointSet*>(SawPoints->GetData());
		PlanePoint1 = SawPointSet->GetPoint(0);
		PlanePoint2 = SawPointSet->GetPoint(1);
		PlanePoint3 = SawPointSet->GetPoint(2);
		// ��Զ������ȡ��������
		SawPoints->SetVisibility(true);
	}
	else
	{
		m_Controls.textBrowser_Action->append("SawLandMarkPointSet Not Found.");
		return false;
	}
	if (Saw) {
		// ��Զ������ȡ��������
		Saw->SetVisibility(true);
	}
	else
	{
		m_Controls.textBrowser_Action->append("Saw model Not Found.");
		return false;
	}

	// ����ھ�ƽ��� ������ normalSaw �����ݰھ��ϵ������궨��λ����ƽ��ķ�����
	Eigen::Vector3d normalVector;
	Eigen::Vector3d vector1 = Eigen::Vector3d(PlanePoint2[0] - PlanePoint1[0], PlanePoint2[1] - PlanePoint1[1], PlanePoint2[2] - PlanePoint1[2]);
	Eigen::Vector3d vector2 = Eigen::Vector3d(PlanePoint3[0] - PlanePoint1[0], PlanePoint3[1] - PlanePoint1[1], PlanePoint3[2] - PlanePoint1[2]);
	normalVector = vector1.cross(vector2).normalized();

	// ������
	double normalSaw[3]
	{
		normalVector[0],
		normalVector[1],
		normalVector[2]
	};

	// ����ƽ������ĵ�Ϊ ��������, ����λ���ƶ����ھ���ǰ��
	double originSaw[3]
	{
		(PlanePoint1[0] + PlanePoint1[0] + PlanePoint1[0]) / 3 + 25,
		(PlanePoint1[1] + PlanePoint1[1] + PlanePoint1[1]) / 3,
		(PlanePoint1[2] + PlanePoint1[2] + PlanePoint1[2]) / 3
	};


	// 2. ���ɰھ�ع�ƽ��
	// ����һ����ʼ�Ľع�ƽ�棬������λ�úͷ���
	auto realTimeSawPlaneSource = vtkSmartPointer<vtkPlaneSource>::New();

	// ���ɳ�ʼƽ��
	realTimeSawPlaneSource->SetOrigin(0, 0, 0);

	// �趨���ӻ��ع�ƽ��Ĵ�С��70*70
	realTimeSawPlaneSource->SetPoint1(0, 70, 0);
	realTimeSawPlaneSource->SetPoint2(70, 0, 0);
	realTimeSawPlaneSource->SetNormal(0, 0, 1);//���÷�����

	// ����ʼ�����Ľع�ƽ���ƶ������������λ������
	realTimeSawPlaneSource->SetCenter(originSaw);
	realTimeSawPlaneSource->SetNormal(normalSaw);
	

	// ͼ�����
	realTimeSawPlaneSource->Update();

	// ������
	auto realTimeSawPlane = mitk::Surface::New();
	realTimeSawPlane->SetVtkPolyData(realTimeSawPlaneSource->GetOutput());

	// ����ʵʱƽ��
	auto tmpPlane = GetDataStorage()->GetNamedNode("StateCutPlane02");
	if (tmpPlane)
	{
		tmpPlane->SetData(realTimeSawPlane);
		m_Controls.textBrowser_Action->append("Plane updated.");
	}
	else
	{
		// �����ع�ƽ��DataNode����
		auto planeNode = mitk::DataNode::New();
		planeNode->SetData(realTimeSawPlane);
		planeNode->SetColor(1.0, 1.0, 0.0);
		planeNode->SetOpacity(0.5);
		planeNode->SetName("StateCutPlane02");

		// ��ӵ���
		GetDataStorage()->Add(planeNode);
		m_Controls.textBrowser_Action->append("Plane created.");
	}

	// �����Ƭ��ǰ�˵�λ�ã�����ع����

	// 3. ���ɽع�����ǰ�˵㣬ͬ��ǰ�滮����

	// ����ع�ĩ�˺�ҳ��ĳ�ʼ����
	mitk::Point3D point0;
	mitk::Point3D point1;
	mitk::Point3D point2;
	mitk::Point3D point3;
	mitk::Point3D point4;
	mitk::Point3D planeCenterPoint;//ˮƽ�����ĵ�

	// �� 1 - ǰ���е�
	point0[0] = originSaw[0] - 35;
	point0[1] = originSaw[1];
	point0[2] = originSaw[2];
	// �� 2 - ǰ�����
	point1[0] = originSaw[0] - 35;
	point1[1] = originSaw[1] - 35;
	point1[2] = originSaw[2];
	// �� 3 - ǰ���Ҳ�
	point2[0] = originSaw[0] - 35;
	point2[1] = originSaw[1] + 35;
	point2[2] = originSaw[2];
	// �� 4 - ĩ�����
	point3[0] = originSaw[0] + 35;
	point3[1] = originSaw[1] - 35;
	point3[2] = originSaw[2];
	// �� 5 - ĩ���е� 
	point4[0] = originSaw[0] + 35;
	point4[1] = originSaw[1];
	point4[2] = originSaw[2];

	// �� 6 - ����ԭ��
	planeCenterPoint[0] = originSaw[0];
	planeCenterPoint[1] = originSaw[1];
	planeCenterPoint[2] = originSaw[2];


	// ��������ӵ�mitk::PointSet��
	// ���������ع������ת����
	mitkPointSetRealTime->InsertPoint(0, point0);//��һ�ع���ĩ���е���
	mitkPointSetRealTime->InsertPoint(1, point1);
	mitkPointSetRealTime->InsertPoint(2, point2);
	mitkPointSetRealTime->InsertPoint(3, point3);
	mitkPointSetRealTime->InsertPoint(4, point4);
	mitkPointSetRealTime->InsertPoint(4, planeCenterPoint);//ƽ��ԭ��

	// ���ӻ��ڵ�
	auto tmpNodes = GetDataStorage()->GetNamedNode("pointSetInRealPlaneSag");
	if (tmpNodes) {
		//GetDataStorage()->Remove(tmpNodes);
		//// ����Ѿ��������������Ϣ
		tmpNodes->SetData(mitkPointSetRealTime);
	}
	else
	{
		// ���򴴽��µ�ƽ��
		mitk::DataNode::Pointer pointSetInPlaneCutPlane = mitk::DataNode::New();
		pointSetInPlaneCutPlane->SetName("pointSetInRealPlaneSag");
		// ��ɫ����С 5.0
		pointSetInPlaneCutPlane->SetColor(1.0, 0.0, 0.0);
		pointSetInPlaneCutPlane->SetData(mitkPointSetRealTime);
		pointSetInPlaneCutPlane->SetFloatProperty("pointsize", 5.0);
		GetDataStorage()->Add(pointSetInPlaneCutPlane);
	}



	// 4. ��������ƽ��ļн�

	// ��ȡˮƽ�ع���ķ�����
	auto cutPlaneSource01 = dynamic_cast<mitk::Surface*>(GetDataStorage()->GetNamedNode("1st cut plane")->GetData());
	auto cutPlanePolyData01 = cutPlaneSource01->GetVtkPolyData();

	Eigen::Vector3d normalPlane01;
	normalPlane01 = ExtractNormalFromPlane("1st cut plane");

	// ������������
	cout << "normalPlane: " << " " << normalPlane01[0] << " " << normalPlane01[1] << " " << normalPlane01[2] << endl;
	cout << "normalSaw  : " << " " << normalSaw[0] << " " << normalSaw[1] << " " << normalSaw[2] << endl;

	//cout << "CenterPlane: " << " " << centerPlane01[0] << " " << centerPlane01[1] << " " << centerPlane01[2] << endl;
	cout << "CenterSaw  : " << " " << originSaw[0] << " " << originSaw[1] << " " << originSaw[2] << endl;

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

	// ����нǵ�ʵ�ʵ�λ��
	m_Controls.textBrowser_SagCut->append("Real time Angle Miss: " + QString::number(angleInDegrees));

	GetDataStorage()->Modified();
	mitk::RenderingManager::GetInstance()->RequestUpdateAll();
	return true;


	return true;
}

bool HTONDI::OnStartSagCutClicked()
{
	/* ���������ع�
	1. �ı�ھ�ع�״̬
	2. ����ʵʱ �ع����
	3. ���нعǱ���
	*/

	m_Controls.textBrowser_Action->append("Action: Check Drill Node.");

	return true;
}

bool HTONDI::OnStartDrillGuideClicked()
{
	/* ��ʼ��׵���
	1. ����ʵʱĥ��λ��
	2. ���� ʵʱ���λ�� �� �滮���λ�� ���
	*/

	m_Controls.textBrowser_Action->append("Action: Check Drill Node.");

	return true;
}

bool HTONDI::OnStartDrillHoleClicked()
{
	/* �������
	1. �ı�ĥ����е״̬
	2. ����ʵʱ ������
	3. ������ױ���
	*/

	m_Controls.textBrowser_Action->append("Action: Check Drill Node.");

	return true;
}

bool HTONDI::OnStartStateDrillHoleClicked()
{
	/* �������
	1. �ı�ĥ����е״̬
	2. ����ʵʱ ������
	3. ������ױ���
	*/

	m_Controls.textBrowser_Action->append("Action: Check Drill Node.");

	return true;
}



// ������֤
bool HTONDI::OnShowResClicked()
{
	// ��ʼ���
	m_Controls.textBrowser_Action->append("Action: Check Drill Node.");

	return true;
}


bool HTONDI::OnUnshowResClicked()
{
	// ��ʼ���
	m_Controls.textBrowser_Action->append("Action: Check Drill Node.");

	return true;
}


bool HTONDI::OnCaculateErrorClicked()
{
	// ��ʼ���
	m_Controls.textBrowser_Action->append("Action: Check Drill Node.");

	return true;
}