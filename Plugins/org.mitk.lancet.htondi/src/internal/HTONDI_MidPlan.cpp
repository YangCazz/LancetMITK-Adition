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
#include "surfaceregistraion.h"
#include "vtkPlaneSource.h"



// lancet
#include <lancetRobotTrackingTool.h>


/*========================= ����ע�� ==================================
HTONDI_MidPlan.cpp
--01--------------------------------------------------------------
== ̽����ת�궨
== �ɹ�/�ֹǱ����֤
== װ����е��׼��
== �ھ�궨
== ĥ��궨
== ��ǵ���֤
--02--------------------------------------------------------------
== װ�ع�����׼��
== �ɹ�ͷ���ļ���
== �ɹǴ���׼
== �ֹǴ���׼
== �ֹǾ���׼
== ��ǵ㲶׽
=====================================================================*/

bool HTONDI::OnSelectPointerClicked()
{
	/* ѡ��̽�빤�� 
	   ̽�빤��������Ϊ:ProbeRF
	*/
	m_Controls.textBrowser_Action->append("Action: Select Pointer.");
	// ��ȡ���ߴ洢�е�̽�빤��
	auto toolProbe = m_VegaToolStorage->GetToolByName("ProbeRF");
	if (toolProbe.IsNull())
	{
		MITK_ERROR << "ProbeRF tool not found.";
		return false;
	}

	// ��ֵ m_ToolToCalibrate
	m_ToolToCalibrate = toolProbe;
	MITK_INFO << "Tool selected: " << m_ToolToCalibrate->GetToolName();
	return true;
}


bool HTONDI::OnCalibratePointerClicked()
{
	/* ̽��궨����
	1. ��ת�ɼ�̽��ת��������Ϣ 100��
	2. ������ϱ궨
	3. ����궨���

	̽�빤��������Ϊ:ProbeRF
	*/

	m_Controls.textBrowser_Action->append("Action: Collect Probe Pos.");

	// ��ʼ�ɼ�������һ��ʱ���ڵ�̽��λ����Ϣ
	m_ProbePositions.clear();
	m_Transformations.clear();
	m_Controls.textBrowser_Action->append("Start collect probe data, please move slowly...");

	cout << "test 01" << endl;
	// ������ʱ�������ڲɼ�����
	if (m_ProbeDataCollectionTimer == nullptr)
	{
		cout << "test 01-01" << endl;
		m_ProbeDataCollectionTimer = new QTimer(this);
		cout << "test 01-02" << endl;
		connect(m_ProbeDataCollectionTimer, &QTimer::timeout, this, &HTONDI::CollectProbeData);
	}
	cout << "test 03" << endl;
	m_ProbeDataCollectionTimer->start(100); // ÿ100ms�ɼ�һ������

	cout << "test 02" << endl;
	// �ȴ������ռ����
	QEventLoop loop;
	connect(m_ProbeDataCollectionTimer, &QTimer::timeout, &loop, [&]() {
		if (m_ProbePositions.size() >= number)
		{
			loop.quit();
		}
		});
	loop.exec();

	cout << "test 03" << endl;
	// ֹͣ��ʱ��
	m_ProbeDataCollectionTimer->stop();

	// ����̽�����λ��
	if (!CalculateProbeTip())
	{
		m_Controls.textBrowser_Action->append("ERROR: Failed to calculate probe tip.");
		return false;
	}

	cout << "test 04" << endl;
	// ע��̽��λ��
	RegisterProbePosition();

	cout << "test 05" << endl;
	// ���㲢��ʾ�궨���
	double calibrationError = CalculateCalibrationError();
	m_Controls.textBrowser_Action->append("Calibration Error (RMS): " + QString::number(calibrationError));

	cout << "test 06" << endl;
	// ���ӻ��ɼ����ĵ������
	// OnVisualizeCollectedPoints();

	return true;
}

bool HTONDI::CollectProbeData()
{
	/* ̽����Ϣ�ɼ�����
	1. ÿ100ms�ɼ�һ�ε�ǰ̽��λ��
	2. �洢̽��λ�õ�ȫ�ֱ���

	̽�빤��������Ϊ:ProbeRF
	*/

	// ��ȡ��ǰ̽��׷������λ��
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
		// ����4x4��α任����
		Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
		// ��3x3����ת�������õ�4x4��������Ͻ�
		transformation.block<3, 3>(0, 0) = orientation.toRotationMatrix();
		// ��3x1��ƽ���������õ�4x4��������Ͻ�
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
	/* ������ϼ���̽����̽��RF�µ�λ��
	1. ����������ϣ������ľ�ֵ����Ϊ̽��λ��

	̽�빤��������Ϊ:ProbeRF
	*/
	m_Controls.textBrowser_Action->append("Action: Calculate Probe Tip.");

	// ����ֳ�20��һ��
	//int groupSize = 100;
	int numGroups = m_ProbePositions.size() / groupSize;
	cout << "numGroups: " << numGroups << endl;
	std::vector<Eigen::Vector3d> centers;

	for (int i = 0; i < numGroups; ++i) {
		std::vector<Eigen::Vector3d> subsetPoints(m_ProbePositions.begin() + i * groupSize, m_ProbePositions.begin() + (i + 1) * groupSize);
		auto [center, _] = FitSphere(subsetPoints);
		centers.push_back(center);
	}

	// �������ĵľ�ֵ
	Eigen::Vector3d averageCenter(0, 0, 0);
	for (const auto& center : centers) {
		averageCenter += center;
	}
	averageCenter /= centers.size();
	m_Center = averageCenter;

	// ����̽������F_probe�µ�����ֵ
	std::vector<Eigen::Vector3d> probeTips;
	for (const auto& T_cameraToProbe : m_Transformations)
	{
		Eigen::Vector4d p_homogeneous(averageCenter[0], averageCenter[1], averageCenter[2], 1.0);
		Eigen::Vector4d s_homogeneous = T_cameraToProbe.inverse() * p_homogeneous;
		Eigen::Vector3d s = s_homogeneous.head<3>();
		probeTips.push_back(s);
	}

	// ��������̽��������ֵ��ƽ��ֵ
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
	/* ע��̽����̽��RF�µ�λ�� */

	if (m_ToolToCalibrate.IsNull())
	{
		MITK_ERROR << "No tool to calibrate!";
	}

	// �� Eigen::Vector3d ת��Ϊ mitk::Point3D
	mitk::Point3D probeTip;
	probeTip[0] = m_ProbeTip[0];
	probeTip[1] = m_ProbeTip[1];
	probeTip[2] = m_ProbeTip[2];

	// ���¹��ߴ洢�еĹ�����Ϣ
	m_ToolToCalibrate->SetToolTipPosition(probeTip);

	// ��ӡ������Ϣ
	MITK_INFO << "Registered probe position: " << m_ProbeTip;
	MITK_INFO << "Tool tip position: " << m_ToolToCalibrate->GetToolTipPosition();

	//m_ToolToCalibrate->GetTrackingError();

}


double HTONDI::CalculateCalibrationError()
{
	/* ����̽��궨����� 
	  ��100����̬��ÿ����̬����ʵ�ʵ�̽��λ�ã�
	  ��������λ�õ�̽����λ�õ�������ĵ�λ��
	*/
	if (m_ProbePositions.empty())
	{
		MITK_ERROR << "No probe positions recorded!";
		return -1.0;
	}

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

	// ����ÿ���ɼ�����λ�ú����ռ���õ�������λ��֮������
	double sumSquaredError = 0.0;
	for (size_t i = 0; i < m_ProbePositions.size(); ++i)
	{
		double error = (m_Center - probeTipsInCamera[i]).norm();
		//cout << "Probe Tip in Camera: " << probeTipsInCamera[i].x() << ", " << probeTipsInCamera[i].y() << ", " << probeTipsInCamera[i].z() << endl;
		//cout << "current error " << i <<": "<< error << endl;
		sumSquaredError += error * error;
	}

	// �����������RMS��
	double rmsError = std::sqrt(sumSquaredError / m_ProbePositions.size());
	return rmsError;
}

// �ӿ���װ�ر궨����Ϣ
bool HTONDI::OnLoadDeviceLandmarkClicked()
{
	/* �ӿ���װ�ذھ��ĥ��ı궨����Ϣ
	1. װ�ذھ� + �ھ�궨��
	2. װ��ĥ�� + ĥ��궨��

	====== ����0823 ======
	����Ҫ��ע����棬ֱ������չʾ����ģ��������ע��
	*/
	m_Controls.textBrowser_Action->append("Action: Load Device Landmark Nodes.");

	saw_image = lancet::NavigationObject::New();
	drill_image = lancet::NavigationObject::New();

	// �ھ�Saw
	auto SawPoints = GetDataStorage()->GetNamedNode("SawLandMarkPointSet");
	if (SawPoints)
	{
		m_Controls.textBrowser_Action->append("load SawPoints.");
		SawPoints->SetVisibility(true);

		// ���� �ھ� ����Ŀ��ı궨��
		saw_image->SetLandmarks(dynamic_cast<mitk::PointSet*>(SawPoints->GetData()));

		// ������Ĳο��������Ϊ�궨�������ϵ
		//saw_image->SetReferencFrameName(SawPoints->GetName());
	}

	// ĥ��Drill
	auto DrillPoints = GetDataStorage()->GetNamedNode("DrillLandMarkPointSet");
	if (DrillPoints)
	{
		m_Controls.textBrowser_Action->append("load DrillPointSet.");
		DrillPoints->SetVisibility(true);

		// ���� ĥ�� ����Ŀ��ı궨��
		drill_image->SetLandmarks(dynamic_cast<mitk::PointSet*>(DrillPoints->GetData()));

		// ������Ĳο��������Ϊ�궨�������ϵ
		//drill_image->SetReferencFrameName(DrillPoints->GetName());
	}

	// ���ӻ���
	auto Saw = GetDataStorage()->GetNamedNode("Saw");
	auto Drill = GetDataStorage()->GetNamedNode("Drill");

	if (Saw)
	{
		m_Controls.textBrowser_Action->append("load Saw.");
		Saw->SetVisibility(true);

		// ���ðھ⵼��Ŀ��
		saw_image->SetDataNode(Saw);
	}
	if (Drill)
	{
		m_Controls.textBrowser_Action->append("load Drill.");
		Drill->SetVisibility(true);

		// ����ĥ�굼��Ŀ��
		drill_image->SetDataNode(Drill);
	}
	// Ȼ���������������޹�ģ�ͣ�

	cout << "test load model" << endl;

	GetDataStorage()->Modified();
	mitk::RenderingManager::GetInstance()->RequestUpdateAll();
	return true;
}

bool HTONDI::OnLoadBonePointsClicked()
{
	/* �ӿ���װ�عɹǺ��ֹǵ���׼����Ϣ�������������Ӿ��豸֮�����ִ��
	1. װ�عɹǴ���׼��
	2. װ���ֹǴ���׼��
	3. װ���ֹǾ���׼��

	=======�Ѽ���=======
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
	
	// ��Ҫ�ȳ�ʼ��
	femur_image = lancet::NavigationObject::New();
	tibia_image = lancet::NavigationObject::New();

	// װ��femur��landmark�㣬����ֻװ���������㣬��Ҫ�������ɹ�ͷ���Ĳ�װ��
	// �ɹ����׵� �ɹ����׵�
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
		
		// ȡ������
		mitk::DataNode::Pointer hipCenter = GetDataStorage()->GetNamedNode("hipCenterPoint");
		auto hipCenterPointSet = dynamic_cast<mitk::PointSet*>(hipCenter->GetData());
		mitk::Point3D hipCenterPoint = hipCenterPointSet->GetPoint(0);

		// ����landmark
		landmark->InsertPoint(hipCenterPoint);
	}

	GetDataStorage()->Modified();
	mitk::RenderingManager::GetInstance()->RequestUpdateAll();
	return true;
}


bool HTONDI::OnFemurCheckClicked()
{
	/* ��ǵ�ע��-�ɹǱ�ǵ�
	1. ̽��ɵ�
	2. ע���ǵ��ڹɹ�RF�µ����λ�� m_TibiaCheckPointOnTibiaRF

	====== ����0823 ======
	��ǵ���֤��Ŀ����ȷ���ο�RF��������
	*/

	// ע��ɹ�����֤���λ��
	// ���ɹ�RF��Ȼ��ʹ��̽��ɼ��ɹ��ϱ�ǵ�λ��
	m_Controls.textBrowser_Action->append("Action: Register Tibia Node.");

	auto probeIndex = m_VegaToolStorage->GetToolIndexByName("ProbeRF");
	auto femurRFIndex = m_VegaToolStorage->GetToolIndexByName("FemurRF");
	if (probeIndex == -1 || femurRFIndex == -1)
	{
		m_Controls.textBrowser_Action->append("There is no 'ProbeRF' or 'FemurRF' in the toolStorage!");
		return false;
	}

	// ��ȡ̽�����λ��
	auto toolProbe = m_VegaToolStorage->GetToolByName("ProbeRF");
	mitk::Point3D pos_TipOnProbeRF = toolProbe->GetToolTipPosition();

	// ת��Ϊ4d
	Eigen::Vector4d pos_TipOnProbeRF4d(pos_TipOnProbeRF[0], pos_TipOnProbeRF[1], pos_TipOnProbeRF[2], 1.0);

	// ȡ��̽������������µ�λ��
	// 4x4��α任����
	// T_FemurRF2ProbeRF = T_Camera2FemurRF.inverse() * T_Camera2ProbeRF
	Eigen::Matrix4d T_Camera2FemurRF = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d T_Camera2ProbeRF = Eigen::Matrix4d::Identity();

	mitk::NavigationData::Pointer pos_ProbeRFOnCamera = m_VegaSource->GetOutput(probeIndex);
	if (pos_ProbeRFOnCamera)
	{
		// 	����T_FemurRF2Camera
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

		// ��ӡ��Ϣ
		m_Controls.textBrowser_Action->append("Added pos_ProbeRFOnCamera: "
			+ QString::number(position[0]) +
			"/ " + QString::number(position[1]) +
			"/ " + QString::number(position[2])
		);

		// ����̽��Tip������µ�λ��
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
		// 	����T_FemurRF2Camera
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

	// ����̽������FemurRF�µ�����ֵ
	// Pos = T_Camera2FemurRF.inverse() * T_Camera2ProbeRF * pos_TipOnProbeRF
	Eigen::Vector4d pos_TipOnFemurRF4d = T_Camera2FemurRF.inverse() * T_Camera2ProbeRF * pos_TipOnProbeRF4d;
	// ���ע��
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
	/* ��ǵ�ע��-�ֹǱ�ǵ�
	1. ̽��ɵ�
	2. ע���ǵ��ڹɹ�RF�µ����λ�� m_TibiaCheckPointOnTibiaRF

	====== ����0823 ======
	��ǵ���֤��Ŀ����ȷ���ο�RF��������
	*/

	// ע��ɹ�����֤���λ��
	// ���ɹ�RF��Ȼ��ʹ��̽��ɼ��ɹ��ϱ�ǵ�λ��
	m_Controls.textBrowser_Action->append("Action: Register Tibia Node.");

	auto probeIndex = m_VegaToolStorage->GetToolIndexByName("ProbeRF");
	auto tibiaRFIndex = m_VegaToolStorage->GetToolIndexByName("TibiaRF");
	if (probeIndex == -1 || tibiaRFIndex == -1)
	{
		m_Controls.textBrowser_Action->append("There is no 'ProbeRF' or 'TibiaRF' in the toolStorage!");
		return false;
	}

	// ��ȡ̽�����λ��
	auto toolProbe = m_VegaToolStorage->GetToolByName("ProbeRF");
	mitk::Point3D pos_TipOnProbeRF = toolProbe->GetToolTipPosition();

	// ת��Ϊ4d
	Eigen::Vector4d pos_TipOnProbeRF4d(pos_TipOnProbeRF[0], pos_TipOnProbeRF[1], pos_TipOnProbeRF[2], 1.0);

	// ȡ��̽������������µ�λ��
	// 4x4��α任����
	// T_TibiaRF2ProbeRF = T_Camera2TibiaRF.inverse() * T_Camera2ProbeRF
	Eigen::Matrix4d T_Camera2TibiaRF = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d T_Camera2ProbeRF = Eigen::Matrix4d::Identity();

	mitk::NavigationData::Pointer pos_ProbeRFOnCamera = m_VegaSource->GetOutput(probeIndex);
	if (pos_ProbeRFOnCamera)
	{
		// 	����T_Camera2ProbeRF
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

		// ��ӡ��Ϣ
		m_Controls.textBrowser_Action->append("Added pos_ProbeRFOnCamera: "
			+ QString::number(position[0]) +
			"/ " + QString::number(position[1]) +
			"/ " + QString::number(position[2])
		);

		// ����̽��Tip������µ�λ��
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
		// 	����T_Camera2Tibia
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
	
	// ����̽������FemurRF�µ�����ֵ
	// Pos = T_Camera2TibiaRF.inverse() * T_Camera2ProbeRF * pos_TipOnProbeRF
	Eigen::Vector4d pos_TipOnTibiaRF4d = T_Camera2TibiaRF.inverse() * T_Camera2ProbeRF * pos_TipOnProbeRF4d;
	// ���ע��
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
	/* ��е�궨-�ھ�궨
	1. ��¼̽�����λ�ã�̽������λ��Ҳ�ʹ����˰ھ�궨���λ�ã�
	2. ��¼�ھ�궨��λ��[��������] 3����
	3. ����m_SawPointsOnSawRF = T_CameraToTool.inverse() * m_SawPointsOnCamera

	====== �޸�0821 ======
	����ֻ���㲢��¼�õ��ھ�궨���λ��m_SawPointsOnSawRF = Metrix[4,3]

	====== �޸�0823 ======
	  �޸ļ�������ģʽ

	====== �޸�0826 ======
	  �޸ı궨����
	*/

	m_Controls.textBrowser_Action->append("Action: Calibrate Saw Node.");

	cout << "test 01" << endl;

	// 1. ��¼̽�����λ�ã�̽������λ��Ҳ�ʹ����˰ھ�궨���λ��
	// ��ȡ̽��Ͱھ���Ϣ
	auto probeIndex = m_VegaToolStorage->GetToolIndexByName("ProbeRF");
	auto sawRfIndex = m_VegaToolStorage->GetToolIndexByName("SawRF");
	if (probeIndex == -1 || sawRfIndex == -1)
	{
		m_Controls.textBrowser_Action->append("There is no 'Probe' or 'SawRf' in the toolStorage!");
		return false;
	}

	

	// ���ݴ洢���ݵ���������ȡ��Ҫ��̽���˵��λ����Ϣ
	if (m_SawPointsOnSawRF.size() < 3)
	{
		m_Controls.textBrowser_Action->append("Get " + QString::number(m_SawPointsOnSawRF.size()) + "-th Saw Point");

		// ��ȡ ProbeRF �� NDI �е�λ�úͷ�������
		mitk::NavigationData::Pointer nd_ndiToProbe = m_VegaSource->GetOutput(probeIndex);

		// ��ȡ SawRF �� NDI �е�λ�úͷ�������
		mitk::NavigationData::Pointer nd_ndiToObjectRf = m_VegaSource->GetOutput(sawRfIndex);

		// �� ProbeTipOnCameraRF ת��Ϊ ProbeTipOnSawRF
		// Ҳ���ǽ��궨��� SawPointsOnCameraRF ת��Ϊ SawPointsOnSawRF
		mitk::NavigationData::Pointer nd_rfToProbe = GetNavigationDataInRef(nd_ndiToProbe, nd_ndiToObjectRf);

		// ��ȡת����ı궨����Ϣ
		mitk::Point3D probeTipPointUnderRf = nd_rfToProbe->GetPosition();

		// �洢��ȫ��λ�� m_SawPointsOnSawRF [X, Y, Z, 1.0]
		// ���� 3 ��
		Eigen::Vector4d CurrentSawPointsOnSawRF =
		{
			probeTipPointUnderRf[0],
			probeTipPointUnderRf[1],
			probeTipPointUnderRf[2],
			1.0
		};

		// ������õ���̽����λ�����ӵ� �㼯 ��
		m_SawPointsOnSawRF.push_back(CurrentSawPointsOnSawRF);

		// ���ڵ����ӵ��ھ��landmark_probe
		saw_image->GetLandmarks_probe()->InsertPoint(probeTipPointUnderRf);

		// ��ӡ��Ϣ
		m_Controls.textBrowser_Action->append("Added CurrentSawPointsOnSawRF: "
			+ QString::number(CurrentSawPointsOnSawRF[0]) +
			"/ " + QString::number(CurrentSawPointsOnSawRF[1]) +
			"/ " + QString::number(CurrentSawPointsOnSawRF[2])
		);
	}
	else if (m_SawPointsOnSawRF.size() == 3)
	{
		m_Controls.textBrowser_Action->append("Get Enough Saw Points.");

		// ����궨����: 1-2 1-3 2-3
		double caculateDistance[3] = {
			(m_SawPointsOnSawRF[0] - m_SawPointsOnSawRF[1]).norm(),
			(m_SawPointsOnSawRF[0] - m_SawPointsOnSawRF[2]).norm(),
			(m_SawPointsOnSawRF[1] - m_SawPointsOnSawRF[2]).norm(),
		};

		// �������
		double SawError01 = sqrt(
			  pow((caculateDistance[0] - m_SawDistance[0]), 2)
			+ pow((caculateDistance[1] - m_SawDistance[1]), 2)
			+ pow((caculateDistance[2] - m_SawDistance[2]), 2)) / 3;

		// ====== �����µ����Landmark ======

		// Calculate T_imageToObjectRf
		auto combinedRegistrator = mitk::SurfaceRegistration::New();
		combinedRegistrator->SetLandmarksSrc(saw_image->GetLandmarks());
		combinedRegistrator->SetLandmarksTarget(saw_image->GetLandmarks_probe());
		combinedRegistrator->SetSurfaceSrc(GetDataStorage()->GetNamedObject<mitk::Surface>("Saw"));
		combinedRegistrator->ComputeLandMarkResult();

		//m_ObjectRfToImageMatrix_hto->DeepCopy(combinedRegistrator->GetResult());
		double SawError02 = combinedRegistrator->GetavgLandmarkError();

		m_Controls.textBrowser_CalibrateRes->append("Current Saw Calibrate ERROR01: " + QString::number(SawError01));

		m_Controls.textBrowser_CalibrateRes->append("Current Saw Calibrate ERROR02: " + QString::number(SawError02));

		// ͬʱ���ƾ�̬�ع�ƽ��ͼ��
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


		// ���ɰھ�ع�ƽ��
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

		// �������
		auto realTimeSawPlane = mitk::Surface::New();
		realTimeSawPlane->SetVtkPolyData(realTimeSawPlaneSource->GetOutput());

		// ����ʵʱƽ��
		auto tmpPlane = GetDataStorage()->GetNamedNode("CurrentCutPlane01");
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
			planeNode->SetName("CurrentCutPlane01");

			// ���ӵ���
			GetDataStorage()->Add(planeNode);
			m_Controls.textBrowser_Action->append("Plane created.");
		}

		// Ȼ�󣬻��ƽع�ƽ����ǰ�˵�������
		auto tmpNodes = GetDataStorage()->GetNamedNode("pointSetInRealPlaneAxial");
		if (tmpNodes) {
			GetDataStorage()->Remove(tmpNodes);
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

			// ���������ӵ�mitk::PointSet��
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
			pointSetInPlaneCutPlane->SetColor(0.0, 1.0, 0.0);
			pointSetInPlaneCutPlane->SetData(mitkPointSetRealTime);
			pointSetInPlaneCutPlane->SetFloatProperty("pointsize", 3.0);
			GetDataStorage()->Add(pointSetInPlaneCutPlane);
		}
	}

	// �޸ı궨���Ϊlandmark���

	// �������
	return false;
}

bool HTONDI::OnCalibrateDrillClicked()
{
	/* ��е�궨-ĥ��궨
	1. ��¼̽�����λ�ã�̽������λ��Ҳ�ʹ�����ĥ��궨���λ�ã�
	2. ��¼�ھ�궨��λ��[��������] 3 ����
	3. ����m_DrillPointsOnDrillRF = T_CameraToTool.inverse() * m_DrillPointsOnCamera

	====== �޸�0821 ======
	����ֻ���㲢��¼�õ��ھ�궨���λ��m_DrillPointsOnDrillRF = Metrix[4,3]

	====== ע��0823 ======
	��Ҫ����ĥ��������㷽������ȷ��
	*/

	m_Controls.textBrowser_Action->append("Action: Calibrate Drill Node.");


	// 1. ��¼̽�����λ�ã�̽������λ��Ҳ�ʹ����˰ھ�궨���λ��
	// ��ȡ̽��Ͱھ���Ϣ
	auto probeIndex = m_VegaToolStorage->GetToolIndexByName("ProbeRF");
	auto DrillRfIndex = m_VegaToolStorage->GetToolIndexByName("DrillRF");
	if (probeIndex == -1 || DrillRfIndex == -1)
	{
		m_Controls.textBrowser_Action->append("There is no 'Probe' or 'DrillRF' in the toolStorage!");
		return false;
	}

	// ���ݴ洢���ݵ���������ȡ��Ҫ��̽���˵��λ����Ϣ
	if (m_DrillPointsOnDrillRF.size() < 3)
	{
		m_Controls.textBrowser_Action->append("Get " + QString::number(m_DrillPointsOnDrillRF.size()) + "-th Drill Point");

		// ��ȡ ProbeRF �� NDI �е�λ�úͷ�������
		mitk::NavigationData::Pointer nd_ndiToProbe = m_VegaSource->GetOutput(probeIndex);

		// ��ȡ SawRF �� NDI �е�λ�úͷ�������
		mitk::NavigationData::Pointer nd_ndiToObjectRf = m_VegaSource->GetOutput(DrillRfIndex);

		// �� ProbeTipOnCameraRF ת��Ϊ ProbeTipOnSawRF
		// Ҳ���ǽ��궨��� SawPointsOnCameraRF ת��Ϊ SawPointsOnSawRF
		mitk::NavigationData::Pointer nd_rfToProbe = GetNavigationDataInRef(nd_ndiToProbe, nd_ndiToObjectRf);

		// ��ȡת����ı궨����Ϣ
		mitk::Point3D probeTipPointUnderRf = nd_rfToProbe->GetPosition();

		// �洢��ȫ��λ�� m_SawPointsOnSawRF [X, Y, Z, 1.0]
		// ���� 3 ��
		Eigen::Vector4d CurrentDrillPointsOnSawRF =
		{
			probeTipPointUnderRf[0],
			probeTipPointUnderRf[1],
			probeTipPointUnderRf[2],
			1.0
		};

		// ������õ���̽����λ�����ӵ� �㼯 ��
		m_DrillPointsOnDrillRF.push_back(CurrentDrillPointsOnSawRF);

		// ��ӡ��Ϣ
		m_Controls.textBrowser_Action->append("Add CurrentDrillPointsOnSawRF: "
			+ QString::number(CurrentDrillPointsOnSawRF[0]) +
			"/ " + QString::number(CurrentDrillPointsOnSawRF[1]) +
			"/ " + QString::number(CurrentDrillPointsOnSawRF[2])
		);
	}
	else if (m_DrillPointsOnDrillRF.size() == 3)
	{
		m_Controls.textBrowser_Action->append("Get Enough Drill Points.");
		// ============== ��Ҫ���������㷽�� ================
		double DrillError = 0.0;
		// ==================================-================
		m_Controls.textBrowser_CalibrateRes->append("Current Saw Calibrate ERROR: " + QString::number(DrillError));
	}

	// �������
	return false;
}
bool HTONDI::OnSawVisualizeClicked()
{
	/* �ھ���ӻ� */
	m_Controls.textBrowser_Action->append("Action: Visualize Saw.");
	auto saw = GetDataStorage()->GetNamedNode("Saw");
	auto sawPoints = GetDataStorage()->GetNamedNode("SawLandMarkPointSet");
	if (saw && sawPoints)
	{
		// ��ȡ״̬
		bool currentVisibility = saw->IsVisible(nullptr, "visible", true);
		// չʾ�ھ�͵㼯��
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
	/* ĥ����ӻ� */
	m_Controls.textBrowser_Action->append("Action: Visualize Drill.");
	auto drill = GetDataStorage()->GetNamedNode("Drill");
	auto drillPoints = GetDataStorage()->GetNamedNode("DrillLandMarkPointSet");
	if (drill && drillPoints)
	{
		// ��ȡ״̬
		bool currentVisibility = drill->IsVisible(nullptr, "visible", true);
		// չʾ�ְ�͵㼯��
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
	/* ���ðھ�궨 */
	m_Controls.textBrowser_Action->append("Action: Reset Cali Saw.");
	m_SawPointsOnSawRF.clear();

	return true;
}

bool HTONDI::OnresetDrillCaliClicked()
{
	/* ���ðھ�궨 */
	m_Controls.textBrowser_Action->append("Action: Reset Cali Drill.");
	m_DrillPointsOnDrillRF.clear();

	return true;
}

// ����׼����
// �ɹǴ���׼
bool HTONDI::OnGetFemurLandmarkClicked()
{
	/* �ɹǵ��������׼��ɼ� = �ɹ����׵� �ɹ����׵� + �ɹ�ͷ����
	1. ����׼��ɼ�������ɼ�ǰ�����㣬�ɹ�ͷ���Ĳ�����ϵķ������
	
	��Ҫ̽�뼰�ɹǵ�RF: ProbeRF FemurRF

	====== �޸�0823 ======
	�ɹ�ʵ���ϲ���Ҫ��׼��ֱ��ȥ���㼸����ǵ��ڹɹ�����ϵ�µ�λ�ü���
	�����Ļ�������Ĵ���Ӧ�ú͹ɹǱ�ǵ�ע����һ����
	*/

	m_Controls.textBrowser_Action->append("Action: Get Femur Landmark Nodes.");

	// ȡ���Ѿ��õ���Target��׼�㼯
	auto pointSet_probeLandmark = femur_image->GetLandmarks_probe();
	auto pointSet_landmark = femur_image->GetLandmarks();

	
	// �ɹǴ���׼��Ĳɼ�ֻ��2����
	auto tmpNum = femur_image->GetLandmarks_probe()->GetSize();
	if (tmpNum >= 2)
	{
		m_Controls.textBrowser_Action->append("'Femur' landmark point done, please caculate hipcenter!");
		return false;
	}

	// ��ʼѡ��궨��
	// �ɹ����׵� -> �ɹ����׵�
	// Pos = T_Camera2TibiaRF.inverse() * T_Camera2ProbeRF * pos_TipOnProbeRF
	auto probeIndex = m_VegaToolStorage->GetToolIndexByName("ProbeRF");
	auto tibiaRFIndex = m_VegaToolStorage->GetToolIndexByName("FemurRF");
	if (probeIndex == -1 || tibiaRFIndex == -1)
	{
		m_Controls.textBrowser_Action->append("There is no 'ProbeRF' or 'FemurRF' in the toolStorage!");
		return false;
	}

	// ��ȡ̽�����λ��
	auto toolProbe = m_VegaToolStorage->GetToolByName("ProbeRF");
	mitk::Point3D pos_TipOnProbeRF = toolProbe->GetToolTipPosition();

	// ת��Ϊ4d
	Eigen::Vector4d pos_TipOnProbeRF4d(pos_TipOnProbeRF[0], pos_TipOnProbeRF[1], pos_TipOnProbeRF[2], 1.0);

	// ȡ��̽������������µ�λ��
	// 4x4��α任����
	// T_FemurRF2ProbeRF = T_Camera2FemurRF.inverse() * T_Camera2ProbeRF
	Eigen::Matrix4d T_Camera2FemurRF = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d T_Camera2ProbeRF = Eigen::Matrix4d::Identity();

	mitk::NavigationData::Pointer pos_ProbeRFOnCamera = m_VegaSource->GetOutput(probeIndex);
	if (pos_ProbeRFOnCamera)
	{
		// 	����T_FemurRF2Camera
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

	mitk::NavigationData::Pointer pos_FemurRFOnCamera = m_VegaSource->GetOutput(probeIndex);
	if (pos_FemurRFOnCamera)
	{
		// 	����T_FemurRF2Camera
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

	// ����̽������FemurRF�µ�����ֵ
	// Pos = T_Camera2FemurRF.inverse() * T_Camera2ProbeRF * pos_TipOnProbeRF
	Eigen::Vector4d pos_TipOnFemurRF4d = T_Camera2FemurRF.inverse() * T_Camera2ProbeRF * pos_TipOnProbeRF4d;
	// ���ӱ�ǵ�����
	mitk::Point3D CurrentFemurLandmarkPoint = (pos_TipOnFemurRF4d[0], pos_TipOnFemurRF4d[1], pos_TipOnFemurRF4d[2]);
	pointSet_landmark->InsertPoint(CurrentFemurLandmarkPoint);

	m_Controls.textBrowser_Action->append("Added CurrentFemurLandmarkPointOnFemurRF: ("
		+ QString::number(CurrentFemurLandmarkPoint[0]) +
		", " + QString::number(CurrentFemurLandmarkPoint[1]) +
		", " + QString::number(CurrentFemurLandmarkPoint[2]) + ")"
	);
	return true;
}

// ============================�ɹǴ���׼��ɼ�=====================================
// �ɹ����׵�=>�ɹ����׵�=>�ɹ�ͷ����(�Ѿ�����õ�) 

// �ζ����Ȼ�ùɹ�ͷ����
// �ɼ��ζ������е� 100 �������������
bool HTONDI::OnCollectBonePosClicked()
{
	/* �������Ȼ�ùɹ�ͷ����
	1. �ɼ����Ȼζ����̵�RF��̬
	2. ������ϣ��������/�ɹ�ͷ����
	*/

	m_Controls.textBrowser_Action->append("Action: Collect Femora Pos.");

	cout << "test 01" << endl;
	// ��ʼ�ɼ�������һ��ʱ���ڵĹɹ�λ����Ϣ
	m_FemurPositions.clear();
	m_Controls.textBrowser_Action->append("Start collect femora data, please move slowly...");

	cout << "test 02" << endl;
	// ������ʱ�������ڲɼ�����
	if (m_FemurDataCollectionTimer == nullptr)
	{
		cout << "test 03" << endl;
		m_FemurDataCollectionTimer = new QTimer(this);
		connect(m_FemurDataCollectionTimer, &QTimer::timeout, this, &HTONDI::CollectFemurData);
	}
	m_FemurDataCollectionTimer->start(100); // ÿ100ms�ɼ�һ������

	return true;
}

bool HTONDI::CollectFemurData()
{
	/* ��ȡ��ǰ�ɹ�׷������λ��
	1. �ɼ�100���ɹ���̬

	�ɹǲο�������������Ϊ��FemurRF
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
	��ϼ�����ζ��Ĺɹ�ͷ����
	*/

	m_Controls.textBrowser_Action->append("Action: Caculate FemoralHead Center.");

	if (m_FemurPositions.size() >= 100) // ȷ�����㹻�����ݵ�
	{
		CalculateHipCenter();
	}
	else
	{
		m_Controls.textBrowser_Action->append("Femora data < " + number);
	}

	return true;
}

// ��Ϻ���
bool HTONDI::CalculateHipCenter()
{
	/* �������
	1. 100����̬��ÿ10��һ���������
	2. ������ľ�ֵΪ�ɹ�ͷ����
	*/
	m_Controls.textBrowser_Action->append("Action: Collect Femora Pos.");

	// ����ֳ�10��һ��
	int groupSize = 10;
	int numGroups = m_FemurPositions.size() / groupSize;
	std::vector<Eigen::Vector3d> centers;

	for (int i = 0; i < numGroups; ++i) {
		std::vector<Eigen::Vector3d> subsetPoints(m_FemurPositions.begin() + i * groupSize, m_FemurPositions.begin() + (i + 1) * groupSize);
		auto [center, _] = FitSphere(subsetPoints);
		centers.push_back(center);
	}

	// �������ĵľ�ֵ
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

	// ����ɹ�ͷ���ĵ�λ��

	return true;
}



bool HTONDI::OnCaculateFemurLandmarkClicked()
{
	/* �ɹǵ��������׼���㼰Ӧ��
	1. ���ɹɹǴ���׼��
	2. ��׼����
	3. ����ɹǵ���׼����

	�ɹ�RF����ΪFemurRf
	
	====== �޸�0822 ======
	����Ҫ�ٰ��������

	====== �޸�0823 ======
	�������׼���㣬���ٽ���Ӧ��

	ʵ���ϣ��ֹǵĴ���׼�;���׼�õ�����׼����Ϳ���ֱ�ӽ��ɹǱ�������ͶӰ��
	���Ǵ������ȱҪ�������׼����ֻ�ñ���ɹǵ���׼����
	*/

	// ������õ��� �ɹ�ͷ���� Ҳ���ӵ� landmark �б� 
	m_Controls.textBrowser_Action->append("Action: Caculate Femur Landmark.");

	// ���ɹ�ͷ���ĵ����ӽ���
	// ȡ���Ѿ��õ���Target��׼�㼯
	auto pointSet_probeLandmark = femur_image->GetLandmarks_probe();

	// ������õ���̽����λ�����ӵ� �㼯 ��
	mitk::Point3D tmpHipCenter;
	tmpHipCenter[0] = m_HipCenter[0];
	tmpHipCenter[1] = m_HipCenter[1];
	tmpHipCenter[2] = m_HipCenter[2];
	pointSet_probeLandmark->SetPoint(2, tmpHipCenter);

	// ȡ�ô���׼�㼯 �� ��ǰ�Ѿ�ȡ�õĵ�
	int femurRF_SurfaceSrcNum = femur_image->GetLandmarks()->GetSize();
	int femurRF_SurfaceTargetNum = femur_image->GetLandmarks_probe()->GetSize();

	// �б�ɼ�����
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

	// ���¹ɹǴ�ʵ�ʿռ䵽ͼ��ռ�ı任���� m_
	femur_image->UpdateObjectToRfMatrix();

	// ��ӡ��ǰ��׼����Ľ��
	// lanmark
	m_Controls.textBrowser_FemurRes->append("Avg landmark error:" + QString::number(femur_image->GetlandmarkRegis_avgError()));
	m_Controls.textBrowser_FemurRes->append("Max landmark error:" + QString::number(femur_image->GetlandmarkRegis_maxError()));

	// ����ת������ m_MetrixFemurRFToImage
	// ��¼ת������
	m_MetrixFemurRFToImage = mitk::AffineTransform3D::New();

	// ������ͼ��Ķ��󵽲ο���ܵı任�����VTK��ʽת��ΪITK��ʽ�����洢
	mitk::TransferVtkMatrixToItkTransform(femur_image->GetT_Object2ReferenceFrame(), m_MetrixFemurRFToImage.GetPointer());

	// ����׼����Ӧ�õ� FemurRF ������
	m_VegaToolStorage->GetToolByName("FemurRF")->SetToolRegistrationMatrix(m_MetrixFemurRFToImage);

	// ��ȡ����
	// ��ȡ����任����
	mitk::AffineTransform3D::MatrixType matrix = m_MetrixFemurRFToImage->GetMatrix();


	// �� mitk::AffineTransform3D �ľ���ת��Ϊ Eigen::Matrix4d
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			m_Metrix4dFemurRFToImage(i, j) = matrix(i, j);
		}
	}

	return true;
}


bool HTONDI::OnGetTibiaLandmarkClicked()
{
	/* �ֹǱ������׼��ɼ����ֹǽ������� �ֹǽ����ڲ�� �ֹ�Զ�����׵� �ֹ�Զ�����׵�
	1. ����˳����д���׼��Ĳɼ�
	2. �б����ӵ������

	��Ҫ����ǰ�滮������ɼ��ķ���һ��
	====== ����0826 ======
	�޸����ݻ�ȡ����
	====== �޸�0828 ======
	Ŀǰ���� OnCollectHTOTibiaLandmarkClicked() �����滻
	*/
	m_Controls.textBrowser_Action->append("Action: Get Tibia Landmark.");

	// ��ȡ�ɹǵı�������
	auto TibiaRF_Surface = GetDataStorage()->GetNamedNode("tibiaSurface");

	if (TibiaRF_Surface == nullptr)
	{
		m_Controls.textBrowser_Action->append("ERROR: Tibia model Not Found!");
		return false;
	}

	if (m_VegaToolStorage == nullptr)
	{
		m_Controls.textBrowser->append("NDI hasn't been set up yet!");
		return false;
	}

	// ȡ���Ѿ��õ���Target��׼�㼯
	auto pointSet_landmark = tibia_image->GetLandmarks();
	auto pointSet_probeLandmark = tibia_image->GetLandmarks_probe();


	if (pointSet_probeLandmark->GetSize() >= pointSet_landmark->GetSize())
	{
		m_Controls.textBrowser_Action->append("Tibia landmark points is enough!");
		return false;
	}


	// ��ʼ������׼��ɼ�
	// Pos_Tip = T_Camera2ProbeRF * pos_TipOnProbeRF
	auto probeIndex = m_VegaToolStorage->GetToolIndexByName("ProbeRF");
	auto tibiaRFIndex = m_VegaToolStorage->GetToolIndexByName("TibiaRF");
	if (probeIndex == -1 || tibiaRFIndex == -1)
	{
		m_Controls.textBrowser_Action->append("There is no 'ProbeRF' or 'TibiaRF' in the toolStorage!");
		return false;
	}
	
	// Check the availability of the optic tools in the FOV
	mitk::NavigationData::Pointer nd_ndiToProbe = m_VegaSource->GetOutput(probeIndex);
	mitk::NavigationData::Pointer nd_ndiToObjectRf = m_VegaSource->GetOutput(tibiaRFIndex);

	if (nd_ndiToObjectRf->IsDataValid() == 0 || nd_ndiToProbe->IsDataValid() == 0)
	{
		m_Controls.textBrowser_Action->append("Can not get data!");
		return false;
	}

	// Update m_Pset_thaPosHorn_rf
	mitk::NavigationData::Pointer nd_rfToProbe = GetNavigationDataInRef(nd_ndiToProbe, nd_ndiToObjectRf);

	mitk::Point3D probeTipPointUnderRf = nd_rfToProbe->GetPosition();

	pointSet_probeLandmark->InsertPoint(probeTipPointUnderRf);



	// ���̽��λ��
	mitk::NavigationData::Pointer probe = m_VegaSource->GetOutput(probeIndex);
	mitk::Point3D pos_tip = probe->GetPosition();

	m_Controls.textBrowser_Action->append("Added CurrentProbeRF: ("
		     + QString::number(probeTipPointUnderRf[0]) +
		", " + QString::number(probeTipPointUnderRf[1]) +
		", " + QString::number(probeTipPointUnderRf[2]) + ")"
	);

	m_Controls.textBrowser_Action->append("Added CurrentTibiaLandmarkPoint: ("
		     + QString::number(pos_tip[0]) +
		", " + QString::number(pos_tip[1]) +
		", " + QString::number(pos_tip[2]) + ")"
	);
	return true;
}

bool HTONDI::OnCaculateTibiaLandmarlClicked()
{	
	/* �ֹǵ��ĵ����׼���㼰Ӧ��
	1. ����׼����
	2. ����׼Ӧ��

	====����֤0813====


	====== �޸�0827 ======
	ֱ�Ӽ�����׼���󣬲��ٽ���ͼ�����

	====== �޸�0828 ======
	Ŀǰ���� OnCollectHTOTibiaRegisClicked ��������
	*/
	m_Controls.textBrowser_Action->append("Action: Caculate Tibia Landmark.");

	// ȡ���Ѿ��õ���Target��׼�㼯
	auto pointSet_probeLandmark = tibia_image->GetLandmarks_probe();

	// ȡ�ô���׼�㼯 �� ��ǰ�Ѿ�ȡ�õĵ�
	int tibiaRF_SurfaceSrcNum = tibia_image->GetLandmarks()->GetSize();
	int tibiaRF_SurfaceTargetNum = tibia_image->GetLandmarks_probe()->GetSize();

	// �б�ɼ�����
	if (tibiaRF_SurfaceSrcNum == tibiaRF_SurfaceTargetNum)
	{
		m_Controls.textBrowser_Action->append("--- Enough tibia landmarks have been collected ----");
	}
	else
	{
		m_Controls.textBrowser_Action->append("--- Not Enough tibia landmarks have been collected ----");
		return true;
	}

	
	/*
	// �����׼���㣬����ʼ��ʾ̽���ʵʱλ��
	// ===== 0827 ======

	// ���µ���ͼ��Ķ��󵽲ο���ܵı任����
	auto landmarkRegistrator = mitk::SurfaceRegistration::New();
	landmarkRegistrator->SetLandmarksSrc(tibia_image->GetLandmarks_probe());
	landmarkRegistrator->SetLandmarksTarget(tibia_image->GetLandmarks());
	landmarkRegistrator->ComputeLandMarkResult();

	// ������׼����
	m_ObjectRfToImageMatrix_hto->DeepCopy(landmarkRegistrator->GetResult());

	// ��ӡ��ǰ��׼����Ľ��
	m_Controls.textBrowser_TibieRes->append("Avg landmark error:" + QString::number(landmarkRegistrator->GetavgLandmarkError()));
	m_Controls.textBrowser_TibieRes->append("Max landmark error:" + QString::number(landmarkRegistrator->GetmaxLandmarkError()));
*/


	// ��ʼ��׼����
	// ����һ���µľ�̬ͼ�������׼������ʵ��
	m_surfaceRegistrationStaticImageFilter = lancet::ApplySurfaceRegistratioinStaticImageFilter::New();

	// �����������ӵ�Vega��������Դ
	m_surfaceRegistrationStaticImageFilter->ConnectTo(m_VegaSource);

	// ����һ���µķ���任�������ڴ洢��׼���
	m_MetrixTibiaRFToImage = mitk::AffineTransform3D::New();

	// ���µ���ͼ��Ķ��󵽲ο���ܵı任����
	tibia_image->UpdateObjectToRfMatrix();

	// ��ӡ��ǰ��׼����Ľ��
	// lanmark
	m_Controls.textBrowser_TibieRes->append("Avg landmark error:" + QString::number(tibia_image->GetlandmarkRegis_avgError()));
	m_Controls.textBrowser_TibieRes->append("Max landmark error:" + QString::number(tibia_image->GetlandmarkRegis_maxError()));

	// ������ͼ��Ķ��󵽲ο���ܵı任�����VTK��ʽת��ΪITK��ʽ�����洢
	mitk::TransferVtkMatrixToItkTransform(tibia_image->GetT_Object2ReferenceFrame(), m_MetrixTibiaRFToImage.GetPointer());

	// Ӱ����׼�������Ź����ο������ڲ��ϱ仯 
	// T_C2M = T_C2TibiaRF * T_TibiaRF2M

	// ����׼����Ӧ�õ� BoneRF ������
	m_VegaToolStorage->GetToolByName("TibiaRF")->SetToolRegistrationMatrix(m_MetrixTibiaRFToImage);

	// ��ȡ����任����
	mitk::AffineTransform3D::MatrixType matrix = m_MetrixTibiaRFToImage->GetMatrix();

	// �� mitk::AffineTransform3D �ľ���ת��Ϊ Eigen::Matrix4d
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			m_Metrix4dTibiaRFToImage(i, j) = matrix(i, j);
		}
	}

	// ����׼�������õ���̬ͼ�������׼��������
	m_surfaceRegistrationStaticImageFilter->SetRegistrationMatrix(m_VegaToolStorage->GetToolByName("TibiaRF")->GetToolRegistrationMatrix());

	// ���òο���ܵĵ�������
	m_surfaceRegistrationStaticImageFilter->SetNavigationDataOfRF(m_VegaSource->GetOutput("TibiaRF"));

	// ֹͣVega���ӻ���ʱ��
	m_VegaVisualizeTimer->stop();

	// ��Vega���ӻ������ӵ���̬ͼ�������׼������
	m_VegaVisualizer->ConnectTo(m_surfaceRegistrationStaticImageFilter);

	// ��������Vega���ӻ���ʱ������ʾ��׼����Ľ��
	m_VegaVisualizeTimer->start();
	

	// ����̽��λ��
	if (m_HTOPrboeUpdateTimer == nullptr)
	{
		m_HTOPrboeUpdateTimer = new QTimer(this); //create a new timer
	}
	//cout << "test disconnect" << endl;
	disconnect(m_HTOPrboeUpdateTimer, SIGNAL(timeout()), this, SLOT(UpdateHTOProbe()));
	//cout << "test connect" << endl;
	connect(m_HTOPrboeUpdateTimer, SIGNAL(timeout()), this, SLOT(UpdateHTOProbe()));

	m_HTOPrboeUpdateTimer->start(100);


	return true;
}

bool HTONDI::OnGetTibieICPClicked()
{
	/* �ֹǱ��澫��׼��ɼ�
	1. ����ɵ�

	=========����֤0814===========

	====== �޸�0828 ======
	Ŀǰ���� OnCollectHTOTibiaICPClicked �����滻
	*/
	m_Controls.textBrowser_Action->append("Action: Get Tibie ICP.");

	// ���ȼ���ֹ��Ƿ��Ѿ���ʼ��
	if (tibia_image == nullptr)
	{
		m_Controls.textBrowser_Action->append("Please setup the tibia_image first!");
		return false;
	}

	// ��ȡICP(���������)�㼯
	auto pointSet_probeIcp = tibia_image->GetIcpPoints_probe();

	// ��ȡ̽��Ͳο�����(����)������
	auto probeIndex = m_VegaToolStorage->GetToolIndexByName("ProbeRF");
	auto objectRfIndex = m_VegaToolStorage->GetToolIndexByName("TibiaRF");

	// ��鹤���Ƿ����
	if (probeIndex == -1 || objectRfIndex == -1)
	{
		m_Controls.textBrowser_Action->append("There is no 'Probe' or 'TibiaRF' in the toolStorage!");
	}

	// ��ȡ̽��Ͳο�������NDI����ϵ�еĵ�������
	mitk::NavigationData::Pointer nd_ndiToProbe = m_VegaSource->GetOutput(probeIndex);
	mitk::NavigationData::Pointer nd_ndiToObjectRf = m_VegaSource->GetOutput(objectRfIndex);

	// ��̽��ĵ�������ת�����ο����������ϵ��
	mitk::NavigationData::Pointer nd_rfToProbe = GetNavigationDataInRef(nd_ndiToProbe, nd_ndiToObjectRf);

	// ��ȡ̽�����ڲο���������ϵ�е�λ��
	mitk::Point3D probeTipPointUnderRf = nd_rfToProbe->GetPosition();

	// ����������ӵ�ICP�㼯��
	pointSet_probeIcp->InsertPoint(probeTipPointUnderRf);

	// ���û���������ʾ���ӵĵ������
	m_Controls.textBrowser_TibieRes->append("Added icp point: " + QString::number(probeTipPointUnderRf[0]) +
		"/ " + QString::number(probeTipPointUnderRf[1]) + "/ " + QString::number(probeTipPointUnderRf[2]));

	return true;
}
bool HTONDI::OnCaculateTibiaICPClicked()
{
	/* �ֹǵľ���׼���㼰��׼�ں�Ӧ��
	1. ����׼����
	2. ����׼�����׼���ں�Ӧ��
	3. ���������

	====����֤0814====

	====== �޸�0827 ======
	�޸���׼������߼�

	====== �޸�0828 ======
	Ŀǰ�Ѿ��� OnCollectHTOTibiaRegisClicked �滻
	*/
	m_Controls.textBrowser_Action->append("Action: Caculate Tibia ICP.");

	// ��������׼���
	// ����һ���µľ�̬ͼ�������׼������ʵ��
	m_surfaceRegistrationStaticImageFilter = lancet::ApplySurfaceRegistratioinStaticImageFilter::New();
	// �����������ӵ�Vega��������Դ
	m_surfaceRegistrationStaticImageFilter->ConnectTo(m_VegaSource);
	// ����һ���µķ���任�������ڴ洢��׼���
	m_imageRegistrationMatrix = mitk::AffineTransform3D::New();
	// ���µ���ͼ��Ķ��󵽲ο���ܵı任����
	tibia_image->UpdateObjectToRfMatrix();

	// ��ӡ��ǰ��׼����Ľ��
	// lanmark
	m_Controls.textBrowser_TibieRes->append("Avg landmark error:" + QString::number(tibia_image->GetlandmarkRegis_avgError()));
	m_Controls.textBrowser_TibieRes->append("Max landmark error:" + QString::number(tibia_image->GetlandmarkRegis_maxError()));
	// icp
	m_Controls.textBrowser_TibieRes->append("Avg ICP error:" + QString::number(tibia_image->GetIcpRegis_avgError()));
	m_Controls.textBrowser_TibieRes->append("Max ICP error:" + QString::number(tibia_image->GetIcpRegis_maxError()));

	// ��ȡ��׼����
	// ����һ���µķ���任�������ڴ洢��׼���
	m_MetrixTibiaRFToImage = mitk::AffineTransform3D::New();

	// ������ͼ��Ķ��󵽲ο���ܵı任�����VTK��ʽת��ΪITK��ʽ�����洢
	mitk::TransferVtkMatrixToItkTransform(tibia_image->GetT_Object2ReferenceFrame(), m_MetrixTibiaRFToImage.GetPointer());

	// ��ȡ����任����
	mitk::AffineTransform3D::MatrixType matrix = m_MetrixTibiaRFToImage->GetMatrix();

	// �� mitk::AffineTransform3D �ľ���ת��Ϊ Eigen::Matrix4d
	Eigen::Matrix4d m_Metrix4dTibiaRFToImage_icp;
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			m_Metrix4dTibiaRFToImage_icp(i, j) = matrix(i, j);
		}
	}
	m_Metrix4dTibiaRFToImage = m_Metrix4dTibiaRFToImage_icp * m_Metrix4dTibiaRFToImage_icp;


	// ����׼����Ӧ�õ� BoneRF ������
	m_VegaToolStorage->GetToolByName("TibiaRF")->SetToolRegistrationMatrix(m_MetrixTibiaRFToImage);
	// ����׼�������õ���̬ͼ�������׼��������
	m_surfaceRegistrationStaticImageFilter->SetRegistrationMatrix(m_VegaToolStorage->GetToolByName("TibiaRF")->GetToolRegistrationMatrix());
	// ���òο���ܵĵ�������
	m_surfaceRegistrationStaticImageFilter->SetNavigationDataOfRF(m_VegaSource->GetOutput("TibiaRF"));
	// ֹͣVega���ӻ���ʱ��
	m_VegaVisualizeTimer->stop();
	// ��Vega���ӻ������ӵ���̬ͼ�������׼������
	m_VegaVisualizer->ConnectTo(m_surfaceRegistrationStaticImageFilter);
	// ��������Vega���ӻ���ʱ������ʾ��׼����Ľ��
	m_VegaVisualizeTimer->start();



	// ����̽�붯������
	if (m_HTOPrboeUpdateTimer == nullptr)
	{
		m_HTOPrboeUpdateTimer = new QTimer(this); //create a new timer
	}
	disconnect(m_HTOPrboeUpdateTimer, SIGNAL(timeout()), this, SLOT(UpdateTHOProbe()));

	connect(m_HTOPrboeUpdateTimer, SIGNAL(timeout()), this, SLOT(UpdateTHOProbe()));

	m_HTOPrboeUpdateTimer->start(100);


/*
	// ����һ���µľ�̬ͼ�������׼������ʵ��
	m_surfaceRegistrationStaticImageFilter = lancet::ApplySurfaceRegistratioinStaticImageFilter::New();
	// �����������ӵ�Vega��������Դ
	m_surfaceRegistrationStaticImageFilter->ConnectTo(m_VegaSource);
	// ����һ���µķ���任�������ڴ洢��׼���
	m_imageRegistrationMatrix = mitk::AffineTransform3D::New();
	// ���µ���ͼ��Ķ��󵽲ο���ܵı任����
	tibia_image->UpdateObjectToRfMatrix();

	// ��ӡ��ǰ��׼����Ľ��
	// lanmark
	m_Controls.textBrowser_TibieRes->append("Avg landmark error:" + QString::number(tibia_image->GetlandmarkRegis_avgError()));
	m_Controls.textBrowser_TibieRes->append("Max landmark error:" + QString::number(tibia_image->GetlandmarkRegis_maxError()));
	// icp
	m_Controls.textBrowser_TibieRes->append("Avg ICP error:" + QString::number(tibia_image->GetIcpRegis_avgError()));
	m_Controls.textBrowser_TibieRes->append("Max ICP error:" + QString::number(tibia_image->GetIcpRegis_maxError()));

	// ������ͼ��Ķ��󵽲ο���ܵı任�����VTK��ʽת��ΪITK��ʽ�����洢
	mitk::TransferVtkMatrixToItkTransform(tibia_image->GetT_Object2ReferenceFrame(), m_imageRegistrationMatrix.GetPointer());
	// ����׼����Ӧ�õ� BoneRF ������
	m_VegaToolStorage->GetToolByName("TibiaRF")->SetToolRegistrationMatrix(m_imageRegistrationMatrix);
	// ����׼�������õ���̬ͼ�������׼��������
	m_surfaceRegistrationStaticImageFilter->SetRegistrationMatrix(m_VegaToolStorage->GetToolByName("TibiaRF")->GetToolRegistrationMatrix());
	// ���òο���ܵĵ�������
	m_surfaceRegistrationStaticImageFilter->SetNavigationDataOfRF(m_VegaSource->GetOutput("TibiaRF"));
	// ֹͣVega���ӻ���ʱ��
	m_VegaVisualizeTimer->stop();
	// ��Vega���ӻ������ӵ���̬ͼ�������׼������
	m_VegaVisualizer->ConnectTo(m_surfaceRegistrationStaticImageFilter);
	// ��������Vega���ӻ���ʱ������ʾ��׼����Ľ��
	m_VegaVisualizeTimer->start();
*/




	// ������׼���� m_PreviousImageRegistrationMatrix
	// m_PreviousImageRegistrationMatrix = m_imageRegistrationMatrix;

	// ���������
	// ��ȡ��ǰ�ֹ�RF��λ��
	auto TibiaRF = m_VegaToolStorage->GetToolByName("TibiaRF");

	// ��ȡ�ֹǵı�������
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

	// ���������
	TibiaRF->SetDataNode(TibiaRF_Surface);
	TibiaRF_Surface->SetVisibility(true);



	return true;
}


void HTONDI::UpdateHTOProbe()
{
	cout << "test probe 01" << endl;
	if (GetDataStorage()->GetNamedNode("Probe_forVisual") == nullptr)
	{
		cout << "test 02-No Probe_forVisual" << endl;
		return;
	}
	auto probeIndex = m_VegaToolStorage->GetToolIndexByName("ProbeRF");
	auto objectRfIndex = m_VegaToolStorage->GetToolIndexByName("TibiaRF");

	if (probeIndex == -1 || objectRfIndex == -1)
	{
		m_Controls.textBrowser_Action->append("There is no 'ProbeRF' or 'TibiaRF' in the toolStorage!");
		return;
	}

	cout << "test probe 03" << endl;

	// Check the availability of the optic tools in the FOV
	mitk::NavigationData::Pointer nd_ndiToProbe = m_VegaSource->GetOutput(probeIndex);
	mitk::NavigationData::Pointer nd_ndiToObjectRf = m_VegaSource->GetOutput(objectRfIndex);

	if (nd_ndiToObjectRf->IsDataValid() == 0 || nd_ndiToProbe->IsDataValid() == 0)
	{
		return;
	}

	cout << "test probe 04" << endl;
	mitk::NavigationData::Pointer nd_rfToProbe = GetNavigationDataInRef(nd_ndiToProbe, nd_ndiToObjectRf);

	cout << "test probe 05" << endl;
	vtkNew<vtkMatrix4x4> vtkMatrix_rfToProbe;
	mitk::TransferItkTransformToVtkMatrix(nd_rfToProbe->GetAffineTransform3D().GetPointer(), vtkMatrix_rfToProbe);

	cout << "test probe 06" << endl;
	vtkNew<vtkMatrix4x4> imageToRfMatrix;
	imageToRfMatrix->DeepCopy(m_ObjectRfToImageMatrix_hto);
	imageToRfMatrix->Invert();

	cout << "test probe 07" << endl;
	vtkNew<vtkTransform> tmpTrans;
	tmpTrans->PostMultiply();
	tmpTrans->SetMatrix(vtkMatrix_rfToProbe);
	tmpTrans->Concatenate(imageToRfMatrix);
	tmpTrans->Update();

	cout << "test probe 08" << endl;
	GetDataStorage()->GetNamedNode("Probe_forVisual")->GetData()->GetGeometry()->SetIndexToWorldTransformByVtkMatrix(tmpTrans->GetMatrix());
	GetDataStorage()->GetNamedNode("Probe_forVisual")->GetData()->GetGeometry()->Modified();

	cout << "Current probe visulizetion done" << endl;
}


void HTONDI::OnInitHTOTibiaRegisClicked()
{
	/* ��ʼ���ֹ���׼ */

	m_Controls.textBrowser_Action->append("Action: Tibia Regis init.");

	// �ֹǽ������� �ֹǽ����ڲ�� �ֹ�Զ�����׵� �ֹ�Զ�����׵�
	m_tibiaProximalLateralPoint = mitk::PointSet::New();
	m_tibiaProximalMedialPoint = mitk::PointSet::New();
	m_tibiaDistalLateralPoint = mitk::PointSet::New();
	m_tibiaDistalMedialPoint = mitk::PointSet::New();
	// �ֹǴ���׼��
	m_tibiaLandmarkPoints = mitk::PointSet::New();
	// �ֹǾ���׼��
	m_tibiaICPPoints = mitk::PointSet::New();

	// �ֹǾ���׼��
	m_Surface_HTOTibiaSurface = mitk::Surface::New();

	// ��ʼ����׼����
	m_ObjectRfToImageMatrix_hto->Identity();

	// ��ʼ������
	m_Num_TibiaLandmark = 0;

	// Set up the surface and the image landmark
	if (GetDataStorage()->GetNamedNode("tibiaSurface") == nullptr ||
		GetDataStorage()->GetNamedNode("tibiaLandmarkPointSet") == nullptr)
	{
		m_Controls.textBrowser_Action->append("tibiaSurface or tibiaLandmarkPointSet is missing");
		return;
	}

	

	// Turn off the visibility of all nodes but for the boneSurface and landmark_image
	mitk::DataStorage::SetOfObjects::ConstPointer allNodes = GetDataStorage()->GetAll();

	for (auto it = allNodes->Begin(); it != allNodes->End(); ++it)
	{
		mitk::DataNode::Pointer node = it->Value();
		node->SetVisibility(false);
	}

	// visuliaze tibiaSurface and tibiaLandmarkPointSet
	GetDataStorage()->GetNamedNode("tibiaSurface")->SetVisibility(true);
	GetDataStorage()->GetNamedNode("tibiaLandmarkPointSet")->SetVisibility(true);

	m_Surface_HTOTibiaSurface = GetDataStorage()->GetNamedObject<mitk::Surface>("tibiaSurface");
	m_tibiaLandmarkPoints = GetDataStorage()->GetNamedObject<mitk::PointSet>("tibiaLandmarkPointSet");

	// ��ʾ��һ����
	auto tibiaPoints = GetDataStorage()->GetNamedNode("tibiaLandmarkPointSet");

	// ���ص�
	auto currentPoint = dynamic_cast<mitk::PointSet*>(tibiaPoints->GetData())->GetPoint(0);
	mitk::PointSet::Pointer currentPointSet = mitk::PointSet::New();
	currentPointSet->InsertPoint(currentPoint);

	// �����ϴεĵ����
	auto currentLandmark = GetDataStorage()->GetNamedNode("currentLandmarkPoint");
	if (currentLandmark)
	{
		GetDataStorage()->Remove(currentLandmark);
	}

	// �������ӻ�����
	mitk::DataNode::Pointer currentPointSurface = mitk::DataNode::New();
	// װ������
	currentPointSurface->SetData(currentPointSet);
	currentPointSurface->SetName("currentLandmarkPoint");
	currentPointSurface->SetProperty("color", mitk::ColorProperty::New(0.0, 0.0, 1.0)); // ������ɫΪ��ɫ
	currentPointSurface->SetProperty("pointsize", mitk::FloatProperty::New(10.0)); // ���õ�Ĵ�СΪ10

	// 
	// ֹͣ׷��
	if (m_HTOPrboeUpdateTimer != nullptr)
	{
		disconnect(m_HTOPrboeUpdateTimer, SIGNAL(timeout()), this, SLOT(UpdateHTOProbe()));
		m_HTOPrboeUpdateTimer->stop();
		m_HTOPrboeUpdateTimer = { nullptr };
	}

	GetDataStorage()->Add(currentPointSurface);

	m_Controls.textBrowser_TibieRes->append("tibiaRigis init");
	
}

void HTONDI::OnInitHTOFemurRegisClicked()
{
	/* ��ʼ���ֹ���׼ */

	// �ɹ���׼��
	m_tibiaProximalLateralPoint = mitk::PointSet::New();
	m_tibiaProximalMedialPoint = mitk::PointSet::New();
	m_tibiaDistalLateralPoint = mitk::PointSet::New();
	m_tibiaDistalMedialPoint = mitk::PointSet::New();
	// �ɹǴ���׼��
	m_femurLandmarkPoints = mitk::PointSet::New();

	// �ֹǾ���׼��
	m_Surface_HTOFemurSurface = mitk::Surface::New();

	// ��ʼ����׼����
	m_ObjectRfToImageMatrix_hto->Identity();

	// ��ʼ�����
	m_Num_FemurLandmark = 0;

	// Set up the surface and the image landmark
	if (GetDataStorage()->GetNamedNode("femurSurface") == nullptr ||
		GetDataStorage()->GetNamedNode("femurLandmarkPointSet") == nullptr)
	{
		m_Controls.textBrowser_Action->append("femurSurface or femurLandmarkPointSet is missing");
		return;
	}

	// Turn off the visibility of all nodes but for the boneSurface and landmark_image
	mitk::DataStorage::SetOfObjects::ConstPointer allNodes = GetDataStorage()->GetAll();

	// Iterate over all nodes
	for (auto it = allNodes->Begin(); it != allNodes->End(); ++it)
	{
		mitk::DataNode::Pointer node = it->Value();

		// Set the visibility property to false for each node
		node->SetVisibility(false);
	}

	GetDataStorage()->GetNamedNode("femurSurface")->SetVisibility(true);

	m_Surface_HTOFemurSurface = GetDataStorage()->GetNamedObject<mitk::Surface>("femurSurface");
	m_femurLandmarkPoints = GetDataStorage()->GetNamedObject<mitk::PointSet>("femurLandmarkPointSet");

	//ThaSwitchStateMachine();
}


void HTONDI::CollectHTOTibiaLandmark(int index)
{
	m_Controls.textBrowser_Action->append("Action: Collect Tibia Landmark, index: " + QString::number(index));

	// Check the camera prerequisites: connection & tool storage 
	if (m_VegaToolStorage == nullptr)
	{
		m_Controls.textBrowser->append("NDI hasn't been set up yet!");
		return;
	}

	auto probeRFIndex = m_VegaToolStorage->GetToolIndexByName("ProbeRF");
	auto TibiaRFIndex = m_VegaToolStorage->GetToolIndexByName("TibiaRF");

	if (probeRFIndex == -1 || TibiaRFIndex == -1)
	{
		m_Controls.textBrowser_Action->append("There is no 'ProbeRF' or 'TibiaRF' in the toolStorage!");
		return;
	}

	// Check the availability of the optic tools in the FOV
	mitk::NavigationData::Pointer nd_ndiToProbe = m_VegaSource->GetOutput(probeRFIndex);
	mitk::NavigationData::Pointer nd_ndiToObjectRf = m_VegaSource->GetOutput(TibiaRFIndex);

	if (nd_ndiToObjectRf->IsDataValid() == 0 || nd_ndiToProbe->IsDataValid() == 0)
	{
		return;
	}

	// Update m_Pset_thaCrest_rf
	mitk::NavigationData::Pointer nd_rfToProbe = GetNavigationDataInRef(nd_ndiToProbe, nd_ndiToObjectRf);

	mitk::Point3D probeTipPointUnderRf = nd_rfToProbe->GetPosition();

	

	// �����ֹǽڵ�ɼ�
	// �ֹǽ������� �ֹǽ����ڲ�� �ֹ�Զ�����׵� �ֹ�Զ�����׵�
	if (index == 0)
	{
		m_tibiaProximalLateralPoint->InsertPoint(probeTipPointUnderRf);
		m_Controls.textBrowser_TibieRes->append("Added m_tibiaProximalLateralPoint: ("
			     + QString::number(probeTipPointUnderRf[0]) +
			", " + QString::number(probeTipPointUnderRf[1]) +
			", " + QString::number(probeTipPointUnderRf[2]) + ")"
		);
	}
	else if (index == 1)
	{
		m_tibiaProximalMedialPoint->InsertPoint(probeTipPointUnderRf);
		m_Controls.textBrowser_TibieRes->append("Added m_tibiaProximalMedialPoint: ("
			     + QString::number(probeTipPointUnderRf[0]) +
			", " + QString::number(probeTipPointUnderRf[1]) +
			", " + QString::number(probeTipPointUnderRf[2]) + ")"
		);
	}
	else if (index == 2)
	{
		m_tibiaDistalLateralPoint->InsertPoint(probeTipPointUnderRf);
		m_Controls.textBrowser_TibieRes->append("Added m_tibiaDistalLateralPoint: ("
			     + QString::number(probeTipPointUnderRf[0]) +
			", " + QString::number(probeTipPointUnderRf[1]) +
			", " + QString::number(probeTipPointUnderRf[2]) + ")"
		);
	}
	else if (index == 3)
	{
		m_tibiaDistalMedialPoint->InsertPoint(probeTipPointUnderRf);
		m_Controls.textBrowser_TibieRes->append("Added m_tibiaDistalMedialPoint: ("
			     + QString::number(probeTipPointUnderRf[0]) +
			", " + QString::number(probeTipPointUnderRf[1]) +
			", " + QString::number(probeTipPointUnderRf[2]) + ")"
		);
	}
	else
	{
		m_Controls.textBrowser_TibieRes->append("Tibia Landmark Collect Done!");
	}

}


void HTONDI::CollectHTOFemurLandmark(int index)
{
	// Check the camera prerequisites: connection & tool storage 
	if (m_VegaToolStorage == nullptr)
	{
		m_Controls.textBrowser->append("NDI hasn't been set up yet!");
		return;
	}

	auto probeRFIndex = m_VegaToolStorage->GetToolIndexByName("ProbeRF");
	auto FemurRFIndex = m_VegaToolStorage->GetToolIndexByName("FemurRF");

	if (probeRFIndex == -1 || FemurRFIndex == -1)
	{
		m_Controls.textBrowser_Action->append("There is no 'ProbeRF' or 'FemurRF' in the toolStorage!");
		return;
	}

	// Check the availability of the optic tools in the FOV
	mitk::NavigationData::Pointer nd_ndiToProbe = m_VegaSource->GetOutput(probeRFIndex);
	mitk::NavigationData::Pointer nd_ndiToObjectRf = m_VegaSource->GetOutput(probeRFIndex);

	if (nd_ndiToObjectRf->IsDataValid() == 0 || nd_ndiToProbe->IsDataValid() == 0)
	{
		return;
	}

	// Update m_Pset_thaCrest_rf
	mitk::NavigationData::Pointer nd_rfToProbe = GetNavigationDataInRef(nd_ndiToProbe, nd_ndiToObjectRf);

	mitk::Point3D probeTipPointUnderRf = nd_rfToProbe->GetPosition();

	// �����ֹǽڵ�ɼ�
	// �ֹǽ������� �ֹǽ����ڲ�� �ֹ�Զ�����׵� �ֹ�Զ�����׵�
	if (index == 0)
	{
		m_tibiaProximalLateralPoint->InsertPoint(probeTipPointUnderRf);
	}
	else if (index == 1)
	{
		m_tibiaProximalMedialPoint->InsertPoint(probeTipPointUnderRf);
	}
	else
	{
		m_Controls.textBrowser_Action->append("Femur Landmark Collect Done!");
	}
}

void HTONDI::OnCollectHTOTibiaLandmarkClicked()
{
	m_Controls.textBrowser_Action->append("m_Num_TibiaLandmark: " + QString::number(m_Num_TibiaLandmark));
	auto tibiaPoints = GetDataStorage()->GetNamedNode("tibiaLandmarkPointSet");
	if (tibiaPoints)
	{
		// �ɼ���һ����
		CollectHTOTibiaLandmark(m_Num_TibiaLandmark);
		m_Num_TibiaLandmark += 1;

		// ��ʾ��ǰ������
		if (m_Num_TibiaLandmark < (tibia_image->GetLandmarks()->GetSize()))
		{
			// ���ص�
			auto currentPoint = dynamic_cast<mitk::PointSet*>(tibiaPoints->GetData())->GetPoint(m_Num_TibiaLandmark);
			mitk::PointSet::Pointer currentPointSet = mitk::PointSet::New();
			currentPointSet->InsertPoint(currentPoint);
			// �����ϴεĵ����
			auto currentLandmark = GetDataStorage()->GetNamedNode("currentLandmarkPoint");
			if (currentLandmark)
			{
				GetDataStorage()->Remove(currentLandmark);
			}
			// �������ӻ�����
			mitk::DataNode::Pointer currentPointSurface = mitk::DataNode::New();
			// װ������
			currentPointSurface->SetData(currentPointSet);
			currentPointSurface->SetName("currentLandmarkPoint");
			currentPointSurface->SetProperty("color", mitk::ColorProperty::New(0.0, 0.0, 1.0)); // ������ɫΪ��ɫ
			currentPointSurface->SetProperty("pointsize", mitk::FloatProperty::New(10.0)); // ���õ�Ĵ�СΪ5
			GetDataStorage()->Add(currentPointSurface);
		}
	}
	else
	{
		m_Controls.textBrowser_Action->append("tibiaLandmarkPointSet hasn't been loaded yet!");
	}
}

void HTONDI::OnCollectHTOFemurLandmarkClicked()
{
	GetDataStorage()->GetNamedNode("femurLandmarkPointSet")->SetVisibility(true);
	// ��ʾ��ǰ������

	CollectHTOTibiaLandmark(m_Num_FemurLandmark);
	m_Num_FemurLandmark += 1;
}

void HTONDI::OnCollectHTOTibiaICPClicked()
{
	// Check the camera prerequisites: connection & tool storage 
	if (m_VegaToolStorage == nullptr)
	{
		m_Controls.textBrowser_Action->append("NDI hasn't been set up yet!");
		return;
	}

	auto probeIndex = m_VegaToolStorage->GetToolIndexByName("ProbeRF");
	auto objectRfIndex = m_VegaToolStorage->GetToolIndexByName("TibiaRF");

	if (probeIndex == -1 || objectRfIndex == -1)
	{
		m_Controls.textBrowser_Action->append("There is no 'ProbeRF' or 'TibiaRF' in the toolStorage!");
		return;
	}

	// Check the availability of the optic tools in the FOV
	mitk::NavigationData::Pointer nd_ndiToProbe = m_VegaSource->GetOutput(probeIndex);
	mitk::NavigationData::Pointer nd_ndiToObjectRf = m_VegaSource->GetOutput(objectRfIndex);

	if (nd_ndiToObjectRf->IsDataValid() == 0 || nd_ndiToProbe->IsDataValid() == 0)
	{
		return;
	}

	// Update m_Pset_thaAntHorn_rf
	mitk::NavigationData::Pointer nd_rfToProbe = GetNavigationDataInRef(nd_ndiToProbe, nd_ndiToObjectRf);

	mitk::Point3D probeTipPointUnderRf = nd_rfToProbe->GetPosition();

	double disThres{ 4.0 };

	if (m_tibiaICPPoints->GetSize() < 1)
	{
		m_tibiaICPPoints->InsertPoint(probeTipPointUnderRf);
	}
	else
	{
		auto p = m_tibiaICPPoints->GetPoint(m_tibiaICPPoints->GetSize() - 1);
		double tmpDis = sqrt(pow(p[0] - probeTipPointUnderRf[0], 2) +
			pow(p[1] - probeTipPointUnderRf[1], 2) +
			pow(p[2] - probeTipPointUnderRf[2], 2));

		if (tmpDis > disThres)
		{
			m_tibiaICPPoints->InsertPoint(probeTipPointUnderRf);
			m_Controls.textBrowser_TibieRes->append("Added CurrentTibiaICPPoint: ("
				+ QString::number(probeTipPointUnderRf[0]) +
				", " + QString::number(probeTipPointUnderRf[1]) +
				", " + QString::number(probeTipPointUnderRf[2]) + ")"
			);
		}
		else
		{
			m_Controls.textBrowser_Action->append("Current ICP point is too close to the previous one");
		}
	}

}


bool HTONDI::OnCollectHTOTibiaRegisClicked()
{
	/* check data complete then start rigis 
		output: 
			m_Regis_Error
			m_ObjectRfToImageMatrix_hto
			UpdateHTOProbe
	*/
	if (m_tibiaProximalLateralPoint->GetSize() < 1 ||
		m_tibiaProximalMedialPoint->GetSize() < 1 ||
		m_tibiaDistalLateralPoint->GetSize() < 1 ||
		m_tibiaDistalMedialPoint->GetSize() < 1 ||
		m_Surface_HTOTibiaSurface->GetVtkPolyData() == nullptr)
	{
		m_Controls.textBrowser_Action->append("not enough points");
		return false;
	}

	auto landmark_rf = mitk::PointSet::New();

	landmark_rf->InsertPoint(m_tibiaProximalLateralPoint->GetPoint(0));
	landmark_rf->InsertPoint(m_tibiaProximalMedialPoint->GetPoint(0));
	landmark_rf->InsertPoint(m_tibiaDistalLateralPoint->GetPoint(0));
	landmark_rf->InsertPoint(m_tibiaDistalMedialPoint->GetPoint(0));

	auto icp_rf = mitk::PointSet::New();

	for (int i{ 0 }; i < 15; i++)
	{
		icp_rf->InsertPoint(m_tibiaICPPoints->GetPoint(i));
	}

	// Calculate T_imageToObjectRf
	auto combinedRegistrator = mitk::SurfaceRegistration::New();
	// combinedRegistrator->SetLandmarksTarget(m_Pset_thaBoneLandmark);
	// combinedRegistrator->SetLandmarksSrc(landmark_rf);
	combinedRegistrator->SetLandmarksSrc(m_tibiaLandmarkPoints);
	combinedRegistrator->SetLandmarksTarget(landmark_rf);
	combinedRegistrator->SetIcpPoints(icp_rf);
	combinedRegistrator->SetSurfaceSrc(m_Surface_HTOTibiaSurface);
	combinedRegistrator->ComputeLandMarkResult();
	combinedRegistrator->ComputeIcpResult();

	m_ObjectRfToImageMatrix_hto->DeepCopy(combinedRegistrator->GetResult());

	// ��ӡ��ǰ��׼����Ľ��
	// lanmark
	m_Controls.textBrowser_TibieRes->append("Avg landmark error:" + QString::number(combinedRegistrator->GetavgLandmarkError()));
	m_Controls.textBrowser_TibieRes->append("Max landmark error:" + QString::number(combinedRegistrator->GetmaxLandmarkError()));
	// icp
	m_Controls.textBrowser_TibieRes->append("Avg ICP error:" + QString::number(combinedRegistrator->GetavgIcpError()));
	m_Controls.textBrowser_TibieRes->append("Max ICP error:" + QString::number(combinedRegistrator->GetmaxIcpError()));


	// ����̽�뵼��
	if (m_HTOPrboeUpdateTimer == nullptr)
	{
		m_HTOPrboeUpdateTimer = new QTimer(this); //create a new timer
	}

	// ��ʾ̽��
	GetDataStorage()->GetNamedNode("Probe_forVisual")->SetVisibility(true);

	// ��ʼ����̽��λ��
	disconnect(m_HTOPrboeUpdateTimer, SIGNAL(timeout()), this, SLOT(UpdateHTOProbe()));
	connect(m_HTOPrboeUpdateTimer, SIGNAL(timeout()), this, SLOT(UpdateHTOProbe()));
	m_HTOPrboeUpdateTimer->start(100);


	return true;

}

