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
		saw_image->SetReferencFrameName(SawPoints->GetName());
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
		drill_image->SetReferencFrameName(DrillPoints->GetName());
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
	/* ��ǵ���֤-��֤�ɹǱ�ǵ�
	1. ̽��ɵ�
	2. ���������ɼ����ĵ���ʵ�ʹ滮�ĵ����������Ƚ�
	*/

	// ��֤�ɹǵ�
	// ���ɹ�RF��Ȼ��ʹ��̽��ɼ��ɹ��ϱ�ǵ�λ�ã�����ǰע���λ�ý���������
	m_Controls.textBrowser_Action->append("Action: Check Femur Node.");

	// ��ȡ̽�������λ��
	auto probeRF = m_VegaToolStorage->GetToolByName("ProbeRF");
	if (probeRF.IsNull())
	{
		m_Controls.textBrowser_Action->append("ERROR: ProbeRF tool not found.");
		return false;
	}

	// ��ȡ���λ��
	mitk::Point3D TipProbe = probeRF->GetToolTipPosition();


	// ��ȡ��ǰ�ɹ�RF��λ��
	auto femurRF = m_VegaToolStorage->GetToolByName("FemurRF");
	if (femurRF.IsNull())
	{
		m_Controls.textBrowser_Action->append("ERROR: FemurRF not found, try reload .IGTToolStorage");
		return false;
	}

	// ��ȡ����
	m_femurRFNode = femurRF;

	// ��ȡ��֤�� VerifyPoint
	mitk::Point3D valid_point;

	// �������VerifyPoint�����ǻ�������֤�㣿
	valid_point = femurRF->GetVerifyPoint();

	// �������
	double vx = TipProbe[0] - valid_point[0];
	double vy = TipProbe[1] - valid_point[1];
	double vz = TipProbe[2] - valid_point[2];
	double FemurError = sqrt(pow(vx, 2) + pow(vy, 2) + pow(vz, 2));

	// ������
	m_Controls.textBrowser_CheckProbe->append("Probe to FemurPoint ERROR: " + QString::number(FemurError));

	return true;
}


bool HTONDI::OnTibiaCheckClicked()
{
	/* ��ǵ���֤-��֤�ֹǱ�ǵ�
	1. ̽��ɵ�
	2. ���������ɼ����ĵ���ʵ�ʹ滮�ĵ����������Ƚ�
	*/

	// ����ֹ�RF��Ȼ��ʹ��̽��ɼ��ֹ��ϱ�ǵ�λ�ã�����ǰע���λ�ý���������
	m_Controls.textBrowser_Action->append("Action: Check Tibia Node.");

	// ��ȡ̽�������λ��
	auto probeRF = m_VegaToolStorage->GetToolByName("ProbeRF");
	if (probeRF.IsNull())
	{
		m_Controls.textBrowser_Action->append("ERROR: ProbeRF tool not found.");
		return false;
	}

	// ��ȡ���λ��
	mitk::Point3D TipProbe = probeRF->GetToolTipPosition();

	// ��ȡ��ǰ�ɹ�RF��λ��
	auto tibiaRF = m_VegaToolStorage->GetToolByName("TibiaRF");
	if (tibiaRF.IsNull())
	{
		m_Controls.textBrowser_Action->append("ERROR: TibiaRF not found, try reload .IGTToolStorage");
		return false;
	}

	// ��ȡ����
	m_tibiaRFNode = tibiaRF;

	// ��ȡ��֤�� VerifyPoint
	mitk::Point3D valid_point;

	// �������VerifyPoint�����ǻ�������֤�㣿
	valid_point = m_tibiaRFNode->GetVerifyPoint();

	// �������
	double vx = TipProbe[0] - valid_point[0];
	double vy = TipProbe[1] - valid_point[1];
	double vz = TipProbe[2] - valid_point[2];
	double FemurError = sqrt(pow(vx, 2) + pow(vy, 2) + pow(vz, 2));

	// ������
	m_Controls.textBrowser_CheckProbe->append("Probe to TibiaPoint ERROR: " + QString::number(FemurError));

	return true;
}


bool HTONDI::OnCalibrateSawClicked()
{
	/* ��е�궨-�ھ�궨
	1. ��¼̽�����λ�ã�ע��ھ�궨��λ��
	2. ��׼�ھ����λ�ã�����T_saw�µİھ�궨�������
	3. Ӧ����׼���
	4. ���������ͼ��

	====����֤0813====
	*/

	m_Controls.textBrowser_Action->append("Action: Calibrate Saw Node.");
	
	cout << "test 01" << endl;

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

	// ��ȡ����
	m_saw = sawRF;

	cout << "test 03" << endl;
	// ��ʼ�궨��ɼ��ͱ궨����
	// ȡ�ô���׼�㼯 �� ��ǰ�Ѿ�ȡ�õĵ�

	// �����׼��������Ѿ��㹻����ʼ������׼����
	int sawRF_SurfaceSrcNum = saw_image->GetLandmarks()->GetSize();
	int sawRF_SurfaceTargetNum = saw_image->GetLandmarks_probe()->GetSize();

	if (sawRF_SurfaceSrcNum == sawRF_SurfaceTargetNum)
	{
		cout << "test 06" << endl;
		m_Controls.textBrowser_Action->append("--- Enough Saw landmarks have been collected ----");
		// ��ʼ��׼����

		// ����һ���µľ�̬ͼ�������׼������ʵ��
		m_surfaceRegistrationStaticImageFilter = lancet::ApplySurfaceRegistratioinStaticImageFilter::New();

		cout << "test 07" << endl;
		// �����������ӵ�Vega��������Դ
		m_surfaceRegistrationStaticImageFilter->ConnectTo(m_VegaSource);

		// ����һ���µķ���任�������ڴ洢��׼���
		m_imageRegistrationMatrix = mitk::AffineTransform3D::New();

		cout << "test 08" << endl;
		// ���µ���ͼ��Ķ��󵽲ο���ܵı任����
		saw_image->UpdateObjectToRfMatrix();

		cout << "test 09" << endl;
		// ��ӡ��ǰ��׼����Ľ��
		// lanmark
		m_Controls.textBrowser_CalibrateRes->append("Avg landmark error:" + QString::number(saw_image->GetlandmarkRegis_avgError()));
		m_Controls.textBrowser_CalibrateRes->append("Max landmark error:" + QString::number(saw_image->GetlandmarkRegis_maxError()));

		cout << "Avg landmark error:" << saw_image->GetlandmarkRegis_avgError() << endl;
		cout << "Max landmark error:" << saw_image->GetlandmarkRegis_maxError() << endl;

		cout << "test 10" << endl;
		// ������ͼ��Ķ��󵽲ο���ܵı任�����VTK��ʽת��ΪITK��ʽ�����洢
		mitk::TransferVtkMatrixToItkTransform(saw_image->GetT_Object2ReferenceFrame(), m_imageRegistrationMatrix.GetPointer());

		cout << "test 11" << endl;
		// ����׼����Ӧ�õ� SawRF ������
		m_VegaToolStorage->GetToolByName("SawRF")->SetToolRegistrationMatrix(m_imageRegistrationMatrix);

		cout << "test 12" << endl;
		// ����׼�������õ���̬ͼ�������׼��������
		//m_surfaceRegistrationStaticImageFilter->SetRegistrationMatrix(m_VegaToolStorage->GetToolByName("SawRF")->GetToolRegistrationMatrix());

		cout << "test 13" << endl;
		// ���òο���ܵĵ�������
		/*
			�����ʱ���е�����RF����Ҫ���Խ������ڵ�����RF�½�������˶�
		*/
		//m_surfaceRegistrationStaticImageFilter->SetNavigationDataOfRF(m_VegaSource->GetOutput("SawRF"));

		cout << "test 14" << endl;
		// ֹͣVega���ӻ���ʱ��
		m_VegaVisualizeTimer->stop();

		cout << "test 15" << endl;
		// ��Vega���ӻ������ӵ���̬ͼ�������׼������
		m_VegaVisualizer->ConnectTo(m_surfaceRegistrationStaticImageFilter);

		cout << "test 16" << endl;
		// ��������Vega���ӻ���ʱ������ʾ��׼����Ľ��
		m_VegaVisualizeTimer->start();

		cout << "test 17" << endl;
		// 4. ���������
		sawRF->SetDataNode(sawRF_Surface);
		sawRF_Surface->SetVisibility(true);

		return true;
	}

	// �����ɵ㲻�㹻��������ɵ�
	cout << "test 04" << endl;
	// ȡ���Ѿ��õ���Target��׼�㼯
	auto pointSet_probeLandmark = saw_image->GetLandmarks_probe();
	auto pointSet_landmark = saw_image->GetLandmarks();

	// ��ʼѡ��궨��
	// ��Զ������ʼ�궨

	// ��⹤���Ƿ���Ч
	auto probeIndex = m_VegaToolStorage->GetToolIndexByName("ProbeRF");
	auto sawRfIndex = m_VegaToolStorage->GetToolIndexByName("SawRF");
	if (probeIndex == -1 || sawRfIndex == -1)
	{
		m_Controls.textBrowser_Action->append("There is no 'Probe' or 'SawRf' in the toolStorage!");
	}

	cout << "test 05" << endl;
	// ��ȡ̽���� NDI ����ϵͳ����ϵ�е�λ�úͷ�������
	mitk::NavigationData::Pointer nd_ndiToProbe = m_VegaSource->GetOutput(probeIndex);
	
	// ��ȡ�ھ���NDI����ϵ�е�λ�úͷ�������
	mitk::NavigationData::Pointer nd_ndiToObjectRf = m_VegaSource->GetOutput(sawRfIndex);
	
	// ��̽���λ�ô�NDI����ϵת�����ھ������ϵ��
	mitk::NavigationData::Pointer nd_rfToProbe = GetNavigationDataInRef(nd_ndiToProbe, nd_ndiToObjectRf);
	
	// ��ת����ĵ�����������ȡ̽���˵�λ��, ���λ������������ڲο���������ϵ��
	mitk::Point3D probeTipPointUnderRf = nd_rfToProbe->GetPosition();
	
	// ������õ���̽����λ����ӵ� �㼯 ��
	pointSet_probeLandmark->InsertPoint(probeTipPointUnderRf);
	
	// ��ӡ��Ϣ
	m_Controls.textBrowser_Action->append("Added saw landmark: " + QString::number(probeTipPointUnderRf[0]) +
		"/ " + QString::number(probeTipPointUnderRf[1]) + "/ " + QString::number(probeTipPointUnderRf[2]));

	return true;
}

bool HTONDI::OnCalibrateDrillClicked()
{
	/* ĥ��궨
	1. ���������ͼ��
	2. ��¼̽�����λ�ã�ע��ĥ��궨��λ��
	3. ��׼�ھ����λ�ã�����T_drill�µİھ�궨�������
	4. Ӧ����׼���

	====����֤0813====
	*/

	m_Controls.textBrowser_Action->append("Action: Calibrate drill Node.");

	// ��ȡ��ǰ�ھ�RF��λ��
	auto drillRF = m_VegaToolStorage->GetToolByName("DrillRF");

	// ��ȡ�ھ�ı�������
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

	// ��ȡ����
	m_drill = drillRF;

	// ��ʼ�궨��ɼ��ͱ궨����
	// ȡ�ô���׼�㼯 �� ��ǰ�Ѿ�ȡ�õĵ�
	int drillRF_SurfaceSrcNum = drill_image->GetLandmarks()->GetSize();
	int drillRF_SurfaceTargetNum = drill_image->GetLandmarks_probe()->GetSize();

	if (drillRF_SurfaceSrcNum == drillRF_SurfaceTargetNum)
	{
		m_Controls.textBrowser_Action->append("--- Enough Drill landmarks have been collected ----");
		// ��ʼ��׼����
		// ����һ���µľ�̬ͼ�������׼������ʵ��
		m_surfaceRegistrationStaticImageFilter = lancet::ApplySurfaceRegistratioinStaticImageFilter::New();

		// �����������ӵ�Vega��������Դ
		m_surfaceRegistrationStaticImageFilter->ConnectTo(m_VegaSource);

		// ����һ���µķ���任�������ڴ洢��׼���
		m_imageRegistrationMatrix = mitk::AffineTransform3D::New();

		// ���µ���ͼ��Ķ��󵽲ο���ܵı任����
		drill_image->UpdateObjectToRfMatrix();

		// ��ӡ��ǰ��׼����Ľ��
		// lanmark
		m_Controls.textBrowser_CalibrateRes->append("Avg landmark error:" + QString::number(drill_image->GetlandmarkRegis_avgError()));
		m_Controls.textBrowser_CalibrateRes->append("Max landmark error:" + QString::number(drill_image->GetlandmarkRegis_maxError()));

		// ������ͼ��Ķ��󵽲ο���ܵı任�����VTK��ʽת��ΪITK��ʽ�����洢
		mitk::TransferVtkMatrixToItkTransform(drill_image->GetT_Object2ReferenceFrame(), m_imageRegistrationMatrix.GetPointer());

		// ����׼����Ӧ�õ� DrillRF ������
		m_VegaToolStorage->GetToolByName("DrillRF")->SetToolRegistrationMatrix(m_imageRegistrationMatrix);

		// ����׼�������õ���̬ͼ�������׼��������
		m_surfaceRegistrationStaticImageFilter->SetRegistrationMatrix(m_VegaToolStorage->GetToolByName("DrillRF")->GetToolRegistrationMatrix());

		// ���òο���ܵĵ�������
		m_surfaceRegistrationStaticImageFilter->SetNavigationDataOfRF(m_VegaSource->GetOutput("DrillRF"));

		// ֹͣVega���ӻ���ʱ��
		m_VegaVisualizeTimer->stop();

		// ��Vega���ӻ������ӵ���̬ͼ�������׼������
		m_VegaVisualizer->ConnectTo(m_surfaceRegistrationStaticImageFilter);

		// ��������Vega���ӻ���ʱ������ʾ��׼����Ľ��
		m_VegaVisualizeTimer->start();

		// ���������
		drillRF->SetDataNode(drillRF_Surface);
		drillRF_Surface->SetVisibility(true);

		return true;
	}

	// û�вɵ��㹻��ʱ�򣬼����ɵ�
	// ȡ���Ѿ��õ���Target��׼�㼯
	auto pointSet_probeLandmark = drill_image->GetLandmarks_probe();
	auto pointSet_landmark = drill_image->GetLandmarks();

	// ��ʼѡ��궨��
	// ��Զ������ʼ�궨

	// ��⹤���Ƿ���Ч
	auto probeIndex = m_VegaToolStorage->GetToolIndexByName("ProbeRF");
	auto drillRfIndex = m_VegaToolStorage->GetToolIndexByName("DrillRF");
	if (probeIndex == -1 || drillRfIndex == -1)
	{
		m_Controls.textBrowser_Action->append("There is no 'Probe' or 'SawRf' in the toolStorage!");
	}
	// ��ȡ̽���� NDI ����ϵͳ����ϵ�е�λ�úͷ�������
	mitk::NavigationData::Pointer nd_ndiToProbe = m_VegaSource->GetOutput(probeIndex);

	// ��ȡĥ����NDI����ϵ�е�λ�úͷ�������
	mitk::NavigationData::Pointer nd_ndiToObjectRf = m_VegaSource->GetOutput(drillRfIndex);

	// ��̽���λ�ô�NDI����ϵת�����ھ������ϵ��
	mitk::NavigationData::Pointer nd_rfToProbe = GetNavigationDataInRef(nd_ndiToProbe, nd_ndiToObjectRf);

	// ��ת����ĵ�����������ȡ̽���˵�λ��, ���λ������������ڲο���������ϵ��
	mitk::Point3D probeTipPointUnderRf = nd_rfToProbe->GetPosition();

	// ������õ���̽����λ����ӵ� �㼯 ��
	pointSet_probeLandmark->InsertPoint(probeTipPointUnderRf);

	// ��ӡ��Ϣ
	m_Controls.textBrowser_Action->append("Added drill landmark: " + QString::number(probeTipPointUnderRf[0]) +
		"/ " + QString::number(probeTipPointUnderRf[1]) + "/ " + QString::number(probeTipPointUnderRf[2]));

	return true;
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
		// չʾ�ְ�͵㼯��
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


// ����׼����
bool HTONDI::OnGetFemurLandmarkClicked()
{
	/* �ɹǵ��������׼��ɼ� = �ɹ����׵� �ɹ����׵� �ɼ�
	1. ����׼��ɼ�������ɼ�ǰ�����㣬�ɹ�ͷ���Ĳ�����ϵķ������

	��Ҫ̽�뼰�ɹǵ�RF: ProbeRF FemurRF
	*/
	m_Controls.textBrowser_Action->append("Action: Get Femur Landmark Nodes.");

	cout << "test 01" << endl;
	// ��ȡ�ɹǵı�������
	auto FemurRF_Surface = GetDataStorage()->GetNamedNode("femurSurface");

	if (FemurRF_Surface == nullptr)
	{
		m_Controls.textBrowser_Action->append("ERROR: Femur model Not Found!");
		return false;
	}
	cout << "test 02" << endl;
	// ȡ���Ѿ��õ���Target��׼�㼯
	auto pointSet_probeLandmark = femur_image->GetLandmarks_probe();
	auto pointSet_landmark = femur_image->GetLandmarks();


	// ��������
	// 162.517/ 379.001/ 147.178
	// 141.182/ 416.836/ 153.199
	if (true)
	{
		// ����һ���µ� Point3D �������洢�����õ������
		mitk::Point3D testPoint1;
		testPoint1[0] = 162.517;
		testPoint1[1] = 379.001;
		testPoint1[2] = 147.178;
		// ������һ�� Point3D �������洢�ڶ��������õ������
		mitk::Point3D testPoint2;
		testPoint2[0] = 141.182;
		testPoint2[1] = 416.836;
		testPoint2[2] = 153.199;
		pointSet_probeLandmark->InsertPoint(testPoint1);
		pointSet_probeLandmark->InsertPoint(testPoint2);
	}


	// ��ʼѡ��궨��
	// �ɹ����׵� -> �ɹ����׵�

	cout << "test 03" << endl;
	// ��⹤���Ƿ���Ч
	auto probeIndex = m_VegaToolStorage->GetToolIndexByName("ProbeRF");
	auto femurRfIndex = m_VegaToolStorage->GetToolIndexByName("FemurRF");

	if (probeIndex == -1 || femurRfIndex == -1)
	{
		m_Controls.textBrowser_Action->append("There is no 'Probe' or 'Femur' in the toolStorage!");
	}

	// �ɹǴ���׼��Ĳɼ�ֻ��2����
	auto tmpNum = femur_image->GetLandmarks_probe()->GetSize();
	if (tmpNum >= 2)
	{
		m_Controls.textBrowser_Action->append("'Femur' landmark point done, please caculate hipcenter!");
		return false;
	}

	cout << "test 04" << endl;
	// ��ȡ̽���� NDI ����ϵͳ����ϵ�е�λ�úͷ�������
	mitk::NavigationData::Pointer nd_ndiToProbe = m_VegaSource->GetOutput(probeIndex);

	// ��ȡ�ھ���NDI����ϵ�е�λ�úͷ�������
	mitk::NavigationData::Pointer nd_ndiToObjectRf = m_VegaSource->GetOutput(femurRfIndex);

	// ��̽���λ�ô�NDI����ϵת�����ھ������ϵ��
	mitk::NavigationData::Pointer nd_rfToProbe = GetNavigationDataInRef(nd_ndiToProbe, nd_ndiToObjectRf);

	// ��ת����ĵ�����������ȡ̽���˵�λ��, ���λ������������ڲο���������ϵ��
	mitk::Point3D probeTipPointUnderRf = nd_rfToProbe->GetPosition();

	// ������õ���̽����λ����ӵ� �㼯 ��
	pointSet_probeLandmark->InsertPoint(probeTipPointUnderRf);

	// ��ӡ��Ϣ
	m_Controls.textBrowser_FemurRes->append("Added femur landmark: " + QString::number(probeTipPointUnderRf[0]) +
		"/ " + QString::number(probeTipPointUnderRf[1]) + "/ " + QString::number(probeTipPointUnderRf[2]));

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
	1. ��׼����
	2. Ӧ����׼����
	3. ���������ͼ�� 

	�ɹ�RF����ΪFemurRf
	====����֤0813====
	*/

	// ������õ��� �ɹ�ͷ���� Ҳ��ӵ� landmark �б� 
	m_Controls.textBrowser_Action->append("Action: Caculate Femur Landmark.");

	// ���ɹ�ͷ���ĵ���ӽ���
	// ȡ���Ѿ��õ���Target��׼�㼯
	auto pointSet_probeLandmark = femur_image->GetLandmarks_probe();

	// ������õ���̽����λ����ӵ� �㼯 ��
	mitk::Point3D tmpHipCenter;
	tmpHipCenter[0] = m_HipCenter[0];
	tmpHipCenter[1] = m_HipCenter[1];
	tmpHipCenter[2] = m_HipCenter[2];
	pointSet_probeLandmark->InsertPoint(tmpHipCenter);

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

	// ��ʼ��׼����
	// ����һ���µľ�̬ͼ�������׼������ʵ��
	m_surfaceRegistrationStaticImageFilter = lancet::ApplySurfaceRegistratioinStaticImageFilter::New();

	// �����������ӵ�Vega��������Դ
	m_surfaceRegistrationStaticImageFilter->ConnectTo(m_VegaSource);

	// ����һ���µķ���任�������ڴ洢��׼���
	m_imageRegistrationMatrix = mitk::AffineTransform3D::New();

	// ���µ���ͼ��Ķ��󵽲ο���ܵı任����
	femur_image->UpdateObjectToRfMatrix();

	// ��ӡ��ǰ��׼����Ľ��
	// lanmark
	m_Controls.textBrowser_FemurRes->append("Avg landmark error:" + QString::number(femur_image->GetlandmarkRegis_avgError()));
	m_Controls.textBrowser_FemurRes->append("Max landmark error:" + QString::number(femur_image->GetlandmarkRegis_maxError()));

	// ������ͼ��Ķ��󵽲ο���ܵı任�����VTK��ʽת��ΪITK��ʽ�����洢
	mitk::TransferVtkMatrixToItkTransform(femur_image->GetT_Object2ReferenceFrame(), m_imageRegistrationMatrix.GetPointer());

	// ����׼����Ӧ�õ� FemurRF ������
	m_VegaToolStorage->GetToolByName("FemurRF")->SetToolRegistrationMatrix(m_imageRegistrationMatrix);

	// ����׼�������õ���̬ͼ�������׼��������
	m_surfaceRegistrationStaticImageFilter->SetRegistrationMatrix(m_VegaToolStorage->GetToolByName("FemurRF")->GetToolRegistrationMatrix());

	// ���òο���ܵĵ�������
	m_surfaceRegistrationStaticImageFilter->SetNavigationDataOfRF(m_VegaSource->GetOutput("FemurRF"));

	// ֹͣVega���ӻ���ʱ��
	m_VegaVisualizeTimer->stop();

	// ��Vega���ӻ������ӵ���̬ͼ�������׼������
	m_VegaVisualizer->ConnectTo(m_surfaceRegistrationStaticImageFilter);

	// ��������Vega���ӻ���ʱ������ʾ��׼����Ľ��
	m_VegaVisualizeTimer->start();

	// ��ͼ�����
	// ��ȡ��ǰ�ھ�RF��λ��
	auto FemurRF = m_VegaToolStorage->GetToolByName("FemurRF");

	// ��ȡ�ھ�ı�������
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

	// ���������
	FemurRF->SetDataNode(FemurRF_Surface);
	FemurRF_Surface->SetVisibility(true);

	return true;
}


bool HTONDI::OnGetTibiaLandmarkClicked()
{
	/* �ֹǱ������׼��ɼ����ֹǽ������� �ֹǽ����ڲ�� �ֹ�Զ�����׵� �ֹ�Զ�����׵�
	1. ����׼��ɼ�
	*/
	m_Controls.textBrowser_Action->append("Action: Get Tibia Landmark.");

	// ��ȡ�ɹǵı�������
	auto TibiaRF_Surface = GetDataStorage()->GetNamedNode("tibiaSurface");

	if (TibiaRF_Surface == nullptr)
	{
		m_Controls.textBrowser_Action->append("ERROR: Tibia model Not Found!");
		return false;
	}

	// ȡ���Ѿ��õ���Target��׼�㼯
	auto pointSet_probeLandmark = tibia_image->GetLandmarks_probe();
	auto pointSet_landmark = tibia_image->GetLandmarks();

	// ��ʼѡ��궨��
	// �ɹ����׵� -> �ɹ����׵�

	// ��⹤���Ƿ���Ч
	auto probeIndex = m_VegaToolStorage->GetToolIndexByName("ProbeRF");
	auto tibiaRfIndex = m_VegaToolStorage->GetToolIndexByName("TibiaRF");

	if (probeIndex == -1 || tibiaRfIndex == -1)
	{
		m_Controls.textBrowser_Action->append("There is no 'Probe' or 'Femur' in the toolStorage!");
	}
	// ��ȡ̽���� NDI ����ϵͳ����ϵ�е�λ�úͷ�������
	mitk::NavigationData::Pointer nd_ndiToProbe = m_VegaSource->GetOutput(probeIndex);

	// ��ȡ�ھ���NDI����ϵ�е�λ�úͷ�������
	mitk::NavigationData::Pointer nd_ndiToObjectRf = m_VegaSource->GetOutput(tibiaRfIndex);

	// ��̽���λ�ô�NDI����ϵת�����ھ������ϵ��
	mitk::NavigationData::Pointer nd_rfToProbe = GetNavigationDataInRef(nd_ndiToProbe, nd_ndiToObjectRf);

	// ��ת����ĵ�����������ȡ̽���˵�λ��, ���λ������������ڲο���������ϵ��
	mitk::Point3D probeTipPointUnderRf = nd_rfToProbe->GetPosition();

	// ������õ���̽����λ����ӵ� �㼯 ��
	pointSet_probeLandmark->InsertPoint(probeTipPointUnderRf);

	// ��ӡ��Ϣ
	m_Controls.textBrowser_TibieRes->append("Added tibia landmark src: " + QString::number(probeTipPointUnderRf[0]) +
		"/ " + QString::number(probeTipPointUnderRf[1]) + "/ " + QString::number(probeTipPointUnderRf[2]));

	return true;
}

bool HTONDI::OnCaculateTibiaLandmarlClicked()
{	
	/* �ֹǵ��ĵ����׼���㼰Ӧ��
	1. ����׼����
	2. ����׼Ӧ��

	====����֤0813====
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

	// ��ʼ��׼����
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

	return true;
}

bool HTONDI::OnGetTibieICPClicked()
{
	/* �ֹǱ��澫��׼��ɼ�
	1. ����ɵ�

	=========����֤0814===========
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
	*/
	m_Controls.textBrowser_Action->append("Action: Caculate Tibia ICP.");

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
		m_Controls.textBrowser_Action->append("ERROR: Femur model Not Found!");
		return false;
	}

	// ���������
	TibiaRF->SetDataNode(TibiaRF_Surface);
	TibiaRF_Surface->SetVisibility(true);

	return true;
}


