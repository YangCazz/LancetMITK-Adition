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
#include <QMessageBox>
#include <QtWidgets/QFileDialog>
#include <QtCore\qmath.h>

// mitk
#include <ep/include/vtk-9.1/vtkTransformFilter.h>
#include <mitkPointSet.h>
#include <mitkRemeshing.h>
#include "mitkVtkInterpolationProperty.h"

// vtk
#include <vtkPolyDataPlaneClipper.h>
#include <vtkFillHolesFilter.h>
#include <vtkAppendPolyData.h>
#include <vtkCleanPolyData.h>
#include <vtkClipPolyData.h>
#include "vtkOBBTree.h"
#include <vtkPlaneSource.h>
#include <vtkCutter.h>

/*========================= ��ǰ�滮 ==================================
HTONDI_PreoperativePlan.cpp
----------------------------------------------------------------
== 1.У������׼��
== �ع���滮
== ��֫���߹滮
== �ſ��Ƕȹ滮
== �ְ�滮
== �ع�ЧӦ
=====================================================================*/



// 1.У������׼��
bool HTONDI::OnCheckBaseDataClicked()
{
	// У����Թ��̵������걸��
	m_Controls.textBrowser_Action->append("Action_01: Check Base data.");
	auto femurLandmark = GetDataStorage()->GetNamedNode("femurLandmarkPointSet");
	auto tibiaLandmark = GetDataStorage()->GetNamedNode("tibiaLandmarkPointSet");
	auto femurHeadPoints = GetDataStorage()->GetNamedNode("femoralHead");
	auto femur = GetDataStorage()->GetNamedNode("femurSurface");
	auto tibia = GetDataStorage()->GetNamedNode("tibiaSurface");
	auto steelPlate = GetDataStorage()->GetNamedNode("SteelPlate");
	auto steelPlatePoints = GetDataStorage()->GetNamedNode("SteelPlatePointSet");
	auto Saw = GetDataStorage()->GetNamedNode("Saw");
	auto SawPoints = GetDataStorage()->GetNamedNode("SawLandMarkPointSet");
	auto Drill = GetDataStorage()->GetNamedNode("Drill");
	auto DrillPoints = GetDataStorage()->GetNamedNode("DrillLandMarkPointSet");
	auto LinkPin = GetDataStorage()->GetNamedNode("LinkPin");
	auto LinkPinLandmark = GetDataStorage()->GetNamedNode("LinkPinLandMarkPointSet");
	auto Probe = GetDataStorage()->GetNamedNode("Probe");
	auto ProbeLandmark = GetDataStorage()->GetNamedNode("ProbeLandMarkPointSet");

	// ȷ������ģ�͵�ģʽ
	if (m_Controls.checkBox_LeftlimbHTO->isChecked())
	{
		judgModel_flag = 1;
		m_Controls.textBrowser_Action->append("Set action mode as Left Limb!");
	}
	else if (m_Controls.checkBox_RightlimbHTO->isChecked())
	{
		judgModel_flag = 0;
		m_Controls.textBrowser_Action->append("Set action mode as Right Limb!");
	}

	if (femurLandmark)
	{
		m_Controls.checkBox_point02->setChecked(true);
		m_Controls.checkBox_point03->setChecked(true);
		m_Controls.textBrowser_Action->append("load femurLandmarkPointSet.");
		m_Controls.textBrowser_Action->append("load femurLateralMalleolusPoint.");
		m_Controls.textBrowser_Action->append("load femurMedialMalleolusPoint.");

		SetModelColor(femurLandmark, 1.0f, 0.0f, 0.0f);
		SetNodeSize(femurLandmark, 10.0f);


	}
	if (tibiaLandmark)
	{
		m_Controls.checkBox_point04->setChecked(true);
		m_Controls.checkBox_point05->setChecked(true);
		m_Controls.checkBox_point06->setChecked(true);
		m_Controls.checkBox_point07->setChecked(true);
		m_Controls.textBrowser_Action->append("load tibiaLandmarkPointSet.");
		m_Controls.textBrowser_Action->append("load tibiaProximalLateralPoint.");
		m_Controls.textBrowser_Action->append("load tibiaProximalMedialPoint.");
		m_Controls.textBrowser_Action->append("load tibiaDistalLateralPoint.");
		m_Controls.textBrowser_Action->append("load tibiaDistalMedialPoint.");

		SetModelColor(tibiaLandmark, 1.0f, 0.0f, 0.0f);
		SetNodeSize(tibiaLandmark, 10.0f);

		// װ��tibia��landmark�㣬����װ�����ĸ���
		// �ֹǽ������� �ֹǽ����ڲ�� �ֹ�Զ�����׵� �ֹ�Զ�����׵�
		cout << "test 02" << endl;
		// femur_image->SetLandmarks(dynamic_cast<mitk::PointSet*>(tibiaLandmark->GetData()));
	}
	if (femurHeadPoints)
	{
		m_Controls.textBrowser_Action->append("load femurHeadPoints.");
		m_Controls.checkBox_point01->setChecked(true);

	}
	if (femur)
	{
		m_Controls.textBrowser_Action->append("load femurSurface.");
		m_Controls.checkBox_point08->setChecked(true);

	}
	if (tibia)
	{
		m_Controls.textBrowser_Action->append("load tibiaSurface.");
		m_Controls.checkBox_point09->setChecked(true);

	}

	// ����������Ϊ���ɼ�����
	if (steelPlate)
	{
		m_Controls.textBrowser_Action->append("load SteelPlate.");
		m_Controls.checkBox_point11->setChecked(true);
		steelPlate->SetVisibility(false);
		// ��¼�µ�ǰ���ְ�ĳ�ʼλ��==����ԭ������
		m_steelPosition = steelPlate->GetData()->GetGeometry()->GetOrigin();

	}
	if (Saw)
	{
		m_Controls.textBrowser_Action->append("load Saw.");
		m_Controls.checkBox_point12->setChecked(true);
		Saw->SetVisibility(false);

	}
	if (Drill)
	{
		m_Controls.textBrowser_Action->append("load Drill.");
		m_Controls.checkBox_point13->setChecked(true);
		Drill->SetVisibility(false);

	}

	// ���߱�ǵ㣬����landmark��
	if (steelPlatePoints)
	{
		m_Controls.textBrowser_Action->append("load SteelPlatePointSet.");
		m_Controls.checkBox_point14->setChecked(true);
		steelPlatePoints->SetVisibility(false);
	}
	if (SawPoints)
	{
		m_Controls.textBrowser_Action->append("load SawPoints.");
		m_Controls.checkBox_point15->setChecked(true);
		SawPoints->SetVisibility(false);
	
	}
	if (DrillPoints)
	{
		m_Controls.textBrowser_Action->append("load DrillPointSet.");
		m_Controls.checkBox_point16->setChecked(true);
		DrillPoints->SetVisibility(false);

	}
	if (LinkPin)
	{
		m_Controls.textBrowser_Action->append("load LinkPin.");
		m_Controls.checkBox_point17->setChecked(true);
		LinkPin->SetVisibility(false);

	}
	if (LinkPinLandmark)
	{
		m_Controls.textBrowser_Action->append("load LinkPinLandMarkPointSet.");
		m_Controls.checkBox_point17->setChecked(true);
		LinkPinLandmark->SetVisibility(false);

	}
	if (Probe)
	{
		m_Controls.textBrowser_Action->append("load Probe.");
		m_Controls.checkBox_point18->setChecked(true);
		Probe->SetVisibility(false);

	}
	if (ProbeLandmark)
	{
		m_Controls.textBrowser_Action->append("load ProbeLandmark.");
		m_Controls.checkBox_point19->setChecked(true);
		ProbeLandmark->SetVisibility(false);

	}

	GetDataStorage()->Modified();
	mitk::RenderingManager::GetInstance()->RequestUpdateAll();

	return true;
}

// �ع���滮
bool HTONDI::OnGenerateCutPlaneClicked()
{
	/* ���ɽع���
	1. ˮƽ�ع���Ϊ��ʼλ��
	2. �����ع�����ˮƽ��滮֮������

	�������ʱ��ɾ���Ѿ����ɵ�ƽ������������ɣ�����ֱ�Ӹ�������
	*/
	// ���ɽع�ƽ��
	m_Controls.textBrowser_Action->append("Action: Generate Cut Plane.");

	// 1. ˮƽ�ع�
	if (m_Controls.cutAxialTibet_radioButton->isChecked())
	{
		if (CreateOneCutPlane())
		{
			// ����ɾ��ͬ����ƽ��
			auto cutPlane = GetDataStorage()->GetNamedNode("1st cut plane");
			if (cutPlane) {
				GetDataStorage()->Remove(cutPlane);
			}
			// ����ƽ������ýع�ƽ��Ϊ ��ɫ 0.5͸����
			auto cutSurfaceNode = GetDataStorage()->GetNamedNode("tibia cut plane");
			cutSurfaceNode->SetColor(0, 1, 0);
			cutSurfaceNode->SetOpacity(0.5);
			cutSurfaceNode->SetName("1st cut plane");
			return true;
		}
	}

	// 2. �����ع�
	if (m_Controls.cutSagTibet_radioButton->isChecked())
	{
		// ɾ��ͬ��ƽ��
		auto cutPlane_2 = GetDataStorage()->GetNamedNode("2nd cut plane");
		if (cutPlane_2) {
			GetDataStorage()->Remove(cutPlane_2);
		}


		// ���ڵ�ǰˮƽ�ع���Ĺ滮�����������ع���
		auto cutPlaneNode = GetDataStorage()->GetNamedNode("1st cut plane");
		auto mitkCutPlane_0 = dynamic_cast<mitk::Surface*>(cutPlaneNode->GetData());
		auto tmpVtkSurface_initial = mitkCutPlane_0->GetVtkPolyData();

		vtkNew<vtkTransform> cutPlaneTransform;
		cutPlaneTransform->SetMatrix(mitkCutPlane_0->GetGeometry()->GetVtkMatrix());

		vtkNew<vtkTransformFilter> cutPlaneTransformFilter;
		cutPlaneTransformFilter->SetTransform(cutPlaneTransform);
		cutPlaneTransformFilter->SetInputData(tmpVtkSurface_initial);
		cutPlaneTransformFilter->Update();

		// ������ƽ��
		vtkNew<vtkPolyData> tmpVtkSurface;
		tmpVtkSurface->DeepCopy(cutPlaneTransformFilter->GetPolyDataOutput());

		auto cutSurface1 = mitk::Surface::New();
		cutSurface1->SetVtkPolyData(tmpVtkSurface);

		auto planeNode1 = mitk::DataNode::New();
		planeNode1->SetData(cutSurface1);
		planeNode1->SetName("2 cut plane");


		GetDataStorage()->Add(planeNode1);
		auto cutSurfaceNode2 = GetDataStorage()->GetNamedNode("2 cut plane");
		cutSurfaceNode2->SetColor(1, 0, 0);
		cutSurfaceNode2->SetOpacity(0.5);
		cutSurfaceNode2->SetName("2nd cut plane");

		// ������ת����
		double vx, vy, vz, length;
		double angle;
		// 0:����. 1������
		// ������ת�ᵥλ����
		if (judgModel_flag == 0) {
			cout << "Left Limb" << endl;
			// �����Ƚ�����ת��Ӧ���ǵ�0->��1
			double vx_1 = mitkPointSet1->GetPoint(2)[0] - mitkPointSet1->GetPoint(1)[0];
			double vy_1 = mitkPointSet1->GetPoint(2)[1] - mitkPointSet1->GetPoint(1)[1];
			double vz_1 = mitkPointSet1->GetPoint(2)[2] - mitkPointSet1->GetPoint(1)[2];
			double length_1 = sqrt(pow(vx_1, 2) + pow(vy_1, 2) + pow(vz_1, 2));
			vx = vx_1;
			vy = vy_1;
			vz = vz_1;
			length = length_1;
			// ����: B->A, ˳ʱ��110��
			angle = 110.0;
		}
		else if (judgModel_flag == 1) {
			cout << "Right Limb" << endl;
			// ����������ת����
			// �ع���ԳƱ仯��ԭ���Ľع������õ�
			double vx_2 = mitkPointSet1->GetPoint(1)[0] - mitkPointSet1->GetPoint(2)[0];
			double vy_2 = mitkPointSet1->GetPoint(1)[1] - mitkPointSet1->GetPoint(2)[1];
			double vz_2 = mitkPointSet1->GetPoint(1)[2] - mitkPointSet1->GetPoint(2)[2];
			double length_2 = sqrt(pow(vx_2, 2) + pow(vy_2, 2) + pow(vz_2, 2));
			vx = vx_2;
			vy = vy_2;
			vz = vz_2;
			length = length_2;
			// ����: A->B, ��ʱ��110��
			angle = -110.0;
		}

		// ��ת������׼��
		double direction[3]{ vx / length, vy / length, vz / length };

		// ����ع������ĵ�
		double center[3];
		center[0] = (mitkPointSet1->GetPoint(1)[0] + mitkPointSet1->GetPoint(2)[0]) / 2.0;
		center[1] = (mitkPointSet1->GetPoint(1)[1] + mitkPointSet1->GetPoint(2)[1]) / 2.0;
		center[2] = (mitkPointSet1->GetPoint(1)[2] + mitkPointSet1->GetPoint(2)[2]) / 2.0;

		// �Խع��������ת
		auto tibiaNode = GetDataStorage()->GetNamedNode("2nd cut plane");
		Rotate(center, direction, angle, tibiaNode->GetData());
	}

	// ��� ��������־�����չʾ
	// ֪ͨ���ݴ洢��Ҫ�������ڲ�״̬���Ա�����Ҫʱ��ȷ�������ݽڵ�ĸ���
	// ����ɾ��
	GetDataStorage()->Modified();

	mitk::RenderingManager::GetInstance()->RequestUpdateAll();
	return true;

}

bool HTONDI::OnCutTibetClicked()
{
	/* ���нعǼ���
	1. ��ƽ��ع�
	2. ˫ƽ��ع�

	*/

	// Surface Segementation
	m_Controls.textBrowser_Action->append("Action: Generate Cutting Surface.");

	// ���нعǼ��� 
	auto AxialPlane = GetDataStorage()->GetNamedNode("1st cut plane");
	auto SagPlane = GetDataStorage()->GetNamedNode("2nd cut plane");

	// ���нع�
	if (AxialPlane != nullptr && SagPlane == nullptr)
	{
		CutTibiaWithOnePlane();
	}
	else if (AxialPlane != nullptr && SagPlane != nullptr) {
		CutTibiaWithTwoPlanes();
	}
	else {
		m_Controls.textBrowser_Action->append("Please Modify Cut first!");
		return false;
	}
	// 
	auto proximalTibiaSurface = GetDataStorage()->GetNamedNode("proximal tibiaSurface");
	auto distalTibiaSurface = GetDataStorage()->GetNamedNode("distal tibiaSurface");
	// ��¼�������ʼλ�ã����ں������ò���
	m_distalTibiaPosition = distalTibiaSurface->GetData()->GetGeometry()->GetOrigin();
	SetModelOpacity(proximalTibiaSurface, 0.5f);
	SetModelOpacity(distalTibiaSurface, 0.5f);
	// �عǺ����ϸ���
	GetDistancefromTibia();
	return true;
}

bool HTONDI::OnResetCutClicked()
{
	// ���ýعǲ���
	m_Controls.textBrowser_Action->append("Action: Reset Cut Plane.");
	// ���ĳ�����󲻴��ڣ�����Remove�����������������Ϊ�ڳ����Ƴ������ڵĶ���ʱ��������򵥵غ��Ըò���
	GetDataStorage()->Remove(GetDataStorage()->GetNamedNode("1st cut plane"));
	GetDataStorage()->Remove(GetDataStorage()->GetNamedNode("2nd cut plane"));
	GetDataStorage()->Remove(GetDataStorage()->GetNamedNode("distal tibiaSurface"));
	GetDataStorage()->Remove(GetDataStorage()->GetNamedNode("proximal tibiaSurface"));
	GetDataStorage()->Remove(GetDataStorage()->GetNamedNode("pointSetInPlaneCutPlane1"));
	GetDataStorage()->Remove(GetDataStorage()->GetNamedNode("planeAndTibiaIntersectionPoint"));

	GetDataStorage()->Remove(GetDataStorage()->GetNamedNode("legForceLineNew"));
	GetDataStorage()->Remove(GetDataStorage()->GetNamedNode("ankleCenterPoint"));
	GetDataStorage()->Remove(GetDataStorage()->GetNamedNode("tibiaDistalPointSet"));

	// �ָ��ֹǵĿɼ���
	mitk::DataNode::Pointer tibiaSurface = GetDataStorage()->GetNamedNode("tibiaSurface");
	if (tibiaSurface.IsNotNull())
	{
		// �л���֫���߽ڵ�Ŀɼ���
		// bool currentVisibility = tibiaSurface->IsVisible(nullptr, "visible", true);
		tibiaSurface->SetVisibility(true);
	}
	return true;
}

bool HTONDI::CreateOneCutPlane()
{
	// �����ع�ƽ��
	auto dataNode_tibiaSurface = GetDataStorage()->GetNamedNode("tibiaSurface");

	// �������Ƿ������Ҫ������
	if (dataNode_tibiaSurface == nullptr)
	{
		m_Controls.textBrowser_Action->append("tibiaSurface is missing");
		return false;
	}

	// ��ȡ�ֹǱ��������
	auto tibiaSurface = dynamic_cast<mitk::Surface*>(dataNode_tibiaSurface->GetData());
	// ��ȡ�ֹǱ����PolyData�����ڼ��μ���
	auto initialTibiaPolyData = tibiaSurface->GetVtkPolyData();

	// �������α任����ʵ��
	vtkNew<vtkTransform> tibiaTransform;
	tibiaTransform->SetMatrix(tibiaSurface->GetGeometry()->GetVtkMatrix());

	// �����任������ʵ��
	vtkNew<vtkTransformFilter> tmpFilter;
	tmpFilter->SetTransform(tibiaTransform);
	tmpFilter->SetInputData(initialTibiaPolyData);
	tmpFilter->Update();

	// ȡ���任�������ʵ��
	// ����������Ϊ��ȷ����ԭʼ���ݵı任��������Ӱ�쵽ԭʼ���ݱ���
	vtkNew<vtkPolyData> tibiaPolyData;
	tibiaPolyData->DeepCopy(tmpFilter->GetPolyDataOutput());

	// ������Χ��ʵ��
	vtkNew<vtkOBBTree> obbTree;
	obbTree->SetDataSet(tibiaPolyData);
	obbTree->SetMaxLevel(2);
	obbTree->BuildLocator();

	// corner OBB�Ľǵ�����λ��
	// max OBB���-����
	// mid OBB�γ���-����
	// min OBB��̱�-����
	//��ȡ����Χ��������ݼ���OBB�����йؼ���Ϣ����������λ�á���С�ͷ���
	double corner[3], max[3], mid[3], min[3], size[3];
	obbTree->ComputeOBB(tibiaPolyData, corner, max, mid, min, size);


	//=====================OBB���ӻ�==============================
	if (OBB) {
		// ��ȡ����
		vtkNew<vtkPolyData> obbPolydata;
		obbTree->GenerateRepresentation(0, obbPolydata);
		// ������ʱ����
		auto tmpNode = mitk::DataNode::New();
		auto tmpSurface = mitk::Surface::New();
		tmpSurface->SetVtkPolyData(obbPolydata);
		tmpNode->SetData(tmpSurface);
		tmpNode->SetOpacity(0.2);
		tmpNode->SetName("OBB");
		// ����£����ӻ�
		GetDataStorage()->Add(tmpNode);
	}
	//=====================OBB���ӻ�==============================


	// ����һ����ʼ�Ľع�ƽ�棬������λ�úͷ���
	auto cutPlaneSource = vtkSmartPointer<vtkPlaneSource>::New();
	// ƽ���ԭ������
	cutPlaneSource->SetOrigin(0, 0, 0);
	// �趨���ӻ��ع�ƽ��Ĵ�С��70*70
	cutPlaneSource->SetPoint1(0, 70, 0);
	cutPlaneSource->SetPoint2(70, 0, 0);
	//���÷�����
	// cutPlaneSource->SetNormal(max);
	cutPlaneSource->SetNormal(0, 0, 1);//���÷�����

	// ����ع�ĩ�˺�ҳ��ĳ�ʼ����
	mitk::Point3D point0;//ˮƽ������е�
	mitk::Point3D point1;//ˮƽ�����½�
	mitk::Point3D point2;//ˮƽ�����½�
	mitk::Point3D point3;//ˮƽ���ұ��е�
	mitk::Point3D planeCenterPoint;//ˮƽ�����ĵ�

	// ȷ�����Žع�λ��

	// �洢�عǵõ�����������
	// �ع��� 01 - 0.85
	vtkNew<vtkPolyData> largerSubpart_0;
	vtkNew<vtkPolyData> smallerSubpart_0;
	//�µĽع�ƽ�����ĵ㣬ͨ�������м������ݼ�Ȩƽ���õ�
	double origin_0[3]
	{
		corner[0] + 0.5 * (1.7 * max[0] + mid[0] + min[0]),
		corner[1] + 0.5 * (1.7 * max[1] + mid[1] + min[1]),
		corner[2] + 0.5 * (1.7 * max[2] + mid[2] + min[2])
	};

	// �ع��� 02 - 0.15
	vtkNew<vtkPolyData> largerSubpart_1;
	vtkNew<vtkPolyData> smallerSubpart_1;
	double origin_1[3]
	{
		corner[0] + 0.5 * (0.3 * max[0] + mid[0] + min[0]),
		corner[1] + 0.5 * (0.3 * max[1] + mid[1] + min[1]),
		corner[2] + 0.5 * (0.3 * max[2] + mid[2] + min[2])
	};


	//ִ�нعǲ���
	CutPolyDataWithPlane(tibiaPolyData, largerSubpart_0, smallerSubpart_0, origin_0, max);
	CutPolyDataWithPlane(tibiaPolyData, largerSubpart_1, smallerSubpart_1, origin_1, max);

	double origin_final[3];
	if (smallerSubpart_0->GetNumberOfCells() >= smallerSubpart_1->GetNumberOfCells())
	{
		// ѡ����ѵķ���������
		origin_final[0] = origin_0[0];
		origin_final[1] = origin_0[1];
		origin_final[2] = origin_0[2];
	}
	else
	{
		origin_final[0] = origin_1[0];
		origin_final[1] = origin_1[1];
		origin_final[2] = origin_1[2];
	}

	// ����ʼ�����Ľع�ƽ���ƶ������ŵ�λ������
	cutPlaneSource->SetCenter(origin_final);
	/*
	 ----------------
	 |              |
	 |              |
	 1       5      4
	 |              |
	 |              |
	 2--------------3
	*/

	// �� 1 - ĩ���е� 
	point0[0] = origin_final[0] - 35;
	point0[1] = origin_final[1];
	point0[2] = origin_final[2];
	// �� 2 - ���½�
	point1[0] = origin_final[0] - 35;
	point1[1] = origin_final[1] - 35;
	point1[2] = origin_final[2];
	// �� 3 - ���½�
	point2[0] = origin_final[0] + 35;
	point2[1] = origin_final[1] - 35;
	point2[2] = origin_final[2];
	// �� 4 - ĩ���е� 
	point3[0] = origin_final[0] + 35;
	point3[1] = origin_final[1];
	point3[2] = origin_final[2];
	// �� 5 - ����ԭ��
	planeCenterPoint[0] = origin_final[0];
	planeCenterPoint[1] = origin_final[1];
	planeCenterPoint[2] = origin_final[2];

	// ��������ӵ�mitk::PointSet��
	// ���������ع������ת����
	mitkPointSet1->InsertPoint(0, point0);//��һ�ع���ĩ���е���
	mitkPointSet1->InsertPoint(1, point1);//ƽ�����Ͻ�
	mitkPointSet1->InsertPoint(2, point2);//ƽ�����½�
	mitkPointSet1->InsertPoint(3, point3);//ƽ���ұ��е�
	mitkPointSet1->InsertPoint(4, planeCenterPoint);//ƽ��ԭ��

	// ɾ���ϴ����ɵĵ�
	auto tmpNodes = GetDataStorage()->GetNamedNode("pointSetInPlaneCutPlane1");
	if (tmpNodes) {
		GetDataStorage()->Remove(tmpNodes);
	}

	mitk::DataNode::Pointer pointSetInPlaneCutPlane = mitk::DataNode::New();
	pointSetInPlaneCutPlane->SetName("pointSetInPlaneCutPlane1");
	// ��ɫ����С 5.0
	pointSetInPlaneCutPlane->SetColor(1.0, 0.0, 0.0);
	pointSetInPlaneCutPlane->SetData(mitkPointSet1);
	pointSetInPlaneCutPlane->SetFloatProperty("pointsize", 5.0);
	GetDataStorage()->Add(pointSetInPlaneCutPlane);

	// ͼ�����
	cutPlaneSource->Update();

	// ��ȡ�ع�ƽ�������
	auto cutSurface = mitk::Surface::New();
	cutSurface->SetVtkPolyData(cutPlaneSource->GetOutput());

	// �����ع�ƽ��DataNode����
	auto planeNode = mitk::DataNode::New();
	planeNode->SetData(cutSurface);
	planeNode->SetName("tibia cut plane");

	// ��DataNode��ӵ����ݴ洢��
	GetDataStorage()->Add(planeNode);

	return true;
}

bool HTONDI::CutTibiaWithOnePlane()
{
	// ��ƽ��عǺ���
	m_Controls.textBrowser_Action->append("Cutting with one plane");
	auto cutPlaneNode = GetDataStorage()->GetNamedNode("1st cut plane");
	auto tibiaSurfaceNode = GetDataStorage()->GetNamedNode("tibiaSurface");

	// ����Ƿ��������Ҫ������
	if (cutPlaneNode == nullptr)
	{
		m_Controls.textBrowser_Action->append("'tibia cut plane' is not ready");
		return false;
	}
	if (tibiaSurfaceNode == nullptr)
	{
		m_Controls.textBrowser_Action->append("'tibiaSurface' is not ready");
		return false;
	}

	// ��ȡ �ع�ƽ�� �� ԭʼ��ƽ�� ������
	// ���е�ԭʼ����
	auto mitkCutSurface = dynamic_cast<mitk::Surface*>(cutPlaneNode->GetData());
	auto tibiaMitkSurface = dynamic_cast<mitk::Surface*>(tibiaSurfaceNode->GetData());
	// ���е�PolyData
	auto tmpVtkSurface_initial = mitkCutSurface->GetVtkPolyData();
	auto tibiaVtkSurface_initial = tibiaMitkSurface->GetVtkPolyData();

	// �ع���
	//=======================================================================
	// ����ת������ʵ��
	vtkNew<vtkTransform> cutPlaneTransform;
	cutPlaneTransform->SetMatrix(mitkCutSurface->GetGeometry()->GetVtkMatrix());
	// ����ת��ʵ��
	vtkNew<vtkTransformFilter> cutPlaneTransformFilter;
	cutPlaneTransformFilter->SetTransform(cutPlaneTransform);
	cutPlaneTransformFilter->SetInputData(tmpVtkSurface_initial);
	cutPlaneTransformFilter->Update();
	// �������
	vtkNew<vtkPolyData> tmpVtkSurface;
	tmpVtkSurface->DeepCopy(cutPlaneTransformFilter->GetPolyDataOutput());
	//=======================================================================
	// �ֹǱ���
	//=======================================================================
	// ����ת������ʵ��
	vtkNew<vtkTransform> tibiaTransform;
	tibiaTransform->SetMatrix(tibiaMitkSurface->GetGeometry()->GetVtkMatrix());
	// ����ת��ʵ��
	vtkNew<vtkTransformFilter> tibiaTransformFilter;
	tibiaTransformFilter->SetTransform(tibiaTransform);
	tibiaTransformFilter->SetInputData(tibiaVtkSurface_initial);
	tibiaTransformFilter->Update();
	// �������
	vtkNew<vtkPolyData> tibiaVtkSurface;
	tibiaVtkSurface->DeepCopy(tibiaTransformFilter->GetPolyDataOutput());
	//=======================================================================

	// �Ӹ�����ƽ����������ȡ�������������ĵ�����
	double cutPlaneCenter[3];
	double surfaceNormal[3];

	// ��ȡ����
	GetPlaneProperty(tmpVtkSurface, surfaceNormal, cutPlaneCenter);

	// ����ͼ��ָ�
	vtkNew<vtkPolyData> proximalTibiaSurface;
	vtkNew<vtkPolyData> distalTibiaSurface;

	CutPolyDataWithPlane(tibiaVtkSurface, distalTibiaSurface, proximalTibiaSurface, cutPlaneCenter, surfaceNormal);

	// ��ȡ�������ֵ�PloyData
	auto mitkProximalSurface = mitk::Surface::New();
	auto mitkDistalSurface = mitk::Surface::New();
	mitkProximalSurface->SetVtkPolyData(proximalTibiaSurface);
	mitkDistalSurface->SetVtkPolyData(distalTibiaSurface);

	// ����ֲڻ��������ڼ���ʵʱ��Ⱦ
	auto proximal_remehsed = mitk::Remeshing::Decimate(mitkProximalSurface, 1, true, true);
	auto distal_remehsed = mitk::Remeshing::Decimate(mitkDistalSurface, 1, true, true);

	// ��ն�-����
	vtkNew<vtkFillHolesFilter> holeFiller0;
	holeFiller0->SetInputData(proximal_remehsed->GetVtkPolyData());
	holeFiller0->SetHoleSize(500);
	holeFiller0->Update();
	// ��ȡ���ı�������
	vtkNew<vtkPolyData> proximalTibia;
	proximalTibia->DeepCopy(holeFiller0->GetOutput());
	auto proximalSurface = mitk::Surface::New();
	proximalSurface->SetVtkPolyData(proximalTibia);

	// ��ն�-Զ��
	vtkNew<vtkFillHolesFilter> holeFiller1;
	holeFiller1->SetInputData(distal_remehsed->GetVtkPolyData());
	holeFiller1->SetHoleSize(500);
	holeFiller1->Update();
	// ��ȡ���ı�������
	vtkNew<vtkPolyData> distalTibia;
	distalTibia->DeepCopy(holeFiller1->GetOutput());
	auto distalSurface = mitk::Surface::New();
	distalSurface->SetVtkPolyData(distalTibia);

	// Զ��
	// ����Ƿ����
	auto checktmp01 = GetDataStorage()->GetNamedNode("proximal tibiaSurface");
	if (checktmp01)
	{
		GetDataStorage()->Remove(GetDataStorage()->GetNamedNode("proximal tibiaSurface"));
	}
	// �½�
	auto tmpNode0 = mitk::DataNode::New();
	tmpNode0->SetData(proximalSurface);
	tmpNode0->SetName("proximal tibiaSurface");
	// ���й⻬ƽ����Ⱦ
	mitk::VtkInterpolationProperty::Pointer interpolationProp0;
	tmpNode0->GetProperty(interpolationProp0, "material.interpolation");
	interpolationProp0->SetInterpolationToFlat();
	tmpNode0->SetProperty("material.interpolation", interpolationProp0);

	// ����
	// ����Ƿ����
	auto checktmp02 = GetDataStorage()->GetNamedNode("distal tibiaSurface");
	if (checktmp02)
	{
		GetDataStorage()->Remove(GetDataStorage()->GetNamedNode("distal tibiaSurface"));
	}
	auto tmpNode1 = mitk::DataNode::New();
	tmpNode1->SetData(distalSurface);
	tmpNode1->SetName("distal tibiaSurface");
	// ���й⻬ƽ����Ⱦ
	mitk::VtkInterpolationProperty::Pointer interpolationProp1;
	tmpNode1->GetProperty(interpolationProp1, "material.interpolation");
	interpolationProp1->SetInterpolationToFlat();
	tmpNode1->SetProperty("material.interpolation", interpolationProp1);

	// ����ԭ�е��ֹ�Ϊ���ɼ�
	mitk::DataNode::Pointer tibiaSurface = GetDataStorage()->GetNamedNode("tibiaSurface");
	if (tibiaSurface.IsNotNull())
	{
		// �л���֫���߽ڵ�Ŀɼ���
		bool currentVisibility = tibiaSurface->IsVisible(nullptr, "visible", true);
		tibiaSurface->SetVisibility(!currentVisibility);
	}

	GetDataStorage()->Add(tmpNode0);
	GetDataStorage()->Add(tmpNode1);

	return true;
}

bool HTONDI::CutTibiaWithTwoPlanes()
{
	m_Controls.textBrowser_Action->append("Cutting with two plane");
	// ͬʱʹ������ƽ����нع�
	auto cutplane_0 = GetDataStorage()->GetNamedNode("1st cut plane");
	auto cutplane_1 = GetDataStorage()->GetNamedNode("2nd cut plane");
	auto tibiaNode = GetDataStorage()->GetNamedNode("tibiaSurface");

	// ����Ƿ��������Ҫ������
	if (cutplane_0 == nullptr || cutplane_1 == nullptr)
	{
		m_Controls.textBrowser_Action->append("'1st cut plane' or '2nd cut plane' is not ready");
		return false;
	}
	if (tibiaNode == nullptr)
	{
		m_Controls.textBrowser_Action->append("'tibiaSurface' is not ready");
		return false;
	}

	auto mitkCutPlane_0 = dynamic_cast<mitk::Surface*>(cutplane_0->GetData());
	auto mitkCutPlane_1 = dynamic_cast<mitk::Surface*>(cutplane_1->GetData());
	auto mitkTibia = dynamic_cast<mitk::Surface*>(tibiaNode->GetData());

	vtkNew<vtkPolyData> vtkCutPlane_0;
	vtkNew<vtkPolyData> vtkCutPlane_1;
	vtkNew<vtkPolyData> vtkTibia;

	// �ع���- Axial
	vtkNew<vtkTransform> cutPlaneTransform_0;
	cutPlaneTransform_0->SetMatrix(mitkCutPlane_0->GetGeometry()->GetVtkMatrix());
	vtkNew<vtkTransformFilter> cutPlaneTransformFilter_0;
	cutPlaneTransformFilter_0->SetTransform(cutPlaneTransform_0);
	cutPlaneTransformFilter_0->SetInputData(mitkCutPlane_0->GetVtkPolyData());
	cutPlaneTransformFilter_0->Update();
	vtkCutPlane_0->DeepCopy(cutPlaneTransformFilter_0->GetPolyDataOutput());

	// �ع���- Sag
	vtkNew<vtkTransform> cutPlaneTransform_1;
	cutPlaneTransform_1->SetMatrix(mitkCutPlane_1->GetGeometry()->GetVtkMatrix());
	vtkNew<vtkTransformFilter> cutPlaneTransformFilter_1;
	cutPlaneTransformFilter_1->SetTransform(cutPlaneTransform_1);
	cutPlaneTransformFilter_1->SetInputData(mitkCutPlane_1->GetVtkPolyData());
	cutPlaneTransformFilter_1->Update();
	vtkCutPlane_1->DeepCopy(cutPlaneTransformFilter_1->GetPolyDataOutput());

	// �Ǳ���
	vtkNew<vtkTransform> tibiaTransform;
	tibiaTransform->SetMatrix(mitkTibia->GetGeometry()->GetVtkMatrix());
	vtkNew<vtkTransformFilter> tibiaTransformFilter;
	tibiaTransformFilter->SetTransform(tibiaTransform);
	tibiaTransformFilter->SetInputData(mitkTibia->GetVtkPolyData());
	tibiaTransformFilter->Update();
	vtkTibia->DeepCopy(tibiaTransformFilter->GetPolyDataOutput());

	// ��ȡ�����ع���ķ�������ƽ������
	double cutPlaneCenter_0[3];
	double cutPlaneNormal_0[3];
	double cutPlaneCenter_1[3];
	double cutPlaneNormal_1[3];
	GetPlaneProperty(vtkCutPlane_0, cutPlaneNormal_0, cutPlaneCenter_0);
	GetPlaneProperty(vtkCutPlane_1, cutPlaneNormal_1, cutPlaneCenter_1);

	vtkNew<vtkPolyData> largetPart;
	vtkNew<vtkPolyData> tmpMiddlePart;
	vtkNew<vtkPolyData> middlePart;
	vtkNew<vtkPolyData> smallPart;

	// Part01: ��ʹ��Axial�عǣ��ﵽ A-��-С ��������
	// Part02: ȡ�� A-С������Sag�عǣ��õ� S-��-С ��������
	// Part03: �ϲ� S-С �� A-��������Ϊ A-�� + S-��

	// Part01:
	CutPolyDataWithPlane(vtkTibia, largetPart, tmpMiddlePart, cutPlaneCenter_0, cutPlaneNormal_0);
	// Part02:
	CutPolyDataWithPlane(tmpMiddlePart, middlePart, smallPart, cutPlaneCenter_1, cutPlaneNormal_1);

	// Part03
	vtkSmartPointer<vtkAppendPolyData> appendFilter =
		vtkSmartPointer<vtkAppendPolyData>::New();
	vtkSmartPointer<vtkCleanPolyData> cleanFilter =
		vtkSmartPointer<vtkCleanPolyData>::New();

	// �ϲ� A-�� �� S-С
	appendFilter->AddInputData(largetPart);
	appendFilter->AddInputData(smallPart);
	appendFilter->Update();

	cleanFilter->SetInputData(appendFilter->GetOutput());
	cleanFilter->Update();

	auto proximalSurface = mitk::Surface::New();
	auto distalSurface = mitk::Surface::New();

	proximalSurface->SetVtkPolyData(middlePart);
	distalSurface->SetVtkPolyData(cleanFilter->GetOutput());

	//auto proximal_remehsed = mitk::Remeshing::Decimate(proximalSurface, 1, true, true);
	//auto distal_remehsed = mitk::Remeshing::Decimate(distalSurface, 1, true, true);

	// ����Ƿ����
	auto checktmp01 = GetDataStorage()->GetNamedNode("proximal tibiaSurface");
	if (checktmp01)
	{
		GetDataStorage()->Remove(GetDataStorage()->GetNamedNode("proximal tibiaSurface"));
	}
	// ����Ƿ����
	auto checktmp02 = GetDataStorage()->GetNamedNode("distal tibiaSurface");
	if (checktmp02)
	{
		GetDataStorage()->Remove(GetDataStorage()->GetNamedNode("distal tibiaSurface"));
	}
	// �½�

	auto proximalNode = mitk::DataNode::New();
	auto distalNode = mitk::DataNode::New();

	proximalNode->SetName("proximal tibiaSurface");
	proximalNode->SetData(proximalSurface);
	proximalNode->SetColor(0, 1, 0);

	distalNode->SetName("distal tibiaSurface");
	distalNode->SetData(distalSurface);
	//distalNode->SetColor(0,0,1);

	GetDataStorage()->Add(distalNode);
	GetDataStorage()->Add(proximalNode);

	// ����ԭ�е��ֹ�Ϊ���ɼ�
	mitk::DataNode::Pointer tibiaSurface = GetDataStorage()->GetNamedNode("tibiaSurface");
	if (tibiaSurface.IsNotNull())
	{
		// �л���֫���߽ڵ�Ŀɼ���
		tibiaSurface->SetVisibility(false);
	}

	GetDataStorage()->Modified();
	mitk::RenderingManager::GetInstance()->RequestUpdateAll();

	return true;
}

bool HTONDI::GetPlaneProperty(vtkSmartPointer<vtkPolyData> plane, double normal[3], double center[3])
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

	return true;
}





bool HTONDI::CutPolyDataWithPlane(vtkSmartPointer<vtkPolyData> dataToCut,
	vtkSmartPointer<vtkPolyData> largerSubPart,
	vtkSmartPointer<vtkPolyData> smallerSubPart,
	double planeOrigin[3], double planeNormal[3])
{
	/* ���нعǼ���
	����ԭ���ݡ��зֺ�Ĵ�С���ݴ洢�塢�ع������ġ��ع��淨����
	������������ݽ��н�ȡ�������зֺ�����ݣ��ֽ�Ϊ�������С����

	*/

	// ����һ��vtkPlane��������ƽ��ķ�������ԭ��
	vtkNew<vtkPlane> implicitPlane;
	implicitPlane->SetNormal(planeNormal);
	implicitPlane->SetOrigin(planeOrigin);

	// ����һ��vtkClipPolyData���������������ݲ����ݸ���ƽ������и�
	vtkNew<vtkClipPolyData> clipper;
	clipper->SetInputData(dataToCut);
	clipper->GenerateClippedOutputOn();
	clipper->SetClipFunction(implicitPlane);
	clipper->Update();

	// �����и������ݵ��µ�vtkPolyData������
	vtkNew<vtkPolyData> tibiaPart_0;
	tibiaPart_0->DeepCopy(clipper->GetClippedOutput());
	// ��ȡ�����Ķ��������
	int cellNum_0 = tibiaPart_0->GetNumberOfCells();

	// ����clipper����ȡ��һ�����и�����
	clipper->Update();
	vtkNew<vtkPolyData> tibiaPart_1;
	tibiaPart_1->DeepCopy(clipper->GetOutput());
	// ��ȡ�����Ķ��������
	int cellNum_1 = tibiaPart_1->GetNumberOfCells();

	// ʹ��vtkFillHolesFilter����ܴ��ڵĿն�
	// �� tibiaPart_0 ������ͬ�Ŀն������
	vtkNew<vtkFillHolesFilter> holeFiller0;
	holeFiller0->SetInputData(tibiaPart_0);
	holeFiller0->SetHoleSize(500);
	holeFiller0->Update();
	vtkNew<vtkPolyData> tibia_filled_0;
	tibia_filled_0->DeepCopy(holeFiller0->GetOutput());

	// �� tibiaPart_1 ������ͬ�Ŀն������
	vtkNew<vtkFillHolesFilter> holeFiller1;
	holeFiller1->SetInputData(tibiaPart_1);
	holeFiller1->SetHoleSize(500);
	holeFiller1->Update();
	vtkNew<vtkPolyData> tibia_filled_1;
	tibia_filled_1->DeepCopy(holeFiller1->GetOutput());

	// ���ݶ��������ȷ���Ĳ����Ǵ�ķ������Ĳ�����С�������������ݸ��Ƶ���Ӧ�Ķ�����
	if (cellNum_1 >= cellNum_0)
	{
		largerSubPart->DeepCopy(tibia_filled_1);
		smallerSubPart->DeepCopy(tibia_filled_0);
	}
	else
	{
		largerSubPart->DeepCopy(tibia_filled_0);
		smallerSubPart->DeepCopy(tibia_filled_1);
	}

	return true;
}

// �ְ�滮
bool HTONDI::OnShowSteelClick()
{
	/* ��ʾ/���� �ְ�ģ�� */


	m_Controls.textBrowser_Action->append("Action: Show SteelPlate.");
	auto steelPlate = GetDataStorage()->GetNamedNode("SteelPlate");
	auto steelPlatePoint = GetDataStorage()->GetNamedNode("SteelPlatePointSet");
	if (steelPlate && steelPlatePoint)
	{
		// ��ȡ״̬
		bool currentVisibility = steelPlate->IsVisible(nullptr, "visible", true);
		// չʾ�ְ�͵㼯��
		steelPlate->SetVisibility(!currentVisibility);
		steelPlatePoint->SetVisibility(!currentVisibility);
	}
	else 
	{
		m_Controls.textBrowser_Action->append("SteelPlate or SteelPlatePoint Not Found!");
	}
	GetDataStorage()->Modified();
	mitk::RenderingManager::GetInstance()->RequestUpdateAll();
	return true;
}

bool HTONDI::OnResetSteelClick()
{
	/* ���øְ�ģ�� */


	m_Controls.textBrowser_Action->append("Action: Reset SteelPlate.");
	auto steelPlate = GetDataStorage()->GetNamedNode("SteelPlate");
	if (steelPlate)
	{
		steelPlate->SetVisibility(true);
		steelPlate->GetData()->GetGeometry()->SetOrigin(m_steelPosition);
		// �����ϵĽڵ�Ҳһ��Ǩ��
		auto steelPlatePoints = GetDataStorage()->GetNamedNode("SteelPlatePointSet");
		steelPlatePoints->GetData()->GetGeometry()->SetOrigin(m_steelPosition);
	}
	else
	{
		m_Controls.textBrowser_Action->append("SteelPlate model Not Found!");
	}
	GetDataStorage()->Modified();
	mitk::RenderingManager::GetInstance()->RequestUpdateAll();
	return true;
}

bool HTONDI::OnSetFinalPosClick()
{
	/* ȷ���ְ�����λ�ã�����Ǹֶ�λ�� */

	m_Controls.textBrowser_Action->append("Action: Set Final SteelPlate Pos.");
	
	// ��Ǹֶ��滮��
	// ��֪����Ŀ����˳��Ϊ �� -> �� -> ��
	mitk::DataNode::Pointer steelPlatePoints = GetDataStorage()->GetNamedNode("SteelPlatePointSet");
	if (steelPlatePoints)
	{
		// ����һ���㼯�����ڼ�¼���м�������������λ��
		mitk::PointSet::Pointer holeCenters = mitk::PointSet::New();
		// �� -> �� -> ��
		auto point1 = dynamic_cast<mitk::PointSet*>(steelPlatePoints->GetData())->GetPoint(0);
		auto point2 = dynamic_cast<mitk::PointSet*>(steelPlatePoints->GetData())->GetPoint(1);
		auto point3 = dynamic_cast<mitk::PointSet*>(steelPlatePoints->GetData())->GetPoint(2);
		holeCenters->InsertPoint(0, point1);
		holeCenters->InsertPoint(1, point2);
		holeCenters->InsertPoint(2, point3);

		// ���п��ӻ�
		auto holeCentersNode = GetDataStorage()->GetNamedNode("SteelPlateHolePointSet");
		if (holeCentersNode != nullptr)
		{	
			// ��������ֱ�Ӹ���
			m_Controls.textBrowser_Action->append("Change SteelPlateHole Pos");
			holeCentersNode->SetData(holeCenters);
		}
		else
		{
			// û�е��򴴽���
			m_Controls.textBrowser_Action->append("Init SteelPlateHole Pos");
			mitk::DataNode::Pointer holeCentersNode = mitk::DataNode::New();
			holeCentersNode->SetData(holeCenters);
			holeCentersNode->SetName("SteelPlateHolePointSet");
			holeCentersNode->SetColor(0.0, 0.0, 1.0);
			holeCentersNode->SetProperty("pointsize", mitk::FloatProperty::New(5.0));
			GetDataStorage()->Add(holeCentersNode);
		}
	}
	else
	{
		m_Controls.textBrowser_Action->append("SteelPlatePointSet Not Found!");
	}
	// ����ǵ���в��ɼ���
	if (steelPlatePoints)
	{
		steelPlatePoints->SetVisibility(false);
	}
	else
	{
		m_Controls.textBrowser_Action->append("SteelPlatePointSet model Not Found!");
	}
	GetDataStorage()->Modified();
	mitk::RenderingManager::GetInstance()->RequestUpdateAll();
	return true;
}

// �ſ��Ƕȹ滮
bool HTONDI::OnCaculateStrechAngleClicked()
{
	/* �ƻ������ſ��ǶȲ�Ӧ��Զ����תλ��
	// �����߽�����һ�����ʵ�λ�ã�Ҳ���ǽ����ߴ����ֹ�ƽ̨���е�
	// �����ϣ����ֹ�Զ�˲�����ת������������ߵ��ӳ����ϼ���
	*/

	m_Controls.textBrowser_Action->append("Action: Caculate the Angel.");

	// ��ȡ�ɹ����ĵ㡢�ֹ�ƽ̨�е�
	cout << "test 01" << endl;
	mitk::DataNode::Pointer hipCenter = GetDataStorage()->GetNamedNode("hipCenterPoint");
	auto hipCenterPointSet = dynamic_cast<mitk::PointSet*>(hipCenter->GetData());
	// ���и�ʽת��
	mitk::Point3D hipCenterPoint = hipCenterPointSet->GetPoint(0);

	// ��ȡ�ֹ��׵㼯
	mitk::DataNode::Pointer tibiaLandmarkNode = GetDataStorage()->GetNamedNode("tibiaLandmarkPointSet");
	auto tibiaLandmarkPointSet = dynamic_cast<mitk::PointSet*>(tibiaLandmarkNode->GetData());

	// ��ȡ��һ�͵ڶ�����
	// �ֹǽ�������: tibiaProximalLateralPoint
	// �ֹǽ����ڲ��: tibiaProximalMedialPoint
	mitk::Point3D tibiaProximalLateralPoint = tibiaLandmarkPointSet->GetPoint(1);
	mitk::Point3D tibiaProximalMedialPoint = tibiaLandmarkPointSet->GetPoint(2);

	// �����ֹǽ���(�ֹ�ƽ̨����)�����ĵ�
	mitk::Point3D tibiaProximalCenterPoint;
	tibiaProximalCenterPoint[0] = (tibiaProximalLateralPoint[0] + tibiaProximalMedialPoint[0]) / 2;
	tibiaProximalCenterPoint[1] = (tibiaProximalLateralPoint[1] + tibiaProximalMedialPoint[1]) / 2;
	tibiaProximalCenterPoint[2] = (tibiaProximalLateralPoint[2] + tibiaProximalMedialPoint[2]) / 2;

	cout << "test 02" << endl;
	// ����Ŀ�����ߵ��ӳ���
	// ���ȣ�ȡ���ߵķ���
	mitk::Vector3D newLegForceDirection = tibiaProximalCenterPoint - hipCenterPoint;
	// ���������ĳ���
	double length = sqrt(pow(newLegForceDirection[0], 2) +
		pow(newLegForceDirection[1], 2) +
		pow(newLegForceDirection[2], 2));
	// ��һ������
	if (length != 0) {
		newLegForceDirection[0] /= length;
		newLegForceDirection[1] /= length;
		newLegForceDirection[2] /= length;
	}
	// �������߳��ȼ���Ŀ��������
	// ���ԡ������ķ��� => �������߳��Ȳ��� => ȥ����������ת��Ŀ��λ�õĵ�λ��
	// �������߳��ȼ���Ŀ��������
	mitk::Vector3D transPos;
	transPos[0] = newLegForceDirection[0] * Line_length + hipCenterPoint[0];
	transPos[1] = newLegForceDirection[1] * Line_length + hipCenterPoint[1];
	transPos[2] = newLegForceDirection[2] * Line_length + hipCenterPoint[2];
	std::cout << "Ŀ���ӳ����ϵĵ�C����Ϊ: (" << transPos[0] << ", " << transPos[1] << ", " << transPos[2] << ")" << endl;

	// 0:����. 1������
	if (judgModel_flag == 1) {
		// ����
		cout << "left Limb" << endl;
		std::cout << "��ҳ��Ϊ: (" << mitkPointSet1->GetPoint(3)[0] << ", " << mitkPointSet1->GetPoint(3)[1] << ", " << mitkPointSet1->GetPoint(3)[2] << ")" << endl;
		auto tmp = GetDataStorage()->GetNamedNode("legForceLine");
		if (tmp == nullptr)
		{
			m_Controls.textBrowser_Action->append("legForceLine Not Found.");
			return false;
		}
		auto linePointSet = dynamic_cast<mitk::PointSet*>(GetDataStorage()->GetNamedNode("legForceLine")->GetData());
		//���㵱ǰ���ߵ�������
		double vx_1 = linePointSet->GetPoint(1)[0] - mitkPointSet1->GetPoint(3)[0];//��ǰ�����յ�-��ҳ��
		double vy_1 = linePointSet->GetPoint(1)[1] - mitkPointSet1->GetPoint(3)[1];
		double vz_1 = linePointSet->GetPoint(1)[2] - mitkPointSet1->GetPoint(3)[2];
		double length_1 = sqrt(pow(vx_1, 2) + pow(vy_1, 2) + pow(vz_1, 2));
		double ux_1 = vx_1 / length_1;
		double uy_1 = vy_1 / length_1;
		double uz_1 = vz_1 / length_1;
		double direction1[3]{ ux_1,uy_1,uz_1 };
		//�����������ߵ�����
		double vx_2 = transPos[0] - mitkPointSet1->GetPoint(3)[0];//���������յ�-��ҳ��
		double vy_2 = transPos[1] - mitkPointSet1->GetPoint(3)[1];
		double vz_2 = transPos[2] - mitkPointSet1->GetPoint(3)[2];
		double length_2 = sqrt(pow(vx_2, 2) + pow(vy_2, 2) + pow(vz_2, 2));
		double ux_2 = vx_2 / length_2;
		double uy_2 = vy_2 / length_2;
		double uz_2 = vz_2 / length_2;
		double direction2[3]{ ux_2,uy_2,uz_2 };
		// ����������������֮��ļн�
		double dotProduct = direction1[0] * direction2[0] + direction1[1] * direction2[1] + direction1[2] * direction2[2]; // ���
		double angleInRadians = acos(dotProduct); // ʹ�÷����Һ����õ��ǶȵĻ���ֵ
		double angleInDegrees = round(angleInRadians * (180.0 / M_PI) * 10) / 10; // ����һλС��

		m_Controls.LineEdit_angle->setText(QString::number(angleInDegrees));
		std::cout << "The angle between the two directions is: " << angleInDegrees << " degrees." << std::endl;
		m_Controls.LineEdit_transAngle->setText(QString::number(angleInDegrees));

		CaculateStrechHeigh();
		// �������ת�ᰴ��LinEdit�е���ֵ�����У���ֵ��ǰ�����нǵ�ʱ��ͱ�������
		RotatePlus();
		
	}
	else if (judgModel_flag == 0)
	{
		// ��ǰģ��Ϊ����
		cout << "Right Limb" << endl;
		std::cout << "��ҳ��Ϊ: (" << mitkPointSet1->GetPoint(0)[0] << ", " << mitkPointSet1->GetPoint(0)[1] << ", " << mitkPointSet1->GetPoint(0)[2] << ")" << endl;
		auto tmp = GetDataStorage()->GetNamedNode("legForceLine");
		if (tmp == nullptr)
		{
			m_Controls.textBrowser_Action->append("legForceLine Not Found.");
			return false;
		}
		auto linePointSet = dynamic_cast<mitk::PointSet*>(GetDataStorage()->GetNamedNode("legForceLine")->GetData());
		//���㵱ǰ���ߵ�������
		double vx_1 = linePointSet->GetPoint(1)[0] - mitkPointSet1->GetPoint(0)[0];//��ǰ�����յ�-��ҳ��
		double vy_1 = linePointSet->GetPoint(1)[1] - mitkPointSet1->GetPoint(0)[1];
		double vz_1 = linePointSet->GetPoint(1)[2] - mitkPointSet1->GetPoint(0)[2];
		double length_1 = sqrt(pow(vx_1, 2) + pow(vy_1, 2) + pow(vz_1, 2));
		double ux_1 = vx_1 / length_1;
		double uy_1 = vy_1 / length_1;
		double uz_1 = vz_1 / length_1;
		double direction1[3]{ ux_1,uy_1,uz_1 };
		//�����������ߵ�����
		double vx_2 = transPos[0] - mitkPointSet1->GetPoint(0)[0];//���������յ�-��ҳ��
		double vy_2 = transPos[1] - mitkPointSet1->GetPoint(0)[1];
		double vz_2 = transPos[2] - mitkPointSet1->GetPoint(0)[2];
		double length_2 = sqrt(pow(vx_2, 2) + pow(vy_2, 2) + pow(vz_2, 2));
		double ux_2 = vx_2 / length_2;
		double uy_2 = vy_2 / length_2;
		double uz_2 = vz_2 / length_2;
		double direction2[3]{ ux_2,uy_2,uz_2 };
		// ����������������֮��ļн�
		double dotProduct = direction1[0] * direction2[0] + direction1[1] * direction2[1] + direction1[2] * direction2[2]; // ���
		double angleInRadians = acos(dotProduct); // ʹ�÷����Һ����õ��ǶȵĻ���ֵ
		double angleInDegrees = round(angleInRadians * (180.0 / M_PI) * 10) / 10; // ����һλС��
		m_Controls.LineEdit_angle->setText(QString::number(angleInDegrees));
		std::cout << "The angle between the two directions is: " << angleInDegrees << " degrees." << std::endl;
		m_Controls.LineEdit_transAngle->setText(QString::number(angleInDegrees));

		CaculateStrechHeigh();
		// �������ת�ᰴ��LinEdit�е���ֵ�����У���ֵ��ǰ�����нǵ�ʱ��ͱ�������
		RotateMinus();
	}
	return true;
}

bool HTONDI::OnResetAngleClicked()
{
	/* ���óſ��Ƕȹ滮
		ɾ���滮���
		���½��нع�����
	*/
	m_Controls.textBrowser_Action->append("Action: ResetAngle.");

	GetDataStorage()->Remove(GetDataStorage()->GetNamedNode("legForceLineNew"));
	GetDataStorage()->Remove(GetDataStorage()->GetNamedNode("ankleCenterPointNew"));
	GetDataStorage()->Remove(GetDataStorage()->GetNamedNode("tibiaDistalPointSet"));
	GetDataStorage()->Remove(GetDataStorage()->GetNamedNode("distal tibiaSurface"));
	GetDataStorage()->Remove(GetDataStorage()->GetNamedNode("proximal tibiaSurface"));

	// ������к��������ɽع���
	OnCutTibetClicked();
	return true;
}

// ��֫���߹滮
bool HTONDI::OnShowMachineLineClicked()
{
	/* ���㲢��ʾ��֫����
	// ѡ�� �ɹ�ͷ���� �� �׹ؽ�����
	// ===================================
	// �ɹ�ͷ���ĵ� �ɼ�����ϵó�
	// �׹ؽ������� ������ �����е�ó�
	// ===================================
	*/
	
	m_Controls.textBrowser_Action->append("Action: Show The Line of Force.");

	// ֱ�Ӵӿ��м��عɹ�ͷ���ĵ�
	cout << "test 01" << endl;
	mitk::DataNode::Pointer hipCenterNode = GetDataStorage()->GetNamedNode("hipCenterPoint");
	auto hipCenterPoint = dynamic_cast<mitk::PointSet*>(hipCenterNode->GetData())->GetPoint(0);
	cout << "test 02" << endl;
	// ��ȡ�ֹ��׵㼯
	mitk::DataNode::Pointer tibiaLandmarkNode = GetDataStorage()->GetNamedNode("tibiaLandmarkPointSet");
	auto tibiaLandmarkPointSet = dynamic_cast<mitk::PointSet*>(tibiaLandmarkNode->GetData());
	cout << "test 03" << endl;
	// ��ȡ�����͵��ĸ���
	mitk::Point3D anklePoint1 = tibiaLandmarkPointSet->GetPoint(2);
	mitk::Point3D anklePoint2 = tibiaLandmarkPointSet->GetPoint(3);
	cout << "test 04" << endl;
	// �����׹ؽ����ĵ�
	mitk::Point3D ankleCenterPoint;
	ankleCenterPoint[0] = (anklePoint1[0] + anklePoint2[0]) / 2;
	ankleCenterPoint[1] = (anklePoint1[1] + anklePoint2[1]) / 2;
	ankleCenterPoint[2] = (anklePoint1[2] + anklePoint2[2]) / 2;
	cout << "test 05" << endl;
	// ���ӻ��׹ؽ����ĵ�
	// ��ͼ���ϻ����׹ؽ����ĵ�

	// ����ɾ��ԭ�е�
	auto ankleCenter = GetDataStorage()->GetNamedNode("ankleCenterPoint");
	if (ankleCenter != nullptr) {
		cout << "test 06" << endl;
		GetDataStorage()->Remove(ankleCenter);
	}

	cout << "test 07" << endl;
	mitk::PointSet::Pointer ankleCenterPointSet = mitk::PointSet::New();
	ankleCenterPointSet->InsertPoint(0, ankleCenterPoint);
	mitk::DataNode::Pointer ankleCenterNode = mitk::DataNode::New();
	ankleCenterNode->SetData(ankleCenterPointSet);
	ankleCenterNode->SetName("ankleCenterPoint");
	ankleCenterNode->SetProperty("color", mitk::ColorProperty::New(1.0, 0.0, 0.0)); // ������ɫΪ��ɫ
	ankleCenterNode->SetProperty("pointsize", mitk::FloatProperty::New(5.0)); // ���õ�Ĵ�СΪ5
	GetDataStorage()->Add(ankleCenterNode);

	cout << "test 08" << endl;
	// ������֫���߷���
	mitk::Vector3D legForceDirection = ankleCenterPoint - hipCenterPoint;

	cout << "test 09" << endl;
	// �����µĵ㼯
	mitk::PointSet::Pointer legForceLine = mitk::PointSet::New();
	legForceLine->SetPoint(0, hipCenterPoint);
	legForceLine->SetPoint(1, ankleCenterPoint);

	cout << "test 10" << endl;
	// ��ѯ�Ƿ������Ϊ"LegForceLine"�Ľڵ�
	mitk::DataNode::Pointer existingLegForceLineNode = GetDataStorage()->GetNamedNode("legForceLine");
	cout << "test 11" << endl;
	// ������֫����͸����
	auto tibiaSurface = GetDataStorage()->GetNamedNode("tibiaSurface");
	SetModelOpacity(tibiaSurface, 0.5f);
	cout << "test 12" << endl;
	if (existingLegForceLineNode.IsNotNull())
	{
		// ����ڵ��Ѵ��ڣ�����������
		existingLegForceLineNode->SetData(legForceLine);
	}
	else
	{
		// ����ڵ㲻���ڣ������µĽڵ�
		mitk::DataNode::Pointer legForceLineNode = mitk::DataNode::New();
		legForceLineNode->SetData(legForceLine);
		legForceLineNode->SetName("legForceLine");

		// �����ڴ˴���������������ã�������ʾ����
		legForceLineNode->SetBoolProperty("show contour", true);
		legForceLineNode->SetFloatProperty("contoursize", 1.0);
		legForceLineNode->SetVisibility(true);
		// ���½ڵ���ӵ����ݴ洢��
		GetDataStorage()->Add(legForceLineNode);
	}
	cout << "test 13" << endl;
	// ���㲢��������ռ��ֵ
	updateProportation();
	cout << "test 14" << endl;
	// ����ͼ��
	GetDataStorage()->Modified();
	mitk::RenderingManager::GetInstance()->RequestUpdateAll();
	return true;
}

bool HTONDI::OnShowMachineLine02Clicked()
{
	/* ���㲢��ʾ�滮����֫����
	// ѡ�� �ɹ�ͷ���� �� �׹ؽ�������λ��
	// ===================================
	// �ɹ�ͷ���ĵ� �ɼ�����ϵó�
	// �׹ؽ������� ������ �����е�ó�
	// ===================================
	*/
	
	m_Controls.textBrowser_Action->append("Action: Show The pro Line of Force.");

	// ֱ�Ӵӿ��м��عɹ�ͷ���ĵ�
	cout << "test 01" << endl;
	mitk::DataNode::Pointer hipCenterNode = GetDataStorage()->GetNamedNode("hipCenterPoint");
	auto hipCenterPoint = dynamic_cast<mitk::PointSet*>(hipCenterNode->GetData())->GetPoint(0);
	cout << "test 02" << endl;
	// ��ȡ�ֹ��׵㼯
	mitk::DataNode::Pointer tibiaLandmarkNode = GetDataStorage()->GetNamedNode("tibiaDistalPointSet");
	auto tibiaLandmarkPointSet = dynamic_cast<mitk::PointSet*>(tibiaLandmarkNode->GetData());
	cout << "test 03" << endl;
	// ��ȡ�����͵��ĸ���
	mitk::Point3D anklePoint1 = tibiaLandmarkPointSet->GetPoint(0);
	mitk::Point3D anklePoint2 = tibiaLandmarkPointSet->GetPoint(1);
	cout << "test 04" << endl;
	// �����׹ؽ����ĵ�
	mitk::Point3D ankleCenterPoint;
	ankleCenterPoint[0] = (anklePoint1[0] + anklePoint2[0]) / 2;
	ankleCenterPoint[1] = (anklePoint1[1] + anklePoint2[1]) / 2;
	ankleCenterPoint[2] = (anklePoint1[2] + anklePoint2[2]) / 2;
	cout << "test 05" << endl;
	// ���ӻ��׹ؽ����ĵ�
	// ��ͼ���ϻ����׹ؽ����ĵ�

	// ����ɾ��ԭ�е�
	auto ankleCenter = GetDataStorage()->GetNamedNode("ankleCenterPointNew");
	if (ankleCenter != nullptr) {
		cout << "test 06" << endl;
		GetDataStorage()->Remove(ankleCenter);
	}

	cout << "test 07" << endl;
	mitk::PointSet::Pointer ankleCenterPointSet = mitk::PointSet::New();
	ankleCenterPointSet->InsertPoint(0, ankleCenterPoint);
	mitk::DataNode::Pointer ankleCenterNode = mitk::DataNode::New();
	ankleCenterNode->SetData(ankleCenterPointSet);
	ankleCenterNode->SetName("ankleCenterPointNew");
	ankleCenterNode->SetProperty("color", mitk::ColorProperty::New(0.0, 0.0, 1.0)); // ������ɫΪ��ɫ
	ankleCenterNode->SetProperty("pointsize", mitk::FloatProperty::New(5.0)); // ���õ�Ĵ�СΪ5
	GetDataStorage()->Add(ankleCenterNode);

	cout << "test 08" << endl;
	// ������֫���߷���
	mitk::Vector3D legForceDirection = ankleCenterPoint - hipCenterPoint;

	cout << "test 09" << endl;
	// �����µĵ㼯
	mitk::PointSet::Pointer legForceLine = mitk::PointSet::New();
	legForceLine->SetPoint(0, hipCenterPoint);
	legForceLine->SetPoint(1, ankleCenterPoint);
	legForceLine->SetProperty("color", mitk::ColorProperty::New(0.0, 0.0, 1.0));

	cout << "test 10" << endl;
	// ��ѯ�Ƿ������Ϊ"LegForceLine"�Ľڵ�
	mitk::DataNode::Pointer existingLegForceLineNode = GetDataStorage()->GetNamedNode("legForceLineNew");
	cout << "test 11" << endl;
	// ������֫����͸����
	auto tibiaSurface = GetDataStorage()->GetNamedNode("tibiaSurface");
	SetModelOpacity(tibiaSurface, 0.5f);
	cout << "test 12" << endl;
	if (existingLegForceLineNode.IsNotNull())
	{
		// ����ڵ��Ѵ��ڣ�����������
		existingLegForceLineNode->SetData(legForceLine);
	}
	else
	{
		// ����ڵ㲻���ڣ������µĽڵ�
		mitk::DataNode::Pointer legForceLineNode = mitk::DataNode::New();
		legForceLineNode->SetData(legForceLine);
		legForceLineNode->SetName("legForceLineNew");

		// �����ڴ˴���������������ã�������ʾ����
		legForceLineNode->SetBoolProperty("show contour", true);
		legForceLineNode->SetFloatProperty("contoursize", 1.0);

		legForceLineNode->SetVisibility(true);
		// ���½ڵ���ӵ����ݴ洢��
		GetDataStorage()->Add(legForceLineNode);
	}
	cout << "test 13" << endl;
	// ���㲢��������ռ��ֵ
	updateProportation02();
	cout << "test 14" << endl;
	// ����ͼ��
	GetDataStorage()->Modified();
	mitk::RenderingManager::GetInstance()->RequestUpdateAll();
	return true;
}

bool HTONDI::OnUnshowMachineLineClicked()
{
	/* ��ʾ/���� ��֫���� */

	// ������֫����
	m_Controls.textBrowser_Action->append("Action: Unshow The Line of Force.");
	// �����ݴ洢�л�ȡ��֫���߽ڵ�
	mitk::DataNode::Pointer legForceLineNode = GetDataStorage()->GetNamedNode("legForceLine");
	if (legForceLineNode.IsNotNull())
	{
		// �л���֫���߽ڵ�Ŀɼ���
		bool currentVisibility = legForceLineNode->IsVisible(nullptr, "visible", true);
		legForceLineNode->SetVisibility(!currentVisibility);
	}

	GetDataStorage()->Modified();
	mitk::RenderingManager::GetInstance()->RequestUpdateAll();
	return true;
}

bool HTONDI::OnUnshowMachineLine02Clicked()
{
	// ������֫����
	m_Controls.textBrowser_Action->append("Action: Unshow The pro Line of Force.");
	// �����ݴ洢�л�ȡ��֫���߽ڵ�
	mitk::DataNode::Pointer legForceLineNode = GetDataStorage()->GetNamedNode("legForceLineNew");
	if (legForceLineNode.IsNotNull())
	{
		// �л���֫���߽ڵ�Ŀɼ���
		bool currentVisibility = legForceLineNode->IsVisible(nullptr, "visible", true);
		legForceLineNode->SetVisibility(!currentVisibility);
	}

	GetDataStorage()->Modified();
	mitk::RenderingManager::GetInstance()->RequestUpdateAll();
	return true;
}

void HTONDI::updateProportation()
{
	// �����������ֹ�ƽ̨ռ��
	cout << "test 01" << endl;
	m_Controls.textBrowser_Action->append("Show Force Line Rate");
	// ֱ�Ӵӿ��м��� �ɹ����ĵ� �� �׹ؽ����ĵ�
	mitk::DataNode::Pointer ankleCenter = GetDataStorage()->GetNamedNode("ankleCenterPoint");
	auto ankleCenterPoint = dynamic_cast<mitk::PointSet*>(ankleCenter->GetData());
	cout << "test 02" << endl;
	mitk::DataNode::Pointer hipCenter = GetDataStorage()->GetNamedNode("hipCenterPoint");
	auto hipCenterPoint = dynamic_cast<mitk::PointSet*>(hipCenter->GetData());
	cout << "test 03" << endl;
	// ȡ�����ߵ����˵�
	auto ForcePoint1 = ankleCenterPoint->GetPoint(0);
	auto ForcePoint2 = hipCenterPoint->GetPoint(0);
	cout << "test 04" << endl;
	// ���ֹ��׵㼯��ȡ���ֹ�ƽ̨���˵�
	mitk::DataNode::Pointer tibiaLandmarkNode = GetDataStorage()->GetNamedNode("tibiaLandmarkPointSet");
	auto tibiaLandmarkPointSet = dynamic_cast<mitk::PointSet*>(tibiaLandmarkNode->GetData());
	cout << "test 05" << endl;
	// ��ȡ�ֹǽ��� ���� + �ڲ��
	auto Point_0 = tibiaLandmarkPointSet->GetPoint(0);
	auto Point_1 = tibiaLandmarkPointSet->GetPoint(1);

	// �����ֹ�ƽ̨�Ŀ�� = abs(x1 - x2) ���� ���Ÿ�
	double tibia_length = sqrt(pow(Point_0[0] - Point_1[0], 2));

	// ============================
	//���ߵĳ���
	Line_length = sqrt(pow(ForcePoint1[0] - ForcePoint2[0], 2)
		+ pow(ForcePoint1[1] - ForcePoint2[1], 2)
		+ pow(ForcePoint1[2] - ForcePoint2[2], 2));
	// ============================

	// ����ˮƽ�߶�, �߶�ȡ�ֹǽ�������������ƽ���߶�
	double coronalZ = (Point_0[2] + Point_1[2]) / 2;

	// XoZ ƽ�� (x, z)
	// ����һ����άƽ���ϵ�ˮƽ�߶�, �߶ε������յ�ֱ����ڹ�״����ͶӰ��������
	Eigen::Vector2d line1ProjStart = { Point_0[0], coronalZ };
	Eigen::Vector2d line1ProjEnd = { Point_1[0], coronalZ };

	// ������б�߶�
	Eigen::Vector2d line2ProjStart = { ForcePoint1[0], ForcePoint1[2] };
	Eigen::Vector2d line2ProjEnd = { ForcePoint2[0], ForcePoint2[2] };

	cout << "test 06" << endl;
	// ʹ��VTK��ѧ������ά�ռ������߶εĽ���
	Eigen::Vector2d intersection;
	if (LineLineIntersection(intersection, line1ProjStart, line1ProjEnd, line2ProjStart, line2ProjEnd)) {
		cout << "test 07" << endl;
		// ���㽻����ˮƽ�߶��ϵ�λ��
		double propatation = sqrt(pow(intersection[0] - Point_0[0], 2)) / sqrt(pow(Point_1[0] - Point_0[0], 2));
		cout << "propatation = " << propatation << endl;
		propatation = ceil(propatation * 100);
		m_Controls.textBrowser_Action->append(QString::number(propatation) + "%");
		m_Controls.showForceLine_textEdit->setText(QString::number(propatation) + "%");
	}
	else {
		cout << "test 08" << endl;
		std::cout << "The lines do not intersect in the coronal plane." << std::endl;
		m_Controls.textBrowser_Action->append(QString::number(0) + "%");
		m_Controls.showForceLine_textEdit->setText(QString::number(0) + "%");
	}
}

void HTONDI::updateProportation02()
{
	// �������������ֹ�ƽ̨ռ��
	cout << "test 01" << endl;
	m_Controls.textBrowser_Action->append("Show Force Line Rate");
	// ֱ�Ӵӿ��м��� �ɹ����ĵ� �� �׹ؽ����ĵ�
	mitk::DataNode::Pointer ankleCenter = GetDataStorage()->GetNamedNode("ankleCenterPointNew");
	auto ankleCenterPoint = dynamic_cast<mitk::PointSet*>(ankleCenter->GetData());
	cout << "test 02" << endl;
	mitk::DataNode::Pointer hipCenter = GetDataStorage()->GetNamedNode("hipCenterPoint");
	auto hipCenterPoint = dynamic_cast<mitk::PointSet*>(hipCenter->GetData());
	cout << "test 03" << endl;
	// ȡ�����ߵ����˵�
	auto ForcePoint1 = ankleCenterPoint->GetPoint(0);
	auto ForcePoint2 = hipCenterPoint->GetPoint(0);
	cout << "test 04" << endl;
	// ���ֹ��׵㼯��ȡ���ֹ�ƽ̨���˵�
	mitk::DataNode::Pointer tibiaLandmarkNode = GetDataStorage()->GetNamedNode("tibiaLandmarkPointSet");
	auto tibiaLandmarkPointSet = dynamic_cast<mitk::PointSet*>(tibiaLandmarkNode->GetData());
	cout << "test 05" << endl;
	// ��ȡ�ֹǽ��� ���� + �ڲ��
	auto Point_0 = tibiaLandmarkPointSet->GetPoint(0);
	auto Point_1 = tibiaLandmarkPointSet->GetPoint(1);

	// �����ֹ�ƽ̨�Ŀ�� = abs(x1 - x2) ���� ���Ÿ�
	double tibia_length = sqrt(pow(Point_0[0] - Point_1[0], 2));

	// ����ˮƽ�߶�, �߶�ȡ�ֹǽ�������������ƽ���߶�
	double coronalZ = (Point_0[2] + Point_1[2]) / 2;

	// XoZ ƽ�� (x, z)
	// ����һ����άƽ���ϵ�ˮƽ�߶�, �߶ε������յ�ֱ����ڹ�״����ͶӰ��������
	Eigen::Vector2d line1ProjStart = { Point_0[0], coronalZ };
	Eigen::Vector2d line1ProjEnd = { Point_1[0], coronalZ };

	// ������б�߶�
	Eigen::Vector2d line2ProjStart = { ForcePoint1[0], ForcePoint1[2] };
	Eigen::Vector2d line2ProjEnd = { ForcePoint2[0], ForcePoint2[2] };

	cout << "test 06" << endl;
	// ʹ��VTK��ѧ������ά�ռ������߶εĽ���
	Eigen::Vector2d intersection;
	if (LineLineIntersection(intersection, line1ProjStart, line1ProjEnd, line2ProjStart, line2ProjEnd)) {
		cout << "test 07" << endl;
		// ���㽻����ˮƽ�߶��ϵ�λ��
		double propatation = sqrt(pow(intersection[0] - Point_0[0], 2)) / sqrt(pow(Point_1[0] - Point_0[0], 2));
		cout << "propatation = " << propatation << endl;
		propatation = ceil(propatation * 100);
		m_Controls.textBrowser_Action->append(QString::number(propatation) + "%");
		m_Controls.showForceLine02_textEdit->setText(QString::number(propatation) + "%");
	}
	else {
		cout << "test 08" << endl;
		std::cout << "The lines do not intersect in the coronal plane." << std::endl;
		m_Controls.textBrowser_Action->append(QString::number(0) + "%");
		m_Controls.showForceLine02_textEdit->setText(QString::number(0) + "%");
	}
}

bool HTONDI::LineLineIntersection(Eigen::Vector2d& intersection, Eigen::Vector2d p1, Eigen::Vector2d p2, Eigen::Vector2d q1, Eigen::Vector2d q2)
{
	// ����ռ��������߶εĽ���
	// ���룺�����߶ε������յ� p1, p2, q1, q2
	// ��������� intersection

	// ��������
	Eigen::Vector2d r = p2 - p1;
	Eigen::Vector2d s = q2 - q1;

	// ������
	double rxs = r.x() * s.y() - r.y() * s.x();
	double qpxr = (q1 - p1).x() * r.y() - (q1 - p1).y() * r.x();

	// ����Ƿ�ƽ��
	if (rxs == 0 && qpxr == 0) {
		// ����
		return false;
	}
	if (rxs == 0 && qpxr != 0) {
		// ƽ���Ҳ�����
		return false;
	}

	// ������� t �� u
	double t = ((q1 - p1).x() * s.y() - (q1 - p1).y() * s.x()) / rxs;
	double u = ((q1 - p1).x() * r.y() - (q1 - p1).y() * r.x()) / rxs;

	// ��齻���Ƿ����߶���
	if (rxs != 0 && t >= 0 && t <= 1 && u >= 0 && u <= 1) {
		intersection = p1 + t * r;
		return true;
	}

	// û�н���
	return false;
}

void HTONDI::CaculateStrechHeigh()
{
	// ���Ƕ�ת��Ϊ����
	std::cout << "angleInDegrees: " << angleInDegrees << std::endl;
	double angle_rad = angleInDegrees * M_PI / 180.0;
	std::cout << "angle_rad: " << angle_rad << std::endl;
	double height = round(sin(angle_rad) * depth);
	m_Controls.LineEdit_height->setText(QString::number(height));
}

// �ع�ЧӦ
bool HTONDI::GetIntersectionLine()
{
	/* ʵʱ��ȡ�ع��� 
		ԭ��: ���ýع�ƽ��͹Ǳ�������и���㣬�õ�һ����ƵĽ���
	
	*/
	auto cutPlaneNode = GetDataStorage()->GetNamedNode("1st cut plane");
	auto tibiaSurfaceNode = GetDataStorage()->GetNamedNode("tibiaSurface");
	if (cutPlaneNode == nullptr)
	{
		m_Controls.textBrowser_Action->append("'tibia cut plane' is not ready");
		return false;
	}
	if (tibiaSurfaceNode == nullptr)
	{
		m_Controls.textBrowser_Action->append("'tibiaSurface' is not ready");
		return false;
	}
	auto mitkCutSurface = dynamic_cast<mitk::Surface*>(cutPlaneNode->GetData());

	auto tibiaMitkSurface = dynamic_cast<mitk::Surface*>(tibiaSurfaceNode->GetData());

	auto tmpVtkSurface_initial = mitkCutSurface->GetVtkPolyData();

	auto tibiaVtkSurface_initial = tibiaMitkSurface->GetVtkPolyData();

	// ���ڸ��Ƴ�һ���µ�ͼ�񣬶�����ԭʼ���ݲ������õ�Ӱ��
	// ���ƽع�ƽ��
	vtkNew<vtkTransform> cutPlaneTransform;
	cutPlaneTransform->SetMatrix(mitkCutSurface->GetGeometry()->GetVtkMatrix());
	vtkNew<vtkTransformFilter> cutPlaneTransformFilter;
	cutPlaneTransformFilter->SetTransform(cutPlaneTransform);
	cutPlaneTransformFilter->SetInputData(tmpVtkSurface_initial);
	cutPlaneTransformFilter->Update();

	// �õ������µ�����
	vtkNew<vtkPolyData> tmpVtkSurface;
	tmpVtkSurface->DeepCopy(cutPlaneTransformFilter->GetPolyDataOutput());

	// ���ƹǱ���
	vtkNew<vtkTransform> tibiaTransform;
	tibiaTransform->SetMatrix(tibiaMitkSurface->GetGeometry()->GetVtkMatrix());
	vtkNew<vtkTransformFilter> tibiaTransformFilter;
	tibiaTransformFilter->SetTransform(tibiaTransform);
	tibiaTransformFilter->SetInputData(tibiaVtkSurface_initial);
	tibiaTransformFilter->Update();

	vtkNew<vtkPolyData> tibiaVtkSurface;
	tibiaVtkSurface->DeepCopy(tibiaTransformFilter->GetPolyDataOutput());

	// ����ع�ƽ��ĵ�ǰ��������ƽ������
	double surfaceNormal[3];
	double cutPlaneCenter[3];

	GetPlaneProperty(tmpVtkSurface, surfaceNormal, cutPlaneCenter);


	// ʹ�û�ȡ�ķ�������ԭ�㴴��һ��vtkPlane����
	vtkSmartPointer<vtkPlane> cutPlane = vtkSmartPointer<vtkPlane>::New();
	cutPlane->SetNormal(surfaceNormal);
	cutPlane->SetOrigin(cutPlaneCenter);

	// Ȼ�����cutPlane���ø�vtkCutter
	vtkSmartPointer<vtkCutter> cutter_plane = vtkSmartPointer<vtkCutter>::New();
	cutter_plane->SetCutFunction(cutPlane);
	//������������
	cutter_plane->SetInputData(tibiaVtkSurface); // tibiaModelPolyData �Ǳ�ʾ�ֹǵ�vtkPolyData����
	//���»�ȡ����
	cutter_plane->Update();
	vtkSmartPointer<vtkPolyData> intersectionLine = cutter_plane->GetOutput();
	TraverseIntersectionLines(intersectionLine);//��ȡ�и�ƽ����tibiaPolyData�Ľ����Լ��ض���

	return true;
}

void HTONDI::TraverseIntersectionLines(vtkSmartPointer<vtkPolyData> intersectionLine)
{
	// ��ȡ�ع����������Ҳ��

	if (!planeAndTibiaIntersectionPoint)
	{
		planeAndTibiaIntersectionPoint = mitk::PointSet::New();
	}
	planeAndTibiaIntersectionPoint->Clear();

	// ��ȡ�㼯
	vtkSmartPointer<vtkPoints> points = intersectionLine->GetPoints();

	// ��ȡ�������
	int numberOfPoints = points->GetNumberOfPoints();
	double minX = std::numeric_limits<double>::max();
	int minXIndex = -1;
	double maxX = -std::numeric_limits<double>::max(); // ��ʼ��ΪС�ڿ��ܵ���Сxֵ
	int maxXIndex = -1;

	std::cout << "Intersection Line Points:" << std::endl;
	for (int i = 0; i < numberOfPoints; ++i)
	{
		double point[3];
		points->GetPoint(i, point); // ��ȡ��i���������

		std::cout << "Point " << i << ": (" << point[0] << ", " << point[1] << ", " << point[2] << ")" << std::endl;
		//��鵱ǰ���x�����Ƿ�С����֪����Сxֵ
		if (point[0] < minX)
		{
			minX = point[0]; // ������Сxֵ
			minXIndex = i;   // ��¼��Сxֵ��Ӧ�ĵ������
		}
		if (point[0] > maxX)
		{
			maxX = point[0];
			maxXIndex = i;
		}
	}
	if (minXIndex != -1 && maxXIndex != -1)
	{
		std::cout << "The point with the smallest x-coordinate is Point " << minXIndex << ": ("
			<< minX << ", " << points->GetPoint(minXIndex)[1] << ", " << points->GetPoint(minXIndex)[2] << ")" << std::endl;

		std::cout << "The point with the largest x-coordinate is Point " << maxXIndex << ": ("
			<< maxX << ", " << points->GetPoint(maxXIndex)[1] << ", " << points->GetPoint(maxXIndex)[2] << ")" << std::endl;
	}
	else
	{
		std::cout << "No points were processed." << std::endl;
	}

	////�洢�ҵ��Ľع������ֹ�ģ�͵Ľ����ϵ����������ڵ㣻
	minPoint[0] = points->GetPoint(minXIndex)[0];
	minPoint[1] = points->GetPoint(minXIndex)[1];
	minPoint[2] = points->GetPoint(minXIndex)[2];

	maxPoint[0] = points->GetPoint(maxXIndex)[0];
	maxPoint[1] = points->GetPoint(maxXIndex)[1];
	maxPoint[2] = points->GetPoint(maxXIndex)[2];


	planeAndTibiaIntersectionPoint->InsertPoint(0, minPoint);
	planeAndTibiaIntersectionPoint->InsertPoint(1, maxPoint);

	// ɾ��֮ǰ�Ĳ��Խڵ�
	auto tmpCutLine = GetDataStorage()->GetNamedNode("planeAndTibiaIntersectionPoint");
	if (tmpCutLine != nullptr) {
		GetDataStorage()->Remove(tmpCutLine);
	}
	mitk::DataNode::Pointer cutLine = mitk::DataNode::New();
	cutLine->SetName("planeAndTibiaIntersectionPoint");
	cutLine->SetColor(1.0, 0.0, 0.0);                                  // Set color to red
	cutLine->SetData(planeAndTibiaIntersectionPoint);                                // Add poinset into datanode
	cutLine->SetProperty("pointsize", mitk::FloatProperty::New(10.0)); // Change the point size
	mitk::DataStorage::Pointer dataStorage = this->GetDataStorage();
	dataStorage->Add(cutLine);
	dataStorage->Modified();

}

void HTONDI::GetDistancefromTibia()
{
	//��ȡƽ��λ�õ������ֹ������ƽ̨�ľ����Լ���������ҳ
	auto tmpPointSet2 = dynamic_cast<mitk::PointSet*>(GetDataStorage()->GetNamedNode("tibiaLandmarkPointSet")->GetData());
	auto Point_0 = tmpPointSet2->GetPoint(0);
	auto Point_1 = tmpPointSet2->GetPoint(1);

	// 0:����. 1������
	if (judgModel_flag == 0)
	{
		cout << "left Limb" << endl;
		distance1 = Point_0[2] - maxPoint[2];//�ع���ĩ�˾����ƽ̨
		distance2 = sqrt(pow((Point_1[0] - minPoint[0]), 2) +
			pow((Point_1[1] - minPoint[1]), 2) +
			pow((Point_1[2] - minPoint[2]), 2));//�ع�����ھ��ڲ�ƽ̨
		distance3 = sqrt(pow((maxPoint[0] - mitkPointSet1->GetPoint(3)[0]), 2) +
			pow((maxPoint[1] - mitkPointSet1->GetPoint(3)[1]), 2) +
			pow((maxPoint[2] - mitkPointSet1->GetPoint(3)[2]), 2));//��������ҳ
		if (mitkPointSet1->GetPoint(3)[0] > maxPoint[0]) {
			depth = abs(maxPoint[0] - minPoint[0]);//������
		}
		else {
			depth = abs(mitkPointSet1->GetPoint(3)[0] - minPoint[0]);//������
		}

		std::cout << "mitkPointSet1�� " << ": (" << mitkPointSet1->GetPoint(3)[0] << ", " << mitkPointSet1->GetPoint(3)[1] << ", " << mitkPointSet1->GetPoint(3)[2] << ")" << std::endl;
		m_Controls.LineEdit_distance1->setText(QString::number(distance1));
		m_Controls.LineEdit_distance2->setText(QString::number(distance2));
		m_Controls.LineEdit_hy->setText(QString::number(distance3));
		m_Controls.LineEdit_depth->setText(QString::number(depth));
	}
	else {
		// 0:����. 1������
		if (judgModel_flag == 1)
		{
			cout << "Right Limb" << endl;
			distance1 = Point_0[2] - minPoint[2];//�ع���ĩ�˾����ƽ̨
			distance2 = sqrt(pow((Point_1[0] - maxPoint[0]), 2) +
				pow((Point_1[1] - maxPoint[1]), 2) +
				pow((Point_1[2] - maxPoint[2]), 2));//�ع�����ھ��ڲ�ƽ̨
			distance3 = sqrt(pow((minPoint[0] - mitkPointSet1->GetPoint(0)[0]), 2) +
				pow((minPoint[1] - mitkPointSet1->GetPoint(0)[1]), 2) +
				pow((minPoint[2] - mitkPointSet1->GetPoint(0)[2]), 2));//��������ҳ
			if (mitkPointSet1->GetPoint(0)[0] < minPoint[0]) {
				depth = abs(minPoint[0] - maxPoint[0]);//�ع���������
			}
			else {
				depth = abs(mitkPointSet1->GetPoint(0)[0] - maxPoint[0]);//�ع���������
			}
			std::cout << "mitkPointSet1�� " << ": (" << mitkPointSet1->GetPoint(0)[0] << ", " << mitkPointSet1->GetPoint(0)[1] << ", " << mitkPointSet1->GetPoint(0)[2] << ")" << std::endl;
			m_Controls.LineEdit_distance1->setText(QString::number(distance1));
			m_Controls.LineEdit_distance2->setText(QString::number(distance2));
			m_Controls.LineEdit_hy->setText(QString::number(distance3));
			m_Controls.LineEdit_depth->setText(QString::number(depth));
		}
	}
}



