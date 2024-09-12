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

// mitk image
#include <mitkImage.h>

const std::string HTONDI::VIEW_ID = "org.mitk.views.htondi";

/*===============================================================
HTONDI.cpp
----------------------------------------------------------------
== ע�����ؼ�
== ��ʼ������������(δ����)
===============================================================*/






void HTONDI::SetFocus()
{
	// do nothing, but you can't delete it
}

void HTONDI::CreateQtPartControl(QWidget *parent)
{
  // create GUI widgets from the Qt Designer's .ui file

  /* ������HTO�����
	�����ؼ�������ע��ģ��
	-. ע������������
	1. ��ǰ�滮
	2. ����ע��
	3. ���е���
	4. ��������
	-. �����ƶ�
	-. ����ҳ
	-. ������ʼ��
  */

  m_Controls.setupUi(parent);


  // -. ע������������
  CreatQT_BasicSet();

  // 1. ��ǰ�滮
  CreateQT_PreoperativePlan();


  // 2. ����ע��
  CreateQT_MidPlan();

  // 3. ���е���
  CreateQT_MidOperation();

  // 4. ��������
  CreateQT_PostoperativeValid();


  // -. �����ƶ�
  CreateQT_MoveData();


  // -. ����ҳ
  CreateQT_FuncTest();
}

void HTONDI::ParamInit()
{
	/* ��ʼ���������Ʊ�׼(�ı�����)

	==========��δ����===========
		1. �Ǳ����
		2. ��е��׼��
		3. RF��������
		4. ��ǰ�滮 ��Ʊ�����
		5. ����ע�� ��Ʊ�����
		6. ���е��� ��Ʊ�����
		7. ������֤ ��Ʊ�����
	==========��δ����===========

	*/

	// ��е�ο���������
	std::string name_RF_Femur = "FemurRF";
	std::string name_RF_Tibia = "TibiaRF";
	std::string name_RF_Probe = "ProbeRF";
	std::string name_RF_Saw = "SawRF";
	std::string name_RF_Drill = "DrillRF";
	std::string name_RF_Calibrator = "CalibratorRF";

	// ������ - 0���� 1����
	bool type_Leg = 0;

	// �Ǳ����


	// 1.1. ��ǰ�滮01
	// PT�㼯 SF���� ND�� + ��;

	// װ��׼���õ�����
	std::string name_PT_FemurHead_Fit = "femurHeadPointSet";
	std::string name_PT_Femur_Landmark = "femurLandmarkPointSet";
	std::string name_PT_Tibia_Landmark = "tibiaLandmarkPointSet";
	std::string name_PT_Drill_Landmark = "DrillLandMarkPointSet";
	std::string name_PT_Saw_Landmark = "SawLandMarkPointSet";
	std::string name_PT_SteelPlate = "steelPlatePointSet";

	std::string name_SF_FemurSurface = "femurSurface";
	std::string name_SF_TibiaSurface = "tibiaSurface";
	std::string name_SF_Probe = "Probe";
	std::string name_SF_Saw = "Saw";
	std::string name_SF_Drill = "Drill";
	std::string name_SF_SteelPlate = "steelPlate";

	std::string name_ND_TibiaSurface = "femoralHead";

	// 1.2. ��ǰ�滮02

	// 1.2.1 �ع���滮
	std::string name_SF_CutPlaneAxial = "1st cut plane";
	std::string name_SF_CutPlaneSag = "2nd cut plane";
	std::string name_SF_ProximalTibiaSurface = "proximal tibiaSurface";
	std::string name_SF_DistalTibiaSurface = "distal tibiaSurface";
	std::string name_PT_PointsInCutPlaneAxial = "pointSetInPlaneCutPlane1";

	// 1.2.3 ���߹滮
	std::string name_ND_legForceLine01 = "legForceLine";
	std::string name_ND_legForceLine02 = "legForceLineNew";
	std::string name_ND_legForceLine03 = "legForceLineLast";

	std::string name_ND_AnkleCenter01 = "ankleCenterPoint";
	std::string name_ND_AnkleCenter02 = "ankleCenterPointNew";
	std::string name_ND_AnkleCenter03 = "ankleCenterPointLast";

	std::string name_PT_TibiaDistalPointSet = "tibiaDistalPointSet";

	// 2. ����ע��


	// 3. ���е���


	// 4. ������֤




}

void HTONDI::CreatQT_BasicSet()
{	
	/* �����������
		1. �������
	*/

	// 1. ������� NDI
	connect(m_Controls.connectNDI_pushButton, &QPushButton::clicked, this, &HTONDI::OnConnectNDIClicked);

}

void HTONDI::CreateQT_FuncTest()
{
	/* ���ܲ���ҳ
	
	*/


	// load STL data
	InitSurfaceSelector(m_Controls.setSTL_mitkSlecectButton);
	InitPointSetSelector(m_Controls.setLandMark_mitkSlecectButton);
	InitPointSetSelector(m_Controls.setBoneNode_mitkSlecectButton);

	connect(m_Controls.checkImage_pushButton, &QPushButton::clicked, this, &HTONDI::OnSetNavigateClicked);
	connect(m_Controls.showNode_pushButton, &QPushButton::clicked, this, &HTONDI::OnShowNodeClicked);
	connect(m_Controls.reuseRegistrateMatrix_pushButton, &QPushButton::clicked, this, &HTONDI::OnReuseMatrixClicked);
	
	// Move test
	connect(m_Controls.moveTest_pushButton, &QPushButton::clicked, this, &HTONDI::OnMoveTestClicked);
	connect(m_Controls.moveBone_pushButton, &QPushButton::clicked, this, &HTONDI::OnMoveBoneClicked);


	// collect registration point set
	connect(m_Controls.getLandMark_pushButton, &QPushButton::clicked, this, &HTONDI::OnGetLandMarkPointClicked);
	connect(m_Controls.getICP_pushButton, &QPushButton::clicked, this, &HTONDI::OnGetICPPointClicked);
	connect(m_Controls.showCollectedPoint_pushButton, &QPushButton::clicked, this, &HTONDI::OnShowCollectionClicked);
	connect(m_Controls.caculateRegistration_pushButton, &QPushButton::clicked, this, &HTONDI::OnCaculateRegistrationClicked);

	// accuracy test
	InitPointSetSelector(m_Controls.selectCheckNode_mitkSlecectButton);
	connect(m_Controls.checkAccuracyPoints_pushButton, &QPushButton::clicked, this, &HTONDI::OnCheckAccuracyPointsClicked);
	connect(m_Controls.checkNode2Node_pushButton, &QPushButton::clicked, this, &HTONDI::OnCheckNode2NodeClicked);
	connect(m_Controls.checkNode2Surface_pushButton, &QPushButton::clicked, this, &HTONDI::OnCheckNode2SurfaceClicked);

	// ��ͼ��
	connect(m_Controls.pushButton_linkSurface, &QPushButton::clicked, this, &HTONDI::OnLinkSurfaceClicked);
	

	// ����
	connect(m_Controls.pushButton_HTO_init, &QPushButton::clicked, this, &HTONDI::OnInitHTOTibiaRegisClicked);
	connect(m_Controls.pushButton_HTO_Collect_landmark, &QPushButton::clicked, this, &HTONDI::OnCollectHTOTibiaLandmarkClicked);
	connect(m_Controls.pushButton_HTO_Collect_lcp, &QPushButton::clicked, this, &HTONDI::OnCollectHTOTibiaICPClicked);
	connect(m_Controls.pushButton_HTO_Caculate_regis, &QPushButton::clicked, this, &HTONDI::OnCollectHTOTibiaRegisClicked);

	// ���°ھ�ĥ��λ��
	connect(m_Controls.pushButton_Renew_Saw, &QPushButton::clicked, this, &HTONDI::RenewSaw);
	connect(m_Controls.pushButton_Renew_Drill, &QPushButton::clicked, this, &HTONDI::RenewDrill);
}

void HTONDI::CreateQT_MoveData()
{
	/* �����ƶ�
		1. ��ǰ�滮
		2. ���е���
	*/

	// ��ǰ�滮 �����ƶ�
	connect(m_Controls.pushButton_xp, &QPushButton::clicked, this, &HTONDI::TranslatePlusX);
	connect(m_Controls.pushButton_yp, &QPushButton::clicked, this, &HTONDI::TranslatePlusY);
	connect(m_Controls.pushButton_zp, &QPushButton::clicked, this, &HTONDI::TranslatePlusZ);
	connect(m_Controls.pushButton_xm, &QPushButton::clicked, this, &HTONDI::TranslateMinusX);
	connect(m_Controls.pushButton_ym, &QPushButton::clicked, this, &HTONDI::TranslateMinusY);
	connect(m_Controls.pushButton_zm, &QPushButton::clicked, this, &HTONDI::TranslateMinusZ);
	connect(m_Controls.pushButton_rxp, &QPushButton::clicked, this, &HTONDI::RotatePlusX);
	connect(m_Controls.pushButton_ryp, &QPushButton::clicked, this, &HTONDI::RotatePlusY);
	connect(m_Controls.pushButton_rzp, &QPushButton::clicked, this, &HTONDI::RotatePlusZ);
	connect(m_Controls.pushButton_rxm, &QPushButton::clicked, this, &HTONDI::RotateMinusX);
	connect(m_Controls.pushButton_rym, &QPushButton::clicked, this, &HTONDI::RotateMinusY);
	connect(m_Controls.pushButton_rzm, &QPushButton::clicked, this, &HTONDI::RotateMinusZ);

	// ���е��� �����˶�
	//connect(m_Controls.pushButton_xp_2, &QPushButton::clicked, this, &HTONDI::TranslatePlusX2);
	//connect(m_Controls.pushButton_yp_2, &QPushButton::clicked, this, &HTONDI::TranslatePlusY2);
	//connect(m_Controls.pushButton_zp_2, &QPushButton::clicked, this, &HTONDI::TranslatePlusZ2);
	//connect(m_Controls.pushButton_xm_2, &QPushButton::clicked, this, &HTONDI::TranslateMinusX2);
	//connect(m_Controls.pushButton_ym_2, &QPushButton::clicked, this, &HTONDI::TranslateMinusY2);
	//connect(m_Controls.pushButton_zm_2, &QPushButton::clicked, this, &HTONDI::TranslateMinusZ2);
	//connect(m_Controls.pushButton_rxp_2, &QPushButton::clicked, this, &HTONDI::RotatePlusX2);
	//connect(m_Controls.pushButton_ryp_2, &QPushButton::clicked, this, &HTONDI::RotatePlusY2);
	//connect(m_Controls.pushButton_rzp_2, &QPushButton::clicked, this, &HTONDI::RotatePlusZ2);
	//connect(m_Controls.pushButton_rxm_2, &QPushButton::clicked, this, &HTONDI::RotateMinusX2);
	//connect(m_Controls.pushButton_rym_2, &QPushButton::clicked, this, &HTONDI::RotateMinusY2);
	//connect(m_Controls.pushButton_rzm_2, &QPushButton::clicked, this, &HTONDI::RotateMinusZ2);
}




void HTONDI::CreateQT_PreoperativePlan()
{
	/* ��ǰ�滮
		1. ��������׼��
		2. �ع���滮
		3. ��ʽ���滮
		3. ���߹滮
		4. �ſ��Ƕȹ滮
		5. �ְ�滮
	*/


	// 1. ��������׼��
	// ��������׼��
	connect(m_Controls.checkBaseData_pushButton, &QPushButton::clicked, this, &HTONDI::OnCheckBaseDataClicked);

	// ����Ӱ��ռ�ɹ�����
	connect(m_Controls.getFemurCenter_pushButton, &QPushButton::clicked, this, &HTONDI::OnFemurCenterClicked);


	// 2. �ع���滮
	connect(m_Controls.createCutPlane_pushButton, &QPushButton::clicked, this, &HTONDI::OnGenerateCutPlaneClicked);
	connect(m_Controls.cutTibia_pushButton, &QPushButton::clicked, this, &HTONDI::OnCutTibetClicked);
	connect(m_Controls.resetCut_pushButton, &QPushButton::clicked, this, &HTONDI::OnResetCutClicked);

	// 3. ��ʽ���滮
	connect(m_Controls.generateKeshiPinPos_pushButton, &QPushButton::clicked, this, &HTONDI::OnGenerateKeshiPinClicked);
	connect(m_Controls.recordKeshiPinPos_pushButton, &QPushButton::clicked, this, &HTONDI::OnRecordKeshiPinClicked);
	connect(m_Controls.visuliazeKeshiPin_pushButton, &QPushButton::clicked, this, &HTONDI::OnVisiualKeshiPinClicked);


	// 3. ���߹滮
	// ���㲢��ʾ����
	connect(m_Controls.showForceLine_pushButton, &QPushButton::clicked, this, &HTONDI::OnShowMachineLineClicked);

	// ����/��ʾ����
	connect(m_Controls.unshowForceLine_pushButton, &QPushButton::clicked, this, &HTONDI::OnUnshowMachineLineClicked);

	// �滮��֫����ʵʱ����
	connect(m_Controls.showForceLine02_pushButton, &QPushButton::clicked, this, &HTONDI::OnShowMachineLine02Clicked);
	connect(m_Controls.unshowForceLine02_pushButton, &QPushButton::clicked, this, &HTONDI::OnUnshowMachineLine02Clicked);


	// 4. �ſ��Ƕȹ滮
	// ����滮�ǶȲ����ӻ�
	connect(m_Controls.caculateAngle_pushButton, &QPushButton::clicked, this, &HTONDI::OnCaculateStrechAngleClicked);

	// ���ù滮�Ƕ�
	connect(m_Controls.resetAngle_pushButton, &QPushButton::clicked, this, &HTONDI::OnResetAngleClicked);

	// ��ת�ֹ�Զ��
	connect(m_Controls.pushButton_lt, &QPushButton::clicked, this, &HTONDI::RotateMinus);
	connect(m_Controls.pushButton_rt, &QPushButton::clicked, this, &HTONDI::RotatePlus);


	// 5. �ְ�滮
	// ��ʾ/���� �ְ�
	connect(m_Controls.showSteel_pushButton, &QPushButton::clicked, this, &HTONDI::OnShowSteelClick);

	// ���øְ�λ��
	connect(m_Controls.resetSteel_pushButton, &QPushButton::clicked, this, &HTONDI::OnResetSteelClick);

	// ȷ�ϸְ�滮λ��
	connect(m_Controls.setFinalSteelPos_pushButton, &QPushButton::clicked, this, &HTONDI::OnSetFinalPosClick);

}

void HTONDI::CreateQT_MidPlan()
{
	/* ����ע��
		1. ̽��궨
		2. ��ǵ���֤
		3. ������е�궨
		4. װ����׼��
		5. ʵʱ��ǵ���֤
		6. ��׼
	
	*/

	// 1. ̽��궨
	connect(m_Controls.selectPointer_pushButton, &QPushButton::clicked, this, &HTONDI::OnSelectPointerClicked);
	connect(m_Controls.calibratePointer_pushButton, &QPushButton::clicked, this, &HTONDI::OnCalibratePointerClicked);


	// 2. ��ǵ���֤
	connect(m_Controls.checkFemurNode_pushButton, &QPushButton::clicked, this, &HTONDI::OnFemurCheckClicked);
	connect(m_Controls.checkTibiaNode_pushButton, &QPushButton::clicked, this, &HTONDI::OnTibiaCheckClicked);

	// ʵʱ��������
	connect(m_Controls.checkFemurNodeTime_pushButton, &QPushButton::clicked, this, &HTONDI::OnFemurCheckClicked);
	connect(m_Controls.checkTibiaNodeTime_pushButton, &QPushButton::clicked, this, &HTONDI::OnTibiaCheckClicked);


	// 3. ������е�궨
	// װ�� �ھ� �� ĥ�� ��Ϣ
	connect(m_Controls.loadDeviceLand_pushButton, &QPushButton::clicked, this, &HTONDI::OnLoadDeviceLandmarkClicked);

	// �ھ�궨/ĥ��궨 �� ��֤
	connect(m_Controls.calibrateSaw_pushButton, &QPushButton::clicked, this, &HTONDI::OnCalibrateSawClicked);
	connect(m_Controls.calibrateDrill_pushButton, &QPushButton::clicked, this, &HTONDI::OnCalibrateDrillClicked);

	// ��ʾ/����
	connect(m_Controls.visualizeSaw_pushButton, &QPushButton::clicked, this, &HTONDI::OnSawVisualizeClicked);
	connect(m_Controls.visualizeDrill_pushButton, &QPushButton::clicked, this, &HTONDI::OnDrillVisualizeClicked);

	// ����
	connect(m_Controls.resetSawCali_pushButton, &QPushButton::clicked, this, &HTONDI::OnresetSawCaliClicked);
	connect(m_Controls.resetDrillCali_pushButton, &QPushButton::clicked, this, &HTONDI::OnresetDrillCaliClicked);


	// 4. װ�� �Ǳ��� ��׼����Ϣ
	connect(m_Controls.loadBonePoints_pushButton, &QPushButton::clicked, this, &HTONDI::OnLoadBonePointsClicked);
	

	// 5. ��׼
	// ����׼ ��׼�ɵ� �� ��׼����
	connect(m_Controls.getFemurLandmark_pushButton, &QPushButton::clicked, this, &HTONDI::OnGetFemurLandmarkClicked);
	
	// ��ùɹ�ͷ����
	connect(m_Controls.collectBonePos_pushButton, &QPushButton::clicked, this, &HTONDI::OnCollectBonePosClicked);
	connect(m_Controls.cacultateBoneCenter_pushButton, &QPushButton::clicked, this, &HTONDI::OnCaculateBoneCenterClicked);
	
	// �ɹǴ���׼
	connect(m_Controls.caculateFemurLandmark_pushButton, &QPushButton::clicked, this, &HTONDI::OnCaculateFemurLandmarkClicked);

	// ����
	connect(m_Controls.pushButton_HTO_initTibia, &QPushButton::clicked, this, &HTONDI::OnInitHTOTibiaRegisClicked);

	// �ֹǴ���׼
	connect(m_Controls.getTibiaLandmark_pushButton, &QPushButton::clicked, this, &HTONDI::OnCollectHTOTibiaLandmarkClicked);
	connect(m_Controls.caculateTibiaLandmark_pushButton, &QPushButton::clicked, this, &HTONDI::OnCollectHTOTibiaRegisClicked);
	

	// ����׼ ��׼�ɵ� �� ��׼����
	connect(m_Controls.getTibieICP_pushButton, &QPushButton::clicked, this, &HTONDI::OnCollectHTOTibiaICPClicked);
	connect(m_Controls.caculateTibiaICP_pushButton, &QPushButton::clicked, this, &HTONDI::OnCollectHTOTibiaRegisClicked);

}

void HTONDI::CreateQT_MidOperation()
{
	/* ���е���
	* 1. ĥ�굼��
	* 2. �عǵ���
	* 3. ĥ�굼��
	* 4. ����������֤
	*/ 
	
	// 1. ĥ�굼��
	connect(m_Controls.pushButton_ChangeKeShiPin, &QPushButton::clicked, this, &HTONDI::OnChangeKeShiPinClicked);
	connect(m_Controls.pushButton_ChangeLinkPin, &QPushButton::clicked, this, &HTONDI::OnChangeLinkPinClicked);

	connect(m_Controls.pushButton_setLink01, &QPushButton::clicked, this, &HTONDI::OnSetLink01Clicked);
	connect(m_Controls.pushButton_setLink02, &QPushButton::clicked, this, &HTONDI::OnSetLink02Clicked);
	connect(m_Controls.pushButton_setLink03, &QPushButton::clicked, this, &HTONDI::OnSetLink03Clicked);

	connect(m_Controls.pushButton_setKeShi01, &QPushButton::clicked, this, &HTONDI::OnSetKeShi01Clicked);
	connect(m_Controls.pushButton_setKeShi02, &QPushButton::clicked, this, &HTONDI::OnSetKeShi02Clicked);

	connect(m_Controls.pushButton_finishCurrent, &QPushButton::clicked, this, &HTONDI::OnFinishCurrentDrillClicked);
	// �ְ尲װ����
	connect(m_Controls.guideDrill_pushButton, &QPushButton::clicked, this, &HTONDI::OnStartDrillGuideClicked);
	connect(m_Controls.startDrill_pushButton, &QPushButton::clicked, this, &HTONDI::OnStartDrillHoleClicked);
	

	// 2. �عǵ���
	
	// 2.1 �Ƕ���֤
	connect(m_Controls.guideAxialCut_pushButton, &QPushButton::clicked, this, &HTONDI::OnStartAxialGuideClicked);
	connect(m_Controls.pushButton_startAngleCheck, &QPushButton::clicked, this, &HTONDI::OnStartAngleCheckClicked);
	connect(m_Controls.startAxialCut_pushButton, &QPushButton::clicked, this, &HTONDI::OnSetAxialCutClicked);

	// 2.2 ˮƽ�عǿ���
	connect(m_Controls.pushButton_startAxialCut, &QPushButton::clicked, this, &HTONDI::OnStartAxialCutClicked);
	connect(m_Controls.pushButton_stopAxialCut, &QPushButton::clicked, this, &HTONDI::OnStopAxialCutClicked);
	// ��̬ģ��
	connect(m_Controls.generateAxialCut_pushButton, &QPushButton::clicked, this, &HTONDI::OnStartStateAxialCutClicked);
	connect(m_Controls.checkAxialCut_pushButton, &QPushButton::clicked, this, &HTONDI::OnCheckStateCutClicked);
	
	// 2.3 �����عǿ���
	connect(m_Controls.guideSagCut_pushButton, &QPushButton::clicked, this, &HTONDI::OnStartSagGuideClicked);
	connect(m_Controls.setSagCut_pushButton, &QPushButton::clicked, this, &HTONDI::OnSetSagCutClicked);
	connect(m_Controls.startSagCut_pushButton, &QPushButton::clicked, this, &HTONDI::OnStartSagCutClicked);
	connect(m_Controls.endSagCut_pushButton, &QPushButton::clicked, this, &HTONDI::OnEndSagCutClicked);
	// ��̬ģ��
	connect(m_Controls.generateSagCut_pushButton, &QPushButton::clicked, this, &HTONDI::OnStartStateSagCutClicked);
	connect(m_Controls.checkAxialCut_pushButton, &QPushButton::clicked, this, &HTONDI::OnCheckStateCutClicked);


	// 3. ����������֤
	// 3.1 ����ģ�ͽع� + �ſ�����
	connect(m_Controls.checkCutResult_pushButton, &QPushButton::clicked, this, &HTONDI::OnCheckCutResultClicked);
	connect(m_Controls.generateCutSurface_pushButton, &QPushButton::clicked, this, &HTONDI::OnGenerateCutSurfaceClicked);
	connect(m_Controls.startCutAngleGuide_pushButton, &QPushButton::clicked, this, &HTONDI::OnStartCutAngleGuideClicked);

	// 3.2 ����������ʾ
	connect(m_Controls.forceLineCaculate03_pushButton, &QPushButton::clicked, this, &HTONDI::OnForceLineCaculate03Clicked);
	connect(m_Controls.forceLineVisulize_pushButton, &QPushButton::clicked, this, &HTONDI::OnForceLineVisulizeClicked);

	// 3.3 �ְ�̶�ȷ��
	connect(m_Controls.setSteelPos_pushButton, &QPushButton::clicked, this, &HTONDI::OnSetSteelClicked);

	
}

void HTONDI::CreateQT_PostoperativeValid()
{
	/* ��������
		1. ���߼��㲢��ʾ
		2. ����/��ʾ����
		3. ����/��ʾ����
	*/
	
	// ���߼��㲢��ʾ
	connect(m_Controls.showRes_pushButton, &QPushButton::clicked, this, &HTONDI::OnShowResClicked);
	// ����/��ʾ����
	connect(m_Controls.unshowRes_pushButton, &QPushButton::clicked, this, &HTONDI::OnUnshowResClicked);
	// ����/��ʾ����
	connect(m_Controls.caculateError_pushButton, &QPushButton::clicked, this, &HTONDI::OnCaculateErrorClicked);
}

