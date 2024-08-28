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

	// ��ʼ������������ı�����
	std::string femurHeadPointSet = "femurHeadPointSet";
	std::string hipCenterPoint = "hipCenterPoint";
	std::string femurTrochanterPoint = "femurTrochanterPoint";
	std::string femurLateralMalleolusPoint = "femurLateralMalleolusPoint";
	std::string femurMedialMalleolusPoint = "femurMedialMalleolusPoint";
	std::string femurLandmarkPointSet = "femurLandmarkPointSet";


	std::string tibiaProximalLateralPoint = "tibiaProximalLateralPoint";
	std::string tibiaProximalMedialPoint = "tibiaProximalMedialPoint";
	std::string tibiaDistalLateralPoint = "tibiaDistalLateralPoint";
	std::string tibiaDistalMedialPoint = "tibiaDistalMedialPoint";
	std::string tibiaLandmarkPointSet = "tibiaLandmarkPointSet";


	std::string femurSurface = "femurSurface";
	std::string tibiaSurface = "tibiaSurface";

	std::string cutPlane01 = "1st cut plane";
	std::string cutPlane02 = "2nd cut plane";

	std::string pointOnCutPlane01 = "pointSetInPlaneCutPlane1";
	std::string proximalTibiaSurface = "proximal tibiaSurface";
	std::string distalTibiaSurface = "distal tibiaSurface";

	std::string legForceLine = "legForceLine";
	std::string ankleCenterPoint = "ankleCenterPoint";

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
	connect(m_Controls.pushButton_xp_2, &QPushButton::clicked, this, &HTONDI::TranslatePlusX2);
	connect(m_Controls.pushButton_yp_2, &QPushButton::clicked, this, &HTONDI::TranslatePlusY2);
	connect(m_Controls.pushButton_zp_2, &QPushButton::clicked, this, &HTONDI::TranslatePlusZ2);
	connect(m_Controls.pushButton_xm_2, &QPushButton::clicked, this, &HTONDI::TranslateMinusX2);
	connect(m_Controls.pushButton_ym_2, &QPushButton::clicked, this, &HTONDI::TranslateMinusY2);
	connect(m_Controls.pushButton_zm_2, &QPushButton::clicked, this, &HTONDI::TranslateMinusZ2);
	connect(m_Controls.pushButton_rxp_2, &QPushButton::clicked, this, &HTONDI::RotatePlusX2);
	connect(m_Controls.pushButton_ryp_2, &QPushButton::clicked, this, &HTONDI::RotatePlusY2);
	connect(m_Controls.pushButton_rzp_2, &QPushButton::clicked, this, &HTONDI::RotatePlusZ2);
	connect(m_Controls.pushButton_rxm_2, &QPushButton::clicked, this, &HTONDI::RotateMinusX2);
	connect(m_Controls.pushButton_rym_2, &QPushButton::clicked, this, &HTONDI::RotateMinusY2);
	connect(m_Controls.pushButton_rzm_2, &QPushButton::clicked, this, &HTONDI::RotateMinusZ2);
}




void HTONDI::CreateQT_PreoperativePlan()
{
	/* ��ǰ�滮
		1. ��������׼��
		2. �ع���滮
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

	// �ֹǴ���׼
	connect(m_Controls.getTibiaLandmark_pushButton, &QPushButton::clicked, this, &HTONDI::OnGetTibiaLandmarkClicked);
	connect(m_Controls.caculateTibiaLandmark_pushButton, &QPushButton::clicked, this, &HTONDI::OnCaculateTibiaLandmarlClicked);
	

	// ����׼ ��׼�ɵ� �� ��׼����
	connect(m_Controls.getTibieICP_pushButton, &QPushButton::clicked, this, &HTONDI::OnGetTibieICPClicked);
	connect(m_Controls.caculateTibiaICP_pushButton, &QPushButton::clicked, this, &HTONDI::OnCaculateTibiaICPClicked);

}

void HTONDI::CreateQT_MidOperation()
{
	/* ���е���
		1. �عǵ���
		2. ĥ�굼��
		3. ����������֤
	*/ 
	

	// 1. �عǵ���
	// ˮƽ�ع�
	connect(m_Controls.guideAxialCut_pushButton, &QPushButton::clicked, this, &HTONDI::OnStartAxialGuideClicked);
	connect(m_Controls.startAxialCut_pushButton, &QPushButton::clicked, this, &HTONDI::OnStartAxialCutClicked);
	// ���ɾ�̬ˮƽ�ع�ƽ��
	connect(m_Controls.generateAxialCut_pushButton, &QPushButton::clicked, this, &HTONDI::OnStartStateAxialCutClicked);
	// ȷ��ˮƽ�ع�ƽ��λ��
	connect(m_Controls.checkAxialCut_pushButton, &QPushButton::clicked, this, &HTONDI::OnCheckStateCutClicked);
	


	// �����ع�
	connect(m_Controls.guideSagCut_pushButton, &QPushButton::clicked, this, &HTONDI::OnStartSagGuideClicked);
	connect(m_Controls.startSagCut_pushButton, &QPushButton::clicked, this, &HTONDI::OnStartSagCutClicked);
	// ���ɾ�̬�����ع�ƽ��
	connect(m_Controls.generateSagCut_pushButton, &QPushButton::clicked, this, &HTONDI::OnStartStateSagCutClicked);
	// ȷ�������ع�ƽ��λ��
	connect(m_Controls.checkAxialCut_pushButton, &QPushButton::clicked, this, &HTONDI::OnCheckStateCutClicked);



	// 2. �ְ尲װ����
	connect(m_Controls.guideDrill_pushButton, &QPushButton::clicked, this, &HTONDI::OnStartDrillGuideClicked);
	connect(m_Controls.startDrill_pushButton, &QPushButton::clicked, this, &HTONDI::OnStartDrillHoleClicked);


	// 3. ����������֤


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

