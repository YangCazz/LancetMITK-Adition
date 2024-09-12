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
== 注册面板控件
== 初始化变量控制名(未适用)
===============================================================*/






void HTONDI::SetFocus()
{
	// do nothing, but you can't delete it
}

void HTONDI::CreateQtPartControl(QWidget *parent)
{
  // create GUI widgets from the Qt Designer's .ui file

  /* 《春立HTO插件》
	各面板控件及功能注册模块
	-. 注册基础功能面板
	1. 术前规划
	2. 术中注册
	3. 术中导航
	4. 术后评估
	-. 数据移动
	-. 测试页
	-. 变量初始化
  */

  m_Controls.setupUi(parent);


  // -. 注册基础功能面板
  CreatQT_BasicSet();

  // 1. 术前规划
  CreateQT_PreoperativePlan();


  // 2. 术中注册
  CreateQT_MidPlan();

  // 3. 术中导航
  CreateQT_MidOperation();

  // 4. 术后评估
  CreateQT_PostoperativeValid();


  // -. 数据移动
  CreateQT_MoveData();


  // -. 测试页
  CreateQT_FuncTest();
}

void HTONDI::ParamInit()
{
	/* 初始化变量名称标准(文本类型)

	==========尚未适用===========
		1. 骨表面点
		2. 器械配准点
		3. RF定义命名
		4. 术前规划 设计变量名
		5. 术中注册 设计变量名
		6. 术中导航 设计变量名
		7. 术后验证 设计变量名
	==========尚未适用===========

	*/

	// 器械参考阵列命名
	std::string name_RF_Femur = "FemurRF";
	std::string name_RF_Tibia = "TibiaRF";
	std::string name_RF_Probe = "ProbeRF";
	std::string name_RF_Saw = "SawRF";
	std::string name_RF_Drill = "DrillRF";
	std::string name_RF_Calibrator = "CalibratorRF";

	// 左右腿 - 0左腿 1右腿
	bool type_Leg = 0;

	// 骨表面点


	// 1.1. 术前规划01
	// PT点集 SF表面 ND点 + 用途

	// 装载准备好的数据
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

	// 1.2. 术前规划02

	// 1.2.1 截骨面规划
	std::string name_SF_CutPlaneAxial = "1st cut plane";
	std::string name_SF_CutPlaneSag = "2nd cut plane";
	std::string name_SF_ProximalTibiaSurface = "proximal tibiaSurface";
	std::string name_SF_DistalTibiaSurface = "distal tibiaSurface";
	std::string name_PT_PointsInCutPlaneAxial = "pointSetInPlaneCutPlane1";

	// 1.2.3 力线规划
	std::string name_ND_legForceLine01 = "legForceLine";
	std::string name_ND_legForceLine02 = "legForceLineNew";
	std::string name_ND_legForceLine03 = "legForceLineLast";

	std::string name_ND_AnkleCenter01 = "ankleCenterPoint";
	std::string name_ND_AnkleCenter02 = "ankleCenterPointNew";
	std::string name_ND_AnkleCenter03 = "ankleCenterPointLast";

	std::string name_PT_TibiaDistalPointSet = "tibiaDistalPointSet";

	// 2. 术中注册


	// 3. 术中导航


	// 4. 术后验证




}

void HTONDI::CreatQT_BasicSet()
{	
	/* 基本功能面板
		1. 相机连接
	*/

	// 1. 相机连接 NDI
	connect(m_Controls.connectNDI_pushButton, &QPushButton::clicked, this, &HTONDI::OnConnectNDIClicked);

}

void HTONDI::CreateQT_FuncTest()
{
	/* 功能测试页
	
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

	// 绑定图像
	connect(m_Controls.pushButton_linkSurface, &QPushButton::clicked, this, &HTONDI::OnLinkSurfaceClicked);
	

	// 测试
	connect(m_Controls.pushButton_HTO_init, &QPushButton::clicked, this, &HTONDI::OnInitHTOTibiaRegisClicked);
	connect(m_Controls.pushButton_HTO_Collect_landmark, &QPushButton::clicked, this, &HTONDI::OnCollectHTOTibiaLandmarkClicked);
	connect(m_Controls.pushButton_HTO_Collect_lcp, &QPushButton::clicked, this, &HTONDI::OnCollectHTOTibiaICPClicked);
	connect(m_Controls.pushButton_HTO_Caculate_regis, &QPushButton::clicked, this, &HTONDI::OnCollectHTOTibiaRegisClicked);

	// 更新摆锯磨钻位置
	connect(m_Controls.pushButton_Renew_Saw, &QPushButton::clicked, this, &HTONDI::RenewSaw);
	connect(m_Controls.pushButton_Renew_Drill, &QPushButton::clicked, this, &HTONDI::RenewDrill);
}

void HTONDI::CreateQT_MoveData()
{
	/* 物体移动
		1. 术前规划
		2. 术中导航
	*/

	// 术前规划 物体移动
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

	// 术中导航 物体运动
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
	/* 术前规划
		1. 调试数据准备
		2. 截骨面规划
		3. 克式钉规划
		3. 力线规划
		4. 撑开角度规划
		5. 钢板规划
	*/


	// 1. 调试数据准备
	// 基本数据准备
	connect(m_Controls.checkBaseData_pushButton, &QPushButton::clicked, this, &HTONDI::OnCheckBaseDataClicked);

	// 计算影像空间股骨中心
	connect(m_Controls.getFemurCenter_pushButton, &QPushButton::clicked, this, &HTONDI::OnFemurCenterClicked);


	// 2. 截骨面规划
	connect(m_Controls.createCutPlane_pushButton, &QPushButton::clicked, this, &HTONDI::OnGenerateCutPlaneClicked);
	connect(m_Controls.cutTibia_pushButton, &QPushButton::clicked, this, &HTONDI::OnCutTibetClicked);
	connect(m_Controls.resetCut_pushButton, &QPushButton::clicked, this, &HTONDI::OnResetCutClicked);

	// 3. 克式钉规划
	connect(m_Controls.generateKeshiPinPos_pushButton, &QPushButton::clicked, this, &HTONDI::OnGenerateKeshiPinClicked);
	connect(m_Controls.recordKeshiPinPos_pushButton, &QPushButton::clicked, this, &HTONDI::OnRecordKeshiPinClicked);
	connect(m_Controls.visuliazeKeshiPin_pushButton, &QPushButton::clicked, this, &HTONDI::OnVisiualKeshiPinClicked);


	// 3. 力线规划
	// 计算并显示力线
	connect(m_Controls.showForceLine_pushButton, &QPushButton::clicked, this, &HTONDI::OnShowMachineLineClicked);

	// 隐藏/显示力线
	connect(m_Controls.unshowForceLine_pushButton, &QPushButton::clicked, this, &HTONDI::OnUnshowMachineLineClicked);

	// 规划下肢力线实时生成
	connect(m_Controls.showForceLine02_pushButton, &QPushButton::clicked, this, &HTONDI::OnShowMachineLine02Clicked);
	connect(m_Controls.unshowForceLine02_pushButton, &QPushButton::clicked, this, &HTONDI::OnUnshowMachineLine02Clicked);


	// 4. 撑开角度规划
	// 计算规划角度并可视化
	connect(m_Controls.caculateAngle_pushButton, &QPushButton::clicked, this, &HTONDI::OnCaculateStrechAngleClicked);

	// 重置规划角度
	connect(m_Controls.resetAngle_pushButton, &QPushButton::clicked, this, &HTONDI::OnResetAngleClicked);

	// 旋转胫骨远端
	connect(m_Controls.pushButton_lt, &QPushButton::clicked, this, &HTONDI::RotateMinus);
	connect(m_Controls.pushButton_rt, &QPushButton::clicked, this, &HTONDI::RotatePlus);


	// 5. 钢板规划
	// 显示/隐藏 钢板
	connect(m_Controls.showSteel_pushButton, &QPushButton::clicked, this, &HTONDI::OnShowSteelClick);

	// 重置钢板位置
	connect(m_Controls.resetSteel_pushButton, &QPushButton::clicked, this, &HTONDI::OnResetSteelClick);

	// 确认钢板规划位置
	connect(m_Controls.setFinalSteelPos_pushButton, &QPushButton::clicked, this, &HTONDI::OnSetFinalPosClick);

}

void HTONDI::CreateQT_MidPlan()
{
	/* 术中注册
		1. 探针标定
		2. 标记点验证
		3. 手术器械标定
		4. 装载配准点
		5. 实时标记点验证
		6. 配准
	
	*/

	// 1. 探针标定
	connect(m_Controls.selectPointer_pushButton, &QPushButton::clicked, this, &HTONDI::OnSelectPointerClicked);
	connect(m_Controls.calibratePointer_pushButton, &QPushButton::clicked, this, &HTONDI::OnCalibratePointerClicked);


	// 2. 标记点验证
	connect(m_Controls.checkFemurNode_pushButton, &QPushButton::clicked, this, &HTONDI::OnFemurCheckClicked);
	connect(m_Controls.checkTibiaNode_pushButton, &QPushButton::clicked, this, &HTONDI::OnTibiaCheckClicked);

	// 实时测量精度
	connect(m_Controls.checkFemurNodeTime_pushButton, &QPushButton::clicked, this, &HTONDI::OnFemurCheckClicked);
	connect(m_Controls.checkTibiaNodeTime_pushButton, &QPushButton::clicked, this, &HTONDI::OnTibiaCheckClicked);


	// 3. 手术器械标定
	// 装载 摆锯 和 磨钻 信息
	connect(m_Controls.loadDeviceLand_pushButton, &QPushButton::clicked, this, &HTONDI::OnLoadDeviceLandmarkClicked);

	// 摆锯标定/磨钻标定 与 验证
	connect(m_Controls.calibrateSaw_pushButton, &QPushButton::clicked, this, &HTONDI::OnCalibrateSawClicked);
	connect(m_Controls.calibrateDrill_pushButton, &QPushButton::clicked, this, &HTONDI::OnCalibrateDrillClicked);

	// 显示/隐藏
	connect(m_Controls.visualizeSaw_pushButton, &QPushButton::clicked, this, &HTONDI::OnSawVisualizeClicked);
	connect(m_Controls.visualizeDrill_pushButton, &QPushButton::clicked, this, &HTONDI::OnDrillVisualizeClicked);

	// 重置
	connect(m_Controls.resetSawCali_pushButton, &QPushButton::clicked, this, &HTONDI::OnresetSawCaliClicked);
	connect(m_Controls.resetDrillCali_pushButton, &QPushButton::clicked, this, &HTONDI::OnresetDrillCaliClicked);


	// 4. 装载 骨表面 配准点信息
	connect(m_Controls.loadBonePoints_pushButton, &QPushButton::clicked, this, &HTONDI::OnLoadBonePointsClicked);
	

	// 5. 配准
	// 粗配准 配准采点 与 配准计算
	connect(m_Controls.getFemurLandmark_pushButton, &QPushButton::clicked, this, &HTONDI::OnGetFemurLandmarkClicked);
	
	// 获得股骨头中心
	connect(m_Controls.collectBonePos_pushButton, &QPushButton::clicked, this, &HTONDI::OnCollectBonePosClicked);
	connect(m_Controls.cacultateBoneCenter_pushButton, &QPushButton::clicked, this, &HTONDI::OnCaculateBoneCenterClicked);
	
	// 股骨粗配准
	connect(m_Controls.caculateFemurLandmark_pushButton, &QPushButton::clicked, this, &HTONDI::OnCaculateFemurLandmarkClicked);

	// 重置
	connect(m_Controls.pushButton_HTO_initTibia, &QPushButton::clicked, this, &HTONDI::OnInitHTOTibiaRegisClicked);

	// 胫骨粗配准
	connect(m_Controls.getTibiaLandmark_pushButton, &QPushButton::clicked, this, &HTONDI::OnCollectHTOTibiaLandmarkClicked);
	connect(m_Controls.caculateTibiaLandmark_pushButton, &QPushButton::clicked, this, &HTONDI::OnCollectHTOTibiaRegisClicked);
	

	// 精配准 配准采点 与 配准计算
	connect(m_Controls.getTibieICP_pushButton, &QPushButton::clicked, this, &HTONDI::OnCollectHTOTibiaICPClicked);
	connect(m_Controls.caculateTibiaICP_pushButton, &QPushButton::clicked, this, &HTONDI::OnCollectHTOTibiaRegisClicked);

}

void HTONDI::CreateQT_MidOperation()
{
	/* 术中导航
	* 1. 磨钻导航
	* 2. 截骨导航
	* 3. 磨钻导航
	* 4. 术中力线验证
	*/ 
	
	// 1. 磨钻导航
	connect(m_Controls.pushButton_ChangeKeShiPin, &QPushButton::clicked, this, &HTONDI::OnChangeKeShiPinClicked);
	connect(m_Controls.pushButton_ChangeLinkPin, &QPushButton::clicked, this, &HTONDI::OnChangeLinkPinClicked);

	connect(m_Controls.pushButton_setLink01, &QPushButton::clicked, this, &HTONDI::OnSetLink01Clicked);
	connect(m_Controls.pushButton_setLink02, &QPushButton::clicked, this, &HTONDI::OnSetLink02Clicked);
	connect(m_Controls.pushButton_setLink03, &QPushButton::clicked, this, &HTONDI::OnSetLink03Clicked);

	connect(m_Controls.pushButton_setKeShi01, &QPushButton::clicked, this, &HTONDI::OnSetKeShi01Clicked);
	connect(m_Controls.pushButton_setKeShi02, &QPushButton::clicked, this, &HTONDI::OnSetKeShi02Clicked);

	connect(m_Controls.pushButton_finishCurrent, &QPushButton::clicked, this, &HTONDI::OnFinishCurrentDrillClicked);
	// 钢板安装导航
	connect(m_Controls.guideDrill_pushButton, &QPushButton::clicked, this, &HTONDI::OnStartDrillGuideClicked);
	connect(m_Controls.startDrill_pushButton, &QPushButton::clicked, this, &HTONDI::OnStartDrillHoleClicked);
	

	// 2. 截骨导航
	
	// 2.1 角度验证
	connect(m_Controls.guideAxialCut_pushButton, &QPushButton::clicked, this, &HTONDI::OnStartAxialGuideClicked);
	connect(m_Controls.pushButton_startAngleCheck, &QPushButton::clicked, this, &HTONDI::OnStartAngleCheckClicked);
	connect(m_Controls.startAxialCut_pushButton, &QPushButton::clicked, this, &HTONDI::OnSetAxialCutClicked);

	// 2.2 水平截骨控制
	connect(m_Controls.pushButton_startAxialCut, &QPushButton::clicked, this, &HTONDI::OnStartAxialCutClicked);
	connect(m_Controls.pushButton_stopAxialCut, &QPushButton::clicked, this, &HTONDI::OnStopAxialCutClicked);
	// 静态模拟
	connect(m_Controls.generateAxialCut_pushButton, &QPushButton::clicked, this, &HTONDI::OnStartStateAxialCutClicked);
	connect(m_Controls.checkAxialCut_pushButton, &QPushButton::clicked, this, &HTONDI::OnCheckStateCutClicked);
	
	// 2.3 上升截骨控制
	connect(m_Controls.guideSagCut_pushButton, &QPushButton::clicked, this, &HTONDI::OnStartSagGuideClicked);
	connect(m_Controls.setSagCut_pushButton, &QPushButton::clicked, this, &HTONDI::OnSetSagCutClicked);
	connect(m_Controls.startSagCut_pushButton, &QPushButton::clicked, this, &HTONDI::OnStartSagCutClicked);
	connect(m_Controls.endSagCut_pushButton, &QPushButton::clicked, this, &HTONDI::OnEndSagCutClicked);
	// 静态模拟
	connect(m_Controls.generateSagCut_pushButton, &QPushButton::clicked, this, &HTONDI::OnStartStateSagCutClicked);
	connect(m_Controls.checkAxialCut_pushButton, &QPushButton::clicked, this, &HTONDI::OnCheckStateCutClicked);


	// 3. 术中力线验证
	// 3.1 术中模型截骨 + 撑开导航
	connect(m_Controls.checkCutResult_pushButton, &QPushButton::clicked, this, &HTONDI::OnCheckCutResultClicked);
	connect(m_Controls.generateCutSurface_pushButton, &QPushButton::clicked, this, &HTONDI::OnGenerateCutSurfaceClicked);
	connect(m_Controls.startCutAngleGuide_pushButton, &QPushButton::clicked, this, &HTONDI::OnStartCutAngleGuideClicked);

	// 3.2 术中力线显示
	connect(m_Controls.forceLineCaculate03_pushButton, &QPushButton::clicked, this, &HTONDI::OnForceLineCaculate03Clicked);
	connect(m_Controls.forceLineVisulize_pushButton, &QPushButton::clicked, this, &HTONDI::OnForceLineVisulizeClicked);

	// 3.3 钢板固定确认
	connect(m_Controls.setSteelPos_pushButton, &QPushButton::clicked, this, &HTONDI::OnSetSteelClicked);

	
}

void HTONDI::CreateQT_PostoperativeValid()
{
	/* 术后评估
		1. 力线计算并显示
		2. 隐藏/显示力线
		3. 隐藏/显示力线
	*/
	
	// 力线计算并显示
	connect(m_Controls.showRes_pushButton, &QPushButton::clicked, this, &HTONDI::OnShowResClicked);
	// 隐藏/显示力线
	connect(m_Controls.unshowRes_pushButton, &QPushButton::clicked, this, &HTONDI::OnUnshowResClicked);
	// 隐藏/显示力线
	connect(m_Controls.caculateError_pushButton, &QPushButton::clicked, this, &HTONDI::OnCaculateErrorClicked);
}

