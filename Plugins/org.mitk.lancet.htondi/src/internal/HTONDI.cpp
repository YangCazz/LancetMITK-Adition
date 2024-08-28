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

	// 初始化定义物体的文本定义
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
	/* 术前规划
		1. 调试数据准备
		2. 截骨面规划
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

	// 胫骨粗配准
	connect(m_Controls.getTibiaLandmark_pushButton, &QPushButton::clicked, this, &HTONDI::OnGetTibiaLandmarkClicked);
	connect(m_Controls.caculateTibiaLandmark_pushButton, &QPushButton::clicked, this, &HTONDI::OnCaculateTibiaLandmarlClicked);
	

	// 精配准 配准采点 与 配准计算
	connect(m_Controls.getTibieICP_pushButton, &QPushButton::clicked, this, &HTONDI::OnGetTibieICPClicked);
	connect(m_Controls.caculateTibiaICP_pushButton, &QPushButton::clicked, this, &HTONDI::OnCaculateTibiaICPClicked);

}

void HTONDI::CreateQT_MidOperation()
{
	/* 术中导航
		1. 截骨导航
		2. 磨钻导航
		3. 术中力线验证
	*/ 
	

	// 1. 截骨导航
	// 水平截骨
	connect(m_Controls.guideAxialCut_pushButton, &QPushButton::clicked, this, &HTONDI::OnStartAxialGuideClicked);
	connect(m_Controls.startAxialCut_pushButton, &QPushButton::clicked, this, &HTONDI::OnStartAxialCutClicked);
	// 生成静态水平截骨平面
	connect(m_Controls.generateAxialCut_pushButton, &QPushButton::clicked, this, &HTONDI::OnStartStateAxialCutClicked);
	// 确定水平截骨平面位置
	connect(m_Controls.checkAxialCut_pushButton, &QPushButton::clicked, this, &HTONDI::OnCheckStateCutClicked);
	


	// 上升截骨
	connect(m_Controls.guideSagCut_pushButton, &QPushButton::clicked, this, &HTONDI::OnStartSagGuideClicked);
	connect(m_Controls.startSagCut_pushButton, &QPushButton::clicked, this, &HTONDI::OnStartSagCutClicked);
	// 生成静态上升截骨平面
	connect(m_Controls.generateSagCut_pushButton, &QPushButton::clicked, this, &HTONDI::OnStartStateSagCutClicked);
	// 确定上升截骨平面位置
	connect(m_Controls.checkAxialCut_pushButton, &QPushButton::clicked, this, &HTONDI::OnCheckStateCutClicked);



	// 2. 钢板安装导航
	connect(m_Controls.guideDrill_pushButton, &QPushButton::clicked, this, &HTONDI::OnStartDrillGuideClicked);
	connect(m_Controls.startDrill_pushButton, &QPushButton::clicked, this, &HTONDI::OnStartDrillHoleClicked);


	// 3. 术中力线验证


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

