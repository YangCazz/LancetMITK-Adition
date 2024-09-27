/*============================================================================
Robot-assisted Surgical Navigation Extension based on MITK
Copyright (c) Hangzhou LancetRobotics,CHINA
All rights reserved.
============================================================================*/

#ifndef HTONDI_h
#define HTONDI_h

#include <berryISelectionListener.h>
#include <QmitkAbstractView.h>

//#include "kukaRobotDevice.h"
#include "mitkVirtualTrackingDevice.h"
#include "mitkVirtualTrackingTool.h"
#include "lancetNavigationObjectVisualizationFilter.h"
#include "lancetApplyDeviceRegistratioinFilter.h"
#include "lancetApplySurfaceRegistratioinFilter.h"
#include "lancetApplySurfaceRegistratioinStaticImageFilter.h"
#include "lancetPathPoint.h"
#include "mitkTrackingDeviceSource.h"
#include "robotRegistration.h"
#include <mitkDataStorage.h>
#include <mitkDataNode.h>
#include "QmitkSingleNodeSelectionWidget.h"
#include "mitkPointSet.h"
#include <QmitkDataStorageTreeModel.h>
#include "mitkGizmo.h"

// user .h
#include "ui_HTONDIControls.h"

/**
  \brief HTONDI
  \warning  This class is not yet documented. Use "git blame" and ask the author to provide basic documentation.
  \sa QmitkAbstractView
  \ingroup ${plugin_target}_internal
*/
class HTONDI : public QmitkAbstractView
{
  // this is needed for all Qt objects that should have a Qt meta-object
  // (everything that derives from QObject and wants to have signal/slots)
  Q_OBJECT

public:
  static const std::string VIEW_ID;

  
// 槽函数
public slots:
	// ========================= 基本功能 ==============================
	bool OnConnectNDIClicked();
	bool OnVegaVisualizeTimer();
	bool UpdateToolStatusWidget();
	bool ShowToolStatus_Vega();
	// ================================================================

	// ========================= 测试页面 ==============================
	// Image Resgistration
	bool OnSetNavigateClicked();
	bool OnShowNodeClicked();
	bool OnReuseMatrixClicked();

	
	bool OnGetLandMarkPointClicked();
	bool OnGetICPPointClicked();
	bool OnShowCollectionClicked();
	bool OnCaculateRegistrationClicked();

	bool OnCheckAccuracyPointsClicked();
	bool OnCheckNode2NodeClicked();
	bool OnCheckNode2SurfaceClicked();
	
	// 移动测试
	bool OnMoveTestClicked();
	bool OnMoveBoneClicked();
	
	// 绑定图像
	bool OnLinkSurfaceClicked();

	void RenewSaw();
	void RenewDrill();
	// =================================================================

	// ========================== 术前校准 ==============================
	bool CollectProbeData();
	bool OnCollectPointerPosClicked();
	// =================================================================

	// ========================== 术前规划 ==============================
	// 校验数据准备
	bool OnCheckBaseDataClicked();

	// 计算股骨头中心
	bool OnFemurCenterClicked();

	// 截骨面规划
	bool OnGenerateCutPlaneClicked();
	bool OnCutTibetClicked();
	bool OnResetCutClicked();

	// 克式钉规划
	bool OnGenerateKeshiPinClicked();
	bool OnRecordKeshiPinClicked();
	bool OnVisiualKeshiPinClicked();

	// 下肢力线规划
	bool OnShowMachineLineClicked(); 
	bool OnUnshowMachineLineClicked();
	bool OnShowMachineLine02Clicked();
	bool OnUnshowMachineLine02Clicked();

	// 撑开角度计划
	bool OnCaculateStrechAngleClicked();
	bool OnCaculateStrechAngleClicked02();
	bool OnResetAngleClicked();
	bool OnsetAngleLineClicked();

	// 旋转截骨远端
	void RotateMinus();
	void RotatePlus();

	// 钢板规划
	bool OnShowSteelClick();
	bool OnResetSteelClick();
	bool OnSetFinalPosClick();
	// =================================================================

	// ========================== 术中注册 ==============================
	// 探针校准
	bool OnSelectPointerClicked();
	bool OnCalibratePointerClicked();

	// 标记点验证
	bool OnFemurCheckClicked();
	bool OnTibiaCheckClicked();

	// 数据装载-设备标记点
	bool OnLoadDeviceLandmarkClicked();

	// 器械标定
	bool OnCalibrateSawClicked();
	bool OnCalibrateDrillClicked();

	// 可视化
	bool OnSawVisualizeClicked();
	bool OnDrillVisualizeClicked();

	// 重置
	bool OnresetSawCaliClicked();
	bool OnresetDrillCaliClicked();

	// 数据装载-骨标记点
	bool OnLoadBonePointsClicked();

	// 配准计算
	bool OnGetFemurLandmarkClicked();
	// 获取世界坐标系下的股骨头中心位置
	bool OnCollectBonePosClicked();
	bool OnCaculateBoneCenterClicked();
	// 股骨粗配准
	bool OnCaculateFemurLandmarkClicked();

	// 胫骨粗配准
	bool OnGetTibiaLandmarkClicked();
	bool OnCaculateTibiaLandmarlClicked();

	// 胫骨精配准
	bool OnGetTibieICPClicked();
	bool OnCaculateTibiaICPClicked();

	// 探针跟踪
	void UpdateHTOProbe();
	void UpdateHTOProbe02();

	// 参数初始化
	void OnInitHTOTibiaRegisClicked();
	void OnInitHTOFemurRegisClicked();

	// 胫骨粗配准采集
	void OnCollectHTOTibiaLandmarkClicked();

	// 股骨粗配准采集
	void OnCollectHTOFemurLandmarkClicked();

	// 胫骨精配准采集
	void OnCollectHTOTibiaICPClicked();

	// 胫骨配准计算
	bool OnCollectHTOTibiaRegisClicked();
	// =================================================================

	// ========================== 术中导航 ==============================
	// 获取实时信息
	void GenerateRealTimeBoneSurface();

	// 开始摆锯导航
	bool OnStartAxialGuideClicked();
	bool OnStartAngleCheckClicked();
	bool OnSetAxialCutClicked();
	// 水平截骨
	bool OnStartAxialCutClicked();
	bool OnStopAxialCutClicked();

	// 上升截骨
	bool OnStartSagGuideClicked();
	bool OnSetSagCutClicked();
	bool OnStartSagCutClicked();
	bool OnEndSagCutClicked();

	// 静态
	bool OnCheckStateCutClicked();
	bool OnStartStateAxialCutClicked();
	bool OnStartStateSagCutClicked();

	
	// 磨钻导航
	bool OnStartDrillGuideClicked();
	bool OnStartDrillHoleClicked();
	bool OnStartStateDrillHoleClicked();

	void UpdateHTOSaw();
	void UpdateHTODrill();

	void UpdateHTOSaw02();
	void UpdateHTODrill02();

	void UpdateHTOFemur();

	bool OnChangeKeShiPinClicked();
	bool OnChangeLinkPinClicked();

	bool OnSetKeShi01Clicked();
	bool OnSetKeShi02Clicked();

	bool OnSetLink01Clicked();
	bool OnSetLink02Clicked();
	bool OnSetLink03Clicked();

	bool OnFinishCurrentDrillClicked();

	// 术中力线验证
	bool OnCheckCutResultClicked();
	bool OnGenerateCutSurfaceClicked();
	bool OnStartCutAngleGuideClicked();

	bool OnForceLineCaculate03Clicked();
	bool OnForceLineVisulizeClicked();

	bool OnSetSteelClicked();

	// =================================================================
	
	// ========================== 术后验证 ==============================
	bool OnShowResClicked();
	bool OnUnshowResClicked();
	bool OnCaculateErrorClicked();
	// ================================================================

	// ========================= 物体移动 ==============================
    // 物体旋转
	void Rotate(const double center[3], double direction[3], double counterclockwiseDegree, mitk::BaseData* data);
	// 物体平移
	void Translate(double direction[3], double length, mitk::BaseData* data);
	// 物体的一般运动
	void TranslatePlusX();
	void TranslatePlusY();
	void TranslatePlusZ();
	void TranslateMinusX();
	void TranslateMinusY();
	void TranslateMinusZ();
	void RotatePlusX();
	void RotatePlusY();
	void RotatePlusZ();
	void RotateMinusX();
	void RotateMinusY();
	void RotateMinusZ();

	// 术中模拟运动
	void TranslatePlusX2();
	void TranslatePlusY2();
	void TranslatePlusZ2();
	void TranslateMinusX2();
	void TranslateMinusY2();
	void TranslateMinusZ2();
	void RotatePlusX2();
	void RotatePlusY2();
	void RotatePlusZ2();
	void RotateMinusX2();
	void RotateMinusY2();
	void RotateMinusZ2();
	// ================================================================

protected:
  // QT Plane
  virtual void CreateQtPartControl(QWidget *parent) override;
  virtual void SetFocus() override;
  // -----------------QT Plane init---------------------------
  // 注册基础功能面板
  void CreatQT_BasicSet();

  // 1. 术前规划
  void CreateQT_PreoperativePlan();

  // 2. 术中注册
  void CreateQT_MidPlan();

  // 3. 术中导航
  void CreateQT_MidOperation();

  // 4. 术后评估
  void CreateQT_PostoperativeValid();

  // 数据移动
  void CreateQT_MoveData();

  // 测试页
  void CreateQT_FuncTest();
  // =================================================================

  //-------------------funcs and prams------------------------
  // param init
  void HTONDI::ParamInit();
  
  // NDI
  // vega trackingDeviceSource
  mitk::TrackingDeviceSource::Pointer m_VegaSource;
  mitk::NavigationToolStorage::Pointer m_VegaToolStorage;
  lancet::NavigationObjectVisualizationFilter::Pointer m_VegaVisualizer;
  QTimer* m_VegaVisualizeTimer{ nullptr };
  std::vector<mitk::NavigationData::Pointer> m_VegaNavigationData;
  lancet::ApplySurfaceRegistratioinStaticImageFilter::Pointer m_surfaceRegistrationStaticImageFilter;
  mitk::AffineTransform3D::Pointer m_imageRegistrationMatrix;
  mitk::AffineTransform3D::Pointer m_PreviousImageRegistrationMatrix;
  // state update

  // Image Resgiration
  // Mitk object load
  void InitSurfaceSelector(QmitkSingleNodeSelectionWidget* widget);
  void InitPointSetSelector(QmitkSingleNodeSelectionWidget* widget);
  // Naviagation Image
  lancet::NavigationObject::Pointer navigatedImage;
  // 创建两个点集，一个用于存储已采集的点，一个用于存储待采集的点
  mitk::DataNode::Pointer collectedPointsNode = mitk::DataNode::New();
  mitk::PointSet::Pointer collectedPoints = mitk::PointSet::New();
  mitk::Point3D currentpoint;
  bool firstnode = true;
  // set model param
  // 设置模型参数
  void SetModelColor(mitk::DataNode::Pointer node, float r, float g, float b);
  void SetNodeSize(mitk::DataNode::Pointer node, float size);
  void SetModelOpacity(mitk::DataNode::Pointer node, float opacity);
  // 拟合股骨头中
  std::pair<Eigen::Vector3d, double> FitSphere(const std::vector<Eigen::Vector3d>& points);
  // 点击后选中物体  
  QmitkDataStorageTreeModel* m_NodetreeModel{ nullptr };
  mitk::BaseData* m_baseDataToMove{ nullptr };
  mitk::DataNode* m_currentSelectedNode{ nullptr };
  std::string current = "";
  virtual void OnSelectionChanged(berry::IWorkbenchPart::Pointer source,
	  const QList<mitk::DataNode::Pointer>& nodes) override;


  

  // HTO Surgical Simulate
  // Get Center
  std::vector<Eigen::Vector3d> m_FemurPositions; // 存储晃动过程中的位置
  Eigen::Vector3d m_HipCenter; // 拟合股骨头中心点
  QTimer* m_FemurDataCollectionTimer = { nullptr }; //实时采集信息
  // Curve funcs
  bool CollectFemurData();
  bool CalculateHipCenter();
  //
  mitk::NavigationData::Pointer GetNavigationDataInRef(mitk::NavigationData::Pointer nd,
	  mitk::NavigationData::Pointer nd_ref);
  
  // Surgical Simulate
  int number = 100;
  mitk::NavigationTool::Pointer m_ToolToCalibrate;
  mitk::NavigationData::Pointer m_ComputedToolTipTransformation;
  std::vector<Eigen::Vector3d> m_ProbePositions; // 存储晃动过程中的位置
  std::vector<Eigen::Matrix4d> m_Transformations; // 存储F_camera至F_probe的转换矩阵
  Eigen::Vector3d m_ProbeTip;
  Eigen::Vector3d m_Center;
  int groupSize = 50;
  QTimer* m_ProbeDataCollectionTimer = {nullptr};
  // 计算针尖位置
  bool CalculateProbeTip();
  void RegisterProbePosition();
  // 计算探针标定误差
  double CalculateCalibrationError();
  // 可视化采集的节点
  bool OnVisualizeCollectedPoints();
  // ========================= 全局导航数据 ==========================
  double m_T_cameraToProbeRF[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
  double m_T_cameraToFemurRF[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
  double m_T_cameraToTibiaRF[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
  double m_T_cameraToSawRF[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
  double m_T_cameraToDrillRF[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
  double m_T_cameraToCalibratorRF[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
  
  // 计算得到转化矩阵
  double m_T_ProbeRFtoImage_probe[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
  double m_T_imageToProbe[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
  double m_T_ImageToDrill[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };

  double m_T_calibratorRFtoDrill[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
  double m_T_DrillRFtoImage_drill[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
  
  double m_T_SawRFtoImage_saw[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
  double m_T_ImageToSaw[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };

  // ========================= 术前规划 ==============================
  // 手术类型注册-默认为左腿
  // 0:右腿. 1：左腿
  int judgModel_flag = 1;
  bool CreateOneCutPlane();
  bool CutTibiaWithOnePlane();
  bool CutTibiaWithTwoPlanes();
  bool GetPlaneProperty(vtkSmartPointer<vtkPolyData> plane, double normal[3], double center[3]);
  // 截骨函数
  bool CutPolyDataWithPlane(vtkSmartPointer<vtkPolyData> dataToCut,
	  vtkSmartPointer<vtkPolyData> largerSubPart,
	  vtkSmartPointer<vtkPolyData> smallerSubPart,
	  double planeOrigin[3], double planeNormal[3]);
  // 截骨效应函数
  // 获取实时截骨线
  bool GetIntersectionLine();
  void TraverseIntersectionLines(vtkSmartPointer<vtkPolyData> intersectionLine);
  void GetDistancefromTibia();
  
  //存放规划面与胫骨的交点
  mitk::PointSet::Pointer planeAndTibiaIntersectionPoint;
  mitk::Point3D minPoint;//截骨线末端中点
  mitk::Point3D maxPoint;//截骨线入口点
  
  //HTO
  //Cut tibia into two parts with a plane 
  double distance1;//截骨面入口距内侧平台
  double distance2;//截骨面末端距外侧平台
  double distance3;//保留外侧合页
  double distance4;//距离粗隆
  double depth;//截骨面入骨深度
  double Line_length;//力线的长度
  double angleInDegrees;//撑开角度


  mitk::PointSet::Pointer mitkPointSet1 = mitk::PointSet::New();
 

  mitk::DataNode::Pointer m_DataNode; // Member variable to store the DataNode
  bool OBB = false;

  // 下肢力线规划
  // 力线占比计算和更新
  void updateProportation();
  void updateProportation02();
  void updateProportation03();
  // 计算两个线段的交点
  bool LineLineIntersection(Eigen::Vector2d& intersection, Eigen::Vector2d p1, Eigen::Vector2d p2, Eigen::Vector2d q1, Eigen::Vector2d q2);
  // 计算高度
  void CaculateStrechHeigh();
  // 钢板的初始位置
  mitk::Point3D m_steelPosition;
  // 下肢远端初始位置
  mitk::Point3D m_distalTibiaPosition;

  std::vector<Eigen::Vector3d> InsectionLine2Surface(vtkSmartPointer<vtkPolyData> plane, double start[3], double end[3]);
  // ================================================================

  // ========================= 术中注册 ==============================
  mitk::NavigationTool::Pointer m_femurRFNode;
  mitk::NavigationTool::Pointer m_tibiaRFNode;

  mitk::NavigationTool::Pointer m_saw;
  mitk::NavigationTool::Pointer m_drill;

  lancet::NavigationObject::Pointer probe_image;

  lancet::NavigationObject::Pointer saw_image;
  lancet::NavigationObject::Pointer drill_image;

  lancet::NavigationObject::Pointer femur_image;
  lancet::NavigationObject::Pointer tibia_image;
  // =================================================================

  // -----------------------术中导航-----------------------------
  
  void trackingObjectPos();
  
  
  void CalculateRealTimeCutAngle();

  // 截骨导航 timer
  QTimer* m_timer_saw{ nullptr };
  QTimer* m_timer_strech{ nullptr };


  // 存储实时截骨点
  mitk::PointSet::Pointer mitkPointSetRealTime = mitk::PointSet::New();
  // =================================================================
  
  // 拟合平面 3d, 返回平面中心和法向量
  std::pair<Eigen::Vector3d, Eigen::Vector3d> FitPlane3D(const std::vector<Eigen::Vector3d>& points);
  // 拟合圆 2d, 返回圆心
  std::pair<Eigen::Vector3d, double> FitCircle3D(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& p3);
  // 三角形外心 
  // Eigen::Vector3d HTONDI::CalculateCircumcenter(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& p3);
  bool GetPlaneNormal(vtkSmartPointer<vtkPolyData> plane, double normal[3], double center[3]);
  
  // 提取三个点计算法向量
  Eigen::Vector3d HTONDI::ExtractNormalFromPlane(const std::string& planeName);
  
  // 计算偏差夹角
  int m_cutType = -1;
  void CaculateCutPlaneMiss01();
  void CaculateCutPlaneMiss02();

  // 计算平面上点的相对位置
  std::vector<mitk::Point3D> CalculateRelativePoints(const std::vector<mitk::Point3D>& points, const Eigen::Vector3d& normalVector, const Eigen::Vector3d& origin);
  std::vector<mitk::Point3D> m_PointsOnSaw;
  std::vector<mitk::Point3D> CalculateActualPoints(const std::vector<mitk::Point3D>& relativePoints, const Eigen::Vector3d& normalVector, const Eigen::Vector3d& origin);

  // 计算实时水平截骨线
  bool GetRealTimeIntersectionLine(const std::string& cutPlaneName, const std::string& surfaceName);
  void RealTimeTraverseIntersectionLines(vtkSmartPointer<vtkPolyData> intersectionLine, double pos_min[3], double pos_max[3]);
  // =================================================================
  // 定义-股骨标记点在股骨RF下的位置
  Eigen::Vector4d m_FemurCheckPointOnFemurRF;
  Eigen::Vector4d m_TibiaCheckPointOnTibiaRF;
  // 当前-股骨标记点在Camera下的位置
  Eigen::Vector4d m_CurrentFemurCheckPointOnCamera;
  Eigen::Vector4d m_CurrentTibiaCheckPointOnCamera;
  // 当前-股骨标记点在Image下的位置
  Eigen::Vector4d m_CurrentFemurCheckPointOnImage;
  Eigen::Vector4d m_CurrentTibiaCheckPointOnImage;


  // 当前-探针尖端点在Camera下的位置
  Eigen::Vector4d m_CurrentTipOnCamera;
  // 当前-探针尖端点在Image下的位置
  Eigen::Vector4d m_CurrentTipOnImage;

  // 定义探针两点在坐标系下的点位置
  Eigen::Vector4d m_ProbePointOnProbeRF_A = { 0.0, 0.0, 0.0, 1.0 };
  Eigen::Vector4d m_ProbePointOnProbeRF_B = { -1.0, 0.0, 0.0, 1.0 };

  // 定义磨钻末端点在磨钻物理坐标系下的点位置
  Eigen::Vector4d m_DrillPointOnCalibratorRF_head = { 120.0, 0.0, 0.0, 1.0 };
  Eigen::Vector4d m_DrillPointOnCalibratorRF_tail = { 0.0, 0.0, 0.0, 1.0 };

  // 定义器械标定点在RF下的位置
  std::vector<Eigen::Vector4d> m_SawPointsOnSawRF;
  std::vector<Eigen::Vector4d> m_DrillPointsOnDrillRF;

  // 磨钻在RF下的轴向向量
  Eigen::Vector4d m_DrillAxisOnDrillRF;

  //// 当前器械标定点在Camera下的位置
  //Eigen::Matrix4d m_CurrentSawPointsOnCamera;
  //Eigen::Matrix4d m_CurrentDrillPointsOnCamera;
  //// 当前器械标定点在Image下的位置
  //Eigen::Matrix4d m_CurrentSawPointsOnImage;
  //Eigen::Matrix4d m_CurrentDrillPointsOnImage;

  // 当前器械标定点在Camera下的位置
  std::vector<Eigen::Vector4d> m_CurrentSawPointsOnCamera;
  std::vector<Eigen::Vector4d> m_CurrentDrillPointsOnCamera;
  // 当前器械标定点在Image下的位置
  std::vector<Eigen::Vector4d> m_CurrentSawPointsOnImage;
  std::vector<Eigen::Vector4d> m_CurrentDrillPointsOnImage;


  // 定义 配准矩阵 4*4 
  mitk::AffineTransform3D::Pointer m_MetrixFemurRFToImage;
  mitk::AffineTransform3D::Pointer m_MetrixTibiaRFToImage;
  mitk::AffineTransform3D::Pointer m_MetrixCameraToImage;

  // 定义 配准矩阵 4*4 
  Eigen::Matrix4d m_Metrix4dFemurRFToImage;
  Eigen::Matrix4d m_Metrix4dTibiaRFToImage;
  Eigen::Matrix4d m_Metrix4dCameraToImage;

  // 定义摆锯 [由外向内三点] 之间的两两距离, 1-2 1-3 2-3
  double m_SawDistance[3] = {25.8118, 53.1507, 28.6400};


  // 配准计算
  vtkNew<vtkMatrix4x4> m_ObjectRfToImageMatrix_hto;

  // 导航工具更新计时器
  QTimer* m_HTOPrboeUpdateTimer{ nullptr };
  QTimer* m_HTOSawUpdateTimer{ nullptr };
  QTimer* m_HTODrillUpdateTimer{ nullptr };
  QTimer* m_HTOFemurUpdateTimer{ nullptr };

  // HTO registration optimization
  // 胫骨近端外侧点 胫骨近端内侧点 胫骨远端外踝点 胫骨远端内踝点
  mitk::PointSet::Pointer m_tibiaProximalLateralPoint = mitk::PointSet::New();
  mitk::PointSet::Pointer m_tibiaProximalMedialPoint = mitk::PointSet::New();
  mitk::PointSet::Pointer m_tibiaDistalLateralPoint = mitk::PointSet::New();
  mitk::PointSet::Pointer m_tibiaDistalMedialPoint = mitk::PointSet::New();

  mitk::PointSet::Pointer m_tibiaLandmarkPoints = mitk::PointSet::New();
  mitk::PointSet::Pointer m_femurLandmarkPoints = mitk::PointSet::New();

  mitk::PointSet::Pointer m_tibiaICPPoints = mitk::PointSet::New();

  mitk::Surface::Pointer m_Surface_HTOTibiaSurface = mitk::Surface::New();
  mitk::Surface::Pointer m_Surface_HTOFemurSurface = mitk::Surface::New();

  int m_Num_TibiaLandmark = 0;
  int m_Num_FemurLandmark = 0;
  void CollectHTOTibiaLandmark(int index);
  void CollectHTOFemurLandmark(int index);

  bool start_probe = false;
  bool start_drill = false;
  bool start_saw = false;
  
  bool m_RealtimeAngleCheck = false;
  int AngleCheck_type = -1;
  // 计算初始化截骨面的截骨线
  bool m_RealtimeCutCheck = false;
  bool m_RealtimeCutPlaneCheck = false;
  

  double m_CutIntersectionMinPos[3];
  double m_CutIntersectionMaxPos[3];

  //计算点到直线的距离
  double DistancePointToLine(double pointA[3], double pointB[3], double pointC[3]);

  // 设置截骨阈值
  double m_threshold_SawDepth = 5.0;
  double m_threshold_DrillDepth = 5.0;

  // 摆锯电源状态控制
  bool m_SawPower = false;
  bool m_DrillPower = false;

  // 磨钻标定参数

  // 标记当前磨钻末端应该到达的位置
  double m_drill_tip[3] = { 0.0,0.0,0.0 };

  double m_KeShi_Pos01[3] = { 0.0,0.0,0.0 };
  double m_KeShi_Pos02[3] = { 0.0,0.0,0.0 };

  double m_Current_drill_Pos01[3] = { 0.0,0.0,0.0 };

  //
  std::vector<mitk::Point3D> KeShikPinPos_Set;
  std::vector<mitk::Point3D> LinkPinPos_Set;

  // 记录当前位置

  // 0-keshi 1-link
  int Drilltype = -1;

  mitk::Point3D CurrentDrill_Head;
  mitk::Point3D CurrentDrill_Tail;


  double Destin_Tail[3] = { 0.0,0.0,0.0 };

  double angle_set_xoy[2] = { 60.0,-30.0 };
  double angle_set_xoz[2] = { 10.0,-20.0 };
  double angle_set_yoz[2] = { 10.0,10.0 };

  Eigen::Vector3d normal_KeShi01, normal_KeShi02;

  mitk::PointSet::Pointer KeshiSetpoints = mitk::PointSet::New();
  double Keshi_depth[2];

  // 开始钻孔，记录钻孔初始点位置
  mitk::Point3D StartDrillPoint;

  // 记录当前选中drill模型index
  // 0 1 Keshi, 2 3 4 LinkPin, -1 无
  int Drill_DepthType = -1;

  bool check_axialCut = false;
  bool check_sagCut = false;
  bool check_cutPlane = false;

  bool CutTibiaWithTwoPlanes02();

  // 左腿的转轴是 3->2, a3 b2
  // 右腿的转轴是 0->1, a0 b1
  Eigen::Vector4d point_a_femurRF;
  Eigen::Vector4d point_b_femurRF;

  Eigen::Vector4d point_a_femurRF_current;
  Eigen::Vector4d point_b_femurRF_current;

  Eigen::Vector4d point_a_tibiaRF;
  Eigen::Vector4d point_b_tibiaRF;

  // 保存力线占比和掰开角度
  double line_set, angle_set;

  // 计算合页旋转角
  double calculateAngle(Eigen::Vector2d A, Eigen::Vector2d C, Eigen::Vector2d B1, Eigen::Vector2d direction);

  Ui::HTONDIControls m_Controls;
};

#endif // HTONDI_h