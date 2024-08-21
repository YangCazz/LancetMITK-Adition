/*============================================================================
Robot-assisted Surgical Navigation Extension based on MITK
Copyright (c) Hangzhou LancetRobotics,CHINA
All rights reserved.
============================================================================*/


#ifndef HTOTookits_h
#define HTOTookits_h

#include <berryISelectionListener.h>
#include <QmitkAbstractView.h>

// Kuka
#include "kukaRobotDevice.h"
#include "mitkVirtualTrackingDevice.h"
#include "mitkVirtualTrackingTool.h"

// lancet
#include "lancetNavigationObjectVisualizationFilter.h"
#include "lancetApplyDeviceRegistratioinFilter.h"
#include "lancetApplySurfaceRegistratioinFilter.h"
#include "lancetApplySurfaceRegistratioinStaticImageFilter.h"
#include "lancetPathPoint.h"
#include "mitkTrackingDeviceSource.h"
#include "robotRegistration.h"

// Aimooe x86
#include "AimPositionAPI.h"
#include "AimPositionDef.h"

// user heads
#include "ui_HTOTookitsControls.h"

/**
  \brief HTOTookits

  \warning  This class is not yet documented. Use "git blame" and ask the author to provide basic documentation.

  \sa QmitkAbstractView
  \ingroup ${plugin_target}_internal
*/
class HTOTookits : public QmitkAbstractView
{
  // this is needed for all Qt objects that should have a Qt meta-object
  // (everything that derives from QObject and wants to have signal/slots)
  Q_OBJECT

public:
  static const std::string VIEW_ID;
  // file load
  bool SelectFile(QTextBrowser* textBrowser, QString type);
  bool updateConnectionStatus(const QString& link_type, AIMPOS_TYPE devtype);
  bool TimerConnect(QTimer*& timer, const char* slotr);
  bool ToolSelect(int index, QComboBox* comboBox, std::string, QString type);
  // probe sphere curve

  // Aimooe state
  E_ReturnValue Aim_CheckDeviceConnection(AimHandle aimHandle);
  bool InitializeAndConnectDevice();
  void CheckDeviceConnection();
  Eigen::Vector3d HTOTookits::TransformToReferenceFrame(const Eigen::Vector3d& point, T_AimToolDataResult* rfToolData);

  // 设置模型参数
  void SetModelColor(mitk::DataNode::Pointer node, float r, float g, float b);
  void SetNodeSize(mitk::DataNode::Pointer node, float size);
  void SetModelOpacity(mitk::DataNode::Pointer node, float opacity);
  // 模型移动
  // 平移
  void Translate(double direction[3], double length, mitk::BaseData* data);
  // 旋转
  void Rotate(const double center[3], double direction[3], double counterclockwiseDegree, mitk::BaseData* data);

  // NDI相机系统
  void ShowToolStatus_Vega();

public slots:
	// main cpp
	// device connect
	bool OnConnectAimooeClicked();
	bool OnConnectNDIClicked();
	bool OnConnectArieClicked();
	bool OnConnectKukaClicked();
	bool OnConnectDazuClicked();
	// file load
	bool OnSelectToolFileClicked();
	bool OnSelectToolFolderClicked();
	// tools show
	bool OnStartCollectClicked();
	// tool tracking
	bool OnCheckToolPosClicked();
	// bool UpdateCheckData();
	void UpdateCheckData();
	// comboBox
	bool OnProbeSelected(int index);
	bool OnRfSelected(int index);
	// stop date collect
	void OnStopDataClicked();
	// tracking probe
	bool OnTrackProbeClicked();
	// tracking RF
	bool OnTrackRFClicked();
	// collect Probe
	bool OnCollectProbeClicked();
	// caculate probe
	bool OnCaculateProbeClicked();
	// calibrate RF
	bool OnCaculateRFClicked();
	// 
	bool OnCalibrateRFClicked();

	// Image Registration
	bool OnSetNavigateClicked();
	bool OnShowNodeClicked();
	bool OnReuseMatrixClicked();

	bool OnFemurCenterClicked();
	bool OnGetLandMarkPointClicked();
	bool OnGetICPPointClicked();
	bool OnShowCollectionClicked();
	bool OnTestRegistrationClicked();
	bool OnCaculateRegistrationClicked();
	bool OnApplyRegistration();

	bool OnCheckAccuracyPointsClicked();
	bool OnCheckNode2NodeClicked();
	bool OnCheckNode2SurfaceClicked();

	bool OnMoveTestClicked();
	bool OnMoveBoneClicked();

	// NDI button
	bool OnVegaVisualizeTimer();

protected:
  virtual void CreateQtPartControl(QWidget *parent) override;
  virtual void SetFocus() override;
  // Device Connect
  
  // file load
  QString ToolPath;
  QStringList FileNames;
  bool folder;
  char* toolFolderPath;
  std::vector<std::string> ToolNames;
  
  // Aimooe 句柄 ===> 重构Aimooe项目
#define GET_DATA_FROM_HARDWARE 1
  AimHandle aimHandle = NULL;
  E_Interface EI;
  bool isConnected = false;
  QTimer* connectionCheckTimer = nullptr;

  // Aimooe timer
  QTimer* Timer_Check = nullptr;
  // Aimooe data get
  // 存储每次的数据
  std::map<std::string, std::vector<T_AimToolDataResult>> toolDataResults; 
  // 设定采集的最大数量
  const int MAX_SAMPLES = 20; 
  // tool select 
  std::string toolRf;
  std::string toolProbe;
  std::string current = "all";
  // stop flag
  bool stopDataCollection = false;
  // collect flag
  bool startCollect = false;
  // 存储某个指定工具的信息
  std::vector<T_AimToolDataResult> currentTool;

  // 存储标定后的工具的参考点位置
  std::map<std::string, std::vector<Eigen::Vector3d>> g_calibrationPointsMap;

  // Image Registration
  void InitSurfaceSelector(QmitkSingleNodeSelectionWidget* widget);
  void InitPointSetSelector(QmitkSingleNodeSelectionWidget* widget);
  // Naviagation Image
  lancet::NavigationObject::Pointer navigatedImage;
  // Curve sphere
  std::pair<Eigen::Vector3d, double> FitSphere(const std::vector<Eigen::Vector3d>& points);


  // NDI设备句柄
  // 跟踪设备源
  mitk::TrackingDeviceSource::Pointer m_VegaSource;
  lancet::NavigationObjectVisualizationFilter::Pointer m_VegaVisualizer;
  QTimer* m_VegaVisualizeTimer{ nullptr };
  mitk::NavigationToolStorage::Pointer m_VegaToolStorage;
  std::vector<mitk::NavigationData::Pointer> m_VegaNavigationData;

  Ui::HTOTookitsControls m_Controls;
};

#endif // HTOTookits_h
