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

  
// �ۺ���
public slots:
	// ========================= �������� ==============================
	bool OnConnectNDIClicked();
	bool OnVegaVisualizeTimer();
	bool UpdateToolStatusWidget();
	bool ShowToolStatus_Vega();
	// ================================================================

	// ========================= ����ҳ�� ==============================
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
	
	// �ƶ�����
	bool OnMoveTestClicked();
	bool OnMoveBoneClicked();
	
	// ��ͼ��
	bool OnLinkSurfaceClicked();
	
	// =================================================================

	// ========================== ��ǰУ׼ ==============================
	bool CollectProbeData();
	bool OnCollectPointerPosClicked();
	// =================================================================

	// ========================== ��ǰ�滮 ==============================
	// У������׼��
	bool OnCheckBaseDataClicked();

	// ����ɹ�ͷ����
	bool OnFemurCenterClicked();

	// �ع���滮
	bool OnGenerateCutPlaneClicked();
	bool OnCutTibetClicked();
	bool OnResetCutClicked();

	// ��֫���߹滮
	bool OnShowMachineLineClicked(); 
	bool OnUnshowMachineLineClicked();
	bool OnShowMachineLine02Clicked();
	bool OnUnshowMachineLine02Clicked();

	// �ſ��Ƕȼƻ�
	bool OnCaculateStrechAngleClicked();
	bool OnResetAngleClicked();
	// ��ת�ع�Զ��
	void RotateMinus();
	void RotatePlus();

	// �ְ�滮
	bool OnShowSteelClick();
	bool OnResetSteelClick();
	bool OnSetFinalPosClick();
	// =================================================================

	// ========================== ����ע�� ==============================
	// ̽��У׼
	bool OnSelectPointerClicked();
	bool OnCalibratePointerClicked();

	// ��ǵ���֤
	bool OnFemurCheckClicked();
	bool OnTibiaCheckClicked();

	// ����װ��-�豸��ǵ�
	bool OnLoadDeviceLandmarkClicked();

	// ��е�궨
	bool OnCalibrateSawClicked();
	bool OnCalibrateDrillClicked();

	// ���ӻ�
	bool OnSawVisualizeClicked();
	bool OnDrillVisualizeClicked();

	// ����
	bool OnresetSawCaliClicked();
	bool OnresetDrillCaliClicked();

	// ����װ��-�Ǳ�ǵ�
	bool OnLoadBonePointsClicked();

	// ��׼����
	bool OnGetFemurLandmarkClicked();
	// ��ȡ��������ϵ�µĹɹ�ͷ����λ��
	bool OnCollectBonePosClicked();
	bool OnCaculateBoneCenterClicked();
	// �ɹǴ���׼
	bool OnCaculateFemurLandmarkClicked();

	// �ֹǴ���׼
	bool OnGetTibiaLandmarkClicked();
	bool OnCaculateTibiaLandmarlClicked();

	// �ֹǾ���׼
	bool OnGetTibieICPClicked();
	bool OnCaculateTibiaICPClicked();

	void UpdateHTOProbe();
	void UpdateHTOSaw();
	void UpdateHTODrill();

	void OnInitHTOTibiaRegisClicked();
	void OnInitHTOFemurRegisClicked();
	void OnCollectHTOTibiaLandmarkClicked();
	void OnCollectHTOFemurLandmarkClicked();
	void OnCollectHTOTibiaICPClicked();

	bool OnCollectHTOTibiaRegisClicked();
	// =================================================================

	// ========================== ���е��� ==============================
	// ��ȡʵʱ��Ϣ
	void GenerateRealTimeBoneSurface();

	bool OnStartAxialGuideClicked();
	bool OnStartAxialCutClicked();
	bool OnStartStateAxialCutClicked();
	bool OnCheckStateCutClicked();

	bool OnStartSagGuideClicked();
	bool OnStartSagCutClicked();
	bool OnStartStateSagCutClicked();

	bool OnStartDrillGuideClicked();
	bool OnStartDrillHoleClicked();
	bool OnStartStateDrillHoleClicked();
	// =================================================================
	
	// ========================== ������֤ ==============================
	bool OnShowResClicked();
	bool OnUnshowResClicked();
	bool OnCaculateErrorClicked();
	// ================================================================

	// ========================= �����ƶ� ==============================
    // ������ת
	void Rotate(const double center[3], double direction[3], double counterclockwiseDegree, mitk::BaseData* data);
	// ����ƽ��
	void Translate(double direction[3], double length, mitk::BaseData* data);
	// �����һ���˶�
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

	// ����ģ���˶�
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
  // ע������������
  void CreatQT_BasicSet();

  // 1. ��ǰ�滮
  void CreateQT_PreoperativePlan();

  // 2. ����ע��
  void CreateQT_MidPlan();

  // 3. ���е���
  void CreateQT_MidOperation();

  // 4. ��������
  void CreateQT_PostoperativeValid();

  // �����ƶ�
  void CreateQT_MoveData();

  // ����ҳ
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
  // ���������㼯��һ�����ڴ洢�Ѳɼ��ĵ㣬һ�����ڴ洢���ɼ��ĵ�
  mitk::DataNode::Pointer collectedPointsNode = mitk::DataNode::New();
  mitk::PointSet::Pointer collectedPoints = mitk::PointSet::New();
  mitk::Point3D currentpoint;
  bool firstnode = true;
  // set model param
  // ����ģ�Ͳ���
  void SetModelColor(mitk::DataNode::Pointer node, float r, float g, float b);
  void SetNodeSize(mitk::DataNode::Pointer node, float size);
  void SetModelOpacity(mitk::DataNode::Pointer node, float opacity);
  // ��Ϲɹ�ͷ��
  std::pair<Eigen::Vector3d, double> FitSphere(const std::vector<Eigen::Vector3d>& points);
  // �����ѡ������  
  QmitkDataStorageTreeModel* m_NodetreeModel{ nullptr };
  mitk::BaseData* m_baseDataToMove{ nullptr };
  mitk::DataNode* m_currentSelectedNode{ nullptr };
  std::string current = "";
  virtual void OnSelectionChanged(berry::IWorkbenchPart::Pointer source,
	  const QList<mitk::DataNode::Pointer>& nodes) override;


  

  // HTO Surgical Simulate
  // Get Center
  std::vector<Eigen::Vector3d> m_FemurPositions; // �洢�ζ������е�λ��
  Eigen::Vector3d m_HipCenter; // ��Ϲɹ�ͷ���ĵ�
  QTimer* m_FemurDataCollectionTimer = { nullptr }; //ʵʱ�ɼ���Ϣ
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
  std::vector<Eigen::Vector3d> m_ProbePositions; // �洢�ζ������е�λ��
  std::vector<Eigen::Matrix4d> m_Transformations; // �洢F_camera��F_probe��ת������
  Eigen::Vector3d m_ProbeTip;
  Eigen::Vector3d m_Center;
  int groupSize = 50;
  QTimer* m_ProbeDataCollectionTimer = {nullptr};
  // �������λ��
  bool CalculateProbeTip();
  void RegisterProbePosition();
  // ����̽��궨���
  double CalculateCalibrationError();
  // ���ӻ��ɼ��Ľڵ�
  bool OnVisualizeCollectedPoints();
  
  

  // ========================= ��ǰ�滮 ==============================
  bool CreateOneCutPlane();
  bool CutTibiaWithOnePlane();
  bool CutTibiaWithTwoPlanes();
  bool GetPlaneProperty(vtkSmartPointer<vtkPolyData> plane, double normal[3], double center[3]);
  // �عǺ���
  bool CutPolyDataWithPlane(vtkSmartPointer<vtkPolyData> dataToCut,
	  vtkSmartPointer<vtkPolyData> largerSubPart,
	  vtkSmartPointer<vtkPolyData> smallerSubPart,
	  double planeOrigin[3], double planeNormal[3]);
  // �ع�ЧӦ����
  // ��ȡʵʱ�ع���
  bool GetIntersectionLine();
  void TraverseIntersectionLines(vtkSmartPointer<vtkPolyData> intersectionLine);
  void GetDistancefromTibia();
  
  //��Ź滮�����ֹǵĽ���
  mitk::PointSet::Pointer planeAndTibiaIntersectionPoint;
  mitk::Point3D minPoint;//�ع���ĩ���е�
  mitk::Point3D maxPoint;//�ع�����ڵ�
  
  //HTO
  //Cut tibia into two parts with a plane 
  double distance1;//�ع�����ھ��ڲ�ƽ̨
  double distance2;//�ع���ĩ�˾����ƽ̨
  double distance3;//��������ҳ
  double distance4;//�����¡
  double depth;//�ع���������
  double Line_length;//���ߵĳ���
  double angleInDegrees;//�ſ��Ƕ�

  // ����ע��
  int judgModel_flag = 1;
  mitk::PointSet::Pointer mitkPointSet1 = mitk::PointSet::New();
 

  mitk::DataNode::Pointer m_DataNode; // Member variable to store the DataNode
  bool OBB = false;

  // ��֫���߹滮
  // ����ռ�ȼ���͸���
  void updateProportation();
  void updateProportation02();
  // ���������߶εĽ���
  bool LineLineIntersection(Eigen::Vector2d& intersection, Eigen::Vector2d p1, Eigen::Vector2d p2, Eigen::Vector2d q1, Eigen::Vector2d q2);
  // ����߶�
  void CaculateStrechHeigh();
  // �ְ�ĳ�ʼλ��
  mitk::Point3D m_steelPosition;
  // ��֫Զ�˳�ʼλ��
  mitk::Point3D m_distalTibiaPosition;
  // ================================================================

  // ========================= ����ע�� ==============================
  mitk::NavigationTool::Pointer m_femurRFNode;
  mitk::NavigationTool::Pointer m_tibiaRFNode;

  mitk::NavigationTool::Pointer m_saw;
  mitk::NavigationTool::Pointer m_drill;

  lancet::NavigationObject::Pointer saw_image;
  lancet::NavigationObject::Pointer drill_image;

  lancet::NavigationObject::Pointer femur_image;
  lancet::NavigationObject::Pointer tibia_image;
  // =================================================================

  // -----------------------���е���-----------------------------
  
  void trackingObjectPos();
  
  
  void CalculateRealTimeCutAngle();

  // �عǵ��� timer
  QTimer* m_timer_saw{ nullptr };
  QTimer* m_timer_strech{ nullptr };


  // �洢ʵʱ�عǵ�
  mitk::PointSet::Pointer mitkPointSetRealTime = mitk::PointSet::New();
  // =================================================================
  
  // ���ƽ�� 3d, ����ƽ�����ĺͷ�����
  std::pair<Eigen::Vector3d, Eigen::Vector3d> FitPlane3D(const std::vector<Eigen::Vector3d>& points);
  // ���Բ 2d, ����Բ��
  std::pair<Eigen::Vector3d, double> FitCircle3D(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& p3);
  // ���������� 
  // Eigen::Vector3d HTONDI::CalculateCircumcenter(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& p3);
  bool GetPlaneNormal(vtkSmartPointer<vtkPolyData> plane, double normal[3], double center[3]);
  
  // ��ȡ��������㷨����
  Eigen::Vector3d HTONDI::ExtractNormalFromPlane(const std::string& planeName);
  
  // ����ƫ��н�
  int m_cutType = -1;
  void CaculateCutPlaneMiss01();
  void CaculateCutPlaneMiss02();

  // ����ƽ���ϵ�����λ��
  std::vector<mitk::Point3D> CalculateRelativePoints(const std::vector<mitk::Point3D>& points, const Eigen::Vector3d& normalVector, const Eigen::Vector3d& origin);
  std::vector<mitk::Point3D> m_PointsOnSaw;
  std::vector<mitk::Point3D> CalculateActualPoints(const std::vector<mitk::Point3D>& relativePoints, const Eigen::Vector3d& normalVector, const Eigen::Vector3d& origin);

  // =================================================================
  // ����-�ɹǱ�ǵ��ڹɹ�RF�µ�λ��
  Eigen::Vector4d m_FemurCheckPointOnFemurRF;
  Eigen::Vector4d m_TibiaCheckPointOnTibiaRF;
  // ��ǰ-�ɹǱ�ǵ���Camera�µ�λ��
  Eigen::Vector4d m_CurrentFemurCheckPointOnCamera;
  Eigen::Vector4d m_CurrentTibiaCheckPointOnCamera;
  // ��ǰ-�ɹǱ�ǵ���Image�µ�λ��
  Eigen::Vector4d m_CurrentFemurCheckPointOnImage;
  Eigen::Vector4d m_CurrentTibiaCheckPointOnImage;


  // ��ǰ-̽���˵���Camera�µ�λ��
  Eigen::Vector4d m_CurrentTipOnCamera;
  // ��ǰ-̽���˵���Image�µ�λ��
  Eigen::Vector4d m_CurrentTipOnImage;


  // ������е�궨����RF�µ�λ��
  std::vector<Eigen::Vector4d> m_SawPointsOnSawRF;
  std::vector<Eigen::Vector4d> m_DrillPointsOnDrillRF;
  //// ��ǰ��е�궨����Camera�µ�λ��
  //Eigen::Matrix4d m_CurrentSawPointsOnCamera;
  //Eigen::Matrix4d m_CurrentDrillPointsOnCamera;
  //// ��ǰ��е�궨����Image�µ�λ��
  //Eigen::Matrix4d m_CurrentSawPointsOnImage;
  //Eigen::Matrix4d m_CurrentDrillPointsOnImage;

  // ��ǰ��е�궨����Camera�µ�λ��
  std::vector<Eigen::Vector4d> m_CurrentSawPointsOnCamera;
  std::vector<Eigen::Vector4d> m_CurrentDrillPointsOnCamera;
  // ��ǰ��е�궨����Image�µ�λ��
  std::vector<Eigen::Vector4d> m_CurrentSawPointsOnImage;
  std::vector<Eigen::Vector4d> m_CurrentDrillPointsOnImage;


  // ���� ��׼���� 4*4 
  mitk::AffineTransform3D::Pointer m_MetrixFemurRFToImage;
  mitk::AffineTransform3D::Pointer m_MetrixTibiaRFToImage;
  mitk::AffineTransform3D::Pointer m_MetrixCameraToImage;

  // ���� ��׼���� 4*4 
  Eigen::Matrix4d m_Metrix4dFemurRFToImage;
  Eigen::Matrix4d m_Metrix4dTibiaRFToImage;
  Eigen::Matrix4d m_Metrix4dCameraToImage;

  // ����ھ� [������������] ֮�����������, 1-2 1-3 2-3
  double m_SawDistance[3] = {25.8118, 53.1507, 28.6400};


  // ��׼����
  vtkNew<vtkMatrix4x4> m_ObjectRfToImageMatrix_hto;

  // �������߸��¼�ʱ��
  QTimer* m_HTOPrboeUpdateTimer{ nullptr };
  QTimer* m_HTOSawUpdateTimer{ nullptr };
  QTimer* m_HTODrillUpdateTimer{ nullptr };

  // HTO registration optimization
  // �ֹǽ������� �ֹǽ����ڲ�� �ֹ�Զ�����׵� �ֹ�Զ�����׵�
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
  Ui::HTONDIControls m_Controls;
};

#endif // HTONDI_h
