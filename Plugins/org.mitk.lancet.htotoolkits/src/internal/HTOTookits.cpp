/*============================================================================
Robot-assisted Surgical Navigation Extension based on MITK
Copyright (c) Hangzhou LancetRobotics,CHINA
All rights reserved.
============================================================================*/

// Blueberry
#include <berryISelectionService.h>
#include <berryIWorkbenchWindow.h>

// Qmitk
#include "HTOTookits.h"

// Qt
#include <QMessageBox>
#include <QWidget>
#include <QThread>
#include <QTimer>
#include <QFileDialog>

// mitk image
#include <mitkImage.h>
#include <mitkAffineTransform3D.h>
#include <mitkMatrixConvert.h>

//igt
#include <lancetVegaTrackingDevice.h>

#include <lancetApplyDeviceRegistratioinFilter.h>
#include <mitkNavigationDataToPointSetFilter.h>
#include <lancetPathPoint.h>
#include <vtkQuaternion.h>

#include "lancetTrackingDeviceSourceConfigurator.h"
#include "mitkNavigationToolStorageDeserializer.h"
#include <QtWidgets\qfiledialog.h>

#include "mitkIGTIOException.h"
#include "mitkNavigationToolStorageSerializer.h"
#include "QmitkIGTCommonHelper.h"
#include "lancetTreeCoords.h"

const std::string HTOTookits::VIEW_ID = "org.mitk.views.htotookits";

void HTOTookits::SetFocus()
{
  
}

void HTOTookits::CreateQtPartControl(QWidget *parent)
{
  // create GUI widgets from the Qt Designer's .ui file
  m_Controls.setupUi(parent);

  // PART 01 ==== Device Connection
  
  //------------------- funcs list ----------------
// PART 01 ===> Device connection
// Camera: Aimooe, NDI, Ariemedi(瑞瞳)
  connect(m_Controls.connectAimooe_pushButton, &QPushButton::clicked, this, &HTOTookits::OnConnectAimooeClicked);
  connect(m_Controls.connectNDI_pushButton, &QPushButton::clicked, this, &HTOTookits::OnConnectNDIClicked);
  connect(m_Controls.connectArie_pushButton, &QPushButton::clicked, this, &HTOTookits::OnConnectArieClicked);
  
  // Robot: Kuka，大族
  connect(m_Controls.connectKuka_pushButton, &QPushButton::clicked, this, &HTOTookits::OnConnectKukaClicked);
  connect(m_Controls.connectDazu_pushButton, &QPushButton::clicked, this, &HTOTookits::OnConnectDazuClicked);

  // PART 02 ===> Config files selection
  connect(m_Controls.selectFile_pushButton, &QPushButton::clicked, this, &HTOTookits::OnSelectToolFileClicked);
  connect(m_Controls.selectFolder_pushButton, &QPushButton::clicked, this, &HTOTookits::OnSelectToolFolderClicked);
  connect(m_Controls.startCollect_pushButton, &QPushButton::clicked, this, &HTOTookits::OnStartCollectClicked);

  // PART 03 ===> Funcs on HTO
  connect(m_Controls.trackingDevice_pushButton, &QPushButton::clicked, this, &HTOTookits::OnCheckToolPosClicked);
  connect(m_Controls.stopTracking_pushButton, &QPushButton::clicked, this, &HTOTookits::OnStopDataClicked);
  connect(m_Controls.selectProbe_comboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &HTOTookits::OnProbeSelected);
  connect(m_Controls.selectRF_comboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &HTOTookits::OnRfSelected);
  // probe
  connect(m_Controls.trackingProbe_pushButton, &QPushButton::clicked, this, &HTOTookits::OnTrackProbeClicked);
  connect(m_Controls.collectProbe_pushButton, &QPushButton::clicked, this, &HTOTookits::OnCollectProbeClicked);
  connect(m_Controls.caculateProbe_pushButton, &QPushButton::clicked, this, &HTOTookits::OnCaculateProbeClicked);
  // tool RF 
  connect(m_Controls.trackingRF_pushButton, &QPushButton::clicked, this, &HTOTookits::OnTrackRFClicked);
  connect(m_Controls.caculateRFMatrix_pushButton, &QPushButton::clicked, this, &HTOTookits::OnCaculateRFClicked);
  connect(m_Controls.calibrateTool_pushButton, &QPushButton::clicked, this, &HTOTookits::OnCalibrateRFClicked);

  // PART 04 ===> Image Registration
  // load STL data
  InitSurfaceSelector(m_Controls.setSTL_mitkSlecectButton);
  InitPointSetSelector(m_Controls.setLandMark_mitkSlecectButton);
  InitPointSetSelector(m_Controls.setBoneNode_mitkSlecectButton);

  connect(m_Controls.checkImage_pushButton, &QPushButton::clicked, this, &HTOTookits::OnSetNavigateClicked);
  connect(m_Controls.showNode_pushButton, &QPushButton::clicked, this, &HTOTookits::OnShowNodeClicked);
  connect(m_Controls.reuseRegistrateMatrix_pushButton, &QPushButton::clicked, this, &HTOTookits::OnReuseMatrixClicked);
  connect(m_Controls.moveTest_pushButton, &QPushButton::clicked, this, &HTOTookits::OnMoveTestClicked);
  connect(m_Controls.moveBone_pushButton, &QPushButton::clicked, this, &HTOTookits::OnMoveBoneClicked);


  // collect registration point set
  connect(m_Controls.getFemurCenter_pushButton, &QPushButton::clicked, this, &HTOTookits::OnFemurCenterClicked);
  connect(m_Controls.getLandMark_pushButton, &QPushButton::clicked, this, &HTOTookits::OnGetLandMarkPointClicked);
  connect(m_Controls.getICP_pushButton, &QPushButton::clicked, this, &HTOTookits::OnGetICPPointClicked);
  connect(m_Controls.showCollectedPoint_pushButton, &QPushButton::clicked, this, &HTOTookits::OnShowCollectionClicked);
  connect(m_Controls.testRegistration_pushButton, &QPushButton::clicked, this, &HTOTookits::OnTestRegistrationClicked);
  connect(m_Controls.caculateRegistration_pushButton, &QPushButton::clicked, this, &HTOTookits::OnCaculateRegistrationClicked);
  connect(m_Controls.applyRegistration_pushButton, &QPushButton::clicked, this, &HTOTookits::OnApplyRegistration);
  
  // accuracy test
  InitPointSetSelector(m_Controls.selectCheckNode_mitkSlecectButton);
  connect(m_Controls.checkAccuracyPoints_pushButton, &QPushButton::clicked, this, &HTOTookits::OnCheckAccuracyPointsClicked);
  connect(m_Controls.checkNode2Node_pushButton, &QPushButton::clicked, this, &HTOTookits::OnCheckNode2NodeClicked);
  connect(m_Controls.checkNode2Surface_pushButton, &QPushButton::clicked, this, &HTOTookits::OnCheckNode2SurfaceClicked);

  // PART 05 ===> Robot Registration

  // PART 06 ===> HTO Simulation

}

//======================================funcs release===================================================
// PART 01 ==== Device Connection
// Aimooe connect and Init
bool HTOTookits::InitializeAndConnectDevice()
{	
	stopDataCollection = false;
	QString link_type;
	// if (isConnected) {
	// 	m_Controls.textBrowser_Action->append("Aimooe Already Connected.");
	// 	return true;
	// }
	m_Controls.textBrowser_Action->append("Action: Aimooe Connection.");

	//// 初始化设备
	aimHandle = NULL;
	E_ReturnValue rlt = Aim_API_Initial(aimHandle);
	if (rlt != AIMOOE_OK) {
		m_Controls.textBrowser_Action->append("ERROR: Aim_API_Initial Failed.");
		return false;
	}

	// 检查设备是否已经初始化
	//if (aimHandle == nullptr) {
	//	// 初始化设备
	//	E_ReturnValue rlt = Aim_API_Initial(aimHandle);
	//	if (rlt != AIMOOE_OK) {
	//		m_Controls.textBrowser_Action->append("ERROR: Aim_API_Initial Failed.");
	//		return false;
	//	}
	//}

	// 获取连接模式
	if (m_Controls.checkBox_USB->isChecked()) {
		EI = I_USB;
		link_type = "USB";
	}
	else if (m_Controls.checkBox_Ehernet->isChecked()) {
		EI = I_ETHERNET;
		link_type = "Ehernet";
		Aim_SetEthernetConnectIP(aimHandle, 192, 168, 31, 10);
	}
	else if (m_Controls.checkBox_Wifi->isChecked()) {
		EI = I_WIFI;
		link_type = "Wifi";
	}

	// 连接设备
	T_AIMPOS_DATAPARA mPosDataPara;
	rlt = Aim_ConnectDevice(aimHandle, EI, mPosDataPara);
	if (rlt != AIMOOE_OK) {
		m_Controls.textBrowser_Action->append("ERROR: Fail to Aim_ConnectDevice : " + link_type);
		return false;
	}

	// 设置数据获取类型
#if GET_DATA_FROM_HARDWARE
	rlt = Aim_SetAcquireData(aimHandle, EI, DT_NONE);
#else
	rlt = Aim_SetAcquireData(aimHandle, EI, DT_INFO);
#endif
	if (rlt != AIMOOE_OK) {
		m_Controls.textBrowser_Action->append("ERROR: Aim_SetAcquireData Failed.");
		return false;
	}

	isConnected = true;
	m_Controls.textBrowser_Action->append("Aimooe Connected.");
	// 打印设备系统信息
	updateConnectionStatus(link_type, mPosDataPara.devtype);
	// 每5秒检查一次设备连接状态
	// connectionCheckTimer->start(5000); 
	return true;
}


// connect check
E_ReturnValue HTOTookits::Aim_CheckDeviceConnection(AimHandle aimHandle)
{
	T_AimPosStatusInfo statusSt;
	E_ReturnValue rlt = Aim_GetStatusInfo(aimHandle, EI, statusSt);
	return rlt;
}
// slot for use
void HTOTookits::CheckDeviceConnection()
{
	if (!isConnected) {
		m_Controls.textBrowser_Action->append("ERROR: Aimooe Not Connected!");
		return;
	}

	// 检查设备连接状态
	E_ReturnValue rlt = Aim_CheckDeviceConnection(aimHandle);
	if (rlt != AIMOOE_OK) {
		isConnected = false;
		m_Controls.textBrowser_Action->append("Warning: Connection Break.");
	}
	else {
		m_Controls.textBrowser_Action->append("State: Aimooe Connection Well.");
	}
}




// Aimooe State Check for later use
bool HTOTookits::updateConnectionStatus(const QString& link_type, AIMPOS_TYPE devtype)
{
	QString text = "State: Aimooe Connected | MODE--" + link_type;
	// check Aimooe version
	switch (devtype)
	{
	case AIMPOS_TYPE::eAP_Standard:
		text += ", Standard version!";
		break;
	case AIMPOS_TYPE::eAP_Lite:
		text += ", Light version!";
		break;
	case AIMPOS_TYPE::eAP_Ultimate:
		text += ", Ultimate version!";
		break;
	case AIMPOS_TYPE::eAP_Basic:
		text += ", Basic version!";
		break;
	case AIMPOS_TYPE::eAP_Industry:
		text += ", Industrial version!";
		break;
	default:
		break;
	}
	// Action log
	m_Controls.textBrowser_Action->append(text);
	return true;
}

// Connect Devices
bool HTOTookits::OnConnectAimooeClicked()
{	
	// 初始化定时器
	//if (!connectionCheckTimer) {
	//	connectionCheckTimer = new QTimer(this);
	//	connect(connectionCheckTimer, &QTimer::timeout, this, &HTOTookits::CheckDeviceConnection);
	//}

	//if (InitializeAndConnectDevice()) {
	//	// 每5秒检查一次设备连接状态
	//	connectionCheckTimer->start(5000);
	//	return true;
	//}

	InitializeAndConnectDevice();
	return false;
}

// NDI connect and Init
bool HTOTookits::OnConnectNDIClicked()
{
	m_Controls.textBrowser_Action->append("Action: NDI Connection.");
	// 读取配置文件
	QString filename = QFileDialog::getOpenFileName(nullptr, tr("Open Tool Storage"), "/",
		tr("Tool Storage Files (*.IGTToolStorage)"));
	if (filename.isNull()) return false;

	// 反序列化工具存储
	std::string errorMessage = "";
	mitk::NavigationToolStorageDeserializer::Pointer myDeserializer = mitk::NavigationToolStorageDeserializer::New(
		GetDataStorage());
	m_VegaToolStorage = myDeserializer->Deserialize(filename.toStdString());
	m_VegaToolStorage->SetName(filename.toStdString());

	// 创建NDI Vega跟踪设备
	MITK_INFO << "Vega tracking";
	lancet::NDIVegaTrackingDevice::Pointer vegaTrackingDevice = lancet::NDIVegaTrackingDevice::New();

	// 使用工厂类创建跟踪设备源
	lancet::TrackingDeviceSourceConfiguratorLancet::Pointer vegaSourceFactory =
		lancet::TrackingDeviceSourceConfiguratorLancet::New(m_VegaToolStorage, vegaTrackingDevice);

	m_VegaSource = vegaSourceFactory->CreateTrackingDeviceSource(m_VegaVisualizer);
	m_VegaSource->SetToolMetaDataCollection(m_VegaToolStorage);

	// 连接并开始跟踪
	m_VegaSource->Connect();
	m_VegaSource->StartTracking();

	// 设置定时器更新可视化
	if (m_VegaVisualizeTimer == nullptr)
	{
		m_VegaVisualizeTimer = new QTimer(this);
	}
	connect(m_VegaVisualizeTimer, SIGNAL(timeout()), this, SLOT(OnVegaVisualizeTimer()));
	connect(m_VegaVisualizeTimer, SIGNAL(timeout()), this, SLOT(UpdateToolStatusWidget()));
	ShowToolStatus_Vega();
	m_VegaVisualizeTimer->start(100);

	auto geo = this->GetDataStorage()->ComputeBoundingGeometry3D(this->GetDataStorage()->GetAll());
	mitk::RenderingManager::GetInstance()->InitializeViews(geo);


	return true;
}

void HTOTookits::ShowToolStatus_Vega()
{
	//m_VegaNavigationData.clear();
	//for (std::size_t i = 0; i < m_VegaSource->GetNumberOfOutputs(); i++)
	//{
	//	m_VegaNavigationData.push_back(m_VegaSource->GetOutput(i));
	//}
	////initialize widget
	//m_Controls.m_StatusWidgetVegaToolToShow->RemoveStatusLabels();
	//m_Controls.m_StatusWidgetVegaToolToShow->SetShowPositions(true);
	//m_Controls.m_StatusWidgetVegaToolToShow->SetTextAlignment(Qt::AlignLeft);
	//m_Controls.m_StatusWidgetVegaToolToShow->SetNavigationDatas(&m_VegaNavigationData);
	//m_Controls.m_StatusWidgetVegaToolToShow->ShowStatusLabels();
}

bool HTOTookits::OnVegaVisualizeTimer()
{
	//Here we call the Update() method from the Visualization Filter. Internally the filter checks if
	//new NavigationData is available. If we have a new NavigationData the cone position and orientation
	//will be adapted.
	if (m_VegaVisualizer.IsNotNull())
	{
		m_VegaVisualizer->Update();
		// auto geo = this->GetDataStorage()->ComputeBoundingGeometry3D(this->GetDataStorage()->GetAll());
		// mitk::RenderingManager::GetInstance()->InitializeViews(geo);
		this->RequestRenderWindowUpdate();
	}
	return false;
}


//
bool HTOTookits::OnConnectArieClicked()
{
	m_Controls.textBrowser_Action->append("Action: Arie Connection.");
	return true;
}

bool HTOTookits::OnConnectKukaClicked()
{
	m_Controls.textBrowser_Action->append("Action: Kuka Connection.");
	return true;
}

bool HTOTookits::OnConnectDazuClicked()
{
	m_Controls.textBrowser_Action->append("Action: Dazu Connection.");
	return true;
}

// PART 02 ===> Config files selection
// file select and display
bool HTOTookits::SelectFile(QTextBrowser* textBrowser, QString type)
{
    // 读取文件名
    if (type == "file") {
        folder = false;
        ToolPath = QFileDialog::getOpenFileName(nullptr, tr("Open File"), "/",
            tr("Aimtool Files (*.aimtool);;Text Files (*.txt);;All Files (*)"));
    }
    else if (type == "folder") {
        folder = true;
        ToolPath = QFileDialog::getExistingDirectory(nullptr, tr("Open Directory"), "/",
            QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    }
    if (ToolPath.isNull()) return false;
    if (!ToolPath.isEmpty()) {
        // 将文件路径显示在文本框中
        textBrowser->setText(ToolPath);

        // 将路径保存到 char* 中，并进行路径处理
        QByteArray byteArray = ToolPath.toLocal8Bit();
        std::string toolFolderPathStr = byteArray.data();

        // 替换路径中的斜杠为反斜杠
        std::replace(toolFolderPathStr.begin(), toolFolderPathStr.end(), '/', '\\');

        // 将单斜杠转换为双斜杠
        std::string::size_type pos = 0;
        while ((pos = toolFolderPathStr.find('\\', pos)) != std::string::npos) {
            toolFolderPathStr.insert(pos, "\\");
            pos += 2;
        }

        // 在路径末尾添加两个反斜杠
        if (toolFolderPathStr.back() != '\\') {
            toolFolderPathStr += "\\\\";
        } else {
            toolFolderPathStr += "\\";
        }

        // 将处理后的路径保存到 std::vector<char> 中
		std::vector<char> toolFolderPathVec = std::vector<char>(toolFolderPathStr.begin(), toolFolderPathStr.end());
        toolFolderPathVec.push_back('\0'); // 确保字符串以空字符终止

        // 将 std::vector<char> 转换为 char* 类型
        toolFolderPath = toolFolderPathVec.data();
        std::cout << toolFolderPath << std::endl;
    }
    return true;
}

// select button
bool HTOTookits::OnSelectToolFileClicked()
{
	SelectFile(m_Controls.textBrowser_SelectTools, "file");
	return true;
}

bool HTOTookits::OnSelectToolFolderClicked()
{
	SelectFile(m_Controls.textBrowser_SelectTools, "folder");
	return true;
}

bool HTOTookits::OnStartCollectClicked()
{	
	m_Controls.textBrowser_Action->append("Action: File Load.");
	// state: Input folder path
	if (folder && !ToolPath.isEmpty()) {
		// 清理两个下拉框的信息
		m_Controls.selectProbe_comboBox->clear();
		m_Controls.selectRF_comboBox->clear();
		QDir dir(ToolPath);
		QStringList filters;
		filters << "*.aimtool" << "*.txt";
		dir.setNameFilters(filters);
		QFileInfoList fileList = dir.entryInfoList();
		FileNames.clear();
		m_Controls.selectProbe_comboBox->clear();
		for (const QFileInfo& fileInfo : fileList) {
			// get only file name with .xxx
			FileNames << fileInfo.completeBaseName(); 
			m_Controls.selectProbe_comboBox->addItem(fileInfo.fileName());
			m_Controls.selectRF_comboBox->addItem(fileInfo.fileName());
		}
	}
	// Input file path
	else if (!ToolPath.isEmpty()) {
		QFileInfo fileInfo(ToolPath);
		if (fileInfo.exists() && (fileInfo.suffix() == "aimtool" || fileInfo.suffix() == "txt")) {
			FileNames.clear();
			// get only file name with .xxx
			FileNames << fileInfo.completeBaseName();
			m_Controls.selectProbe_comboBox->addItem(fileInfo.fileName());
			m_Controls.selectRF_comboBox->addItem(fileInfo.fileName());
		}
	}
	QString text;
	for (const QString& fileName : FileNames) {
		text += " " + fileName;
		ToolNames.push_back(fileName.toStdString());
	}
	m_Controls.textBrowser_Action->append("The loaded tools: " + text);
	return true;
}

// PART 03 ===> Real Funcs on HTO
// timer connect
bool HTOTookits::TimerConnect(QTimer*& timer, const char* slotr)
{
	if (timer == nullptr)
	{	// create a new timer
		timer = new QTimer(this);
	}
	if (timer->isActive()) {
		m_Controls.textBrowser_Action->append("Timer is used!");
		timer->stop();
	}
	// disconnect all
	QObject::disconnect(timer, nullptr, this, nullptr);
	// start new one
	connect(timer, SIGNAL(timeout()), this, slotr);
	// Every 100ms the method OnTimer() is called. -> 10fps
	timer->start(100);
	return true;
}

// boot tracking
bool HTOTookits::OnCheckToolPosClicked()
{	
	stopDataCollection = false;
	current = "all";
	// 开始设备追踪
	m_Controls.textBrowser_Action->append("Action: Aimooe Tools Tracking.");
	// 首先检查连接状态
	if (!InitializeAndConnectDevice()) {
		return false;
	}
	// 绑定更新函数
	TimerConnect(Timer_Check, SLOT(UpdateCheckData()));
	return true;
}

void HTOTookits::UpdateCheckData()
{
	try {
		//if (!isConnected) {
		//	throw std::runtime_error("ERROR: Aimooe Not Connected!");
		//}
		m_Controls.textBrowser_Action->append("Acquire Data.......");
		// -------------------------------------------
		AimHandle aimHandle = NULL;
		E_Interface EI = I_ETHERNET;
		T_AIMPOS_DATAPARA mPosDataPara;

		E_ReturnValue rlt = Aim_API_Initial(aimHandle);
		// 连接设备
		if (EI == I_ETHERNET) {
			Aim_SetEthernetConnectIP(aimHandle, 192, 168, 31, 10);
		}
		rlt = Aim_ConnectDevice(aimHandle, EI, mPosDataPara);
		if (rlt != AIMOOE_OK) {
			std::cout << "connect error." << std::endl;
			return;
		}

		// 设置数据获取类型
#if GET_DATA_FROM_HARDWARE 
		Aim_SetAcquireData(aimHandle, EI, DT_NONE);
#else
		Aim_SetAcquireData(aimHandle, EI, DT_INFO);
#endif
		if (rlt != AIMOOE_OK) {
			std::cout << "set data error." << std::endl;
			return;
		}

		// -------------------------------------------
		// 设置工具文件路径
		//char* path = "D:\\Aimooe\\ToolBoxFiles\\Config\\AimTools\\";
		char* path = "C:\\Users\\zhouz\\Desktop\\toolkits\\0726\\";
		rlt = Aim_SetToolInfoFilePath(aimHandle, path);
		//E_ReturnValue rlt = Aim_SetToolInfoFilePath(aimHandle, toolFolderPath);
		if (rlt != AIMOOE_OK) {
			throw std::runtime_error("ERROR: Aim_SetToolInfoFilePath Failed.");
		}

		// 获取标记点和状态信息
		T_MarkerInfo markerSt;
		T_AimPosStatusInfo statusSt;
		rlt = Aim_GetMarkerAndStatusFromHardware(aimHandle, EI, markerSt, statusSt);
		if (rlt != AIMOOE_OK) {
			throw std::runtime_error("ERROR: Aim_GetMarkerAndStatusFromHardware Failed.");
		}
		// 获取工具信息
		// std::vector<std::string> toolidarr = { "PTM-02", "PTM-01" };
		T_AimToolDataResult* mtoolsrlt = new T_AimToolDataResult;
		mtoolsrlt->next = NULL;

		// 获取数据
		do {
#if GET_DATA_FROM_HARDWARE 
			rlt = Aim_GetMarkerAndStatusFromHardware(aimHandle, EI, markerSt, statusSt);
#else
			rlt = Aim_GetMarkerInfo(aimHandle, EI, markerSt);
#endif    
			if (rlt == AIMOOE_NOT_REFLASH)
				Sleep(10);
			if (stopDataCollection) {
				throw std::runtime_error("Handle Stop.");
			}
		} while (rlt == AIMOOE_NOT_REFLASH);

		if (rlt != AIMOOE_OK) {
			delete mtoolsrlt;
			throw std::runtime_error("ERROR: Aim_GetMarkerInfo Failed.");
		}
		// 加载前面从文件夹中取得的所有工具
		rlt = Aim_FindSpecificToolInfo(aimHandle, markerSt, ToolNames, mtoolsrlt, 3);
		T_AimToolDataResult* prlt = mtoolsrlt;
		QString toolsRealTime;
		if (rlt == AIMOOE_OK) {
			do {
				if (prlt->validflag) {

					QString output;
					output += QString("Tool Name: %1  MeanError: %2  RMS: %3\n")
						.arg(QString::fromStdString(prlt->toolname))
						.arg(prlt->MeanError)
						.arg(prlt->Rms);
					output += QString("Tool Origin: %1, %2, %3\n")
						.arg(prlt->OriginCoor[0])
						.arg(prlt->OriginCoor[1])
						.arg(prlt->OriginCoor[2]);
					output += QString("Tool Angle: %1, %2, %3\n")
						.arg(prlt->rotationvector[0] * 180 / 3.141592)
						.arg(prlt->rotationvector[1] * 180 / 3.141592)
						.arg(prlt->rotationvector[2] * 180 / 3.141592);
					output += "Marker:\n";
					for (int i = 0; i < prlt->toolptidx.size(); i++) {
						int idx = prlt->toolptidx[i];
						if (idx < 0)
							output += "0 0 0\n";
						else
							output += QString("%1 %2 %3\n")
							.arg(markerSt.MarkerCoordinate[idx * 3 + 0])
							.arg(markerSt.MarkerCoordinate[idx * 3 + 1])
							.arg(markerSt.MarkerCoordinate[idx * 3 + 2]);
					}

					// 保存工具信息
					toolDataResults[prlt->toolname].push_back(*prlt);
					if (toolDataResults[prlt->toolname].size() > MAX_SAMPLES) {
						toolDataResults[prlt->toolname].erase(toolDataResults[prlt->toolname].begin());
					}
					if ((startCollect == true) && (currentTool.size() < MAX_SAMPLES)) {
						currentTool.push_back(*prlt);
					}
					else if ((startCollect == true) && (currentTool.size() == MAX_SAMPLES)) {
						startCollect = false;
						m_Controls.textBrowser_Action->append("State: Collect Current Tool Down.");
					}

					// 将输出信息传递到 textBrowser_PosCheck
					// 依据当前匹配工具名称
					if (current == "all") {
						QMetaObject::invokeMethod(m_Controls.showState_textBrowser, "append", Q_ARG(QString, output));
					}
					else if (current != "all") {
						if (prlt->toolname == current) {
							QMetaObject::invokeMethod(m_Controls.showState_textBrowser, "setText", Q_ARG(QString, output));
						}
					}

					// 添加工具名称到列表
					toolsRealTime += QString::fromStdString(prlt->toolname) + "\n";
				}
				T_AimToolDataResult* pnext = prlt->next;
				delete prlt;
				prlt = pnext;
			} while (prlt != NULL);
			//m_Controls.textBrowser_Action->append("查找结束！");
		}
		else {
			delete prlt;
			m_Controls.textBrowser_Action->append("No tools in visiual.");
		}
		// 更新实时工具名称
		m_Controls.showDevice_textBrowser->setText(toolsRealTime);

		// 关闭设备
		rlt = Aim_API_Close(aimHandle);
		if (rlt != AIMOOE_OK) {
			throw std::runtime_error("ERROR: Aim_API_Close Failed.");
		}
	}
	catch (const std::exception & e) {
		// 捕获异常并停止计时器
		if (Timer_Check->isActive()) {
			Timer_Check->stop();
		}
		m_Controls.textBrowser_Action->append("Error: " + QString::fromStdString(e.what()));
	}
}
// stop tracking
void HTOTookits::OnStopDataClicked()
{
	m_Controls.textBrowser_Action->append("Action: Stop Data Collection.");
	stopDataCollection = true;
}

// tool select  
bool HTOTookits::ToolSelect(int index, QComboBox* comboBox, std::string tool, QString type)
{
	if (index >= 0) {
		QString selectedTool = comboBox->itemText(index);
		QByteArray byteArray = selectedTool.toLocal8Bit();
		const char* toolName = byteArray.data();

		// 将选中的工具名称添加到字符数组中
		//tools.push_back(toolName);
		tool = toolName;

		// 输出到控制台或其他调试窗口
		m_Controls.textBrowser_Action->append("Selected "+ type + " : " + selectedTool);
	}
	return true;
}

// probe select
bool HTOTookits::OnProbeSelected(int index)
{
	m_Controls.textBrowser_Action->append("Action: Probe Select.");
	ToolSelect(index, m_Controls.selectProbe_comboBox, toolProbe, "Probe");
	return true;
}

// RF select
bool HTOTookits::OnRfSelected(int index)
{
	m_Controls.textBrowser_Action->append("Action: RF Select.");
	ToolSelect(index, m_Controls.selectRF_comboBox, toolRf, "RF");
	return true;
}

// probe tracking
bool HTOTookits::OnTrackProbeClicked()
{
	current = toolProbe;
	cout << current << endl;
	m_Controls.textBrowser_Action->append("Action: Probe Tracking.");
	return true;
}

// RF tracking
bool HTOTookits::OnTrackRFClicked()
{
	current = toolRf;
	cout << current << endl;
	m_Controls.textBrowser_Action->append("Action: RF Tracking.");
	return true;
}

// Probe collect & normalize
// Collect probe dynamic pos information
bool HTOTookits::OnCollectProbeClicked()
{	
	// set flag => Action inf => refresh data
	startCollect = true;
	m_Controls.textBrowser_Action->append("Action: Probe normalize - Data collect.");
	currentTool = std::vector<T_AimToolDataResult>();
	return true;
}

// Normalize
// check if data is enough => curve spheres => get node pos => get avg pos
// use API case
bool HTOTookits::OnCaculateProbeClicked()
{
    m_Controls.textBrowser_Action->append("Action: Probe normalize - Node get.");

    try {
		if (!isConnected) {
			throw std::runtime_error("ERROR: Aimooe Not Connected");
		}
		// ------------------------------------------------
		AimHandle aimHandle = NULL;
		E_Interface EI = I_ETHERNET;
		T_AIMPOS_DATAPARA mPosDataPara;

		E_ReturnValue rlt = Aim_API_Initial(aimHandle);
		// 连接设备
		if (EI == I_ETHERNET) {
			Aim_SetEthernetConnectIP(aimHandle, 192, 168, 31, 10);
		}
		rlt = Aim_ConnectDevice(aimHandle, EI, mPosDataPara);
		if (rlt != AIMOOE_OK) {
			std::cout << "connect error." << std::endl;
			return false;
		}

		// 设置数据获取类型
#if GET_DATA_FROM_HARDWARE 
		Aim_SetAcquireData(aimHandle, EI, DT_NONE);
#else
		Aim_SetAcquireData(aimHandle, EI, DT_INFO);
#endif
		if (rlt != AIMOOE_OK) {
			std::cout << "set data error." << std::endl;
			return false;
		}
		// ------------------------------------------------
		char* path = "C:\\Users\\zhouz\\Desktop\\toolkits\\0726\\";
		rlt = Aim_SetToolInfoFilePath(aimHandle, path);
		//E_ReturnValue rlt = Aim_SetToolInfoFilePath(aimHandle, toolFolderPath);
		if (rlt != AIMOOE_OK) {
			throw std::runtime_error("ERROR: Aim_SetToolInfoFilePath Failed.");
		}

		Aim_GetToolInfoFilePath(aimHandle);

		// 初始化工具针尖标定
		//E_ReturnValue rlt = Aim_InitToolTipPivotWithToolId(aimHandle, toolProbe.c_str());
		//rlt = Aim_InitToolTipPivotWithToolId(aimHandle, toolProbe.c_str());
		rlt = Aim_InitToolTipPivotWithToolId(aimHandle, "PTM-01");
		if (rlt != AIMOOE_OK) {
			m_Controls.textBrowser_Action->append("ERROR: Aim_InitToolTipPivotWithToolId Failed");
			return false;
		}

		T_ToolTipPivotInfo toolTipInfo;
		toolTipInfo.isPivotFinished = false;
		int cntTimes = 0;

        // 循环获取标记点和状态信息，并进行工具针尖标定
        do {
            Sleep(50);
            cntTimes++;

			T_MarkerInfo markerSt;
			T_AimPosStatusInfo statusSt;

#if GET_DATA_FROM_HARDWARE 
            rlt = Aim_GetMarkerAndStatusFromHardware(aimHandle, EI, markerSt, statusSt);
#else
            rlt = Aim_GetMarkerInfo(aimHandle, EI, markerSt);
#endif    
            if (rlt != AIMOOE_OK) {
                continue;
            }
			// process tip normolize
            rlt = Aim_ProceedToolTipPivot(aimHandle, markerSt, toolTipInfo);
			// when do, pivotRate ++, then update error
            m_Controls.textBrowser_Action->append("Process: " + QString::number((int)(toolTipInfo.pivotRate * 100)) + "%");

        } while (toolTipInfo.isPivotFinished == false && cntTimes < 1000);

        if (toolTipInfo.isPivotFinished && toolTipInfo.pivotMeanError < 1) 
		{
            rlt = Aim_SaveToolTipCalibration(aimHandle);
            if (rlt == AIMOOE_OK) {
                m_Controls.textBrowser_Action->append("Probe init down, error(mm): " + QString::number(toolTipInfo.pivotMeanError));
            } else {
                m_Controls.textBrowser_Action->append("ERROR: Aim_SaveToolTipCalibration Failed.");
            }
        } else if (toolTipInfo.isPivotFinished && toolTipInfo.pivotMeanError > 1) {
            m_Controls.textBrowser_Action->append("Probe init failed, error > 1 mm");
        }

        return true;
    } catch (const std::exception & e) {
        m_Controls.textBrowser_Action->append("Error: " + QString::fromStdString(e.what()));
        return false;
    }
}


// RF calibrate
bool HTOTookits::OnCaculateRFClicked()
{
	m_Controls.textBrowser_Action->append("Action: Register Experimental Tool.");
	QString output = "Calibration Points for " + QString::fromStdString(toolRf) + " in Reference Frame:\n";
	m_Controls.calibrateTool_textBrowser->append(output);

	stopDataCollection = false;

	try {
		if (!isConnected) {
			throw std::runtime_error("ERROR: Aimooe Not Connected");
		}

		// 初始化设备
		AimHandle aimHandle = NULL;
		E_Interface EI = I_ETHERNET;
		T_AIMPOS_DATAPARA mPosDataPara;

		E_ReturnValue rlt = Aim_API_Initial(aimHandle);
		if (rlt != AIMOOE_OK) {
			throw std::runtime_error("ERROR: Aim_API_Initial Failed.");
		}

		// 连接设备
		if (EI == I_ETHERNET) {
			Aim_SetEthernetConnectIP(aimHandle, 192, 168, 31, 10);
		}
		rlt = Aim_ConnectDevice(aimHandle, EI, mPosDataPara);
		if (rlt != AIMOOE_OK) {
			throw std::runtime_error("ERROR: Aim_ConnectDevice Failed.");
		}

		// 设置数据获取类型
#if GET_DATA_FROM_HARDWARE 
		rlt = Aim_SetAcquireData(aimHandle, EI, DT_NONE);
#else
		rlt = Aim_SetAcquireData(aimHandle, EI, DT_INFO);
#endif
		if (rlt != AIMOOE_OK) {
			throw std::runtime_error("ERROR: Aim_SetAcquireData Failed.");
		}

		// 设置工具文件路径
		char* path = "C:\\Users\\zhouz\\Desktop\\toolkits\\0726\\";
		rlt = Aim_SetToolInfoFilePath(aimHandle, path);
		if (rlt != AIMOOE_OK) {
			throw std::runtime_error("ERROR: Aim_SetToolInfoFilePath Failed.");
		}

		// 获取校准点的数量
		int numCalibrationPoints = 5; // 假设有5个校准点
		std::vector<Eigen::Vector3d> calibrationPoints;
		std::vector<Eigen::Vector3d> rfPositions;
		calibrationPoints.reserve(numCalibrationPoints);
		rfPositions.reserve(numCalibrationPoints);

		for (int i = 0; i < numCalibrationPoints; ++i) {
			m_Controls.textBrowser_Action->append("Place the probe on calibration point " + QString::number(i + 1) + " and press OK.");

			// 等待用户确认
			QMessageBox::information(nullptr, "Calibration", "Place the probe on calibration point " + QString::number(i + 1) + " and press OK.");

			// 获取标记点和状态信息
			T_MarkerInfo markerSt;
			T_AimPosStatusInfo statusSt;
			rlt = Aim_GetMarkerAndStatusFromHardware(aimHandle, EI, markerSt, statusSt);
			if (rlt != AIMOOE_OK) {
				throw std::runtime_error("ERROR: Aim_GetMarkerAndStatusFromHardware Failed.");
			}
			// 获取工具信息
			// std::vector<std::string> toolidarr = { "PTM-02", "PTM-01" };
			T_AimToolDataResult* mtoolsrlt = new T_AimToolDataResult;
			mtoolsrlt->next = NULL;

			// 获取数据
			do {
#if GET_DATA_FROM_HARDWARE 
				rlt = Aim_GetMarkerAndStatusFromHardware(aimHandle, EI, markerSt, statusSt);
#else
				rlt = Aim_GetMarkerInfo(aimHandle, EI, markerSt);
#endif    
				if (rlt == AIMOOE_NOT_REFLASH)
					Sleep(10);
				if (stopDataCollection) {
					throw std::runtime_error("Handle Stop.");
				}
			} while (rlt == AIMOOE_NOT_REFLASH);

			if (rlt != AIMOOE_OK) {
				delete mtoolsrlt;
				throw std::runtime_error("ERROR: Aim_GetMarkerInfo Failed.");
			}
			// 加载前面从文件夹中取得的所有工具
			rlt = Aim_FindSpecificToolInfo(aimHandle, markerSt, ToolNames, mtoolsrlt, 3);
			T_AimToolDataResult* prlt = mtoolsrlt;
			// 遍历工具信息，找到探针和参考阵列
			T_AimToolDataResult* probeToolData = nullptr;
			T_AimToolDataResult* rfToolData = nullptr;
			while (prlt != nullptr) {
				cout << prlt->toolname << endl;

				if (prlt->validflag) {
					cout << "find one" << endl;
					if (strcmp(prlt->toolname, "PTM-ProbeRF") == 0) {
						probeToolData = prlt;
						cout << "Find PTM-ProbeRF" << endl;
					}
					else if (strcmp(prlt->toolname, "PTM-BoneRF") == 0) {
						rfToolData = prlt;
						cout << "find PTM-BoneRF" << endl;
					}
				}
				prlt = prlt->next;
			}

			if (probeToolData == nullptr || rfToolData == nullptr) {
				delete prlt;
				throw std::runtime_error("Required tools not found.");
			}

			// 获取探针和参考阵列的位置
			Eigen::Vector3d probeTip(probeToolData->tooltip[0], probeToolData->tooltip[1], probeToolData->tooltip[2]);
			Eigen::Vector3d rfPosition(rfToolData->tooltip[0], rfToolData->tooltip[1], rfToolData->tooltip[2]);

			// 将数据添加到列表
			calibrationPoints.push_back(probeTip);
			rfPositions.push_back(rfPosition);

			// 打印当前标定点的坐标
			QString pointStr = QString("Calibration Point %1: Probe Tip (%2, %3, %4), RF Position (%5, %6, %7)")
				.arg(i + 1)
				.arg(probeTip.x())
				.arg(probeTip.y())
				.arg(probeTip.z())
				.arg(rfPosition.x())
				.arg(rfPosition.y())
				.arg(rfPosition.z());
			m_Controls.calibrateTool_textBrowser->append(pointStr);

			delete mtoolsrlt;
		}

		// 计算探针针尖代表的标定点相对于参考阵列的相对位置
		std::vector<Eigen::Vector3d> relativePositions;
		for (int i = 0; i < numCalibrationPoints; ++i) {
			Eigen::Vector3d relativePosition = calibrationPoints[i] - rfPositions[i];
			relativePositions.push_back(relativePosition);

			// 打印相对位置
			QString relativeStr = QString("Relative Position %1: (%2, %3, %4)")
				.arg(i + 1)
				.arg(relativePosition.x())
				.arg(relativePosition.y())
				.arg(relativePosition.z());
			m_Controls.calibrateTool_textBrowser->append(relativeStr);
		}

		// 更新全局变量
		g_calibrationPointsMap[toolRf] = relativePositions;

		return true;
	}
	catch (const std::exception & e) {
		m_Controls.textBrowser_Action->append("Error: " + QString::fromStdString(e.what()));
		return false;
	}
}

// 矩阵变换
Eigen::Vector3d HTOTookits::TransformToReferenceFrame(const Eigen::Vector3d& point, T_AimToolDataResult* rfToolData)
{
	// 将点从相机坐标系转化到工具参考阵列坐标系
	// 这里需要实现具体的转换逻辑
	Eigen::Matrix3d rotationMatrix;
	rotationMatrix << rfToolData->Rto[0][0], rfToolData->Rto[0][1], rfToolData->Rto[0][2],
		rfToolData->Rto[1][0], rfToolData->Rto[1][1], rfToolData->Rto[1][2],
		rfToolData->Rto[2][0], rfToolData->Rto[2][1], rfToolData->Rto[2][2];

	Eigen::Vector3d translationVector(rfToolData->Tto[0], rfToolData->Tto[1], rfToolData->Tto[2]);

	// 计算逆矩阵
	Eigen::Matrix3d invRotationMatrix = rotationMatrix.transpose();
	Eigen::Vector3d invTranslationVector = -invRotationMatrix * translationVector;

	// 转换点的坐标
	Eigen::Vector3d transformedPoint = invRotationMatrix * point + invTranslationVector;
	return transformedPoint;
}


bool HTOTookits::OnCalibrateRFClicked()
{
	return true;
}