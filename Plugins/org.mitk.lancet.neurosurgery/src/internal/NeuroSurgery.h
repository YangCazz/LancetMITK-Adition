/*============================================================================
Robot-assisted Surgical Navigation Extension based on MITK
Copyright (c) Hangzhou LancetRobotics,CHINA
All rights reserved.
============================================================================*/

#ifndef NeuroSurgery_h
#define NeuroSurgery_h

#include <berryISelectionListener.h>
#include <QmitkAbstractView.h>
#include "ui_NeuroSurgeryControls.h"

#include "QmitkSingleNodeSelectionWidget.h"
#include "mitkTrackingDeviceSource.h"

// itk
#include <itkImage.h>

// mitk
#include <mitkImage.h>

class NeuroSurgery : public QmitkAbstractView
{
	Q_OBJECT

public:
	static const std::string VIEW_ID;

public slots:
	// Basic func
	void OnCheckDataClicked();
	void OnCheckPETMaskClicked();
	void OnCheckPETColorfyClicked();
	void OnRegistrateImageClicked();

protected:
	virtual void CreateQtPartControl(QWidget* parent) override;
	virtual void SetFocus() override;

	// Parm Init
	void ParamsInit();

	// UI Funcs Connection
	void CreatQT_Basic();
	void CreateQT_ImageProcess();
	void CreateQT_PreoperativePlan();
	void CreateQT_IntraoperativePlan();
	void CreateQT_PostoperativeVerify();
	void CreateQT_AccVerifyy();

	// Basic func
	void InitSurfaceSelector(QmitkSingleNodeSelectionWidget* widget);
	void InitPointSetSelector(QmitkSingleNodeSelectionWidget* widget);

	// Image processing
	void RegisterPETToMRI(mitk::Image::Pointer petImage, mitk::Image::Pointer mriImage);

	Ui::NeuroSurgeryControls m_Controls;
};

#endif // NeuroSurgery_h