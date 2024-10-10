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
#include "NS_Basic.h" // °üº¬ NS_Basic.h

class NeuroSurgery : public QmitkAbstractView
{
	Q_OBJECT

public:
	static const std::string VIEW_ID;

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
	void DataCheck();

	Ui::NeuroSurgeryControls m_Controls;
};

#endif // NeuroSurgery_h