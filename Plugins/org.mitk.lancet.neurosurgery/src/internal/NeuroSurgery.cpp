/*============================================================================
Robot-assisted Surgical Navigation Extension based on MITK
Copyright (c) Hangzhou LancetRobotics,CHINA
All rights reserved.
============================================================================*/

#include "NeuroSurgery.h"
#include <berryISelectionService.h>
#include <berryIWorkbenchWindow.h>
#include <QMessageBox>
#include <mitkImage.h>

const std::string NeuroSurgery::VIEW_ID = "org.mitk.views.neurosurgery";

void NeuroSurgery::SetFocus()
{
    // do nothing, but you can't delete it
}

void NeuroSurgery::CreateQtPartControl(QWidget* parent)
{
    m_Controls.setupUi(parent);

    // Param Init
    ParamsInit();

    // make UI plane for different func plane.
    CreatQT_Basic();
    CreateQT_ImageProcess();
    CreateQT_PreoperativePlan();
    CreateQT_IntraoperativePlan();
    CreateQT_PostoperativeVerify();
    CreateQT_AccVerifyy();

}

void NeuroSurgery::CreatQT_Basic()
{
    /* Basic Funs
    * 1.
    * 2.
    */
    // Initialize selectors
    InitSurfaceSelector(m_Controls.mitk_T1_Pic, this);
    //InitPointSetSelector(m_Controls.someWidget, this);
}

void NeuroSurgery::CreateQT_PreoperativePlan()
{
    /* Funcs for Preoperative planning
    * 1.
    * 2.
    */
}

void NeuroSurgery::CreateQT_ImageProcess()
{
    /* Funcs for Multimodal Brain Image Processing
    * 1.
    * 2.
    */
}

void NeuroSurgery::CreateQT_IntraoperativePlan()
{
    /* Funcs for Intraoperative planning
    * 1.
    * 2.
    */
}

void NeuroSurgery::CreateQT_PostoperativeVerify()
{
    /* Funcs for Intraoperative planning
    * 1.
    * 2.
    */
}

void NeuroSurgery::CreateQT_AccVerifyy()
{
    /* Funcs for NS system accuracy verification
    * 1.
    * 2.
    */
}