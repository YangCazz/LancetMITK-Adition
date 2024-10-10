/*============================================================================
Robot-assisted Surgical Navigation Extension based on MITK
Copyright (c) Hangzhou LancetRobotics,CHINA
All rights reserved.
============================================================================*/

#include "NeuroSurgery.h"
#include "NS_basic.h"

// mitk
#include <mitkImage.h>
#include <mitkAffineTransform3D.h>
#include <mitkMatrixConvert.h>
#include <mitkNodePredicateAnd.h>
#include <mitkNodePredicateDataType.h>
#include <mitkNodePredicateNot.h>
#include <mitkNodePredicateOr.h>
#include <mitkNodePredicateProperty.h>
#include <mitkRenderingManager.h>
#include <mitkRotationOperation.h>
#include <mitkOperation.h>
#include <mitkTrackingDevice.h>
#include <mitkNavigationData.h>
#include <mitkPointOperation.h>
#include <mitkInteractionConst.h>
#include "mitkVirtualTrackingDevice.h"
#include "mitkVirtualTrackingTool.h"

void NeuroSurgery::ParamsInit()
{
	/* NS Param Init Here.
	* 1.
	* 2.
	*/


}

void NeuroSurgery::DataCheck()
{
	/* Data Test.
	* 1.
	* 2.
	*/


}

void InitSurfaceSelector(QmitkSingleNodeSelectionWidget* widget, NeuroSurgery* ns)
{
    // surface data Load
    widget->SetDataStorage(ns->GetDataStorage());
    widget->SetNodePredicate(mitk::NodePredicateAnd::New(
        mitk::TNodePredicateDataType<mitk::Surface>::New(),
        mitk::NodePredicateNot::New(mitk::NodePredicateOr::New(mitk::NodePredicateProperty::New("helper object"),
            mitk::NodePredicateProperty::New("hidden object")))));

    widget->SetSelectionIsOptional(true);
    widget->SetAutoSelectNewNodes(true);
    widget->SetEmptyInfo(QString("Please select a surface"));
    widget->SetPopUpTitel(QString("Select surface"));
}

void InitPointSetSelector(QmitkSingleNodeSelectionWidget* widget, NeuroSurgery* ns)
{
    // ÊµÏÖ InitPointSetSelector º¯Êý
}