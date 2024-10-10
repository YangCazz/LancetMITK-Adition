/*============================================================================
Robot-assisted Surgical Navigation Extension based on MITK
Copyright (c) Hangzhou LancetRobotics,CHINA
All rights reserved.
============================================================================*/
#pragma once

#ifndef Basic_H
#define Basic_H

#include <QmitkSingleNodeSelectionWidget.h>
#include <mitkNodePredicateDataType.h>
#include <mitkNodePredicateNot.h>
#include <mitkNodePredicateOr.h>
#include <mitkNodePredicateProperty.h>

class NeuroSurgery; // 前向声明

// 声明函数
void InitSurfaceSelector(QmitkSingleNodeSelectionWidget* widget, NeuroSurgery* ns);
void InitPointSetSelector(QmitkSingleNodeSelectionWidget* widget, NeuroSurgery* ns);

#endif // Basic_H