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
#include <QTimer>
#include <QMessageBox>
#include <QtWidgets/QFileDialog>

// mitk image
#include <mitkImage.h>
#include <mitkPlane.h>
#include <vtkVector.h>
#include <vtkAppendPolyData.h>
#include <vtkCleanPolyData.h>
#include <vtkClipPolyData.h>
#include <vtkFeatureEdges.h>
#include <vtkPlane.h>
#include <vtkPlaneSource.h>
#include <vtkCellIterator.h>
#include <vtkSmoothPolyDataFilter.h>
#include <vtkStripper.h>
#include <vtkTriangleFilter.h>
#include <ep/include/vtk-9.1/vtkTransformFilter.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPolyDataMapper.h>
#include "mitkBoundingShapeCropper.h"
#include <mitkRemeshing.h>
#include "mitkInteractionConst.h"
#include <vtkSmartPointer.h>
#include <mitkPointSet.h>
#include <mitkRenderingManager.h>
#include "mitkRotationOperation.h"
#include <vtkPolyDataPlaneClipper.h>
#include <vtkFillHolesFilter.h>
#include "mitkSurface.h"
#include "mitkSurfaceToImageFilter.h"
#include "mitkVtkInterpolationProperty.h"
#include <lancetNavigationObject.h>
#include "vtkOBBTree.h"
#include "vtkDelaunay2D.h"
#include <vtkCutter.h>
#include <ep\include\vtk-9.1\vtkPolyDataMapper.h>
#include <vtkIntersectionPolyDataFilter.h>
#include <QtCore\qmath.h>
#include <mitkGizmo.h>

#include <vtkMath.h>
#include <vtkPoints.h>

/*===============================================================
HTONDI_SurgicalSimulate.cpp
----------------------------------------------------------------
== 
== 
== 
== 
== 
== 
===============================================================*/

bool HTONDI::OnCollectPointerPosClicked()
{
	// Collect Pointer Pos
	m_Controls.textBrowser_Action->append("Action: Collect Pointer Pos.");
	// 开始采集接下来一段时间内的探针位置信息
	m_ProbePositions.clear();
	m_Transformations.clear();
	m_Controls.textBrowser_Action->append("Start collect probe data, please move slowly...");

	// 启动定时器，定期采集数据
	if (!m_ProbeDataCollectionTimer)
	{
		m_ProbeDataCollectionTimer = new QTimer(this);
		connect(m_ProbeDataCollectionTimer, &QTimer::timeout, this, &HTONDI::CollectProbeData);
	}
	m_ProbeDataCollectionTimer->start(100); // 每100ms采集一次数据

	return true;
}


bool HTONDI::OnVisualizeCollectedPoints()
{
    // Action
    m_Controls.textBrowser_Action->append("Action: Visualize Collected Points.");

    try {
        // 删除之前的测试节点
        auto previousCollectedPointsNode = GetDataStorage()->GetNamedNode("CollectedPointsNode");
        if (previousCollectedPointsNode) {
            GetDataStorage()->Remove(previousCollectedPointsNode);
        }

        auto previousCenterPointNode = GetDataStorage()->GetNamedNode("CenterPointNode");
        if (previousCenterPointNode) {
            GetDataStorage()->Remove(previousCenterPointNode);
        }

        auto previousProbeTipsNode = GetDataStorage()->GetNamedNode("ProbeTipsNode");
        if (previousProbeTipsNode) {
            GetDataStorage()->Remove(previousProbeTipsNode);
        }

        // 创建一个新的点集并设置采集到的点
        mitk::PointSet::Pointer pointSet = mitk::PointSet::New();
        for (size_t i = 0; i < m_ProbePositions.size(); ++i) {
            mitk::Point3D point;
            point[0] = m_ProbePositions[i][0] - m_Center[0];
            point[1] = m_ProbePositions[i][1] - m_Center[1];
            point[2] = m_ProbePositions[i][2] - m_Center[2];
            pointSet->InsertPoint(i, point);
        }

        // 添加球心点
        mitk::Point3D centerPoint;
        centerPoint[0] = 0.0;
        centerPoint[1] = 0.0;
        centerPoint[2] = 0.0;
        pointSet->InsertPoint(m_ProbePositions.size(), centerPoint);

        // 创建一个新的数据节点并设置点集
        mitk::DataNode::Pointer pointNode = mitk::DataNode::New();
        pointNode->SetData(pointSet);
        pointNode->SetName("CollectedPointsNode");
        // 设置节点颜色为红色
        pointNode->SetColor(1.0, 0.0, 0.0); // RGB: Red
        pointNode->SetFloatProperty("pointsize", 10);

        // 创建一个新的数据节点用于球心
        mitk::DataNode::Pointer centerNode = mitk::DataNode::New();
        mitk::PointSet::Pointer centerPointSet = mitk::PointSet::New();
        centerPointSet->InsertPoint(0, centerPoint);
        centerNode->SetData(centerPointSet);
        centerNode->SetName("CenterPointNode");
        centerNode->SetColor(0.0, 1.0, 0.0); // RGB: Green
        centerNode->SetFloatProperty("pointsize", 10);

        // 将数据节点添加到数据存储中
        GetDataStorage()->Add(pointNode);
        GetDataStorage()->Add(centerNode);

        // 计算探针针尖在双目相机坐标系下的位置
        std::vector<Eigen::Vector3d> probeTipsInCamera;
        for (const auto& T_cameraToProbe : m_Transformations)
        {
            // 将探针针尖位置转换为齐次坐标
            Eigen::Vector4d s_homogeneous(m_ProbeTip[0], m_ProbeTip[1], m_ProbeTip[2], 1.0);

            // 计算探针针尖在双目相机坐标系下的位置
            Eigen::Vector4d p_homogeneous = T_cameraToProbe * s_homogeneous;

            // 提取前3个元素，得到探针针尖在双目相机坐标系下的3D坐标
            Eigen::Vector3d p = p_homogeneous.head<3>();

            // 存储结果
            probeTipsInCamera.push_back(p);
        }

        // 创建一个新的点集并设置探针针尖在双目相机坐标系下的位置
        mitk::PointSet::Pointer probeTipPointSet = mitk::PointSet::New();
        for (size_t i = 0; i < probeTipsInCamera.size(); ++i) {
            mitk::Point3D point;
            point[0] = probeTipsInCamera[i][0];
            point[1] = probeTipsInCamera[i][1];
            point[2] = probeTipsInCamera[i][2];
            probeTipPointSet->InsertPoint(i, point);
        }

        // 创建一个新的数据节点并设置点集
        mitk::DataNode::Pointer probeTipPointNode = mitk::DataNode::New();
        probeTipPointNode->SetData(probeTipPointSet);
        probeTipPointNode->SetName("ProbeTipsNode");
        // 设置节点颜色为蓝色
        probeTipPointNode->SetColor(0.0, 0.0, 1.0); // RGB: Blue
        probeTipPointNode->SetFloatProperty("pointsize", 10);

        // 将数据节点添加到数据存储中
        GetDataStorage()->Add(probeTipPointNode);

        // 刷新视图
        mitk::RenderingManager::GetInstance()->RequestUpdateAll();
    }
    catch (const std::exception & e) {
        m_Controls.textBrowser_Action->append(QString("Error: %1").arg(e.what()));
    }
    return true;
}





