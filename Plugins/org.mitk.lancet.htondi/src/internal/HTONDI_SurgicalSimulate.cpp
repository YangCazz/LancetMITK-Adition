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
	// ��ʼ�ɼ�������һ��ʱ���ڵ�̽��λ����Ϣ
	m_ProbePositions.clear();
	m_Transformations.clear();
	m_Controls.textBrowser_Action->append("Start collect probe data, please move slowly...");

	// ������ʱ�������ڲɼ�����
	if (!m_ProbeDataCollectionTimer)
	{
		m_ProbeDataCollectionTimer = new QTimer(this);
		connect(m_ProbeDataCollectionTimer, &QTimer::timeout, this, &HTONDI::CollectProbeData);
	}
	m_ProbeDataCollectionTimer->start(100); // ÿ100ms�ɼ�һ������

	return true;
}


bool HTONDI::OnVisualizeCollectedPoints()
{
    // Action
    m_Controls.textBrowser_Action->append("Action: Visualize Collected Points.");

    try {
        // ɾ��֮ǰ�Ĳ��Խڵ�
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

        // ����һ���µĵ㼯�����òɼ����ĵ�
        mitk::PointSet::Pointer pointSet = mitk::PointSet::New();
        for (size_t i = 0; i < m_ProbePositions.size(); ++i) {
            mitk::Point3D point;
            point[0] = m_ProbePositions[i][0] - m_Center[0];
            point[1] = m_ProbePositions[i][1] - m_Center[1];
            point[2] = m_ProbePositions[i][2] - m_Center[2];
            pointSet->InsertPoint(i, point);
        }

        // ������ĵ�
        mitk::Point3D centerPoint;
        centerPoint[0] = 0.0;
        centerPoint[1] = 0.0;
        centerPoint[2] = 0.0;
        pointSet->InsertPoint(m_ProbePositions.size(), centerPoint);

        // ����һ���µ����ݽڵ㲢���õ㼯
        mitk::DataNode::Pointer pointNode = mitk::DataNode::New();
        pointNode->SetData(pointSet);
        pointNode->SetName("CollectedPointsNode");
        // ���ýڵ���ɫΪ��ɫ
        pointNode->SetColor(1.0, 0.0, 0.0); // RGB: Red
        pointNode->SetFloatProperty("pointsize", 10);

        // ����һ���µ����ݽڵ���������
        mitk::DataNode::Pointer centerNode = mitk::DataNode::New();
        mitk::PointSet::Pointer centerPointSet = mitk::PointSet::New();
        centerPointSet->InsertPoint(0, centerPoint);
        centerNode->SetData(centerPointSet);
        centerNode->SetName("CenterPointNode");
        centerNode->SetColor(0.0, 1.0, 0.0); // RGB: Green
        centerNode->SetFloatProperty("pointsize", 10);

        // �����ݽڵ���ӵ����ݴ洢��
        GetDataStorage()->Add(pointNode);
        GetDataStorage()->Add(centerNode);

        // ����̽�������˫Ŀ�������ϵ�µ�λ��
        std::vector<Eigen::Vector3d> probeTipsInCamera;
        for (const auto& T_cameraToProbe : m_Transformations)
        {
            // ��̽�����λ��ת��Ϊ�������
            Eigen::Vector4d s_homogeneous(m_ProbeTip[0], m_ProbeTip[1], m_ProbeTip[2], 1.0);

            // ����̽�������˫Ŀ�������ϵ�µ�λ��
            Eigen::Vector4d p_homogeneous = T_cameraToProbe * s_homogeneous;

            // ��ȡǰ3��Ԫ�أ��õ�̽�������˫Ŀ�������ϵ�µ�3D����
            Eigen::Vector3d p = p_homogeneous.head<3>();

            // �洢���
            probeTipsInCamera.push_back(p);
        }

        // ����һ���µĵ㼯������̽�������˫Ŀ�������ϵ�µ�λ��
        mitk::PointSet::Pointer probeTipPointSet = mitk::PointSet::New();
        for (size_t i = 0; i < probeTipsInCamera.size(); ++i) {
            mitk::Point3D point;
            point[0] = probeTipsInCamera[i][0];
            point[1] = probeTipsInCamera[i][1];
            point[2] = probeTipsInCamera[i][2];
            probeTipPointSet->InsertPoint(i, point);
        }

        // ����һ���µ����ݽڵ㲢���õ㼯
        mitk::DataNode::Pointer probeTipPointNode = mitk::DataNode::New();
        probeTipPointNode->SetData(probeTipPointSet);
        probeTipPointNode->SetName("ProbeTipsNode");
        // ���ýڵ���ɫΪ��ɫ
        probeTipPointNode->SetColor(0.0, 0.0, 1.0); // RGB: Blue
        probeTipPointNode->SetFloatProperty("pointsize", 10);

        // �����ݽڵ���ӵ����ݴ洢��
        GetDataStorage()->Add(probeTipPointNode);

        // ˢ����ͼ
        mitk::RenderingManager::GetInstance()->RequestUpdateAll();
    }
    catch (const std::exception & e) {
        m_Controls.textBrowser_Action->append(QString("Error: %1").arg(e.what()));
    }
    return true;
}





