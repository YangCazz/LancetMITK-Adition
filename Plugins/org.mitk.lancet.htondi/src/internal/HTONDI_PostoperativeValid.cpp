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





