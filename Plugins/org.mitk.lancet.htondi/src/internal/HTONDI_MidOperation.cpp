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
#include <QtCore\qmath.h>

// mitk
#include <mitkAffineTransform3D.h>
#include <mitkMatrixConvert.h>
#include "mitkTrackingTool.h"
#include "surfaceregistraion.h"
#include "vtkTransformFilter.h"
#include "vtkCutter.h"

#include <mitkSurfaceToImageFilter.h>


// lancet
#include <lancetRobotTrackingTool.h>

// vtk
#include <vtkPlane.h>
#include <vtkPlaneSource.h>

/*=========================术中导航==============================
HTONDI_MidOperation.cpp
----------------------------------------------------------------
== 截骨导航
== 钻孔导航
== 术中力线验证
===============================================================*/


void HTONDI::UpdateHTODrill()
{
	/* 实时磨钻导航
	1. 影像实时磨钻导航---初始状态进行投影, 后续状态进行实时更新
	2. 计算摆锯前端点位置
	3. 计算前端点到目标点差距
	4. 进行磨钻深度导航

	====== 修改0903 ======
	修改磨钻导航方法，采 用点 + 方向向量 的计算方法进行初始位置矫正
	*/

	// 1. 进行影像投影计算
	cout << "test drill 01" << endl;
	if (GetDataStorage()->GetNamedNode("Drill") == nullptr)
	{
		cout << "test drill 02-No Drill" << endl;
		return;
	}
	auto drillIndex = m_VegaToolStorage->GetToolIndexByName("DrillRF");
	auto objectRfIndex = m_VegaToolStorage->GetToolIndexByName("TibiaRF");

	if (drillIndex == -1 || objectRfIndex == -1)
	{
		m_Controls.textBrowser_Action->append("There is no 'DrillRF' or 'TibiaRF' in the toolStorage!");
		return;
	}

	cout << "test drill 03" << endl;

	// Get T_Camera_To_DrillRF
	mitk::NavigationData::Pointer nd_ndiToDrill = m_VegaSource->GetOutput(drillIndex);
	// Get T_Camera_To_TibiaRF
	mitk::NavigationData::Pointer nd_ndiToObjectRf = m_VegaSource->GetOutput(objectRfIndex);

	if (nd_ndiToObjectRf->IsDataValid() == 0 || nd_ndiToDrill->IsDataValid() == 0)
	{
		return;
	}

	cout << "test drill 04" << endl;
	// Get T_TibiaRF_To_DrillRF
	mitk::NavigationData::Pointer nd_rfToDrill = GetNavigationDataInRef(nd_ndiToDrill, nd_ndiToObjectRf);

	cout << "test drill 05" << endl;
	// Get N_DrillRF_Under_TibiaRF
	
	vtkNew<vtkMatrix4x4> vtkMatrix_rfToDrill;
	mitk::TransferItkTransformToVtkMatrix(nd_rfToDrill->GetAffineTransform3D().GetPointer(), vtkMatrix_rfToDrill);

	cout << "test drill 06" << endl;
	// Get T_TibiaRF_To_Image
	vtkNew<vtkMatrix4x4> imageToRfMatrix;
	imageToRfMatrix->DeepCopy(m_ObjectRfToImageMatrix_hto);
	imageToRfMatrix->Invert();

	cout << "test drill 07" << endl;
	// Get T_Image_To_DrillRF
	// T_TibiaRF_To_Image.inverse() * T_TibiaRF_To_DrillRF = T_Image_To_DrillRF
	vtkNew<vtkTransform> tmpTrans;
	tmpTrans->PostMultiply();
	tmpTrans->SetMatrix(vtkMatrix_rfToDrill);
	tmpTrans->Concatenate(imageToRfMatrix);
	tmpTrans->Update();

	// 格式转化
	Eigen::Matrix4d T_Image_To_DrillRF;
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			T_Image_To_DrillRF(i, j) = tmpTrans->GetMatrix()->GetElement(i, j);
		}
	}

	// 将m_DrillPointsOnDrillRF转化为矩阵 [4,n]
	Eigen::Matrix4Xd m_DrillPointsOnDrillRF4d(4, m_DrillPointsOnDrillRF.size());
	for (int i = 0; i < m_DrillPointsOnDrillRF.size(); i++) {
		m_DrillPointsOnDrillRF4d.col(i) = m_DrillPointsOnDrillRF[i];
	}

	auto pointSet_landmark = drill_image->GetLandmarks();
	Eigen::Matrix4Xd N_TP_Under_Image_Init(4, pointSet_landmark->GetSize());
	for (int i = 0; i < pointSet_landmark->GetSize(); i++)
	{
		N_TP_Under_Image_Init(0, i) = pointSet_landmark->GetPoint(i)[0];
		N_TP_Under_Image_Init(1, i) = pointSet_landmark->GetPoint(i)[1];
		N_TP_Under_Image_Init(2, i) = pointSet_landmark->GetPoint(i)[2];
		N_TP_Under_Image_Init(3, i) = 1.0;
	}


	// 计算 current 位置的 A_current 和 B_current
	Eigen::Matrix4Xd N_TP_Under_Image_Current = T_Image_To_DrillRF * m_DrillPointsOnDrillRF4d;
	// 计算方向向量 n_current = B_current->A_current
	Eigen::Vector4d tmp_current = N_TP_Under_Image_Current.col(0) - N_TP_Under_Image_Current.col(1);
	Eigen::Vector3d n_current = { tmp_current[0], tmp_current[1], tmp_current[2] };
	n_current.normalize();

	cout << "Drill_A_init" << "( " << N_TP_Under_Image_Init.col(0)[0] << " , " << N_TP_Under_Image_Init.col(0)[1] << " , " << N_TP_Under_Image_Init.col(0)[2] << ")." << endl;
	cout << "Drill_B_init" << "( " << N_TP_Under_Image_Init.col(1)[0] << " , " << N_TP_Under_Image_Init.col(1)[1] << " , " << N_TP_Under_Image_Init.col(1)[2] << ")." << endl;
	cout << "Drill_A_current" << "( " << N_TP_Under_Image_Current.col(0)[0] << " , " << N_TP_Under_Image_Current.col(0)[1] << " , " << N_TP_Under_Image_Current.col(0)[2] << ")." << endl;
	cout << "Drill_B_current" << "( " << N_TP_Under_Image_Current.col(1)[0] << " , " << N_TP_Under_Image_Current.col(1)[1] << " , " << N_TP_Under_Image_Current.col(1)[2] << ")." << endl;

	// 计算 初始位置 的方向向量
	// 计算方向向量 n_init = B_init->A_init
	Eigen::Vector4d tmp_init = N_TP_Under_Image_Init.col(0) - N_TP_Under_Image_Init.col(1);
	Eigen::Vector3d n_init = { tmp_init[0], tmp_init[1], tmp_init[2] };
	n_init.normalize();

	// 构建转化矩阵 T_current
	// 首先，创建平移矩阵
	Eigen::Vector4d T_Current_Translate = N_TP_Under_Image_Current.col(1) - N_TP_Under_Image_Init.col(1);

	// 然后，创建旋转矩阵
	// 计算旋转矩阵 n_init -> n_current ，无法直接使用矩阵计算，这里使用角度计算方法
	Eigen::Vector3d axis = n_init.cross(n_current);
	double angle = std::acos(n_init.dot(n_current));
	// 取出旋转矩阵
	Eigen::AngleAxisd rotation(angle, axis);
	Eigen::Matrix3d T_Current_Rotation = rotation.toRotationMatrix();

	// 最后，拼接变化矩阵
	Eigen::Matrix4Xd T_Current = Eigen::Matrix4d::Identity();
	// 复制旋转矩阵到左上角
	T_Current.block<3, 3>(0, 0) = T_Current_Rotation;
	// 复制平移向量到第四列
	T_Current.col(3).head(3) = T_Current_Translate.head(3);
	// 设置最后一行
	T_Current.row(3) << 0, 0, 0, 1;

	// T_Current转化回 vtkMatrix4x4
	vtkNew<vtkMatrix4x4> T_Current_vtk;
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < T_Current.cols(); j++) {
			T_Current_vtk->SetElement(i, j, T_Current(i, j));
		}
	}

	// 将其应用到初始位置上
	GetDataStorage()->GetNamedNode("Drill")->GetData()->GetGeometry()->SetIndexToWorldTransformByVtkMatrix(T_Current_vtk);
	GetDataStorage()->GetNamedNode("Drill")->GetData()->GetGeometry()->Modified();

	GetDataStorage()->GetNamedNode("DrillLandMarkPointSet")->GetData()->GetGeometry()->SetIndexToWorldTransformByVtkMatrix(T_Current_vtk);
	GetDataStorage()->GetNamedNode("DrillLandMarkPointSet")->GetData()->GetGeometry()->Modified();

	// 然后记录当前的末端点和安装点的位置
	
}

void HTONDI::UpdateHTODrill02()
{
	/* 更新磨钻针尖位置
		采用直接的矩阵计算方法来进行计算
		计算末端两点(AB)到当前两点的位置

		T
	*/
	cout << "test Drill 01" << endl;
	if (GetDataStorage()->GetNamedNode("Drill") == nullptr)
	{
		cout << "test 02-No Drill" << endl;
		return;
	}
	auto drillIndex = m_VegaToolStorage->GetToolIndexByName("DrillRF");
	auto tibiaIndex = m_VegaToolStorage->GetToolIndexByName("TibiaRF");

	if (drillIndex == -1 || tibiaIndex == -1)
	{
		m_Controls.textBrowser_Action->append("There is no 'DrillRF' or 'TibiaRF' in the toolStorage!");
		//m_HTODrillUpdateTimer->stop();
		return;
	}

	// 取出实时矩阵
	auto T_TibiaRFToCamera = vtkMatrix4x4::New();
	T_TibiaRFToCamera->DeepCopy(m_T_cameraToTibiaRF);
	T_TibiaRFToCamera->Invert();

	auto T_CameraToDrillRF = vtkMatrix4x4::New();
	T_CameraToDrillRF->DeepCopy(m_T_cameraToDrillRF);

	auto T_ImageToTibiaRF = vtkMatrix4x4::New();
	T_ImageToTibiaRF->DeepCopy(m_ObjectRfToImageMatrix_hto);
	T_ImageToTibiaRF->Invert();

	auto T_DrillRFtoImage_drill = vtkMatrix4x4::New();
	T_DrillRFtoImage_drill->DeepCopy(m_T_DrillRFtoImage_drill);

	auto tmpTrans = vtkTransform::New();
	tmpTrans->Identity();
	tmpTrans->PostMultiply();
	tmpTrans->SetMatrix(T_DrillRFtoImage_drill);
	tmpTrans->Concatenate(T_CameraToDrillRF);
	tmpTrans->Concatenate(T_TibiaRFToCamera);
	tmpTrans->Concatenate(T_ImageToTibiaRF);
	tmpTrans->Update();

	auto T_ImageToDrill = tmpTrans->GetMatrix();

	memcpy_s(m_T_ImageToDrill, sizeof(double) * 16, T_ImageToDrill->GetData(), sizeof(double) * 16);

	GetDataStorage()->GetNamedNode("Drill")->GetData()->GetGeometry()->SetIndexToWorldTransformByVtkMatrix(T_ImageToDrill);
	GetDataStorage()->GetNamedNode("Drill")->GetData()->GetGeometry()->Modified();

	GetDataStorage()->GetNamedNode("DrillLandMarkPointSet")->GetData()->GetGeometry()->SetIndexToWorldTransformByVtkMatrix(T_ImageToDrill);
	GetDataStorage()->GetNamedNode("DrillLandMarkPointSet")->GetData()->GetGeometry()->Modified();

	GetDataStorage()->GetNamedNode("LinkPin")->GetData()->GetGeometry()->SetIndexToWorldTransformByVtkMatrix(T_ImageToDrill);
	GetDataStorage()->GetNamedNode("LinkPin")->GetData()->GetGeometry()->Modified();

	GetDataStorage()->GetNamedNode("LinkPinLandMarkPointSet")->GetData()->GetGeometry()->SetIndexToWorldTransformByVtkMatrix(T_ImageToDrill);
	GetDataStorage()->GetNamedNode("LinkPinLandMarkPointSet")->GetData()->GetGeometry()->Modified();

	auto tmpKeShiPin = GetDataStorage()->GetNamedNode("Drill");
	auto tmpKeShiPin_Points = GetDataStorage()->GetNamedNode("DrillLandMarkPointSet");

	auto tmpLinkPin = GetDataStorage()->GetNamedNode("LinkPin");
	auto tmpLinkPin_Points = GetDataStorage()->GetNamedNode("LinkPinLandMarkPointSet");

	if (Drilltype == 0)
	{
		// 同时记录下当前末端位置
		CurrentDrill_Head = dynamic_cast<mitk::PointSet*>(tmpKeShiPin_Points->GetData())->GetPoint(1);
		CurrentDrill_Tail = dynamic_cast<mitk::PointSet*>(tmpKeShiPin_Points->GetData())->GetPoint(0);

		// copy
		Eigen::Vector3d Current_Head, Current_Tail;
		Current_Head[0] = CurrentDrill_Head[0];
		Current_Head[1] = CurrentDrill_Head[1];
		Current_Head[2] = CurrentDrill_Head[2];

		Current_Tail[0] = CurrentDrill_Tail[0];
		Current_Tail[1] = CurrentDrill_Tail[1];
		Current_Tail[2] = CurrentDrill_Tail[2];

		// 设置数值
		m_Controls.label_des_x->setText(QString::number(CurrentDrill_Tail[0]));
		m_Controls.label_des_y->setText(QString::number(CurrentDrill_Tail[1]));
		m_Controls.label_des_z->setText(QString::number(CurrentDrill_Tail[2]));


		// 设置差值
		double error_x = Destin_Tail[0] - Current_Tail[0];
		double error_y = Destin_Tail[1] - Current_Tail[1];
		double error_z = Destin_Tail[2] - Current_Tail[2];

		// 设置数值差值 区分 正负
		if (Destin_Tail[0] - CurrentDrill_Tail[0] > 0) {
			m_Controls.LineEdit_distance_End_x->setText("+" + QString::number(Destin_Tail[0] - CurrentDrill_Tail[0]));
		}
		else
		{
			m_Controls.LineEdit_distance_End_x->setText(QString::number(Destin_Tail[0] - CurrentDrill_Tail[0]));
		}
		if (Destin_Tail[1] - CurrentDrill_Tail[1] > 0) {
			m_Controls.LineEdit_distance_End_x->setText("+" + QString::number(Destin_Tail[1] - CurrentDrill_Tail[1]));
		}
		else
		{
			m_Controls.LineEdit_distance_End_x->setText(QString::number(Destin_Tail[1] - CurrentDrill_Tail[1]));
		}
		if (Destin_Tail[2] - CurrentDrill_Tail[2] > 0) {
			m_Controls.LineEdit_distance_End_x->setText("+" + QString::number(Destin_Tail[2] - CurrentDrill_Tail[2]));
		}
		else
		{
			m_Controls.LineEdit_distance_End_x->setText(QString::number(Destin_Tail[2] - CurrentDrill_Tail[2]));
		}

		// 计算角度误差
		Eigen::Vector3d normal_current;
		normal_current = Current_Tail - Current_Head;
		normal_current.norm();

		// 得到当前方向量在三个平面的投影夹角分量
		double angle_current_xoy = 180 * asin(normal_current[2]) / 3.141592654;
		double angle_current_xoz = 180 * asin(normal_current[1]) / 3.141592654;
		double angle_current_yoz = 180 * asin(normal_current[0]) / 3.141592654;

		// 得到当前方向量在三个平面的投影夹角分量
		double angle_set_xoy = 180 * asin(normal_KeShi01[2]) / 3.141592654;
		double angle_set_xoz = 180 * asin(normal_KeShi01[1]) / 3.141592654;
		double angle_set_yoz = 180 * asin(normal_KeShi01[0]) / 3.141592654;

		double error_xoy = angle_set_xoy - angle_current_xoy;
		double error_xoz = angle_set_xoz - angle_current_xoz;
		double error_yoz = angle_set_yoz - angle_current_yoz;

		if (error_xoy > 0)
		{
			m_Controls.LineEdit_distance_Angle_xoy->setText("+" + QString::number(error_xoy));
		}
		else
		{
			m_Controls.LineEdit_distance_Angle_xoy->setText(QString::number(error_xoy));
		}
		if (error_xoz > 0)
		{
			m_Controls.LineEdit_distance_Angle_xoy->setText("+" + QString::number(error_xoz));
		}
		else
		{
			m_Controls.LineEdit_distance_Angle_xoy->setText(QString::number(error_xoy));
		}
		if (error_yoz > 0)
		{
			m_Controls.LineEdit_distance_Angle_xoy->setText("+" + QString::number(error_yoz));
		}
		else
		{
			m_Controls.LineEdit_distance_Angle_xoy->setText(QString::number(error_xoy));
		}
	}
	else if (Drilltype == 1)
	{
		// 同时记录下当前末端位置
		CurrentDrill_Head = dynamic_cast<mitk::PointSet*>(tmpLinkPin_Points->GetData())->GetPoint(1);
		CurrentDrill_Tail = dynamic_cast<mitk::PointSet*>(tmpLinkPin_Points->GetData())->GetPoint(0);

		// copy
		Eigen::Vector3d Current_Head, Current_Tail;
		Current_Head = { CurrentDrill_Head[0], CurrentDrill_Head[1], CurrentDrill_Head[2] };
		Current_Tail = { CurrentDrill_Tail[0], CurrentDrill_Tail[1], CurrentDrill_Tail[2] };

		// 设置数值
		m_Controls.label_des_x->setText("x:" + QString::number(CurrentDrill_Tail[0]));
		m_Controls.label_des_y->setText("y:" + QString::number(CurrentDrill_Tail[1]));
		m_Controls.label_des_z->setText("z:" + QString::number(CurrentDrill_Tail[2]));


		// 设置差值
		double error_x = Destin_Tail[0] - CurrentDrill_Tail[0];
		double error_y = Destin_Tail[1] - CurrentDrill_Tail[1];
		double error_z = Destin_Tail[2] - CurrentDrill_Tail[2];

		// 设置数值差值 区分 正负
		if (error_x > 0) {
			m_Controls.LineEdit_distance_End_x->setText("+" + QString::number(error_x));
		}
		else
		{
			m_Controls.LineEdit_distance_End_x->setText(QString::number(error_x));
		}
		if (error_y > 0) {
			m_Controls.LineEdit_distance_End_x->setText("+" + QString::number(error_y));
		}
		else
		{
			m_Controls.LineEdit_distance_End_x->setText(QString::number(error_y));
		}
		if (error_z > 0) {
			m_Controls.LineEdit_distance_End_x->setText("+" + QString::number(error_z));
		}
		else
		{
			m_Controls.LineEdit_distance_End_x->setText(QString::number(error_z));
		}

		// 计算角度误差
		Eigen::Vector3d normal_current;
		normal_current = Current_Tail - Current_Head;
		normal_current.norm();

		// 得到当前方向量在三个平面的投影夹角分量
		double angle_current_xoy = 180 * asin(normal_current[2]) / 3.141592654;
		double angle_current_xoz = 180 * asin(normal_current[1]) / 3.141592654;
		double angle_current_yoz = 180 * asin(normal_current[0]) / 3.141592654;

		// 得到当前方向量在三个平面的投影夹角分量
		double angle_set_xoy = 180 * asin(normal_KeShi01[2]) / 3.141592654;
		double angle_set_xoz = 180 * asin(normal_KeShi01[1]) / 3.141592654;
		double angle_set_yoz = 180 * asin(normal_KeShi01[0]) / 3.141592654;

		double error_xoy = angle_set_xoy - angle_current_xoy;
		double error_xoz = angle_set_xoz - angle_current_xoz;
		double error_yoz = angle_set_yoz - angle_current_yoz;

		if (error_xoy > 0)
		{
			m_Controls.LineEdit_distance_Angle_xoy->setText("+" + QString::number(error_xoy));
		}
		else
		{
			m_Controls.LineEdit_distance_Angle_xoy->setText(QString::number(error_xoy));
		}
		if (error_xoz > 0)
		{
			m_Controls.LineEdit_distance_Angle_xoy->setText("+" + QString::number(error_xoz));
		}
		else
		{
			m_Controls.LineEdit_distance_Angle_xoy->setText(QString::number(error_xoy));
		}
		if (error_yoz > 0)
		{
			m_Controls.LineEdit_distance_Angle_xoy->setText("+" + QString::number(error_yoz));
		}
		else
		{
			m_Controls.LineEdit_distance_Angle_xoy->setText(QString::number(error_xoy));
		}
	}
	else
	{
		m_Controls.textBrowser_Action->append("There is no 'DrillLandMarkPointSet' or 'LinkPinLandMarkPointSet' in the Storage!");
		//m_HTODrillUpdateTimer->stop();
		return;
	}



}

void HTONDI::UpdateHTOSaw()
{
	/* 实时截骨导航
		1. 影像实时摆锯导航---初始状态进行投影, 后续状态进行实时更新
		2. 计算摆锯平面夹角误差
		3. 计算截骨深度
		4. 进行截骨模拟渲染
	*/

	// 1. 进行影像投影计算

	cout << "test Saw 01" << endl;
	if (GetDataStorage()->GetNamedNode("Saw") == nullptr)
	{
		cout << "test Saw 02-No Saw" << endl;
		return;
	}
	auto probeIndex = m_VegaToolStorage->GetToolIndexByName("SawRF");
	auto objectRfIndex = m_VegaToolStorage->GetToolIndexByName("TibiaRF");

	if (probeIndex == -1 || objectRfIndex == -1)
	{
		m_Controls.textBrowser_Action->append("There is no 'SawRF' or 'TibiaRF' in the toolStorage!");
		return;
	}

	cout << "test Saw 03" << endl;

	// Check the availability of the optic tools in the FOV
	mitk::NavigationData::Pointer nd_ndiToProbe = m_VegaSource->GetOutput(probeIndex);
	mitk::NavigationData::Pointer nd_ndiToObjectRf = m_VegaSource->GetOutput(objectRfIndex);

	if (nd_ndiToObjectRf->IsDataValid() == 0 || nd_ndiToProbe->IsDataValid() == 0)
	{
		return;
	}

	cout << "test Saw 04" << endl;
	mitk::NavigationData::Pointer nd_rfToProbe = GetNavigationDataInRef(nd_ndiToProbe, nd_ndiToObjectRf);

	cout << "test Saw 05" << endl;
	// 使用探针时，首次需要保证在相机下
	vtkNew<vtkMatrix4x4> vtkMatrix_rfToProbe;
	mitk::TransferItkTransformToVtkMatrix(nd_rfToProbe->GetAffineTransform3D().GetPointer(), vtkMatrix_rfToProbe);

	cout << "test Saw 06" << endl;
	vtkNew<vtkMatrix4x4> imageToRfMatrix;
	imageToRfMatrix->DeepCopy(m_ObjectRfToImageMatrix_hto);
	imageToRfMatrix->Invert();

	cout << "test Saw 07" << endl;
	vtkNew<vtkTransform> tmpTrans;
	tmpTrans->PostMultiply();
	tmpTrans->SetMatrix(vtkMatrix_rfToProbe);
	tmpTrans->Concatenate(imageToRfMatrix);
	tmpTrans->Update();

	if (start_saw == false)
	{
		/* 在初始情况下，landmark点的位置是实际位置，这个时候可以借助这个点集来计算 */
		// 将T_Image_To_SawRF转化为Eigen::Matrix4d格式
		cout << "test Saw 08-01" << endl;
		Eigen::Matrix4d T_Image_To_SawRF;
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				T_Image_To_SawRF(i, j) = tmpTrans->GetMatrix()->GetElement(i, j);
			}
		}

		// 将m_SawPointsOnSawRF转化为矩阵 [4,n]
		Eigen::Matrix4Xd m_SawPointsOnSawRF4d(4, m_SawPointsOnSawRF.size());
		for (int i = 0; i < m_SawPointsOnSawRF.size(); i++) {
			m_SawPointsOnSawRF4d.col(i) = m_SawPointsOnSawRF[i];
		}

		cout << "test Saw 08-02" << endl;
		// 计算current位置
		Eigen::Matrix4Xd N_TP_Under_Image_Recent = T_Image_To_SawRF * m_SawPointsOnSawRF4d;

		// 取出last位置
		// 取出landmark点同样转化为矩阵 [4,n]
		auto pointSet_landmark = saw_image->GetLandmarks();
		Eigen::Matrix4Xd N_TP_Under_Image_Last(4, pointSet_landmark->GetSize());
		for (int i = 0; i < pointSet_landmark->GetSize(); i++) {
			N_TP_Under_Image_Last(0, i) = pointSet_landmark->GetPoint(i)[0];
			N_TP_Under_Image_Last(1, i) = pointSet_landmark->GetPoint(i)[1];
			N_TP_Under_Image_Last(2, i) = pointSet_landmark->GetPoint(i)[2];
			N_TP_Under_Image_Last(3, i) = 1;
		}

		cout << "test Saw 08-03" << endl;
		// 计算转化矩阵T_Current
		Eigen::Matrix4Xd T_Current = N_TP_Under_Image_Recent * N_TP_Under_Image_Last.inverse();

		// -------- 出于一些问题，这里计算结果不用 ---------
		cout << "test Saw 08-04" << endl;
		// T_Current转化回 vtkMatrix4x4
		//vtkNew<vtkMatrix4x4> T_Current_vtk;
		//for (int i = 0; i < 4; i++) {
		//	for (int j = 0; j < T_Current.cols(); j++) {
		//		T_Current_vtk->SetElement(i, j, T_Current(i, j));
		//	}
		//}
		// ----------------------------------------------

		cout << "test Saw 08-05" << endl;
		/* 方法: 将当前映射后的点 N_SP_Under_Camera_current 与  N_SP_Under_Camera_last 的配准矩阵 T_Current2Last */
		// 首先将当前点的位置投影到Image中 N_SP_Under_Camera_current = T_Image2SawRF * N_SP_Under_SawRF

		auto landmark_rf = mitk::PointSet::New();
		// 打印当前点的信息
		for (int i = 0; i < saw_image->GetLandmarks()->GetSize(); i++)
		{
			cout << "test Saw 08-05-01" << endl;
			Eigen::VectorXd columnData = N_TP_Under_Image_Recent.col(i);
			mitk::Point3D point;
			point[0] = columnData[0];
			point[1] = columnData[1];
			point[2] = columnData[2];
			cout << "test Saw 08-05-02" << endl;
			landmark_rf->InsertPoint(point);
			cout << "test Saw 08-05-03" << endl;
		}

		cout << "test Saw 08-06" << endl;
		auto Current2Last_Registrator = mitk::SurfaceRegistration::New();
		Current2Last_Registrator->SetLandmarksSrc(saw_image->GetLandmarks());
		Current2Last_Registrator->SetLandmarksTarget(landmark_rf);
		Current2Last_Registrator->ComputeLandMarkResult();

		// 提取T_Current2Last
		vtkNew<vtkMatrix4x4> T_Current2Last;
		T_Current2Last->DeepCopy(Current2Last_Registrator->GetResult());

		cout << "test Saw 08-07" << endl;
		// 将Last转化为Current
		GetDataStorage()->GetNamedNode("Saw")->GetData()->GetGeometry()->SetIndexToWorldTransformByVtkMatrix(T_Current2Last);
		GetDataStorage()->GetNamedNode("Saw")->GetData()->GetGeometry()->Modified();

		cout << "test Saw 08-08" << endl;
		// 更新实时截骨面
		GetDataStorage()->GetNamedNode("CurrentCutPlane01")->GetData()->GetGeometry()->SetIndexToWorldTransformByVtkMatrix(T_Current2Last);
		GetDataStorage()->GetNamedNode("CurrentCutPlane01")->GetData()->GetGeometry()->Modified();

		cout << "test Saw 08-09" << endl;
		// 更新实时截骨面前端点
		GetDataStorage()->GetNamedNode("pointSetInRealPlaneAxial")->GetData()->GetGeometry()->SetIndexToWorldTransformByVtkMatrix(T_Current2Last);
		GetDataStorage()->GetNamedNode("pointSetInRealPlaneAxial")->GetData()->GetGeometry()->Modified();

		// 更新标准位
		start_saw = true;

		cout << "saw init done" << endl;
	}
	else if (start_saw == true)
	{
		cout << "test Saw 09" << endl;
		// 更新摆锯位置
		GetDataStorage()->GetNamedNode("Saw")->GetData()->GetGeometry()->SetIndexToWorldTransformByVtkMatrix(tmpTrans->GetMatrix());
		GetDataStorage()->GetNamedNode("Saw")->GetData()->GetGeometry()->Modified();

		// 更新实时截骨面
		GetDataStorage()->GetNamedNode("CurrentCutPlane01")->GetData()->GetGeometry()->SetIndexToWorldTransformByVtkMatrix(tmpTrans->GetMatrix());
		GetDataStorage()->GetNamedNode("CurrentCutPlane01")->GetData()->GetGeometry()->Modified();

		// 更新实时截骨面前端点
		GetDataStorage()->GetNamedNode("pointSetInRealPlaneAxial")->GetData()->GetGeometry()->SetIndexToWorldTransformByVtkMatrix(tmpTrans->GetMatrix());
		GetDataStorage()->GetNamedNode("pointSetInRealPlaneAxial")->GetData()->GetGeometry()->Modified();
	}

	//// 更新摆锯位置
	//GetDataStorage()->GetNamedNode("Saw")->GetData()->GetGeometry()->SetIndexToWorldTransformByVtkMatrix(tmpTrans->GetMatrix());
	//GetDataStorage()->GetNamedNode("Saw")->GetData()->GetGeometry()->Modified();

	//// 更新实时截骨面
	//GetDataStorage()->GetNamedNode("CurrentCutPlane01")->GetData()->GetGeometry()->SetIndexToWorldTransformByVtkMatrix(tmpTrans->GetMatrix());
	//GetDataStorage()->GetNamedNode("CurrentCutPlane01")->GetData()->GetGeometry()->Modified();

	//// 更新实时截骨面前端点
	//GetDataStorage()->GetNamedNode("pointSetInRealPlaneAxial")->GetData()->GetGeometry()->SetIndexToWorldTransformByVtkMatrix(tmpTrans->GetMatrix());
	//GetDataStorage()->GetNamedNode("pointSetInRealPlaneAxial")->GetData()->GetGeometry()->Modified();



	// 2. 首先确保克式钉确定的截骨平面是正确的
	if (m_RealtimeAngleCheck) {
		cout << "test Saw 10" << endl;
		// 计算法向量夹角
		auto sourceCutPlane01 = dynamic_cast<mitk::Surface*>(GetDataStorage()->GetNamedNode("1st cut plane")->GetData());
		auto sourceCutPlanePolyData01 = sourceCutPlane01->GetVtkPolyData();

		cout << "test Saw 11" << endl;
		auto currentCutPlane01 = dynamic_cast<mitk::Surface*>(GetDataStorage()->GetNamedNode("CurrentCutPlane01")->GetData());
		auto currentCutPlanePolyData01 = currentCutPlane01->GetVtkPolyData();

		cout << "test Saw 12" << endl;
		Eigen::Vector3d normalSourcePlane01, normalCurrentPlane01;
		normalSourcePlane01 = ExtractNormalFromPlane("1st cut plane");
		normalCurrentPlane01 = ExtractNormalFromPlane("CurrentCutPlane01");

		// 计算两个方向向量之间的夹角
		double dotProduct = normalSourcePlane01[0] * normalCurrentPlane01[0] + normalSourcePlane01[1] * normalCurrentPlane01[1] + normalSourcePlane01[2] * normalCurrentPlane01[2]; // 点积

		// 计算向量长度
		double magnitude1 = sqrt(pow(normalSourcePlane01[0], 2) + pow(normalSourcePlane01[1], 2) + pow(normalSourcePlane01[2], 2));
		double magnitude2 = sqrt(pow(normalCurrentPlane01[0], 2) + pow(normalCurrentPlane01[1], 2) + pow(normalCurrentPlane01[2], 2));

		// 计算夹角的余弦值
		double cosAngle = dotProduct / (magnitude1 * magnitude2);

		// 使用反余弦函数计算夹角（弧度）
		double angleInRadians = acos(cosAngle);

		// 将弧度转换为度数，保留1位小数
		double angleInDegrees = round(angleInRadians * (180.0 / M_PI) * 10) / 10;
		if (angleInDegrees > 90)
		{
			angleInDegrees = 180.0 - angleInDegrees;
		}

		// 输出夹角到实际的位置
		m_Controls.textBrowser_AxialCut->append("Real time Angle Miss: " + QString::number(angleInDegrees));

		if (angleInDegrees < 0.5)
		{
			m_Controls.textBrowser_AxialCut03->setText("Current Angle Missing < 0.5 is acceptable.");
		}
		else
		{
			m_Controls.textBrowser_AxialCut03->setText("Current Angle Missing > 0.5 !!");
		}
	}
	

	// 3. 在确定截骨角度正常后，开始进行截骨
	// - 首先计算截骨线 ==> 然后计算截骨深度
	// - 这里假设摆锯前端与规划截骨面前端平行

	// 3.1 计算截骨线，获取最值点
	if (m_RealtimeCutPlaneCheck == true)
	{
		// 以调整好的克式钉角度进行截骨计算
		GetRealTimeIntersectionLine("CurrentCutPlane01", "tibiaSurface");
		m_RealtimeCutPlaneCheck = false;
	}

	// 3.2 计算截骨深度
	if (m_RealtimeCutCheck == true)
	{
		// 取出实时截骨面的前端点位置
		auto tmpNodes = GetDataStorage()->GetNamedNode("pointSetInRealPlaneAxial");
		if (tmpNodes) {
			// 获取数据
			mitk::Point3D point1 = dynamic_cast<mitk::PointSet*>(tmpNodes->GetData())->GetPoint(1);
			mitk::Point3D point2 = dynamic_cast<mitk::PointSet*>(tmpNodes->GetData())->GetPoint(2);

			// 计算截骨深度，点到直线的距离
			double front2Min, front2Max;
			double pos_FrontLeft[3], pos_FrontRight[3];
			pos_FrontLeft[0] = point1[0];
			pos_FrontLeft[1] = point1[1];
			pos_FrontLeft[2] = point1[2];

			pos_FrontRight[0] = point2[0];
			pos_FrontRight[1] = point2[1];
			pos_FrontRight[2] = point2[2];

			// 得到点线距离
			front2Min = DistancePointToLine(pos_FrontLeft, m_CutIntersectionMinPos, m_CutIntersectionMaxPos);
			front2Max = DistancePointToLine(pos_FrontRight, m_CutIntersectionMaxPos, m_CutIntersectionMaxPos);

			// 依据点坐标来进行判别输出
			if (pos_FrontLeft[0] > m_CutIntersectionMinPos[0] && pos_FrontLeft[0] < m_CutIntersectionMaxPos[0])
			{
				/* 锯片前端出于胫骨内部 */
				m_Controls.textBrowser_AxialCut02->setText("Current Cut Depth: " + QString::number(front2Min));

				// 进行截骨阈值判断
				if (m_threshold_SawDepth - front2Min >= 0.2)
				{
					m_Controls.textBrowser_AxialCut03->setText("State: Safe");
				}
				else if (m_threshold_SawDepth - front2Min >= 0 && m_threshold_SawDepth - front2Min < 0.2)
				{
					m_Controls.textBrowser_AxialCut03->setText("State: Warning!");
				}
			}
			else if (pos_FrontLeft[0] < m_CutIntersectionMinPos[0])
			{
				/* 锯片前端未抵达胫骨 */
				m_Controls.textBrowser_AxialCut02->setText("Current Cut Depth: 0, Current Distance From SawFront to Tibia: " + QString::number(front2Min));
			}
			else if (pos_FrontLeft[0] > m_CutIntersectionMaxPos[0])
			{
				/* 锯片前端磨穿达胫骨 */
				m_Controls.textBrowser_AxialCut03->setText("State: Saw System Off.");
				m_SawPower = false;
			}
		}
		else {
			m_Controls.textBrowser_Action->append("Error: pointSetInRealPlaneAxial is missing!");
		}
	}

	// 3.3 模拟截骨
	if (m_SawPower == true)
	{
		// 电源于开启状态则进行截骨模拟
		// TestCut
	}
	
	

	cout << "refresh" << endl;
}

void  HTONDI::UpdateHTOSaw02()
{
	/* 截骨面导航
		1. 影像实时摆锯导航---初始状态进行投影, 后续状态进行实时更新
		2. 计算摆锯平面夹角误差
		3. 计算截骨深度
		4. 进行截骨模拟渲染
	*/

	// 1. 进行影像投影计算

	cout << "test Saw 01" << endl;
	if (GetDataStorage()->GetNamedNode("Saw") == nullptr)
	{
		cout << "test Saw 02-No Saw" << endl;
		return;
	}
	auto probeIndex = m_VegaToolStorage->GetToolIndexByName("SawRF");
	auto objectRfIndex = m_VegaToolStorage->GetToolIndexByName("TibiaRF");

	if (probeIndex == -1 || objectRfIndex == -1)
	{
		m_Controls.textBrowser_Action->append("There is no 'SawRF' or 'TibiaRF' in the toolStorage!");
		return;
	}

	// 取出实时矩阵
	auto T_TibiaRFToCamera = vtkMatrix4x4::New();
	T_TibiaRFToCamera->DeepCopy(m_T_cameraToTibiaRF);
	T_TibiaRFToCamera->Invert();

	auto T_CameraToSawRF = vtkMatrix4x4::New();
	T_CameraToSawRF->DeepCopy(m_T_cameraToSawRF);

	auto T_ImageToTibiaRF = vtkMatrix4x4::New();
	T_ImageToTibiaRF->DeepCopy(m_ObjectRfToImageMatrix_hto);
	T_ImageToTibiaRF->Invert();

	
	auto T_SawRFtoImage_saw = vtkMatrix4x4::New();
	T_SawRFtoImage_saw->DeepCopy(m_T_SawRFtoImage_saw);

	auto tmpTrans = vtkTransform::New();
	tmpTrans->Identity();
	tmpTrans->PostMultiply();
	tmpTrans->SetMatrix(T_SawRFtoImage_saw);
	tmpTrans->Concatenate(T_CameraToSawRF);
	tmpTrans->Concatenate(T_TibiaRFToCamera);
	tmpTrans->Concatenate(T_ImageToTibiaRF);
	tmpTrans->Update();

	auto T_ImageToSaw = tmpTrans->GetMatrix();

	memcpy_s(m_T_ImageToSaw, sizeof(double) * 16, T_ImageToSaw->GetData(), sizeof(double) * 16);

	GetDataStorage()->GetNamedNode("Saw")->GetData()->GetGeometry()->SetIndexToWorldTransformByVtkMatrix(T_ImageToSaw);
	GetDataStorage()->GetNamedNode("Saw")->GetData()->GetGeometry()->Modified();


	GetDataStorage()->GetNamedNode("SawLandMarkPointSet")->GetData()->GetGeometry()->SetIndexToWorldTransformByVtkMatrix(T_ImageToSaw);
	GetDataStorage()->GetNamedNode("SawLandMarkPointSet")->GetData()->GetGeometry()->Modified();

	// 更新实时截骨面
	GetDataStorage()->GetNamedNode("CurrentCutPlane01")->GetData()->GetGeometry()->SetIndexToWorldTransformByVtkMatrix(T_ImageToSaw);
	GetDataStorage()->GetNamedNode("CurrentCutPlane01")->GetData()->GetGeometry()->Modified();

	cout << "test Saw 08-09" << endl;
	// 更新实时截骨面前端点
	GetDataStorage()->GetNamedNode("pointSetInRealPlaneAxial")->GetData()->GetGeometry()->SetIndexToWorldTransformByVtkMatrix(T_ImageToSaw);
	GetDataStorage()->GetNamedNode("pointSetInRealPlaneAxial")->GetData()->GetGeometry()->Modified();




	// 2. 首先确保克式钉确定的截骨平面是正确的
	if (m_RealtimeAngleCheck) {
		cout << "test Saw 10" << endl;
		// 计算法向量夹角
		auto sourceCutPlane01 = dynamic_cast<mitk::Surface*>(GetDataStorage()->GetNamedNode("1st cut plane")->GetData());
		auto sourceCutPlanePolyData01 = sourceCutPlane01->GetVtkPolyData();

		cout << "test Saw 11" << endl;
		auto currentCutPlane01 = dynamic_cast<mitk::Surface*>(GetDataStorage()->GetNamedNode("CurrentCutPlane01")->GetData());
		auto currentCutPlanePolyData01 = currentCutPlane01->GetVtkPolyData();

		cout << "test Saw 12" << endl;
		Eigen::Vector3d normalSourcePlane01, normalCurrentPlane01;
		normalSourcePlane01 = ExtractNormalFromPlane("1st cut plane");
		normalCurrentPlane01 = ExtractNormalFromPlane("CurrentCutPlane01");

		// 计算两个方向向量之间的夹角
		double dotProduct = normalSourcePlane01[0] * normalCurrentPlane01[0] + normalSourcePlane01[1] * normalCurrentPlane01[1] + normalSourcePlane01[2] * normalCurrentPlane01[2]; // 点积

		// 计算向量长度
		double magnitude1 = sqrt(pow(normalSourcePlane01[0], 2) + pow(normalSourcePlane01[1], 2) + pow(normalSourcePlane01[2], 2));
		double magnitude2 = sqrt(pow(normalCurrentPlane01[0], 2) + pow(normalCurrentPlane01[1], 2) + pow(normalCurrentPlane01[2], 2));

		// 计算夹角的余弦值
		double cosAngle = dotProduct / (magnitude1 * magnitude2);

		// 使用反余弦函数计算夹角（弧度）
		double angleInRadians = acos(cosAngle);

		// 将弧度转换为度数，保留1位小数
		double angleInDegrees = round(angleInRadians * (180.0 / M_PI) * 10) / 10;
		if (angleInDegrees > 90)
		{
			angleInDegrees = 180.0 - angleInDegrees;
		}

		// 输出夹角到实际的位置
		m_Controls.textBrowser_AxialCut->append("Real time Angle Miss: " + QString::number(angleInDegrees));

		if (angleInDegrees < 0.5)
		{
			m_Controls.textBrowser_AxialCut03->setText("Current Angle Missing < 0.5 is acceptable.");
		}
		else
		{
			m_Controls.textBrowser_AxialCut03->setText("Current Angle Missing > 0.5 !!");
		}
	}


	// 3. 在确定截骨角度正常后，开始进行截骨
	// - 首先计算截骨线 ==> 然后计算截骨深度
	// - 这里假设摆锯前端与规划截骨面前端平行

	// 3.1 计算截骨线，获取最值点
	if (m_RealtimeCutPlaneCheck == true)
	{
		// 以调整好的克式钉角度进行截骨计算
		GetRealTimeIntersectionLine("CurrentCutPlane01", "tibiaSurface");
		m_RealtimeCutPlaneCheck = false;
	}

	// 3.2 计算截骨深度
	if (m_RealtimeCutCheck == true)
	{
		// 取出实时截骨面的前端点位置
		auto tmpNodes = GetDataStorage()->GetNamedNode("pointSetInRealPlaneAxial");
		if (tmpNodes) {
			// 获取数据
			mitk::Point3D point1 = dynamic_cast<mitk::PointSet*>(tmpNodes->GetData())->GetPoint(1);
			mitk::Point3D point2 = dynamic_cast<mitk::PointSet*>(tmpNodes->GetData())->GetPoint(2);

			// 计算截骨深度，点到直线的距离
			double front2Min, front2Max;
			double pos_FrontLeft[3], pos_FrontRight[3];
			pos_FrontLeft[0] = point1[0];
			pos_FrontLeft[1] = point1[1];
			pos_FrontLeft[2] = point1[2];

			pos_FrontRight[0] = point2[0];
			pos_FrontRight[1] = point2[1];
			pos_FrontRight[2] = point2[2];

			// 得到点线距离
			front2Min = DistancePointToLine(pos_FrontLeft, m_CutIntersectionMinPos, m_CutIntersectionMaxPos);
			front2Max = DistancePointToLine(pos_FrontRight, m_CutIntersectionMaxPos, m_CutIntersectionMaxPos);

			// 依据点坐标来进行判别输出
			if (pos_FrontLeft[0] > m_CutIntersectionMinPos[0] && pos_FrontLeft[0] < m_CutIntersectionMaxPos[0])
			{
				/* 锯片前端出于胫骨内部 */
				m_Controls.textBrowser_AxialCut02->setText("Current Cut Depth: " + QString::number(front2Min));

				// 进行截骨阈值判断
				if (m_threshold_SawDepth - front2Min >= 0.2)
				{
					m_Controls.textBrowser_AxialCut03->setText("State: Safe");
				}
				else if (m_threshold_SawDepth - front2Min >= 0 && m_threshold_SawDepth - front2Min < 0.2)
				{
					m_Controls.textBrowser_AxialCut03->setText("State: Warning!");
				}
			}
			else if (pos_FrontLeft[0] < m_CutIntersectionMinPos[0])
			{
				/* 锯片前端未抵达胫骨 */
				m_Controls.textBrowser_AxialCut02->setText("Current Cut Depth: 0, Current Distance From SawFront to Tibia: " + QString::number(front2Min));
			}
			else if (pos_FrontLeft[0] > m_CutIntersectionMaxPos[0])
			{
				/* 锯片前端磨穿达胫骨 */
				m_Controls.textBrowser_AxialCut03->setText("State: Saw System Off.");
				m_SawPower = false;
			}
		}
		else {
			m_Controls.textBrowser_Action->append("Error: pointSetInRealPlaneAxial is missing!");
		}
	}

	// 3.3 模拟截骨
	if (m_SawPower == true)
	{
		// 电源于开启状态则进行截骨模拟
		// TestCut
	}



	cout << "refresh" << endl;
}

bool HTONDI::OnStartAxialGuideClicked()
{
	/* 开始水平截骨导航
	1. 生成实时截骨面
	2. 计算 实时截骨面 与 规划截骨面 的夹角

	开始水平截骨导航时，需要计算截骨面的法向量和前端位置
	那么，也就需要知道实时的摆锯最前端的截骨面点以及对应的截骨面法向量

	====== 修改0828 ======
	修改导航算法
	*/

	m_Controls.textBrowser_Action->append("Action: Start Axial Cut Guide.");
	m_cutType = 1;

	// 1. 显示规划的截骨面 + 显示实时截骨面位置
	auto preCutPlane01 = GetDataStorage()->GetNamedNode("1st cut plane");
	auto realTimeSaw = GetDataStorage()->GetNamedNode("Saw");
	auto realCutplane01 = GetDataStorage()->GetNamedNode("CurrentCutPlane01");
	auto realPointsOnCutplane01 = GetDataStorage()->GetNamedNode("pointSetInRealPlaneAxial");

	// 检测数据存在，然后打开截骨导航
	if (preCutPlane01 && realTimeSaw && realCutplane01 && realPointsOnCutplane01)
	{
		preCutPlane01->SetVisibility(true);
		realTimeSaw->SetVisibility(true);
		realCutplane01->SetVisibility(true);
		realPointsOnCutplane01->SetVisibility(true);
	}
	else
	{
		m_Controls.textBrowser_Action->append("Cut plane or Saw model Not Found!");
		return false;
	}

	// 先运行一次，更新到目标位置上来
	//UpdateHTOSaw();
	
	// 对Saw生成实时截骨平面
	if (m_HTOSawUpdateTimer == nullptr)
	{
		m_HTOSawUpdateTimer = new QTimer(this);
	}
	m_Controls.textBrowser_Action->append("Generate Real time Cut plane");
	
	// 先终止探针的识别
	//m_HTOPrboeUpdateTimer->stop();
	//cout << "stop probe visulization" << endl;


	connect(m_HTOSawUpdateTimer, SIGNAL(timeout()), this, SLOT(UpdateHTOSaw02()));
	m_HTOSawUpdateTimer->start(100);
	
	// 启动导航的同时，计算误差角度
	m_RealtimeAngleCheck = true;

	return true;
}

void HTONDI::RenewSaw()
{
	UpdateHTOSaw();
}

void HTONDI::RenewDrill()
{
	UpdateHTODrill();
}

// 术中导航
void HTONDI::trackingObjectPos()
{
	/* 追踪并返回导航物体在相机视角下的转化矩阵 
		1. 股骨 FemurRF - m_MetrixCameraToFemurRF
		2. 胫骨 TibiaRF - m_MetrixCameraToTibiaRF
		3. 探针 ProbeRF - m_MetrixCameraToProbeRF
		4. 摆锯 SawRF   - m_MetrixCameraToSawRF
		5. 磨钻 DrillRF - m_MetrixCameraToDrillRF
	*/
	// 开始进行配准点采集
	// Pos_Tip = T_Camera2ProbeRF * pos_TipOnProbeRF
	auto femurIndex = m_VegaToolStorage->GetToolIndexByName("FemurRF");
	auto tibiaRFIndex = m_VegaToolStorage->GetToolIndexByName("TibiaRF");
	auto probeIndex = m_VegaToolStorage->GetToolIndexByName("ProbeRF");
	auto sawIndex = m_VegaToolStorage->GetToolIndexByName("SawRF");
	auto drillRFIndex = m_VegaToolStorage->GetToolIndexByName("DrillRF");

	// 获取探针针尖位置
	auto boneFemur = m_VegaToolStorage->GetToolByName("FemurRF");
	auto boneTibia = m_VegaToolStorage->GetToolByName("TibiaRF");
	auto toolProbe = m_VegaToolStorage->GetToolByName("ProbeRF");
	auto toolSaw = m_VegaToolStorage->GetToolByName("SawRF");
	auto toolDrill = m_VegaToolStorage->GetToolByName("DrillRF");

	// 取出探针针尖点在相机下的位置
	// 4x4齐次变换矩阵
	Eigen::Matrix4d T_Camera2FemurRF = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d T_Camera2TibiaRF = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d T_Camera2ProbeRF = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d T_Camera2SawRF = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d T_Camera2DrillRF = Eigen::Matrix4d::Identity();

	// 计算当前标定点位置
	Eigen::Vector4d CurrentFemurCheckPointOnImage;
	Eigen::Vector4d CurrentTibiaCheckPointOnImage;
	Eigen::Vector4d CurrentTipOnImage;
	Eigen::Vector4d CurrentSawPointsOnImage;
	Eigen::Vector4d CurrentDrillPointsOnImage;

	// 1. FemurRF
	// 股骨的存在主要计算截骨面夹角
	if (femurIndex != -1)
	{
		m_Controls.textBrowser_Action->append("Record Current FemurRF Pos:");
		mitk::NavigationData::Pointer pos_FemurRFOnCamera = m_VegaSource->GetOutput(probeIndex);
		if (pos_FemurRFOnCamera)
		{
			// 	构建T_Camera2TibiaRF
			Eigen::Vector3d position(
				pos_FemurRFOnCamera->GetPosition()[0], pos_FemurRFOnCamera->GetPosition()[1], pos_FemurRFOnCamera->GetPosition()[2]);
			Eigen::Quaterniond orientation(
				pos_FemurRFOnCamera->GetOrientation().r(), pos_FemurRFOnCamera->GetOrientation().x(),
				pos_FemurRFOnCamera->GetOrientation().y(), pos_FemurRFOnCamera->GetOrientation().z()
			);

			// 将3x3的旋转矩阵设置到4x4矩阵的左上角
			T_Camera2FemurRF.block<3, 3>(0, 0) = orientation.toRotationMatrix();
			// 将3x1的平移向量设置到4x4矩阵的右上角
			T_Camera2FemurRF.block<3, 1>(0, 3) = position;
		}

		// 计算Femur标定点在Camera下的坐标值
		// pos_FemurCheckPointOnCamera = T_Camera2FemurRF * pos_FemurCheckPointOnFemurRF
		Eigen::Vector4d CurrentFemurCheckPointOnCamera;
		CurrentFemurCheckPointOnCamera = T_Camera2FemurRF * m_FemurCheckPointOnFemurRF;
		m_CurrentFemurCheckPointOnCamera = CurrentFemurCheckPointOnCamera;
		m_Controls.textBrowser_Action->append("Added CurrentFemurCheckPointOnCamera: ("
			     + QString::number(m_CurrentFemurCheckPointOnCamera[0]) +
			", " + QString::number(m_CurrentFemurCheckPointOnCamera[1]) +
			", " + QString::number(m_CurrentFemurCheckPointOnCamera[2]) +
			", " + QString::number(m_CurrentFemurCheckPointOnCamera[3]) + ")"
		);

		// 计算Femur标定点在Image下的坐标
		// Pos_FemurCheckPointOnImage = T_FemurRF2M.inverse() * T_Camera2FemurRF.inverse() * pos_FemurCheckPointOnCamera
		CurrentFemurCheckPointOnImage = m_Metrix4dFemurRFToImage.inverse() * T_Camera2FemurRF.inverse() * m_CurrentFemurCheckPointOnCamera;
		m_CurrentFemurCheckPointOnImage = CurrentFemurCheckPointOnImage;
		m_Controls.textBrowser_Action->append("Added CurrentFemurCheckPointOnImage: ("
			     + QString::number(m_CurrentFemurCheckPointOnImage[0]) +
			", " + QString::number(m_CurrentFemurCheckPointOnImage[1]) +
			", " + QString::number(m_CurrentFemurCheckPointOnImage[2]) +
			", " + QString::number(m_CurrentFemurCheckPointOnImage[3]) + ")"
		);

		// 进行图像更新计算


		// 应用图像更新转化



	}
	

	// 2. TibiaRF
	// 胫骨
	if (femurIndex != -1)
	{
		m_Controls.textBrowser_Action->append("Record Current TibiaRF Pos:");
		mitk::NavigationData::Pointer pos_TibiaRFOnCamera = m_VegaSource->GetOutput(probeIndex);
		if (pos_TibiaRFOnCamera)
		{
			// 	构建T_Camera2TibiaRF
			Eigen::Vector3d position(
				pos_TibiaRFOnCamera->GetPosition()[0], pos_TibiaRFOnCamera->GetPosition()[1], pos_TibiaRFOnCamera->GetPosition()[2]);
			Eigen::Quaterniond orientation(
				pos_TibiaRFOnCamera->GetOrientation().r(), pos_TibiaRFOnCamera->GetOrientation().x(),
				pos_TibiaRFOnCamera->GetOrientation().y(), pos_TibiaRFOnCamera->GetOrientation().z()
			);

			// 将3x3的旋转矩阵设置到4x4矩阵的左上角
			T_Camera2TibiaRF.block<3, 3>(0, 0) = orientation.toRotationMatrix();
			// 将3x1的平移向量设置到4x4矩阵的右上角
			T_Camera2TibiaRF.block<3, 1>(0, 3) = position;
		}

		// 计算Tibia标定点在Camera下的坐标值
		// Pos_TibiaCheckPoint = T_Camera2TibiaRF * pos_TibiaCheckPointOnTibiaRF
		Eigen::Vector4d CurrentFemurCheckPointOnCamera;
		CurrentFemurCheckPointOnCamera = T_Camera2TibiaRF * m_TibiaCheckPointOnTibiaRF;
		m_CurrentTibiaCheckPointOnCamera = CurrentFemurCheckPointOnCamera;
		m_Controls.textBrowser_Action->append("Added CurrentTibiaCheckPoint: ("
			     + QString::number(m_CurrentTibiaCheckPointOnCamera[0]) +
			", " + QString::number(m_CurrentTibiaCheckPointOnCamera[1]) +
			", " + QString::number(m_CurrentTibiaCheckPointOnCamera[2]) +
			", " + QString::number(m_CurrentTibiaCheckPointOnCamera[3]) + ")"
		);

		// 计算Tibia标定点在Image下的坐标
		// Pos_TibiaCheckPointOnImage = T_TibiaRF2M.inverse() * T_Camera2TibiaRF.inverse() * pos_TibiaCheckPointOnCamera
		CurrentTibiaCheckPointOnImage = m_Metrix4dTibiaRFToImage.inverse() * T_Camera2TibiaRF.inverse() * m_CurrentTibiaCheckPointOnCamera;
		m_CurrentTibiaCheckPointOnImage = CurrentTibiaCheckPointOnImage;
		m_Controls.textBrowser_Action->append("Added CurrentFemurCheckPointOnImage: ("
			     + QString::number(m_CurrentTibiaCheckPointOnImage[0]) +
			", " + QString::number(m_CurrentTibiaCheckPointOnImage[1]) +
			", " + QString::number(m_CurrentTibiaCheckPointOnImage[2]) +
			", " + QString::number(m_CurrentTibiaCheckPointOnImage[3]) + ")"
		);

		// 进行图像更新计算


		// 应用图像更新转化



	}
	

	// 3. ProbeRF
	// 探针RF用于识别探针位置
	if (femurIndex != -1)
	{
		m_Controls.textBrowser_Action->append("Record Current ProbeRF Pos:");
		// 获取探针针尖位置
		mitk::Point3D pos_TipOnProbeRF = toolProbe->GetToolTipPosition();
		// 转化为4d
		Eigen::Vector4d pos_TipOnProbeRF4d(pos_TipOnProbeRF[0], pos_TipOnProbeRF[1], pos_TipOnProbeRF[2], 1.0);
		mitk::NavigationData::Pointer pos_ProbeRFOnCamera = m_VegaSource->GetOutput(probeIndex);
		if (pos_ProbeRFOnCamera)
		{
			// 	构建T_Camera2TibiaRF
			Eigen::Vector3d position(
				pos_ProbeRFOnCamera->GetPosition()[0], pos_ProbeRFOnCamera->GetPosition()[1], pos_ProbeRFOnCamera->GetPosition()[2]);
			Eigen::Quaterniond orientation(
				pos_ProbeRFOnCamera->GetOrientation().r(), pos_ProbeRFOnCamera->GetOrientation().x(),
				pos_ProbeRFOnCamera->GetOrientation().y(), pos_ProbeRFOnCamera->GetOrientation().z()
			);

			// 将3x3的旋转矩阵设置到4x4矩阵的左上角
			T_Camera2ProbeRF.block<3, 3>(0, 0) = orientation.toRotationMatrix();
			// 将3x1的平移向量设置到4x4矩阵的右上角
			T_Camera2ProbeRF.block<3, 1>(0, 3) = position;
		}
		cout << "pos_TipOnProbeRF: (" << pos_TipOnProbeRF[0] << " , " << pos_TipOnProbeRF[1] << " , " << pos_TipOnProbeRF[2] << ")" << endl;
		
		// 计算探针尖端在Camera下的坐标值
		// Pos_Tip = T_Camera2ProbeRF * pos_TipOnProbeRF
		Eigen::Vector4d pos_TipOnCamera = T_Camera2ProbeRF * pos_TipOnProbeRF4d;

		cout << "pos_TipOnCamera: (" << pos_TipOnCamera[0] << " , " << pos_TipOnCamera[1] << " , " << pos_TipOnCamera[2] << ")" << endl;

		// 计算ProbeTip在Camera下的坐标值
		// Pos_Tip = T_Camera2ProbeRF * pos_TipOnProbeRF
		pos_TipOnCamera = T_Camera2ProbeRF * pos_TipOnProbeRF4d;
		m_CurrentTipOnCamera = pos_TipOnCamera;
		m_Controls.textBrowser_Action->append("Added m_CurrentTipOnCamera: ("
			+ QString::number(m_CurrentTipOnCamera[0]) +
			", " + QString::number(m_CurrentTipOnCamera[1]) +
			", " + QString::number(m_CurrentTipOnCamera[2]) +
			", " + QString::number(m_CurrentTipOnCamera[3]) + ")"
		);

		// 计算ProbeTip在Image下的坐标
		// Pos_TipOnImage = T_TibiaRF2M.inverse() * T_Camera2TibiaRF.inverse() * pos_TibiaCheckPointOnCamera
		CurrentTipOnImage = m_Metrix4dTibiaRFToImage.inverse() * T_Camera2TibiaRF.inverse() * m_CurrentTipOnCamera;
		m_CurrentTipOnImage = CurrentTipOnImage;
		m_Controls.textBrowser_Action->append("Added CurrentTipOnImage: ("
			     + QString::number(m_CurrentTipOnImage[0]) +
			", " + QString::number(m_CurrentTipOnImage[1]) +
			", " + QString::number(m_CurrentTipOnImage[2]) +
			", " + QString::number(m_CurrentTipOnImage[3]) + ")"
		);

		// 进行图像更新计算， 利用上个位置和当前位置来进行更替
		// T_last2recent = N_pointsOnM_recent * N_pointsOnM_last.inverse()
		// 对于探针，则直接改变探针位置
		auto probeRf = GetDataStorage()->GetNamedNode("ProbeRF");
		auto probe = GetDataStorage()->GetNamedNode("Probe");
		mitk::Point3D pos_tip;
		pos_tip[0] = CurrentTipOnImage[0];
		pos_tip[1] = CurrentTipOnImage[1];
		pos_tip[2] = CurrentTipOnImage[2];

		// 应用图像更新转化
		//probeRf->GetData()->GetGeometry()->SetOrigin(pos_tip);
		probe->GetData()->GetGeometry()->SetOrigin(pos_tip);
	}


	// 4. SawRF
	// 用于截骨导航
	if (femurIndex != -1)
	{
		m_Controls.textBrowser_Action->append("Record Current SawRF Pos:");
		mitk::NavigationData::Pointer pos_SawRFOnCamera = m_VegaSource->GetOutput(probeIndex);
		if (pos_SawRFOnCamera)
		{
			// 	构建T_Camera2TibiaRF
			Eigen::Vector3d position(
				pos_SawRFOnCamera->GetPosition()[0], pos_SawRFOnCamera->GetPosition()[1], pos_SawRFOnCamera->GetPosition()[2]);
			Eigen::Quaterniond orientation(
				pos_SawRFOnCamera->GetOrientation().r(), pos_SawRFOnCamera->GetOrientation().x(),
				pos_SawRFOnCamera->GetOrientation().y(), pos_SawRFOnCamera->GetOrientation().z()
			);

			// 将3x3的旋转矩阵设置到4x4矩阵的左上角
			T_Camera2SawRF.block<3, 3>(0, 0) = orientation.toRotationMatrix();
			// 将3x1的平移向量设置到4x4矩阵的右上角
			T_Camera2SawRF.block<3, 1>(0, 3) = position;
		}

		// 计算Saw标定点在Camera下的坐标值
		// Pos_SawPoints = T_Camera2SawRF * pos_SawPointsOnTibiaRF
		for (int i = 0; i < m_SawPointsOnSawRF.size(); i++) {
			Eigen::Vector4d SawPoint_tmp = T_Camera2SawRF * m_SawPointsOnSawRF[i];
			// CurrentSawPoints.push_back(SawPoint);
			m_Controls.textBrowser_Action->append("Added CurrentSawPoint: ("
				+ QString::number(SawPoint_tmp[0]) +
				", " + QString::number(SawPoint_tmp[1]) +
				", " + QString::number(SawPoint_tmp[2]) +
				", " + QString::number(SawPoint_tmp[3]) + ")"
			);
			// 记录当前在相机下的位置
			m_CurrentSawPointsOnCamera.push_back(SawPoint_tmp);
			// 计算并记录在影像空间的位置
			Eigen::Vector4d SawPoint = m_Metrix4dTibiaRFToImage.inverse() * T_Camera2TibiaRF.inverse() * SawPoint_tmp;
			m_CurrentSawPointsOnImage.push_back(SawPoint);
		}
		
		// 进行图像更新计算， 利用上个位置和当前位置来进行更替
		// T_last2recent = N_pointsOnM_recent * N_pointsOnM_last.inverse()
		// 对于摆锯，去计算当前图像留存点到投影点的位置




		// 应用图像更新转化

		// 已知 A B C三点在自己坐标系下的坐标
		// D F 为摆锯边缘点，根据向量计算方程
		// AD = aAB + bAC => a = 403/628, b = -437/628
		// AF = cAB + dAC => c = 733/628, d = -131/628



	}
	

	// 5. DrillRF
	// 用于磨钻导航
	if (femurIndex != -1)
	{
		m_Controls.textBrowser_Action->append("Record Current DrillRF Pos:");
		mitk::NavigationData::Pointer pos_DrillRFOnCamera = m_VegaSource->GetOutput(probeIndex);
		if (pos_DrillRFOnCamera)
		{
			// 	构建T_Camera2TibiaRF
			Eigen::Vector3d position(
				pos_DrillRFOnCamera->GetPosition()[0], pos_DrillRFOnCamera->GetPosition()[1], pos_DrillRFOnCamera->GetPosition()[2]);
			Eigen::Quaterniond orientation(
				pos_DrillRFOnCamera->GetOrientation().r(), pos_DrillRFOnCamera->GetOrientation().x(),
				pos_DrillRFOnCamera->GetOrientation().y(), pos_DrillRFOnCamera->GetOrientation().z()
			);

			// 将3x3的旋转矩阵设置到4x4矩阵的左上角
			T_Camera2DrillRF.block<3, 3>(0, 0) = orientation.toRotationMatrix();
			// 将3x1的平移向量设置到4x4矩阵的右上角
			T_Camera2DrillRF.block<3, 1>(0, 3) = position;
		}

		// 计算Drill标定点在Camera下的坐标值
		// Pos_DrillPoints = T_Camera2DrillRF * pos_DrillPointsOnDrillRF
		for (int i = 0; i < m_DrillPointsOnDrillRF.size(); i++) {
			Eigen::Vector4d DrillPoint_tmp = T_Camera2DrillRF * m_DrillPointsOnDrillRF[i];
			// CurrentSawPoints.push_back(SawPoint);
			m_Controls.textBrowser_Action->append("Added CurrentDrillPoint: ("
				     + QString::number(DrillPoint_tmp[0]) +
				", " + QString::number(DrillPoint_tmp[1]) +
				", " + QString::number(DrillPoint_tmp[2]) +
				", " + QString::number(DrillPoint_tmp[3]) + ")"
			);

			// 记录当前在相机下的位置
			m_CurrentSawPointsOnCamera.push_back(DrillPoint_tmp);
			// 计算并记录在影像空间的位置
			Eigen::Vector4d DrillPoint = m_Metrix4dTibiaRFToImage.inverse() * T_Camera2TibiaRF.inverse() * DrillPoint_tmp;
			m_CurrentSawPointsOnImage.push_back(DrillPoint);

		}
		
		// 进行图像更新计算


		// 应用图像更新转化



	}
}



void HTONDI::GenerateRealTimeBoneSurface()
{
	/* 生成实时的截骨面
	1. 找到器械上的标定点，计算法向量和前端点位置，生成截骨面
	2. 计算实时面和规划面的夹角
	*/

	// 取出摆锯上的几个标定点位 => 计算摆锯的法向量
	mitk::Point3D PlanePoint1, PlanePoint2, PlanePoint3;
	
	if (saw_image) {
		auto SawPointSet = saw_image->GetLandmarks();
		// 由远及近
		PlanePoint1 = SawPointSet->GetPoint(0);
		PlanePoint2 = SawPointSet->GetPoint(1);
		PlanePoint3 = SawPointSet->GetPoint(2);
	}
	else
	{
		m_Controls.textBrowser_Action->append("saw_image Not init, stop record real time saw!");
		m_timer_saw->stop();
		return;
	}
	
	// 计算平面的法向量，根据摆锯上的三个标定点位计算平面的法向量
	Eigen::Vector3d normalVector;
	Eigen::Vector3d vector1 = Eigen::Vector3d(PlanePoint2[0] - PlanePoint1[0], PlanePoint2[1] - PlanePoint1[1], PlanePoint2[2] - PlanePoint1[2]);
	Eigen::Vector3d vector2 = Eigen::Vector3d(PlanePoint3[0] - PlanePoint1[0], PlanePoint3[1] - PlanePoint1[1], PlanePoint3[2] - PlanePoint1[2]);
	normalVector = vector1.cross(vector2).normalized();
	
	// 法向量
	double normal[3]
	{
		normalVector[0],
		normalVector[1],
		normalVector[2]
	};

	// 计算平面的中心点为 几何中心
	double origin[3]
	{
		(PlanePoint1[0] + PlanePoint1[0] + PlanePoint1[0]) / 3,
		(PlanePoint1[1] + PlanePoint1[1] + PlanePoint1[1]) / 3,
		(PlanePoint1[2] + PlanePoint1[2] + PlanePoint1[2]) / 3
	};

	// 创建一个初始的截骨平面，并定义位置和方向
	auto realTimeSawPlaneSource = vtkSmartPointer<vtkPlaneSource>::New();
	// 生成初始平面
	realTimeSawPlaneSource->SetOrigin(0, 0, 0);

	// 设定可视化截骨平面的大小，70*70
	realTimeSawPlaneSource->SetPoint1(0, 70, 0);
	realTimeSawPlaneSource->SetPoint2(70, 0, 0);
	realTimeSawPlaneSource->SetNormal(normal);//设置法向量

	// 将初始构建的截骨平面移动到计算出来的位置上来
	realTimeSawPlaneSource->SetCenter(origin);

	// 添加入库
	auto realTimeSawPlane = mitk::Surface::New();
	realTimeSawPlane->SetVtkPolyData(realTimeSawPlaneSource->GetOutput());

	// 创建实时平面
	auto tmpPlane = GetDataStorage()->GetNamedNode("realTimeCutPlane");
	if (tmpPlane)
	{
		tmpPlane->SetData(realTimeSawPlane);
	}
	else 
	{
		// 创建截骨平面DataNode对象
		auto planeNode = mitk::DataNode::New();
		planeNode->SetData(realTimeSawPlane);
		planeNode->SetColor(1.0, 0.0, 0.0);
		planeNode->SetOpacity(0.5);
		planeNode->SetName("realTimeCutPlane");
		GetDataStorage()->Add(planeNode);
	}
	
	// 计算锯片最前端的位置，计算截骨深度

	// 计算两个平面的夹角
	auto cutPlaneSource01 = GetDataStorage()->GetNamedNode("1st cut plane")->GetData();
}

void HTONDI::CalculateRealTimeCutAngle()
{
	/* 计算截骨面的夹角 */

}




bool HTONDI::OnStartAxialCutClicked()
{
	/* 启动水平截骨
		1. 改变摆锯截骨状态
		2. 计算实时 截骨深度
		3. 进行截骨保护

	====== 注意 ======
	持摆锯截骨时，摆锯锯片在克式针上，锯片应该与克式针平行
	这样才能保证深度计算正确
	*/

	// 开始水平截骨，启动摆锯，启动截骨导航
	m_Controls.textBrowser_Action->append("Action: Turn On Saw Power.");

	// 1. 改变导航状态
	m_RealtimeAngleCheck = false;
	m_RealtimeCutCheck = true;
	m_SawPower = true;

	return true;
}

bool HTONDI::OnStartStateAxialCutClicked()
{
	/* 静态截骨模拟，生成静态平面
		1. 获取静态模型表面标定点
		2. 生成初始的静态模型代表的截骨面
		3. 生成截骨平面初始点, 然后应用到当前相对与初始的法向量
		4. 计算当前截骨面夹角

	采用物体控制来模拟截骨过程
	*/
	m_Controls.textBrowser_Action->append("Action: Generate State Axial Cut Plane.");

	// 设置截骨方法的状态
	m_cutType = 1;

	// 1. 获取当前静态模型表面数据和标定点的当前位置
	mitk::Point3D PlanePoint1, PlanePoint2, PlanePoint3;
	auto SawPoints = GetDataStorage()->GetNamedNode("SawLandMarkPointSet");
	auto Saw = GetDataStorage()->GetNamedNode("Saw");

	// 由远及近
	if (SawPoints) {
		// 由远及近，取出三个点
		auto SawPointSet = dynamic_cast<mitk::PointSet*>(SawPoints->GetData());
		PlanePoint1 = SawPointSet->GetPoint(0);
		PlanePoint2 = SawPointSet->GetPoint(1);
		PlanePoint3 = SawPointSet->GetPoint(2);
		// 由远及近，取出三个点
		SawPoints->SetVisibility(true);
	}
	else
	{
		m_Controls.textBrowser_Action->append("SawLandMarkPointSet Not Found.");
		return false;
	}
	if (Saw) {
		// 由远及近，取出三个点
		Saw->SetVisibility(true);
	}
	else
	{
		m_Controls.textBrowser_Action->append("Saw model Not Found.");
		return false;
	}

	// 计算当前摆锯平面的 法向量 normalSaw ，根据摆锯上的三个标定点位计算平面的法向量
	Eigen::Vector3d normalVector;
	Eigen::Vector3d vector1 = Eigen::Vector3d(PlanePoint2[0] - PlanePoint1[0], PlanePoint2[1] - PlanePoint1[1], PlanePoint2[2] - PlanePoint1[2]);
	Eigen::Vector3d vector2 = Eigen::Vector3d(PlanePoint3[0] - PlanePoint1[0], PlanePoint3[1] - PlanePoint1[1], PlanePoint3[2] - PlanePoint1[2]);
	normalVector = vector1.cross(vector2).normalized();

	// 法向量
	double normalSaw[3]
	{
		normalVector[0],
		normalVector[1],
		normalVector[2]
	};

	// 计算平面的中心点为 几何中心, 将其位置移动到摆锯最前端
	double originSaw[3]
	{
		(PlanePoint1[0] + PlanePoint2[0] + PlanePoint3[0]) / 3,
		(PlanePoint1[1] + PlanePoint2[1] + PlanePoint3[1]) / 3,
		(PlanePoint1[2] + PlanePoint2[2] + PlanePoint3[2]) / 3
	};


	// 2. 生成摆锯截骨平面
	// 创建一个初始的截骨平面，并定义位置和方向
	auto realTimeSawPlaneSource = vtkSmartPointer<vtkPlaneSource>::New();

	// 生成初始平面
	//realTimeSawPlaneSource->SetOrigin(0, 0, 0);
	
	// 设定可视化截骨平面的大小，70*70
	realTimeSawPlaneSource->SetPoint1(0, 70, 0);
	realTimeSawPlaneSource->SetPoint2(70, 0, 0);


	// 将初始构建的截骨平面移动到计算出来的位置上来
	realTimeSawPlaneSource->SetCenter(originSaw);
	realTimeSawPlaneSource->SetNormal(normalSaw);

	// 图像更新
	realTimeSawPlaneSource->Update();

	// 添加入库
	auto realTimeSawPlane = mitk::Surface::New();
	realTimeSawPlane->SetVtkPolyData(realTimeSawPlaneSource->GetOutput());

	// 创建实时平面
	auto tmpPlane = GetDataStorage()->GetNamedNode("StateCutPlane01");
	if (tmpPlane)
	{
		tmpPlane->SetData(realTimeSawPlane);
		m_Controls.textBrowser_Action->append("Plane updated.");
	}
	else
	{
		// 创建截骨平面DataNode对象
		auto planeNode = mitk::DataNode::New();
		planeNode->SetData(realTimeSawPlane);
		planeNode->SetColor(1.0, 1.0, 0.0);
		planeNode->SetOpacity(0.5);
		planeNode->SetName("StateCutPlane01");

		// 添加到库
		GetDataStorage()->Add(planeNode);
		m_Controls.textBrowser_Action->append("Plane created.");
	}

	// 计算锯片最前端的位置，计算截骨深度

	// 3. 生成截骨面最前端点，同术前规划方法

	// 可视化节点
	auto tmpNodes = GetDataStorage()->GetNamedNode("pointSetInRealPlaneAxial");
	if (tmpNodes) {
		//GetDataStorage()->Remove(tmpNodes);
		/* 如果不是第一次生成摆锯平面
			1. 取出当前的原点和法向量
			2. 依据存储的相对位置来计算实际位置
			3. 更新节点数据
		*/
		mitk::Point3D point0;
		mitk::Point3D point1;
		mitk::Point3D point2;
		mitk::Point3D point3;
		mitk::Point3D point4;
		mitk::Point3D planeCenterPoint;//水平面中心点
		// 使用 CalculateActualPoints 函数计算实际位置
		std::vector<mitk::Point3D> actualPoints = CalculateActualPoints(m_PointsOnSaw, normalVector, Eigen::Vector3d(originSaw[0], originSaw[1], originSaw[2]));
		mitkPointSetRealTime->SetPoint(0, point0);//第一截骨面末端中点标记
		mitkPointSetRealTime->SetPoint(1, point1);
		mitkPointSetRealTime->SetPoint(2, point2);
		mitkPointSetRealTime->SetPoint(3, point3);
		mitkPointSetRealTime->SetPoint(4, point4);
		// 更新数据
		tmpNodes->SetData(mitkPointSetRealTime);
	}
	else 
	{
		/* 如果是第一次生成摆锯的平面
			1. 初始化为水平位置(依据模型初始位置)
			2. 计算并存储平面关键点在摆锯平面下的位置
			3. 然后进行可视化更新

		注：这里默认初始加载的摆锯是水平
		*/

		/* 1. 水平面位置初始化 */
		// 定义截骨末端合页点的初始坐标
		mitk::Point3D point0;
		mitk::Point3D point1;
		mitk::Point3D point2;
		mitk::Point3D point3;
		mitk::Point3D point4;
		mitk::Point3D planeCenterPoint;//水平面中心点

		// 点 1 - 前端中点
		point0[0] = originSaw[0] - 35;
		point0[1] = originSaw[1];
		point0[2] = originSaw[2];
		// 点 2 - 前端左侧
		point1[0] = originSaw[0] - 35;
		point1[1] = originSaw[1] - 35;
		point1[2] = originSaw[2];
		// 点 3 - 前端右侧
		point2[0] = originSaw[0] - 35;
		point2[1] = originSaw[1] + 35;
		point2[2] = originSaw[2];
		// 点 4 - 末端左侧
		point3[0] = originSaw[0] + 35;
		point3[1] = originSaw[1] - 35;
		point3[2] = originSaw[2];
		// 点 5 - 末端中点 
		point4[0] = originSaw[0] + 35;
		point4[1] = originSaw[1];
		point4[2] = originSaw[2];
		// 点 6 - 设置原点
		planeCenterPoint[0] = originSaw[0];
		planeCenterPoint[1] = originSaw[1];
		planeCenterPoint[2] = originSaw[2];

		// 计算这些点相对于摆锯坐标系的位置
		m_PointsOnSaw = CalculateRelativePoints({ point0, point1, point2, point3, point4 }, 
			normalVector, Eigen::Vector3d(originSaw[0], originSaw[1], originSaw[2]));

		// 将坐标添加到mitk::PointSet中
		// 用于上升截骨面的旋转计算
		mitkPointSetRealTime->InsertPoint(0, point0);//第一截骨面末端中点标记
		mitkPointSetRealTime->InsertPoint(1, point1);
		mitkPointSetRealTime->InsertPoint(2, point2);
		mitkPointSetRealTime->InsertPoint(3, point3);
		mitkPointSetRealTime->InsertPoint(4, point4);
		mitkPointSetRealTime->InsertPoint(5, planeCenterPoint);//平面原点
		// 否则创建新的平面
		mitk::DataNode::Pointer pointSetInPlaneCutPlane = mitk::DataNode::New();
		pointSetInPlaneCutPlane->SetName("pointSetInRealPlaneAxial");
		// 红色，大小 5.0
		pointSetInPlaneCutPlane->SetColor(1.0, 0.0, 0.0);
		pointSetInPlaneCutPlane->SetData(mitkPointSetRealTime);
		pointSetInPlaneCutPlane->SetFloatProperty("pointsize", 5.0);
		GetDataStorage()->Add(pointSetInPlaneCutPlane);
	}
	


	// 4. 计算两个平面的夹角

	// 提取水平截骨面的法向量
	auto cutPlaneSource01 = dynamic_cast<mitk::Surface*>(GetDataStorage()->GetNamedNode("1st cut plane")->GetData());
	auto cutPlanePolyData01 = cutPlaneSource01->GetVtkPolyData();

	Eigen::Vector3d normalPlane01;
	normalPlane01 = ExtractNormalFromPlane("1st cut plane");

	// 输出法向量情况
	cout << "normalPlane: " << " " << normalPlane01[0] << " " << normalPlane01[1] << " " << normalPlane01[2] << endl;
	cout << "normalSaw  : " << " " << normalSaw[0] << " " << normalSaw[1] << " " << normalSaw[2] << endl;

	//cout << "CenterPlane: " << " " << centerPlane01[0] << " " << centerPlane01[1] << " " << centerPlane01[2] << endl;
	cout << "CenterSaw  : " << " " << originSaw[0] << " " << originSaw[1] << " " << originSaw[2] << endl;

	// 计算两个方向向量之间的夹角
	double dotProduct = normalPlane01[0] * normalSaw[0] + normalPlane01[1] * normalSaw[1] + normalPlane01[2] * normalSaw[2]; // 点积

	// 计算向量长度
	double magnitude1 = sqrt(normalPlane01[0] * normalPlane01[0] + normalPlane01[1] * normalPlane01[1] + normalPlane01[2] * normalPlane01[2]);
	double magnitude2 = sqrt(normalSaw[0] * normalSaw[0] + normalSaw[1] * normalSaw[1] + normalSaw[2] * normalSaw[2]);

	// 计算夹角的余弦值
	double cosAngle = dotProduct / (magnitude1 * magnitude2);

	// 使用反余弦函数计算夹角（弧度）
	double angleInRadians = acos(cosAngle);

	// 将弧度转换为度数，保留1位小数
	double angleInDegrees = round(angleInRadians * (180.0 / M_PI) * 10) / 10;
	if (angleInDegrees > 90)
	{
		angleInDegrees = 180.0 - angleInDegrees;
	}

	// 输出夹角到实际的位置
	m_Controls.textBrowser_AxialCut->append("Real time Angle Miss: " + QString::number(angleInDegrees));

	GetDataStorage()->Modified();
	mitk::RenderingManager::GetInstance()->RequestUpdateAll();
	return true;
}

bool HTONDI::OnCheckStateCutClicked()
{
	/* 确定水平截骨面位置	*/
	m_Controls.textBrowser_Action->append("Action: Check State Cut Plane.");
	m_cutType = -1;
	return true;
}


bool HTONDI::OnStartSagGuideClicked()
{
	/* 确定水平截骨面位置	*/
	m_Controls.textBrowser_Action->append("Action: Check State Cut Plane.");
	m_cutType = 2;

	// 1. 显示规划的截骨面 + 显示实时截骨面位置
	auto preCutPlane01 = GetDataStorage()->GetNamedNode("2nd cut plane");
	auto realTimeSaw = GetDataStorage()->GetNamedNode("Saw");

	// 检测数据存在，然后打开截骨导航
	if (preCutPlane01 && realTimeSaw)
	{
		preCutPlane01->SetVisibility(true);
		realTimeSaw->SetVisibility(true);
	}
	else
	{
		m_Controls.textBrowser_Action->append("Cut plane or Saw model Not Found!");
		return false;
	}

	// 对Saw生成实时截骨平面
	if (m_HTOSawUpdateTimer == nullptr)
	{
		m_HTOSawUpdateTimer = new QTimer(this);
	}
	m_Controls.textBrowser_Action->append("Generate Real time Cut plane");

	disconnect(m_HTOSawUpdateTimer, SIGNAL(timeout()), this, SLOT(UpdateHTOSaw02()));
	connect(m_HTOSawUpdateTimer, SIGNAL(timeout()), this, SLOT(UpdateHTOSaw02()));
	m_HTOSawUpdateTimer->start(100);




	return true;
}

bool HTONDI::OnStartStateSagCutClicked()
{
	/* 静态截骨模拟，生成静态平面
		1. 获取静态模型表面标定点
		2. 生成初始的静态模型代表的截骨面
		3. 生成截骨平面初始点
		4. 计算当前截骨面夹角

	采用物体控制来模拟截骨过程
	*/

	m_Controls.textBrowser_Action->append("Action: Generate State Sag Cut Plane.");

	// 设置截骨方法的状态
	m_cutType = 2;

	// 1. 获取静态模型表面的标定点
	mitk::Point3D PlanePoint1, PlanePoint2, PlanePoint3;
	auto SawPoints = GetDataStorage()->GetNamedNode("SawLandMarkPointSet");
	auto Saw = GetDataStorage()->GetNamedNode("Saw");

	// 由远及近
	if (SawPoints) {
		// 由远及近，取出三个点
		auto SawPointSet = dynamic_cast<mitk::PointSet*>(SawPoints->GetData());
		PlanePoint1 = SawPointSet->GetPoint(0);
		PlanePoint2 = SawPointSet->GetPoint(1);
		PlanePoint3 = SawPointSet->GetPoint(2);
		// 由远及近，取出三个点
		SawPoints->SetVisibility(true);
	}
	else
	{
		m_Controls.textBrowser_Action->append("SawLandMarkPointSet Not Found.");
		return false;
	}
	if (Saw) {
		// 由远及近，取出三个点
		Saw->SetVisibility(true);
	}
	else
	{
		m_Controls.textBrowser_Action->append("Saw model Not Found.");
		return false;
	}

	// 计算摆锯平面的 法向量 normalSaw ，根据摆锯上的三个标定点位计算平面的法向量
	Eigen::Vector3d normalVector;
	Eigen::Vector3d vector1 = Eigen::Vector3d(PlanePoint2[0] - PlanePoint1[0], PlanePoint2[1] - PlanePoint1[1], PlanePoint2[2] - PlanePoint1[2]);
	Eigen::Vector3d vector2 = Eigen::Vector3d(PlanePoint3[0] - PlanePoint1[0], PlanePoint3[1] - PlanePoint1[1], PlanePoint3[2] - PlanePoint1[2]);
	normalVector = vector1.cross(vector2).normalized();

	// 法向量
	double normalSaw[3]
	{
		normalVector[0],
		normalVector[1],
		normalVector[2]
	};

	// 计算平面的中心点为 几何中心, 将其位置移动到摆锯最前端
	double originSaw[3]
	{
		(PlanePoint1[0] + PlanePoint1[0] + PlanePoint1[0]) / 3 + 25,
		(PlanePoint1[1] + PlanePoint1[1] + PlanePoint1[1]) / 3,
		(PlanePoint1[2] + PlanePoint1[2] + PlanePoint1[2]) / 3
	};


	// 2. 生成摆锯截骨平面
	// 创建一个初始的截骨平面，并定义位置和方向
	auto realTimeSawPlaneSource = vtkSmartPointer<vtkPlaneSource>::New();

	// 生成初始平面
	realTimeSawPlaneSource->SetOrigin(0, 0, 0);

	// 设定可视化截骨平面的大小，70*70
	realTimeSawPlaneSource->SetPoint1(0, 70, 0);
	realTimeSawPlaneSource->SetPoint2(70, 0, 0);
	realTimeSawPlaneSource->SetNormal(0, 0, 1);//设置法向量

	// 将初始构建的截骨平面移动到计算出来的位置上来
	realTimeSawPlaneSource->SetCenter(originSaw);
	realTimeSawPlaneSource->SetNormal(normalSaw);
	

	// 图像更新
	realTimeSawPlaneSource->Update();

	// 添加入库
	auto realTimeSawPlane = mitk::Surface::New();
	realTimeSawPlane->SetVtkPolyData(realTimeSawPlaneSource->GetOutput());

	// 创建实时平面
	auto tmpPlane = GetDataStorage()->GetNamedNode("StateCutPlane02");
	if (tmpPlane)
	{
		tmpPlane->SetData(realTimeSawPlane);
		m_Controls.textBrowser_Action->append("Plane updated.");
	}
	else
	{
		// 创建截骨平面DataNode对象
		auto planeNode = mitk::DataNode::New();
		planeNode->SetData(realTimeSawPlane);
		planeNode->SetColor(1.0, 1.0, 0.0);
		planeNode->SetOpacity(0.5);
		planeNode->SetName("StateCutPlane02");

		// 添加到库
		GetDataStorage()->Add(planeNode);
		m_Controls.textBrowser_Action->append("Plane created.");
	}

	// 计算锯片最前端的位置，计算截骨深度

	// 3. 生成截骨面最前端点，同术前规划方法

	// 定义截骨末端合页点的初始坐标
	mitk::Point3D point0;
	mitk::Point3D point1;
	mitk::Point3D point2;
	mitk::Point3D point3;
	mitk::Point3D point4;
	mitk::Point3D planeCenterPoint;//水平面中心点

	// 点 1 - 前端中点
	point0[0] = originSaw[0] - 35;
	point0[1] = originSaw[1];
	point0[2] = originSaw[2];
	// 点 2 - 前端左侧
	point1[0] = originSaw[0] - 35;
	point1[1] = originSaw[1] - 35;
	point1[2] = originSaw[2];
	// 点 3 - 前端右侧
	point2[0] = originSaw[0] - 35;
	point2[1] = originSaw[1] + 35;
	point2[2] = originSaw[2];
	// 点 4 - 末端左侧
	point3[0] = originSaw[0] + 35;
	point3[1] = originSaw[1] - 35;
	point3[2] = originSaw[2];
	// 点 5 - 末端中点 
	point4[0] = originSaw[0] + 35;
	point4[1] = originSaw[1];
	point4[2] = originSaw[2];

	// 点 6 - 设置原点
	planeCenterPoint[0] = originSaw[0];
	planeCenterPoint[1] = originSaw[1];
	planeCenterPoint[2] = originSaw[2];


	// 将坐标添加到mitk::PointSet中
	// 用于上升截骨面的旋转计算
	mitkPointSetRealTime->InsertPoint(0, point0);//第一截骨面末端中点标记
	mitkPointSetRealTime->InsertPoint(1, point1);
	mitkPointSetRealTime->InsertPoint(2, point2);
	mitkPointSetRealTime->InsertPoint(3, point3);
	mitkPointSetRealTime->InsertPoint(4, point4);
	mitkPointSetRealTime->InsertPoint(4, planeCenterPoint);//平面原点

	// 可视化节点
	auto tmpNodes = GetDataStorage()->GetNamedNode("pointSetInRealPlaneSag");
	if (tmpNodes) {
		//GetDataStorage()->Remove(tmpNodes);
		//// 如果已经创建则更新其信息
		tmpNodes->SetData(mitkPointSetRealTime);
	}
	else
	{
		// 否则创建新的平面
		mitk::DataNode::Pointer pointSetInPlaneCutPlane = mitk::DataNode::New();
		pointSetInPlaneCutPlane->SetName("pointSetInRealPlaneSag");
		// 红色，大小 5.0
		pointSetInPlaneCutPlane->SetColor(1.0, 0.0, 0.0);
		pointSetInPlaneCutPlane->SetData(mitkPointSetRealTime);
		pointSetInPlaneCutPlane->SetFloatProperty("pointsize", 5.0);
		GetDataStorage()->Add(pointSetInPlaneCutPlane);
	}



	// 4. 计算两个平面的夹角

	// 提取水平截骨面的法向量
	auto cutPlaneSource01 = dynamic_cast<mitk::Surface*>(GetDataStorage()->GetNamedNode("1st cut plane")->GetData());
	auto cutPlanePolyData01 = cutPlaneSource01->GetVtkPolyData();

	Eigen::Vector3d normalPlane01;
	normalPlane01 = ExtractNormalFromPlane("1st cut plane");

	// 输出法向量情况
	cout << "normalPlane: " << " " << normalPlane01[0] << " " << normalPlane01[1] << " " << normalPlane01[2] << endl;
	cout << "normalSaw  : " << " " << normalSaw[0] << " " << normalSaw[1] << " " << normalSaw[2] << endl;

	//cout << "CenterPlane: " << " " << centerPlane01[0] << " " << centerPlane01[1] << " " << centerPlane01[2] << endl;
	cout << "CenterSaw  : " << " " << originSaw[0] << " " << originSaw[1] << " " << originSaw[2] << endl;

	// 计算两个方向向量之间的夹角
	double dotProduct = normalPlane01[0] * normalSaw[0] + normalPlane01[1] * normalSaw[1] + normalPlane01[2] * normalSaw[2]; // 点积

	// 计算向量长度
	double magnitude1 = sqrt(normalPlane01[0] * normalPlane01[0] + normalPlane01[1] * normalPlane01[1] + normalPlane01[2] * normalPlane01[2]);
	double magnitude2 = sqrt(normalSaw[0] * normalSaw[0] + normalSaw[1] * normalSaw[1] + normalSaw[2] * normalSaw[2]);

	// 计算夹角的余弦值
	double cosAngle = dotProduct / (magnitude1 * magnitude2);

	// 使用反余弦函数计算夹角（弧度）
	double angleInRadians = acos(cosAngle);

	// 将弧度转换为度数，保留1位小数
	double angleInDegrees = round(angleInRadians * (180.0 / M_PI) * 10) / 10;
	if (angleInDegrees > 90)
	{
		angleInDegrees = 180.0 - angleInDegrees;
	}

	// 输出夹角到实际的位置
	m_Controls.textBrowser_SagCut->append("Real time Angle Miss: " + QString::number(angleInDegrees));

	GetDataStorage()->Modified();
	mitk::RenderingManager::GetInstance()->RequestUpdateAll();
	return true;


	return true;
}

bool HTONDI::OnStartSagCutClicked()
{
	/* 启动上升截骨
	1. 改变摆锯截骨状态
	2. 计算实时 截骨深度
	3. 进行截骨保护
	*/

	m_Controls.textBrowser_Action->append("Action: Check Drill Node.");

	return true;
}

bool HTONDI::OnStartDrillGuideClicked()
{
	/* 开始磨钻导航
	1. 生成实时磨钻位置
	2. 计算 实时钻孔位置 和 规划钻孔位置 差距
	*/

	m_Controls.textBrowser_Action->append("Action: Check Drill Node.");

	// 1. 显示规划的截骨面 + 显示实时截骨面位置
	auto realTimeDrill = GetDataStorage()->GetNamedNode("Drill");

	// 检测数据存在，然后打开截骨导航
	if (realTimeDrill)
	{
		realTimeDrill->SetVisibility(true);
	}
	else
	{
		m_Controls.textBrowser_Action->append("Cut Drill model Not Found!");
		return false;
	}

	// 对Drill生成实时磨钻圆柱体
	if (m_HTODrillUpdateTimer == nullptr)
	{
		m_HTODrillUpdateTimer = new QTimer(this);
	}
	m_Controls.textBrowser_Action->append("Generate Real time Drill");

	disconnect(m_HTODrillUpdateTimer, SIGNAL(timeout()), this, SLOT(UpdateHTODrill02()));
	connect(m_HTODrillUpdateTimer, SIGNAL(timeout()), this, SLOT(UpdateHTODrill02()));
	m_HTODrillUpdateTimer->start(100);

	return true;
}

bool HTONDI::OnStartDrillHoleClicked()
{
	/* 启动钻孔
	1. 改变磨钻器械状态
	2. 计算实时 钻孔深度
	3. 进行钻孔保护
	*/

	m_Controls.textBrowser_Action->append("Action: Check Drill Node.");

	return true;
}

bool HTONDI::OnStartStateDrillHoleClicked()
{
	/* 启动钻孔
	1. 改变磨钻器械状态
	2. 计算实时 钻孔深度
	3. 进行钻孔保护
	*/

	m_Controls.textBrowser_Action->append("Action: Check Drill Node.");

	return true;
}



// 术后验证
bool HTONDI::OnShowResClicked()
{
	// 开始钻孔
	m_Controls.textBrowser_Action->append("Action: Check Drill Node.");

	return true;
}


bool HTONDI::OnUnshowResClicked()
{
	// 开始钻孔
	m_Controls.textBrowser_Action->append("Action: Check Drill Node.");

	return true;
}


bool HTONDI::OnCaculateErrorClicked()
{
	// 开始钻孔
	m_Controls.textBrowser_Action->append("Action: Check Drill Node.");

	return true;
}

// 截骨效应
bool HTONDI::GetRealTimeIntersectionLine(const std::string& cutPlaneName, const std::string& surfaceName)
{
	/* 实时获取截骨线
		原理: 利用截骨平面和骨表面进行切割计算，得到一组点云的交集
	*/
	cout << "GetRealTimeIntersectionLine 01" << endl;
	auto cutPlaneNode = GetDataStorage()->GetNamedNode(cutPlaneName);
	auto surfaceNode = GetDataStorage()->GetNamedNode(surfaceName);

	if (cutPlaneNode == nullptr || surfaceNode == nullptr)
	{
		m_Controls.textBrowser_Action->append(QString::fromStdString(cutPlaneName) + " or " + QString::fromStdString(surfaceName) + "is not ready!");
		return false;
	}

	// 使用tmp的目的是获取最新的数据而非初始数据

	// tmp-切割平面
	auto cutPlaneSource = dynamic_cast<mitk::Surface*>(cutPlaneNode->GetData());
	auto tmpcutPlaneSource_vtk = cutPlaneSource->GetVtkPolyData();
	// 复制切割平面
	vtkNew<vtkTransform> cutPlaneTransform;
	cutPlaneTransform->SetMatrix(cutPlaneSource->GetGeometry()->GetVtkMatrix());
	vtkNew<vtkTransformFilter> cutPlaneTransformFilter;
	cutPlaneTransformFilter->SetTransform(cutPlaneTransform);
	cutPlaneTransformFilter->SetInputData(tmpcutPlaneSource_vtk);
	cutPlaneTransformFilter->Update();
	// 得到新的数据
	vtkNew<vtkPolyData> tmpcutPlaneCopy_vtk;
	tmpcutPlaneCopy_vtk->DeepCopy(cutPlaneTransformFilter->GetPolyDataOutput());


	// tmp-被切割物体
	auto surfaceSource = dynamic_cast<mitk::Surface*>(surfaceNode->GetData());
	auto tmpsurfaceSource_vtk = surfaceSource->GetVtkPolyData();
	// 复制被切割物体表面
	vtkNew<vtkTransform> surfaceTransform;
	surfaceTransform->SetMatrix(surfaceSource->GetGeometry()->GetVtkMatrix());
	vtkNew<vtkTransformFilter> surfaceTransformFilter;
	surfaceTransformFilter->SetTransform(surfaceTransform);
	surfaceTransformFilter->SetInputData(tmpsurfaceSource_vtk);
	surfaceTransformFilter->Update();
	// 得到新的数据
	vtkNew<vtkPolyData> tmpSurfaceCopy_vtk;
	tmpSurfaceCopy_vtk->DeepCopy(surfaceTransformFilter->GetPolyDataOutput());


	// 获取切割平面的法向量和中心点
	double cutPlaneNormal[3];
	double cutPlaneCenter[3];
	GetPlaneProperty(tmpcutPlaneCopy_vtk, cutPlaneNormal, cutPlaneCenter);

	cout << "GetRealTimeIntersectionLine 04" << endl;
	// 使用获取的法向量和原点创建一个vtkPlane对象
	vtkSmartPointer<vtkPlane> cutPlane = vtkSmartPointer<vtkPlane>::New();
	cutPlane->SetNormal(cutPlaneNormal);
	cutPlane->SetOrigin(cutPlaneCenter);

	cout << "GetRealTimeIntersectionLine 05" << endl;
	// 然后将这个cutPlane设置给vtkCutter
	vtkSmartPointer<vtkCutter> cutter_plane = vtkSmartPointer<vtkCutter>::New();
	cutter_plane->SetCutFunction(cutPlane);
	//设置输入数据
	cutter_plane->SetInputData(tmpSurfaceCopy_vtk);
	//更新获取交线
	cutter_plane->Update();
	vtkSmartPointer<vtkPolyData> intersectionLine = cutter_plane->GetOutput();

	cout << "GetRealTimeIntersectionLine 06" << endl;
	// 取得无限扩展平面上的截骨面交线，并提取其水平方向上的最值点, 存储到全局变量中
	RealTimeTraverseIntersectionLines(intersectionLine, m_CutIntersectionMinPos, m_CutIntersectionMaxPos);

	// 可以进行可视化
	mitk::Point3D point_min, point_max;
	point_min[0] = m_CutIntersectionMinPos[0];
	point_min[1] = m_CutIntersectionMinPos[1];
	point_min[2] = m_CutIntersectionMinPos[2];
	point_max[0] = m_CutIntersectionMaxPos[0];
	point_max[1] = m_CutIntersectionMaxPos[1];
	point_max[2] = m_CutIntersectionMaxPos[2];
	mitk::PointSet::Pointer CutIntersectionPointSet = mitk::PointSet::New();
	CutIntersectionPointSet->InsertPoint(0, point_min);
	CutIntersectionPointSet->InsertPoint(1, point_max);

	// 删除上次生成的点
	auto tmpNodes = GetDataStorage()->GetNamedNode("CutIntersectionPoints");
	if (tmpNodes) {
		GetDataStorage()->Remove(tmpNodes);
	}

	mitk::DataNode::Pointer pointSetInPlaneCutPlane = mitk::DataNode::New();
	pointSetInPlaneCutPlane->SetName("CutIntersectionPoints");
	// 红色，大小 5.0
	pointSetInPlaneCutPlane->SetColor(0.0, 0.0, 1.0);
	pointSetInPlaneCutPlane->SetData(CutIntersectionPointSet);
	pointSetInPlaneCutPlane->SetFloatProperty("pointsize", 3.0);
	GetDataStorage()->Add(pointSetInPlaneCutPlane);

	return true;
}

void HTONDI::RealTimeTraverseIntersectionLines(vtkSmartPointer<vtkPolyData> intersectionLine, double pos_min[3], double pos_max[3])
{
	// 获取截骨线最左最右侧点

	cout << "RealTimeTraverseIntersectionLines 01" << endl;

	// 获取点集
	vtkSmartPointer<vtkPoints> points = intersectionLine->GetPoints();

	// 获取点的数量
	int numberOfPoints = points->GetNumberOfPoints();
	double minX = std::numeric_limits<double>::max();
	int minXIndex = -1;
	double maxX = -std::numeric_limits<double>::max(); // 初始化为小于可能的最小x值
	int maxXIndex = -1;

	//double pos_min[3];
	//double pos_max[3];

	// 这里提取的是在水平方向上的最大值点
	std::cout << "Intersection Line Points:" << std::endl;
	for (int i = 0; i < numberOfPoints; ++i)
	{
		double point[3];
		points->GetPoint(i, point); 

		//检查当前点的x坐标是否小于已知的最小x值, 记录最小x值对应的点的索引
		if (point[0] < minX)
		{
			minX = point[0]; 
			minXIndex = i;
		}
		if (point[0] > maxX)
		{
			maxX = point[0];
			maxXIndex = i;
		}
	}
	if (minXIndex != -1 && maxXIndex != -1)
	{
		pos_min[0] = points->GetPoint(minXIndex)[0];
		pos_min[1] = points->GetPoint(minXIndex)[1];
		pos_min[2] = points->GetPoint(minXIndex)[2];

		pos_max[0] = points->GetPoint(maxXIndex)[0];
		pos_max[1] = points->GetPoint(maxXIndex)[1];
		pos_max[2] = points->GetPoint(maxXIndex)[2];

		std::cout << "The point with the smallest x-coordinate is Point " << minXIndex << ": ("
			<< minX << ", " << points->GetPoint(minXIndex)[1] << ", " << points->GetPoint(minXIndex)[2] << ")" << std::endl;

		std::cout << "The point with the largest x-coordinate is Point " << maxXIndex << ": ("
			<< maxX << ", " << points->GetPoint(maxXIndex)[1] << ", " << points->GetPoint(maxXIndex)[2] << ")" << std::endl;
	}
	else
	{
		std::cout << "No points were processed." << std::endl;
		return;
	}

}


// 替换磨钻钻头模型
bool HTONDI::OnChangeKeShiPinClicked()
{
	auto tmpKeShiPin = GetDataStorage()->GetNamedNode("Drill");
	auto tmpKeShiPin_Points = GetDataStorage()->GetNamedNode("DrillLandMarkPointSet");

	auto tmpLinkPin = GetDataStorage()->GetNamedNode("LinkPin");
	auto tmpLinkPin_Points = GetDataStorage()->GetNamedNode("LinkPinLandMarkPointSet");

	if (tmpKeShiPin)
	{
		tmpKeShiPin->SetVisibility(true);
		tmpKeShiPin_Points->SetVisibility(true);
	}

	if (tmpLinkPin)
	{
		tmpLinkPin->SetVisibility(false);
		tmpLinkPin_Points->SetVisibility(false);
	}

	Drilltype = 0;

	mitk::RenderingManager::GetInstance()->RequestUpdateAll();

	return true;
}


// 替换磨钻钻头模型
bool HTONDI::OnChangeLinkPinClicked()
{
	auto tmpKeShiPin = GetDataStorage()->GetNamedNode("Drill");
	auto tmpKeShiPin_Points = GetDataStorage()->GetNamedNode("DrillLandMarkPointSet");

	auto tmpLinkPin = GetDataStorage()->GetNamedNode("LinkPin");
	auto tmpLinkPin_Points = GetDataStorage()->GetNamedNode("LinkPinLandMarkPointSet");

	if (tmpKeShiPin)
	{
		tmpKeShiPin->SetVisibility(false);
		tmpKeShiPin_Points->SetVisibility(false);
	}

	if (tmpLinkPin)
	{
		tmpLinkPin->SetVisibility(true);
		tmpLinkPin_Points->SetVisibility(true);
	}
	Drilltype = 1;

	mitk::RenderingManager::GetInstance()->RequestUpdateAll();

	return true;
}


// 设置目标点的位置
bool HTONDI::OnSetKeShi01Clicked()
{
	m_Controls.textBrowser_Action->append("Action: Guide Keshi01 Point.");
	Destin_Tail[0] = KeShikPinPos_Set[0][0];
	Destin_Tail[1] = KeShikPinPos_Set[0][1];
	Destin_Tail[2] = KeShikPinPos_Set[0][2];
	// 设置数值
	m_Controls.label_set_x->setText("x: " + QString::number(Destin_Tail[0]));
	m_Controls.label_set_y->setText("y: " + QString::number(Destin_Tail[1]));
	m_Controls.label_set_z->setText("z: " + QString::number(Destin_Tail[2]));
	return true;
}

bool HTONDI::OnSetKeShi02Clicked()
{
	m_Controls.textBrowser_Action->append("Action: Guide Keshi02 Point.");
	Destin_Tail[0] = KeShikPinPos_Set[1][0];
	Destin_Tail[1] = KeShikPinPos_Set[1][1];
	Destin_Tail[2] = KeShikPinPos_Set[1][2];
	// 设置数值
	m_Controls.label_set_x->setText("x: " + QString::number(Destin_Tail[0]));
	m_Controls.label_set_y->setText("y: " + QString::number(Destin_Tail[1]));
	m_Controls.label_set_z->setText("z: " + QString::number(Destin_Tail[2]));
	return true;
}

bool HTONDI::OnSetLink01Clicked()
{
	m_Controls.textBrowser_Action->append("Action: Guide Link01 Point.");
	Destin_Tail[0] = LinkPinPos_Set[0][0];
	Destin_Tail[1] = LinkPinPos_Set[0][1];
	Destin_Tail[2] = LinkPinPos_Set[0][2];

	// 设置数值
	m_Controls.label_set_x->setText("x: " + QString::number(Destin_Tail[0]));
	m_Controls.label_set_y->setText("y: " + QString::number(Destin_Tail[1]));
	m_Controls.label_set_z->setText("z: " + QString::number(Destin_Tail[2]));
	return true;
}

bool HTONDI::OnSetLink02Clicked()
{
	m_Controls.textBrowser_Action->append("Action: Guide Link02 Point.");
	Destin_Tail[0] = LinkPinPos_Set[1][0];
	Destin_Tail[1] = LinkPinPos_Set[1][1];
	Destin_Tail[2] = LinkPinPos_Set[1][2];
	// 设置数值
	m_Controls.label_set_x->setText("x: " + QString::number(Destin_Tail[0]));
	m_Controls.label_set_y->setText("y: " + QString::number(Destin_Tail[1]));
	m_Controls.label_set_z->setText("z: " + QString::number(Destin_Tail[2]));
	return true;
}

bool HTONDI::OnSetLink03Clicked()
{
	m_Controls.textBrowser_Action->append("Action: Guide Link03 Point.");
	Destin_Tail[0] = LinkPinPos_Set[2][0];
	Destin_Tail[1] = LinkPinPos_Set[2][1];
	Destin_Tail[2] = LinkPinPos_Set[2][2];
	// 设置数值
	m_Controls.label_set_x->setText("x: " + QString::number(Destin_Tail[0]));
	m_Controls.label_set_y->setText("y: " + QString::number(Destin_Tail[1]));
	m_Controls.label_set_z->setText("z: " + QString::number(Destin_Tail[2]));
	return true;
}

