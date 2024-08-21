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






// 术中导航
void HTONDI::trackingObjectPos()
{
	/* 追踪并返回摆锯的位置 
	

	*/


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


bool HTONDI::OnStartAxialGuideClicked()
{
	/* 开始水平截骨导航
	1. 生成实时截骨面
	2. 计算 实时截骨面 与 规划截骨面 的夹角

	开始水平截骨导航时，需要计算截骨面的法向量和前端位置
	那么，也就需要知道实时的摆锯最前端的截骨面点以及对应的截骨面法向量
	*/

	m_Controls.textBrowser_Action->append("Action: Start Axial Cut Guide.");

	// 1. 显示规划的截骨面 + 显示实时截骨面位置
	auto preCutPlane01 = GetDataStorage()->GetNamedNode("1st cut plane");
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
	if (m_timer_saw == nullptr)
	{
		m_timer_saw = new QTimer(this);
	}
	m_Controls.textBrowser_Action->append("Generate Real time Cut plane");
	connect(m_timer_saw, SIGNAL(timeout()), this, SLOT(GenerateRealTimeBoneSurface()));
	m_timer_saw->start(100);

	return true;
}

bool HTONDI::OnStartAxialCutClicked()
{
	/* 启动水平截骨
	1. 改变摆锯截骨状态
	2. 计算实时 截骨深度
	3. 进行截骨保护
	*/

	// 开始水平截骨，启动摆锯，启动截骨导航
	m_Controls.textBrowser_Action->append("Action: Check Drill Node.");

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
		(PlanePoint1[0] + PlanePoint1[0] + PlanePoint1[0]) / 3,
		(PlanePoint1[1] + PlanePoint1[1] + PlanePoint1[1]) / 3,
		(PlanePoint1[2] + PlanePoint1[2] + PlanePoint1[2]) / 3
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
	m_cutType = -1;
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
	/* 开始钻孔导航
	1. 生成实时磨钻位置
	2. 计算 实时钻孔位置 和 规划钻孔位置 差距
	*/

	m_Controls.textBrowser_Action->append("Action: Check Drill Node.");

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