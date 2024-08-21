/*============================================================================
Robot-assisted Surgical Navigation Extension based on MITK
Copyright (c) Hangzhou LancetRobotics,CHINA
All rights reserved.
============================================================================*/


//void AimoeTest::CreateQtPartControl(QWidget* parent)
//{
//	// ... existing code ...
//
//	connect(m_Controls.pushButton_xp, &QPushButton::clicked, this, &AimoeTest::TranslatePlusX);
//	connect(m_Controls.pushButton_yp, &QPushButton::clicked, this, &AimoeTest::TranslatePlusY);
//	connect(m_Controls.pushButton_zp, &QPushButton::clicked, this, &AimoeTest::TranslatePlusZ);
//	connect(m_Controls.pushButton_xm, &QPushButton::clicked, this, &AimoeTest::TranslateMinusX);
//	connect(m_Controls.pushButton_ym, &QPushButton::clicked, this, &AimoeTest::TranslateMinusY);
//	connect(m_Controls.pushButton_zm, &QPushButton::clicked, this, &AimoeTest::TranslateMinusZ);
//	connect(m_Controls.pushButton_rxp, &QPushButton::clicked, this, &AimoeTest::RotatePlusX);
//	connect(m_Controls.pushButton_ryp, &QPushButton::clicked, this, &AimoeTest::RotatePlusY);
//	connect(m_Controls.pushButton_rzp, &QPushButton::clicked, this, &AimoeTest::RotatePlusZ);
//	connect(m_Controls.pushButton_rxm, &QPushButton::clicked, this, &AimoeTest::RotateMinusX);
//	connect(m_Controls.pushButton_rym, &QPushButton::clicked, this, &AimoeTest::RotateMinusY);
//	connect(m_Controls.pushButton_rzm, &QPushButton::clicked, this, &AimoeTest::RotateMinusZ);
//
//	// ... existing code ...
//}


//void AimoeTest::TranslatePlusX()
//{
//	double direction[3] = { 1, 0, 0 };
//	Translate(direction, 1.0, m_baseDataToMove);
//}
//
//void AimoeTest::TranslatePlusY()
//{
//	double direction[3] = { 0, 1, 0 };
//	Translate(direction, 1.0, m_baseDataToMove);
//}
//
//void AimoeTest::TranslatePlusZ()
//{
//	double direction[3] = { 0, 0, 1 };
//	Translate(direction, 1.0, m_baseDataToMove);
//}
//
//void AimoeTest::TranslateMinusX()
//{
//	double direction[3] = { -1, 0, 0 };
//	Translate(direction, 1.0, m_baseDataToMove);
//}
//
//void AimoeTest::TranslateMinusY()
//{
//	double direction[3] = { 0, -1, 0 };
//	Translate(direction, 1.0, m_baseDataToMove);
//}
//
//void AimoeTest::TranslateMinusZ()
//{
//	double direction[3] = { 0, 0, -1 };
//	Translate(direction, 1.0, m_baseDataToMove);
//}
//
//void AimoeTest::RotatePlusX()
//{
//	double center[3] = { 0, 0, 0 };
//	double direction[3] = { 1, 0, 0 };
//	Rotate(center, direction, 10.0, m_baseDataToMove);
//}
//
//void AimoeTest::RotatePlusY()
//{
//	double center[3] = { 0, 0, 0 };
//	double direction[3] = { 0, 1, 0 };
//	Rotate(center, direction, 10.0, m_baseDataToMove);
//}
//
//void AimoeTest::RotatePlusZ()
//{
//	double center[3] = { 0, 0, 0 };
//	double direction[3] = { 0, 0, 1 };
//	Rotate(center, direction, 10.0, m_baseDataToMove);
//}
//
//void AimoeTest::RotateMinusX()
//{
//	double center[3] = { 0, 0, 0 };
//	double direction[3] = { -1, 0, 0 };
//	Rotate(center, direction, 10.0, m_baseDataToMove);
//}
//
//void AimoeTest::RotateMinusY()
//{
//	double center[3] = { 0, 0, 0 };
//	double direction[3] = { 0, -1, 0 };
//	Rotate(center, direction, 10.0, m_baseDataToMove);
//}
//
//void AimoeTest::RotateMinusZ()
//{
//	double center[3] = { 0, 0, 0 };
//	double direction[3] = { 0, 0, -1 };
//	Rotate(center, direction, 10.0, m_baseDataToMove);
//}