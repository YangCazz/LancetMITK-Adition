/*============================================================================
Robot-assisted Surgical Navigation Extension based on MITK
Copyright (c) Hangzhou LancetRobotics,CHINA
All rights reserved.
============================================================================*/

#ifndef NS_IMAGEPROCESSING_H
#define NS_IMAGEPROCESSING_H

#include <berryISelectionListener.h>
#include <berryIStructuredSelection.h>
#include <QmitkAbstractView.h>
#include <mitkImage.h> // ��������Ҫ���� MITK ͼ��

class NS_ImageProcessing : public QmitkAbstractView
{
public:
    NS_ImageProcessing();
    ~NS_ImageProcessing();

    //// ʾ������������ͼ��
    //bool LoadImage(const std::string& filePath);

    //// ʾ������������ͼ��
    //void ProcessImage();

    //// ʾ������������ͼ��
    //bool SaveImage(const std::string& filePath);

private:
    mitk::Image::Pointer m_Image; // ������ʹ�� MITK ��ͼ����
};

#endif // NS_IMAGEPROCESSING_H