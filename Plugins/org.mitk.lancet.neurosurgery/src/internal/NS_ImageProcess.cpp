/*============================================================================
Robot-assisted Surgical Navigation Extension based on MITK
Copyright (c) Hangzhou LancetRobotics,CHINA
All rights reserved.
============================================================================*/

#include "NeuroSurgery.h"

#include <iostream>

// MITK
#include "mitkDataStorage.h"
#include "mitkDataNode.h"
#include "mitkImage.h"
#include "mitkImageCast.h"

// ITK
#include "itkImage.h"
#include "itkResampleImageFilter.h"
#include "itkMattesMutualInformationImageToImageMetric.h"
#include "itkLinearInterpolateImageFunction.h"
#include "itkTranslationTransform.h"
#include "itkRegularStepGradientDescentOptimizerv4.h"


void NeuroSurgery::OnRegistrateImageClicked()
{
    /*
    // ��ȡ MRI �� PET ͼ������
    mitk::DataStorage::Pointer dataStorage = GetDataStorage();
    mitk::DataNode::Pointer mriNode = dataStorage->GetNamedNode("MRI");
    mitk::DataNode::Pointer petNode = dataStorage->GetNamedNode("PET");

    if (!mriNode || !petNode) {
        std::cerr << "Error: MRI or PET node not found." << std::endl;
        return;
    }

    mitk::Image::Pointer mriImage = dynamic_cast<mitk::Image*>(mriNode->GetData());
    mitk::Image::Pointer petImage = dynamic_cast<mitk::Image*>(petNode->GetData());

    if (!mriImage || !petImage) {
        std::cerr << "Error: MRI or PET image data is invalid." << std::endl;
        return;
    }

    // ת��Ϊ ITK ͼ������
    typedef itk::Image<double, 3> ImageType;
    ImageType::Pointer itkMriImage;
    ImageType::Pointer itkPetImage;

    try {
        mitk::CastToItkImage(mriImage, itkMriImage);
        mitk::CastToItkImage(petImage, itkPetImage);
    }
    catch (const mitk::Exception& e) {
        std::cerr << "Error during MITK to ITK image conversion: " << e.what() << std::endl;
        return;
    }

    // ������׼����
    typedef itk::TranslationTransform<double, 3> TransformType;
    TransformType::Pointer transform = TransformType::New();
    transform->SetIdentity();

    typedef itk::MattesMutualInformationImageToImageMetric<ImageType, ImageType> MetricType;
    MetricType::Pointer metric = MetricType::New();

    typedef itk::RegularStepGradientDescentOptimizerv4<double> OptimizerType;
    OptimizerType::Pointer optimizer = OptimizerType::New();
    optimizer->SetLearningRate(0.1);
    optimizer->SetNumberOfIterations(100);

    typedef itk::ResampleImageFilter<ImageType, ImageType> ResampleFilterType;
    ResampleFilterType::Pointer resampleFilter = ResampleFilterType::New();
    resampleFilter->SetInput(itkPetImage);
    resampleFilter->SetTransform(transform);
    resampleFilter->SetInterpolator(itk::LinearInterpolateImageFunction<double, 3>::New());
    resampleFilter->SetOutputOrigin(itkMriImage->GetOrigin());
    resampleFilter->SetOutputSpacing(itkMriImage->GetSpacing());
    resampleFilter->SetSize(itkMriImage->GetLargestPossibleRegion().GetSize());

    // ������׼
    optimizer->SetMetric(metric);
    optimizer->SetOptimizable(resampleFilter->GetModifiableTransform());
    try {
        optimizer->StartOptimization();
    }
    catch (itk::ExceptionObject& ex) {
        std::cerr << "Error during optimization: " << ex << std::endl;
        return;
    }

    // ��ȡ��׼��� PET ͼ��
    ImageType::Pointer registeredPetImage = resampleFilter->GetOutput();

    // ����׼��� ITK ͼ��ת���� MITK ͼ�����ͣ����������ݽڵ�
    mitk::Image::Pointer mitkRegisteredPetImage = ConvertITKToMITKImage(registeredPetImage);
    if (!mitkRegisteredPetImage) {
        std::cerr << "Error: Failed to convert registered ITK image to MITK format." << std::endl;
        return;
    }
    petNode->SetData(mitkRegisteredPetImage);

    std::cout << "Image registration completed successfully." << std::endl;
    */
}
