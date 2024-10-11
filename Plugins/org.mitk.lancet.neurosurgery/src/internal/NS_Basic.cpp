/*============================================================================
Robot-assisted Surgical Navigation Extension based on MITK
Copyright (c) Hangzhou LancetRobotics,CHINA
All rights reserved.
============================================================================*/

#include "NeuroSurgery.h"

// Qmitk
#include <QmitkAbstractView.h>
#include <QmitkSingleNodeSelectionWidget.h>
#include <QtWidgets/QFileDialog>
#include <QCheckBox>
#include <QString>

// mitk
#include <mitkImage.h>
#include <mitkDataStorage.h>
#include <mitkDataNode.h>
#include <mitkLookupTable.h>
#include <mitkLookupTableProperty.h>
#include <mitkRenderingManager.h>
#include <mitkImageCast.h>
#include <mitkNodePredicateAnd.h>
#include <mitkNodePredicateDataType.h>
#include <mitkNodePredicateNot.h>
#include <mitkNodePredicateOr.h>
#include <mitkNodePredicateProperty.h>

// itk
#include <itkImage.h>
#include <itkBinaryThresholdImageFilter.h>
#include <mitkITKImageImport.h>

void NeuroSurgery::OnCheckDataClicked()
{
	/* Data Test.
	* 1. basic data load
	* 2. device surface load
	*/
	m_Controls.textBrowser_actionLOG->append("Action: Check Base data.");
    std::map<std::string, QCheckBox*> imageControls = {
        {"T1w", m_Controls.checkBox_T1w},
        {"T2w", m_Controls.checkBox_T2w},
        {"PET", m_Controls.checkBox_PET},
        {"CT", m_Controls.checkBox_CT},
        {"MRA", m_Controls.checkBox_MRA},
        {"CTA", m_Controls.checkBox_CTA},
        {"DSA", m_Controls.checkBox_DSA}
    };

    // ����ͼ�����ƺͿؼ�ӳ��
    for (const auto& [imageName, checkBox] : imageControls) {
        auto imageNode = GetDataStorage()->GetNamedNode(imageName);
        if (imageNode) {
            checkBox->setChecked(true);
            m_Controls.textBrowser_actionLOG->append(QString("load Image_%1.").arg(QString::fromStdString(imageName)));
        }
    }
}

void NeuroSurgery::OnCheckPETMaskClicked()
{
    /* PET mask generate */
    m_Controls.textBrowser_actionLOG->append("Action: Check PET Mask.");

}

void NeuroSurgery::OnCheckPETColorfyClicked()
{
    /* PET data colorfy */
    m_Controls.textBrowser_actionLOG->append("Action: Check Base data.");
    // ��ȡ���ݴ洢
    mitk::DataStorage::Pointer dataStorage = GetDataStorage();

    // ��ȡ��Ϊ "PET" �Ľڵ�
    mitk::DataNode::Pointer petNode = dataStorage->GetNamedNode("PET");
    if (!petNode)
    {
        std::cerr << "Could not find PET node." << std::endl;
        return;
    }

    // ��ȡͼ������
    mitk::Image::Pointer petImage = dynamic_cast<mitk::Image*>(petNode->GetData());
    if (!petImage)
    {
        std::cerr << "Selected node is not an image node." << std::endl;
        return;
    }

    // ����������ɫ�ʲ��ұ�
    mitk::LookupTable::Pointer lookupTable = mitk::LookupTable::New();
    lookupTable->SetType(mitk::LookupTable::JET); // ʹ�� JET ɫ��ӳ��

    // ���� LookupTableProperty �����õ��ڵ�
    mitk::LookupTableProperty::Pointer lookupTableProperty = mitk::LookupTableProperty::New();
    lookupTableProperty->SetLookupTable(lookupTable);
    petNode->SetProperty("LookupTable", lookupTableProperty);

    // ������Ⱦ
    mitk::RenderingManager::GetInstance()->RequestUpdateAll();
}

void NeuroSurgery::ParamsInit()
{
	/* NS Param Init Here.
	* 1.
	* 2.
	*/


}

void NeuroSurgery::InitSurfaceSelector(QmitkSingleNodeSelectionWidget* widget)
{
    // surface data Load
    widget->SetDataStorage(GetDataStorage());
    widget->SetNodePredicate(mitk::NodePredicateAnd::New(
        mitk::TNodePredicateDataType<mitk::Surface>::New(),
        mitk::NodePredicateNot::New(mitk::NodePredicateOr::New(mitk::NodePredicateProperty::New("helper object"),
            mitk::NodePredicateProperty::New("hidden object")))));

    widget->SetSelectionIsOptional(true);
    widget->SetAutoSelectNewNodes(true);
    widget->SetEmptyInfo(QString("Please select a surface"));
    widget->SetPopUpTitel(QString("Select surface"));
}

void NeuroSurgery::InitPointSetSelector(QmitkSingleNodeSelectionWidget* widget)
{
	// load node data
	widget->SetDataStorage(GetDataStorage());
	widget->SetNodePredicate(mitk::NodePredicateAnd::New(
		mitk::TNodePredicateDataType<mitk::PointSet>::New(),
		mitk::NodePredicateNot::New(mitk::NodePredicateOr::New(mitk::NodePredicateProperty::New("helper object"),
			mitk::NodePredicateProperty::New("hidden object")))));

	widget->SetSelectionIsOptional(true);
	widget->SetAutoSelectNewNodes(true);
	widget->SetEmptyInfo(QString("Please select a point set"));
	widget->SetPopUpTitel(QString("Select point set"));
}