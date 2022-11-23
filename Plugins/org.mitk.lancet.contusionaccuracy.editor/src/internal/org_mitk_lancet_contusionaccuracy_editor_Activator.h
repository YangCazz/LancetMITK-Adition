/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/


#ifndef org_mitk_lancet_contusionaccuracy_editor_Activator_H
#define org_mitk_lancet_contusionaccuracy_editor_Activator_H

#include <ctkPluginActivator.h>
#include <org_mitk_lancet_contusionaccuracy_editor_Export.h>

/**
 * \ingroup org_mitk_lancet_example_editor
 */
class ORG_MITK_LANCET_CONTUSIONACCURACY_EDITOR
	org_mitk_lancet_contusionaccuracy_editor_Activator : public QObject, public ctkPluginActivator
{
  Q_OBJECT
  Q_PLUGIN_METADATA(IID "org.mitk.lancet.contusionaccuracy.editor")
  Q_INTERFACES(ctkPluginActivator)

public:

  void start(ctkPluginContext* context) override;
  void stop(ctkPluginContext* context) override;

  static ctkPluginContext* GetContext();

private:

  static ctkPluginContext* m_Context;

};

typedef org_mitk_lancet_contusionaccuracy_editor_Activator PluginActivator;

#endif /* org_mitk_lancet_contusionaccuracy_editor_Activator_H */
