/*============================================================================
Robot-assisted Surgical Navigation Extension based on MITK
Copyright (c) Hangzhou LancetRobotics,CHINA
All rights reserved.
============================================================================*/


#include "org_mitk_lancet_htotoolkits_Activator.h"
#include "HTOTookits.h"

namespace mitk
{
  void org_mitk_lancet_htotoolkits_Activator::start(ctkPluginContext *context)
  {
    BERRY_REGISTER_EXTENSION_CLASS(HTOTookits, context)
  }

  void org_mitk_lancet_htotoolkits_Activator::stop(ctkPluginContext *context) { Q_UNUSED(context) }
}
