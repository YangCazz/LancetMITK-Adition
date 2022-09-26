set(H_FILES
  DataManagement/lancetNavigationObject.h
  DataManagement/lancetTrackingDeviceSourceConfigurator.h
  DataManagement/lancetPathPoint.h
  
  IO/lancetNavigationObjectWriter.h

  Algorithms/lancetNavigationDataInReferenceCoordFilter.h
  Algorithms/lancetApplyDeviceRegistratioinFilter.h
  Rendering/lancetNavigationObjectVisualizationFilter.h

  TrackingDevices/lancetRobotTrackingTool.h
  TrackingDevices/lancetKukaTrackingDeviceTypeInformation.h
  TrackingDevices/kukaRobotDevice.h

  UI/QmitkLancetKukaWidget.cpp
)

set(CPP_FILES
  DataManagement/lancetNavigationObject.cpp
  DataManagement/lancetTrackingDeviceSourceConfigurator.cpp
  DataManagement/lancetPathPoint.cpp
  
  IO/lancetNavigationObjectWriter.cpp
  
  Algorithms/lancetNavigationDataInReferenceCoordFilter.cpp
  Algorithms/lancetApplyDeviceRegistratioinFilter.cpp
  Rendering/lancetNavigationObjectVisualizationFilter.cpp

  TrackingDevices/lancetRobotTrackingTool.cpp
  TrackingDevices/lancetKukaTrackingDeviceTypeInformation.cpp
  TrackingDevices/kukaRobotDevice.cpp

  UI/QmitkLancetKukaWidget.cpp
)

set(UI_FILES
  UI/QmitkLancetKukaWidget.ui
)

set(MOC_H_FILES
  UI/QmitkLancetKukaWidget.h
  TrackingDevices/kukaRobotDevice.h
)

set(RESOURCE_FILES
)
