#include "QLancetThaApplicationWorkbenchWindowAdvisor.h"

// ctk
#include <ctkPluginContext.h>

// berry
#include <berryPlatformUI.h>
#include <berryIWorkbenchPage.h>
#include <berryWorkbenchPlugin.h>

// Qt
#include <QMenuBar>
#include <QToolButton>
#include <QVBoxLayout>
#include <QMainWindow>

// statemachine
#include <lancetIScxmlStateMachineService.h>

// tha
#include "QLancetThaWorkbenBarAdvisor.h"
#include "QLancetThaWorkbenchMenuBar.h"
#include "org_mitk_lancet_tha_application_Activator.h"

QLancetThaApplicationWorkbenchWindowAdvisor::QLancetThaApplicationWorkbenchWindowAdvisor(
	berry::WorkbenchAdvisor* wbAdvisor,
	const berry::SmartPointer<berry::IWorkbenchWindowConfigurer>& configurer)
	: berry::WorkbenchWindowAdvisor(configurer)
{

}

QLancetThaApplicationWorkbenchWindowAdvisor::~QLancetThaApplicationWorkbenchWindowAdvisor()
{

}

berry::SmartPointer<berry::ActionBarAdvisor> 
QLancetThaApplicationWorkbenchWindowAdvisor::CreateActionBarAdvisor(
	berry::SmartPointer<berry::IActionBarConfigurer> configurer)
{
	return berry::SmartPointer<QLancetThaWorkbenBarAdvisor>(
		new QLancetThaWorkbenBarAdvisor(configurer));
}

void QLancetThaApplicationWorkbenchWindowAdvisor::PostWindowCreate()
{
	qDebug() << "QLancetThaApplicationWorkbenchWindowAdvisor::PostWindowCreate";
	berry::IWorkbenchWindow::Pointer window = this->GetWindowConfigurer()->GetWindow();
	QMainWindow* mainWindow = qobject_cast<QMainWindow*> (window->GetShell()->GetControl());
	mainWindow->setContextMenuPolicy(Qt::PreventContextMenu);

	QVBoxLayout* layout = new QVBoxLayout();
	mainWindow->centralWidget()->setLayout(layout);

	QWidget* mainWndMenuWidget = new QWidget(mainWindow->centralWidget());
	mainWndMenuWidget->setMinimumHeight(40);
	mainWndMenuWidget->setSizePolicy(QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Fixed);
	mainWndMenuWidget->setStyleSheet("background-color:rgb(0,0,255)");

	QLancetThaWorkbenchMenuBar* menuBar = new QLancetThaWorkbenchMenuBar();
	menuBar->setMovable(false);
	menuBar->setMinimumHeight(48);
	menuBar->setOrientation(Qt::Horizontal);
	menuBar->setObjectName("topMainActionsToolBar");
	menuBar->setContextMenuPolicy(Qt::PreventContextMenu);
	menuBar->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
	menuBar->setStyleSheet("background-color:rgb(255,0,0)");
	menuBar->InitializeStateMachineForUi();

	menuBar->GetModuleMenuBar()->setStyleSheet("background-color:rgb(0,255,0)");
	menuBar->GetApplicationToolBar()->setStyleSheet("background-color:rgb(0,0,255)");

	menuBar->GetApplicationToolBar()->addAction("Test");
	auto btn = new QToolButton();
	btn->setText("Test2");

	auto menu = new QMenu("Menu");
	menu->addAction("MT_Menu1");
	menu->addAction("MT_Menu2");
	menu->addAction("MT_Menu3");


	menuBar->GetApplicationToolBar()->addWidget(btn);
	menuBar->GetApplicationToolBar()->addWidget(menu);

	mainWindow->addToolBar(Qt::ToolBarArea::TopToolBarArea, menuBar);

	if (menuBar->GetStateMachineService().IsNotNull())
	{
		menuBar->GetStateMachineService()->StartStateMachine();
	}
}