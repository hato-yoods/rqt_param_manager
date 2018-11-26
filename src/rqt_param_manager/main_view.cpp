#include <rqt_param_manager/main_view.h>

#include <pluginlib/class_list_macros.h>
#include <ros/master.h>
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <QFileDialog>
#include <QMessageBox>
#include <QPainter>
#include <iostream>
#include <QDateTime>
#include <QJsonDocument>
#include <QJsonObject>

namespace rqt_param_manager {

MainView::MainView()
	: rqt_gui_cpp::Plugin()
	, widget_(0)
	, mMonInterval(0)
{
	setObjectName("MainView");
}

void MainView::initPlugin(qt_gui_cpp::PluginContext& context){
	widget_ = new QWidget();
	ui_.setupUi(widget_);

	//if (context.serialNumber() > 1)
	//{
	//  widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
	//}
	const QStringList& argv = context.argv();
	if(argv.isEmpty()){
		ui_.pnlMain->setEnabled(false);
		widget_->setWindowTitle("不明");
		ui_.lblTitle->setText("不明");
	}else{
		
		//const QString title=argv.at(0);
		//ui_.lblTitle->setText(title);
		//widget_->setWindowTitle(title);
		
		if (context.serialNumber() > 1)
		{
			widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
		}
		
		connect(ui_.btnUpdate,SIGNAL(clicked()),this,SLOT(onParamUpdate()));
		connect(ui_.btnSave,SIGNAL(clicked()),this,SLOT(onParamSave()));
	}
	
	
	context.addWidget(widget_);
  
	for(int i=0;i<argv.size();++i){
		const QString param=argv.at(i);
		std::cerr << "param:" << param.toLocal8Bit().constData() << std::endl;
	}
	
}
	
void MainView::showEvent(QShowEvent *event){
	std::cerr << "show event called." << std::endl;
}
	
void MainView::shutdownPlugin(){
	
}

void MainView::onParamUpdate(){
	std::cerr << "on update called" << std::endl;
}

void MainView::onParamSave(){
	std::cerr << "on save called" << std::endl;
}

	
//===============================================================
}


PLUGINLIB_EXPORT_CLASS(rqt_param_manager::MainView, rqt_gui_cpp::Plugin)
