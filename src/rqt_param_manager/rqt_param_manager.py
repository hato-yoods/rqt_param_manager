# -*- coding: utf-8 -*-

import sys
import os
import rospy
import rospkg


from qt_gui.plugin import Plugin

from python_qt_binding import loadUi
from python_qt_binding import QtGui
from python_qt_binding.QtWidgets import QWidget, QTableWidgetItem, QItemDelegate, QHeaderView


class NotEditableDelegate(QItemDelegate):
     def __init__(self, *args):
         super(NotEditableDelegate, self).__init__(*args)
         
     def createEditor(self, parent, option, index):
         return None
     
     def editorEvent (self,event,model,option,index):
         return False
         
class RqtParamManagerPlugin(Plugin):
    def __init__(self, context):
        super(RqtParamManagerPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('RqtParamManagerPlugin')
        
        # Process standalone plugin command-line arguments
        #from argparse import ArgumentParser
        #parser = ArgumentParser()
        # Add argument(s) to the parser.
        #parser.add_argument("-q", "--quiet", action="store_true",
        #              dest="quiet",
        #              help="Put plugin in silent mode")
        #args, unknowns = parser.parse_known_args(sys.argv)
        #if not args.quiet:
        #print 'arguments: ', args
        #print 'unknowns: ', unknowns

        resultLoadConfFile , title = self._loadConfFile(sys.argv)
        
        
        # Create QWidget
        self._widget = QWidget()
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_param_manager'), 'resource', 'RqtParamManagerPlugin.ui')
        loadUi(ui_file, self._widget)
        
        self._widget.setObjectName('RqtParamManagerPluginUi')
        self._widget.setWindowTitle(title)
        
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        
        context.add_widget(self._widget)
        
        self._setupParamsTable(self._widget.tblParams)
        
    def _setupParamsTable(self,table):
        table.setColumnCount(3)
        #self._widget.tblParams.setEditTriggers(QAbstractItemView::DoubleClicked
        #                            | QAbstractItemView::SelectedClicked
        #                            | QAbstractItemView::EditKeyPressed );
        table.setRowCount(200);
        table.setItem(0,0,QTableWidgetItem("ABC"));
        table.setItem(0,1,QTableWidgetItem( "CDE" ));
        table.setItem(0,2,QTableWidgetItem( "GHI" ));
        
        noEditDelegate=NotEditableDelegate();
        table.setItemDelegateForColumn(0,noEditDelegate);
        table.setItemDelegateForColumn(1,noEditDelegate);
        
        #header columns
        headerCol1 = QTableWidgetItem()
        headerCol1.setText("パラメータ名")
        table.setHorizontalHeaderItem(0,headerCol1)
        
        headerCol2 = QTableWidgetItem()
        headerCol2.setText("現在値")
        table.setHorizontalHeaderItem(1,headerCol2)
        
        headerCol3 = QTableWidgetItem()
        headerCol3.setText("更新値")
        table.setHorizontalHeaderItem(2,headerCol3)
                
        #header resize
        header = table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.Stretch)
        header.setSectionResizeMode(1, QHeaderView.Fixed)
        header.setSectionResizeMode(2, QHeaderView.Fixed)
        table.setColumnWidth(1,120)
        table.setColumnWidth(2,120)
        
        table.verticalHeader().hide()
        
    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
        
    def _loadConfFile(self , argv):
        result=False
        
        if not len(sys.argv) > 1 :
            rospy.logerr("argv '_conffile' is not specified.")
        else:
            tokens=sys.argv[1].split(":=")
            if len(tokens) == 2 and tokens[0] == "_conffile" :
                confFilePath=tokens[1]
                result=self._parseConfFile(confFilePath)
            else:
                rospy.logerr("argv '_conffile' is wrong format. %s",sys.argv[1])
        
        return result

    def _parseConfFile(self,confFilePath):
        result=False
        title = "不明"
        print "confFilePath3="+confFilePath
        rospy.loginfo("load conf file. path=%s",confFilePath)
        import json
        try:
            f = open(confFilePath, 'r')
            json_dict = json.load(f)
            print('json_dict:{}'.format(type(json_dict)))
            #print json_dict["title"]
            title=json_dict["title"]
        except IOError as e:
            print (e)
            rospy.logerr("json file load failed. %s",e)
        
        return result,title