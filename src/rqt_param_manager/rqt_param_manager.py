# -*- coding: utf-8 -*-

import sys
import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding import QtGui
from python_qt_binding.QtCore import QTimer, QVariant
from python_qt_binding.QtWidgets import (
    QWidget,
    QTableWidgetItem,
    QItemDelegate,
    QHeaderView,
    QMessageBox,
)

# ================ 定数一覧 ================
LOG_HEADER = "<RqtParamManagerPlugin>"
FILE_ENC = "utf-8"
INVALID_VAL = "---"
TBL_COL_PARAM_NM = 0
TBL_COL_PARAM_CUR_VAL = 1
TBL_COL_PARAM_UPD_VAL = 2
KEY_CONFFILE_TITLE = "title"
KEY_CONFFILE_GET_INTERVAL = "getInterval"
KEY_CONFFILE_DUMP_YAML = "dumpYaml"
KEY_CONFFILE_PARAMS = "params"
KEY_CONFFILE_PARAM_NM = "paramName"
KEY_CONFFILE_PARAM_DISP = "paramDisp"


class NotEditableDelegate(QItemDelegate):
    """特定の列のセルを編集不可にする為に使用するDelegateクラス"""

    def __init__(self, *args):
        super(NotEditableDelegate, self).__init__(*args)

    def createEditor(self, parent, option, index):
        return None

    def editorEvent(self, event, model, option, index):
        return False


class RqtParamManagerPlugin(Plugin):
    """UIのメインクラス"""

    def __init__(self, context):
        super(RqtParamManagerPlugin, self).__init__(context)

        # クラス変数初期化
        self.title = "不明"
        self.get_interval = 0
        self.dump_yaml_file_path = ""
        self.params = {}
        self.get_timer = QTimer()

        self.setObjectName('RqtParamManagerPlugin')

        resultLoadConfFile = self.loadConfFile(sys.argv)

        # Create QWidget
        self._widget = QWidget()
        ui_file = os.path.join(
            rospkg.RosPack().get_path('rqt_param_manager'),
            'resource',
            'RqtParamManagerPlugin.ui'
        )

        loadUi(ui_file, self._widget)

        self._widget.setObjectName('RqtParamManagerPluginUi')
        self._widget.setWindowTitle(self.title)

        serNum = context.serial_number()
        if serNum > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (' (%d)' % serNum))

        context.add_widget(self._widget)

        self.setupParamsTable(self._widget.tblParams)

        if not resultLoadConfFile:
            self._widget.btnUpdate.setEnabled(False)
            self._widget.btnSave.setEnabled(False)
        else:
            # bind connections
            self._widget.btnUpdate.clicked.connect(self.onExecUpdate)
            self._widget.btnSave.clicked.connect(self.onExecSave)
            self.get_timer.timeout.connect(self.onGetParams)

            self.loadParamsTableItem(self._widget.tblParams, self.params)

            # テーブル行とパラメータ数のチェック
            tableRowNum = self._widget.tblParams.rowCount()
            paramNum = len(self.params)
            if tableRowNum != paramNum or paramNum == 0:
                self._widget.btnUpdate.setEnabled(False)
                self._widget.btnSave.setEnabled(False)

            # 定期監視処理の開始
            if self.get_interval > 0:
                self.onGetParams()
                rospy.loginfo(
                    "%s start monitor. interval=%d sec",
                    LOG_HEADER,
                    self.get_interval
                )
                self.get_timer.start(self.get_interval * 1000)

    def setupParamsTable(self, table):
        # 列は3列
        table.setColumnCount(3)

        # 列1,2は編集不可
        noEditDelegate = NotEditableDelegate()
        table.setItemDelegateForColumn(TBL_COL_PARAM_NM, noEditDelegate)
        table.setItemDelegateForColumn(TBL_COL_PARAM_CUR_VAL, noEditDelegate)

        # ヘッダー列の設定
        headerCol1 = QTableWidgetItem()
        headerCol1.setText("パラメータ名")
        table.setHorizontalHeaderItem(TBL_COL_PARAM_NM, headerCol1)

        headerCol2 = QTableWidgetItem()
        headerCol2.setText("現在値")
        table.setHorizontalHeaderItem(TBL_COL_PARAM_CUR_VAL, headerCol2)

        headerCol3 = QTableWidgetItem()
        headerCol3.setText("更新値")
        table.setHorizontalHeaderItem(TBL_COL_PARAM_UPD_VAL, headerCol3)

        header = table.horizontalHeader()
        header.setSectionResizeMode(TBL_COL_PARAM_NM, QHeaderView.Stretch)
        header.setSectionResizeMode(TBL_COL_PARAM_CUR_VAL, QHeaderView.Fixed)
        header.setSectionResizeMode(TBL_COL_PARAM_UPD_VAL, QHeaderView.Fixed)
        table.setColumnWidth(TBL_COL_PARAM_CUR_VAL, 120)
        table.setColumnWidth(TBL_COL_PARAM_UPD_VAL, 120)

        table.verticalHeader().hide()

    def shutdown_plugin(self):
        self.get_timer.stop()

        # UIが終了してもrosparamに「/rqt_gui_py_node_<no>/conffile」
        # が残っているので削除。この処理が必要なのかどうか不明だが。
        # まぁやっておく。
        self_ros_param_names = [s for s in rospy.get_param_names()
                                if rospy.get_name() in s]
        if len(self_ros_param_names):
            for self_ros_param_name in self_ros_param_names:
                rospy.delete_param(self_ros_param_name)

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    # def trigger_configuration(self):
        # Comment in to signal that the plugin has a way
        # to configure
        # This will enable a setting button (gear icon)
        # in each dock widget title bar
        # Usually used to open a modal configuration dialog

    def loadConfFile(self, argv):
        result = False

        if not len(sys.argv) > 1:
            rospy.logerr("%s argv '_conffile' is not specified.", LOG_HEADER)
        else:
            tokens = sys.argv[1].split(":=")
            if len(tokens) == 2 and tokens[0] == "_conffile":
                confFilePath = tokens[1]
                result = self.parseConfFile(confFilePath)
            else:
                rospy.logerr(
                    "%s argv '_conffile' is wrong format. %s",
                    LOG_HEADER,
                    sys.argv[1]
                )

        return result

    def parseConfFile(self, confFilePath):
        result = False

        rospy.loginfo("%s load conf file. path=%s", LOG_HEADER, confFilePath)
        import json
        try:
            f = open(confFilePath, 'r')
            json_dict = json.load(f)
            self.title = json_dict[KEY_CONFFILE_TITLE]
            self.get_interval = json_dict[KEY_CONFFILE_GET_INTERVAL]
            self.dump_yaml_file_path = json_dict[KEY_CONFFILE_DUMP_YAML]

            rospy.loginfo(
                "%s title=%s",
                LOG_HEADER,
                self.title.encode(FILE_ENC)
            )
            rospy.loginfo(
                "%s getInterval=%s sec",
                LOG_HEADER,
                self.get_interval
            )
            rospy.loginfo(
                "%s dumpYaml=%s",
                LOG_HEADER,
                self.dump_yaml_file_path.encode(FILE_ENC)
            )

            self.params = json_dict[KEY_CONFFILE_PARAMS]

            result = True
        except IOError as e:
            rospy.logerr("%s json file load failed. %s", LOG_HEADER, e)

        return result

    def loadParamsTableItem(self, table, params):
        rowNum = len(params)
        table.setRowCount(rowNum)
        n = 0
        for param in params:
            try:
                label = param[KEY_CONFFILE_PARAM_DISP]
                table.setItem(n, TBL_COL_PARAM_NM, QTableWidgetItem(label))
            except KeyError as e:
                table.setItem(n, TBL_COL_PARAM_NM, QTableWidgetItem("不明"))
                rospy.logerr("%s conf file key error. %s", LOG_HEADER, e)

            table.setItem(
                n,
                TBL_COL_PARAM_CUR_VAL,
                QTableWidgetItem(INVALID_VAL)
            )
            table.setItem(n, TBL_COL_PARAM_UPD_VAL, QTableWidgetItem(""))
            n += 1

    def onGetParams(self):
        paramNum = len(self.params)
        for n in range(paramNum):
            param = self.params[n]
            val = INVALID_VAL
            try:
                paramNm = param[KEY_CONFFILE_PARAM_NM]
                val = rospy.get_param(paramNm)
            except KeyError as e:
                # エラーに出すと数がすごいことになりそうなので出さない
                pass
            table = self._widget.tblParams
            table.setItem(
                n,
                TBL_COL_PARAM_CUR_VAL,
                QTableWidgetItem("%s" % val)
            )
            updVal = table.item(n, TBL_COL_PARAM_UPD_VAL).text()
            # 無効データ取得 もしくは 初回データ更新
            if INVALID_VAL == val \
               or len(updVal) == 0 \
               or (INVALID_VAL != val and updVal == INVALID_VAL):
                table.setItem(
                    n,
                    TBL_COL_PARAM_UPD_VAL,
                    QTableWidgetItem("%s" % val)
                )

    def onExecUpdate(self):
        result = False
        table = self._widget.tblParams
        rowNum = table.rowCount()
        
        updNum = 0
        okNum = 0
        for n in range(rowNum):
            curVal = table.item(n, TBL_COL_PARAM_CUR_VAL).text()
            updVal = table.item(n, TBL_COL_PARAM_UPD_VAL).text()

            if curVal == updVal or \
               INVALID_VAL != updVal or \
               len(updVal) <= 0:

                param = self.params[n]
                updNum += 1

                try:
                    paramNm = param[KEY_CONFFILE_PARAM_NM]

                    rospy.set_param(paramNm, updVal)
                    rospy.loginfo(
                        "%s paramNm=%s val=%s",
                        LOG_HEADER,
                        paramNm,
                        updVal
                    )
                    okNum += 1
                except KeyError as e:
                    rospy.logerr(
                        "%s update faild. paramNo=%d cause=%s",
                        LOG_HEADER,
                        n,
                        e
                    )

        if updNum != okNum:
            QMessageBox.warning(self._widget, "警告", "一部パラメータの更新に失敗しました。")
        else:
            result = True
        return result

    def onExecSave(self):
        self.get_timer.stop()
        self._widget.setEnabled(False)
        
        if not self.onExecUpdate():
            self.get_timer.start()
            QMessageBox.critical(self._widget, "エラー", "保存に失敗しました。")
            return

        # http://docs.ros.org/api/rosparam/html/
        import rosparam
        try:
            rosparam.dump_params("/tmp/ros.yaml","/")
            QMessageBox.information(self._widget,"お知らせ","設定を保存しました。")
        except IOError as e:
            rospy.logerr("%s dump failed. %s", LOG_HEADER, e)
            QMessageBox.critical(self._widget, "エラー", "保存に失敗しました。")

        self.get_timer.start()
        self._widget.setEnabled(True)
