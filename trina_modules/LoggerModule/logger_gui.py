import sys
from datetime import datetime
from PyQt5 import QtCore, QtGui, QtWidgets, uic
from reem.connection import RedisInterface
from reem.datatypes import KeyValueStore
import redis

qtcreator_file  = "logger.ui" # Enter file here.
Ui_MainWindow, QtBaseClass = uic.loadUiType(qtcreator_file)


class MyWindow(QtWidgets.QMainWindow, Ui_MainWindow):
    def __init__(self):
        QtWidgets.QMainWindow.__init__(self)
        Ui_MainWindow.__init__(self)
        self.setupUi(self)
        self._connect_reem()
        self._UI_logic()

    def _connect_reem(self):
        self.interface = RedisInterface(host="localhost")
        self.interface.initialize()
        self.server = KeyValueStore(self.interface)
        self.server['LOGGER_ITEMS']={'Video':False, 'UIState':False, 'RobotState':False, 'Commands':False}
        self.server['LOGGER_STATUS']={'Start':False, 'Pause':False, 'Stop':False}
        self.server['LOGGER_NAME']=''
        self.server['LOGGER_LABELS'] = ''
        self.comment_queue = TrinaQueue('LOGGER_COMMENTS')
        print("reem connected")
    
    def _UI_logic(self):
        self.pushButton_confirm1.clicked.connect(self.report_name)
        self.pushButton_confirm2.clicked.connect(self.report_comment)
        self.listWidget_tag.itemClicked.connect(self.report_tag)
        self.pushButton_start.clicked.connect(self.report_start)
        self.pushButton_stop.clicked.connect(self.report_stop)
        self.pushButton_pause.clicked.connect(self.report_pause)
        self.checkBox_Video.clicked.connect(self.report_video)
        self.checkBox_Commands.clicked.connect(self.report_command)
        self.checkBox_UIstate.clicked.connect(self.report_uistate)
        self.checkBox_Robotstate.clicked.connect(self.report_robotstate)
        self.pushButton_confirm.clicked.connect(self.add_new_tag)
        self.actionReset.triggered.connect(self.reset)
        pass
    
    def reset(self):
        self.listWidget_msg.clear()
        self.listWidget_msg.addItem("Welcome to TRINA logger!")
        self.listWidget_tag.clear()
        self.listWidget_tag.addItem("Test Label")
        self.input_tag.clear()

    def add_new_tag(self):
        new_tag = self.input_tag.text()
        self.listWidget_tag.addItem(new_tag)

    def report_video(self):
        if self.checkBox_Video.isChecked():
            self.server['LOGGER_ITEMS']['Video'] = True
            self.report("Logging result now includes Video.")
        else:
            self.server['LOGGER_ITEMS']['Video'] = False
            self.report("Logging result now excludes Video.")
    
    def report_command(self):
        if self.checkBox_Commands.isChecked():
            self.server['LOGGER_ITEMS']['Commands'] = True
            self.report("Logging result now includes Commands.")
        else:
            self.server['LOGGER_ITEMS']['Commands'] = False
            self.report("Logging result now excludes Commands.")

    def report_uistate(self):
        if self.checkBox_UIstate.isChecked():
            self.server['LOGGER_ITEMS']['UIState'] = True
            self.report("Logging result now includes UI state.")
        else:
            self.server['LOGGER_ITEMS']['UIState'] = False
            self.report("Logging result now excludes UI state.")
    
    def report_robotstate(self):
        if self.checkBox_Robotstate.isChecked():
            self.server['LOGGER_ITEMS']['RobotState'] = True
            self.report("Logging result now includes robot state.")
        else:
            self.server['LOGGER_ITEMS']['RobotState'] = False
            self.report("Logging result now excludes robot state.")
     
    def report_start(self):
        self.server['LOGGER_STATUS']['Start'] = True
        self.server['LOGGER_STATUS']['Stop'] = False
        self.server['LOGGER_STATUS']['Pause'] = False
        self.report("Logging Started.")

    def report_stop(self):
        self.server['LOGGER_STATUS']['Start'] = False
        self.server['LOGGER_STATUS']['Stop'] = True
        self.server['LOGGER_STATUS']['Pause'] = False
        self.report("Logging Stopped.")

    def report_pause(self):
        self.server['LOGGER_STATUS']['Pause'] = True
        self.server['LOGGER_STATUS']['Start'] = False
        self.server['LOGGER_STATUS']['Stop'] = False
        self.report("Logging Paused.")
    
    def report_tag(self,item):
        self.server['LOGGER_LABELS'] = item.text()
        self.report(item.text() + " is tagged.")
    
    def report_name(self):
        self.server['LOGGER_NAME']=self.input_name.text()
        self.report("Experiment Name "+ self.input_name.text() + " is sent.")
    
    def report_comment(self):
        self.comment_queue.push(str([self.input_comment.toPlainText(),self.server['TRINA_TIME'].read()]))
        self.report("Comment added.")
    
    def report(self,msg):
        self.listWidget_msg.addItem(datetime.now().strftime("%H:%M:%S")+" --------- "+ msg)


class TrinaQueue(object):
	def __init__(self,key, host = 'localhost', port = 6379):
		self.r = redis.Redis(host = host, port = port)
		self.key = key
	def push(self,item):
		self.r.rpush(self.key,item)



if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = MyWindow()
    window.show()
    sys.exit(app.exec_())