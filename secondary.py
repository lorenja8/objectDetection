# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'secondary.ui'
#
# Created by: PyQt5 UI code generator 5.15.4
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(1013, 402)
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap("logo.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        Form.setWindowIcon(icon)
        self.gridLayout = QtWidgets.QGridLayout(Form)
        self.gridLayout.setObjectName("gridLayout")
        self.settings_footage_label2 = QtWidgets.QLabel(Form)
        self.settings_footage_label2.setText("")
        self.settings_footage_label2.setObjectName("settings_footage_label2")
        self.gridLayout.addWidget(self.settings_footage_label2, 0, 5, 3, 1)
        self.settings_footage_label1 = QtWidgets.QLabel(Form)
        self.settings_footage_label1.setText("")
        self.settings_footage_label1.setObjectName("settings_footage_label1")
        self.gridLayout.addWidget(self.settings_footage_label1, 0, 4, 3, 1)
        self.settings_slider1 = QtWidgets.QSlider(Form)
        self.settings_slider1.setMaximum(255)
        self.settings_slider1.setSliderPosition(160)
        self.settings_slider1.setOrientation(QtCore.Qt.Vertical)
        self.settings_slider1.setObjectName("settings_slider1")
        self.gridLayout.addWidget(self.settings_slider1, 0, 0, 1, 1)
        self.settings_slider4 = QtWidgets.QSlider(Form)
        self.settings_slider4.setMaximum(255)
        self.settings_slider4.setSliderPosition(255)
        self.settings_slider4.setOrientation(QtCore.Qt.Vertical)
        self.settings_slider4.setObjectName("settings_slider4")
        self.gridLayout.addWidget(self.settings_slider4, 0, 3, 1, 1)
        self.settings_slider2 = QtWidgets.QSlider(Form)
        self.settings_slider2.setMaximum(255)
        self.settings_slider2.setSliderPosition(255)
        self.settings_slider2.setOrientation(QtCore.Qt.Vertical)
        self.settings_slider2.setObjectName("settings_slider2")
        self.gridLayout.addWidget(self.settings_slider2, 0, 1, 1, 1)
        self.settings_slider3 = QtWidgets.QSlider(Form)
        self.settings_slider3.setMaximum(255)
        self.settings_slider3.setSliderPosition(240)
        self.settings_slider3.setOrientation(QtCore.Qt.Vertical)
        self.settings_slider3.setObjectName("settings_slider3")
        self.gridLayout.addWidget(self.settings_slider3, 0, 2, 1, 1)
        self.settings_default_pushbutton = QtWidgets.QPushButton(Form)
        self.settings_default_pushbutton.setObjectName("settings_default_pushbutton")
        self.gridLayout.addWidget(self.settings_default_pushbutton, 2, 0, 1, 4)
        self.settings_method_combobox = QtWidgets.QComboBox(Form)
        self.settings_method_combobox.setObjectName("settings_method_combobox")
        self.settings_method_combobox.addItem("")
        self.settings_method_combobox.addItem("")
        self.gridLayout.addWidget(self.settings_method_combobox, 1, 0, 1, 4)

        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "Settings"))
        self.settings_default_pushbutton.setText(_translate("Form", "Default"))
        self.settings_method_combobox.setItemText(0, _translate("Form", "Threshold"))
        self.settings_method_combobox.setItemText(1, _translate("Form", "White Filter"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    Form = QtWidgets.QWidget()
    ui = Ui_Form()
    ui.setupUi(Form)
    Form.show()
    sys.exit(app.exec_())
