# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'src/robot_movil_4_ruedas/robot_movil_control/GUI/gui.ui'
#
# Created: Tue May 31 17:02:30 2016
#      by: PyQt4 UI code generator 4.10.4
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName(_fromUtf8("Form"))
        Form.resize(241, 357)
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(_fromUtf8("../../../../../../../../media/kaiser/Data/Kaiser_1/pegatinas/41mc+XLgDTL._SY300_.jpg")), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        Form.setWindowIcon(icon)
        self.btn_arriba = QtGui.QPushButton(Form)
        self.btn_arriba.setGeometry(QtCore.QRect(70, 60, 98, 27))
        self.btn_arriba.setObjectName(_fromUtf8("btn_arriba"))
        self.btn_izq = QtGui.QPushButton(Form)
        self.btn_izq.setGeometry(QtCore.QRect(20, 90, 98, 27))
        self.btn_izq.setObjectName(_fromUtf8("btn_izq"))
        self.btn_der = QtGui.QPushButton(Form)
        self.btn_der.setGeometry(QtCore.QRect(120, 90, 98, 27))
        self.btn_der.setObjectName(_fromUtf8("btn_der"))
        self.btn_abajo = QtGui.QPushButton(Form)
        self.btn_abajo.setGeometry(QtCore.QRect(70, 120, 98, 27))
        self.btn_abajo.setObjectName(_fromUtf8("btn_abajo"))
        self.sld_vel = QtGui.QSlider(Form)
        self.sld_vel.setGeometry(QtCore.QRect(30, 230, 160, 29))
        self.sld_vel.setMaximum(90)
        self.sld_vel.setProperty("value", 45)
        self.sld_vel.setTracking(True)
        self.sld_vel.setOrientation(QtCore.Qt.Horizontal)
        self.sld_vel.setTickPosition(QtGui.QSlider.TicksBelow)
        self.sld_vel.setTickInterval(10)
        self.sld_vel.setObjectName(_fromUtf8("sld_vel"))
        self.ch_mantener = QtGui.QCheckBox(Form)
        self.ch_mantener.setGeometry(QtCore.QRect(40, 160, 161, 22))
        self.ch_mantener.setObjectName(_fromUtf8("ch_mantener"))
        self.lb_velocidad = QtGui.QLabel(Form)
        self.lb_velocidad.setGeometry(QtCore.QRect(80, 200, 66, 17))
        self.lb_velocidad.setObjectName(_fromUtf8("lb_velocidad"))
        self.btn_conectar = QtGui.QPushButton(Form)
        self.btn_conectar.setGeometry(QtCore.QRect(70, 10, 98, 27))
        self.btn_conectar.setObjectName(_fromUtf8("btn_conectar"))
        self.btn_salir = QtGui.QCommandLinkButton(Form)
        self.btn_salir.setGeometry(QtCore.QRect(70, 320, 81, 31))
        self.btn_salir.setObjectName(_fromUtf8("btn_salir"))
        self.btn_parar = QtGui.QPushButton(Form)
        self.btn_parar.setGeometry(QtCore.QRect(60, 280, 98, 27))
        self.btn_parar.setObjectName(_fromUtf8("btn_parar"))

        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        Form.setWindowTitle(_translate("Form", "Control Robot Movil", None))
        self.btn_arriba.setText(_translate("Form", "Up", None))
        self.btn_izq.setText(_translate("Form", "Left", None))
        self.btn_der.setText(_translate("Form", "Right", None))
        self.btn_abajo.setText(_translate("Form", "Down", None))
        self.ch_mantener.setText(_translate("Form", "Mantener direccion", None))
        self.lb_velocidad.setText(_translate("Form", "Velocidad", None))
        self.btn_conectar.setText(_translate("Form", "Conectar", None))
        self.btn_salir.setText(_translate("Form", "Salir", None))
        self.btn_parar.setText(_translate("Form", "Stop", None))


if __name__ == "__main__":
    import sys
    app = QtGui.QApplication(sys.argv)
    Form = QtGui.QWidget()
    ui = Ui_Form()
    ui.setupUi(Form)
    Form.show()
    sys.exit(app.exec_())

