# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'gui.ui'
#
# Created: Tue Apr 26 01:57:00 2016
#      by: PyQt4 UI code generator 4.10.4
#
# WARNING! All changes made in this file will be lost!
import math
import sys
import actionlib
import rospy

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from PyQt4 import QtCore, QtGui
import urdf_parser_py.urdf
import urdf_parser_py.xml_reflection
import kdl_parser_py.urdf
import PyKDL

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
        Form.resize(468, 325)

        self.gridLayout = QtGui.QGridLayout(Form)
        self.gridLayout.setObjectName(_fromUtf8("gridLayout"))

        self.error_text = QtGui.QLineEdit(Form)
        self.error_text.setObjectName(_fromUtf8("error_text"))
        self.gridLayout.addWidget(self.error_text, 10, 2, 1, 1)
        self.btn_traj = QtGui.QPushButton(Form)
        self.btn_traj.setObjectName(_fromUtf8("btn_traj"))
        self.gridLayout.addWidget(self.btn_traj, 20, 1, 1, 1)

        self.label = QtGui.QLabel(Form)
        self.label.setObjectName(_fromUtf8("label"))
        self.gridLayout.addWidget(self.label, 9, 2, 1, 1)
        self.label_8 = QtGui.QLabel(Form)
        self.label_8.setEnabled(True)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_8.sizePolicy().hasHeightForWidth())
        self.label_8.setSizePolicy(sizePolicy)
        self.label_8.setMinimumSize(QtCore.QSize(20, 20))
        self.label_8.setObjectName(_fromUtf8("label_8"))
        self.gridLayout.addWidget(self.label_8, 9, 0, 1, 1)
        self.position_x = QtGui.QLineEdit(Form)
        self.position_x.setObjectName(_fromUtf8("position_x"))
        self.gridLayout.addWidget(self.position_x, 10, 1, 1, 1)
        self.position_y = QtGui.QLineEdit(Form)
        self.position_y.setObjectName(_fromUtf8("position_y"))
        self.gridLayout.addWidget(self.position_y, 11, 1, 1, 1)
        self.position_z = QtGui.QLineEdit(Form)
        self.position_z.setObjectName(_fromUtf8("position_z"))
        self.gridLayout.addWidget(self.position_z, 12, 1, 1, 1)
        self.btn_ik = QtGui.QPushButton(Form)
        self.btn_ik.setObjectName(_fromUtf8("btn_ik"))
        self.gridLayout.addWidget(self.btn_ik, 20, 0, 1, 1)
        self.btn_salir = QtGui.QPushButton(Form)
        self.btn_salir.setObjectName(_fromUtf8("btn_salir"))
        self.gridLayout.addWidget(self.btn_salir, 20, 2, 1, 1)
        self.btn_conectar = QtGui.QPushButton(Form)
        self.btn_conectar.setObjectName(_fromUtf8("btn_conectar"))
        self.gridLayout.addWidget(self.btn_conectar, 12, 2, 1, 1)
        self.label_9 = QtGui.QLabel(Form)
        self.label_9.setEnabled(True)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_9.sizePolicy().hasHeightForWidth())
        self.label_9.setSizePolicy(sizePolicy)
        self.label_9.setMinimumSize(QtCore.QSize(20, 20))
        self.label_9.setObjectName(_fromUtf8("label_9"))
        self.gridLayout.addWidget(self.label_9, 11, 2, 1, 1)

        self.label_10 = QtGui.QLabel(Form)
        self.label_10.setEnabled(True)
        self.label_10.setSizePolicy(sizePolicy)
        self.label_10.setMinimumSize(QtCore.QSize(20, 20))
        self.label_10.setObjectName(_fromUtf8("label_10"))
        self.label_10.setAlignment(QtCore.Qt.AlignRight)
        self.gridLayout.addWidget(self.label_10, 10, 0, 1, 1)

        self.label_11 = QtGui.QLabel(Form)
        self.label_11.setEnabled(True)
        self.label_11.setSizePolicy(sizePolicy)
        self.label_11.setMinimumSize(QtCore.QSize(20, 20))
        self.label_11.setObjectName(_fromUtf8("label_11"))
        self.label_11.setAlignment(QtCore.Qt.AlignRight)
        self.gridLayout.addWidget(self.label_11, 11, 0, 1, 1)

        self.label_12 = QtGui.QLabel(Form)
        self.label_12.setEnabled(True)
        self.label_12.setSizePolicy(sizePolicy)
        self.label_12.setMinimumSize(QtCore.QSize(20, 20))
        self.label_12.setObjectName(_fromUtf8("label_12"))
        self.label_12.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        self.gridLayout.addWidget(self.label_12, 12, 0, 1, 1)

        self.text_joint = []
        for each in range(6): self.text_joint.append(QtGui.QLineEdit(Form))
        x = 0
        y = 2
        cont = 1
        for i in self.text_joint:
            i.setText(QtCore.QString.number(0))
            i.setObjectName(_fromUtf8("text_joint" + `cont`))
            if cont > 3 and y != 5:
                y = 5
                x = 0
            self.gridLayout.addWidget(i, y, x, 1, 1)
            x += 1
            #print cont
            cont += 1


        self.sliders = []
        for each in range(6): self.sliders.append(QtGui.QSlider(Form))
        x = 0
        y = 1
        cont = 1
        for i in self.sliders:
            i.setMinimum(-90)
            i.setMaximum(90)
            i.setValue(0)
            # self.sl.setTickPosition(QSlider.TicksBelow)
            i.setTickInterval(1)
            i.setSingleStep(1)
            i.setOrientation(QtCore.Qt.Horizontal)
            i.setObjectName(_fromUtf8("sl_joint" + `cont`))
            if cont > 3 and y!=4:
                y = 4
                x = 0
            self.gridLayout.addWidget(i, y, x, 1, 1)
            #print cont
            cont += 1
            x +=1

        self.sliders[0].valueChanged.connect(lambda: self.sl_cambia(0))
        self.sliders[1].valueChanged.connect(lambda: self.sl_cambia(1))
        self.sliders[2].valueChanged.connect(lambda: self.sl_cambia(2))
        self.sliders[3].valueChanged.connect(lambda: self.sl_cambia(3))
        self.sliders[4].valueChanged.connect(lambda: self.sl_cambia(4))
        self.sliders[5].valueChanged.connect(lambda: self.sl_cambia(5))
        self.text_joint[0].connect(self.text_joint[0], QtCore.SIGNAL("editingFinished()"), lambda: self.text_cambiado(0))
        self.text_joint[1].connect(self.text_joint[1], QtCore.SIGNAL("editingFinished()"), lambda: self.text_cambiado(1))
        self.text_joint[2].connect(self.text_joint[2], QtCore.SIGNAL("editingFinished()"), lambda: self.text_cambiado(2))
        self.text_joint[3].connect(self.text_joint[3], QtCore.SIGNAL("editingFinished()"), lambda: self.text_cambiado(3))
        self.text_joint[4].connect(self.text_joint[4], QtCore.SIGNAL("editingFinished()"), lambda: self.text_cambiado(4))
        self.text_joint[5].connect(self.text_joint[5], QtCore.SIGNAL("editingFinished()"), lambda: self.text_cambiado(5))

        self.btn_traj.clicked.connect(self.enviar_traj)
        self.btn_conectar.clicked.connect(self.crearCliente)

        etiquetas = []
        for each in range(6): etiquetas.append(QtGui.QLabel(Form))
        x = 0
        y = 0
        cont = 1
        for i in etiquetas:
            i.setObjectName(_fromUtf8("label" + `cont`))
            i.setEnabled(True)
            sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Minimum)
            sizePolicy.setHorizontalStretch(0)
            sizePolicy.setVerticalStretch(0)
            sizePolicy.setHeightForWidth(i.sizePolicy().hasHeightForWidth())
            i.setSizePolicy(sizePolicy)
            i.setMinimumSize(QtCore.QSize(20, 20))
            i.setText(_translate("Form", "Joint "+ `cont`, None))
            self.gridLayout.addWidget(i, y, x, 1, 1)
            if cont > 3 and y != 3:
                y = 3
                x = 0
            self.gridLayout.addWidget(i, y, x, 1, 1)
            cont += 1
            x += 1
            #print i


        self.retranslateUi(Form)
        self.cargarURDF()

        QtCore.QMetaObject.connectSlotsByName(Form)

        self.btn_salir.clicked.connect(exit)
        self.btn_ik.clicked.connect(self.enviar_ik)

    def retranslateUi(self, Form):
        Form.setWindowTitle(_translate("Form", "Control Robot", None))
        self.btn_traj.setText(_translate("Form", "Enviar Trayectoria", None))
        self.label.setText(_translate("Form", "Errores", None))
        self.label_8.setText(_translate("Form", "End-effector position", None))
        self.btn_ik.setText(_translate("Form", "Enviar IK", None))
        self.btn_salir.setText(_translate("Form", "Salir", None))
        self.btn_conectar.setText(_translate("Form", "Conectar", None))
        self.label_9.setText(_translate("Form", "Conectar Servidor", None))

        self.label_10.setText(_translate("Form", "Posicion x:", None))
        self.label_11.setText(_translate("Form", "Posicion y:", None))
        self.label_12.setText(_translate("Form", "Posicion z:", None))

    def sl_cambia(self,i):
        self.text_joint[i].setText(QtCore.QString.number(self.sliders[i].value()))

    def text_cambiado(self,i):
        self.sliders[i].setValue(QtCore.QVariant(self.text_joint[i].text()).toInt()[0])

    def enviar_traj(self):
        arm_goal = [self.sliders[0].value() * math.pi / 180, self.sliders[1].value() * math.pi / 180,
                    self.sliders[2].value() * math.pi / 180, self.sliders[3].value() * math.pi / 180,
                    self.sliders[4].value() * math.pi / 180, self.sliders[5].value() * math.pi / 180]
        # print type(self.sl_joint1.value()*math.pi/180)
        # self.crearCliente()
        self.enviarTrayectoria(arm_goal)

    def crearCliente(self):
        rospy.init_node('trajectory_demo')
        self.arm_client = actionlib.SimpleActionClient('joint_trajectory_action', FollowJointTrajectoryAction)
        print "Esperando al servidor"
        self.arm_client.wait_for_server()
        print "Conectado al servidor"

    def enviarTrayectoria(self, arm_goal1):
        # arm_goal = [0, 0, 0, 0, 0, 0]
        # arm_goal = [-0.3, 0.5, -1.0, 1.8, 0.3, 0.6]
        arm_joints = ['joint_1',
                      'joint_2',
                      'joint_3',
                      'joint_4',
                      'joint_5',
                      'joint_6']
        # Connect to the right arm trajectory action server
        rospy.loginfo('Waiting for right arm trajectory controller...')

        rospy.loginfo('...connected.')

        # Create a single-point arm trajectory with the arm_goal as the end-point
        arm_trajectory = JointTrajectory()
        arm_trajectory.joint_names = arm_joints
        arm_trajectory.points.append(JointTrajectoryPoint())
        arm_trajectory.points[0].positions = arm_goal1
        arm_trajectory.points[0].velocities = [0.0 for i in arm_joints]
        arm_trajectory.points[0].accelerations = [0.0 for i in arm_joints]
        arm_trajectory.points[0].time_from_start = rospy.Duration(1.5)

        rospy.loginfo('Moving the arm to goal position...')

        # Create an empty trajectory goal
        arm_goal = FollowJointTrajectoryGoal()

        # Set the trajectory component to the goal trajectory created above
        arm_goal.trajectory = arm_trajectory

        # Specify zero tolerance for the execution time
        arm_goal.goal_time_tolerance = rospy.Duration(0.0)

        # Send the goal to the action server
        self.arm_client.send_goal(arm_goal)

        # Wait for up to 5 seconds for the motion to complete
        self.arm_client.wait_for_result(rospy.Duration(5.0))
        self.actualizar_IK(arm_goal1)

        rospy.loginfo('...done')

    def cargarURDF(self):
        # filename = '/home/kaiser/WS_ROS/catkin_ws/src/abb_experimental-indigo-devel/abb_irb120_gazebo/urdf/modelo_exportado.urdf'
        # filename = '/home/kaiser/WS_ROS/catkin_ws/src/urdf_serp/urdf/IRB120_URDF_v2/robots/IRB120_URDF_v2.URDF'
        filename = '/home/kaiser/WS_ROS/catkin_ws/src/urdf_serp/urdf/irb120_peq/irb120_peq.urdf'
        robot = urdf_parser_py.urdf.URDF.from_xml_file(filename)

        (ok, tree) = kdl_parser_py.urdf.treeFromFile(filename)
        self.chain = tree.getChain("base_link", "link_6")
        print self.chain.getNrOfSegments()
        self.solverCinematicaDirecta = PyKDL.ChainFkSolverPos_recursive(self.chain)
        self.vik = PyKDL.ChainIkSolverVel_pinv(self.chain)
        self.ik = PyKDL.ChainIkSolverPos_NR(self.chain, self.solverCinematicaDirecta, self.vik)
        self.actualizar_IK([0.0,0.0,0.0,0.0,0.0,0.0])


    def actualizar_IK(self,posicion_efector):
        nj_izq = self.chain.getNrOfJoints()
        self.posicionInicial = PyKDL.JntArray(nj_izq)
        self.posicionInicial[0] = posicion_efector[0]
        self.posicionInicial[1] = posicion_efector[1]
        self.posicionInicial[2] = posicion_efector[2]
        self.posicionInicial[3] = posicion_efector[3]
        self.posicionInicial[4] = posicion_efector[4]
        self.posicionInicial[5] = posicion_efector[5]
        print "Forward kinematics"
        finalFrame = PyKDL.Frame()
        self.solverCinematicaDirecta.JntToCart(self.posicionInicial, finalFrame)
        print "Rotational Matrix of the final Frame: "
        print  finalFrame.M
        print "End-effector position: ", finalFrame.p
        self.position_x.setText(QtCore.QString.number(finalFrame.p[0]))
        self.position_y.setText(QtCore.QString.number(finalFrame.p[1]))
        self.position_z.setText(QtCore.QString.number(finalFrame.p[2]))

    def enviar_ik(self):
        print "Inverse Kinematics"
        q_init = self.posicionInicial  # initial angles
        desiredFrame = PyKDL.Frame(PyKDL.Vector(QtCore.QString.toFloat(self.position_x.text())[0],
                                                QtCore.QString.toFloat(self.position_y.text())[0],
                                                QtCore.QString.toFloat(self.position_z.text())[0]))
        print "Desired Position: ", desiredFrame.p
        q_out = PyKDL.JntArray(6)
        self.ik.CartToJnt(q_init, desiredFrame, q_out)
        print "Output angles in rads: ", q_out
        arm_goal = [q_out[0],q_out[1],q_out[2],q_out[3],q_out[4],q_out[5]]
        self.enviarTrayectoria(arm_goal)



if __name__ == "__main__":
    import sys
    app = QtGui.QApplication(sys.argv)
    Form = QtGui.QWidget()
    ui = Ui_Form()
    ui.setupUi(Form)
    Form.show()
    sys.exit(app.exec_())

