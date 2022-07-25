# Contains PyQt5 UIs

from PyQt5 import QtCore, QtGui, QtWidgets, QtOpenGL
from PyQt5.Qt import Qt
from OpenGL import GL, GLU
from OpenGL.GL.shaders import compileProgram, compileShader
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from numpy import cos, sin, pi

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import ctypes
import pyrr
import time

from GUI.helpers import Eframe2Glframe, normalize

vertex_src = """
# version 330
layout(location = 0) in vec3 a_position;
layout(location = 1) in vec3 a_color;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

out vec4 v_color;
void main()
{
    gl_Position = projection * view * model * vec4(a_position, 1.0);
    v_color = vec4(a_color, 1);
}
"""

fragment_src = """
#version 330 core
out vec4 FragColor;
in vec4 v_color;

void main()
{
    FragColor = v_color;
}   
"""

FPS = 30        # frames per second


class GLWidgetSimulation(QtOpenGL.QGLWidget):
    def __init__(self, parent=None):
        self.parent = parent
        QtOpenGL.QGLWidget.__init__(self, parent)

        self.dt = 1 / FPS       # time steps
        self.simulation = None  # simulation object

        # object orientation
        self.theta = 0

        # camera config
        self.mousePosInit = np.array([0, 0])
        self.mousePosFin = np.array([0, 0])
        self.yaw = -122
        self.pitch = -23
        self.cameraPos = np.array([0.5, 0.5, 0.8])
        self.cameraFront = np.array([-0.487, -0.39, -0.782])
        self.cameraUp = np.array([0, 1, 0])

    def initializeGL(self):
        self.qglClearColor(QtGui.QColor(0, 191, 255))       # Initialize blue screen
        GL.glEnable(GL.GL_DEPTH_TEST)                       # Enable depth testing
        self.setUpdatesEnabled(True)

        self.shader = compileProgram(compileShader(vertex_src, GL.GL_VERTEX_SHADER),
                                     compileShader(fragment_src, GL.GL_FRAGMENT_SHADER))

        self.initGeometry()

        self.modelMatLoc =   GL.glGetUniformLocation(self.shader, "model")
        self.viewMatLoc = GL.glGetUniformLocation(self.shader, "view")
        self.projectionMatLoc = GL.glGetUniformLocation(self.shader, "projection")

    def resizeGL(self, width, height):
        height = height if height !=0 else 1
        GL.glViewport(0, 0, width, height)
        GL.glMatrixMode(GL.GL_PROJECTION)
        GL.glLoadIdentity()
        self.aspect = width / float(height)
        GLU.gluPerspective(45.0, self.aspect, 1.0, 100.0)
        GL.glMatrixMode(GL.GL_MODELVIEW)

    def paintGL(self):
        GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)
        GL.glUseProgram(self.shader)
        # model matrix
        theta = float(np.radians(self.theta))
        model = pyrr.matrix33.create_from_x_rotation(theta)                                 # transformation in E-frame
        model = pyrr.Matrix44(Eframe2Glframe(model))                                # E-frame to GL-frame
        # view matrix
        view = pyrr.matrix44.create_look_at(self.cameraPos, self.cameraPos + self.cameraFront, self.cameraUp)
        # projection matrix
        projection = pyrr.matrix44.create_perspective_projection_matrix(45, self.aspect, 0.1, 100)
        # update shader uniforms
        GL.glUniformMatrix4fv(self.modelMatLoc, 1, GL.GL_FALSE, model)
        GL.glUniformMatrix4fv(self.viewMatLoc, 1, GL.GL_FALSE, view)
        GL.glUniformMatrix4fv(self.projectionMatLoc, 1, GL.GL_FALSE, projection)
        # draw objects
        GL.glBindVertexArray(self.VAO[0])
        GL.glDrawElements(GL.GL_TRIANGLES, 6, GL.GL_UNSIGNED_INT, None)
        GL.glBindVertexArray(self.VAO[1])
        GL.glDrawArrays(GL.GL_LINES, 0, 6)

        # simulate body-fixed frame
        if self.simulation:
            model = self.simulation.simulate()

            GL.glUniformMatrix4fv(self.modelMatLoc, 1, GL.GL_FALSE, model)
            GL.glBindVertexArray(self.VAO[2])
            GL.glDrawArrays(GL.GL_LINES, 0, 6)

    def initGeometry(self):
        # Construction of VAO to draw ground square and coordinate system
        square_vertices = np.array([1, 1, 0, 0.34, 0.49, 0.275,                            # Earth-fixed reference frame
                                    1, -1, 0, 0.34, 0.49, 0.275,
                                    -1, -1, 0, 0.34, 0.49, 0.275,
                                    -1, 1, 0, 0.34, 0.49, 0.275], dtype=np.float32)
        square_indices = np.array([0, 1, 3,
                                   1, 2, 3])

        coord_vertices_earth = np.array([0, 0, 0, 1, 0, 0,                                 # Earth-fixed reference frame
                                        1, 0, 0, 1, 0, 0,
                                        0, 0, 0, 0, 1, 0,
                                        0, 1, 0, 0, 1, 0,
                                        0, 0, 0, 0, 0, 1,
                                        0, 0, -1, 0, 0, 1], dtype=np.float32)

        coord_vertices_body = np.array([0, 0, 0, 0, 0, 1,                                  # Body-fixed reference frame
                                        0, 0, 0.1, 0, 0, 1,
                                        0, 0, 0, 1, 0, 0,
                                        0.1, 0, 0, 1, 0, 0,
                                        0, 0, 0, 0, 1, 0,
                                        0, 0.1, 0, 0, 1, 0], dtype=np.float32)

        coord_vertices = [coord_vertices_earth, coord_vertices_body]

        self.VAO = GL.glGenVertexArrays(3)
        # Create ground surface
        GL.glBindVertexArray(self.VAO[0])

        square_VBO = GL.glGenBuffers(1)
        GL.glBindBuffer(GL.GL_ARRAY_BUFFER, square_VBO)
        GL.glBufferData(GL.GL_ARRAY_BUFFER, square_vertices.nbytes, square_vertices, GL.GL_STATIC_DRAW)

        square_EBO = GL.glGenBuffers(1)
        GL.glBindBuffer(GL.GL_ELEMENT_ARRAY_BUFFER, square_EBO)
        GL.glBufferData(GL.GL_ELEMENT_ARRAY_BUFFER, square_indices.nbytes, square_indices, GL.GL_STATIC_DRAW)

        GL.glEnableVertexAttribArray(0)
        GL.glVertexAttribPointer(0, 3, GL.GL_FLOAT, GL.GL_FALSE, square_vertices.itemsize * 6, ctypes.c_void_p(0))

        GL.glEnableVertexAttribArray(1)
        GL.glVertexAttribPointer(1, 3, GL.GL_FLOAT, GL.GL_FALSE, square_vertices.itemsize * 6, ctypes.c_void_p(12))

        # Create coordinate system
        for i in [1, 2]:
            GL.glBindVertexArray(self.VAO[i])

            global_coord_VBO = GL.glGenBuffers(1)
            GL.glBindBuffer(GL.GL_ARRAY_BUFFER, global_coord_VBO)
            GL.glBufferData(GL.GL_ARRAY_BUFFER, coord_vertices[i - 1].nbytes, coord_vertices[i - 1], GL.GL_STATIC_DRAW)

            GL.glEnableVertexAttribArray(0)
            GL.glVertexAttribPointer(0, 3, GL.GL_FLOAT, GL.GL_FALSE, coord_vertices[i - 1].itemsize * 6, ctypes.c_void_p(0))

            GL.glEnableVertexAttribArray(1)
            GL.glVertexAttribPointer(1, 3, GL.GL_FLOAT, GL.GL_FALSE, coord_vertices[i - 1].itemsize * 6, ctypes.c_void_p(12))

    def processKeyInput(self, event):
        cameraSpeed = 2.5 * self.dt
        if event.key() == Qt.Key_W:
            self.cameraPos = self.cameraPos + cameraSpeed * self.cameraFront
        if event.key() == Qt.Key_S:
            self.cameraPos = self.cameraPos - cameraSpeed * self.cameraFront
        if event.key() == Qt.Key_A:
            self.cameraPos = self.cameraPos - normalize(np.cross(self.cameraFront, self.cameraUp)) * cameraSpeed
        if event.key() == Qt.Key_D:
            self.cameraPos = self.cameraPos + normalize(np.cross(self.cameraFront, self.cameraUp)) * cameraSpeed
        if event.key() == Qt.Key_Shift:
            self.cameraPos = self.cameraPos + np.array([0, cameraSpeed, 0])
        if event.key() == Qt.Key_Control:
            self.cameraPos = self.cameraPos + np.array([0, -cameraSpeed, 0])

    def processMouseInput(self, event):
        sensitivity = 0.05
        offset = (self.mousePosFin - self.mousePosInit) * sensitivity
        self.mousePosInit = self.mousePosFin
        self.yaw -= offset[0]
        self.pitch += offset[1]
        if self.pitch > 89:
            self.pitch = 89
        if self.pitch < -89:
            self.pitch = -89
        directionx = np.cos(np.radians(self.yaw)) * np.cos(np.radians(self.pitch))
        directiony = np.sin(np.radians(self.pitch))
        directionz = np.sin(np.radians(self.yaw)) * np.cos(np.radians(self.pitch))
        direction = np.array([directionx, directiony, directionz])
        self.cameraFront = normalize(direction)


class MplCanvas(FigureCanvasQTAgg):

    def __init__(self, parent=None, width=5, height=4, dpi=100):
        #fig, self.axes = plt.subplots(figsize=(width, height), dpi=dpi)
        self.fig = plt.Figure()
        super().__init__(self.fig)
        self.setParent(parent)

    def plot2D(self, x, y, xlabel, ylabel):
        self.fig.clear()
        self.axes = self.fig.add_subplot()
        self.axes.plot(x, y)
        self.axes.grid()
        self.axes.set_xlabel(xlabel)
        self.axes.set_ylabel(ylabel)
        self.draw()

    def plot3d(self, x, y, z, xlabel, ylabel, zlabel):
        self.fig.clear()
        self.axes = self.fig.add_subplot(111, projection="3d")
        self.axes.plot(x, y, -z)
        self.axes.set_xlabel(xlabel)
        self.axes.set_ylabel(ylabel)
        self.axes.set_zlabel(zlabel)
        self.draw()


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1280, 720)
        MainWindow.setMinimumSize(1280, 720)
        font = QtGui.QFont()
        font.setBold(False)
        font.setWeight(50)
        MainWindow.setFont(font)
        MainWindow.setTabShape(QtWidgets.QTabWidget.Rounded)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")

        self.verticalLayout = QtWidgets.QHBoxLayout(self.centralwidget)
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout.setObjectName("verticalLayout")

        # Tab widget
        self.tabWidget = QtWidgets.QTabWidget(self.centralwidget)
        font = QtGui.QFont()
        font.setPointSize(10)
        self.tabWidget.setFont(font)
        self.tabWidget.setTabShape(QtWidgets.QTabWidget.Triangular)
        self.tabWidget.setObjectName("tabWidget")


        # First tab (simulation)
        self.tab_1 = QtWidgets.QWidget()
        self.tab_1.setObjectName("tab_1")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.tab_1)
        self.horizontalLayout.setContentsMargins(10, 10, 10, 10)
        self.horizontalLayout.setObjectName("horizontalLayout")

        # Group box: 3D view
        self.TrajectorySim = QtWidgets.QGroupBox(self.tab_1)
        font = QtGui.QFont()
        font.setPointSize(12)
        self.TrajectorySim.setFont(font)
        self.TrajectorySim.setObjectName("TrajectorySim")
        self.horizontalLayout.addWidget(self.TrajectorySim)
        self.horizontalLayout.setStretch(0, 10)
        self.verticalLayout_3Dbox = QtWidgets.QVBoxLayout(self.TrajectorySim)
        self.verticalLayout_3Dbox.setContentsMargins(10, 10, 10, 10)
        self.verticalLayout_3Dbox.setObjectName("verticalLayout_3Dbox")

        ## GL window
        self.openGLWidget = GLWidgetSimulation(self.TrajectorySim)
        self.openGLWidget.setObjectName("openGLWidget")
        self.verticalLayout_3Dbox.addWidget(self.openGLWidget)
        self.verticalLayout_3Dbox.setStretch(0, 19)

        ## Simulation time control/indication
        self.horizontalLayout_timelineWidget = QtWidgets.QWidget(self.TrajectorySim)
        self.horizontalLayout_timelineWidget.setObjectName("horizontalLayout_timelineWidget")
        self.horizontalLayout_timeline = QtWidgets.QHBoxLayout(self.horizontalLayout_timelineWidget)
        self.horizontalLayout_timeline.setContentsMargins(5, 5, 5, 5)
        self.horizontalLayout_timeline.setObjectName("verticalLayout_3Dbox")
        self.verticalLayout_3Dbox.addWidget(self.horizontalLayout_timelineWidget)
        self.verticalLayout_3Dbox.setStretch(1, 1)

        ### play button
        self.pushButton = QtWidgets.QPushButton(self.TrajectorySim)
        self.pushButton.setObjectName("pushButton")
        self.horizontalLayout_timeline.addWidget(self.pushButton)
        self.horizontalLayout_timeline.setStretch(1, 1)

        ### simulation timeline
        self.horizontalSlider = QtWidgets.QSlider(self.TrajectorySim)
        self.horizontalSlider.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider.setObjectName("horizontalSlider")
        self.horizontalLayout_timeline.addWidget(self.horizontalSlider)
        self.horizontalLayout_timeline.setStretch(1, 9)


        # Group box: States
        self.States = QtWidgets.QGroupBox(self.tab_1)
        font = QtGui.QFont()
        font.setPointSize(12)
        self.States.setFont(font)
        self.States.setObjectName("States")
        self.horizontalLayout.addWidget(self.States)
        self.horizontalLayout.setStretch(1, 3)
        self.verticalLayout_Statesbox = QtWidgets.QVBoxLayout(self.States)
        self.verticalLayout_Statesbox.setContentsMargins(10, 10, 10, 10)
        self.verticalLayout_Statesbox.setObjectName("verticalLayout_Statesbox")

        ## group box 1: earth-fixed frame
        self.earthFrame = QtWidgets.QGroupBox(self.States)
        font = QtGui.QFont()
        font.setPointSize(10)
        self.earthFrame.setFont(font)
        self.earthFrame.setObjectName("earthFrame")
        self.verticalLayout_Statesbox.addWidget(self.earthFrame)
        self.verticalLayout_Statesbox.setStretch(0, 6)

        self.pitch_angle_label = QtWidgets.QLabel(self.earthFrame)
        self.pitch_angle_label.setGeometry(QtCore.QRect(20, 60, 81, 21))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.pitch_angle_label.setFont(font)
        self.pitch_angle_label.setObjectName("pitch_angle_label")

        self.yaw_angle_lable = QtWidgets.QLabel(self.earthFrame)
        self.yaw_angle_lable.setGeometry(QtCore.QRect(20, 90, 81, 21))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.yaw_angle_lable.setFont(font)
        self.yaw_angle_lable.setObjectName("yaw_angle_lable")

        self.x_position_label = QtWidgets.QLabel(self.earthFrame)
        self.x_position_label.setGeometry(QtCore.QRect(20, 130, 81, 21))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.x_position_label.setFont(font)
        self.x_position_label.setObjectName("x_position_label")

        self.roll_angle_label = QtWidgets.QLabel(self.earthFrame)
        self.roll_angle_label.setGeometry(QtCore.QRect(20, 30, 81, 21))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.roll_angle_label.setFont(font)
        self.roll_angle_label.setObjectName("roll_angle_label")

        self.y_position_label = QtWidgets.QLabel(self.earthFrame)
        self.y_position_label.setGeometry(QtCore.QRect(20, 160, 81, 21))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.y_position_label.setFont(font)
        self.y_position_label.setObjectName("y_position_label")

        self.z_position_label = QtWidgets.QLabel(self.earthFrame)
        self.z_position_label.setGeometry(QtCore.QRect(20, 190, 81, 21))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.z_position_label.setFont(font)
        self.z_position_label.setObjectName("z_position_label")

        self.line2 = QtWidgets.QFrame(self.earthFrame)
        self.line2.setGeometry(QtCore.QRect(10, 110, 221, 16))
        self.line2.setFrameShape(QtWidgets.QFrame.HLine)
        self.line2.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line2.setObjectName("line2")

        self.roll_angle = QtWidgets.QLabel(self.earthFrame)
        self.roll_angle.setGeometry(QtCore.QRect(110, 30, 61, 21))
        self.roll_angle.setFrameShape(QtWidgets.QFrame.Box)
        self.roll_angle.setFrameShadow(QtWidgets.QFrame.Plain)
        self.roll_angle.setText("")
        self.roll_angle.setObjectName("roll_angle")

        self.roll_angle_unit = QtWidgets.QLabel(self.earthFrame)
        self.roll_angle_unit.setGeometry(QtCore.QRect(190, 30, 31, 21))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.roll_angle_unit.setFont(font)
        self.roll_angle_unit.setObjectName("roll_angle_unit")

        self.pitch_angle = QtWidgets.QLabel(self.earthFrame)
        self.pitch_angle.setGeometry(QtCore.QRect(110, 60, 61, 21))
        self.pitch_angle.setFrameShape(QtWidgets.QFrame.Box)
        self.pitch_angle.setFrameShadow(QtWidgets.QFrame.Plain)
        self.pitch_angle.setText("")
        self.pitch_angle.setObjectName("pitch_angle")

        self.yaw_angle = QtWidgets.QLabel(self.earthFrame)
        self.yaw_angle.setGeometry(QtCore.QRect(110, 90, 61, 21))
        self.yaw_angle.setFrameShape(QtWidgets.QFrame.Box)
        self.yaw_angle.setFrameShadow(QtWidgets.QFrame.Plain)
        self.yaw_angle.setText("")
        self.yaw_angle.setObjectName("yaw_angle")

        self.pitch_angle_unit = QtWidgets.QLabel(self.earthFrame)
        self.pitch_angle_unit.setGeometry(QtCore.QRect(190, 60, 31, 21))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.pitch_angle_unit.setFont(font)
        self.pitch_angle_unit.setObjectName("pitch_angle_unit")

        self.yaw_angle_unit = QtWidgets.QLabel(self.earthFrame)
        self.yaw_angle_unit.setGeometry(QtCore.QRect(190, 90, 31, 21))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.yaw_angle_unit.setFont(font)
        self.yaw_angle_unit.setObjectName("yaw_angle_unit")

        self.z_position = QtWidgets.QLabel(self.earthFrame)
        self.z_position.setGeometry(QtCore.QRect(110, 190, 61, 21))
        self.z_position.setFrameShape(QtWidgets.QFrame.Box)
        self.z_position.setFrameShadow(QtWidgets.QFrame.Plain)
        self.z_position.setText("")
        self.z_position.setObjectName("z_position")

        self.x_position_unit = QtWidgets.QLabel(self.earthFrame)
        self.x_position_unit.setGeometry(QtCore.QRect(190, 130, 21, 21))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.x_position_unit.setFont(font)
        self.x_position_unit.setObjectName("x_position_unit")

        self.y_position_unit = QtWidgets.QLabel(self.earthFrame)
        self.y_position_unit.setGeometry(QtCore.QRect(190, 160, 21, 21))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.y_position_unit.setFont(font)
        self.y_position_unit.setObjectName("y_position_unit")

        self.x_position = QtWidgets.QLabel(self.earthFrame)
        self.x_position.setGeometry(QtCore.QRect(110, 130, 61, 21))
        self.x_position.setFrameShape(QtWidgets.QFrame.Box)
        self.x_position.setFrameShadow(QtWidgets.QFrame.Plain)
        self.x_position.setText("")
        self.x_position.setObjectName("x_position")

        self.z_position_unit = QtWidgets.QLabel(self.earthFrame)
        self.z_position_unit.setGeometry(QtCore.QRect(190, 190, 21, 21))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.z_position_unit.setFont(font)
        self.z_position_unit.setObjectName("z_position_unit")

        self.y_position = QtWidgets.QLabel(self.earthFrame)
        self.y_position.setGeometry(QtCore.QRect(110, 160, 61, 21))
        self.y_position.setFrameShape(QtWidgets.QFrame.Box)
        self.y_position.setFrameShadow(QtWidgets.QFrame.Plain)
        self.y_position.setText("")
        self.y_position.setObjectName("y_position")

        ## group box 2: body frame
        self.bodyFrame = QtWidgets.QGroupBox(self.States)
        font = QtGui.QFont()
        font.setPointSize(10)
        self.bodyFrame.setFont(font)
        self.bodyFrame.setObjectName("bodyFrame")
        self.verticalLayout_Statesbox.addWidget(self.bodyFrame)
        self.verticalLayout_Statesbox.setStretch(1, 6)

        self.pitch_rate_label = QtWidgets.QLabel(self.bodyFrame)
        self.pitch_rate_label.setGeometry(QtCore.QRect(20, 60, 71, 21))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.pitch_rate_label.setFont(font)
        self.pitch_rate_label.setObjectName("pitch_rate_label")

        self.yaw_rate_lable = QtWidgets.QLabel(self.bodyFrame)
        self.yaw_rate_lable.setGeometry(QtCore.QRect(20, 90, 71, 21))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.yaw_rate_lable.setFont(font)
        self.yaw_rate_lable.setObjectName("yaw_rate_lable")

        self.x_velocity_label = QtWidgets.QLabel(self.bodyFrame)
        self.x_velocity_label.setGeometry(QtCore.QRect(20, 130, 81, 21))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.x_velocity_label.setFont(font)
        self.x_velocity_label.setObjectName("x_velocity_label")

        self.roll_rate_label = QtWidgets.QLabel(self.bodyFrame)
        self.roll_rate_label.setGeometry(QtCore.QRect(20, 30, 71, 21))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.roll_rate_label.setFont(font)
        self.roll_rate_label.setObjectName("roll_rate_label")

        self.y_velocity_label = QtWidgets.QLabel(self.bodyFrame)
        self.y_velocity_label.setGeometry(QtCore.QRect(20, 160, 81, 21))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.y_velocity_label.setFont(font)
        self.y_velocity_label.setObjectName("y_velocity_label")

        self.z_velocity_label = QtWidgets.QLabel(self.bodyFrame)
        self.z_velocity_label.setGeometry(QtCore.QRect(20, 190, 81, 21))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.z_velocity_label.setFont(font)
        self.z_velocity_label.setObjectName("z_velocity_label")

        self.line1 = QtWidgets.QFrame(self.bodyFrame)
        self.line1.setGeometry(QtCore.QRect(10, 110, 221, 16))
        self.line1.setFrameShape(QtWidgets.QFrame.HLine)
        self.line1.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line1.setObjectName("line1")

        self.pitch_rate = QtWidgets.QLabel(self.bodyFrame)
        self.pitch_rate.setGeometry(QtCore.QRect(110, 60, 61, 21))
        self.pitch_rate.setFrameShape(QtWidgets.QFrame.Box)
        self.pitch_rate.setFrameShadow(QtWidgets.QFrame.Plain)
        self.pitch_rate.setText("")
        self.pitch_rate.setObjectName("pitch_rate")

        self.roll_rate_unit = QtWidgets.QLabel(self.bodyFrame)
        self.roll_rate_unit.setGeometry(QtCore.QRect(190, 30, 51, 21))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.roll_rate_unit.setFont(font)
        self.roll_rate_unit.setObjectName("roll_rate_unit")

        self.roll_rate = QtWidgets.QLabel(self.bodyFrame)
        self.roll_rate.setGeometry(QtCore.QRect(110, 30, 61, 21))
        self.roll_rate.setFrameShape(QtWidgets.QFrame.Box)
        self.roll_rate.setFrameShadow(QtWidgets.QFrame.Plain)
        self.roll_rate.setText("")
        self.roll_rate.setObjectName("roll_rate")

        self.yaw_rate = QtWidgets.QLabel(self.bodyFrame)
        self.yaw_rate.setGeometry(QtCore.QRect(110, 90, 61, 21))
        self.yaw_rate.setFrameShape(QtWidgets.QFrame.Box)
        self.yaw_rate.setFrameShadow(QtWidgets.QFrame.Plain)
        self.yaw_rate.setText("")
        self.yaw_rate.setObjectName("yaw_rate")

        self.pitch_rate_unit = QtWidgets.QLabel(self.bodyFrame)
        self.pitch_rate_unit.setGeometry(QtCore.QRect(190, 60, 51, 21))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.pitch_rate_unit.setFont(font)
        self.pitch_rate_unit.setObjectName("pitch_rate_unit")

        self.yaw_rate_unit = QtWidgets.QLabel(self.bodyFrame)
        self.yaw_rate_unit.setGeometry(QtCore.QRect(190, 90, 51, 21))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.yaw_rate_unit.setFont(font)
        self.yaw_rate_unit.setObjectName("yaw_rate_unit")

        self.z_velocity_unit = QtWidgets.QLabel(self.bodyFrame)
        self.z_velocity_unit.setGeometry(QtCore.QRect(190, 190, 31, 21))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.z_velocity_unit.setFont(font)
        self.z_velocity_unit.setObjectName("z_velocity_unit")

        self.x_velocity_unit = QtWidgets.QLabel(self.bodyFrame)
        self.x_velocity_unit.setGeometry(QtCore.QRect(190, 130, 31, 21))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.x_velocity_unit.setFont(font)
        self.x_velocity_unit.setObjectName("x_velocity_unit")

        self.x_velocity = QtWidgets.QLabel(self.bodyFrame)
        self.x_velocity.setGeometry(QtCore.QRect(110, 130, 61, 21))
        self.x_velocity.setFrameShape(QtWidgets.QFrame.Box)
        self.x_velocity.setFrameShadow(QtWidgets.QFrame.Plain)
        self.x_velocity.setText("")
        self.x_velocity.setObjectName("x_velocity")

        self.y_velocity_unit = QtWidgets.QLabel(self.bodyFrame)
        self.y_velocity_unit.setGeometry(QtCore.QRect(190, 160, 31, 21))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.y_velocity_unit.setFont(font)
        self.y_velocity_unit.setObjectName("y_velocity_unit")

        self.y_velocity = QtWidgets.QLabel(self.bodyFrame)
        self.y_velocity.setGeometry(QtCore.QRect(110, 160, 61, 21))
        self.y_velocity.setFrameShape(QtWidgets.QFrame.Box)
        self.y_velocity.setFrameShadow(QtWidgets.QFrame.Plain)
        self.y_velocity.setText("")
        self.y_velocity.setObjectName("y_velocity")

        self.z_velocity = QtWidgets.QLabel(self.bodyFrame)
        self.z_velocity.setGeometry(QtCore.QRect(110, 190, 61, 21))
        self.z_velocity.setFrameShape(QtWidgets.QFrame.Box)
        self.z_velocity.setFrameShadow(QtWidgets.QFrame.Plain)
        self.z_velocity.setText("")
        self.z_velocity.setObjectName("z_velocity")

        ## group box 3: inputs
        self.Inputs = QtWidgets.QGroupBox(self.States)
        font = QtGui.QFont()
        font.setPointSize(10)
        self.Inputs.setFont(font)
        self.Inputs.setObjectName("Inputs")
        self.verticalLayout_Statesbox.addWidget(self.Inputs)
        self.verticalLayout_Statesbox.setStretch(2, 3)

        self.gimbal2_label = QtWidgets.QLabel(self.Inputs)
        self.gimbal2_label.setGeometry(QtCore.QRect(10, 50, 111, 21))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.gimbal2_label.setFont(font)
        self.gimbal2_label.setObjectName("gimbal2_label")

        self.rotational_label = QtWidgets.QLabel(self.Inputs)
        self.rotational_label.setGeometry(QtCore.QRect(10, 80, 111, 21))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.rotational_label.setFont(font)
        self.rotational_label.setObjectName("rotational_label")

        self.gimbal1_label = QtWidgets.QLabel(self.Inputs)
        self.gimbal1_label.setGeometry(QtCore.QRect(10, 20, 121, 21))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.gimbal1_label.setFont(font)
        self.gimbal1_label.setObjectName("gimbal1_label")

        self.gimbal1_unit = QtWidgets.QLabel(self.Inputs)
        self.gimbal1_unit.setGeometry(QtCore.QRect(190, 20, 31, 21))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.gimbal1_unit.setFont(font)
        self.gimbal1_unit.setObjectName("gimbal1_unit")

        self.gimbal2_unit = QtWidgets.QLabel(self.Inputs)
        self.gimbal2_unit.setGeometry(QtCore.QRect(190, 50, 31, 21))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.gimbal2_unit.setFont(font)
        self.gimbal2_unit.setObjectName("gimbal2_unit")

        self.gimbal1 = QtWidgets.QLabel(self.Inputs)
        self.gimbal1.setGeometry(QtCore.QRect(130, 20, 51, 21))
        self.gimbal1.setFrameShape(QtWidgets.QFrame.Box)
        self.gimbal1.setFrameShadow(QtWidgets.QFrame.Plain)
        self.gimbal1.setText("")
        self.gimbal1.setObjectName("gimbal1")

        self.gimbal2 = QtWidgets.QLabel(self.Inputs)
        self.gimbal2.setGeometry(QtCore.QRect(130, 50, 51, 21))
        self.gimbal2.setFrameShape(QtWidgets.QFrame.Box)
        self.gimbal2.setFrameShadow(QtWidgets.QFrame.Plain)
        self.gimbal2.setText("")
        self.gimbal2.setObjectName("gimbal2")

        self.rotational = QtWidgets.QLabel(self.Inputs)
        self.rotational.setGeometry(QtCore.QRect(130, 80, 51, 21))
        self.rotational.setFrameShape(QtWidgets.QFrame.Box)
        self.rotational.setFrameShadow(QtWidgets.QFrame.Plain)
        self.rotational.setText("")
        self.rotational.setObjectName("rotational")

        self.rotational_unit = QtWidgets.QLabel(self.Inputs)
        self.rotational_unit.setGeometry(QtCore.QRect(190, 80, 51, 21))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.rotational_unit.setFont(font)
        self.rotational_unit.setObjectName("rotational_unit")

        ## Simulate button
        self.simulate = QtWidgets.QPushButton(self.States)
        font = QtGui.QFont()
        font.setPointSize(12)
        self.simulate.setFont(font)
        self.simulate.setObjectName("simulate")
        self.verticalLayout_Statesbox.addWidget(self.simulate)
        self.verticalLayout_Statesbox.setStretch(3, 0)

        self.tabWidget.addTab(self.tab_1, "")



        # Second tab (analyse)
        self.tab_2 = QtWidgets.QWidget()
        self.tab_2.setObjectName("tab_2")
        self.tabWidget.addTab(self.tab_2, "")
        self.tab2_VerticalLayout = QtWidgets.QVBoxLayout(self.tab_2)
        self.tab2_VerticalLayout.setContentsMargins(10, 10, 10, 10)
        self.tab2_VerticalLayout.setObjectName("tab2_VerticalLayout")

        ## Group box: button box
        self.buttonBox = QtWidgets.QGroupBox(self.tab_2)
        self.tab2_VerticalLayout.addWidget(self.buttonBox)
        self.tab2_VerticalLayout.setStretch(0, 0)
        self.buttonBoxLayout = QtWidgets.QHBoxLayout(self.buttonBox)
        self.buttonBoxLayout.setObjectName("buttonBoxLayout")

        ### Button 1: earth-fixed
        self.button1 = QtWidgets.QPushButton(self.buttonBox)
        self.button1.setObjectName("button1")
        self.buttonBoxLayout.addWidget(self.button1)
        self.buttonBoxLayout.setStretch(0, 1)

        ### Button 2: body-fixed
        self.button2 = QtWidgets.QPushButton(self.buttonBox)
        self.button2.setObjectName("button2")
        self.buttonBoxLayout.addWidget(self.button2)
        self.buttonBoxLayout.setStretch(1, 1)

        ### Button 3: inputs
        self.button3 = QtWidgets.QPushButton(self.buttonBox)
        self.button3.setObjectName("button1")
        self.buttonBoxLayout.addWidget(self.button3)
        self.buttonBoxLayout.setStretch(2, 1)

        ### Button 4: trajectory
        self.button4 = QtWidgets.QPushButton(self.buttonBox)
        self.button4.setObjectName("button4")
        self.buttonBoxLayout.addWidget(self.button4)
        self.buttonBoxLayout.setStretch(3, 1)


        ## Group box: graph box
        self.graphBox = QtWidgets.QStackedWidget(self.tab_2)
        self.tab2_VerticalLayout.addWidget(self.graphBox)
        self.tab2_VerticalLayout.setStretch(1, 1)

        ### Page 1: earth-fixed
        self.page_1 = QtWidgets.QWidget(self.graphBox)
        self.page_1.setObjectName("page_1")
        self.gridLayout_1 = QtWidgets.QGridLayout(self.page_1)
        self.gridLayout_1.setContentsMargins(0, 0, 0, 0)
        self.gridLayout_1.setHorizontalSpacing(10)
        self.gridLayout_1.setVerticalSpacing(10)
        self.gridLayout_1.setObjectName("gridLayout_1")

        self.graph1_1 = MplCanvas(self.page_1)
        self.graph1_1.setObjectName("graph1_1")
        self.gridLayout_1.addWidget(self.graph1_1, 0, 0, 1, 1)
        self.graph1_2 = MplCanvas(self.page_1)
        self.graph1_2.setObjectName("graph1_2")
        self.gridLayout_1.addWidget(self.graph1_2, 0, 2, 1, 1)
        self.graph1_3 = MplCanvas(self.page_1)
        self.graph1_3.setObjectName("graph1_3")
        self.gridLayout_1.addWidget(self.graph1_3, 0, 1, 1, 1)
        self.graph1_4 = MplCanvas(self.page_1)
        self.graph1_4.setObjectName("graph1_4")
        self.gridLayout_1.addWidget(self.graph1_4, 1, 0, 1, 1)
        self.graph1_5 = MplCanvas(self.page_1)
        self.graph1_5.setObjectName("graph1_5")
        self.gridLayout_1.addWidget(self.graph1_5, 1, 1, 1, 1)
        self.graph1_6 = MplCanvas(self.page_1)
        self.graph1_6.setObjectName("graph1_6")
        self.gridLayout_1.addWidget(self.graph1_6, 1, 2, 1, 1)

        self.graphBox.addWidget(self.page_1)

        ### Page 2: body-fixed
        self.page_2 = QtWidgets.QWidget(self.graphBox)
        self.page_2.setObjectName("page_2")
        self.gridLayout_2 = QtWidgets.QGridLayout(self.page_2)
        self.gridLayout_2.setContentsMargins(0, 0, 0, 0)
        self.gridLayout_2.setHorizontalSpacing(10)
        self.gridLayout_2.setVerticalSpacing(10)
        self.gridLayout_2.setObjectName("gridLayout_2")

        self.graph2_1 = MplCanvas(self.page_2)
        self.graph2_1.setObjectName("graph2_1")
        self.gridLayout_2.addWidget(self.graph2_1, 0, 0, 1, 1)
        self.graph2_2 = MplCanvas(self.page_2)
        self.graph2_2.setObjectName("graph2_2")
        self.gridLayout_2.addWidget(self.graph2_2, 0, 2, 1, 1)
        self.graph2_3 = MplCanvas(self.page_2)
        self.graph2_3.setObjectName("graph2_3")
        self.gridLayout_2.addWidget(self.graph2_3, 0, 1, 1, 1)
        self.graph2_4 = MplCanvas(self.page_2)
        self.graph2_4.setObjectName("graph2_4")
        self.gridLayout_2.addWidget(self.graph2_4, 1, 0, 1, 1)
        self.graph2_5 = MplCanvas(self.page_1)
        self.graph2_5.setObjectName("graph2_5")
        self.gridLayout_2.addWidget(self.graph2_5, 1, 1, 1, 1)
        self.graph2_6 = MplCanvas(self.page_2)
        self.graph2_6.setObjectName("graph2_6")
        self.gridLayout_2.addWidget(self.graph2_6, 1, 2, 1, 1)

        self.graphBox.addWidget(self.page_2)

        ### Page 3: inputs
        self.page_3 = QtWidgets.QWidget(self.graphBox)
        self.page_3.setObjectName("page_3")
        self.gridLayout_3 = QtWidgets.QGridLayout(self.page_3)
        self.gridLayout_3.setContentsMargins(0, 0, 0, 0)
        self.gridLayout_3.setHorizontalSpacing(10)
        self.gridLayout_3.setVerticalSpacing(10)
        self.gridLayout_3.setObjectName("gridLayout_3")

        self.graph3_1 = MplCanvas(self.page_3)
        self.graph3_1.setObjectName("graph3_1")
        self.gridLayout_3.addWidget(self.graph3_1, 0, 0, 1, 1)
        self.graph3_2 = MplCanvas(self.page_3)
        self.graph3_2.setObjectName("graph3_2")
        self.gridLayout_3.addWidget(self.graph3_2, 0, 2, 1, 1)
        self.graph3_3 = MplCanvas(self.page_3)
        self.graph3_3.setObjectName("graph3_3")
        self.gridLayout_3.addWidget(self.graph3_3, 0, 1, 1, 1)
        self.graph3_4 = MplCanvas(self.page_3)
        self.graph3_4.setObjectName("graph3_4")
        self.gridLayout_3.addWidget(self.graph3_4, 1, 0, 1, 1)
        self.graph3_5 = MplCanvas(self.page_3)
        self.graph3_5.setObjectName("graph3_5")
        self.gridLayout_3.addWidget(self.graph3_5, 1, 1, 1, 1)
        self.graph3_6 = MplCanvas(self.page_3)
        self.graph3_6.setObjectName("graph3_6")
        self.gridLayout_3.addWidget(self.graph3_6, 1, 2, 1, 1)

        self.graphBox.addWidget(self.page_3)

        ### Page 4: trajectory
        self.page_4 = QtWidgets.QWidget(self.graphBox)
        self.page_4.setObjectName("page_4")
        self.gridLayout_4 = QtWidgets.QGridLayout(self.page_4)
        self.gridLayout_4.setContentsMargins(0, 0, 0, 0)
        self.gridLayout_4.setObjectName("gridLayout_4")

        self.graph4_1 = MplCanvas(self.page_4)
        self.graph4_1.setObjectName("graph4_1")
        self.gridLayout_4.addWidget(self.graph4_1, 0, 0, 1, 1)

        self.graphBox.addWidget(self.page_4)



        # third tab (settings)
        self.tab_3 = QtWidgets.QWidget()
        self.tab_3.setObjectName("tab_3")
        self.tabWidget.addTab(self.tab_3, "")

        self.verticalLayout.addWidget(self.tabWidget)
        MainWindow.setCentralWidget(self.centralwidget)


        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1280, 21))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)

        self.retranslateUi(MainWindow)
        self.tabWidget.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "Simulator"))
        self.TrajectorySim.setTitle(_translate("MainWindow", "3D view"))
        self.pushButton.setText(_translate("MainWindow", "Play"))
        self.States.setWhatsThis(_translate("MainWindow", "<html><head/><body><p>Press to simulate drone trajectory</p><p><br/></p></body></html>"))
        self.States.setTitle(_translate("MainWindow", "States"))
        self.earthFrame.setTitle(_translate("MainWindow", "Earth Fixed Frame"))
        self.pitch_angle_label.setText(_translate("MainWindow", "Pitch angle"))
        self.yaw_angle_lable.setText(_translate("MainWindow", "Yaw angle"))
        self.x_position_label.setText(_translate("MainWindow", "X-position"))
        self.roll_angle_label.setText(_translate("MainWindow", "Roll angle"))
        self.y_position_label.setText(_translate("MainWindow", "Y-position"))
        self.z_position_label.setText(_translate("MainWindow", "Z-position"))
        self.roll_angle_unit.setText(_translate("MainWindow", "deg"))
        self.pitch_angle_unit.setText(_translate("MainWindow", "deg"))
        self.yaw_angle_unit.setText(_translate("MainWindow", "deg"))
        self.x_position_unit.setText(_translate("MainWindow", "m"))
        self.y_position_unit.setText(_translate("MainWindow", "m"))
        self.z_position_unit.setText(_translate("MainWindow", "m"))
        self.bodyFrame.setTitle(_translate("MainWindow", "Body Frame"))
        self.pitch_rate_label.setText(_translate("MainWindow", "Pitch rate"))
        self.yaw_rate_lable.setText(_translate("MainWindow", "Yaw rate"))
        self.x_velocity_label.setText(_translate("MainWindow", "X-velocity"))
        self.roll_rate_label.setText(_translate("MainWindow", "Roll rate"))
        self.y_velocity_label.setText(_translate("MainWindow", "Y-velocity"))
        self.z_velocity_label.setText(_translate("MainWindow", "Z-velocity"))
        self.roll_rate_unit.setText(_translate("MainWindow", "deg/s"))
        self.pitch_rate_unit.setText(_translate("MainWindow", "deg/s"))
        self.yaw_rate_unit.setText(_translate("MainWindow", "deg/s"))
        self.z_velocity_unit.setText(_translate("MainWindow", "m/s"))
        self.x_velocity_unit.setText(_translate("MainWindow", "m/s"))
        self.y_velocity_unit.setText(_translate("MainWindow", "m/s"))
        self.Inputs.setTitle(_translate("MainWindow", "Inputs"))
        self.gimbal2_label.setText(_translate("MainWindow", "Y-gimbal angle"))
        self.rotational_label.setText(_translate("MainWindow", "Rot. rate prop."))
        self.gimbal1_label.setText(_translate("MainWindow", "X-gimbal angle"))
        self.gimbal1_unit.setText(_translate("MainWindow", "deg"))
        self.gimbal2_unit.setText(_translate("MainWindow", "deg"))
        self.rotational_unit.setText(_translate("MainWindow", "deg/s"))
        self.simulate.setWhatsThis(_translate("MainWindow", "Press to simulate drone trajectory"))
        self.simulate.setText(_translate("MainWindow", "Simulate"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_1), _translate("MainWindow", "Simulation"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_2), _translate("MainWindow", "Analyse"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_3), _translate("MainWindow", "Settings"))
        self.button1.setText(_translate("MainWindow", "Earth-Fixed States"))
        self.button2.setText(_translate("MainWindow", "Body-Fixed States"))
        self.button3.setText(_translate("MainWindow", "Inputs"))
        self.button4.setText(_translate("MainWindow", "Trajectory"))


class simulation():
    def __init__(self):
        # Run simulation executable

        # Load simulation data
        self.x_vec = pd.read_csv("data/state.csv", header=None)           # Simulation states
        self.u_vec = pd.read_csv("data/input.csv", header=None)           # Simulation inputs
        self.t_vec = np.genfromtxt('data/time.csv', delimiter=',')        # Simulation time

        # Current data point
        self.t, self.x, self.y, self.z, self.phi, self.theta, self.psi = np.hstack(( self.t_vec[0],
                                                                                     np.hstack(( self.x_vec[0][6:9],
                                                                                                 self.x_vec[0][0:3] )) ))
        self.p, self.q, self.r, self.u, self.v, self.w = np.hstack(( self.x_vec[0][3:6],
                                                                     self.x_vec[0][9:12] ))
        self.theta1, self.theta2, self.omega, self.thetaRate1, self.thetaRate2, self.omegaRate = self.u_vec[0]

        self.R = self.rotationMatrix()

    def rotationMatrix(self):
        R = np.array([[cos(self.theta) * cos(self.psi), sin(self.phi) * sin(self.theta) * cos(self.psi) - cos(self.phi) * sin(self.psi),
                         sin(self.phi) * sin(self.psi) + cos(self.phi) * sin(self.theta) * cos(self.psi)],
                        [cos(self.theta) * sin(self.psi), cos(self.phi) * cos(self.psi) + sin(self.phi) * sin(self.theta) * sin(self.psi),
                         cos(self.phi) * sin(self.theta) * sin(self.psi) - sin(self.phi) * cos(self.psi)],
                        [-sin(self.theta), sin(self.phi) * cos(self.theta), cos(self.phi) * cos(self.theta)]])
        return R

    def simulate(self):
        index = np.where(np.around(self.t_vec, 2) == np.round(self.t, 2))[0][0]
        self.x, self.y, self.z, self.phi, self.theta, self.psi = np.hstack((self.x_vec[index][6:9], self.x_vec[index][0:3]))
        self.p, self.q, self.r, self.u, self.v, self.w = np.hstack(( self.x_vec[index][3:6], self.x_vec[index][9:12] ))
        self.theta1, self.theta2, self.omega, self.thetaRate1, self.thetaRate2, self.omegaRate = self.u_vec[index]
        scale = np.amax(np.abs(np.array(self.x_vec[:][8])))*2
        rotationMatrix = pyrr.Matrix44(Eframe2Glframe(self.rotationMatrix()))
        translationMatrix = pyrr.matrix44.create_from_translation(Eframe2Glframe(self.x_vec[index][6:9] / scale))
        model = rotationMatrix.dot(translationMatrix)
        return model


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        QtWidgets.QMainWindow.__init__(self)

        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        # Window update loop
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(lambda: self.updateFrame())
        self.timer.start(int(1 / FPS * 1000))

        # Handle simulation actions
        self.ui.simulate.clicked.connect(lambda: self.runSimulation())
        self.ui.pushButton.clicked.connect(lambda: self.playSimulation())
        self.ui.horizontalSlider.sliderPressed.connect(lambda: self.playSimulation(True))

        # Handle analysis buttons
        self.ui.button1.clicked.connect(lambda: self.graphSwitch(0))
        self.ui.button2.clicked.connect(lambda: self.graphSwitch(1))
        self.ui.button3.clicked.connect(lambda: self.graphSwitch(2))
        self.ui.button4.clicked.connect(lambda: self.graphSwitch(3))

        # Keep track of states
        self.start = 0                  # start time
        self.offset = 0                 # offset time
        self.max = 0                    # max time
        self.pressed = False            # mouse button pressed
        self.play = False               # simulation playing

    def updateFrame(self):
        # Update simulation
        if self.ui.openGLWidget.simulation:
            if self.play:
                t = time.time() - self.start + self.offset
                self.ui.openGLWidget.simulation.t = t if t <= self.max else self.max
                self.ui.horizontalSlider.setValue(int(100*round(self.ui.openGLWidget.simulation.t, 2)))
            else:
                self.ui.openGLWidget.simulation.t = self.ui.horizontalSlider.value()/100
                self.offset = self.ui.horizontalSlider.value()/100

            # Update state and inputs
            self.ui.roll_angle.setText(str(round(self.ui.openGLWidget.simulation.phi*180/pi, 2)))
            self.ui.pitch_angle.setText(str(round(self.ui.openGLWidget.simulation.theta*180/pi, 2)))
            self.ui.yaw_angle.setText(str(round(self.ui.openGLWidget.simulation.psi*180/pi, 2)))
            self.ui.roll_rate.setText(str(round(self.ui.openGLWidget.simulation.p*180/pi, 2)))
            self.ui.pitch_rate.setText(str(round(self.ui.openGLWidget.simulation.q*180/pi, 2)))
            self.ui.yaw_rate.setText(str(round(self.ui.openGLWidget.simulation.r*180/pi, 2)))
            self.ui.x_position.setText(str(round(self.ui.openGLWidget.simulation.x, 2)))
            self.ui.y_position.setText(str(round(self.ui.openGLWidget.simulation.y, 2)))
            self.ui.z_position.setText(str(round(self.ui.openGLWidget.simulation.z, 2)))
            self.ui.x_velocity.setText(str(round(self.ui.openGLWidget.simulation.u, 2)))
            self.ui.y_velocity.setText(str(round(self.ui.openGLWidget.simulation.v, 2)))
            self.ui.z_velocity.setText(str(round(self.ui.openGLWidget.simulation.w, 2)))

            self.ui.gimbal1.setText(str(round(self.ui.openGLWidget.simulation.theta1*180/pi, 2)))
            self.ui.gimbal2.setText(str(round(self.ui.openGLWidget.simulation.theta2*180/pi, 2)))
            self.ui.rotational.setText(str(round(self.ui.openGLWidget.simulation.omega)))

            self.ui.openGLWidget.repaint()        # Update OpenGL widget

    def runSimulation(self):
        self.ui.openGLWidget.simulation = simulation()                                      # Create simulation object
        self.max = self.ui.openGLWidget.simulation.t_vec[-1]
        self.ui.horizontalSlider.setMaximum(int(100*round(self.max, 2)))
        self.generateGraphs()

    def playSimulation(self, bool=False):
        if self.ui.openGLWidget.simulation:                                                 # Simulation object exists
            if not self.play and not bool:                                                  # Simulation paused
                self.play = True
                self.start = time.time()
            elif self.play or bool:                                                         # Simulation playing
                self.play = False
                self.offset = time.time() - self.start + self.offset

    def graphSwitch(self, idx):
        self.ui.graphBox.setCurrentIndex(idx)

    def generateGraphs(self):
        t = self.ui.openGLWidget.simulation.t_vec
        x = np.array(self.ui.openGLWidget.simulation.x_vec)
        u = np.array(self.ui.openGLWidget.simulation.u_vec)
        # Page 1: earth-fixed graphs
        self.ui.graph1_1.plot2D(t, x[:][6], "Time [s]", "X position [m]")
        self.ui.graph1_2.plot2D(t, x[:][7], "Time [s]", "Y position [m]")
        self.ui.graph1_3.plot2D(t, -x[:][8], "Time [s]", "Z position [m]")

        self.ui.graph1_4.plot2D(t, x[:][0]*180/pi, "Time [s]", "Roll angle [deg]")
        self.ui.graph1_5.plot2D(t, x[:][1]*180/pi, "Time [s]", "Pitch angle [deg]")
        self.ui.graph1_6.plot2D(t, x[:][2]*180/pi, "Time [s]", "Yaw angle [deg]")

        # Page 2: body-fixed graphs
        self.ui.graph2_1.plot2D(t, x[:][9], "Time [s]", "X velocity [m/s]")
        self.ui.graph2_2.plot2D(t, x[:][10], "Time [s]", "Y velocity [m/s]")
        self.ui.graph2_3.plot2D(t, x[:][11], "Time [s]", "Z velocity [m/s]")

        self.ui.graph2_4.plot2D(t, x[:][3]*180/pi, "Time [s]", "Roll rate [deg/s]")
        self.ui.graph2_5.plot2D(t, x[:][4]*180/pi, "Time [s]", "Pitch rate [deg/s]")
        self.ui.graph2_6.plot2D(t, x[:][5]*180/pi, "Time [s]", "Yaw rate [deg/s]")

        # Page 3: inputs
        self.ui.graph3_1.plot2D(t, u[:][0]*180/pi, "Time [s]", "X gimbal angle [deg]")
        self.ui.graph3_3.plot2D(t, u[:][1]*180/pi, "Time [s]", "Y gimbal angle [deg]")
        self.ui.graph3_2.plot2D(t, u[:][2], "Time [s]", "Prop. rotational velocity [deg/s]")

        self.ui.graph3_4.plot2D(t, u[:][3]*180/pi, "Time [s]", "X gimbal rate [deg/s]")
        self.ui.graph3_5.plot2D(t, u[:][4]*180/pi, "Time [s]", "Y gimbal rate [deg/s]")
        self.ui.graph3_6.plot2D(t, u[:][5], "Time [s]", "Prop. rotational acceleration [deg/s^2]")

        # Page 4: trajectory
        self.ui.graph4_1.plot3d(x[:][6], x[:][7], x[:][8], "X position [m]", "Y position [m]", "Z position [m]")

    def keyReleaseEvent(self, event):
        self.ui.openGLWidget.processKeyInput(event)

    def mousePressEvent(self, event):
        self.pressed = True
        self.ui.openGLWidget.mousePosInit = np.array([event.x(), event.y()])

    def mouseReleaseEvent(self, event):
        self.pressed = False
        self.ui.openGLWidget.mousePosFin = np.array([event.x(), event.y()])

    def mouseMoveEvent(self, event):
        if self.pressed:
            self.ui.openGLWidget.mousePosFin = np.array([event.x(), event.y()])
            self.ui.openGLWidget.processMouseInput(event)


