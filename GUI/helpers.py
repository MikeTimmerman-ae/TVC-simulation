import numpy as np
from numpy import cos, sin


def Bframe2Eframe(attitude, axisB):
    phi,theta,psi = attitude

    Mbe = np.array([[cos(theta)*cos(psi), sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi), sin(phi)*sin(psi)+cos(phi)*sin(theta)*cos(psi)],
                    [cos(theta)*sin(psi), cos(phi)*cos(psi)+sin(phi)*sin(theta)*sin(psi), cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi)],
                    [-sin(theta), sin(phi)*cos(theta), cos(phi)*cos(theta)]])

    axisE = Mbe.dot(axisB)

    return axisE


def Eframe2GlframeRotation(axisE):

    Megl = np.array([[1, 0, 0],
                     [0, 0, 1],
                     [0, 1, 0]])

    axisGL = axisE.dot(Megl)

    return axisGL


def Eframe2GlframeTranslation(axisE):

    Megl = np.array([[1, 0, 0],
                     [0, 0, 1],
                     [0, -1, 0]])

    axisGL = axisE.dot(Megl)

    return axisGL


def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0:
        return v
    return v/norm
