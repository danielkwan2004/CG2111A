import numpy as np
import time

def getDelta(thetaRad, distance):
    """TODO: Docstring for projectCoordintates.
    """
    return distance * np.cos(thetaRad), distance * np.sin(thetaRad)
    
def projectCoordinates(x, y, thetaRad, distance):
    """_summary_

    Args:
        x (_type_): _description_
        y (_type_): _description_
        thetaRad (_type_): _description_
        distance (_type_): _description_
    """
    dx, dy = getDelta(thetaRad, distance)
    return x + dx, y + dy

def rotateAboutOrigin(x,y, thetaRad, OriginX=0, OriginY=0):
    """_summary_

    Args:
        x (_type_): _description_
        y (_type_): _description_
        thetaRad (_type_): _description_
    """
    cosine = np.cos(thetaRad)
    sine = np.sin(thetaRad)

    # Translate the point to the origin
    x -= OriginX
    y -= OriginY

    # Perform the rotation
    xRot = x * cosine - y * sine
    yRot = x * sine + y * cosine

    # Translate the point back
    xRot += OriginX
    yRot += OriginY

    return xRot, yRot