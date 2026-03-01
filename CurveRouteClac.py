from math import *

def findCurveRoute(canDistance , trackLength):  #Distance from outer can to car (Meter), track length(Meter)

    AD = 1 - canDistance
    BD = trackLength / 2

    ang_DAB = atan(BD / AD)
    AB = sqrt(BD ** 2 + AD ** 2)
    ang_BCA = pi - 2 * ang_DAB

    AC = (AB * sin(ang_DAB)) / sin(ang_BCA)
    ang_BCE = 2 * ang_BCA

    ang_BEA = pi / 2 - ang_DAB

    print(round(AB, 5), round(AD, 5), round(AC, 5), round(degrees(ang_DAB), 5)) 

    return round(AC,6), round(degrees(ang_BCE), 5), round(degrees(ang_BEA), 5) # R, theta, start angle


if __name__ == "__main__":
    print(findCurveRoute(0.2,7))
    