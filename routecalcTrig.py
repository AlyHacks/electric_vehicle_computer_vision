import math
from math import sin, cos, atan2, sqrt, acos, atan

def degree(rad):
    return rad*180/math.pi

def length(pot1, pot2):
    length = sqrt((pot1[0] - pot2[0])**2 + (pot1[1] + pot2[1])**2)
    return length

def main():
    #Point Coordinates (cm)
    A = (0,0)
    B = (1000,0)
    C = (500, 100)
    D = (500, 70) # Dy can change

    #Car dimension
    w = 20
    L = 30
    R = 10
    T_required = 50


    W1 = (D[0]-L-4, D[1]+w/2+3)
    W2 = (D[0]+L+4, D[1]+w/2+3)

    #Calculate angles
    Ang1 = degree(atan(W1[1]/W1[0]))
    Ang2 = 90 - Ang1

    turn1 = 360 - Ang1
    turn2 = Ang2
    turn3 = Ang1

    #Calculate path length, speed and time
    L1 = length(A, W1)
    L2 = length(W1, W2)
    L3 = length(W2, B)

    Total_L = L1 + L2 + L3

    V = Total_L / T_required

    T1 = L1 / V
    T2 = L2 / V
    T3 = L3 / V

    Total_T = T1 + T2 + T3

    print(f"Total path length: {Total_L:.3f} \nSpeed: {V:.3f}")
    print(f"Turn 1: {turn1:.3f}, time: {T1:.3f}. \nTurn 2: {turn2:.3f}, time: {T2:.3f}. \nTurn 3: {turn3:.3f}, time: {T3:.3f}.")
    print(f"Total time: {Total_T}")

if __name__ == "__main__":
    main()