#include <Servo.h>
#include <math.h>

double degree(double rad) {
    return rad * 180.0 / M_PI;
}

double lengthCalc(double p1x, double p1y, double p2x, double p2y) {
    return sqrt(pow(p1x - p2x, 2) + pow(p1y - p2y, 2));
}

void setup() {
    Serial.begin(9600);

    // Point Coordinates (cm)
    double Ax = 0,    Ay = 0;
    double Bx = 1000, By = 0;
    double Cx = 500,  Cy = 100;
    double Dx = 500,  Dy = 70;   // Dy can be adjusted dynamically

    // Car dimensions
    double w = 20;
    double L = 30;
    double R = 10;
    double T_required = 50;

    // Wheel positions
    double W1x = Dx - L - 4;
    double W1y = Dy + w/2 + 3;

    double W2x = Dx + L + 4;
    double W2y = Dy + w/2 + 3;

    // Angle Calculations
    double Ang1 = degree(atan(W1y / W1x));
    double Ang2 = 90 - Ang1;

    double turn1 = 360 - Ang1;
    double turn2 = Ang2;
    double turn3 = Ang1;

    // Path Lengths
    double L1 = lengthCalc(Ax, Ay, W1x, W1y);
    double L2 = lengthCalc(W1x, W1y, W2x, W2y);
    double L3 = lengthCalc(W2x, W2y, Bx, By);

    double Total_L = L1 + L2 + L3;

    // Speed and Time
    double V = Total_L / T_required;

    double T1 = L1 / V;
    double T2 = L2 / V;
    double T3 = L3 / V;

    double Total_T = T1 + T2 + T3;

    // Output 
    Serial.println("Summary");
    Serial.print("Total path length: "); Serial.println(Total_L, 3);
    Serial.print("Speed: "); Serial.println(V, 3);

    Serial.print("Turn 1: "); Serial.print(turn1, 3);
    Serial.print(", Time: "); Serial.println(T1, 3);

    Serial.print("Turn 2: "); Serial.print(turn2, 3);
    Serial.print(", Time: "); Serial.println(T2, 3);

    Serial.print("Turn 3: "); Serial.print(turn3, 3);
    Serial.print(", Time: "); Serial.println(T3, 3);

    Serial.print("Total Time: "); Serial.println(Total_T, 3);
}

void loop() {
    //nothing needed
}

