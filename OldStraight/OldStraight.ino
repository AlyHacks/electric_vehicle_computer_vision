
/* REMEMBER TO COMMENT OUT THE LOGGING FOR BEST PERFORMANCE*/

// Pin Definition
#define STEP_PIN 9
#define DIR_PIN  12 
#define EN_PIN   8   
#define BUTTON_PIN 2

// parameters
const float DISTANCE_M = 7.2;
const float WHEEL_DIAMETER_M = 0.0762;
const float targetTime = 11.5;
const int STEPS_PER_REV = 1600; 

const float CALIBRATION_FACTOR = 1.2035;

// variables
float totalSteps;
float baseFreq;
unsigned long startTime;
unsigned long lastPrintTime = 0; 
bool isRunning = false;

const float ACCEL_TIME = 5.0;
const float RAMP_DOWN_START = 9.5;

void setup() {
    Serial.begin(115200); 
    
    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(EN_PIN, OUTPUT);
    pinMode(BUTTON_PIN, INPUT);

    digitalWrite(EN_PIN, HIGH); 
    digitalWrite(DIR_PIN, LOW); 

    totalSteps = (DISTANCE_M / (PI * WHEEL_DIAMETER_M)) * STEPS_PER_REV * CALIBRATION_FACTOR;
    baseFreq = totalSteps / targetTime;

    Serial.println("Ready");
    Serial.print("Target distance"); Serial.print(DISTANCE_M); Serial.println(" meters");
    Serial.print("Aprox Base Freq"); Serial.print(baseFreq); Serial.println(" Hz");
    Serial.println("Wait for button");

    setupTimer1(); 
}

void loop() {
    // check for button
    if (!isRunning) {
        if (digitalRead(BUTTON_PIN) == LOW) { 
            delay(50); 
            if (digitalRead(BUTTON_PIN) == LOW) {
                // button must be pushed down for 200ms
                unsigned long confirmStart = millis();
                bool stillPressed = true;
                while (millis() - confirmStart < 200) {
                    if (digitalRead(BUTTON_PIN) == HIGH) {
                        stillPressed = false;
                        break;
                    }
                }

                if (stillPressed) {
                    // wait for release
                    while(digitalRead(BUTTON_PIN) == LOW); 
                    delay(200); 
                    startTask();
                }
            }
        }
    }
    if (isRunning) {
        float elapsed = (millis() - startTime) / 1000.0;
        float currentFreq = 0;

        if (elapsed < targetTime) {
            // S curve acceleration
            if (elapsed < ACCEL_TIME) {
                currentFreq = baseFreq * (elapsed / ACCEL_TIME);
            } else if (elapsed > RAMP_DOWN_START) {
                float rampDuration = targetTime - RAMP_DOWN_START;
                currentFreq = baseFreq * (1.0 - ((elapsed - RAMP_DOWN_START) / rampDuration));
            } else {
                currentFreq = baseFreq;
            }
            
            updateTimer1(currentFreq);

            
            // print status every 500 ms
            if (millis() - lastPrintTime > 500) {
                lastPrintTime = millis();
                float progress = (elapsed / targetTime) * 100.0;
                Serial.print("Progress: "); Serial.print(progress, 1); Serial.print("% | ");
                Serial.print("Time: "); Serial.print(elapsed, 1); Serial.print("s | ");
                Serial.print("Current Freq: "); Serial.print(currentFreq, 0); Serial.println(" Hz");
            }
            

        } else {
            stopMotors();
        }
    }
}

void startTask() {
    Serial.println("\n>>> MOTION START");
    digitalWrite(EN_PIN, LOW); 
    delay(10);
    startTime = millis();
    isRunning = true;
    TCCR1B |= (1 << CS11); 
}

void stopMotors() {
    isRunning = false;
    TCCR1B &= ~(1 << CS11); 
    digitalWrite(STEP_PIN, LOW);
    digitalWrite(EN_PIN, HIGH);
    
    Serial.println(">>> MOTION END");
    Serial.println("Wait for next.");
    
}

void setupTimer1() {
    TCCR1A = (1 << COM1A0); 
    TCCR1B = (1 << WGM12);  
}

void updateTimer1(float freq) {
    if (freq < 10) return;
    uint32_t ocrVal = 1000000UL / (uint32_t)freq - 1;
    if (ocrVal > 65535) ocrVal = 65535;
    OCR1A = (uint16_t)ocrVal;
}