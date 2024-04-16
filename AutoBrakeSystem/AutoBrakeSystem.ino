// Define pins
const int TRIG_PIN = 5;
const int ECHO_PIN = 4;
const int RELAY_PIN_BRAKE = 3;
const int RELAY_PIN_MAIN_MOTOR = 2;

// Function prototypes
void initializePins();
void triggerSensor();
unsigned long getEchoDuration();
int calculateDistance(unsigned long duration);
void controlRelay(int distance);
void printDistance(int distance);

void setup() {
    Serial.begin(9600);
    initializePins();
}

void loop() {
    // Triggering the sensor
    triggerSensor();

    // Measuring the duration
    unsigned long duration = getEchoDuration();

    // Calculating distance in centimeters
    int distance = calculateDistance(duration);

    // Controlling relay based on distance
    controlRelay(distance);

    // Output distance to serial monitor
    // printDistance(distance);

    delay(200);
}

void initializePins() {
    pinMode(TRIG_PIN, OUTPUT);
  	pinMode(ECHO_PIN, INPUT);
  
    pinMode(RELAY_PIN_BRAKE, OUTPUT);
    pinMode(RELAY_PIN_MAIN_MOTOR, OUTPUT);   

}

void triggerSensor() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(5);
    digitalWrite(TRIG_PIN, LOW);
}

unsigned long getEchoDuration() {
    return pulseIn(ECHO_PIN, HIGH);
}

int calculateDistance(unsigned long duration) {
    return duration * 0.034 / 2;
}

void controlRelay(int distance) {
    if (distance < 100) {
        digitalWrite(RELAY_PIN_BRAKE, HIGH);
        digitalWrite(RELAY_PIN_MAIN_MOTOR, HIGH);
    } else {
        digitalWrite(RELAY_PIN_BRAKE, LOW);        
      	digitalWrite(RELAY_PIN_MAIN_MOTOR, LOW);

    }
}

void printDistance(int distance) {
    Serial.print(distance);
    Serial.println(" cm");
}
