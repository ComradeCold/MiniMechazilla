#include <Servo.h>
#include <math.h>
#include <WiFiS3.h>


// Wi-Fi credentials
const char* ssid = "YourPhone";
const char* password = "12345678";


// A: changed variable names i.e. hipLOffset -> LHoffset, changed order and grouped for each leg, made adjustments so servos line up for straight legs in neutral position
#define LHoffset 105 // GOOD, used to be 105,102
#define LKoffset 101 //GOOD, used to be 155
#define LFoffset 127 // good 85


// A: right side might be good (right referring to a person looking at the legs)
#define RHoffset 81 // used to be 81 //
#define RKoffset 79 // GOOD? // used to be 20
#define RFoffset 85 //good, used to be 85

// === LEG DIMENSIONS ===
#define l1 5.0
#define l2 5.7

// === FIXED WALKING HEIGHT ===
#define stepHeight 10  //
#define stepClearance 1

// === SERVO OBJECTS ===
Servo leftHip, leftKnee, leftFoot;
Servo rightHip, rightKnee, rightFoot;

// === SMOOTH SERVO STARTUP ===
void rampToOffset(Servo &servo, int offset, bool invert = false) {
  int start = 90;
  int target = invert ? 2 * offset : offset;

  if (target > start) {
    for (int i = start; i <= target; i++) {
      servo.write(i);
      delay(10);
    }
  } else {
    for (int i = start; i >= target; i--) {
      servo.write(i);
      delay(10);
    }
  }
}

// === APPLY ANGLES TO LEG ===
void updateServoPos(int hipDeg, int kneeDeg, int footDeg, char leg) {
  if (leg == 'l') {
    leftHip.write(LHoffset - hipDeg);
    leftKnee.write(LKoffset - kneeDeg);
    leftFoot.write(LFoffset + footDeg); // Adjusted this to LFoffset - footDeg for better control
  } else if (leg == 'r') {
    rightHip.write(RHoffset + hipDeg);
    rightKnee.write(RKoffset + kneeDeg);
    rightFoot.write(footDeg);
  }
}

void pos(float x, float z, char leg) {
  float hipRad2 = atan(x / z);
  float z2 = z / cos(hipRad2);

  float hipRad1 = acos((sq(l1) + sq(z2) - sq(l2)) / (2 * l1 * z2));
  float kneeRad = PI - acos((sq(l1) + sq(l2) - sq(z2)) / (2 * l1 * l2));
  float footRad = PI / 2 + hipRad2 - acos((sq(l2) + sq(z2) - sq(l1)) / (2 * l2 * z2));

  // Convert to degrees
  float hipDeg = (hipRad1 + hipRad2) * (180 / PI);
  float kneeDeg = kneeRad * (180 / PI);
  float footDeg = footRad * (180 / PI);

  // Increase hip rotation for a larger turn (outward angle)
  hipDeg *= 1.2;  // This will increase the hip angle by 10%, making a larger turn 1.1

  // You can also adjust knee and foot angles for balance, if needed:
  kneeDeg *= 1.05; // Slightly increase knee angle for more leg extension 1.05
  footDeg *= 1.05; // Slight increase in foot angle for more contact with the ground 1.05

  // Limit foot range to prevent excessive movement
  if (footDeg > 60) footDeg = 60;
  if (footDeg < -60) footDeg = -60;

  updateServoPos(hipDeg, kneeDeg, footDeg, leg);
}

void squat() {
  /*
  // A: changed variable names i.e. hipLOffset -> LHoffset, changed order and grouped for each leg, made adjustments so servos line up for straight legs in neutral position
#define LHoffset 105 // GOOD, used to be 105,102
#define LKoffset 101 //GOOD, used to be 155
#define LFoffset 127 // good 85


// A: right side might be good (right referring to a person looking at the legs)
#define RHoffset 81 // used to be 81 //
#define RKoffset 79 // GOOD? // used to be 20
#define RFoffset 85 //good, used to be 85
*/
  rampToOffset(leftHip, 55);
  delay(500);

  rampToOffset(leftKnee, 86);
  delay(500);

  rampToOffset(leftFoot, 152);
  delay(500);

  //rampToOffset(leftFoot, LFoffset, true);


  rampToOffset(rightHip, 106);
  delay(500);

  rampToOffset(rightKnee, 104);
  delay(500);

  rampToOffset(rightFoot, 60);
  delay(500);


  delay(5000);
}


void oneStep() {
  // Pose 1: Right leg forward, lifted — Left leg straight
  pos(2.5, stepHeight, 'r');               // Right foot forward and up
  pos(0, stepHeight - stepClearance, 'l'); // Left planted
  delay(500);  // Increased delay for slower motion

  // Pose 2: Right leg planted — Left leg starts lifting back
  pos(2.5, stepHeight - stepClearance, 'r');
  pos(-2.5, stepHeight, 'l');
  delay(500);  // Increased delay for slower motion

  // Pose 3: Left leg planted forward — Right returns to center
  pos(0, stepHeight - stepClearance, 'r');
  pos(2.5, stepHeight, 'l');
  delay(500);  // Increased delay for slower motion

  // Pose 4: Both return to neutral
  
  pos(0, stepHeight, 'r');
  pos(0, stepHeight, 'l');
  delay(500);  // Increased delay for slower motion
  
}

// === INIT LOWERING OF BOTH LEGS ===
void initialize() {
  for (float i = 10.7; i >= stepHeight; i -= 0.1) {
    pos(0, i, 'l');
    pos(0, i, 'r');
    delay(50);  // Slight pause to allow leg lowering smoothly
  }
}

WiFiServer server(80);

// Pin assignments
const int leftEyePin = 3;
const int rightEyePin = 4;

const int topLEDPin = 7;
const int middleLEDPin = 6;
const int bottomLEDPin = 5;
const int speakerPin = 1;
bool eyesOn = false; // Track eye status

// ROAR sequence: eyes light up, back LEDs animate
void roarAttack() {
  // Temporarily turn on eyes during roar
  digitalWrite(leftEyePin, HIGH);
  digitalWrite(rightEyePin, HIGH);

  digitalWrite(bottomLEDPin, HIGH);
  delay(800);

  digitalWrite(middleLEDPin, HIGH);
  delay(800);

  digitalWrite(topLEDPin, HIGH);
  delay(1000);

  // [PLAY AUDIO HERE]
  digitalWrite(speakerPin,LOW);
  delay(30);
  digitalWrite(speakerPin,Hight);
  delay(3550);
  // Turn off all back LEDs
  digitalWrite(bottomLEDPin, LOW);
  digitalWrite(middleLEDPin, LOW);
  digitalWrite(topLEDPin, LOW);

  // Restore eyes to original state
  digitalWrite(leftEyePin, eyesOn ? HIGH : LOW);
  digitalWrite(rightEyePin, eyesOn ? HIGH : LOW);
}


// === SETUP ===
void setup() {
  Serial.begin(9600);
  while (!Serial);

  pinMode(leftEyePin, OUTPUT);
  pinMode(rightEyePin, OUTPUT);
  pinMode(topLEDPin, OUTPUT);
  pinMode(middleLEDPin, OUTPUT);
  pinMode(bottomLEDPin, OUTPUT);

  digitalWrite(leftEyePin, LOW);
  digitalWrite(rightEyePin, LOW);
  digitalWrite(topLEDPin, LOW);
  digitalWrite(middleLEDPin, LOW);
  digitalWrite(bottomLEDPin, LOW);

  // Attach servos
  leftHip.attach(11); delay(500);
  leftKnee.attach(12); delay(500);
  leftFoot.attach(13); delay(500);
  rightHip.attach(10); delay(500);
  rightKnee.attach(9); delay(500);
  rightFoot.attach(8); delay(500);

  // Set to neutral
  rampToOffset(leftHip, LHoffset); delay(500);
  rampToOffset(leftKnee, LKoffset); delay(500);
  rampToOffset(leftFoot, LFoffset); delay(500);
  rampToOffset(rightHip, RHoffset); delay(500);
  rampToOffset(rightKnee, RKoffset); delay(500);
  rampToOffset(rightFoot, RFoffset); delay(500);

  delay(5000);
  initialize();

  // Wi-Fi connect
  Serial.print("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  server.begin();
}

void loop() {
  WiFiClient client = server.available();
  if (client) {
    Serial.println("New client connected");
    String request = client.readStringUntil('\r');
    Serial.print("Request: ");
    Serial.println(request);
    client.flush();

    // Command handling
    if (request.indexOf("/roar") != -1) {
      roarAttack();
    } else if (request.indexOf("/eyes/on") != -1) {
      eyesOn = true;
      digitalWrite(leftEyePin, HIGH);
      digitalWrite(rightEyePin, HIGH);
    } else if (request.indexOf("/eyes/off") != -1) {
      eyesOn = false;
      digitalWrite(leftEyePin, LOW);
      digitalWrite(rightEyePin, LOW);
    } else if (request.indexOf("/squat") != -1) {
      squat();
    } else if (request.indexOf("/step") != -1) {
      oneStep();
    }

    // Send HTML response
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html");
    client.println("Connection: close");
    client.println();
    client.println("<!DOCTYPE html><html><head><title>Robozilla</title>");
    client.println("<style>body{font-family:Arial;text-align:center;}button{padding:20px;font-size:20px;margin:10px;}</style>");
    client.println("</head><body><h1>Robozilla Control Panel</h1>");
    client.println("<p><a href=\"/roar\"><button>ROAR</button></a></p>");
    client.println("<p><a href=\"/squat\"><button>SQUAT</button></a></p>");
    client.println("<p><a href=\"/step\"><button>STEP FORWARD</button></a></p>");
    client.println("<p><a href=\"/eyes/on\"><button>Eyes ON</button></a>");
    client.println("<a href=\"/eyes/off\"><button>Eyes OFF</button></a></p>");
    client.println("</body></html>");

    delay(1);
    client.stop();
    Serial.println("Client disconnected");
  }

  // Serial command input
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command == "roar") {
      Serial.println("Executing ROAR command...");
      roarAttack();
    } else if (command == "squat") {
      Serial.println("Executing SQUAT command...");
      squat();
    } else if (command == "step") {
      Serial.println("Executing STEP FORWARD command...");
      oneStep();
    } else if (command == "eyes on") {
      eyesOn = true;
      digitalWrite(leftEyePin, HIGH);
      digitalWrite(rightEyePin, HIGH);
      Serial.println("Eyes turned ON");
    } else if (command == "eyes off") {
      eyesOn = false;
      digitalWrite(leftEyePin, LOW);
      digitalWrite(rightEyePin, LOW);
      Serial.println("Eyes turned OFF");
    } else {
      Serial.println("Unknown command");
    }
  }

  delay(100);
}
