// Hard synchronization sketch for Arduino UNO
// Pinout:
// Pin 2: Camera sync signal @ 10Hz, 50% duty cycle
// Pin 4: Livox Mid-40 sync signal @ 1Hz, 20% duty cycle
// Pin 7: Realsense rgbd sync signal @ 10Hz, 50% duty cycle

#define CAMERASYNC 2
#define LIVOXSYNC 4
#define RGBDSYNC 7

void setup() {
  // put your setup code here, to run once:
  pinMode(CAMERASYNC, OUTPUT);
  pinMode(LIVOXSYNC, OUTPUT);
  pinMode(RGBDSYNC, OUTPUT);
  digitalWrite(CAMERASYNC, LOW);
  digitalWrite(LIVOXSYNC, LOW);
  digitalWrite(RGBDSYNC, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(LIVOXSYNC, HIGH);
  
  for(int i=0; i<2; i++){
    digitalWrite(CAMERASYNC, HIGH);
    digitalWrite(RGBDSYNC, HIGH);
    delay(50); // Delay 50 ms for camera high signal
    digitalWrite(CAMERASYNC, LOW);
    digitalWrite(RGBDSYNC, LOW);
    delay(50); // Delay 50 ms for camera low signal
  }

  digitalWrite(LIVOXSYNC, LOW);

  for(int i=0; i<8; i++){
    digitalWrite(CAMERASYNC, HIGH);
    digitalWrite(RGBDSYNC, HIGH);
    delay(50); // Delay 50 ms for camera high signal
    digitalWrite(CAMERASYNC, LOW);
    digitalWrite(RGBDSYNC, LOW);
    delay(50); // Delay 50 ms for camera low signal
   }

  
}
