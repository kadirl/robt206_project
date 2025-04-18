// ---------------------------------------------------------------------------
// Example NewPing library sketch that does a ping about 20 times per second.
// ---------------------------------------------------------------------------

#include <NewPing.h>


#define ECHO_PIN     32  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define TRIGGER_PIN  33  // Arduino pin tied to trigger pin on the ultrasonic sensor.


#define ECHO_PIN_1     34  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define TRIGGER_PIN_1  35  // Arduino pin tied to trigger pin on the ultrasonic sensor.

#define ECHO_PIN_2     36  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define TRIGGER_PIN_2  37  // Arduino pin tied to trigger pin on the ultrasonic sensor.

#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
NewPing sonar1(TRIGGER_PIN_1, ECHO_PIN_1, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
NewPing sonar2(TRIGGER_PIN_2, ECHO_PIN_2, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

void setup() {
    Serial.begin(115200); // Open serial monitor at 115200 baud to see ping results.
}

void loop() {
    delay(60);                     // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
    Serial.print("1Ping: ");
    Serial.print(sonar.ping_cm()); // Send ping, get distance in cm and print result (0 = outside set distance range)
    Serial.print("\n2Ping: ");
    Serial.print(sonar1.ping_cm()); // Send ping, get distance in cm and print result (0 = outside set distance range)
    Serial.print("\n3Ping: ");
    Serial.print(sonar2.ping_cm()); // Send ping, get distance in cm and print result (0 = outside set distance range)
    Serial.println("cm\n");
}