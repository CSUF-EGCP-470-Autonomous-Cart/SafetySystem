#include "RF24Network.h"
#include "RF24.h"
#include "RF24Mesh.h"
#include <SPI.h>
#include "printf.h"

#define nodeID 1
#define ESTOP_PIN 2
#define ESTOP_LED_PIN 3

RF24 radio(8, 9);
RF24Network network(radio);
RF24Mesh mesh(radio, network);

void setup() {
  Serial.begin(115200);
  printf_begin();

  pinMode(ESTOP_PIN, INPUT);
  pinMode(ESTOP_LED_PIN, OUTPUT);
  digitalWrite(ESTOP_LED_PIN, LOW);

  //radio.printDetails();
  mesh.setNodeID(nodeID);
  // Connect to the mesh
  Serial.print(F("Connecting to the mesh..."));

  uint64_t startTime = millis();
  while (!mesh.begin()) {
    if (millis() - startTime > 250) {
      digitalWrite(ESTOP_LED_PIN, !digitalRead(ESTOP_LED_PIN));
      startTime = millis();
    }
  }

  Serial.println("Connected!");
  for (int i = 0; i < 3; i++) {
    digitalWrite(ESTOP_LED_PIN, HIGH);
    delay(100);
    digitalWrite(ESTOP_LED_PIN, LOW);
    delay(100);
  }

}

void loop() {
  mesh.update();

  uint8_t ESTOP_State = digitalRead(ESTOP_PIN);
  digitalWrite(ESTOP_LED_PIN, !ESTOP_State);

  printf("Transmitting State: %d...", ESTOP_State);
  if (!mesh.write(&ESTOP_State, 'S', sizeof(ESTOP_State))) {
    if ( ! mesh.checkConnection() ) {
      //refresh the network address
      Serial.println("Renewing Address");
      mesh.renewAddress();
    } else {
      Serial.println("Send fail, Test OK");
    }
  } else {
    Serial.println("Send OK");
  }

  delay(100);

}
