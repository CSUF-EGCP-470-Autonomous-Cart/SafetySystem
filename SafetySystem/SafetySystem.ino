#include "RF24Network.h"
#include "RF24.h"
#include "RF24Mesh.h"
#include <SPI.h>
//Include eeprom.h for AVR (Uno, Nano) etc. except ATTiny
#include <EEPROM.h>
#include "printf.h"

#define ESTOP_INPUT_PIN 2
#define REMOTE_ESTOP_RELAY_PIN 3
#define CONTORL_TOWER_RED 4
#define CONTROL_TOWER_YELLOW 5
#define CONTORL_TOWER_GREEN 6


RF24 radio(8, 9);
RF24Network network(radio);
RF24Mesh mesh(radio, network);

void setup() {
  Serial.begin(115200);
  printf_begin();

  pinMode(ESTOP_INPUT_PIN, INPUT);
  pinMode(REMOTE_ESTOP_RELAY_PIN, OUTPUT);

  // Set the nodeID to 0 for the master node
  mesh.setNodeID(0);                  // Dump the configuration of the rf unit for debugging
  // Connect to the mesh
  mesh.begin();
  Serial.println("Mesh started");

}

void loop() {
  // Call mesh.update to keep the network updated
  mesh.update();

  // In addition, keep the 'DHCP service' running on the master node so addresses will
  // be assigned to the sensor nodes
  mesh.DHCP();


  // Check for incoming data from the sensors
  if (network.available()) {
    RF24NetworkHeader header;
    network.peek(header);

    uint8_t ESTOP_State = 0;
    switch (header.type) {
      // Display the incoming millis() values from the sensor nodes
      case 'S':
        network.read(header, &ESTOP_State, sizeof(ESTOP_State));
        digitalWrite(REMOTE_ESTOP_RELAY_PIN, abs(ESTOP_State-1));
        Serial.print(millis());
        Serial.print("$ State: ");
        Serial.println(ESTOP_State);
        break;
      default:
        network.read(header, 0, 0);
        Serial.print(millis());
        Serial.print("$ Unknown Header: ");
        Serial.println(header.type);
        break;
    }
  }


  //  if(digitalRead(ESTOP_INPUT_PIN) == HIGH) {
  //    Serial.println("STOPPED");
  //  }
  //  else {
  //    Serial.println("OK");
  //  }

  delay(10);

}
