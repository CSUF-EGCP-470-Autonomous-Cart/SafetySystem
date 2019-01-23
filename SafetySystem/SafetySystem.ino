//Must be before ros.h is imported
#define ROSSERIAL_ARDUINO_TCP

#include "RF24Network.h"
#include "RF24.h"
#include "RF24Mesh.h"
#include <SPI.h>
//Include eeprom.h for AVR (Uno, Nano) etc. except ATTiny
#include <EEPROM.h>
#include <ros.h>
#include <diagnostic_msgs/DiagnosticStatus.h>

#define RATE 1//Hz
#define ESTOP_INPUT_PIN 2
#define REMOTE_ESTOP_RELAY_PIN 3
#define CONTORL_TOWER_RED 4
#define CONTROL_TOWER_YELLOW 5
#define CONTORL_TOWER_GREEN 6
#define CONTORL_TOWER_SIREN 7


byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xEC
};
IPAddress ip(192, 168, 3, 20);
IPAddress server(192 , 168, 3, 11);
const uint16_t serverPort = 11411;

ros::NodeHandle nh;
diagnostic_msgs::DiagnosticStatus pub_diag_status_msg;
ros::Publisher pub_status("safety", &pub_diag_status_msg);

RF24 radio(8, 9);
RF24Network network(radio);
RF24Mesh mesh(radio, network);

void setup() {
  Serial.begin(115200);

  Ethernet.begin(mac, ip);

  //wait for ethernet shield to initalize
  delay(1000);

  Serial.println("");
  Serial.println("Ethernet connected");
  Serial.println("IP address: ");
  Serial.println(Ethernet.localIP());
  
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();
  
  Serial.print("IP = ");
  Serial.println(nh.getHardware()->getLocalIP());

  pinMode(ESTOP_INPUT_PIN, INPUT);
  pinMode(REMOTE_ESTOP_RELAY_PIN, OUTPUT);
  pinMode(CONTORL_TOWER_RED, OUTPUT);
  pinMode(CONTROL_TOWER_YELLOW, OUTPUT);
  pinMode(CONTORL_TOWER_GREEN, OUTPUT);
  pinMode(CONTORL_TOWER_SIREN, OUTPUT);

  //Init relay pins HIGH = Realy 0
  digitalWrite(REMOTE_ESTOP_RELAY_PIN, HIGH);
  digitalWrite(CONTORL_TOWER_RED, HIGH);
  digitalWrite(CONTROL_TOWER_YELLOW, HIGH);
  digitalWrite(CONTORL_TOWER_GREEN, HIGH);
  digitalWrite(CONTORL_TOWER_SIREN, HIGH);

  // Set the nodeID to 0 for the master node
  mesh.setNodeID(0);                  // Dump the configuration of the rf unit for debugging
  // Connect to the mesh
  mesh.begin();
  Serial.println("Mesh started");

  nh.advertise(pub_status);
}

bool remoteStatus = true;
unsigned long prevPollTime =  millis();
void loop() {
  // Call mesh.update to keep the network updated
  mesh.update();

  // In addition, keep the 'DHCP service' running on the master node so addresses will
  // be assigned to the sensor nodes
  mesh.DHCP();

  bool wireStatus = (digitalRead(ESTOP_INPUT_PIN) == HIGH);

  // Check for incoming data from the sensors
  if (network.available()) {
    RF24NetworkHeader header;
    network.peek(header);

    uint8_t ESTOP_State = 0;
    switch (header.type) {
      // Display the incoming millis() values from the sensor nodes
      case 'S':
        network.read(header, &ESTOP_State, sizeof(ESTOP_State));
        digitalWrite(REMOTE_ESTOP_RELAY_PIN, abs(ESTOP_State - 1));
        remoteStatus = (ESTOP_State == 1);
        //        Serial.print(millis());
        //        Serial.print("$ State: ");
        //        Serial.println(ESTOP_State);
        break;
      default:
        network.read(header, 0, 0);
        //        Serial.print(millis());
        //        Serial.print("$ Unknown Header: ");
        //        Serial.println(header.type);
        break;
    }
  }

  pub_diag_status_msg.level = diagnostic_msgs::DiagnosticStatus::ERROR;
  pub_diag_status_msg.name = "ESTOP System";

  if (remoteStatus && wireStatus) {
    pub_diag_status_msg.level = diagnostic_msgs::DiagnosticStatus::OK;
    pub_diag_status_msg.message = "";
  }
  else if (!remoteStatus && !wireStatus) {
    pub_diag_status_msg.message = "Remote and Wired ESTOP triggered.";
  }
  else if (!remoteStatus) {
    pub_diag_status_msg.message = "Remote ESTOP triggered.";
  }
  else if (!wireStatus) {
    pub_diag_status_msg.message = "Wired ESTOP triggered.";
  }

  controlStack(pub_diag_status_msg);

  if ( (millis() - prevPollTime) > (1000 / RATE)) {
    prevPollTime = millis();

    if (nh.connected()) {
      Serial.print("Conneced...");
      pub_status.publish(&pub_diag_status_msg);

      Serial.println("published!");
    }
    else {
      Serial.println("Not conneced");
    }
  }

  nh.spinOnce();
  delay(1);
}

void controlStack(diagnostic_msgs::DiagnosticStatus status) {
  if (status.level == diagnostic_msgs::DiagnosticStatus::OK) {
    digitalWrite(CONTORL_TOWER_RED, HIGH);
    digitalWrite(CONTROL_TOWER_YELLOW, HIGH);
    digitalWrite(CONTORL_TOWER_GREEN, LOW);
  }
  else if (status.level == diagnostic_msgs::DiagnosticStatus::WARN) {
    digitalWrite(CONTORL_TOWER_RED, HIGH);
    digitalWrite(CONTROL_TOWER_YELLOW, LOW);
    digitalWrite(CONTORL_TOWER_GREEN, HIGH);
  }
  else {
    digitalWrite(CONTORL_TOWER_RED, LOW);
    digitalWrite(CONTROL_TOWER_YELLOW, HIGH);
    digitalWrite(CONTORL_TOWER_GREEN, HIGH);
  }
}
