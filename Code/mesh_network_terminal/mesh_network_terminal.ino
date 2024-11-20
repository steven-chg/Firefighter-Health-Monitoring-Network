#include <SPI.h>
#include <LoRa.h>

// Define LoRa Pins
#define LORA_CS 47   // NSS (CS) pin
#define LORA_RST 21  // RST pin
#define LORA_IRQ 14  // DIO0 pin

// LoRa Network Settings
#define TERMINAL_NODE_ID 1 // The ID of the terminal node where messages stop
#define NODE_ID 1          // Unique ID for this node

// Maximum number of messages to track
#define MAX_TRACKED_MESSAGES 50

// Array to store recent message IDs
unsigned long trackedMsgIDs[MAX_TRACKED_MESSAGES];
int trackedIndex = 0;

// Structure to hold message details
struct LoRaMessage {
  int senderID;
  unsigned long msgID;
  String payload;
};

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  while (!Serial);

  // Initialize LoRa
  SPI.begin(36, 35, 48, 47);  // Set SPI pins for LoRa
  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ); // Set LoRa pins
  if (!LoRa.begin(915E6)) {    // Set frequency for North America
    Serial.println("LoRa initialization failed!");
    while (1);
  }
  
  Serial.print("Node ");
  Serial.print(NODE_ID);
  Serial.println(" initialized");
}

// Add a message ID to the tracking array
void trackMessage(unsigned long msgID) {
  trackedMsgIDs[trackedIndex] = msgID;
  trackedIndex = (trackedIndex + 1) % MAX_TRACKED_MESSAGES;
}

// Check if a message ID has already been processed
bool isDuplicate(unsigned long msgID) {
  for (int i = 0; i < MAX_TRACKED_MESSAGES; i++) {
    if (trackedMsgIDs[i] == msgID) {
      return true;
    }
  }
  return false;
}

void loop() {
  // Check for incoming LoRa packets
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // Read the sender ID
    int senderID = LoRa.read();
    int numHop = LoRa.read();
    // Read the message ID (4 bytes)
    unsigned long msgID = 0;
    msgID |= (unsigned long)LoRa.read() << 24;
    msgID |= (unsigned long)LoRa.read() << 16;
    msgID |= (unsigned long)LoRa.read() << 8;
    msgID |= (unsigned long)LoRa.read();

    // Read the payload
    String payload = LoRa.readString();
    
    // Check for duplicates
    if (isDuplicate(msgID)) {
      Serial.println("Duplicate message. Ignoring.");
      return;
    }

    // Track the new message ID
    trackMessage(msgID);

    Serial.println("Message has reached the terminal node!!!!!!");
    Serial.print("Final Message: ");
    Serial.print("Received Message ID: ");
    Serial.print(msgID);
    Serial.print(" from Node ");
    Serial.print(senderID);
    Serial.print(" with # of hop =  ");
    Serial.print(numHop);
    Serial.print(" Payload: ");
    Serial.println(payload);
    Serial.print("' with RSSI ");
    Serial.println(LoRa.packetRssi());
  }

  delay(100); // Small delay to avoid spamming
}

