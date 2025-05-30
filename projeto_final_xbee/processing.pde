import processing.serial.*;

Serial myPort; // Create object from Serial class

/* Communications and XBee related variables */
int portIndex = 1; // WARNING: Set this to the port connected to XBEE Explorer
byte frameStartByte = 0x7E;
byte frameTypeTXrequest = 0x10; // For sending commands
// Ensure these match the SH and SL of the ARDUINO's XBee
int destAddressHigh = 0x13A200;
int destAddressLow = 0x40A099E8; // This should be the SL of the Arduino's XBee

byte status_payload_from_arduino = 0; // Raw status byte received
int rssi_from_arduino = 100; // RSSI value sent by the Arduino (its last received packet's RSSI)

/* Parking and Gate Status Variables */
boolean slot1_occupied = false;
boolean slot2_occupied = false;
boolean slot3_occupied = false;
boolean gate_is_open = false;
boolean gate_is_closed = true; // Initial assumption
boolean gate_is_opening = false;
boolean gate_is_closing = false;

// Bit definitions for the status payload byte (mirroring Arduino)
final byte SLOT1_BIT = 0;
final byte SLOT2_BIT = 1;
final byte SLOT3_BIT = 2;
final byte GATE_OPEN_BIT = 3;
final byte GATE_CLOSED_BIT = 4;
final byte GATE_OPENING_BIT = 5;
final byte GATE_CLOSING_BIT = 6;

/* Drawing related variables */
PFont font;
int fontSize = 14; // Slightly larger font for status
int rectX, rectY;      // Position of square button
int circleX, circleY;  // Position of circle button
int rectSize = 40;     // Diameter of rect button
int circleSize = 40;   // Diameter of circle button

// UI Positions
int statusDisplayX = 50;
int statusDisplayY = 50;
int parkingSlotSize = 50;
int parkingSlotSpacing = 20;


void setup() {
  size(800, 600);
  smooth();
  font = createFont("Arial.bold", fontSize); // Using bold for clarity
  textFont(font);

  // Compute buttons location (moved to right for status on left)
  circleX = width - 150; // "Close Gate" button
  circleY = height/2 - 50;
  rectX = width - 150;   // "Open Gate" button
  rectY = height/2 + 10;
  ellipseMode(CENTER);

  println(Serial.list());
  if (Serial.list().length > portIndex) {
    println("Connecting to -> " + Serial.list()[portIndex]);
    myPort = new Serial(this, Serial.list()[portIndex], 9600);
    myPort.clear();
  } else {
    println("Error: Serial port at index " + portIndex + " not available.");
    println("Available ports:");
    for (int i = 0; i < Serial.list().length; i++) {
      println("[" + i + "] " + Serial.list()[i]);
    }
    exit(); // Stop the sketch if port is not available
  }
}

void draw() {
  background(224); // Light gray background

  if (myPort != null && myPort.available() >= 18) { // Check for full XBee RX frame (1 status byte + 1 RSSI byte payload)
    decodeRXAPIpacket(); // Decode and update status variables
  }

  drawParkingStatus();
  drawGateStatus();
  drawRSSIdisplay(); // Displays RSSI sent by Arduino
  drawbuttons();     // Draw LED ON/OFF (or gate command) buttons
}

void decodeRXAPIpacket() {
  // Function for decoding the received API frame from XBEE
  // Assumes a Zigbee RX Packet (0x90) with 2 bytes of RF data:
  // 1st byte: status_payload
  // 2nd byte: rssi_from_arduino

  if (myPort.read() != frameStartByte) {
    // If the first byte isn't the start byte, something is wrong.
    // Clear buffer until next potential start or until empty to try and resync.
    while(myPort.available() > 0 && myPort.peek() != frameStartByte) {
        myPort.read();
    }
    return; // Wait for a proper frame start
  }

  // Skip Length (2 bytes), Frame Type (1 byte),
  // 64-bit Source Address (8 bytes), 16-bit Source Address (2 bytes),
  // Receive Options (1 byte). Total 14 bytes to skip.
  for (int i = 0; i < 14; i++) {
    if (myPort.available() > 0) {
      myPort.read();
    } else {
      return; // Incomplete frame
    }
  }

  // Read the 2 bytes of RF data
  if (myPort.available() >= 3) { // Need 2 for data, 1 for checksum
    status_payload_from_arduino = (byte) myPort.read();
    rssi_from_arduino = myPort.read(); // This is the RSSI value *sent by the Arduino*

    // Unpack the status byte
    slot1_occupied = (status_payload_from_arduino >> SLOT1_BIT & 1) == 1;
    slot2_occupied = (status_payload_from_arduino >> SLOT2_BIT & 1) == 1;
    slot3_occupied = (status_payload_from_arduino >> SLOT3_BIT & 1) == 1;

    gate_is_open = (status_payload_from_arduino >> GATE_OPEN_BIT & 1) == 1;
    gate_is_closed = (status_payload_from_arduino >> GATE_CLOSED_BIT & 1) == 1;
    gate_is_opening = (status_payload_from_arduino >> GATE_OPENING_BIT & 1) == 1;
    gate_is_closing = (status_payload_from_arduino >> GATE_CLOSING_BIT & 1) == 1;

    // For debugging:
    // print("Status Byte: " + binary(status_payload_from_arduino, 8));
    // print(" S1:" + slot1_occupied + " S2:" + slot2_occupied + " S3:" + slot3_occupied);
    // println(" GOpen:" + gate_is_open + " GClosed:" + gate_is_closed + " GOpening:" + gate_is_opening + " GClosing:" + gate_is_closing);


    myPort.read(); // Read and discard the XBee frame checksum
  }
}

void drawParkingStatus() {
  fill(0);
  textAlign(LEFT, TOP);
  text("Parking Status:", statusDisplayX, statusDisplayY);

  int currentY = statusDisplayY + fontSize + 10;

  // Slot 1
  fill(slot1_occupied ? color(255, 0, 0) : color(0, 255, 0)); // Red if occupied, Green if free
  rect(statusDisplayX, currentY, parkingSlotSize, parkingSlotSize);
  fill(0);
  textAlign(CENTER, CENTER);
  text("P1", statusDisplayX + parkingSlotSize / 2, currentY + parkingSlotSize / 2);

  // Slot 2
  fill(slot2_occupied ? color(255, 0, 0) : color(0, 255, 0));
  rect(statusDisplayX + parkingSlotSize + parkingSlotSpacing, currentY, parkingSlotSize, parkingSlotSize);
  fill(0);
  text("P2", statusDisplayX + parkingSlotSize + parkingSlotSpacing + parkingSlotSize / 2, currentY + parkingSlotSize / 2);

  // Slot 3
  fill(slot3_occupied ? color(255, 0, 0) : color(0, 255, 0));
  rect(statusDisplayX + 2 * (parkingSlotSize + parkingSlotSpacing), currentY, parkingSlotSize, parkingSlotSize);
  fill(0);
  text("P3", statusDisplayX + 2 * (parkingSlotSize + parkingSlotSpacing) + parkingSlotSize / 2, currentY + parkingSlotSize / 2);
}

void drawGateStatus() {
  int currentY = statusDisplayY + fontSize + 10 + parkingSlotSize + 30; // Below parking status
  fill(0);
  textAlign(LEFT, TOP);
  text("Gate Status:", statusDisplayX, currentY);

  String gateStatusText = "Unknown";
  if (gate_is_opening) {
    gateStatusText = "OPENING";
  } else if (gate_is_closing) {
    gateStatusText = "CLOSING";
  } else if (gate_is_open) {
    gateStatusText = "OPEN";
  } else if (gate_is_closed) {
    gateStatusText = "CLOSED";
  }
  // You might want a more prominent color for the status text
  fill(0, 0, 200); // Blue for status text
  text(gateStatusText, statusDisplayX, currentY + fontSize + 10);
}


void drawRSSIdisplay() {
  // Shows the RSSI value that the ARDUINO reported for ITS last received packet
  int rssiX = statusDisplayX;
  int rssiY = statusDisplayY + fontSize + 10 + parkingSlotSize + 30 + 2 * (fontSize + 10) + 20; // Below gate status

  fill(0);
  textAlign(LEFT, TOP);
  text("Remote XBee's Last RX RSSI:", rssiX, rssiY);
  rect(rssiX, rssiY + fontSize + 5, 120, 30, 7);
  fill(255);
  textAlign(LEFT, CENTER); // Align text inside the box
  text("-" + rssi_from_arduino + " dBm", rssiX + 10, rssiY + fontSize + 5 + 15);
}

void keyPressed() {
  if (key == 'C') { // Send command to Close Gate
    formatTXAPIpacket((byte) 'C');
    println("Sent 'C' (Close Gate) command");
  } else if (key == 'O') { // Send command to Open Gate
    formatTXAPIpacket((byte) 'O');
    println("Sent 'O' (Open Gate) command");
  }
}

void mousePressed() {
  if (overCircle(circleX, circleY, circleSize)) { // Close Gate button
    formatTXAPIpacket((byte) 'C');
    println("Sent 'C' (Close Gate) command via button");
  } else if (overRect(rectX, rectY, rectSize, rectSize)) { // Open Gate button
    formatTXAPIpacket((byte) 'O');
    println("Sent 'O' (Open Gate) command via button");
  }
}

boolean overRect(int x, int y, int w, int h) {
  return (mouseX >= x && mouseX <= x + w && mouseY >= y && mouseY <= y + h);
}

boolean overCircle(int x, int y, int diameter) {
  float disX = x - mouseX;
  float disY = y - mouseY;
  return (sqrt(sq(disX) + sq(disY)) < diameter / 2);
}

void drawbuttons() {
  // Draw buttons on screen
  fill(255);
  stroke(0);
  ellipse(circleX, circleY, circleSize, circleSize); // "Close Gate" button
  rect(rectX, rectY, rectSize, rectSize);           // "Open Gate" button

  fill(0);
  textAlign(LEFT, CENTER);
  text("Close Gate ('L')", circleX + circleSize / 2 + 10, circleY);
  text("Open Gate ('D')", rectX + rectSize / 2 + 10, rectY + rectSize / 2);
}

void formatTXAPIpacket(byte value) {
  // Transmit a command (single byte) using XBEE API frame
  if (myPort == null) return;

  int sum = 0;

  myPort.write(frameStartByte);
  myPort.write((byte) 0x00); // Length MSB
  myPort.write((byte) 0x0F); // Length LSB (15 bytes for 1 byte RF data payload)
                           // FT(1)+FID(1)+Addr64(8)+Addr16(2)+BR(1)+Opt(1)+Data(1)=15

  myPort.write(frameTypeTXrequest); // 0x10
  sum += frameTypeTXrequest;

  myPort.write((byte) 0x00); // Frame ID (0x00 = no ACK from XBee module itself)
  sum += 0x00;

  // 64-bit Destination Address (SH + SL of the Arduino's XBee)
  myPort.write((byte) ((destAddressHigh >> 24) & 0xFF)); sum += (byte) ((destAddressHigh >> 24) & 0xFF);
  myPort.write((byte) ((destAddressHigh >> 16) & 0xFF)); sum += (byte) ((destAddressHigh >> 16) & 0xFF);
  myPort.write((byte) ((destAddressHigh >> 8) & 0xFF));  sum += (byte) ((destAddressHigh >> 8) & 0xFF);
  myPort.write((byte) (destAddressHigh & 0xFF));         sum += (byte) (destAddressHigh & 0xFF);
  myPort.write((byte) ((destAddressLow >> 24) & 0xFF));  sum += (byte) ((destAddressLow >> 24) & 0xFF);
  myPort.write((byte) ((destAddressLow >> 16) & 0xFF));  sum += (byte) ((destAddressLow >> 16) & 0xFF);
  myPort.write((byte) ((destAddressLow >> 8) & 0xFF));   sum += (byte) ((destAddressLow >> 8) & 0xFF);
  myPort.write((byte) (destAddressLow & 0xFF));          sum += (byte) (destAddressLow & 0xFF);

  myPort.write((byte) 0xFF); // 16-bit Destination Network Address (0xFFFE for unknown)
  sum += 0xFF;
  myPort.write((byte) 0xFE);
  sum += 0xFE;

  myPort.write((byte) 0x00); // Broadcast Radius (0x00 = max hops)
  sum += 0x00;

  myPort.write((byte) 0x00); // Options (0x00 = default)
  sum += 0x00;

  myPort.write(value); // RF Data (the command byte 'L' or 'D')
  sum += value;

  myPort.write((byte) (0xFF - (sum & 0xFF))); // Checksum
}
