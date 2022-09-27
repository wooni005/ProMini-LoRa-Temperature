/*
	LoRa Simple Gateway/Node Example

	This code uses InvertIQ function to create a simple Gateway/Node logic.

	Gateway - Sends messages with enableInvertIQ()
					- Receives messages with disableInvertIQ()

	Node		- Sends messages with disableInvertIQ()
					- Receives messages with enableInvertIQ()

	With this arrangement a Gateway never receive messages from another Gateway
	and a Node never receive message from another Node.
	Only Gateway to Node and vice versa.

	This code receives messages and sends a message every second.

	InvertIQ function basically invert the LoRa I and Q signals.

	See the Semtech datasheet, http://www.semtech.com/images/datasheet/sx1276.pdf
	for more on InvertIQ register 0x33.

	created 05 August 2018
	by Luiz H. Cassettari
*/

#include <SPI.h>							 // include libraries
#include <LoRa.h>
#include "LowPower.h"
//#include <OneWire.h>
#include <DallasTemperature.h>

// #define DEBUG
//#define ACCEPT_REMOTE_COMMANDS

#define SERIAL_BAUD	 				57600
#define NODE_ID			 			12			// NodeId of this LoRa Node
#define MAX_PACKET_SIZE				10

#define MSG_ID_NODE_STARTUP		1 	 	// Node startup notification
#define MSG_ID_STILL_ALIVE		2  		// Node still alive
#define MSG_ID_CMND_REQUEST		3  		// Node wakeup/cmnd request
#define MSG_ID_TEMPERATURE		4  		// Send measure temperature

//#define SEND_MSG_EVERY	22		// Watchdog is a timerTick on a avg 8,0 sec timebase
										// SEND_MSG_EVERY=8	-> +- 1min
										// SEND_MSG_EVERY=14 -> +- 2min
										// SEND_MSG_EVERY=23 -> +- 3min
										// SEND_MSG_EVERY=30 -> +- 4min
										// SEND_MSG_EVERY=38 -> +- 5min
										// SEND_MSG_EVERY=228 -> 0,5 hours
										// SEND_MSG_EVERY=1824 -> 4 hours

// #define SEND_TEMPERATURE			12    // 16 sec
#define SEND_TEMPERATURE			75    // 10 min
#define SEND_MEASURE_VCC_EVERY		24	// Measure Vcc voltage every N messages
										// MEASURE_EVERY=24 -> +- 4 hour
#define ONE_WIRE_BUS 3

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature oneWireTemp(&oneWire);
int deviceCount = 0;
byte payloadSize = 3; //Without any device
float fTemp;

volatile word sendMsgTimer = SEND_TEMPERATURE - 2;
volatile unsigned char sendMsgVccLevelTimer = SEND_MEASURE_VCC_EVERY;

//Message max 30 bytes
struct Payload {
	byte nodeId;
	byte msgId;
	byte voltageVcc;				 //getVcc 1.0V=0, 1.8V=40, 3,0V=100, 3.3V=115, 5.0V=200, 6.0V=250
	int oneWireTemp1;
	int oneWireTemp2;
	int oneWireTemp3;
} txPayload;

const long loRaFrequency = 866E6;			// LoRa loRaFrequency

const int loRaCsPin = 15;						// LoRa radio chip select
const int loRaResetPin = 14;			 		// LoRa radio reset
const int loRaIrqPin = 2;						// change for your board; must be a hardware interrupt pin

void LoRa_rxMode(){
	LoRa.enableInvertIQ();								// active invert I and Q signals
	LoRa.receive();										// set receive mode
}

void LoRa_txMode(){
	LoRa.idle();											// set standby mode
	LoRa.disableInvertIQ();								// normal mode
}

void LoRa_sendMessage(Payload payload, byte payloadLen) {
	LoRa_txMode();											// set tx mode
	LoRa.beginPacket();								 	// start packet
	LoRa.write((byte*) &payload, payloadLen); 	// add payload
	LoRa.endPacket(true);								// finish packet and send it
}

void onReceive(int packetSize) {
	byte rxPayload [MAX_PACKET_SIZE];

	byte i = 0, rxByte;

	while (LoRa.available()) {
		rxByte = (byte)LoRa.read();
		if (i < MAX_PACKET_SIZE) {
			rxPayload[i] = rxByte;
			i++;
		}
	}

	// Only accept messages with our NodeId
	if (rxPayload[0] == NODE_ID) {
#ifdef DEBUG
		Serial.print("Rx packet OK "); // Start received message
		for (char i = 0; i < packetSize; i++) {
				Serial.print(rxPayload[i], DEC);
				Serial.print(' ');
		}
#endif
	}
}

void onTxDone() {
	// Serial.println("TxDone");
	LoRa_rxMode();
}

static byte vccLevelRead()
{
  // REFS1 REFS0          --> 0 1, AVcc internal ref. -Selects AVcc external reference
  // MUX3 MUX2 MUX1 MUX0  --> 1110 1.1V (VBG)         -Selects channel 14, bandgap voltage, to measure
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Let mux settle a little to get a more stable A/D conversion
  
  // Start a conversion
  ADCSRA |= _BV(ADSC);
  
  // Wait for it to complete
  while (bit_is_set(ADCSRA, ADSC));
  
  // convert ADC readings to fit in one byte, i.e. 20 mV steps:
  // 1.0V = 0, 1.8V = 40, 3.0V = 100, 3.3V = 115, 5.0V = 200, 6.0V = 250
  return (55U * 1023U) / (ADC + 1) - 50;
}

void setup() {
#ifdef DEBUG
	Serial.begin(SERIAL_BAUD);									 // initialize serial
	while (!Serial);

	Serial.println();
	Serial.print("[LORA-NODE.");
	Serial.print(NODE_ID);
	Serial.println("]");
#endif

	LoRa.setPins(loRaCsPin, loRaResetPin, loRaIrqPin);

	if (!LoRa.begin(loRaFrequency)) {
#ifdef DEBUG
		Serial.println("LoRa init failed. Check your connections.");
#endif
		while (true);											 // if failed, do nothing
	}

	//LoRa.setTxPower(20);
	LoRa.enableCrc();
	LoRa.onReceive(onReceive);
	LoRa.onTxDone(onTxDone);
	LoRa_rxMode();

	// Send Node startup msg
	txPayload.nodeId = NODE_ID;
	txPayload.msgId = MSG_ID_NODE_STARTUP;
	LoRa_sendMessage(txPayload, 2); // send a message
	delay(40); // [ms] Give RFM95W time to send the message

	// Start up the library
	oneWireTemp.begin();

#ifdef DEBUG
	// locate devices on the bus
	Serial.print("Locating devices...");
	Serial.print("Found ");
#endif //DEBUG

	deviceCount = oneWireTemp.getDeviceCount();

#ifdef DEBUG
	Serial.print(deviceCount, DEC);
	Serial.println(" devices.");
	Serial.println("");
#endif //DEBUG

	if (deviceCount > 3) {
#ifdef DEBUG
		Serial.println("Too many temperature devices found, limited to 3 devices");
#endif //DEBUG
		deviceCount = 3;
	}
#ifdef DEBUG
	delay(200); // [ms] Give time to print the debug messages before sleep
#endif //DEBUG
	payloadSize = 3 + (deviceCount * 2);
}

void loop() {
	// Enter power down state with ADC and BOD module disabled. Wake up when wake up pin is low
	//Serial.println("Sleep for 8s....");
	//delay(100);
	LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);

	// Waked up by periodic wakeup timer (8s)
	sendMsgTimer++;

	if (sendMsgTimer > SEND_TEMPERATURE) {
		sendMsgTimer = 0;

#ifdef DEBUG
		Serial.println("Send Temperature message");
#endif
		txPayload.nodeId = NODE_ID;
		txPayload.msgId = MSG_ID_TEMPERATURE;

		sendMsgVccLevelTimer++;
		if (sendMsgVccLevelTimer >= SEND_MEASURE_VCC_EVERY) {
			sendMsgVccLevelTimer = 0;
			txPayload.voltageVcc = vccLevelRead();
		}

		//Read out oneWire temperatuur sensor
		oneWireTemp.requestTemperatures(); // Send the command to get temperatures

		// Display temperature from each sensor
		for (int i = 0;  i < deviceCount;  i++)
		{
#ifdef DEBUG
			Serial.print("Sensor ");
			Serial.print(i+1);
			Serial.print(" : ");
#endif //DEBUG
			fTemp = oneWireTemp.getTempCByIndex(i);
			switch(i) {
				case 0: txPayload.oneWireTemp1 = 10 * fTemp + 0.5; break;
				case 1: txPayload.oneWireTemp2 = 10 * fTemp + 0.5; break;
				case 2: txPayload.oneWireTemp3 = 10 * fTemp + 0.5; break;
			}
#ifdef DEBUG
			if (fTemp != DEVICE_DISCONNECTED_C) 
			{
				Serial.print(fTemp);
				Serial.println(" C");
			} else {
				Serial.println("ERROR, sensor disconnected");
			}
#endif //DEBUG
		}

#ifdef DEBUG
		Serial.println("");
#endif //DEBUG

// 		fTemp = oneWireTemp.getTempCByIndex(0);
// 		txPayload.oneWireTemp1 = 10 * fTemp + 0.5;
// #ifdef DEBUG
// 		Serial.print("1-Wire Temp1:");
// 		Serial.println(txPayload.oneWireTemp1, 1);
// #endif
	
		oneWire.depower();
	
		LoRa_sendMessage(txPayload, payloadSize); // send a message
		delay(40); 		// [ms] Give RFM95W time to send the message
		LoRa.sleep(); 	// Put RFM95W in sleep mode
	}
}
