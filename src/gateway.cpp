// Copyright (c) 2023 Think5 GmbH, Mark Schmalohr, MIT license
#include <Arduino.h>
#include <AsyncJson.h>
#include <ArduinoJson.h>
#include "globalConfig.h"
#include "gateway.h"
#ifdef ESP32
#include <WiFi.h>
#else
#include <ESP8266WiFi.h>
#endif
#include <ModbusTCP.h>
#include <ModbusRTU.h>
#include <StreamBuf.h>
#include <SoftwareSerial.h>

#define BSIZE 1024
uint8_t buf1[BSIZE];
uint8_t buf2[BSIZE];
StreamBuf S1(buf1, BSIZE);
StreamBuf S2(buf2, BSIZE);
DuplexBuf P1(&S1, &S2);
DuplexBuf P2(&S2, &S1);

int DE_RE = 2;
static SoftwareSerial S;

ModbusRTU rtu;
ModbusTCP tcp;
IPAddress srcIp;

uint16_t transRunning = 0;  // Currently executed ModbusTCP transaction
uint8_t slaveRunning = 0;   // Current request slave

const int minRequestInterval = 60; // minimal intervall to poll client
unsigned long lastRequest = 0;
uint8_t lastSlave = 0;


bool cbTcpTrans(Modbus::ResultCode event, uint16_t transactionId, void* data) { // Modbus Transaction callback
  if (event != Modbus::EX_SUCCESS)                  // If transaction got an error
    Serial.printf("Modbus TCP result: %02X, Mem: %d\n", event, ESP.getFreeHeap());  // Display Modbus error code (222527)
  if (event == Modbus::EX_TIMEOUT) {    // If Transaction timeout took place
    Serial.println("Timeout");
    tcp.disconnect(tcp.eventSource());          // Close connection
    transRunning = 0;
    slaveRunning = 0;
  }
  return true;
}


bool cbRtuTrans(Modbus::ResultCode event, uint16_t transactionId, void* data) {
  if (event != Modbus::EX_SUCCESS)                  // If transaction got an error
    Serial.printf("\nModbus RTU result: %02X, Mem: %d\n", event, ESP.getFreeHeap());  // Display Modbus error code (222527)
  if (event == Modbus::EX_TIMEOUT) {    // If Transaction timeout took place
    Serial.println("Timeout");
    transRunning = 0;
    slaveRunning = 0;
  }
  return true;
}


// Callback receives raw data
Modbus::ResultCode cbTcpRaw(uint8_t* data, uint8_t len, void* custom) {
  auto src = (Modbus::frame_arg_t*) custom;

  if (transRunning) { // Note that we can't process new requests from TCP-side while waiting for responce from RTU-side.
    Serial.print("TCP IP in - ");
    Serial.print(IPAddress(src->ipaddr));
    Serial.printf(" Fn: %02X, len: %d \n", data[0], len);
    Serial.println("Trans running");
    tcp.setTransactionId(src->transactionId); // Set transaction id as per incoming request
    tcp.errorResponce(IPAddress(src->ipaddr), (Modbus::FunctionCode)data[0], Modbus::EX_SLAVE_DEVICE_BUSY);
    return Modbus::EX_SLAVE_DEVICE_BUSY;
  }

  Serial.printf("Modbus RTU request to address: %d\n", src->unitId);
  rtu.rawRequest(src->unitId, data, len, cbRtuTrans);
  //LG heatpump always has modbus address 33
  if (src->unitId == 33) {
    lastRequest = millis();
    lastSlave = src->unitId;
  }

  if (!src->unitId) { // If broadcast request (no responce from slave is expected)
    tcp.setTransactionId(src->transactionId); // Set transaction id as per incoming request
    tcp.errorResponce(IPAddress(src->ipaddr), (Modbus::FunctionCode)data[0], Modbus::EX_ACKNOWLEDGE);

    transRunning = 0;
    slaveRunning = 0;
    return Modbus::EX_ACKNOWLEDGE;
  }

  srcIp = IPAddress(src->ipaddr);
  slaveRunning = src->unitId;
  transRunning = src->transactionId;
  return Modbus::EX_SUCCESS;
}


// Callback receives raw data from ModbusTCP and sends it on behalf of slave (slaveRunning) to master
Modbus::ResultCode cbRtuRaw(uint8_t* data, uint8_t len, void* custom) {
  auto src = (Modbus::frame_arg_t*) custom;
  if (!transRunning) // Unexpected incoming data
    return Modbus::EX_PASSTHROUGH;
  tcp.setTransactionId(transRunning); // Set transaction id as per incoming request
  uint16_t succeed = tcp.rawResponce(srcIp, data, len, slaveRunning);
  if (!succeed) {
    Serial.println("TCP IP out - failed");
    Serial.printf("RTU Slave: %d, Fn: %02X, len: %d, ", src->slaveId, data[0], len);
    Serial.print("Response TCP IP: ");
    Serial.println(srcIp);
  }

  transRunning = 0;
  slaveRunning = 0;
  return Modbus::EX_PASSTHROUGH;
}


void gateway_setup() {
    // Setup only when in gateway mode
	if (cfgModbusGWActive == 1) {
        //Serial.begin(9600, SERIAL_8E1);
        if (strcmp(cfgRtu1Parity, "8E1") == 0) {
          S.begin(cfgRtu1BaudRate, SWSERIAL_8E1, PIN_RO, PIN_DI);
        }
        if (strcmp(cfgRtu1Parity, "8N1") == 0) {
          S.begin(cfgRtu1BaudRate, SWSERIAL_8N1, PIN_RO, PIN_DI);
        }

        tcp.server(); // Initialize ModbusTCP to pracess as server
        tcp.onRaw(cbTcpRaw); // Assign raw data processing callback

        rtu.begin(&S, PIN_DE_RE);  // Specify RE_DE control pin
        //rtu.begin(&S, PIN_DE_RE);
        rtu.master(); // Initialize ModbusRTU as master
        rtu.onRaw(cbRtuRaw); // Assign raw data processing callback

        Serial.println(F("\nRunning in Modbus RTU<->TCP Gateway mode now...\n"));
    }
}


void gateway_loop() {
    // Run only when in gateway mode
	if (cfgModbusGWActive == 1) {
        rtu.task();
        tcp.task();

        if (lastSlave && (millis() - lastRequest > minRequestInterval * 1000)) {
            rtu.rawRequest(lastSlave, (uint8_t*) "\x01\x00\x00\x00\x01", 5, cbRtuTrans);
            lastRequest = millis();
        }

        yield();
    }
}