// Copyright (c) 2022-23, Think5 GmbH, andreas.miketta, steff393, andy5macht, MIT license
// based on https://github.com/AMiketta/wbec and https://github.com/andy5macht/wbec and https://github.com/mark-sch/wbec
#include <Arduino.h>
#include <AsyncJson.h>
#include <ArduinoJson.h>
#include "globalConfig.h"
#include "logger.h"
#include "inverter.h"
#include <IPAddress.h>
#include <ModbusIP_ESP8266.h>
#include <ModbusRTU.h>
#include <SoftwareSerial.h>
#include "pvAlgo.h"

#define RINGBUF_SIZE 20

const uint8_t m = 1;

typedef struct rb_struct {
	uint8_t   id;       // rtu addr
	uint16_t reg;       // register
	uint16_t val;       // value
	uint16_t * buf;     // write: null  read: buffer where to write the response
} rb_t;


static IPAddress remote;   // Address of Modbus Slave device
static SoftwareSerial S;
static ModbusIP  mbtcp;       // Declare ModbusTCP instance
static ModbusRTU mbrtu2;   // Declare ModbusRTU instance to rtu device 2

static bool      inverterActive             = false;
static bool      isConnected                = false;

static uint16_t  inverterPort               = 0;
static uint16_t  inverterAddr               = 0;
static uint16_t  smartmetAddr               = 0;
static uint16_t  regAcCurrent               = 0;
static uint16_t  regPowerInv                = 0;
static uint16_t  regPowerInvS               = 0;
static uint16_t  regPowerMet                = 0;
static uint16_t  regPowerMetS               = 0;

static int16_t   power_inverter             = 0;
static int16_t   power_inverter_scale       = 0;
static int16_t   power_meter                = 0; // power (pos. = 'Einspeisung', neg. = 'Bezug')
static int16_t   power_meter_scale          = 0;
static uint16_t  ac_current                 = 0;
static uint16_t  power_house                = 0; 
static uint32_t  lastHandleCall             = 0;

static int16_t   pwrInv                     = 0;
static int16_t   pwrMet                     = 0;

static uint8_t   modbusFailureCnt			= 0;
static uint8_t   modbusResultCode			= 0;
static uint32_t  modbusLastMsgSentTime = 0;
uint16_t         contentrtu2[1][55];
uint32_t         modbusLastTimeRtu2 = 0;
static uint8_t   msgCnt = 0;
static uint8_t   id = 1;
static rb_t      rb[RINGBUF_SIZE];    // ring buffer
static uint8_t   rbIn  = 0;           // last element, which was written to ring buffer
static uint8_t   rbOut = 0;           // last element, which was read from ring buffer


static bool cb(Modbus::ResultCode event, uint16_t transactionId, void *data) {
	if (event != Modbus::EX_SUCCESS) {
		Serial.printf("Modbus result: %02X\n", event);
	}
#ifdef DEBUG_INVERTER
	if (event == Modbus::EX_TIMEOUT) {
		Serial.println("Timeout");
	}
#endif
	return true;
}


static int16_t pow_int16(int16_t base, uint16_t exp) {
	int16_t x = 1;
	for (uint16_t i = 0; i < exp; i++) {
		x = x * base;
	}
	return(x);
}


static boolean mb_rtu2_available() {
	// don't allow new msg, when communication is still active (ca.30ms) or minimum delay time not exceeded
	if (mbrtu2.slave() || millis() - modbusLastMsgSentTime < cfgMbDelay) {
		return(false);
	} else {
		return(true);
	}
}


static void timeout(uint8_t id) {
	if (cfgResetOnTimeout) {
		for (int i =  1; i <= 16; i++) { contentrtu2[0][i] = 0;	}
		for (int i = 49; i <= 54; i++) { contentrtu2[0][i] = 0;	}
	}
}


static bool cbWrite(Modbus::ResultCode event, uint16_t transactionId, void* data) {
	modbusResultCode = event;
	if (event) {
		LOG(m, "RTU2: Comm-Failure BusID %d", mbrtu2.slave());
		if (modbusFailureCnt < 250) {
			modbusFailureCnt++;
		}
		if (modbusFailureCnt == 10) {
			// too many consecutive timeouts --> reset values
			LOG(m, "RTU2: Timeout BusID %d", mbrtu2.slave());
			timeout(id);
		}
	} else {
		// no failure
		modbusFailureCnt = 0;
	}
	
	log(m, "RTU2 ResultCode: 0x" + String(event, HEX) + ", BusID: "+ mbrtu2.slave());
	return(true);
}


void mb_rtu2_setup() {
	// Setup only when NOT in gateway mode
	//if (cfgModbusGWActive == 0) {
		// setup SoftwareSerial and Modbus Master
		LOG(m, "Setup Modbus RTU on interface rtu2","");
		S.begin(9600, SWSERIAL_8N1, PIN_RO_RTU2, PIN_DI_RTU2); // inverted
		
		mbrtu2.begin(&S, PIN_DE_RE_RTU2);
		mbrtu2.master();
		modbusFailureCnt = 0;
		modbusResultCode = 0;


		//Modbus RTU queries
		switch(cfgInverterType) { 
			case 31: // SDM630 registers for modbusRTU on rtu device 2
					inverterAddr = 2;
					smartmetAddr = 2;
					regPowerMet  = 30053;   // modbus register for "Total Real Power (sum of active phases)" int16 in Watts
					break;
			default: ;// nothing
		}
	//}
}


void inverter_setup() {
	if (cfgInverterType >= 30) {
		mb_rtu2_setup();
	}

	//Modbus TCP queries
	if (strcmp(cfgInverterIp, "") != 0) {
		if (remote.fromString(cfgInverterIp)) {
			mbtcp.client(); // Act as Modbus TCP server
			inverterActive = true;

			switch(cfgInverterType) {   // select the port and addresses based on the different inverter types
				case 1: // SolarEdge (sunspec) registers for modbusTCP
						inverterPort = 1502;
						inverterAddr = 1;
						smartmetAddr = 1;
						regAcCurrent = 40071;   // 40072 1 I_AC_Current uint16 Amps AC Total Current value
						regPowerInv  = 40083;   // modbus register for "AC Power value", int16 in Watts 
						regPowerInvS = 40084;   // modbus register for "AC Power scale factor" int16
						regPowerMet  = 40206;   // modbus register for "Total Real Power (sum of active phases)" int16 in Watts
						regPowerMetS = 40210;   // modbus register for "AC Real Power Scale Factor" int16 SF
						break;
				case 2: // Fronius (sunspec) registers for modbusTCP
						inverterPort = 502;
						inverterAddr = 1;
						smartmetAddr = 240;
						//regAcCurrent = 40071;   // 40072 1 I_AC_Current uint16 Amps AC Total Current value
						regPowerInv  = 40083;   // modbus register for "AC Power value", int16 in Watts 
						regPowerInvS = 40084;   // modbus register for "AC Power scale factor" int16
						regPowerMet  = 40087;   // modbus register for "Total Real Power (sum of active phases)" int16 in Watts
						regPowerMetS = 40091;   // modbus register for "AC Real Power Scale Factor" int16 SF
						break;
				case 3: // Kostal (sunspec) registers for modbusTCP
						inverterPort = 502;
						inverterAddr = 1;
						smartmetAddr = 240;
						//regAcCurrent = 40071;   // 40072 1 I_AC_Current uint16 Amps AC Total Current value
						//regPowerInv  = 40083;   // modbus register for "AC Power value", int16 in Watts 
						//regPowerInvS = 40084;   // modbus register for "AC Power scale factor" int16
						regPowerMet  = 40087;   // modbus register for "Total Real Power (sum of active phases)" int16 in Watts
						regPowerMetS = 40091;   // modbus register for "AC Real Power Scale Factor" int16 SF
						break;
				case 20: // Deye registers for modbusTCP
						inverterPort = 8899;
						inverterAddr = 1;
						smartmetAddr = 1;
						//regAcCurrent = 40071;   // 40072 1 I_AC_Current uint16 Amps AC Total Current value
						regPowerInv  = 40083;   // modbus register for "AC Power value", int16 in Watts 
						//regPowerInvS = 40084;   // modbus register for "AC Power scale factor" int16
						//regPowerMet  = 40206;   // modbus register for "Total Real Power (sum of active phases)" int16 in Watts
						//regPowerMetS = 40210;   // modbus register for "AC Real Power Scale Factor" int16 SF
						break;
				default: ;// nothing
			}
			// overwrite, if specifically configured by parameter
			if (cfgInverterPort) { inverterPort = cfgInverterPort; }
			if (cfgInverterAddr) { inverterAddr = cfgInverterAddr; }
			if (cfgInvSmartAddr) { smartmetAddr = cfgInvSmartAddr; }
		}
	}
}


void mb_rtu2_loop() {
	// Run only when NOT in gateway mode
	//if (cfgModbusGWActive == 0) {
		// When pointers of the ring buffer are not equal, then there is something to send
		if (rbOut != rbIn) {
			if (mb_rtu2_available()) {			// check, if bus available
				rbOut = (rbOut+1) % RINGBUF_SIZE; 		// increment pointer, but take care of overflow
				if (rb[rbOut].buf != NULL) {
					//mbrtu2.readHreg (rb[rbOut].id + 1, rb[rbOut].reg,  rb[rbOut].buf, 1, cbWrite);
				} else {
					//mbrtu2.writeHreg(rb[rbOut].id + 1, rb[rbOut].reg, &rb[rbOut].val, 1, cbWrite); 
				}
				modbusLastMsgSentTime = millis();
			}
		}

		if (modbusLastTimeRtu2 == 0 || millis() - modbusLastTimeRtu2 > (cfgMbCycleTime*1000)) {
			if (mb_rtu2_available()) {
				//Serial.print(millis());Serial.print(": RTU2, Sending to BusID: ");Serial.print(id+1);Serial.print(" with msgCnt = ");Serial.println(msgCnt);
				if (!modbusResultCode) {
					log(m, String(millis()) + ": RTU2, BusID=" + (2) + ",msgCnt=" + msgCnt);
				}
				switch(msgCnt) {
					case 0:                                                   
						mbrtu2.readIreg (2,  53,              (uint16_t *) &ac_current,  1, cbWrite); 
						LOG(m, "Sending RTU2 query...","");
						break;
					//case 1: if (!modbusResultCode)                          { mbrtu2.readIreg (1, 100,              &contentrtu2[0][15],  17, cbWrite); } break;
					//case 2: if (!modbusResultCode)                          { mbrtu2.readIreg (1, 117,              &contentrtu2[0][32],  17, cbWrite); } break;
					//case 3: if (!modbusResultCode)                          { mbrtu2.readHreg (1, REG_WD_TIME_OUT,  &contentrtu2[0][49],   1, cbWrite); } break;
					//case 6: if (!modbusResultCode)                          { mbrtu2.readHreg (1, REG_CURR_LIMIT,   &contentrtu2[0][53],   2, cbWrite); } break;
					//case 7: if (!modbusResultCode)                          { mbrtu2.writeHreg(1, REG_WD_TIME_OUT,  &cfgMbTimeout,      1, cbWrite); } break;
					//case 8: if (!modbusResultCode)                          { mbrtu2.writeHreg(1, REG_CURR_LIMIT_FS,&cfgFailsafeCurrent,1, cbWrite); } break;
					default: ; // do nothing, should not happen
				}
				modbusLastMsgSentTime = millis();
				msgCnt++;
				
				if (msgCnt > 2) {
					msgCnt = 0;
					//Serial.print("Time:");Serial.println(millis()-modbusLastTimeRtu2);
					modbusLastTimeRtu2 = millis();
				}
			}
		}
		mbrtu2.task();
		yield();
	//}
}


void inverter_loop() {
	// Run Modbus RTU queries on interface rtu2 if configured
	if (cfgInverterType >= 30) {
		mb_rtu2_loop();
	}

	if ((millis() - lastHandleCall < (uint16_t)cfgPvCycleTime * 1000) ||     // avoid unnecessary frequent calls
			(inverterActive == false)) {
		return;
	}
	lastHandleCall = millis();

	isConnected = mbtcp.isConnected(remote);
	
	if (!isConnected) {            // Check if connection to Modbus Slave is established
		mbtcp.connect(remote, inverterPort);    // Try to connect if no connection
	} else {  
		if (regAcCurrent) { mbtcp.readHreg(remote, regAcCurrent, (uint16_t *) &ac_current,           1, cb, inverterAddr); } 
		if (regPowerInv)  { mbtcp.readHreg(remote, regPowerInv,  (uint16_t *) &power_inverter,       1, cb, inverterAddr); }    //Power Inverter
		if (regPowerInvS) { mbtcp.readHreg(remote, regPowerInvS, (uint16_t *) &power_inverter_scale, 1, cb, inverterAddr); }    //Power Inverter Scale Factor
		if (regPowerMet)  { mbtcp.readHreg(remote, regPowerMet,  (uint16_t *) &power_meter,          1, cb, smartmetAddr); }    //Power Zähler
		if (regPowerMetS) { mbtcp.readHreg(remote, regPowerMetS, (uint16_t *) &power_meter_scale,    1, cb, smartmetAddr); }    //Power Zähler Scale Factor
	}
	mbtcp.task();  // Common local Modbus task

	if (power_inverter_scale < 0) {  // if negative, then divide
		pwrInv = power_inverter / pow_int16(10, (uint16_t)(-power_inverter_scale));
	} else {                         // if positive, then multiply
		pwrInv = power_inverter * pow_int16(10, (uint16_t)  power_inverter_scale);
	}
	if (power_meter_scale < 0) {     // if negative, then divide
		pwrMet = power_meter / pow_int16(10, (uint16_t)(-power_meter_scale));
	} else {                         // if positive, then multiply
		pwrMet = power_meter * pow_int16(10, (uint16_t)  power_meter_scale);
	}
	power_house = pwrInv - pwrMet;

	pv_setWatt(-pwrMet); // pvAlgo expects the value inverted 
}


String inverter_getStatus() {
	StaticJsonDocument<INVERTER_JSON_LEN> data;
	data[F("inverter")][F("isConnected")]  = String(isConnected);
	data[F("power")][F("AC_Total")]        = String(ac_current);
	data[F("power")][F("house")]           = String(power_house);
	data[F("power")][F("inverter")]        = String(power_inverter);
	data[F("power")][F("inverter_scale")]  = String(power_inverter_scale);
	data[F("power")][F("meter")]           = String(power_meter);
	data[F("power")][F("meter_scale")]     = String(power_meter_scale);

  String response;
  serializeJson(data, response);
	return(response);
}


int16_t inverter_getPwrInv() {
	return(pwrInv);
}


int16_t inverter_getPwrMet() {
	return(pwrMet);
}
