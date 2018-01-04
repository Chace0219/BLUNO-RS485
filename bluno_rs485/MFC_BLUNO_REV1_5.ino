#include "Arduino.h"

// BLE Protocol Header 
#include "PlainProtocol.h"

// IEEE754 Float Convert Header
#include "IEEE754tools.h"
//
#include <SoftwareSerial.h>

#include <TimerOne.h>
#include "FBD.h"

SoftwareSerial mySerial(3, 2); // RX, TX
//PlainProtocol constructor, define the Serial port and the baudrate.
PlainProtocol myBLUNO(mySerial,115200);

// Status Macro Definitions
#define READY 0 
#define SCANNING 1 
#define SCANNED 2 
#define CONNECTED 3
#define TIMEOUT 9

#define THREADSTOP 0
#define THREADING  1

// 
#define ACK 0x06
#define NAK 0x16


// Device status structure variable                                            
struct
{
    uint8_t MACID; // 
    int FlowRate; //
    unsigned int SetPoint; //
    
    unsigned CommStatus : 4;
    unsigned RetryCnt : 3;
    uint8_t CurrScanID;
    unsigned WriteQueue: 1;
    uint8_t PreSetPoint;
    uint8_t nCommIndex;
}mystatus;
static TON ScanningTON, ConnectTON;

/*
 * Func Name : InitVars
 * Description : Initialize Variales for Status
*/
void InitVars()
{
    mystatus.MACID = 0;
    mystatus.FlowRate = 0;
    mystatus.SetPoint = 0;
    mystatus.CommStatus = READY;
    mystatus.RetryCnt = 0;
    mystatus.SetPoint = 1000;
    mystatus.CurrScanID = 1;
    mystatus.WriteQueue = 0;
    mystatus.nCommIndex = 0;

    // When MACID Scanning Status, it is run per 500ms
    ScanningTON.IN = 0;
    ScanningTON.PT = 500;
    ScanningTON.ET = 0;
    ScanningTON.Q = 0;

    // When MFC Device is connected, it is run per 500ms
    ConnectTON.IN = 0;
    ConnectTON.ET = 0;
    ConnectTON.PT = 500;
    ConnectTON.Q = 0;
}

// Communication Buffer
uint8_t RS485Buff[256];
uint8_t nIndex = 0; 

/**/
void setup() {

    myBLUNO.init();
    InitVars();

    Serial.begin(9600);
    
    Timer1.initialize(5000);
    Timer1.attachInterrupt(BLUNOCheck); // blinkLED to run every 0.15 seconds
    delay(1000);
}

// Calculate Checksum
uint8_t CalcCheckSum(uint8_t *buff, uint8_t nLength)
{
    uint8_t index = 0;
    uint8_t nResult = 0;
    for(index = 0; index < nLength; index++)
        nResult = nResult + buff[index];    
    return nResult;
}

#define TIMEOUTS 500 // 500ms
#define CHARINTV 200 // 200ms

// Write Commnad and Get Response via RS485
uint8_t WriteAndRead(uint8_t *buff, uint8_t nLength)
{
    uint8_t nRecving = false;
    uint8_t index = 0;
    unsigned long nMs = 0;

    // Send Commnad one byte one.
    for(index = 0; index < nLength; index++)
        Serial.write(buff[index]);

    // Delay Response Time.
    nIndex = 0;
    nMs = millis();
    while(!Serial.available())
    {
        if(nMs + TIMEOUTS < millis())
            return 0;  
    }

    // 
    if(Serial.available())
    {
        buff[nIndex] = Serial.read();
        nIndex++;
    }
    
    // 
    while(true)
    {
        nMs = millis();
        while(!Serial.available())
        { // if delay time is longer that CHARINTV, it mean that End response.
            if(nMs + CHARINTV < millis())
                return nIndex;  
        }
        buff[nIndex] = Serial.read();
        nIndex++;
    }
}

/* Function for Sending SetPoint Command to MFC device. */
bool PutSetPoint(unsigned int nSetPoint)
{
    /* Packet Generate */
    // 20 02 81 07 9E 01 69 00 00 80 3F 00 51
    RS485Buff[0] = mystatus.MACID;
    RS485Buff[1] = 0x02;
    RS485Buff[2] = 0x81;
    RS485Buff[3] = 0x07;
    RS485Buff[4] = 0x9E;
    RS485Buff[5] = 0x01;
    RS485Buff[6] = 0x69;

    // 11 - 14 : float Flow rate
    _FLOATCONV flc; // IEEE754 Format Conversion struct , It is implemented in IEEE754tools.h
    // 1000 : 1.0, 100: 0.1
    flc.f = ((float)nSetPoint) / 1000.0;  // I have implemented that if you set 100 in phone app, program sends 1000.
    /* 
     *  Because we have to set 0.1 resolution. As you know, If we set 0 ~ 100(per 1 uint), we cannot set 0.1 resolution.
        Make Sense?
    */
    // Hex value is stored in flc.b[0] ~ b[3].
    RS485Buff[7] = flc.b[0];
    RS485Buff[8] = flc.b[1];
    RS485Buff[9] = flc.b[2];
    RS485Buff[10] = flc.b[3];
    RS485Buff[11] = 0x00;
    
    // 
    RS485Buff[12] = CalcCheckSum(RS485Buff + 1, 11);
    uint8_t nCnt = WriteAndRead(RS485Buff, 13);
    if(nCnt == 2)
      return true;
    // Fail to Read Response
    return false;
}

/* Getting SetPoint */
bool GetSetPoint(uint8_t nMACID)
{
    // Getnerate Indicate flow Packet
    // 20 02 80 03 9E 01 69 00 8D   
    RS485Buff[0] = nMACID;
    RS485Buff[1] = 0x02;
    RS485Buff[2] = 0x80;
    RS485Buff[3] = 0x03;
    RS485Buff[4] = 0x9E;
    RS485Buff[5] = 0x01;
    RS485Buff[6] = 0x69;
    RS485Buff[7] = 0x00;
    
    // Add CheckSum
    RS485Buff[8] = CalcCheckSum(RS485Buff + 1, 7);

    // Write Packet and Read Response
    uint8_t nCnt = WriteAndRead(RS485Buff, 9);
    if(nCnt < 3)
    {
        mystatus.SetPoint = 0;    
        return false;
    }
    _FLOATCONV flc;
    flc.b[0] = RS485Buff[8];
    flc.b[1] = RS485Buff[9];
    flc.b[2] = RS485Buff[10];
    flc.b[3] = RS485Buff[11];
    mystatus.SetPoint = (unsigned int)(flc.f * 1000.0);
    return true;       
}

/* Getting Flow Rate with command */
bool GetFlowRate(uint8_t nMACID)
{
    // Indicate flow Rate
    // 20 02 80 03 A9 01 A8 00 D7
    RS485Buff[0] = nMACID;
    RS485Buff[1] = 0x02;
    RS485Buff[2] = 0x80;
    RS485Buff[3] = 0x03;
    RS485Buff[4] = 0xA9;
    RS485Buff[5] = 0x01;
    RS485Buff[6] = 0xA8;
    RS485Buff[7] = 0x00;
    
    //
    RS485Buff[8] = CalcCheckSum(RS485Buff + 1, 7);
    uint8_t nCnt = WriteAndRead(RS485Buff, 9);
    if(nCnt < 3)
    {
        mystatus.FlowRate = 0;    
        return false;
    }

    // 11 - 14 : float Flow rate
    
    _FLOATCONV flc;
    flc.b[0] = RS485Buff[8];
    flc.b[1] = RS485Buff[9];
    flc.b[2] = RS485Buff[10];
    flc.b[3] = RS485Buff[11];
    mystatus.FlowRate = (int)(flc.f * 1000.0);

    return true;

}

//
bool SCANByMACID(uint8_t nScanID)
{
    //    20 02 80 03 03 01 01 00 8A
    // Packet Gen 
    RS485Buff[0] = nScanID;
    RS485Buff[1] = 0x02;
    RS485Buff[2] = 0x80;
    RS485Buff[3] = 0x03;
    RS485Buff[4] = 0x03;
    RS485Buff[5] = 0x01;
    RS485Buff[6] = 0x01;
    RS485Buff[7] = 0x00;
    
    //
    RS485Buff[8] = CalcCheckSum(RS485Buff + 1, 7);
    //RS485Buff[8] = CalcCheckSum(RS485Buff, 8);
    uint8_t nCnt = WriteAndRead(RS485Buff, 9);
    if(nCnt < 2)
        return false;

    return true;       
}

void loop()
{
    uint8_t index, SmallIndex;

    // When Program enters into Scanning Status
    ScanningTON.IN = (ScanningTON.Q == 0) && (mystatus.CommStatus == SCANNING);
    TONFunc(&ScanningTON);
    if(ScanningTON.Q)
    { // every Scanning interval, Program scan device with current ID
        if(SCANByMACID(mystatus.CurrScanID) == 0)
        {
            mystatus.CurrScanID++; // ID increase
            if(mystatus.CurrScanID > 50)
                mystatus.CommStatus = TIMEOUT;
        }
        else
        { // If device is found, Program enters into SCANNED Status
            mystatus.MACID = mystatus.CurrScanID;
            mystatus.CommStatus = SCANNED;
        }
    }

    // When App is connected specified device.
    ConnectTON.IN = (ConnectTON.Q == 0) && mystatus.CommStatus == CONNECTED;
    TONFunc(&ConnectTON);
    if(ConnectTON.Q)
    { // Every specified interval, Program Get Current Status via RS485.
        if(mystatus.nCommIndex == 0)
        {
            mystatus.nCommIndex = 1;
            if(GetFlowRate(mystatus.MACID) == 0)
            { 
                mystatus.RetryCnt++;
                if(mystatus.RetryCnt > 3)
                {
                    mystatus.RetryCnt = 0;
                    mystatus.CommStatus = TIMEOUT;    
                }
            }
            else
                mystatus.RetryCnt = 0;
             
        }
        else if(mystatus.nCommIndex == 1)
        {
            mystatus.nCommIndex = 0;
            if(GetSetPoint(mystatus.MACID) == 0)
            {
                mystatus.RetryCnt++;
                if(mystatus.RetryCnt > 3)
                {
                    mystatus.RetryCnt = 0;
                    mystatus.CommStatus = TIMEOUT;    
                }
            }
            else
                mystatus.RetryCnt = 0;
        }

    }
    /* If SetPoint Command is in queue. */
    if(mystatus.WriteQueue)
    {
        mystatus.WriteQueue = 0;
        if(mystatus.CommStatus == CONNECTED)
            PutSetPoint(mystatus.PreSetPoint); // SetPoint Command Issue function
    }

}

void BLUNOCheck()
{
    if(myBLUNO.available())
    { // If Any scan command or SetPoint, SetFlowRate command from App, 
      // Program into here.
        if(myBLUNO.equals("SCANCMD"))
        { /* When user click SCAN button on App.*/
            uint8_t nCommand = myBLUNO.read();
            if(nCommand == 0)
            {
                mystatus.CommStatus = READY;
            }
            if(nCommand == 1 && (mystatus.CommStatus == READY || mystatus.CommStatus == TIMEOUT))
            {
                mystatus.CurrScanID = 1;
                mystatus.CommStatus = SCANNING;
            }
            if(nCommand == 2 && mystatus.CommStatus == SCANNED)
            {
                mystatus.CurrScanID = mystatus.MACID + 1;
                mystatus.CommStatus = SCANNING;
            }
        }
        else if(myBLUNO.equals("ATTACH"))
        { /* When user click Attach button on App.*/
            uint8_t nCommand = myBLUNO.read();
            if(nCommand)
                mystatus.CommStatus = CONNECTED;
        }
        else if(myBLUNO.equals("GetCommStatus"))
        { /* App sends GetCommstatus Command every 1 sec. */
            myBLUNO.write("GetCommStatus", mystatus.CommStatus);              
        }
        else if(myBLUNO.equals("PutSetPoint"))
        { /* App sends GetCommstatus Command every 1 sec. */
            if(mystatus.CommStatus == CONNECTED)
            { // Only Arduino is correctly connected to MFC
                mystatus.SetPoint = myBLUNO.read(); // if app setting is 100%, myBLUNO.read() returns 1000
                // , if it is 10.0%, 100 
                mystatus.WriteQueue = 1; // It lets to Communication unit that 
                // it has to issue modify setpoint command for MFC
                mystatus.PreSetPoint = mystatus.SetPoint;// so it has value range from 0 to 1000.
            }
        }
        else if(myBLUNO.equals("GetMACID"))
        { /* App sends GetMACID Command every 1 sec. */
            myBLUNO.write("GetMACID", mystatus.MACID);
        }
        else if(myBLUNO.equals("GetFlowRate"))
        { /* App sends GetFlowRate``                                   Command every 1 sec. */
            myBLUNO.write("GetFlowRate", mystatus.FlowRate);
        }
        else if(myBLUNO.equals("GetSetPoint"))
        { /* App sends GetSetPoint Command every 1 sec. */
            myBLUNO.write("GetSetPoint", mystatus.SetPoint);
        }
    }    
}


