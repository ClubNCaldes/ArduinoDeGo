/****************************************************************************
    Copyright (C) 2002,2003,2004 Alex Shepherd

    This library is free software; you can redistribute it and/or
    modify it under the Serials of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

*****************************************************************************

 Title :   LocoNet Infrared TV/VCR Remote controlled Throttle
 Author:   Alex Shepherd <kiwi64ajs@sourceforge.net>
 Date:     4-Dec-2004
 Software: AVR-GCC
 Target:   AtMega8

 DESCRIPTION
    This project is a LocoNet throttle which gets it commands from a TV/VCR
	Infrared remote control. Initially this will only work with the Sony SIRCS
	remotes as this is what I have several of but support for others will
	probably be added.
	
*****************************************************************************/

#include <LocoNet.h>
#include <Psx.h>

//PSX Pins
#define dataPin 12
#define cmndPin 11
#define attPin 10
#define clockPin 13

//PSX DenshaDeGo controller button masks
#define MASK_SPEED  0x080F
#define MASK_BREAK  0xF000
#define MASK_SELECT 0x80
#define MASK_START  0x10
#define MASK_A      0x100
#define MASK_B      0x200
#define MASK_C      0x400

#define SPEED_0  0xF
#define SPEED_1  0x80E
#define SPEED_2  0xE
#define SPEED_3  0x80B
#define SPEED_4  0xB
#define SPEED_5  0x80A

#define BREAK0 0xD000
#define BREAK1 0x7000
#define BREAK2 0x5000
#define BREAK3 0xE000
#define BREAK4 0xC000
#define BREAK5 0x6000
#define BREAK6 0x4000
#define BREAK7 0xB000
#define BREAK8 0x9000
#define BREAK9 0x3000
#define BREAK10 0x1000
#define BREAK11 0xA000
#define BREAK12 0x8000
#define BREAK13 0x2000
#define BREAK14 0x0

//# of 100ms cycles to accel or break
#define CYCLES 5 //Every 1/2 second

// Address Recall Stack Length
#define RECALL_STACK_LEN  6
#define AJS_RECALL_STACK  { 38, 44, 53, 99, 191, 192 }

typedef struct
{
	word	RecallStack[ RECALL_STACK_LEN ] ;
	word	LastAddress ;
	byte	LastSlot ;
} TH_EE_CONFIG ;

TH_EE_CONFIG		eeConfig __attribute__((section(".eeprom")))
	= { AJS_RECALL_STACK, 0, 0xFF };
	
LocoNetThrottleClass  Throttle ;
word                  LocoAddr ;
byte                  RecallIndex ;
LnBuf                 LnRxBuffer ;
lnMsg                 *RxPacket ;
byte                  i ;
byte                  tByte ;
uint32_t              LastThrottleTimerTick;

Psx Psx;
unsigned int psxdata = 0;                                    // data stores the controller response
unsigned int psxold = 0;
int acceleration=0;
int cyclecount=0;

int locoarray[]={12,30,40};
int locoindex=0;

#define MAXLOCOS 3

void notifyThrottleAddress( uint8_t UserData, TH_STATE State, uint16_t Address, uint8_t Slot )
{
  Serial.print("Throttle Address: ");
  Serial.println(Address);
}

void notifyThrottleSpeed( uint8_t UserData, TH_STATE State, uint8_t Speed )
{
  Serial.print("Throttle Speed: ");
  Serial.println(Speed);
}

void notifyThrottleDirection( uint8_t UserData, TH_STATE State, uint8_t Direction )
{
  Serial.print("Throttle Direction: ");
  Serial.println(Direction ? "Reverse" : "Forward");
}

void notifyThrottleFunction( uint8_t UserData, uint8_t Function, uint8_t Value )
{
  Serial.print("Throttle Function: ");   
  Serial.println(Function, DEC);
}

void notifyThrottleSlotStatus( uint8_t UserData, uint8_t Status ){}

void notifyThrottleState( uint8_t UserData, TH_STATE PrevState, TH_STATE State )
{
  Serial.print("Throttle State: ");
  Serial.print(State, DEC);
  Serial.print(' ');
  Serial.println(Throttle.getStateStr(State));
}

void notifyThrottleError( uint8_t UserData, TH_ERROR Error )
{
  Serial.print("Throttle Error: ");
  Serial.print(Error, DEC);
  Serial.print(' ');
  Serial.println(Throttle.getErrorStr(Error));
}

void setup()
{
  // First initialize the LocoNet interface
  LocoNet.init(7);

  // Configure the serial port for 57600 baud
  Serial.begin(9600);
  
  Throttle.init(0, 0, 9999);

  Psx.setupPins(dataPin, cmndPin, attPin, clockPin, 5);  // Defines what each pin is used
  // (Data Pin #, Cmnd Pin #, Att Pin #, Clk Pin #, Delay)
  // Delay measures how long the clock remains at each state,
  // measured in microseconds.
  // too small delay may not work (under 5)
}

boolean isTime(unsigned long *timeMark, unsigned long timeInterval)
{
    unsigned long timeNow = millis();
    if ( timeNow - *timeMark >= timeInterval) {
        *timeMark = timeNow;
        return true;
    }    
    return false;
}

void loop()
{  
  // Check for any received LocoNet packets
  RxPacket = LocoNet.receive() ;
  if( RxPacket )
  {
    digitalWrite(13, LOW);
    
    if( !LocoNet.processSwitchSensorMessage(RxPacket) )
      Throttle.processMessage(RxPacket) ; 
  }
  
  psxdata = Psx.read();
  if (psxdata != psxold && (psxdata & 0xF000) != 0xF000)
  {   
    if (psxdata & MASK_START)
      Throttle.setDirection(!Throttle.getDirection());
    if (psxdata & MASK_SELECT)
      Throttle.setFunction(0, !Throttle.getFunction(0));
    if (psxdata & MASK_A)
    {      
      if (Throttle.getState()==TH_ST_FREE)
      {
        locoindex--;
        if (locoindex==-1)
          locoindex=MAXLOCOS-1;
        Throttle.setAddress(locoarray[locoindex]);
      }
      else
      {
        Throttle.setFunction(0, 0);
        Throttle.freeAddress(locoarray[locoindex]);
        Throttle.releaseAddress();
      }
    }
    
    if (psxdata & MASK_B)
      Throttle.releaseAddress();
      
      
    if (psxdata & MASK_C)
    {
      if (Throttle.getState()==TH_ST_FREE)
      {
        locoindex++;
        if (locoindex==MAXLOCOS)
          locoindex=0;
        Throttle.setAddress(locoarray[locoindex]);
      } 
      else
      {
        Throttle.setFunction(0, 0);
        Throttle.freeAddress(locoarray[locoindex]);
        Throttle.releaseAddress();
      }
    }
      
    switch (psxdata & MASK_SPEED)
    {
      case SPEED_0:
        if (acceleration>0) acceleration=0;
        break;
      case SPEED_1:
        acceleration=1;
        break;
      case SPEED_2:
        acceleration=2;
        break;
      case SPEED_3:
        acceleration=3;
        break;
      case SPEED_4:
        acceleration=4;
        break;
      case SPEED_5:
        acceleration=5;
        break;
    }
    
    switch (psxdata & MASK_BREAK)
    {
      case BREAK0:
        if (acceleration<0) acceleration=0;
        break;
      case BREAK1:
        acceleration=-1;
        break;
      case BREAK2:
        acceleration=-2;
        break;
      case BREAK3:
        acceleration=-3;
        break;
        case BREAK4:
        acceleration=-4;
        break;
        case BREAK5:
        acceleration=-5;
        break;
        case BREAK6:
        acceleration=-6;
        break;
        case BREAK7:
        acceleration=-7;
        break;
        case BREAK8:
        acceleration=-8;
        break;
        case BREAK9:
        acceleration=-9;
        break;
        case BREAK10:
        acceleration=-10;
        break;
        case BREAK11:
        acceleration=-11;
        break;
        case BREAK12:
        acceleration=-12;
        case BREAK13:
        case BREAK14:
          Throttle.setSpeed(0);
          acceleration=0;
    }
          
    psxold=psxdata;
  }
  
  if( Serial.available())
  {
    int16_t inChar = toupper(Serial.read());
    switch(inChar){
      case 'A': Throttle.setAddress(12);
                break;
      case 'X': Throttle.freeAddress(12);
                break;
      case 'Q': Throttle.releaseAddress(); 
                break;
      case 'F': Throttle.setDirection(0); 
                break;
      case 'R': Throttle.setDirection(1); 
                break;
      case 'T': Throttle.setDirection(!Throttle.getDirection());
                break;
      case '[': if(Throttle.getSpeed() > 0 )
                  Throttle.setSpeed(Throttle.getSpeed() - 1);
                break;
      case ']': if(Throttle.getSpeed() < 127 )
                  Throttle.setSpeed(Throttle.getSpeed() + 1);
                break;
      case ' ': Throttle.setSpeed(0); break;
      default:  if( (inChar >= '0') && (inChar <= '8'))
                  Throttle.setFunction( inChar - '0', !Throttle.getFunction(inChar - '0'));
                break;
    }
  }
  
  if(isTime(&LastThrottleTimerTick, 100))
  {
    Throttle.process100msActions() ; 
    digitalWrite(13, HIGH);
    cyclecount+=1;
    
    if (cyclecount==CYCLES)
    {
      if (acceleration!=0)
        Throttle.setSpeed(Throttle.getSpeed() + acceleration);
      cyclecount=0;
    }
  }
  delay(20);
}
