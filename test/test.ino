#include <Arduino.h>
#include <U8x8lib.h>
#include <Wire.h>

#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

#include <Serial_CAN_Module.h>

#define STANDARD_CAN_11BIT      1       // That depands on your car. some 1 some 0. 

static const int RXPin = 0, TXPin = 2;          // 6, 7
static const uint32_t GPSBaud = 9600;

// The TinyGPSPlus object
TinyGPSPlus gps;

U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* clock=*/ PIN_WIRE_SCL, /* data=*/ PIN_WIRE_SDA, /* reset=*/ U8X8_PIN_NONE);   // OLEDs without Reset of the Display

const int pinLed = LED_BUILTIN;
const int pinBtn = 1;

int speed = 0;
int rpm   = 0;

int disp_st = 0;        // 0-location, 1-speed, 3-rpm

float __longitude = 0;      
float __latitude  = 0;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);
Serial_CAN can;

#define PID_ENGIN_PRM       0x0C
#define PID_VEHICLE_SPEED   0x0D

#if STANDARD_CAN_11BIT
#define CAN_ID_PID          0x7DF
#else
#define CAN_ID_PID          0x18db33f1
#endif


void sendPid(unsigned char __pid) {
    unsigned char tmp[8] = {0x02, 0x01, __pid, 0, 0, 0, 0, 0};
    
#if STANDARD_CAN_11BIT
    can.send(CAN_ID_PID, 0, 0, 8, tmp);   // SEND TO ID:0X55
#else
    can.send(CAN_ID_PID, 1, 0, 8, tmp);   // SEND TO ID:0X55
#endif
}

bool getSpeed(int *s)
{
    sendPid(PID_VEHICLE_SPEED);
    unsigned long __timeout = millis();

    while(millis()-__timeout < 1000)      // 1s time out
    {
        unsigned long id  = 0;
        unsigned char buf[8];

        if (can.recv(&id, buf)) {                // check if get data

            if(buf[1] == 0x41)
            {
                *s = buf[3];
                return 1;
            }
        }
        
        if(0 == digitalRead(pinBtn))break;
    }

    return 0;
}

bool getRPM(int *r)
{
    sendPid(PID_ENGIN_PRM);
    unsigned long __timeout = millis();

    while(millis()-__timeout < 1000)      // 1s time out
    {
        unsigned long id  = 0;
        unsigned char buf[8];

        if (can.recv(&id, buf)) {                // check if get data

            if(buf[1] == 0x41)
            {
                *r = (256*buf[3]+buf[4])/4;
                return 1;
            }
        }
        
        if(0 == digitalRead(pinBtn))break;
    }

    return 0;
}


int getButton()
{
    if(0 == digitalRead(pinBtn))
    {
        delay(10);
        if(0 == digitalRead(pinBtn))
        {
            digitalWrite(pinLed, HIGH);

            disp_st++;
            if(disp_st > 2)disp_st = 0;
            while(0 == digitalRead(pinBtn));        // wait until button released

            return 1;
        }
    }

    return 0;
}

void setup(void) {
    u8x8.begin();
    u8x8.setFlipMode(1);   // set number from 1 to 3, the screen word will rotary 180

   // can.begin(7, 6, 9600);
    ss.begin(GPSBaud);
    pinMode(pinBtn, INPUT_PULLUP);
    pinMode(pinLed, OUTPUT);
    Serial.begin(9600);

}

void loop(void) {

    getButton();
    dispSpeed();
    dispRpm();
    dispGPS();

    makeValue();

    while (ss.available() > 0)
    if (gps.encode(ss.read()))
    {
        __latitude  = gps.location.lat();
        __longitude = gps.location.lng(); 
    }
}

void makeValue()
{
    static unsigned long timer_s = millis();
    if(millis()-timer_s < 100)return;
    timer_s = millis();

    int ret = 0;
    switch(disp_st)
    {
        case 1:     // speed

        speed = random(50, 100);
       /* if(!getSpeed(&speed))
        {
            speed = 0;
        }*/
        

        break;


        case 2:     // rpm

        rpm = random(1000, 5000);
        /*
        if(!getRPM(&rpm))
        {
            rpm = 0;
        }*/

        break;

        default:;
    }
}


void dispSpeed()
{
    if(disp_st != 1)return;

    static unsigned long timer_s = millis();
    if(millis()-timer_s < 1000)return;
    timer_s = millis();

    Serial.println("speed");

    u8x8.setFont(u8x8_font_chroma48medium8_r);
    u8x8.setCursor(0, 0);
    u8x8.print("VEHICLE SPEED   ");

    u8x8.setCursor(0, 3);
    u8x8.print(speed);
    u8x8.print(" km/h   ");
    u8x8.setCursor(0, 4);
    u8x8.print("            ");

}

void dispRpm()
{
    if(disp_st != 2)return;

    static unsigned long timer_s = millis();
    if(millis()-timer_s < 1000)return;
    timer_s = millis();

    Serial.println("rpm");

    u8x8.setFont(u8x8_font_chroma48medium8_r);
    u8x8.setCursor(0, 0);
    u8x8.print("ENGIN SPEED  ");

    u8x8.setCursor(0, 3);
    u8x8.print(rpm);
    u8x8.print(" rpm   ");
    u8x8.setCursor(0, 4);
    u8x8.print("            ");

}

void dispGPS()
{
    if(disp_st != 0)return;

    static unsigned long timer_s = millis();
    if(millis()-timer_s < 1000)return;
    timer_s = millis();

    Serial.println("gps");

    u8x8.setFont(u8x8_font_chroma48medium8_r);
    u8x8.setCursor(0, 0);
    u8x8.print("GPS LOCATION   ");
    u8x8.setCursor(0, 3);
    u8x8.print(__latitude, 6);
    u8x8.setCursor(0, 4);
    u8x8.print(__longitude, 6);  
}

// END FILE