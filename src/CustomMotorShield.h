#ifndef CustomMotorShield_h
#define CustomMotorShield_h

#include "Arduino.h"
#include "Wire.h"

#if !defined( __AVR_ATmega328P__ ) && !defined( __AVR_ATmega2560__ )
#warning "CustomMotorShield is only tested for ATmega328P and ATmega2560"
#endif

#define FORWARD 1
#define BACKWARD 2
#define BRAKE 3
#define RELEASE 4

class MotorShield {
    public:
        MotorShield( uint8_t devAddress = 0x60 );
        void begin();
        void writePWM( uint8_t pin , uint16_t startTime , uint16_t stopTime );
        void writeAnalog( uint8_t pin , uint16_t pulseWidth , uint16_t startTime=0 );
        void writeDigital( uint8_t pin , uint8_t val );
        void setMotorPWM( uint8_t motorNumber , uint16_t val );
        void setMotorPercent( uint8_t motorNumber , float percent );
        void setMotorDirection( uint8_t motorNumber , uint8_t direction );
        void writeServo( uint8_t pin , float percent );
        void releaseServo( uint8_t pin );
    
    private:
        uint8_t devAddress;
        void send( uint8_t regAddress , uint8_t data8 );
        void send16( uint8_t regAddress , uint16_t data16 , bool msByteFirst=false );
        void send( uint8_t regAddress , uint8_t *data8 , size_t quantity );
        void send16( uint8_t regAddress , uint16_t *data16 , size_t quantity , bool msByteFirst=false );
    
};

#endif
