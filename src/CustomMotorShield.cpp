#include "CustomMotorShield.h"

MotorShield::MotorShield( uint8_t devAddress = 0x60 ):
    devAddress( devAddress )
{}

void MotorShield::begin() {
    Wire.begin();
    
    send( 0x00 , 0x10 );
    // regAddress: 0x00 = Mode1 register
    // data:
    //   bit 4 = 1 : Enter sleep mode
    
    send( 0xFE , 107 );
    // regAddress: 0xFE = Prescaler register
    // data:
    //   161 = round( 25MHz / 4096 / 40Hz / 0.944 - 1 )
    //     25MHz = Shield internal clock
    //     4096 = Shield counter resolution
    //     40Hz = Servo refresh rate
    //     0.944 = Correction factor
    
    send( 0x00 , 0x20 );
    // regAddress: 0x00 = Mode1 register
    // data:
    //   bit 4 = 0 : Exit sleep mode
    //   bit 5 = 1 : Auto-increment register address
    
    delay( 1 );
    // Wait at least 500us for the oscillator to stabilize after exiting sleep mode
}

void MotorShield::writePWM( uint8_t pin , uint16_t pulseWidth , uint16_t startTime=0 ) {
    if ( pulseWidth > 0x1000 ) pulseWidth = 0x1000;
    if ( startTime > 0x1000 ) startTime = 0x1000;
    
    uint16_t toSend[] = { startTime , startTime+pulseWidth };
    
    send16( 4*pin+0x06 , toSend , 2 );
    // regAddress: pin addresses are in groups of
    //   n_ON_L , n_ON_H , n_OFF_L , n_OFF_H starting at 0x06 for n=0
    // data:
    //   [n_ON_H , n_ON_L] is a 12bit register storing the time
    //     in the 0x0FFF (12 bit) counting cycle at which pin n turns on.
    //   If the 12th bit ( n_ON_H[4] ) is set, the pin will be always on.
    //   [n_OFF_H , n_OF_L] is a 12bit register storing the time
    //     in the 0x0FFF (12 bit) counting cycle at which pin n turns off.
    //   If the 12th bit ( n_OFF_H[4] ) is set, the pin will be always off.
    //   Always off takes precedence over always on.
}

void MotorShield::writeDigital( uint8_t pin , uint8_t val ) {
    switch ( val ) {
        case HIGH: writePWM( pin , 0x1000 , 0 ); break;
        case LOW:  writePWM( pin , 0 , 0x1000 ); break;
    }
}

void MotorShield::setMotorPWM( uint8_t motorNumber , uint16_t val ) {
    switch ( motorNumber ) {
        case 1: writePWM(  8 , val ); break;
        case 2: writePWM( 13 , val ); break;
        case 3: writePWM(  2 , val ); break;
        case 4: writePWM(  7 , val ); break;
    }
}

void MotorShield::setMotorDirection( uint8_t motorNumber , uint8_t direction ) {
    uint8_t pin1;
    uint8_t pin2;
    switch ( motorNumber ) {
        case 1: pin1 = 10; pin2 =  9; break;
        case 2: pin1 = 11; pin2 = 12; break;
        case 3: pin1 =  4; pin2 =  3; break;
        case 4: pin1 =  5; pin2 =  6; break;
        default: return;
    }
    switch ( direction ) {
        case FORWARD:  writeDigital( pin2 , LOW ); writeDigital( pin1 , HIGH ); break;
        case BACKWARD: writeDigital( pin1 , LOW ); writeDigital( pin2 , HIGH ); break;
        case RELEASE:  writeDigital( pin1 , LOW ); writeDigital( pin2 , LOW  ); break;
    }
}

void MotorShield::writeServo( uint8_t pin , float percent ) {
    // if ( percent > 100 ) percent = 100;
    // if ( percent <   0 ) percent = 0;
    
    writePWM( pin , 123 + percent/100 * ( 613 - 123 ) );
    // Min servo pulse:  82 = round(  500us / 25066us * 4096 ) - 1
    // Max servo pulse: 409 = round( 2500us / 25066us * 4096 ) - 1
    //     500us = servo min pulse time
    //    2500us = servo max pulse time
    //   25066us = servo refresh period (from 40Hz refresh rate)
    //   4096 = Shield counter resolution
}

void MotorShield::releaseServo( uint8_t pin ) {
    writeDigital( pin , LOW );
}

void MotorShield::send( uint8_t regAddress , uint8_t data8 ) {
    Wire.beginTransmission( devAddress );
    Wire.write( regAddress );
    Wire.write( data8 );
    Wire.endTransmission( true );
}

void MotorShield::send16( uint8_t regAddress , uint16_t data16 , bool msByteFirst=false ) {
    Wire.beginTransmission( devAddress );
    Wire.write( regAddress );
    if ( msByteFirst ) {
        Wire.write( (uint8_t)(data16 >> 8) );
        Wire.write( (uint8_t)(data16 | 0x0F) );
    } else {
        Wire.write( (uint8_t)(data16 | 0x0F) );
        Wire.write( (uint8_t)(data16 >> 8) );
    }
    Wire.endTransmission( true );
}

void MotorShield::send( uint8_t regAddress , uint8_t *data8 , size_t quantity ) {
    Wire.beginTransmission( devAddress );
    Wire.write( regAddress );
    for ( size_t i=0 ; i<quantity ; ++i ) {
        Wire.write( data8[i] );
    }
    Wire.endTransmission();
}

void MotorShield::send16( uint8_t regAddress , uint16_t *data16 , size_t quantity , bool msByteFirst=false ) {
    Wire.beginTransmission( devAddress );
    Wire.write( regAddress );
    for ( size_t i=0 ; i<quantity ; ++i ) {
        if ( msByteFirst ) {
            Wire.write( (uint8_t)(data16[i] >> 8) );
            Wire.write( (uint8_t)(data16[i] | 0x0F) );
        } else {
            Wire.write( (uint8_t)(data16[i] | 0x0F) );
            Wire.write( (uint8_t)(data16[i] >> 8) );
        }
    }
    Wire.endTransmission();
}
