#include <avr/io.h>
#include "MiniTinyI2C.h"

void initMiniTinyI2C(const uint16_t baud) {
    //TWI0.MBAUD - Set baud rate
    // Set MBAUD to get 100KHz for SCL from fSCL = CLK_PER / (10 + 2*BAUD + CLK_PER*Trise)
    // uint32_t baud = ((F_CPU / 1000 / 100) - (((F_CPU * 1000) / 1000) / 1000) / 1000 - 10) / 2;
    // 100kHz - Trise = 1000ns + 20MHz -> 0x55
    // 400kHz - Trise = 300ns + 20MHz -> 0x11
    // 1MHz - Trise   = 120ns + 20MHz-> 0x04 (980kHz) 0x03 (1087kHz)
    switch (baud) {
        case 100:
            TWI0.MBAUD = 0x55; //100kHz 
            break;
        case 400:
            TWI0.MBAUD = 0x11; //400kHz 
            break;
        case 800:
            TWI0.MBAUD = 0x05; //800kHz 
            break;
        case 1100:
            TWI0.CTRLA = TWI_FMPEN_bm;
            TWI0.MBAUD = 0x03; //1100kHz 
            break;
    
        default:
            TWI0.MBAUD = 0x55; //100kHz 
            break;
    }

    //TWI0.MSTATUS - Ox1 BUSSTATE - Set to Idle explicitly for I2C
    TWI0.MSTATUS = TWI_BUSSTATE_IDLE_gc; //Idle

    //TWI0.MCTRLA - Enable
    TWI0.MCTRLA = TWI_ENABLE_bm;
}

uint8_t readMiniTinyI2C(bool stop) {
    while (!(TWI0.MSTATUS & TWI_RIF_bm));                       // Wait for read interrupt flag

    uint8_t data = TWI0.MDATA;

    if (!stop)
        TWI0.MCTRLB = TWI_MCMD_RECVTRANS_gc;                    // ACK = more bytes to read
    else
        TWI0.MCTRLB = TWI_ACKACT_bm | TWI_MCMD_RECVTRANS_gc;    // Send NAK

  return data;
}

bool writeMiniTinyI2C(uint8_t data) {
  
    TWI0.MDATA = data;
    TWI0.MCTRLB = TWI_MCMD_RECVTRANS_gc;                        // Do nothing

    while (!(TWI0.MSTATUS & TWI_WIF_bm));                       // Wait for write interrupt flag

    return !(TWI0.MSTATUS & TWI_RXACK_bm);                      // Returns true if slave gave an ACK
}

bool startMiniTinyI2C(uint8_t address, bool read) {
    TWI0.MADDR = address << 1 | read;                           // Send START condition

    while (!(TWI0.MSTATUS & (TWI_WIF_bm | TWI_RIF_bm)));        // Wait for write or read interrupt flag
    
    if ((TWI0.MSTATUS & TWI_ARBLOST_bm))
        return false;                                           // Return false if arbitration lost or bus error
  return !(TWI0.MSTATUS & TWI_RXACK_bm);                        // Return true if slave gave an ACK
}

void stopMiniTinyI2C() {
  TWI0.MCTRLB = TWI_ACKACT_bm | TWI_MCMD_STOP_gc;               // Send STOP
}
