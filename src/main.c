/*
ATTinyTetris
Copyright (C) 2022

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

Author: Robert Alm, almrobert@gmail.com
*/

#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/io.h>

#include <MiniTinyI2C.h>

#define LCD_I2C_ADDR            0x3C
#define LCD_COMMAND             0x00
#define LCD_COMMAND_PAGE_ADDR   0x22
#define LCD_COMMAND_COLUMN_ADDR 0x21
#define LCD_DATA                0x40
#define LCD_ROW_PAGE_MAX           7

#define INIT_LENGTH 29
const uint8_t DisplayInit[INIT_LENGTH] = {
  0xE4,             // Soft Reset
  0xAE,             // Display OFF
  0xA8, 0x3F,       // set multiplex (HEIGHT-1): 0x3F for 128x64
  0xD3, 0x00,       // Display offset to 0
  0x40,             // Set display start line to 0
  0x8D, 0x14,       // Charge pump enabled
  0x20, 0x00,       // Memory addressing mode 0x00 Horizontal 0x01 Vertical
  0xDA, 0x12,       // Set COM Pins hardware configuration to sequential
  0x81, 0xA0,       // Set contrast
  0xD9, 0xFF,       // Set pre-charge period
  0xDB, 0x20,       // Set vcom detect
  LCD_COMMAND_PAGE_ADDR, 0x00, 0x07, // Page min to max
  LCD_COMMAND_COLUMN_ADDR, 0x00, 0x7F, // Column min to max
  0xC8,             //COM scan direction C0/C8
  0xA1,             //Column mapping A0/A1
  0xA4,             //RAM Content mode
  0xAF              // Display on
};

void initDisplay() {
    //Command Init
    startMiniTinyI2C(LCD_I2C_ADDR, false);
    writeMiniTinyI2C(LCD_COMMAND);
    for (uint8_t i = 0; i < INIT_LENGTH; i++) {
        writeMiniTinyI2C(DisplayInit[i]);
    }
    stopMiniTinyI2C();

    //Data 0 - Clear display
    startMiniTinyI2C(LCD_I2C_ADDR, false);
    writeMiniTinyI2C(LCD_DATA);
    for (uint16_t i = 0; i <= 255; i++) {
        writeMiniTinyI2C(0x00);
        writeMiniTinyI2C(0x00);
        writeMiniTinyI2C(0x00);
        writeMiniTinyI2C(0x00);
    }

    stopMiniTinyI2C();    
}

//Set Page and Column Display update constraints
void setDisplayArea(uint8_t column, uint8_t row, uint8_t lines) {
    startMiniTinyI2C(LCD_I2C_ADDR, false);
    writeMiniTinyI2C(LCD_COMMAND);
    writeMiniTinyI2C(LCD_COMMAND_PAGE_ADDR);
    writeMiniTinyI2C(LCD_ROW_PAGE_MAX - column);
    writeMiniTinyI2C(column);
    writeMiniTinyI2C(LCD_COMMAND_COLUMN_ADDR);
    writeMiniTinyI2C(row);
    writeMiniTinyI2C(row+lines);
    stopMiniTinyI2C();
}

void startDrawing(uint8_t column, uint8_t row, uint8_t lines) {
    setDisplayArea(column, row, lines);
    startMiniTinyI2C(LCD_I2C_ADDR, false);
    writeMiniTinyI2C(LCD_DATA);
}

void stopDrawing() {
    stopMiniTinyI2C();
}

void drawSegment(uint8_t column, uint8_t row, uint8_t lines, uint8_t* data) {
    startDrawing(column, row, lines);
    for (uint8_t i = 0; i < lines; i++) {
        writeMiniTinyI2C(data[i]);
    }
    stopDrawing();
}

#define BOARD_ROW_OFFSET    0
#define BOARD_LINES         21
#define BOARD_COLUMNS       12 /* Okay. Some explaining to do. The actual gamefield is 10 wide, but I also count the edges part of the board */

#define NEXT_TILE_COLUMN_OFFSET 6
#define NEXT_TILE_ROW_OFFSET    72

#define LINES_COLUMN_OFFSET 6
#define LINES_ROW_OFFSET    20

#define SCORE_COLUMN_OFFSET 1
#define SCORE_ROW_OFFSET    90

#define HISCORE_COLUMN_OFFSET 1
#define HISCORE_ROW_OFFSET    100

#define BUTTON_ADC_MARGIN    0x05
#define BUTTON_LEFT_VALUE_L  (0xD5 - BUTTON_ADC_MARGIN) 
#define BUTTON_LEFT_VALUE_H  (0xD5 + BUTTON_ADC_MARGIN) 
#define BUTTON_RIGHT_VALUE_L (0x57 - BUTTON_ADC_MARGIN)
#define BUTTON_RIGHT_VALUE_H (0x57 + BUTTON_ADC_MARGIN)
#define BUTTON_DOWN_VALUE_L  (0x7F - BUTTON_ADC_MARGIN)
#define BUTTON_DOWN_VALUE_H  (0x7F + BUTTON_ADC_MARGIN)
#define BUTTON_ROT_CCW_L     (0x7F - BUTTON_ADC_MARGIN)
#define BUTTON_ROT_CCW_H     (0x7F + BUTTON_ADC_MARGIN)
#define BUTTON_ROT_CW_L      (0xD5 - BUTTON_ADC_MARGIN)
#define BUTTON_ROT_CW_H      (0xD5 + BUTTON_ADC_MARGIN)
#define ADC_DIRECTIONAL_PIN ADC_MUXPOS_AIN6_gc 
#define ADC_ROTATIONAL_PIN  ADC_MUXPOS_AIN7_gc 
uint8_t gCurrentADCPin = ADC_DIRECTIONAL_PIN;

#define BUTTON_DIRECTIONAL_LEFT     1
#define BUTTON_DIRECTIONAL_RIGHT    2
#define BUTTON_DIRECTIONAL_DOWN     3

#define BUTTON_ROTATIONAL_CW        1
#define BUTTON_ROTATIONAL_CCW       2

#define BUTTON_UNPRESSED            0

#define SCORE_DOWN                  1
#define SCORE_NEW_TILE              5
#define SCORE_LINESCORE_BASE        4

uint16_t gGameBoard[BOARD_LINES] = {
    0x8010,
    0x8010,
    0x8010,
    0x8010,
    0x8010,
    0x8010,
    0x8010,
    0x8010,
    0x8010,
    0x8010,
    0x8010,
    0x8010,
    0x8010,
    0x8010,
    0x8010,
    0x8010,
    0x8010,
    0x8010,
    0x8010,
    0x8010,
    0xFFF0
};

#define START_POS_X 4
#define START_POS_Y 0

#define NUM_TILES 7
const uint8_t tiles[NUM_TILES] = {
    /* X X O O
       X X O O */
    0b11001100,

    /* O O O O
       X X X X */
    0b00001111,

    /* O X X O
       X X O O */
    0b01101100,

    /* X X O O
       O X X O */
    0b11000110,

    /* O O X O
       X X X O */
    0b00101110,

    /* X O O O
       X X X O */
    0b10001110,

    /* O X O O
       X X X O */
    0b01001110,
};

const uint8_t numbers[16][3] = {
    /* 0
        1110
        1010
        1010
        1010
        1110
        0000
    */
    { 0b11101010, 0b10101010, 0b11100000 },
    /* 1
        0100
        1100
        0100
        0100
        1110
        0000
    */
    { 0b01001100, 0b01000100, 0b11100000 },
    /* 2
        1110
        0010
        0100
        1000
        1110
        0000
    */
    { 0b11100010, 0b01001000, 0b11100000 },
    /* 3
        1110
        0010
        0110
        0010
        1110
        0000
    */
    { 0b11100010, 0b01100010, 0b11100000 },
    /* 4
        1010
        1010
        1110
        0010
        0010
        0000
    */
    { 0b10101010, 0b11100010, 0b00100000 },
    /* 5
        1110
        1000
        1110
        0010
        1110
        0000
    */
    { 0b11101000, 0b11100010, 0b11100000 },
    /* 6
        1110
        1000
        1110
        1010
        1110
        0000
    */
    { 0b11101000, 0b11101010, 0b11100000 },
    /* 7
        1110
        0010
        0100
        0100
        0100
        0000
    */
    { 0b11100010, 0b01000100, 0b01000000 },
    /* 8
        1110
        1010
        1110
        1010
        1110
        0000
    */
    { 0b11101010, 0b11101010, 0b11100000 },
    /* 9
        1110
        1010
        1110
        0010
        1110
        0000
    */
    { 0b11101010, 0b11100010, 0b11100000 },
    /* A
        1110
        1010
        1110
        1010
        1010
        0000
    */
    { 0b11101010, 0b11101010, 0b10100000 },
    /* B
        1110
        1010
        1100
        1010
        1110
        0000
    */
    { 0b11101010, 0b11001010, 0b11100000 },
    /* C
        1110
        1000
        1000
        1000
        1110
        0000
    */
    { 0b11101000, 0b10001000, 0b11100000 },
    /* D
        1100
        1010
        1010
        1010
        1100
        0000
    */
    { 0b11001010, 0b10101010, 0b11000000 },
    /* E
        1110
        1000
        1100
        1000
        1110
        0000
    */
    { 0b11101000, 0b11001000, 0b11100000 },
    /* F
        1110
        1000
        1100
        1000
        1000
        0000
    */
    { 0b11101000, 0b11001000, 0b10000000 }
};

/* Current tile. Top left is position.
   O O O O  0xF000
   O O O O  0x0F00
   O O O O  0x00F0
   O O O O  0x000F
*/
uint16_t gCurrentTile = 0;
uint8_t gCurrentTileIndex = 0;
uint8_t gNextTile = 1;
int8_t gCurrentPos[2] = {START_POS_X, START_POS_Y};
uint16_t gLines = 0;
uint32_t gHiScore = 0;
uint32_t gScore = 0;
uint8_t gDirectionalButton = BUTTON_UNPRESSED;
uint8_t gRotationalButton = BUTTON_UNPRESSED;

static bool checkPosition(const uint8_t x, const uint8_t y) {
    return (gGameBoard[y] & (0x8000 >> x)); 
}

void refreshBoard(bool partial) {
    uint8_t yStart = 0;
    uint8_t yEnd = BOARD_LINES;

    if (partial) {
        if(gCurrentPos[1] > 0) {
            yStart = gCurrentPos[1]-1;
            yEnd = yStart + 5; //one line before + tile height (4x4 tile)
            if(yEnd > BOARD_LINES) yEnd = BOARD_LINES;
        }
    }

    for( uint8_t y = yStart; y < yEnd; y++ ) {
        for ( uint8_t x = 0; x < BOARD_COLUMNS; x += 2 ) {
            uint8_t out[4] = {0, 0, 0, 0};
            bool left      = checkPosition(x, y);
            bool right     = checkPosition(x+1, y);

            out[0] = (left?0xE0:0) | (right?0x0E:0);
            out[1] = out[0];
            out[2] = out[0];
            out[3] = 0;

            drawSegment(x/2, BOARD_ROW_OFFSET + y*4, 4, out);
        }
    }
    if (!partial) {
        drawLines();
        drawScore();
    }

}

bool checkAllowedPositionOrDraw(bool draw, bool erase) {
    uint16_t tileRow = 0;
    for(uint8_t i = 0; i < 4; i++) {
        uint8_t shift = i << 2;
        tileRow = (((gCurrentTile & (0xF000 >> shift)) << shift) >> gCurrentPos[0]);
        if(draw) {
            if (erase) 
                gGameBoard[gCurrentPos[1] + i] &= ~tileRow;
            else
                gGameBoard[gCurrentPos[1] + i] |= tileRow;
        } else {
            if(gGameBoard[gCurrentPos[1] + i] & tileRow) return false;
        }
    }
    return true;
}

bool updateTilePosition(int deltaX, int deltaY, bool rotate) {
    bool ret = true;
    uint16_t backupTile = gCurrentTile;

    //Erase old position
    checkAllowedPositionOrDraw(true, true);

    //Update current position global
    gCurrentPos[0] += deltaX;
    gCurrentPos[1] += deltaY;

    //Rotate Tile?
    if(rotate && (gCurrentTileIndex != 0)) {
        uint16_t newTile = 0;

        //Ugly! But saves flash! Static rotations!
        if (gCurrentTile == 0b0000111100000000) newTile = 0b0100010001000100;
        if (gCurrentTile == 0b0100010001000100) newTile = 0b0000111100000000;

        if (gCurrentTile == 0b0110110000000000) newTile = 0b1000110001000000;
        if (gCurrentTile == 0b1000110001000000) newTile = 0b0110110000000000;
        
        if (gCurrentTile == 0b1100011000000000) newTile = 0b0100110010000000;
        if (gCurrentTile == 0b0100110010000000) newTile = 0b1100011000000000;

        if (gCurrentTile == 0b0010111000000000) newTile = 0b0100010001100000;
        if (gCurrentTile == 0b0100010001100000) newTile = 0b0000111010000000;
        if (gCurrentTile == 0b0000111010000000) newTile = 0b1100010001000000;
        if (gCurrentTile == 0b1100010001000000) newTile = 0b0010111000000000;

        if (gCurrentTile == 0b1000111000000000) newTile = 0b0110010001000000;
        if (gCurrentTile == 0b0110010001000000) newTile = 0b0000111000100000;
        if (gCurrentTile == 0b0000111000100000) newTile = 0b0100010011000000;
        if (gCurrentTile == 0b0100010011000000) newTile = 0b1000111000000000;

        if (gCurrentTile == 0b0100111000000000) newTile = 0b0100011001000000;
        if (gCurrentTile == 0b0100011001000000) newTile = 0b0000111001000000;
        if (gCurrentTile == 0b0000111001000000) newTile = 0b0100110001000000;
        if (gCurrentTile == 0b0100110001000000) newTile = 0b0100111000000000;

        gCurrentTile = newTile;
    }

    if(!checkAllowedPositionOrDraw(false, false)) {
        gCurrentPos[0] -= deltaX;
        gCurrentPos[1] -= deltaY;
        gCurrentTile = backupTile;
        ret = false;
    }

    //Draw new position
    checkAllowedPositionOrDraw(true, false);

    return ret;
}

void drawNextTile() {
    uint8_t out[7];

    // 0bxx000000
    out[0] = ((tiles[gNextTile]&0b10000000)?0xE0:0) | ((tiles[gNextTile]&0b01000000)?0x0E:0);
    out[1] = out[0];
    out[2] = out[0];    
    out[3] = 0x00;
    // 0b0000xx00
    out[4] = ((tiles[gNextTile]&0b00001000)?0xE0:0) | ((tiles[gNextTile]&0b00000100)?0x0E:0);
    out[5] = out[4];
    out[6] = out[4];

    drawSegment(NEXT_TILE_COLUMN_OFFSET, NEXT_TILE_ROW_OFFSET, 7, out);

    // 0b00xx0000
    out[0] = ((tiles[gNextTile]&0b00100000)?0xE0:0) | ((tiles[gNextTile]&0b00010000)?0x0E:0);
    out[1] = out[0];
    out[2] = out[0];    
    out[3] = 0x00;
    // 0b000000xx
    out[4] = ((tiles[gNextTile]&0b00000010)?0xE0:0) | ((tiles[gNextTile]&0b00000001)?0x0E:0);
    out[5] = out[4];
    out[6] = out[4];

    drawSegment(NEXT_TILE_COLUMN_OFFSET+1, NEXT_TILE_ROW_OFFSET, 7, out);
}

void generateNextTile() {
    gCurrentTileIndex = gNextTile;
    gNextTile++;
    gNextTile %= NUM_TILES;
    drawNextTile();
}

bool insertNextTile() {
    bool ret = true;
    gCurrentTile = tiles[gNextTile] << 8;
    generateNextTile();
    gCurrentPos[0] = START_POS_X;
    gCurrentPos[1] = START_POS_Y;
    ret = updateTilePosition(0, 0, false);
    refreshBoard(false);
    return ret;
}

void drawTwoDigits(int column, int row, uint8_t value) {
    uint8_t out[6];
    uint8_t left = (value & 0xF0) >> 4;
    uint8_t right = value & 0x0F;

    out[0] = (numbers[left][0] & 0xF0) | ((numbers[right][0] & 0xF0) >> 4); 
    out[1] = ((numbers[left][0] & 0x0F) << 4) | (numbers[right][0] & 0x0F); 
    out[2] = (numbers[left][1] & 0xF0) | ((numbers[right][1] & 0xF0) >> 4); 
    out[3] = ((numbers[left][1] & 0x0F) << 4) | (numbers[right][1] & 0x0F); 
    out[4] = (numbers[left][2] & 0xF0) | ((numbers[right][2] & 0xF0) >> 4); 
    out[5] = 0; //((numbers[left][2] & 0x0F) << 4) | (numbers[right][2] & 0x0F); 

    drawSegment(column, row, 6, out);
}

void drawLines() {
    drawTwoDigits(LINES_COLUMN_OFFSET, LINES_ROW_OFFSET, gLines >> 8);
    drawTwoDigits(LINES_COLUMN_OFFSET+1, LINES_ROW_OFFSET, gLines & 0x00FF);
}

void drawScore() {
    drawTwoDigits(SCORE_COLUMN_OFFSET, SCORE_ROW_OFFSET, gScore >> 24);
    drawTwoDigits(SCORE_COLUMN_OFFSET+1, SCORE_ROW_OFFSET, (gScore & 0x00FF0000) >> 16);
    drawTwoDigits(SCORE_COLUMN_OFFSET+2, SCORE_ROW_OFFSET, (gScore & 0x0000FF00) >> 8);
    drawTwoDigits(SCORE_COLUMN_OFFSET+3, SCORE_ROW_OFFSET, gScore & 0x000000FF);
}

void drawHiScore() {
    drawTwoDigits(HISCORE_COLUMN_OFFSET, HISCORE_ROW_OFFSET, gHiScore >> 24);
    drawTwoDigits(HISCORE_COLUMN_OFFSET+1, HISCORE_ROW_OFFSET, (gHiScore & 0x00FF0000) >> 16);
    drawTwoDigits(HISCORE_COLUMN_OFFSET+2, HISCORE_ROW_OFFSET, (gHiScore & 0x0000FF00) >> 8);
    drawTwoDigits(HISCORE_COLUMN_OFFSET+3, HISCORE_ROW_OFFSET, gHiScore & 0x000000FF);
}

void initController() {
    //Set ADC to VDD reference voltage and prescaler to 256 divisor (20/256 = 78kHz)
    ADC0.CTRLC = ADC_SAMPCAP_bm | ADC_REFSEL_VDDREF_gc | ADC_PRESC_DIV256_gc;

    //Configure ADC on pin 2 (AIN6) - direction and drop
    //Turn off Digital input buffer
    PORTA.PIN2CTRL = PORT_ISC_INPUT_DISABLE_gc;

    //Configure ADC on pin 3 (AIN7) - rotations
    //Turn off Digital input buffer
    PORTA.PIN3CTRL = PORT_ISC_INPUT_DISABLE_gc;

    //Enable global interrupts
    sei();

    //Clear conversion interrupt (Write 1 to clear!)
    ADC0.INTFLAGS = ADC_RESRDY_bm;

    //Enable conversion interrupt
    ADC0.INTCTRL = ADC_RESRDY_bm;

    //8bit ADC and enable
    ADC0.CTRLA = ADC_RESSEL_8BIT_gc | ADC_ENABLE_bm;

    //Default to AIN6 pin
    ADC0.MUXPOS = gCurrentADCPin;

    //Start ADC loop!
    ADC0.COMMAND |= ADC_STCONV_bm;

}

ISR(ADC0_RESRDY_vect) {
    //Check if it was direction or rotation pin
    //Conversion result is in ADC0.RESL
    uint8_t result = ADC0.RESL;

    if (gCurrentADCPin == ADC_DIRECTIONAL_PIN) {
        //Direction (AIN6):
        //"Left"  (0xD5)
        //"Right" (0x57)
        //"Down"  (0x7F)
        if (result > BUTTON_LEFT_VALUE_L && result < BUTTON_LEFT_VALUE_H) {
            gDirectionalButton = BUTTON_DIRECTIONAL_LEFT;
        } else if (result > BUTTON_RIGHT_VALUE_L && result < BUTTON_RIGHT_VALUE_H) {
            gDirectionalButton = BUTTON_DIRECTIONAL_RIGHT;
        } else if (result > BUTTON_DOWN_VALUE_L && result < BUTTON_DOWN_VALUE_H) {
            gDirectionalButton = BUTTON_DIRECTIONAL_DOWN;
        } else {
            gDirectionalButton = BUTTON_UNPRESSED;
        }
        gCurrentADCPin = ADC_ROTATIONAL_PIN;
    } else { //ADC_ROTATIONAL_PIN
        //Rotation (AIN7):
        //"Rotation Left (CCW)" (0x7F)
        //"Rotation Right (CW)" (0xD5)
        if (result > BUTTON_ROT_CCW_L && result < BUTTON_ROT_CCW_H) {
            gRotationalButton = BUTTON_ROTATIONAL_CCW;
        } else if (result > BUTTON_ROT_CW_L && result < BUTTON_ROT_CW_H) {
            gRotationalButton = BUTTON_ROTATIONAL_CW;
        } else {
            gRotationalButton = BUTTON_UNPRESSED;
        }

        gCurrentADCPin = ADC_DIRECTIONAL_PIN;
    }

    //Clear conversion interrupt (Write 1 to clear!)
    ADC0.INTFLAGS = ADC_RESRDY_bm;

    //Restart conversation again!
    ADC0.MUXPOS = gCurrentADCPin;

    //Start ADC loop!
    ADC0.COMMAND |= ADC_STCONV_bm;
}

void checkButtonsAndDelay() {
    for (uint8_t i = 0; i < 2; i++) {
        switch (gDirectionalButton) {
            case BUTTON_DIRECTIONAL_LEFT:
                updateTilePosition(-1, 0, false);
                break;
            case BUTTON_DIRECTIONAL_RIGHT:
                updateTilePosition(1, 0, false);
                break;
            case BUTTON_DIRECTIONAL_DOWN:
                updateTilePosition(0, 1, false);
                gScore += SCORE_DOWN;
                break;
        }
        switch (gRotationalButton) {
            case BUTTON_ROTATIONAL_CCW:
            case BUTTON_ROTATIONAL_CW:
                updateTilePosition(0, 0, true);
                break;
        }
        refreshBoard(true);
        _delay_ms(25);
    }
}

void checkAndRemoveCompletedLines() {
    uint8_t completedLines = 0;
    for (int8_t i = 3; i >= 0; i--) {
        uint8_t yPos = gCurrentPos[1] + i;
        yPos= (yPos >= BOARD_LINES-1)?BOARD_LINES-2:yPos;
        if (gGameBoard[yPos] == 0xFFF0) {
            completedLines++;
            while (yPos > 0) {
                gGameBoard[yPos] = gGameBoard[yPos-1];
                yPos--;
            }
            i++;
        }
    }
    gLines += completedLines;
    gScore += (SCORE_LINESCORE_BASE << completedLines);
}

void gameOver() {
    for (int8_t i = BOARD_LINES-2; i >= 0; i--) {
        gGameBoard[i] = 0xFFF0;
        refreshBoard(false);
    }
}

//Notes
// 20000000 / (2 * 4 * Hz) = PERBUF --> CMP0BUF == (PERBUF >> 1)
// E4 = 330Hz == 1263
// G4 = 392Hz == 1063
// A4 = 440Hz == 947
// B4 = 494Hz == 843
// C5 = 523Hz == 797
// D5 = 587Hz == 710
// E5 = 659Hz == 632
// F5 = 698Hz == 597
// G5 = 784Hz == 531
// A5 = 880Hz == 473
// E4... G4. B4.. G4. E4. A4... C5. E5.. D5. C5. B4... C5. D5.. E5.. C5.. A4.. A4...
// 256ms / pulse
#define NOTE_COUNT  19
const uint16_t notes[NOTE_COUNT * 2] = {
    1263, 24,
    1063, 8,
    843, 16,
    1063, 8,
    1263, 8,
    947, 24,
    797, 8,
    632, 16,
    710, 8,
    797, 8,
    843, 24,
    797, 8,
    710, 16,
    632, 16,
    797, 16,
    947, 16,
    0, 0,
    947, 24,
    0, 32
};
uint16_t gNotePos = 0;
uint8_t gNoteRepeat = 0;

void initAudio() {
    //PWM peripheral
    PORTMUX.CTRLA = PORTMUX_TCA00_DEFAULT_gc;
    TCA0.SINGLE.CTRLB = TCA_SINGLE_CMP0EN_bm | TCA_SINGLE_WGMODE_DSBOTTOM_gc;
    TCA0.SINGLE.EVCTRL &= ~(TCA_SINGLE_CNTEI_bm);
    TCA0.SINGLE.PERBUF = 0;
    TCA0.SINGLE.CMP0BUF = 0;
    TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV4_gc | TCA_SINGLE_ENABLE_bm;
    PORTA.DIR |= PIN3_bm;

    //RTC Timer @256ms interrupt (8k/32k = 1/4 sec)
    RTC.CLKSEL = RTC_CLKSEL_INT32K_gc;
    RTC.PITINTCTRL = RTC_PI_bm;
    RTC.PITCTRLA = RTC_PERIOD_CYC1024_gc | RTC_PITEN_bm;
}

ISR(RTC_PIT_vect) {
    RTC.PITINTFLAGS = RTC_PI_bm;
    if(gNoteRepeat > 0) {
        gNoteRepeat--;
        return;
    }
    TCA0.SINGLE.PERBUF = notes[gNotePos];
    TCA0.SINGLE.CMP0BUF = notes[gNotePos] >> 1;
    gNoteRepeat = notes[gNotePos + 1];
    gNotePos += 2;
    if(gNotePos == (NOTE_COUNT * 2)) gNotePos = 0;
}

int main() {
    initMiniTinyI2C(1100);
    initDisplay();
    initController();
    initAudio();
    
    drawHiScore();

    insertNextTile();
    while(1) {
        if(!updateTilePosition(0, 1, false)) {
            if(gCurrentPos[1] == 0) {
                gameOver();
                break;
            }
            checkAndRemoveCompletedLines();
            insertNextTile();            
        }
        refreshBoard(true);
        checkButtonsAndDelay();
    }
    while(1) {};
}
