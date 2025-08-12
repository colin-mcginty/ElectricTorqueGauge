#include <Arduino.h>
#include <stdint.h>


/// Pin Definitions
// order based on physical pin location on esp32
// left side of the board
const uint8_t not_used0 = 36; 
const uint8_t not_used1 = 39;
const uint8_t speed_pot_input = 34;
const uint8_t encoder_a_input = 35;
const uint8_t motor_controller_In2 = 32;
const uint8_t motor_controller_In1 = 33;
const uint8_t motor_controller_EN = 25;
const uint8_t LCD_E = 26;
const uint8_t LCD_RW = 27;
const uint8_t LCD_RS= 14;
const uint8_t transducer_input = 12;
const uint8_t rotations_pot_input = 13;
// right side of the board
const uint8_t direction_switch_cw_input = 23; 
const uint8_t direction_switch_ccw_input = 22;
const uint8_t LCD_K = 1;
const uint8_t encoder_b_input = 3;
const uint8_t LCD_A = 21;
const uint8_t LCD_DB7 = 19;
const uint8_t LCD_DB6 = 18;
const uint8_t LCD_DB5 = 5;
const uint8_t LCD_DB4 = 17;
const uint8_t LCD_DB3 = 16;
const uint8_t LCD_DB2 = 4;
const uint8_t LCD_DB1 = 2;
const uint8_t LCD_DB0 = 15;










// put function declarations here:
int myFunction(int, int);

void setup() {
  // put your setup code here, to run once:
  int result = myFunction(2, 3);
}

void loop() {
  // put your main code here, to run repeatedly:
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}