# maxximus
Codigo fonte sistema de controle motor

/*==========================================================================
// Author : Robson Rodrigues <robsonavr@gmail.com>
// Project : Maxximus
// Description : Speed control, bluetooth connection, direction control, key direction control, oled diplay
// Source-Code : MaxximusControl.ino
// Program:  versao beta 0
//==========================================================================
// Connection to the BTS7960 board:
// BTS7960 Pin 1 (RPWM) to Arduino pin 5(PWM)
// BTS7960 Pin 2 (LPWM) to Arduino pin 6(PWM)
// BTS7960 Pin 3 (R_EN), 4 (L_EN) to Arduino 7
// BTS7960 Pin 7 (VCC) to Arduino 5V pin
// BTS7960 Pin 8 (GND) to Arduino GND
// BTS7960 Pin 5 (R_IS) and 6 (L_IS) not connected
//==========================================================================
// Connection to the OLED096 I2C SSD1306
// SSD1306 Pin 1 (GND) to Arduino pin GND
// SSD1306 Pin 2 (VCC) to Arduino pin 5V
// SSD1306 Pin 3 (SCK) to Arduino pin A5 I2C
// SSD1306 Pin 4 (SDA) to Arduino pin A4 I2C
//==========================================================================
// Connection to the Bluetooth HC06
// HC06 Pin 1 (Key) not connected
// HC06 Pin 2 (VCC) to Arduino pin 5V
// HC06 Pin 3 (GND) to Arduino pin GND
// HC06 Pin 4 (TXD) to Arduino pin 9 Serial RX
// HC06 Pin 5 (RXD) to Arduino pin 8 Serial TX
// HC06 Pin 6 (STATE) not connected
//==========================================================================
// Connection to the potenciometer 10kohms
// 10kohms pin 1 (VCC) to Arduino 5V
// 10kohms pin 2 (Signal) to Arduino A0 Analogic 1023
// 10kohms pin 3 (GND) to Arduino GND
//==========================================================================
// Connection to the chave KCD4
// KCD4 Pin 1 (Frente) to Arduino 10 INPUT_PULLUP
// KCD4 Pin 2 (GND) to Arduino GND Resistencia !Kohm
// KCD4 Pin 3 (Traz) to Arduino 11 INPUT_PULLUP
//==========================================================================
// Connection to multimeter divisor tensao
// vin -----
//          |
//         RRR R1
//          |
//          ---------Vout
//          |
//         RRR R2
//          |
//         --- GND
//          -
// 
// Vout = Vin x R2 / (R1 + R2)
// 5 = 15 x R2 / (1M + R2)
// 5 x (1M + R2) = R2 x 15
// 5M + 5 x R2 = R2 x 15
// 5M = 10 x R2
// R2 = 500K ohms
*/
