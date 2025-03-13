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
// 5 = 15 x R2 / (100k + R2)
// 5 x (100k + R2) = R2 x 15
// 100k + 5 x R2 = R2 x 15
// 100k = 10 x R2
// R2 = 10K ohms
*/

//Biblioteca para comunicacao serial com modulo bluetooth
#include <SoftwareSerial.h>

//Biblioteca para comunicacao I2C Display

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

/*
Wire.h > UNO R3, UNO R3 SMD, UNO Mini LE	A4(SDA), A5(SCL) I2C also available on the SDA / SCL pins (digital header).
Adafruit_GFX.h > Adafruit GFX Library
Adafruit_SSD1306.h > These displays use I2C or SPI to communicate, 2 to 5 pins are required to interface.
*/

#define MaxPWM  A0 // Arduino Analogic input pin A0; center pin of the potentiometer
#define V_batt A1 // Arduino Analogic input pin A1; tensao da bateria

#define RPWM_Output  5 // Arduino PWM output pin 5; connect to IBT-2 pin 1 (RPWM) (frente)
#define LPWM_Output  6 // Arduino PWM output pin 6; connect to IBT-2 pin 2 (LPWM) (traz)
#define PWM_EN 7 // Arduino PWM output pin 7; connect to IBT-2 pin 3 (R_EN) and 4 (L_EN)

#define SW1 10  // Arduino input_pullup digital 10; chave KCD4 frente
#define SW2 11  // Arduino input_pullup digital 11; chave KCD4 traz

#define BT_Rx   8 //serial RS232/485
#define BT_Tx   9 //serial RS232/485

#define OLED_RESET -1 // Define a configuracao do display

//Inicialização dos objetos
SoftwareSerial mySerialBT(BT_Rx, BT_Tx); //comunicacao // RX, TX
Adafruit_SSD1306 display(OLED_RESET); 

// prototipo de funcoes
void startMotor();  //rampa aceleracao
void stopMotor(); // rampa frenagem
void logotipo(); //logo inicializacao display
void showBattStatus(); //valor de tensao bateria

//variaveis globais
boolean motor_dirR=false, motor_dirL=false; //direcao do motor selecionada
int current_state1=HIGH, previus_state1=HIGH, current_state2=HIGH, previus_state2=HIGH; //detecao nivel chave de alto para baixo
unsigned int time_ms = 5000; //tempo em ms para rampa de frenagem e aceleracao
unsigned short pwm_max = 0xFF; //valor max do pwm definido pelo potenciometro
unsigned short pwm_value = 0x00; //variavel da rampa de aceleracao
float volts = 0.0; //tensao na bateria
unsigned long millisShowBatt = millis(); // faz a leitura da tensao da bateria

void setup()
{
  Serial.println("Iniciado");
  // BTS7960 board
  pinMode(RPWM_Output, OUTPUT);
  pinMode(LPWM_Output, OUTPUT);
  pinMode(PWM_EN, OUTPUT);

  // chave KCD4
  pinMode(SW1, INPUT_PULLUP);
  pinMode(SW2, INPUT_PULLUP);
  
  // Bluetooth
  Serial.begin(9600);
  mySerialBT.begin(9600); //taxa transmissao bluetooth
  
  // OLED diplay

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); //inicializa display no endereco 0x3C do I2C
  logotipo();
}

void loop()
{
  pwm_max = map(analogRead(MaxPWM), 0, 1023, 0, 0xFF); // maxima velocidade definida pelo potenciometro

  if(mySerialBT.available()) // comando bluetooth
  { 
    char driver_command = mySerialBT.read();  // leitura serial
    switch(driver_command)
    {
      case 'a':
        mySerialBT.println("acelerando direcao R"); 
        motor_dirR = true; //motor R selecionado (frente)
        motor_dirL = false;
        startMotor(); //rampa de aceleracao
        mySerialBT.println(pwm_max);
        showMotor();
        break;
      case 'c':
        mySerialBT.println("acelerando direcao L");
        motor_dirL = true; //motor L selecionado (traz)
        motor_dirR = false;
        startMotor();
        mySerialBT.println(pwm_max);
        showMotor();
        break;
      case 'b':
        mySerialBT.println("desacelerando");
        stopMotor();
        motor_dirR = false;  // desabilita selecao direcao
        motor_dirL = false;
        analogWrite(RPWM_Output, 0);
        analogWrite(LPWM_Output, 0);
        showMotor();
        break;
      default:
        Serial.println("Comando não reconhecido");
    }
  }

  current_state1 = digitalRead(SW1); //leitura do botao (frente)
  current_state2 = digitalRead(SW2); //leitura do botao (traz)

  if (current_state1 != previus_state1) // frente pressionando a chave KCD4
  {
    if (current_state1 == LOW) // nivel de sinal de alto para baixo
    {
      if (motor_dirR || motor_dirL) // se o motor em movimento > pare
      {
        Serial.println("Parar motor");
        stopMotor();
        motor_dirR = false;  // desabilita selecao direcao
        motor_dirL = false;
        analogWrite(RPWM_Output, 0);
        analogWrite(LPWM_Output, 0);
        showMotor();
      }
      else // se o motor esta parado > move para frente
      {
        Serial.println("Motor para frente");
        motor_dirR = true; //motor R selecionado (frente)
        motor_dirL = false;
        startMotor(); //rampa de aceleracao
        showMotor();
      }
    }
    previus_state1 = current_state1;
  }

  if (current_state2 != previus_state2) // traz pressionando a chave KCD4
  {
    if (current_state2 == LOW) // nivel de sinal de alto para baixo
    {
      if (motor_dirL || motor_dirR) // se o motor em movimento > pare
      {
        Serial.println("Parar motor");
        stopMotor();
        motor_dirR = false;  // desabilita selecao direcao
        motor_dirL = false;
        analogWrite(RPWM_Output, 0);
        analogWrite(LPWM_Output, 0);
        showMotor();
      }
      else // se o motor esta parado > move para traz
      {
        Serial.println("Motor para traz");
        motor_dirR = false; 
        motor_dirL = true; //motor L selecionado (traz)
        startMotor(); //rampa de aceleracao
        showMotor();
      }
    }
    previus_state2 = current_state2;
  }
  if((millis() - millisShowBatt) > 5000)
  {
    showBattVoltage();
    millisShowBatt = millis();
  }
}

// desenvolvimento da funcao
void startMotor()
{  
  digitalWrite(PWM_EN, HIGH); // habilita PWM no drive
  while(pwm_value <= (pwm_max-1))
  {
    if (motor_dirR) 
    {
      analogWrite(RPWM_Output, pwm_value);
      pwm_value+=1;
      if (pwm_value < pwm_max/2) pwm_value = short(pwm_max/2);
      delayMicroseconds(time_ms);
    }
    else
    {
      analogWrite(LPWM_Output, pwm_value);
      pwm_value+=1;
      if (pwm_value < pwm_max/2) pwm_value = short(pwm_max/2);
      delayMicroseconds(time_ms);
    }
  }
}

void stopMotor()
{
  while(pwm_value >= 0x01)
  {
    if (motor_dirR) 
    {
      analogWrite(RPWM_Output, pwm_value);
      pwm_value-=1;
      if (pwm_value < pwm_max/2) pwm_value = 0;
      delayMicroseconds(time_ms);
    }
    else
    {
      analogWrite(LPWM_Output, pwm_value);
      pwm_value-=1;
      if (pwm_value < pwm_max/2) pwm_value = 0;
      delayMicroseconds(time_ms);
    }
  }
  digitalWrite(PWM_EN, LOW); //desabilita o PWM no drive
}

void logotipo()
{
  // display.display(); // Show initial display buffer contents on the screen -- the library initializes this with an Adafruit splash screen.
  // delay(1000); // Pause for 2 seconds
  display.clearDisplay();  // Clear the buffer
  display.setTextColor(WHITE); //configura cor para branco para inicializar a tela
  display.setTextSize(1); //configura o tamanho da fonte
  display.setCursor(0,0);
  display.print("Maxximus - Macanhan");
  display.setCursor(0,10);
  display.print("Modulo versao Beta 0");
  display.setCursor(0,20);
  display.print("contato@gmail.com");
  display.display();
  delay(3000);
  showBattVoltage();
}

void showBattVoltage()
{
  float volts = map(analogRead(V_batt), 0, 1023, 0, 15);
  display.clearDisplay();  // Clear the buffer
  display.setTextColor(WHITE); //configura cor para branco para inicializar a tela
  display.setTextSize(1); //configura o tamanho da fonte
  display.setCursor(0,0);
  display.print("Maxximus - Bateria");
  display.setCursor(0,10);
  display.print("Tensao: ");
  display.print(volts);
  display.print(" V");
  display.setCursor(0,20);
  display.print("Autonomia: ");
  display.print((volts-10.5)*6);
  display.print(" h");
  display.display();
}

void showMotor()
{
  display.clearDisplay();  // Clear the buffer
  display.setTextColor(WHITE); //configura cor para branco para inicializar a tela
  display.setTextSize(1); //configura o tamanho da fonte
  display.setCursor(0,0);
  display.print("Maxximus - Motor");
  display.setCursor(0,10);
  display.print("pwm_max: ");
  display.println(pwm_max);
  if (motor_dirR) display.print("Acc R: ");
  if (motor_dirL) display.print("Acc L: ");
  if (!(motor_dirR || motor_dirL)) display.print("Desacc : ");
  display.print(pwm_value/pwm_max*100);
  display.print(" %");
  display.display();
}

  