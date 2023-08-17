#include <Arduino.h>
#include "PinDefinitionsAndMore.h" // Define macros for input and output pin etc.
#include "teste.h"

#if !defined(RAW_BUFFER_LENGTH)
#  if RAMEND <= 0x4FF || RAMSIZE < 0x4FF
#define RAW_BUFFER_LENGTH  180  // 750 (600 if we have only 2k RAM) is the value for air condition remotes. Default is 112 if DECODE_MAGIQUEST is enabled, otherwise 100.
#  elif RAMEND <= 0x8FF || RAMSIZE < 0x8FF
#define RAW_BUFFER_LENGTH  600  // 750 (600 if we have only 2k RAM) is the value for air condition remotes. Default is 112 if DECODE_MAGIQUEST is enabled, otherwise 100.
#  else
#define RAW_BUFFER_LENGTH  750  // 750 (600 if we have only 2k RAM) is the value for air condition remotes. Default is 112 if DECODE_MAGIQUEST is enabled, otherwise 100.
#  endif
#endif
#define MARK_EXCESS_MICROS    20    // Adapt it to your IR receiver module. 20 is recommended for the cheap VS1838 modules.
#include <IRremote.hpp>

//Definição dos pinos de controle do motor
#define M1 6 // Pino_Velocidade 1º Motor ( 0 a 255)_ Porta IN2 ponte H;
#define M2 5 //Pino_Velocidade 2º Motor ( 0 a 255) _ Porta IN4 ponte H;
#define dir1 7 //Pino_Direção do 1º Motor: Para frente / Para trás (HIGH ou LOW)_ porta IN1 ponte H;
#define dir2 4 //Pino_Direção do 2º Motor: Para frente / Para trás (HIGH ou LOW)_ porta IN3 ponte H;

//Definição dos pinos dos sensores
#define pin_S1 9
#define pin_S2 8
bool Sensor1 = 0;
bool Sensor2 = 0;
#define buzzerPin 12

//variável responsável por controlar a velocidade dos motores
int velocidade = 255;

// Parâmetros do PID
double Kp = 1.6;   // Ganho Proporcional
double Ki = 0.15;   // Ganho Integral
double Kd = 0.012;  // Ganho Derivativo

// Variáveis globais
int estado = 0;
double erro = 0;
double erroAnterior = 0;
double somaErro = 0;
double erro2 = 0;
double erroAnterior2 = 0;
double somaErro2 = 0;




//+=============================================================================
// Configure the Arduino
//
void setup() {
pinMode(LED_BUILTIN, OUTPUT);
pinMode(M1, OUTPUT);
pinMode(M2, OUTPUT);
pinMode(dir1, OUTPUT);
pinMode(dir2, OUTPUT);
pinMode(buzzerPin, OUTPUT);

//Setamos os pinos dos sensores como entrada
pinMode(pin_S1, INPUT);
pinMode(pin_S2, INPUT);


    Serial.begin(115200);   // Status message will be sent to PC at 9600 baud
#if defined(__AVR_ATmega32U4__) || defined(SERIAL_PORT_USBVIRTUAL) || defined(SERIAL_USB) /*stm32duino*/|| defined(USBCON) /*STM32_stm32*/|| defined(SERIALUSB_PID) || defined(ARDUINO_attiny3217)
    delay(4000); // To be able to connect Serial monitor after reset or power up and before first print out. Do not wait for an attached Serial Monitor!
#endif
    // Start the receiver and if not 3. parameter specified, take LED_BUILTIN pin from the internal boards definition as default feedback LED
    IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);


}

//+=============================================================================
// The repeating section of the code
//
void loop() {      


  if (IrReceiver.decode()) {  // Grab an IR code            
    if (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_AUTO_REPEAT) {
      IrReceiver.resume();
      return;         
    }

    uint32_t rawData = IrReceiver.decodedIRData.decodedRawData; // Obter o valor do sinal IR recebido

    switch (rawData) {
      case frente: 
        digitalWrite(dir1, LOW);
        digitalWrite(dir2, HIGH);
        analogWrite(M1, velocidade); // Ambos motores ligam na mesma velocidade
        analogWrite(M2, velocidade);
        break;

      case tras: 
        digitalWrite(dir1, HIGH);
        digitalWrite(dir2, LOW);
        analogWrite(M1, velocidade); // Ambos motores ligam na mesma velocidade
        analogWrite(M2, velocidade);
        break; 

      case direita: 
        digitalWrite(dir1, LOW);
        digitalWrite(dir2, HIGH);
        analogWrite(M1, 125); // O motor 1 desliga
        analogWrite(M2, velocidade); // O motor 2 fica ligado, fazendo assim o carrinho virar
        break;

      case esquerda:
        digitalWrite(dir1, LOW);
        digitalWrite(dir2, HIGH); 
        analogWrite(M1, velocidade); 
        analogWrite(M2, 125); 
        break;

      case seguir: 
        Serial.println("Tecla 5");
        break;

      case buzina:
        playMelody();
        IrReceiver.resume(); // Continue a leitura do controle IR após tocar a buzina
        break;

      case seguidor:
      if (estado == 0){
        estado = 1;
        Serial.println("estado1");
      }else {
        estado =0;
      }
      break;

      default:
        noTone(buzzerPin);
        analogWrite(M1, 0); // Ambos motores ligam na mesma velocidade
        analogWrite(M2, 0);
        break;
    }
    IrReceiver.resume();  // Prepare for the next value
  }
  delay(10);
      if (estado == 1){
      Sensor1 = digitalRead(pin_S1);
      Sensor2 = digitalRead(pin_S2);
            // Calcula o erro com base nos valores dos sensores
      if (Sensor1 == 0){
        erro = 1;
      }else if (Sensor1 == 1){
        erro = -2;
      }

        if (Sensor2 == 0){
        erro2 = 1;
      }else if (Sensor2 == 1){
        erro2 = -2;
      }

      // Calcula ação de controle PID
      double acaoP = Kp * erro;
      somaErro += erro;
      double acaoI = Ki * somaErro;
      double acaoD = Kd * (erro - erroAnterior);

      // Calcula ação total
      double acaoTotal = acaoP + acaoI + acaoD;

      // Atualiza o estado anterior
      erroAnterior = erro;

      // Calcula ação de controle PID
      double acaoP2 = Kp * erro2;
      somaErro2 += erro2;
      double acaoI2 = Ki * somaErro2;
      double acaoD2 = Kd * (erro2 - erroAnterior2);

      // Calcula ação total
      double acaoTotal2 = acaoP2 + acaoI2 + acaoD2;

      // Atualiza o estado anterior
      erroAnterior2 = erro2;

      // Calcula a velocidade ajustada com base na ação total
      int velocidadeAjustada = constrain(velocidade + acaoTotal, 0, 255);
      int velocidadeAjustada2 = constrain(velocidade + acaoTotal2, 0, 255);

      digitalWrite(dir1, HIGH);
      digitalWrite(dir2, LOW);
      if((Sensor1 == 0) && (Sensor2 == 0)){ // Se detectar na extremidade das faixas duas cores brancas
      analogWrite(M1, velocidadeAjustada); // Ambos motores ligam na mesma velocidade
      analogWrite(M2, velocidadeAjustada2);
      }
      if((Sensor1 == 1) && (Sensor2 == 0)){ // Se detectar um lado preto e o outro branco (DIREITA)
      digitalWrite(dir1, LOW);
      analogWrite(M1, velocidadeAjustada); // O motor 1 desliga
      analogWrite(M2, velocidadeAjustada2); // O motor 2 fica ligado, fazendo assim o carrinho virar
      }
      if((Sensor1 == 0) && (Sensor2 == 1)){ // Se detectar um lado branco e o outro preto
      digitalWrite(dir2, HIGH);
      analogWrite(M1, velocidadeAjustada); //O motor 1 fica ligado
      analogWrite(M2, velocidadeAjustada2); // O motor 2 desliga, fazendo assim o carrinho virar no outro sentido
      }
      if((Sensor1 == 1) && (Sensor2 == 1)){ // Se detectar um lado branco e o outro preto
      analogWrite(M1, 0); //O motor 1 fica ligado
      analogWrite(M2, 0); // O motor 2 desliga, fazendo assim o carrinho virar no outro sentido
      }
    else if (estado == 0){
      analogWrite(M1, 0);
      analogWrite(M2, 0);
    }
  }
}

