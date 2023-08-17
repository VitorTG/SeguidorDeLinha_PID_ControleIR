#define frente 0xE718FF00
#define tras   0xAD52FF00
#define direita 0xA55AFF00
#define esquerda 0xF708FF00
#define seguir  0xE31CFF00
#define seguidor 0xF20DFF00
#define buzina  0xE916FF00

#define Do 262
#define Re 294 
#define Mi 330
#define Fa 349
#define Sol 392 
#define La 440 
#define Si 494
#define Pausa 0
#define buzzerPin 12
void playMelody(){
  tone(buzzerPin, La);
  delay(500);
  noTone(buzzerPin);
  delay(250);
  
  tone(buzzerPin, La);
  delay(500);
  noTone(buzzerPin);
  delay(250);
  
  tone(buzzerPin, La);
  delay(500);
  noTone(buzzerPin);
  delay(250);
  
  tone(buzzerPin, Fa);
  delay(350);
  noTone(buzzerPin);
  delay(125);
  
  tone(buzzerPin, Do);
  delay(250);
  noTone(buzzerPin);
  delay(150);
  
  tone(buzzerPin, La);
  delay(500);
  noTone(buzzerPin);
  delay(100);
  
  tone(buzzerPin, Fa);
  delay(350);
  noTone(buzzerPin);
  delay(125);
  
  tone(buzzerPin, Do);
  delay(250);
  noTone(buzzerPin);
  delay(150);

  tone(buzzerPin, La);
  delay(500);
  noTone(buzzerPin);
  delay(100);
}