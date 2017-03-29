/*

unsigned long currentTime;

unsigned long loopTime;

const int pin_A = 6; // pin 12

const int pin_B = 5; // pin 11

unsigned char encoder_A;

unsigned char encoder_B;

unsigned char encoder_A_prev=0;

int rotation=100;

void setup() { // declare pin 9 to be an output:

pinMode(9, OUTPUT);

pinMode(pin_A, INPUT);

pinMode(pin_B, INPUT);

Serial.begin (9600);

currentTime = millis();

loopTime = currentTime; 

}

void loop() {

currentTime = millis();

if(currentTime >= (loopTime + 5))
  { // проверяем каждые 5мс (200 Гц)

    encoder_A = digitalRead(pin_A); // считываем состояние выхода А энкодера

    encoder_B = digitalRead(pin_B); // считываем состояние выхода А энкодера

    if((!encoder_A) && (encoder_A_prev))
    { // если состояние изменилось с положительного к нулю

      if(encoder_B) 
        { // выход В в полож. сост., значит вращение по часовой стрелке

          rotation ++;
        }
      else 
        { // выход В в 0 сост., значит вращение против часовой стрелк
          rotation --;
        }

    }

    encoder_A_prev = encoder_A; // сохраняем значение А для следующего цикла
  
    //analogWrite(9, brightness); // устанавливаем яркость на 9 ножку
    
    
    Serial.println(rotation);

    loopTime = currentTime; 
  }
 
}
*/
