#include "BipBop.h"
//Parametry Ruchu
double t_Start;
double t_End;
double A = 1;
double Vmax = 10;
int faza = 0;
bool flaga = false;
double czas;

int X = 10; //Czas od

double zad_l;
double zad_t; 
//Silniki PINy
const byte silnik_1_A = 2;
const byte silnik_1_B = 5;
const byte silnik_2_A = 3;
const byte silnik_2_B = 6;

const int pin_kierunek_silnik_lewy = 7;
const int pin_PWM_silnik_lewy = 9;
const int pin_kierunek_silnik_prawy = 8;
const int pin_PWM_silnik_prawy = 10;

const byte pin_potencjometr = 5;

//PID
double kp = 6;
double ki = 0;
double kd = 0;
//Robot
int d_kola = 42;
double rozstaw = 100; //do policzenia

//Zmienne na potem
int wartosc_PWM_silnik_lewy = 0;
int wartosc_PWM_silnik_prawy = 0;
int PWM_max = 255;

//Zmienne na PWM
//silnik_1 lewy
//silnik_2 prawy
double silnik_1;
double silnik_2;

//enkodery
float kat_1;
float kat_2;

//stan obecny
int state_1_A;
int state_1_B;
int state_2_A;
int state_2_B;

long potencjometr = 0;
long kat_obrot = 0;

int dp1, dp2;
int dp1_last, dp2_last;

BipBop robot;

//Przycisk
int lastState = LOW;      // the previous state from the input pin
int currentState;         // the current reading from the input pin
int czas1 = 0;







void setup() {
  // put your setup code here, to run once:
  pinMode(silnik_1_A, INPUT_PULLUP);
  pinMode(silnik_2_A, INPUT_PULLUP);
  pinMode(4, INPUT);

  attachInterrupt(digitalPinToInterrupt(silnik_1_A), odczyt_1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(silnik_2_A), odczyt_2, CHANGE);
  //inicjalizacja robota
  robot.init(d_kola, rozstaw,  pin_kierunek_silnik_lewy,  pin_PWM_silnik_lewy,    pin_kierunek_silnik_prawy,   pin_PWM_silnik_prawy);
  robot.setPID(kp, ki, kd);

  Serial.begin(9600);
}


void loop() {

  //potencjometr = analogRead(pin_potencjometr); //odczyt z potencjometru
  //kat_obrot = 3* map(potencjometr, 0, 1021, -180, 180); //mapowanie potencjometru na -180 do 180
  
  if(czas+3000<millis())
  kat_obrot++;

  if(czas+5000<millis())
  czas=millis();
  //przycisk();
  Serial.println(flaga);
  if (flaga == true){
czas=millis()- t_Start;
//if(Vmax/A>czas) faza=1;
//else if(t_End-Vmax/A>czas) faza=2;
//else if(czas<t_End) faza = 3;
//else flaga = false;

int i;
    
    flaga = false;
    switch ( faza )
    {
      case 1:

        break;
      case 2:

        break;
      case 3:

        
        break;
    }}

  robot.pwmL(kat_obrot, kat_1);
  robot.pwmR(kat_obrot, kat_2);
  robot.odometria();

}

void przycisk()
{
  if(millis()>czas+10000)flaga=!flaga;

  
  currentState = digitalRead(4);
  if (lastState == HIGH && currentState == LOW && (millis()-czas1)>3500) {
    czas1=millis();
    flaga = true;
    t_Start = millis();
    zad_l=10;
    zad_t=10;
  }
  lastState = currentState;
}
void odczyt_1()
{


  state_1_A = digitalRead(silnik_1_A);
  state_1_B = digitalRead(silnik_1_B);

  if (state_1_A == HIGH) {
    if (state_1_B == LOW) {
      silnik_1--;
    } else {
      silnik_1++;
    }
  }

  dp1 = silnik_1 - dp1_last;
  dp1_last = silnik_1;

  kat_1 = silnik_1 * 360 / 230;

}

void odczyt_2()
{
  state_2_A = digitalRead(silnik_2_A);
  state_2_B = digitalRead(silnik_2_B);

  if ((state_2_A == HIGH)) {
    if (state_2_B == LOW) {
      silnik_2++;
    } else {
      silnik_2--;
    }
  }

  dp2 = silnik_2 - dp2_last;
  dp2_last = silnik_2;

  kat_2 = silnik_2 * 360 / 230;
}
