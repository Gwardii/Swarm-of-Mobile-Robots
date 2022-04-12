#include "BipBop.h"
//Parametry Ruchu
double t_Start;
double t_End;


double epsilon = 0.001; //przyspieszenie katowe [deg/ms/ms]
double T1, omega;
double L, T0, l;


//parametry petli
int X = 10; //Czas odświeżania funkcji zadawania predkosci
double czas_petla;
int zadanie = 0; //zmienna wyboru zadania - prosto itp
bool wykonane = true; //czy bierzące zadanie zostało wykonane?
double tablica[5] = {0, 0, 0, 0, 0}; //tablica do odbioru danych o zadaniu. {Nr zadania, dlugosc (prosta/luk) [mm], czas wykonania [ms], promien luku [mm], kat rotacji w miejscu [deg]}

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
double kp = 1;
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








void setup() {
  // put your setup code here, to run once:
  pinMode(silnik_1_A, INPUT_PULLUP);
  pinMode(silnik_2_A, INPUT_PULLUP);


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

  odbior();

  

  if ( zadanie == 0) //stoj
  {
    digitalWrite(pin_kierunek_silnik_lewy, HIGH);
    analogWrite(pin_PWM_silnik_lewy, 0);
    digitalWrite(pin_kierunek_silnik_prawy, HIGH);
    analogWrite(pin_PWM_silnik_prawy, 0);
  }

  if (zadanie == 1) //jedz prosto
  {
    wykonane = false;
    prosto();
  }




  robot.odometria();

  delay(10);
}

void prosto() {
  omega = epsilon * T1;
  czas_petla = millis();
  l = L / 3.1416 / d_kola; //tu cos nie tak
  T1 = (epsilon * T0  + sqrt((epsilon * T0) * (epsilon * T0)  + 2 * l * epsilon)) / (2 * epsilon); //4ac?
  
  double T1_now = czas_petla + T1;
  double T0_now = czas_petla + T0;
  double katL0 = kat_1;
  double katP0 = kat_2;
  Serial.println(T1);
  while (wykonane == false) { //czy komenda została zakończona?
    
    if (millis() - czas_petla >= X) {
      czas_petla = millis();
   
      if (czas_petla <= T1_now) //Czy przyspiesza?
      {
        kat_obrot = epsilon * millis() * millis() / 2;
        robot.pwmL(katL0 + kat_obrot, kat_1);
        robot.pwmR(katP0 + kat_obrot, kat_2);
      }
      else if (czas_petla <= (T0_now - T1_now))  //Czy stala predkosc?
      {
        kat_obrot = epsilon * T1 * T1 / 2 + omega * (millis() - T1);
        robot.pwmL(katL0 + kat_obrot, kat_1);
        robot.pwmR(katP0 + kat_obrot, kat_2);
      }

      else if (czas_petla <= T0_now)//Czy zwalnia?
      {
        kat_obrot = epsilon * T1 * T1 / 2 + omega * (T0 - T1) - epsilon * (millis() - T0 + T1) * (millis() - T0 + T1) / 2;
        robot.pwmL(katL0 + kat_obrot, kat_1);
        robot.pwmR(katP0 + kat_obrot, kat_2);
      }

      else wykonane = true; //Koniec
      zadanie = 0;
    }
  }
}

void odbior() {
  if (analogRead(pin_potencjometr) > 300)
  {

    tablica[0] = 1; //prosto
    tablica[1] = 50; //100mm
    tablica[2] = 10000; //w 10 sekund
  }
  else
  {

    tablica[0] = 0; //stoj
    tablica[1] = 0;
    tablica[2] = 0;
  }

  zadanie = tablica[0];
  L = tablica[1];
  T0 = tablica[2];
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
