const float kd = 2;
double dt;
double last_t = 0;

const float kp = 6;
const float ki = 0;

//Xbee
bool started = false; //True: Message is started
bool ended   = false;//True: Message is finished
char incomingByte ; //Variable to store the incoming byte
char msg[3];    //Message - array from 0 to 2 (3 values - PWM - e.g. 240)
byte index;     //Index of array
int value = 0;
double Tjazda;
uint8_t XbeeGetTab[16];
int XIndex = 0;

//Parametry Ruchu
double t_Start;
double t_End;
double epsilon = 5 * 0.000001; //przyspieszenie katowe [deg/ms/ms]
//Piny napedow
const byte silnik_1_A = 2;
const byte silnik_1_B = 5;
const byte silnik_2_A = 3;
const byte silnik_2_B = 6;

const int pin_kierunek_silnik_lewy = 7;
const int pin_PWM_silnik_lewy = 9;
const int pin_kierunek_silnik_prawy = 8 ;
const int pin_PWM_silnik_prawy = 10;

int wartosc_PWM_1_silnik_lewy = 0;
int wartosc_PWM_1_silnik_prawy = 0;
int PWM_max = 255;

//silnik_1 lewy
//silnik_2 prawy

long long silnik_1;
long long silnik_2;

double kat_1;
double kat_2;

//stan obecny
int state_1_A;
int state_1_B;
int state_2_A;
int state_2_B;

long potencjometr = 0;
const byte pin_potencjometr = 5;

//Lewy silnik PWM
double suma_bledow_1 = 0;
long long kat_obrot_1 = 0;
float wartosc_PWM_1;
float uchyb_silnik_1 = 0;
float last_uchyb_silnik_1 = 0;
float szybk_bledow_1;

//Prawy silnik PWM
double suma_bledow_2 = 0;
long long kat_obrot_2 = 0;
float wartosc_PWM_2;
float uchyb_silnik_2 = 0;
float last_uchyb_silnik_2 = 0;
float szybk_bledow_2;

double temp;

//Kola
const float d_kola = 42;
int dp1, dp2;
int dp1_last, dp2_last;
double Dl, Dp;
double D, D_last;
float rot = 0, rot_d = 0;
const float rozstaw = 94;
double x_n, y_n;

//Czas w petli
double petla_t1 = 0;
float Xt = 10;
double zad_t = 0; //Czas rozpoczecia wykonywania zadania
double T1_now;
double T0_now;
double katL0;
double katP0;
double L1, L2;
int a;

//Odbior
const int tabN = 20;
struct Task{
  uint8_t id;
  uint16_t distance;
  uint32_t _time;
  uint16_t radius;
  uint16_t orientation;
};
struct Task tablica[tabN]; //tablica do odbioru danych o zadaniu. {Nr zadania, dlugosc (prosta/luk) [mm], czas wykonania [ms], promien luku [mm], kat rotacji w miejscu [deg]}
uint8_t tablica_empty_id = 0;
float bufor[5] = {0, 0, 0, 0, 0};
float T1, beta, delta, L_obr;
uint16_t *L, *Mi;
uint32_t *T0;

void setup() {
  // put your setup code here, to run once:
  pinMode(silnik_1_A, INPUT_PULLUP);
  pinMode(silnik_2_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(silnik_1_A), odczyt_1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(silnik_2_A), odczyt_2, CHANGE);

  pinMode(pin_kierunek_silnik_lewy, OUTPUT);
  pinMode(pin_PWM_silnik_lewy, OUTPUT);
  pinMode(pin_kierunek_silnik_prawy, OUTPUT);
  pinMode(pin_PWM_silnik_prawy, OUTPUT);

  Serial.begin(9600);

  T0 = &tablica[0]._time;
  L = &tablica[0].distance;
  Mi = &tablica[0].orientation;
}

void loop() {
  //Petla co X ms, obsługa zadania
  if ((millis() - petla_t1) >= Xt) {
    petla_t1 = millis();
    if (tablica_empty_id > 0) { //Czy jest komenda do wykonania?

      if (tablica[0].id == 1)//Stoj przez ...
        if (zad_t == 0)
          zad_t = millis(); //Jeżeli zadanie już nie trwa, zapisz czas rozpoczecia


      if (tablica[0].id == 2) {//Jedz prosto
        if (zad_t == 0) {//Czy to rozpoczecie komendy?
          zad_t = millis(); //Jeżeli zadanie jeszcze nie trwa, zapisz czas rozpoczecia
          //Parametry ruchu
          delta = (epsilon * epsilon) * (*T0 * *T0) - 8 * epsilon / d_kola * *L;
          if (delta < 0) Serial.println("Zle zadany ruch");
          T1 = *T0 / 2. - sqrt(delta) / 2 / epsilon;
          T1_now = zad_t + T1;
          T0_now = zad_t + *T0;
          katL0 = kat_1;
          katP0 = kat_2;
        }
        prosto();
      }


      if (tablica[0].id == 3) {//Obrot
        if (zad_t == 0) {//Czy to rozpoczecie komendy?
          zad_t = millis(); //Jeżeli zadanie jeszcze nie trwa, zapisz czas rozpoczecia
          //Parametry ruchu
          if (*Mi > 0) a = 1; else a = -1;
          L_obr = a**Mi / 180. * 3.1416 * rozstaw / 2;
          delta = (epsilon * epsilon) * (*T0 * *T0) - 8 * epsilon / d_kola * L_obr;
          if (delta < 0) Serial.println("Zle zadany ruch");
          T1 = *T0 / 2. - sqrt(delta) / 2 / epsilon;
          T1_now = zad_t + T1;
          T0_now = zad_t + *T0;
          katL0 = kat_1;
          katP0 = kat_2;
        }
        obrot();
      }


      if (tablica[0].id == 4 || tablica[0].id == 5) {//Luk
        if (zad_t == 0) {//Czy to rozpoczecie komendy?
          zad_t = millis(); //Jeżeli zadanie jeszcze nie trwa, zapisz czas rozpoczecia
          //Parametry ruchu
          delta = (epsilon * epsilon) * (*T0 * *T0) - 8 * epsilon / d_kola * *L;
          if (delta < 0) Serial.println("Zle zadany ruch");
          T1 = *T0 / 2. - sqrt(delta) / 2 / epsilon;
          T1_now = zad_t + T1;
          T0_now = zad_t + *T0;
          katL0 = kat_1;
          katP0 = kat_2;
        }
        luk();
      }

      if (millis() - zad_t > tablica[0]._time) koniec_CMD(); //Uplynal czas na wykonanie komendy
    }
  }

  PWM(); //PWM na napedy

  odometria();
  delay(1);
}

void serialEvent() {
  while (Serial.available() > 0) {
    //Odbierz bite
    if (!started) {
      XbeeGetTab[0] = XbeeGetTab[1];
      XbeeGetTab[1] = Serial.read();
    }
    else
    {
      XbeeGetTab[XIndex + 2] = Serial.read();
      XIndex++;
    }
    //Zacznij wiadomosc
    if (XbeeGetTab[0] == 0x20 && XbeeGetTab[1] == 0x40)
    {
      started = true;
      hardCodedProsto();
    }
    //Zakoncz wiadomosc
    else if (XbeeGetTab[14] == 0x50 && XbeeGetTab[15] == 0x60)
    {
      ended = true;
      for (int g = 0; g < 16; g++)
        XbeeGetTab[g] = 0;
      XIndex = 0;

      break; // Done reading - exit from while loop!
    }
    if (XIndex >= 13) { //Jezeli wiadomosc za dluga odrzuc
      ended = false;
      started = false;
      for (int g = 0; g < 16; g++)
        XbeeGetTab[g] = 0;
      XIndex = 0;
      break;
    }

  }

  uint16_t ChSum = 0;
  for (int elo = 0; elo < 14; elo++) {
    ChSum += XbeeGetTab[elo];
  }

  if (((uint16_t)XbeeGetTab[13] << 8) | XbeeGetTab[12] == ChSum) {

    if (started && ended)
    {
      tablica[tablica_empty_id].id = XbeeGetTab[2];
      tablica[tablica_empty_id].distance = ((uint16_t)XbeeGetTab[4] << 8) | XbeeGetTab[3];
      tablica[tablica_empty_id]._time = (((uint32_t)XbeeGetTab[7] << 16) | ((uint32_t)XbeeGetTab[6] << 8)) | XbeeGetTab[5];
      tablica[tablica_empty_id].radius = ((uint16_t)XbeeGetTab[9] << 8) | XbeeGetTab[8];
      tablica[tablica_empty_id].orientation = ((uint16_t)XbeeGetTab[11] << 8) | XbeeGetTab[10];
    }

  }
  tablica_empty_id += 1;
  ChSum = 0;
  ended = false;
  started = false;
  for (int g = 0; g < 16; g++)
    XbeeGetTab[g] = 0;
  XIndex = 0;

}

void hardCodedProsto(){
  delta = (epsilon * epsilon) * (*T0 * *T0) - 8 * epsilon / d_kola * *L;
  if (delta < 0) Serial.println("Zle zadany ruch");
  T1 = *T0 / 2. - sqrt(delta) / 2 / epsilon;
  T1_now = zad_t + T1;
  T0_now = zad_t + *T0;
  katL0 = kat_1;
  katP0 = kat_2;
  tablica[0].id = 2; //prosto
  tablica[0].distance = 500; //500mm
  tablica[0]._time = 10 * 1000; //w 10 sekund
  tablica_empty_id = 1;
  prosto();
}


void luk() {

  double pr_t = millis();

  //Serial.print("Faza: ");

  double kat_obrot = 0;
  if (pr_t <= T1_now) //Czy przyspiesza?
  {
    //    Serial.print(millis() - zad_t);
    //    Serial.print("ms - petla)");
    //Serial.print(1);
    kat_obrot = epsilon * (pr_t - zad_t) * (pr_t - zad_t) / 2;
  }
  else if (pr_t <= (T0_now - T1))  //Czy stala predkosc?
  {
    //    Serial.print(millis() - zad_t);
    //    Serial.print("ms - petla)");
    //Serial.print(2);
    kat_obrot = epsilon * T1 * T1 / 2 + epsilon * T1 * (pr_t - zad_t - T1);
  }

  else if (pr_t <= T0_now)//Czy zwalnia?
  {
    //    Serial.print(millis() - zad_t);
    //    Serial.print("ms - petla)");
    //Serial.print(3);
    kat_obrot = epsilon * T1 * T1 / 2 + epsilon * T1 * (*T0 - 2 * T1) + epsilon * T1 * (pr_t - zad_t - *T0 + T1) - epsilon * (pr_t - zad_t - *T0 + T1) * (pr_t - zad_t - *T0 + T1) / 2;
  }

  else kat_obrot = epsilon * T1 * T1 / 2 + epsilon * T1 * (*T0 - 2 * T1) + epsilon * T1 * T1 - epsilon * T1 * T1 / 2;

  kat_obrot = kat_obrot / 3.1416 * 180;
  //Serial.print("   Kat:");
  //Serial.println(kat_obrot);
  float aa = rozstaw / 2. / tablica[0].radius;

  if (tablica[0].id == 4) {
    kat_obrot_1 = kat_obrot * (1 - aa) + katL0;
    Serial.println(kat_obrot);
    kat_obrot_2 = kat_obrot * (1 + aa) + katP0;
  }

  if (tablica[0].id == 5) {
    kat_obrot_1 = kat_obrot * (1 + aa) + katL0;
    kat_obrot_2 = kat_obrot * (1 - aa) + katP0;
  }
}

void obrot() {
  double pr_t = millis();

  //Serial.print("Faza: ");

  double kat_obrot = 0;
  if (pr_t <= T1_now) //Czy przyspiesza?
  {
    //    Serial.print(millis() - zad_t);
    //    Serial.print("ms - petla)");
    //Serial.print(1);
    kat_obrot = epsilon * (pr_t - zad_t) * (pr_t - zad_t) / 2;
  }
  else if (pr_t <= (T0_now - T1))  //Czy stala predkosc?
  {
    //    Serial.print(millis() - zad_t);
    //    Serial.print("ms - petla)");
    //Serial.print(2);
    kat_obrot = epsilon * T1 * T1 / 2 + epsilon * T1 * (pr_t - zad_t - T1);
  }

  else if (pr_t <= T0_now)//Czy zwalnia?
  {
    //    Serial.print(millis() - zad_t);
    //    Serial.print("ms - petla)");
    //Serial.print(3);
    kat_obrot = epsilon * T1 * T1 / 2 + epsilon * T1 * (*T0 - 2 * T1) + epsilon * T1 * (pr_t - zad_t - *T0 + T1) - epsilon * (pr_t - zad_t - *T0 + T1) * (pr_t - zad_t - *T0 + T1) / 2;
  }

  else kat_obrot = epsilon * T1 * T1 / 2 + epsilon * T1 * (*T0 - 2 * T1) + epsilon * T1 * T1 - epsilon * T1 * T1 / 2;

  kat_obrot = kat_obrot / 3.1416 * 180;
  //Serial.print("   Kat:");
  //Serial.println(kat_obrot);
  kat_obrot_1 = -a * kat_obrot + katL0;
  kat_obrot_2 = a * kat_obrot + katP0;
}

void prosto() {

  double pr_t = millis();

  //Serial.print("Faza: ");

  double kat_obrot = 0;
  if (pr_t <= T1_now) //Czy przyspiesza?
  {
    //    Serial.print(millis() - zad_t);
    //    Serial.print("ms - petla)");
    //Serial.print(1);
    kat_obrot = epsilon * (pr_t - zad_t) * (pr_t - zad_t) / 2;
  }
  else if (pr_t <= (T0_now - T1))  //Czy stala predkosc?
  {
    //    Serial.print(millis() - zad_t);
    //    Serial.print("ms - petla)");
    //Serial.print(2);
    kat_obrot = epsilon * T1 * T1 / 2 + epsilon * T1 * (pr_t - zad_t - T1);
  }

  else if (pr_t <= T0_now)//Czy zwalnia?
  {
    //    Serial.print(millis() - zad_t);
    //    Serial.print("ms - petla)");
    //Serial.print(3);
    kat_obrot = epsilon * T1 * T1 / 2 + epsilon * T1 * (*T0 - 2 * T1) + epsilon * T1 * (pr_t - zad_t - *T0 + T1) - epsilon * (pr_t - zad_t - *T0 + T1) * (pr_t - zad_t - *T0 + T1) / 2;
  }

  else kat_obrot = epsilon * T1 * T1 / 2 + epsilon * T1 * (*T0 - 2 * T1) + epsilon * T1 * T1 - epsilon * T1 * T1 / 2;

  kat_obrot = kat_obrot / 3.1416 * 180;
  //Serial.print("   Kat:");
  //Serial.println(kat_obrot);
  kat_obrot_1 = kat_obrot + katL0;
  kat_obrot_2 = kat_obrot + katP0;
}

void koniec_CMD() {
  for (int i = 0; i < tablica_empty_id; i++)
    tablica[i] = tablica[i+1];
  zad_t = 0;
  tablica_empty_id -=1;
}

void PWM() {
  dt = millis() - last_t; //
  //Lewy PWM
  uchyb_silnik_1 = kat_obrot_1 - kat_1;
  suma_bledow_1 = suma_bledow_1 + uchyb_silnik_1 * dt; //
  szybk_bledow_1 = (uchyb_silnik_1 - last_uchyb_silnik_1) / dt; //
  wartosc_PWM_1 = abs(uchyb_silnik_1 * kp + ki * suma_bledow_1 + kd * szybk_bledow_1); //

  //Prawy PWM
  uchyb_silnik_2 = kat_obrot_2 - kat_2;
  suma_bledow_2 = suma_bledow_2 + uchyb_silnik_2 * dt; //
  szybk_bledow_2 = (uchyb_silnik_2 - last_uchyb_silnik_2) / dt; //
  wartosc_PWM_2 = abs(uchyb_silnik_2 * kp + ki * suma_bledow_2 + kd * szybk_bledow_2); //

  //Lewy obrot
  if (wartosc_PWM_1 > PWM_max) wartosc_PWM_1 = PWM_max;
  if (uchyb_silnik_1 >= 0)
  {
    digitalWrite(pin_kierunek_silnik_lewy, LOW);
    analogWrite(pin_PWM_silnik_lewy, wartosc_PWM_1);
  }
  else
  {
    digitalWrite(pin_kierunek_silnik_lewy, HIGH);
    analogWrite(pin_PWM_silnik_lewy, wartosc_PWM_1);
  }
  //last_t = millis(); //
  last_uchyb_silnik_1 = uchyb_silnik_1; //

  //Prawy obrot
  if (wartosc_PWM_2 > PWM_max) wartosc_PWM_2 = PWM_max;
  if (uchyb_silnik_2 >= 0)
  {
    digitalWrite(pin_kierunek_silnik_prawy, LOW);
    analogWrite(pin_PWM_silnik_prawy, wartosc_PWM_2);
  }
  else
  {
    digitalWrite(pin_kierunek_silnik_prawy, HIGH);
    analogWrite(pin_PWM_silnik_prawy, wartosc_PWM_2);
  }
  last_uchyb_silnik_2 = uchyb_silnik_2; //

  last_t = millis(); //

}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
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

void odometria()
{
  Dl = Dl + (3.1415 * dp1) * d_kola / 230;
  Dp = Dp + (3.1415 * dp2) * d_kola / 230;

  D = (Dl + Dp) / 2;
  rot = (Dp - Dl) / rozstaw;
  if (rot >= 2 * 3.1415) {
    rot = rot - 2 * 3.1415;
  }
  rot_d = rot * 180 / 3.1415;
  x_n = x_n + (D - D_last) * cos(rot);
  y_n = y_n + (D - D_last) * sin(rot);
  D_last = D;
}
