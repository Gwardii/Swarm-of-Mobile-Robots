float kd = 2;
double dt;
double last_t = 0;

float kp = 6;
float ki = 0;

//Parametry Ruchu
double t_Start;
double t_End;
double epsilon = 0.001; //przyspieszenie katowe [deg/ms/ms]

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

long silnik_1;
long silnik_2;

int kat_1;
int kat_2;

//stan obecny
int state_1_A;
int state_1_B;
int state_2_A;
int state_2_B;

long potencjometr = 0;
const byte pin_potencjometr = 5;

//Lewy silnik PWM
double suma_bledow_1 = 0;
long kat_obrot_1 = 0;
float wartosc_PWM_1;
float uchyb_silnik_1 = 0;
float last_uchyb_silnik_1 = 0;
float szybk_bledow_1;

//Prawy silnik PWM
double suma_bledow_2 = 0;
long kat_obrot_2 = 0;
float wartosc_PWM_2;
float uchyb_silnik_2 = 0;
float last_uchyb_silnik_2 = 0;
float szybk_bledow_2;

double temp;

//Kola
int d_kola = 42;
int dp1, dp2;
int dp1_last, dp2_last;
double Dl, Dp;
double D, D_last;
float rot = 0, rot_d = 0, rozstaw = 100;
double x_n, y_n;

//Czas w petli
double petla_t1 = 0;
float Xt = 10;
double zad_t = 0; //Czas rozpoczecia wykonywania zadania
//Odbior
const int tabN = 10;
double tablica[tabN][5]; //tablica do odbioru danych o zadaniu. {Nr zadania, dlugosc (prosta/luk) [mm], czas wykonania [ms], promien luku [mm], kat rotacji w miejscu [deg]}
double bufor[5] = {0, 0, 0, 0, 0};
double T1, omega, l;
double *T0;

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

  T0 = &tablica[0][2];

  for (int i = 0; i < tabN; i++)
    for (int j = 0; j < 5; j++)
      tablica[i][j] = 0;


  //Reczne wypelninie tablicy
  tablica[0][0] = 2; //prosto
  tablica[0][1] = 50; //100mm
  tablica[0][2] = 3 * 1000; //w ... sekund

  tablica[1][0] = 1; //stoj
  tablica[1][1] = 0;
  tablica[1][2] = 3 * 1000; //przez ... sekund
}

void loop() {
  //tablica do odbioru danych o zadaniu. {Nr zadania, dlugosc (prosta/luk) [mm], czas wykonania [ms], promien luku [mm], kat rotacji w miejscu [deg]}


  //kat_obrot_1 = map(potencjometr, 0, 1021, -180, 180);
  //kat_obrot_2 = map(potencjometr, 0, 1021, -180, 180);



  //Petla co X ms, obsługa zadania
  if ((millis() - petla_t1) >= Xt) {
    petla_t1 = millis();
    if (tablica[0][0] > 0) { //Czy jest komenda do wykonania?

      if (tablica[0][0] == 1)//Stoj przez ...
        if (zad_t == 0) zad_t == millis(); //Jeżeli zadanie już nie trwa, zapisz czas rozpoczecia


      if (tablica[0][0] == 2) {//Jedz prosto
        if (zad_t == 0) {//Czy to rozpoczecie komendy?
          //Parametry ruchu
          //l = tablica[0][1] / 3.1416 / d_kola; //tu czy podzielic na 2?
          l = 2 * tablica[0][1] / d_kola;
          T1 = (epsilon * *T0  + sqrt((epsilon * *T0) * (epsilon * tablica[0][2])  + 2 * l * epsilon)) / (2 * epsilon); //4ac?
          Serial.println(T1);
          omega = epsilon * T1;

          zad_t == millis(); //Jeżeli zadanie już nie trwa, zapisz czas rozpoczecia
        }
        prosto();
      }


      if (tablica[0][0] == 3) {//Obrot
        if (zad_t == 0) {//Czy to rozpoczecie komendy?
          //Parametry ruchu
          //L=kat*rozstaw
          l = tablica[0][4] / 180 * 3.1416 * rozstaw / 3.1416 / d_kola; //tu czy podzielic na 2?
          T1 = (epsilon * *T0  + sqrt((epsilon * *T0) * (epsilon * tablica[0][2])  + 2 * l * epsilon)) / (2 * epsilon); //4ac?
          omega = epsilon * T1;

          zad_t == millis(); //Jeżeli zadanie już nie trwa, zapisz czas rozpoczecia
        }
        obrot();
      }

      if (millis() - zad_t > tablica[0][2])koniec_CMD(); //Uplynal czas na wykonanie komendy
    }
  }

  PWM(); //PWM na napedy

  odometria();
  delay(1);
}

void obrot() {
//jak prosto, tylko przeciwne kierunki ruchu
}

void prosto() {

  double pr_t = millis();

  double T1_now = pr_t + T1;
  double T0_now = pr_t + *T0;
  double katL0 = kat_1;
  double katP0 = kat_2;
  long kat_obrot = 0;
  if (pr_t <= T1_now) //Czy przyspiesza?
  {
    Serial.println(1);
    kat_obrot = epsilon * millis() * millis() / 2;
  }
  else if (pr_t <= (T0_now - T1_now))  //Czy stala predkosc?
  {
    Serial.println(2);
    kat_obrot = epsilon * T1 * T1 / 2 + omega * (millis() - T1);
  }

  else if (pr_t <= T0_now)//Czy zwalnia?
  {
    Serial.println(3);
    kat_obrot = epsilon * T1 * T1 / 2 + omega * (*T0 - T1) - epsilon * (millis() - *T0 + T1) * (millis() - *T0 + T1) / 2;
  }
  kat_obrot_1 = kat_obrot;
  kat_obrot_2 = kat_obrot;
}

void koniec_CMD() {
  for (int i = 0; i < tabN - 1; i++)
    for (int j = 0; j < 5; j++)
      tablica[i][j] = tablica[i + 1][j];
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

void odbior() {
  int i = 0;
  //dodac jezeli tablica pelna wywal blad
  //while (tablica[i][0] != 0 && i < tabN) {
  //  i++; //przy implementacji komunikacji usunac przy tabN
  //}

  tablica[i][0] = 2; //prosto
  tablica[i][1] = 50; //100mm
  tablica[i][2] = 3 * 1000; //w ... sekund

  tablica[i + 1][0] = 1; //stoj
  tablica[i + 1][1] = 0;
  tablica[i + 1][2] = 3 * 1000; //przez ... sekund

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
