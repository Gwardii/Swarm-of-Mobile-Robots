

class BipBop {
  //private:
  public:
    int d_kola; //stednica kola
    float rozstaw; //rozstaw kol

    //Piny
    int pin_kierunek_silnik_lewy;
    int pin_PWM_silnik_lewy;
    int pin_kierunek_silnik_prawy;
    int pin_PWM_silnik_prawy;

    //PI
    double kp;
    double ki;
    double kd;
    double suma_bledowL,suma_bledowR;
    double dt,dt2;
    float szybk_bledowL,szybk_bledowR;
    double last_t,last_t2;
    double last_uchyb_silnik_1,last_uchyb_silnik_2;
  
  //public:
    int dp1, dp2;
    int dp1_last, dp2_last;
    double Dl, Dp; //dystans kolo lewe i prawe
    double D, D_last; //dystans srodka i poprzedni srodka
    double rot = 0, rot_d = 0; //kat, kat w deg
    double x_n, y_n;
    double silnik_1;
    double silnik_2;
    int kat_1;
    int kat_2;

    void odometria();
    void setPID(double Kp, double Ki, double Kd);
    void init(int dkol, float rozst,  int pin_kierunek_silnik_l,    int pin_PWM_silnik_l,    int pin_kierunek_silnik_p,    int pin_PWM_silnik_p);
    void odczyt_1();
    void odczyt_2();
    void pwmL(long kat_obrot, float kat_l);
    void pwmR(long kat_obrot, float kat_l);
};

void BipBop::setPID(double Kp, double Ki, double Kd) {
  kp = Kp;
  ki = Ki;
  kd = Kd;
  suma_bledowL = 0;
  suma_bledowR = 0;
}

void BipBop::init(int dkol, float rozst,  int pin_kierunek_silnik_l,    int pin_PWM_silnik_l,    int pin_kierunek_silnik_p,    int pin_PWM_silnik_p) {
  Dl = 0;
  Dp = 0;
  rot = 0;
  rot_d = 0;
  szybk_bledowL = 0;
  szybk_bledowR = 0;
  last_t = 0;
  last_t2 = 0;
  
  last_uchyb_silnik_1 = 0;
  last_uchyb_silnik_2 = 0;
  
  pin_kierunek_silnik_lewy = pin_kierunek_silnik_l;
  pin_PWM_silnik_lewy = pin_PWM_silnik_l;
  pin_kierunek_silnik_prawy = pin_kierunek_silnik_p ;
  pin_PWM_silnik_prawy = pin_PWM_silnik_p;

  d_kola = dkol;
  rozstaw = rozst;

  pinMode(pin_kierunek_silnik_lewy, OUTPUT);
  pinMode(pin_PWM_silnik_lewy, OUTPUT);
  pinMode(pin_kierunek_silnik_prawy, OUTPUT);
  pinMode(pin_PWM_silnik_prawy, OUTPUT);

}

void BipBop::odometria() //zero zero przy odpaleniu
{

  Dl = Dl + (3.1415 * dp1) * d_kola / 230; //dystans kolo lewe
  Dp = Dp + (3.1415 * dp2) * d_kola / 230; //dystans kolo prawe

  D = (Dl + Dp) / 2; // srodka
  rot = (Dp - Dl) / rozstaw; //rotacja srodka w radianach
  if (rot >= 2 * 3.1415) {
    rot = rot - 2 * 3.1415;
  }
  rot_d = rot * 180 / 3.1415; //rotacja w katach

  //x y polozenia srodka, y do przodu, x w prawo
  x_n = x_n + (D - D_last) * cos(rot);
  y_n = y_n + (D - D_last) * sin(rot);
  D_last = D; //D od ostatniego sprawdzenia pozycji
}

void BipBop::pwmL(long kat_obrot, float kat_l) //kat zadany, kat odczytany (deg)
{ 
  dt = millis() - last_t; //
  double uchyb_silnik_1 = kat_obrot - kat_l;
  suma_bledowL = suma_bledowL + uchyb_silnik_1 * dt;
  szybk_bledowL = (uchyb_silnik_1 - last_uchyb_silnik_1) / dt;
  double wartosc_PWM = abs(uchyb_silnik_1 * kp + ki * suma_bledowL + kd * szybk_bledowL); //

  if (uchyb_silnik_1 >= 0)
  {
    digitalWrite(pin_kierunek_silnik_lewy, LOW);
    analogWrite(pin_PWM_silnik_lewy, wartosc_PWM);
  }
  else
  {
    digitalWrite(pin_kierunek_silnik_lewy, HIGH);
    analogWrite(pin_PWM_silnik_lewy, wartosc_PWM);
  }

  last_t = millis(); //
  last_uchyb_silnik_1 = uchyb_silnik_1; //
}

void BipBop::pwmR(long kat_obrot, float kat_l) {
  dt2 = millis() - last_t2; //
  double uchyb_silnik_2 = kat_obrot - kat_l;
  suma_bledowR = suma_bledowR + uchyb_silnik_2 * dt2;
  szybk_bledowR = (uchyb_silnik_2 - last_uchyb_silnik_2) / dt2;
  double wartosc_PWM = abs(uchyb_silnik_2 * kp + ki * suma_bledowR + kd * szybk_bledowR); //

  if (uchyb_silnik_2 >= 0)
  {
    digitalWrite(pin_kierunek_silnik_prawy, LOW);
    analogWrite(pin_PWM_silnik_prawy, wartosc_PWM);
  }
  else
  {
    digitalWrite(pin_kierunek_silnik_prawy, HIGH);
    analogWrite(pin_PWM_silnik_prawy, wartosc_PWM);
  }

  last_t2 = millis(); //
  last_uchyb_silnik_2 = uchyb_silnik_2; //
}


long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
