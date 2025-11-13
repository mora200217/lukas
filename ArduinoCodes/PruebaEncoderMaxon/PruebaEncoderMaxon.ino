// === Pines ===
const int pinA = 2;   // Canal A (INT0)
const int pinB = 3;   // Canal B (INT1, solo lectura para dirección)

// === Encoder y reducción ===
const long PPR = 500;                 // Pulsos por vuelta (por canal)
const int  DECODE_X = 1;              // 1x (rápido). Opcional: 2 para 2x (ver notas)
long CPR_eff = PPR * DECODE_X;        // Cuentas por vuelta de motor efectivas
const double REDUCCION = 5.42;        // motor:salida 5.42 Faulhaber y 3.714 MAXON

// === Variables globales ===
volatile long counts = 0;     // cuentas acumuladas a 1x
volatile bool dirCCW = true;  // CCW positivo

// === ISR 1×: solo flanco RISING de A; dirección con B ===
void isrA_RISING() {
  bool B = bitRead(PIND, 3);         // leer canal B rápido
  if (!B) { counts++; dirCCW = true; }   // CCW
  else    { counts--; dirCCW = false;}   // CW
}

void setup() {
  pinMode(pinA, INPUT);  // usa INPUT_PULLUP si tu encoder es colector abierto
  pinMode(pinB, INPUT);

  // 1×: solo un interrupt en A (flancos ascendentes)
  attachInterrupt(digitalPinToInterrupt(pinA), isrA_RISING, RISING);

  Serial.begin(9600);
  while(!Serial) {}
  Serial.println("Medicion 1x: CCW=+ , CW=- (salida)");
}

void loop() {
  static unsigned long tPrev = millis();
  static long countsPrev = 0;

  unsigned long t = millis();
  if (t - tPrev >= 1) {               // cada 1 ms
    noInterrupts();
    long c = counts;
    interrupts();

    long dc = c - countsPrev;
    countsPrev = c;
    double dt = (t - tPrev) / 1000.0;
    tPrev = t;

    // === Velocidad eje de salida ===
    double rpm_motor  = (double)dc / (double)CPR_eff * (60.0 / dt);
    double rpm_salida = rpm_motor / REDUCCION;
    double w_salida   = rpm_salida * (TWO_PI / 60.0);

    // === Posicion eje de salida ===
    double ang_rad_salida = (double)c / (double)CPR_eff * (TWO_PI / REDUCCION);
    double ang_deg_salida = ang_rad_salida * 180.0 / PI;

    // Vueltas completas (floor para negativos correcto)
    long vueltas = (long)floor(ang_rad_salida / (2.0*PI));
    double resto_rad = ang_rad_salida - (double)vueltas * (2.0*PI);
    double resto_deg = resto_rad * 180.0 / PI;

    // === Salida compacta ===
    /*Serial.print("w=");
    Serial.print(w_salida, 3);
    Serial.print(" rad/s | ");
    Serial.print(rpm_salida, 1);
    Serial.print(" rpm || theta=");
    Serial.print(vueltas);
    Serial.print("v + ");
    Serial.print(resto_rad, 4);
    Serial.print(" rad (");
    Serial.print(resto_deg, 1);
    Serial.println("°)");*/
    Serial.println(ang_rad_salida);
  }
}
