// === Pines ===
const int pinA = 2;   // Canal A
const int pinB = 3;   // Canal B

// === Datos del encoder y reducción ===
const long PPR = 500;                 // Pulsos por revolución del encoder
const long CPR = 4 * PPR;             // 4x -> 2000 counts/vuelta de motor
const double REDUCCION = 26.0 / 7.0;  // ≈ 3.714:1 (motor:salida)

// === Variables globales ===
volatile long counts = 0;      // cuentas acumuladas
volatile bool dirCCW = true;   // dirección (true = CCW)

// === ISR para canal A (4×) ===
void isrA() {
  bool A = bitRead(PIND, 2);   // D2
  bool B = bitRead(PIND, 3);   // D3
  if (A == B) { counts--; dirCCW = false; }  // CW -> negativo
  else        { counts++; dirCCW = true;  }  // CCW -> positivo
}

// === ISR para canal B (4×) ===
void isrB() {
  bool A = bitRead(PIND, 2);
  bool B = bitRead(PIND, 3);
  if (A != B) { counts--; dirCCW = false; }  // CW
  else        { counts++; dirCCW = true;  }  // CCW
}

void setup() {
  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);
  attachInterrupt(digitalPinToInterrupt(pinA), isrA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinB), isrB, CHANGE);

  Serial.begin(115200);
  Serial.println("Velocidad y posicion del EJE DE SALIDA (CCW = +, CW = -)");
}

void loop() {
  static unsigned long tPrev = millis();
  static long countsPrev = 0;

  unsigned long t = millis();
  if (t - tPrev >= 100) {                // cada 100 ms
    noInterrupts();
    long c = counts;
    interrupts();

    long dc = c - countsPrev;            // delta de cuentas
    countsPrev = c;
    double dt = (t - tPrev) / 1000.0;
    tPrev = t;

    // =============================
    // === VELOCIDAD DEL EJE DE SALIDA ===
    // =============================
    // Velocidad motor en RPM (con signo según dirección)
    double rpm_motor = ((double)dc / (double)CPR) * (60.0 / dt);
    // Aplica reducción
    double rpm_salida = rpm_motor / REDUCCION;

    // En rad/s (signo incluido)
    double w_salida = rpm_salida * (TWO_PI / 60.0);

    // =============================
    // === POSICIÓN ANGULAR DEL EJE DE SALIDA ===
    // =============================
    // Ángulo acumulado del eje de salida
    double ang_rad_salida = ((double)c / (double)CPR) * (TWO_PI / REDUCCION);
    double ang_deg_salida = ang_rad_salida * (180.0 / PI);

    // Calcular vueltas completas y residuo
    long vueltas_completas = (long)(ang_rad_salida / (2.0 * PI));
    double resto_rad = ang_rad_salida - (vueltas_completas * 2.0 * PI);
    double resto_deg = resto_rad * (180.0 / PI);

    // Corrige el signo del resto si CCW es positivo, CW negativo
    // (ya lo está implícito porque counts tiene signo)

    // =============================
    // === IMPRESIÓN ===
    // =============================
    Serial.print("ω = ");
    Serial.print(w_salida, 3);
    Serial.print(" rad/s  |  ");
    Serial.print(rpm_salida, 2);
    Serial.print(" RPM   ||   θ = ");

    Serial.print(vueltas_completas);
    Serial.print(" vueltas + ");
    Serial.print(resto_rad, 4);
    Serial.print(" rad (");
    Serial.print(resto_deg, 2);
    Serial.println(" °)");
  }
}
