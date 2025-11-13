// === BTS7960 100% + Encoder 1× (A rising) ===
// RPWM -> D5 (PWM activo) | LPWM -> D6 (forzado a LOW)
// R_EN/L_EN -> 5V | Encoder A->D2 (INT0), B->D3 | GND común

const int PIN_RPWM = 5;
const int PIN_LPWM = 6;

const int pinA = 2;  // Encoder A (INT0)
const int pinB = 3;  // Encoder B

const long PPR      = 500;
const int  DECODE_X = 1;
long CPR_eff        = PPR * DECODE_X;
const double REDUCCION = 5.42;

volatile long counts = 0;
volatile bool dirCCW = true;

// ISR: 1× en flanco RISING de A; dirección con B
void isrA_RISING() {
  bool B = bitRead(PIND, 3);   // D3
  if (!B) { counts++; dirCCW = true; }   // CCW positivo
  else    { counts--; dirCCW = false; }  // CW negativo
}

// Muestreo encoder
unsigned long tPrevSample = 0;
long countsPrev = 0;

void setup() {
  // Salidas BTS7960
  pinMode(PIN_RPWM, OUTPUT);
  pinMode(PIN_LPWM, OUTPUT);

  // Sentido "adelante": LPWM LOW, RPWM al 100%
  digitalWrite(PIN_LPWM, LOW);
  analogWrite(PIN_RPWM, 255);   // 100%

  // Encoder
  pinMode(pinA, INPUT);         // usa INPUT_PULLUP si tu encoder lo requiere
  pinMode(pinB, INPUT);
  attachInterrupt(digitalPinToInterrupt(pinA), isrA_RISING, RISING);

  // Serial
  Serial.begin(9600);
  while (!Serial) {}

  tPrevSample = millis();
}

void loop() {
  unsigned long now = millis();

  // ===== Muestreo encoder (cada ~1 ms) =====
  if (now - tPrevSample >= 1) {
    noInterrupts();
    long c = counts;
    interrupts();

    long dc = c - countsPrev;
    countsPrev = c;

    double dt = (now - tPrevSample) / 1000.0; // s
    tPrevSample = now;

    // Velocidad eje motor y eje salida
    double rpm_motor  = (double)dc / (double)CPR_eff * (60.0 / dt);
    double rpm_salida = rpm_motor / REDUCCION;
    (void)rpm_salida; // evita warning si no se usa en cálculos

    // Posición (radianes) en el eje de salida
    double ang_rad_salida = (double)c / (double)CPR_eff * (TWO_PI / REDUCCION);

    // Salida igual al formato anterior
    Serial.print("Radianes = ");
    Serial.print(ang_rad_salida);
    Serial.print("  RPM = ");
    Serial.println(rpm_salida);
  }

  // Nada más: el motor permanece al 100% siempre.
}
