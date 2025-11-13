// === Eje de SALIDA (R = 5.42:1) ===
// Velocidad (RPM, rad/s) + Ángulo sobrante firmado (-360..360°, -2π..2π) según descomposición por truncado
// Aviso por vueltas completas. Sin textos de CW/CCW/DETENIDO.

#include <ctype.h>   // tolower
#include <stdlib.h>  // labs
#include <math.h>    // fabs

#define USE_PULLUPS 1    // 1=INPUT_PULLUP (open-collector) | 0=INPUT (push-pull TTL)
#define INVERTIR   0     // 0 normal | 1 invierte sentido
#define PPR        500   // PPR del encoder (por canal) -> 1×
#define R          5.42  // Reducción (motor:salida)

const uint8_t pinA = 2;  // INT0
const uint8_t pinB = 3;  // INT1

const unsigned long VENTANA_MS = 200;
const int MIN_PULSOS = 1;

volatile long pos_counts_m = 0;              // posición (motor) con signo, 1×
volatile unsigned long pulse_counter_m = 0;  // pulsos A↑ (magnitud, 1×)

void isrA_rising() {
  int b = digitalRead(pinB);
  int s = (b ? +1 : -1);
  if (INVERTIR) s = -s;
  pos_counts_m += s;
  pulse_counter_m++;
}

void setup() {
  if (USE_PULLUPS) { pinMode(pinA, INPUT_PULLUP); pinMode(pinB, INPUT_PULLUP); }
  else             { pinMode(pinA, INPUT);        pinMode(pinB, INPUT); }

  attachInterrupt(digitalPinToInterrupt(pinA), isrA_rising, RISING);

  Serial.begin(115200);
  Serial.println(F("Salida: velocidad + angulo sobrante firmado + aviso de vueltas [1× en A, R=5.42]"));
  Serial.println(F("Comando 'z' -> resetear contadores"));
}

void loop() {
  // Reset por comando
  if (Serial.available() && tolower(Serial.read()) == 'z') {
    noInterrupts();
    pos_counts_m = 0;
    pulse_counter_m = 0;
    interrupts();
    Serial.println(F("Reset OK"));
  }

  static unsigned long t0 = millis();
  static long pos_prev_m = 0;
  static unsigned long pulses_prev_m = 0;
  static long vueltas_prev_out = 0;

  unsigned long t = millis();
  if (t - t0 >= VENTANA_MS) {
    unsigned long dt_ms = t - t0; 
    t0 = t;

    // Lecturas atómicas
    long pos_c_m; 
    unsigned long p_m;
    noInterrupts(); 
    pos_c_m = pos_counts_m; 
    p_m     = pulse_counter_m; 
    interrupts();

    // Incrementos en ventana
    long dpos_m = pos_c_m - pos_prev_m;
    pos_prev_m  = pos_c_m;
    int sgn     = (dpos_m > 0) - (dpos_m < 0);

    unsigned long dp_m = p_m - pulses_prev_m;
    pulses_prev_m      = p_m;

    // Velocidades motor
    double dt = dt_ms / 1000.0;
    double rpm_motor_mag = (dp_m > 0) ? ((double)dp_m / (double)PPR) * (60.0 / dt) : 0.0;
    double rpm_motor     = sgn * rpm_motor_mag;
    double w_motor       = rpm_motor * (TWO_PI / 60.0);

    // ===== SALIDA =====
    // Velocidad (signada)
    double rpm_out = rpm_motor / R;
    double w_out   = w_motor   / R;

    // Vueltas reales en salida
    double vueltas_out_real = ((double)pos_c_m / (double)PPR) / (double)R;

    // Descomposición simétrica: vueltas enteras (trunc hacia 0) + sobrante firmado
    long   vueltas_out = (long)vueltas_out_real;                   //  2.14 -> 2 | -2.14 -> -2
    double rem_signed  = vueltas_out_real - (double)vueltas_out;   // (-1,1) con mismo signo

    // Correcciones por borde numérico
    if (rem_signed >= 1.0) { rem_signed -= 1.0; vueltas_out += 1; }
    if (rem_signed <= -1.0){ rem_signed += 1.0; vueltas_out -= 1; }
    if (fabs(rem_signed) < 1e-12) rem_signed = 0.0;  // evitar -0.0

    // Ángulo sobrante firmado
    double ang_rad_signed = rem_signed * TWO_PI;   // (-2π..2π)
    double ang_deg_signed = rem_signed * 360.0;    // (-360..360)

    // Aviso de vueltas completas
    long dvueltas_out = vueltas_out - vueltas_prev_out;
    if (dvueltas_out != 0) {
      long k = labs(dvueltas_out);
      if (k == 1) Serial.print(F("1 vuelta "));
      else        { Serial.print(k); Serial.print(F(" vueltas ")); }
      Serial.print(dvueltas_out > 0 ? F("(+) ") : F("(-) "));
      double theta_total_deg = (double)vueltas_out * 360.0;
      double theta_total_rad = (double)vueltas_out * TWO_PI;
      Serial.print(F("| θ_total(salida) = "));
      Serial.print(theta_total_deg, 2); Serial.print(F(" deg, "));
      Serial.print(theta_total_rad, 4); Serial.print(F(" rad"));
      Serial.print(F("  | vueltas_totales(salida) = "));
      Serial.println(vueltas_out);
      vueltas_prev_out = vueltas_out;
    }

    // Línea de estado
    Serial.print(F("v_out = "));
    Serial.print(rpm_out, 2);  Serial.print(F(" rpm, "));
    Serial.print(w_out, 3);    Serial.print(F(" rad/s  |  "));
    Serial.print(F("θ_out_signed = "));
    Serial.print(ang_deg_signed, 2); Serial.print(F(" deg, "));
    Serial.print(ang_rad_signed, 4); Serial.print(F(" rad  |  "));
    Serial.print(F("vueltas_out = "));
    Serial.println(vueltas_out);
  }
}
