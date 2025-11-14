// 1) Inicialización (llamar una vez en setup)
void EM_begin();

// 2) Control de motores (porcentaje de -100 a 100)
void EM_controlMotor1(float percentage);  // Motor 1 (AS5600)
void EM_controlMotor2(float percentage);  // Motor 2 (encoder incremental)

// 3) Lectura + impresión por Serial
void EM_printEncoder1();  // Imprime: M1,angle_deg,vel_deg_s,raw
void EM_printEncoder2();  // Imprime: M2,angle_deg,vel_deg_s,counts
