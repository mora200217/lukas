import serial
import time

# ⚠ Cambia esto al puerto real de tu ESP32
PORT = "/dev/ttyUSB0"
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)  # pequeña pausa para que el ESP32 arranque

# Estado del último comando
global last_motor, last_pwm_pct

last_motor = None       # 1 o 2
last_pwm_pct = 0.0      # en %

def send_pwm(motor: int, pwm_pct: float):
    """
    motor: 1 o 2
    pwm_pct: porcentaje de -100.0 a 100.0
    """
    global last_motor, last_pwm_pct

    if motor == 1:
        motor_id = 0xFA
    elif motor == 2:
        motor_id = 0xFB
    else:
        raise ValueError("motor debe ser 1 o 2")

    # saturar
    if pwm_pct > 100.0:
        pwm_pct = 100.0
    if pwm_pct < -100.0:
        pwm_pct = -100.0

    # int16 en centésimas de %
    pwm_raw = int(pwm_pct * 100.0)  # e.g. 25.00% -> 2500

    # Pasar int16 firmado a dos bytes (big-endian)
    if pwm_raw < 0:
        pwm_raw &= 0xFFFF  # equivalente a (1<<16) + pwm_raw

    pwm_hi = (pwm_raw >> 8) & 0xFF
    pwm_lo = pwm_raw & 0xFF

    frame = bytes([0xFF, motor_id, pwm_hi, pwm_lo])
    ser.write(frame)

    # Actualizar estado del último comando
    last_motor = motor
    last_pwm_pct = pwm_pct

    print(f"Enviado motor {motor}: {pwm_pct:.2f}%  (raw={pwm_raw}, hi=0x{pwm_hi:02X}, lo=0x{pwm_lo:02X})")


if __name__ == "__main__":
    try:
        print("Comandos:")
        print("  '1 25'  -> motor 1 a 25%")
        print("  '2 -40' -> motor 2 a -40%")
        print("  'd'     -> invertir sentido del ÚLTIMO motor comandado")
        print("  'm'     -> detener ambos motores (PWM = 0%)")
        print("  'q'     -> salir\n")

        while True:
            txt = input(">> ")

            cmd = txt.strip().lower()

            # Salir
            if cmd == 'q':
                break

            # Detener ambos motores
            if cmd == 'm':
                print("Deteniendo ambos motores (0%)...")
                send_pwm(1, 0.0)
                send_pwm(2, 0.0)
                continue

            # Invertir sentido del último motor
            if cmd == 'd':
                
                if last_motor is None:
                    print("Aún no se ha enviado ningún comando de PWM.")
                    continue

                new_pwm = -last_pwm_pct
                print(f"Invirtiendo sentido del motor {last_motor}: {last_pwm_pct:.2f}% -> {new_pwm:.2f}%")
                send_pwm(last_motor, new_pwm)
                continue

            # Comandos tipo: "<motor> <pwm_pct>"
            parts = txt.split()
            if len(parts) != 2:
                print("Formato inválido. Usa: <motor> <pwm_pct>, o 'd', 'm', 'q'")
                continue

            try:
                motor = int(parts[0])
                pwm = float(parts[1])
            except ValueError:
                print("Formato inválido. Usa: <motor> <pwm_pct>, o 'd', 'm', 'q'")
                continue

            send_pwm(motor, pwm)

    except KeyboardInterrupt:
        print("\n[INFO] Cancelado por el usuario.")
    finally:
        ser.close()
        print("[INFO] Puerto serie cerrado.")