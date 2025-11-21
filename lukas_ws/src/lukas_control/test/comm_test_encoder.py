import serial
import struct

PORT = "/dev/ttyUSB0"   # cámbialo a tu puerto
BAUD = 115200

def leer_tramas():
    ser = serial.Serial(PORT, BAUD, timeout=0.1)

    sync_found = False

    while True:
        b = ser.read(1)
        if not b:
            continue

        # ---- Buscar sincronización ----
        if not sync_found:
            if b[0] == 0xFF:
                sync_found = True
                buffer = bytearray()
            else:
                continue
        else:
            buffer.append(b[0])

            if len(buffer) == 4:
                # Ya tenemos los 4 bytes payload (2 encoders)
                enc1 = struct.unpack(">h", buffer[0:2])[0]   # int16 big-endian
                enc2 = struct.unpack(">h", buffer[2:4])[0]

                # Convertir a ángulo como en tu C++ (ya vienen en milésimas de grado)
                angle1 = enc1 / 1000.0
                angle2 = enc2 / 1000.0

                print(f"Ángulos → M1: {angle1:.3f}°,  M2: {angle2:.3f}°")

                sync_found = False  # buscar siguiente trama

if __name__ == "__main__":
    leer_tramas()
