#!/usr/bin/env python3
"""
serial_listener.py

Lee y escribe puerto serie, lanza identificación con G1/G2 y guarda telemetría en ./Data.

Comandos por teclado (enter):
  g1 / G1   -> envía "G1" al ESP y comienza a grabar en Data/<timestamp>_G1.csv
  g2 / G2   -> envía "G2" al ESP y comienza a grabar en Data/<timestamp>_G2.csv
  stop      -> detiene la captura actual
  quit      -> cierra el programa
"""

import serial
import threading
import queue
import csv
import os
import time
from pathlib import Path
import sys

# ----------------- CONFIG -----------------
puerto = '/dev/tty.usbserial-130'        # cambia a tu puerto (COM5, COM9, /dev/ttyUSB0, ...)
baudios = 115200
data_folder = Path('Data')
data_folder.mkdir(parents=True, exist_ok=True)
# ------------------------------------------

cmd_q = queue.Queue()
stop_event = threading.Event()

def generar_nombre_archivo(base='esp_data', suf='G', folder=data_folder):
    ts = time.strftime("%Y%m%d_%H%M%S")
    name = f"{ts}_{base}_{suf}.csv"
    path = folder / name
    # asegurar unicidad (no realmente necesario por timestamp, pero por si acaso)
    i = 1
    while path.exists():
        path = folder / f"{ts}_{base}_{suf}_{i}.csv"
        i += 1
    return path

def teclado_thread():
    help_msg = "Comandos: g1 g2 stop quit"
    print(help_msg)
    while not stop_event.is_set():
        try:
            linea = input().strip()
        except EOFError:
            cmd_q.put('QUIT')
            stop_event.set()
            break
        if not linea:
            continue
        cmd = linea.lower()
        if cmd in ('g1', 'g2'):
            cmd_q.put(cmd.upper())
        elif cmd == 'stop':
            cmd_q.put('STOP')
        elif cmd in ('quit', 'exit', 'q'):
            cmd_q.put('QUIT')
            stop_event.set()
            break
        else:
            print("Comando no reconocido. " + help_msg)

def parse_csv_style(line:str):
    # Espera: t_ms,s1,U1,s2,U2   (todo numérico)
    parts = [p.strip() for p in line.split(',')]
    if len(parts) >= 5:
        try:
            t_ms = float(parts[0])
            s1 = float(parts[1])
            u1 = float(parts[2])
            s2 = float(parts[3])
            u2 = float(parts[4])
            return (t_ms/1000.0, s1, u1, s2, u2)
        except Exception:
            return None
    return None

def parse_kv_style(line:str):
    # Ejemplo: "t=1234,U=10,tempF=25.3,t_s=1.234"
    if '=' not in line:
        return None
    try:
        parts = [p.strip() for p in line.split(',')]
        d = {}
        for p in parts:
            if '=' in p:
                k,v = p.split('=',1)
                d[k.strip()] = v.strip()
        # intentamos mapear a columnas esperadas (fallback None)
        t = float(d.get('t'))/1000.0 if ('t' in d) else (float(d.get('t_ms'))/1000.0 if 't_ms' in d else None)
        s1 = float(d.get('s1')) if 's1' in d else (float(d.get('Sensor_M1_deg')) if 'Sensor_M1_deg' in d else None)
        u1 = float(d.get('U')) if 'U' in d else (float(d.get('U1')) if 'U1' in d else None)
        s2 = float(d.get('s2')) if 's2' in d else (float(d.get('Sensor_M2_deg')) if 'Sensor_M2_deg' in d else None)
        u2 = float(d.get('t_s')) if 't_s' in d else (float(d.get('U2')) if 'U2' in d else None)
        # si al menos tenemos tiempo y una medida
        if t is None:
            return None
        return (t, s1, u1, s2, u2)
    except Exception:
        return None

def parse_line(line:str):
    # intenta formatos conocidos
    r = parse_csv_style(line)
    if r is not None:
        return r
    r = parse_kv_style(line)
    return r

def main():
    # verificar pyserial
    try:
        import serial as _s
    except Exception as e:
        print("Instala pyserial: python -m pip install pyserial")
        sys.exit(1)

    # abrir puerto
    try:
        ser = serial.Serial(puerto, baudios, timeout=0.5)
    except Exception as e:
        print(f"[ERROR] no se pudo abrir {puerto}: {e}")
        sys.exit(1)

    print(f"[OK] Puerto abierto {puerto} @ {baudios} baudios")

    # lanzar hilo teclado
    t = threading.Thread(target=teclado_thread, daemon=True)
    t.start()

    capturing = False
    csvf = None
    writer = None
    samples = 0
    current_cmd = None

    try:
        while not stop_event.is_set():
            # comprobar comandos pendientes
            try:
                cmd = cmd_q.get_nowait()
            except queue.Empty:
                cmd = None

            if cmd:
                if cmd in ('G1','G2'):
                    # enviar al ESP
                    try:
                        ser.write((cmd + '\n').encode('utf-8'))
                        ser.flush()
                        print(f"[TX] {cmd} enviado")
                    except Exception as e:
                        print("[ERROR] enviando comando:", e)
                    # iniciar captura
                    if capturing:
                        # cerrar captura anterior antes de iniciar nueva
                        try:
                            csvf.flush()
                            csvf.close()
                        except:
                            pass
                        capturing = False
                    path = generar_nombre_archivo(base='esp', suf=cmd)
                    csvf = open(path, 'w', newline='')
                    writer = csv.writer(csvf)
                    writer.writerow(['Time_s','Sensor_M1_deg','PWM_M1_pct','Sensor_M2_deg','PWM_M2_pct'])
                    csvf.flush()
                    capturing = True
                    samples = 0
                    current_cmd = cmd
                    print(f"[INFO] Capturando en: {path}")
                elif cmd == 'STOP':
                    if capturing:
                        print("[INFO] Stop solicitado por usuario")
                        try:
                            csvf.flush(); csvf.close()
                        except:
                            pass
                        capturing = False
                        current_cmd = None
                    else:
                        print("[INFO] No se estaba capturando")
                elif cmd == 'QUIT':
                    print("[INFO] Quit solicitado")
                    stop_event.set()
                    break

            # leer linea serie (espera hasta timeout)
            try:
                raw = ser.readline()
            except Exception as e:
                print("[ERROR] leyendo serie:", e)
                break
            if not raw:
                continue
            try:
                line = raw.decode('utf-8', errors='ignore').strip()
            except Exception:
                continue
            if not line:
                continue

            # mostrar la línea tal cual (útil para depuración)
            print(line)

            # si estamos capturando, intentar parsear y escribir
            if capturing and writer is not None:
                parsed = parse_line(line)
                if parsed is not None:
                    # parsed = (Time_s, s1, u1, s2, u2) cualquiera puede ser None
                    writer.writerow(parsed)
                    samples += 1
                    # flush periódico
                    if samples % 200 == 0:
                        csvf.flush()
                        print(f"[INFO] {samples} muestras guardadas")
                # detectar fin por mensaje del ESP
                if ("End ident M1" in line) or ("End ident M2" in line) or ("End ident" in line) or ("Endident" in line.replace(" ", "")):
                    print("[INFO] Mensaje de fin detectado por ESP. Cerrando archivo.")
                    try:
                        csvf.flush(); csvf.close()
                    except:
                        pass
                    capturing = False
                    current_cmd = None

    except KeyboardInterrupt:
        print("\n[INFO] Ctrl+C detectado. Saliendo...")
    finally:
        try:
            if capturing:
                csvf.flush(); csvf.close()
        except:
            pass
        try:
            ser.close()
        except:
            pass
        stop_event.set()
        print("Programa terminado.")

if __name__ == '__main__':
    main()