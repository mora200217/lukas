

import serial
import struct

PORT = "/dev/tty.usbmodem1301"        # Cambia a COM3 en Windows o /dev/cu.usbmodemXXXX en macOS
BAUD = 115200
FRAME_SIZE = 4

def parse_frame(frame_bytes):
    b0, b1, b2, ack = struct.unpack(">BBBB", frame_bytes)

    confirm = (b0 >> 4) & 0x0F
    id_val  = b0 & 0x0F
    angle   = (b1 << 8) | b2

    return {
        "confirm": confirm,
        "id": id_val,
        "angle": angle,
        "ack": ack
    }

def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    print("Listening on", PORT)

    while True:
        if ser.in_waiting >= FRAME_SIZE:
            data = ser.read(FRAME_SIZE)
            frame = parse_frame(data)

            print(f"ID {frame['id']:d}  →  angle: {frame['angle']:d}°")

if __name__ == "__main__":
    main()

