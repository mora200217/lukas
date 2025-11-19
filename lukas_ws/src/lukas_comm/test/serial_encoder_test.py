import serial
import struct
import time

# ================
# CONFIG
# ================
PORT = "/dev/tty.usbserial-11230"     # change to COM3 on Windows or /dev/cu.usbserial on macOS
BAUD = 9600

# ================
# FRAME FORMAT
# ================
FRAME_SIZE = 4

def build_frame(confirm, id_val, data, ack):
    """
    Build the 4-byte frame:
    Byte0 = [confirm(4 bits) | id(4 bits)]
    """
    b0 = ((confirm & 0x0F) << 4) | (id_val & 0x0F)
    return struct.pack(">BBHB", b0, (data >> 8) & 0xFF, data & 0xFF, ack)


def parse_frame(frame_bytes):
    """Parse 4-byte frame into dict."""
    b0, b1, b2, ack = struct.unpack(">BBBB", frame_bytes)
    confirm = (b0 >> 4) & 0x0F
    id_val  = b0 & 0x0F
    data    = (b1 << 8) | b2

    return {
        "confirm": confirm,
        "id": id_val,
        "data": data,
        "ack": ack
    }


# ================
# MAIN LOOP
# ================
def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.1)

    print("Connected to", PORT)

    while True:
        # -------------------------
        # SEND COMMAND TO ESP32
        # example: command for motor 1 = 2000 (16-bit)
        # -------------------------
        cmd_frame = build_frame(
            confirm=0xA,
            id_val=0x1,      # motor 1
            data=2000,
            ack=0x55
        )
        ser.write(cmd_frame)

        # -------------------------
        # READ RESPONSE (ENCODER)
        # -------------------------
        if ser.in_waiting >= FRAME_SIZE:
            rx = ser.read(FRAME_SIZE)
            decoded = parse_frame(rx)
            print("RX:", decoded)

        time.sleep(0.05)


if __name__ == "__main__":
    main()
