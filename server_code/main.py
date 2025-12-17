from Points import process_svg as ps_svg
from Communication import ping_port as ping
from Communication import send_json as send_J
from Communication import interactive_session
import serial
import sys

if __name__ == "__main__":
    port = "COM9"
    baud = 115200
    try:
        ser = serial.Serial(port, baud, timeout=0.1)
    except Exception as e:
        print(f"open serial failed: {e}")
        sys.exit(1)
    interactive_session(ser, port, baud)