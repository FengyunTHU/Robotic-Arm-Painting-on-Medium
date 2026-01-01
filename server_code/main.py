from Points import process_svg as ps_svg
from Communication import ping_port as ping
from Communication import send_json as send_J
from Communication import interactive_session
import serial
import sys

if __name__ == "__main__":
    port = "COM8"
    baud = 115200
    x = int(input("1绘画, 2取点."))
    if x == 1:
        try:
            ser = serial.Serial(port, baud, timeout=0.1)
        except Exception as e:
            print(f"open serial failed: {e}")
            sys.exit(1)
        interactive_session(ser, port, baud)
    elif x == 2:
        SVG = r"f:\大四上作业文件\制造工程训练\项目代码\刘_x.svg"
        results = ps_svg(SVG, 102,sample_step=8.0)  # 已存在的函数，会返回采样结果列表