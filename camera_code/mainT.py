# ...existing code...
from maix import camera, display, pinmap, uart, app, time
import threading
import os
import json

# 引入通信模块
import Communication as comm

stuts0  = ""
stuts1  = ""

#摄像头初始化
cam = camera.Camera(1024, 960)
disp = display.Display()

# 串口初始化（保持不变）
pinmap.set_pin_function("A29", "UART2_RX")
pinmap.set_pin_function("A28", "UART2_TX")
device = "/dev/ttyS0"
serial0 = uart.UART(device, 115200)
device = "/dev/ttyS2"
serial1 = uart.UART(device, 115200)

# 启动通信线程（把 serial 对象传入）
comm.start_threads(serial0, serial1, use_echo=False)

# 主循环：从通信模块取出状态并发送到对应串口
while not app.need_exit():
    img = cam.read()
    disp.show(img)
    time.sleep(2)

    # 从通信模块获取并清空状态
    s0, s1 = comm.pop_statuses()
    if s0:
        try:
            serial0.write_str(f"uart0:{s0}")
        except Exception:
            try:
                serial0.write_str(s0)
            except Exception:
                pass
    if s1:
        try:
            serial1.write_str(f"uart1:{s1}")
        except Exception:
            try:
                serial1.write_str(s1)
            except Exception:
                pass
# ...existing code...