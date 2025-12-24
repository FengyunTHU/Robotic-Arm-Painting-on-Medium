"""
- 在maix摄像头上用串口接收文件，并解析json文件后发送给机械臂
- serial0与机械臂通讯、serial1与电脑通讯
"""
from maix import camera, display, pinmap, uart, app, time, gpio, err
import threading
import os
import json
import subprocess

stuts0  = ""
stuts1  = ""

stuts_err1 = ""
stuts_ok1 = ""
stutsoks = ""
stuts_send1TO0 = ""
stuts_hassend0 = False
stuts_05 = 0
file_path = ""
arm_name = ""

### 控制抽屉开关的status
status_C = 0 # 0关1开
status_D = 0 # 0停1动

#摄像头初始化
cam = camera.Camera(1024, 960)
disp = display.Display()

# 串口初始化
pinmap.set_pin_function("A29", "UART2_RX")
pinmap.set_pin_function("A28", "UART2_TX")
device = "/dev/ttyS0"
serial0 = uart.UART(device, 115200)
device = "/dev/ttyS2"
serial1 = uart.UART(device, 115200)

## GPIO初始化
### 18|->启动终止。1启动、0终止
### 19|->正转反转。1正转、0反转
pin_name = "A18"
gpio_name = "GPIOA18"
pin_name2 = "A19"
gpio_name2 = "GPIOA19"
## 设置A18为GPIOA18
err.check_raise(pinmap.set_pin_function(pin_name, gpio_name), "set pin failed")# MaixCAM的引脚是3.3V耐受，请勿输入5V电压。
err.check_raise(pinmap.set_pin_function(pin_name2, gpio_name2), "set pin failed")
servo = gpio.GPIO(gpio_name, gpio.Mode.OUT) # 控制启动终止
servo2 = gpio.GPIO(gpio_name2, gpio.Mode.OUT) # 控制正反

listening_event = threading.Event()
listening_event.set()

def drawerOUT(timex=7.0):
    """抽屉出"""
    global servo, servo2
    time.sleep(1.0)
    servo.value(0)
    servo2.value(1)
    servo.value(1)
    time.sleep(timex)
    servo.value(0)
    time.sleep(1.0)

def drawerIN(timex=7.0):
    """抽屉进"""
    global servo, servo2
    time.sleep(1.0)
    servo.value(0)
    servo2.value(0)
    servo.value(1)
    time.sleep(timex)
    servo.value(0)
    time.sleep(1.0)


# ...existing code...
def parse_points_from_payload(payload: bytes, max_lens = False, max_points: int = 10):
    """
    解析 payload（bytes）并返回 (paths, formatted_list)
    - 期望 JSON 结构： [ { ..., "points": [ [x,y], ... ] }, ... ]
    - 返回：
        paths: list of points-list，每个元素为一条轨迹的完整点列表（[[x,y],...], ...]）
        formatted_list: 每条轨迹对应的字符串 "x1,y1;x2,y2;..."（如果传入 max_points 为 None，则每条轨迹使用其全部点）
    """
    try:
        j = json.loads(payload.decode('utf-8'))
    except Exception:
        return None, None, None

    if not isinstance(j, list) or len(j) == 0:
        return None, None, None

    paths = []
    formatted = []
    colors:list = []
    for entry in j:
        if not (isinstance(entry, dict) and 'points' in entry):
            continue
        pts = entry['points']
        color = entry["attrs"]["fill"]
        if not isinstance(pts, list) or len(pts) == 0:
            paths.append([])
            formatted.append(None)
            continue
        # 存储完整点列
        paths.append(pts)
        colors.append(color)
        # 决定本条轨迹的采样上限->注意这里还是发送了完整的轨迹
        if max_lens:
            use_n = len(pts)
        else:
            use_n = min(max_points, len(pts))
        first = pts[:use_n]
        try:
            s = ';'.join("{:.3f},{:.3f}".format(float(p[0]), float(p[1])) for p in first)
        except Exception:
            try:
                s = ';'.join("{},{}".format(p[0], p[1]) for p in first)
            except Exception:
                s = None
        formatted.append(s)
    if len(paths) == 0:
        return None, None, None
    return paths, colors, formatted


def _parse_header_line(line: bytes):
    """
    期望格式: b'JSONBEGIN len=12345 chk=ab'
    返回 (length:int, chk:int) 或 None
    """
    try:
        s = line.decode('utf-8', errors='ignore').strip()
        if not s.startswith("JSONBEGIN"):
            return None
        parts = s.split()
        length = None
        chk = None
        for p in parts[1:]:
            if p.startswith("len="):
                length = int(p[4:])
            elif p.startswith("chk="):
                chk = int(p[4:], 16) if p[4:].startswith("0x") else int(p[4:])
        if length is None or chk is None:
            return None
        return length, chk
    except Exception:
        return None
    
def re_uart_file(serial):
    """
    串口接收线程（支持接收 header + 固定长度 payload，或处理 ARMSEND 命令）
    协议：
        - JSONBEGIN: 发送端先发一行 header 以换行结束：
            JSONBEGIN len=<bytes> chk=<sum8>\n
            随后发送 exactly <bytes> 的原始 JSON 二进制流（UTF-8）
            接收端校验后回 "OK\n" 或 "ERR\n"
        - ARMSEND: ARMSEND=<NAME>\n，设置 stuts_05 = 1
    """
    global stuts0, stuts1, serial0, serial1, stuts_ok1, stuts_err1, stuts_send1TO0, file_path, stuts_05, arm_name
    buf = bytearray()
    while True:
        data = serial.read()
        if not data:
            continue  # 移除延时，设置为常开无延时
        # serial.read() 可能已经是 bytes
        if isinstance(data, str):
            chunk = data.encode('utf-8', errors='ignore')
        else:
            chunk = data
        buf.extend(chunk)

        # 尝试找到 header 行（以第一个换行符为界）
        while True:
            nlpos = buf.find(b'\n')
            if nlpos == -1:
                break
            line = bytes(buf[:nlpos])  # 包含行（不含换行）
            buf = buf[nlpos+1:]
            line_str = line.decode('utf-8', errors='ignore').strip()
            if line_str.startswith("JSONBEGIN"):
                parsed = _parse_header_line(line)
                if not parsed:
                    # 非法 header，继续查找下一个换行（丢弃此行）
                    continue
                expected_len, expected_chk = parsed
                try:
                    serial.write_str(f"HEADER RECEIVED len={expected_len} chk=0x{expected_chk:02X}\n".encode("utf-8"))
                except Exception:
                    pass
                # 等待 payload 完整到达
                start_time = time.time()
                timeout = max(5, expected_len / 20000)  # 粗略超时（秒），可调
                while len(buf) < expected_len and (time.time() - start_time) < timeout:
                    more = serial.read()
                    if more:
                        if isinstance(more, str):
                            buf.extend(more.encode('utf-8', errors='ignore'))
                        else:
                            buf.extend(more)
                    else:
                        continue  # 无延时
                if len(buf) < expected_len:
                    # 超时或不完整，告诉发送端出错（接收端可根据需要选择不回复以触发重传）
                    try:
                        stuts_err1 = "TIMEOUT\n"
                    except Exception:
                        try:
                            stuts_err1 = "TIMEOUT\n"
                        except Exception:
                            pass
                    # 清空缓冲避免永久阻塞（或选择保留 buf 以重试）
                    buf = bytearray()
                    break  # 等待下一 header
                # 取得 payload
                payload = bytes(buf[:expected_len])
                # 移除已消费的 payload（保留后续可能连续的报文）
                buf = buf[expected_len:]
                # 校验 sum8
                chk = sum(payload) & 0xFF
                if chk != (expected_chk & 0xFF):
                    try:
                        stuts_err1 = "CHK_ERR\n"
                    except Exception:
                        try:
                            stuts_err1 = "CHK_ERR\n"
                        except Exception:
                            pass
                    # 丢弃此 payload，继续
                    continue
                # 校验通过 -> 保存文件（根据需要选择路径）
                try:
                    # 尝试保存到 /sd 或当前目录
                    fname = "/root/JSONS/received.json" if os.path.isdir("/root/JSONS") else "received.json"
                    file_path = fname
                    # 若文件较大可分块写入
                    with open(fname, "wb") as fw:
                        fw.write(payload)
                    # 成功回复
                    try:
                        stuts_ok1 = "OK "+fname+"\n"
                        stuts_05 = 1
                    except Exception:
                        try:
                            stuts_ok1 = "OK "+fname+"\n"
                            stuts_05 = 1
                        except Exception:
                            pass
                    
                except Exception as e:
                    # 保存失败
                    try:
                        stuts_err1 = "FAIL\n"
                    except Exception:
                        try:
                            stuts_err1 = "FAIL\n"
                        except Exception:
                            pass
                # 继续循环，支持缓冲中还有下一个 header/payload
            # end inner while
            elif line_str.startswith("ARMSEND="):
                arm_name = line_str[8:].strip()
                stuts_05 = 1
                try:
                    serial.write_str(b"ARMSEND_ACK\n")
                except Exception:
                    pass
                continue

def send_paths_via_serial0(paths, colors=None, *,
                            start_tag="START",
                            end_tag="END",
                            point_fmt="{:.3f},{:.3f}",
                            per_point_delay=0.005,
                            inter_path_delay=0.05):
    """
    通过 serial0 依次发送每条轨迹（paths: list of point-list）。
    每条轨迹发送格式：
        START <idx> <n>\n
        COLOR <color>\n
        x1,y1\n
        x2,y2\n
        ...
        END <idx>\n

    参数：
        - paths: [[ [x,y], ... ], ...]
        - point_fmt: 单点格式化字符串，两个占位符对应 x,y
        - per_point_delay: 每发送一点后的短延时（秒）
        - inter_path_delay: 路径间延时（秒）
    返回 True 表示发送完成（不做 ACK 验证）
    """
    global serial0
    if serial0 is None:
        return False
    try:
        for idx, pts in enumerate(paths):
            if not isinstance(pts, list):
                continue
            n = len(pts)
            color = colors[idx] if colors and idx < len(colors) else "default"
            # 发送 START 行
            try:
                serial0.write_str(f"{start_tag} {idx} {n}\n".encode("utf-8"))
            except Exception:
                try:
                    serial0.write_str(f"{start_tag} {idx} {n}\n")
                except Exception:
                    pass
            ## 发送color 行
            try:
                serial0.write_str(f"COLOR {color}\n".encode("utf-8"))
            except Exception:
                try:
                    serial0.write_str(f"COLOR {color}\n")
                except Exception:
                    pass
            # 逐点发送
            for p in pts:
                try:
                    x = float(p[0]); y = float(p[1])
                    line = point_fmt.format(x, y) + "\n"
                except Exception:
                    # 非数值则直接 str
                    line = f"{p[0]},{p[1]}\n"
                try:
                    serial0.write_str(line.encode("utf-8"))
                except Exception:
                    try:
                        serial0.write_str(line)
                    except Exception:
                        pass
                time.sleep(per_point_delay)
            # 发送 END 行
            try:
                serial0.write_str(f"{end_tag} {idx}\n".encode("utf-8"))
            except Exception:
                try:
                    serial0.write_str(f"{end_tag} {idx}\n")
                except Exception:
                    pass
            time.sleep(inter_path_delay)
        return True
    except Exception:
        return False


### serial0监视机械臂开关抽屉信息
def handle_serial0(serial):
    global status_C, status_D
    while True:
        if not listening_event.is_set():
            listening_event.wait()
        data = serial.read()
        if data:
            if isinstance(data, bytes):
                msg = data.decode('utf-8', errors='ignore').strip()
            else:
                msg = str(data).strip()
            if msg == "OPEN":
                status_C = 1
                status_D = 1
            elif msg == "CLOSE":
                status_C = 0
                status_D = 1
            else:
                status_C = 0
                status_D = 0


uart0_thread = threading.Thread(target=handle_serial0, args = (serial0,))
uart0_thread.daemon = True
uart0_thread.start()

uart1_thread = threading.Thread(target=re_uart_file, args = (serial1,))
uart1_thread.daemon = True
uart1_thread.start()


while not app.need_exit():
# while True:
    img = cam.read()
    disp.show(img)
    time.sleep(2)
    # u_id   = "biao"
    # x_zb   = int(30.5)
    # y_zb   = int(31)
    # z_zb   = int(-20.12345131)
    # u_data = f"{u_id}{x_zb:04d}{y_zb:04d}{z_zb:04d}".encode("utf-8")
    # serial0.write_str(u_data)
    # time.sleep(2)
    # serial0.write_str("Initialize".encode("utf-8"))
    
    if stuts_err1 != "" :
        serial1.write_str(stuts_err1.encode("utf-8"))
        stuts_err1 = ""

    if stuts_ok1 != "":
        serial1.write_str(stuts_ok1.encode("utf-8"))
        # 执行发送
        if not stuts_hassend0 and stuts_05:
            print("001", arm_name, file_path)
            listening_event.clear()
            # 读取并解析 JSON，取每条轨迹完整点列
            with open(file_path, "rb") as fr:
                payload = fr.read()
            paths, colors, _ = parse_points_from_payload(payload, max_lens=False, max_points=10)
            if paths:
                ok_send = send_paths_via_serial0(paths,colors)
                if ok_send:
                    serial1.write_str(b"SENT_TO_SERIAL0\n")
                    stuts_hassend0 = True
                else:
                    serial1.write_str(b"SEND_FAILED\n")
            else:
                serial1.write_str(b"NO_PATHS_FOUND\n")
            stuts_hassend0 = True
            listening_event.set()
        stuts_ok1 = ""

    if stuts_send1TO0 != "" :
        serial0.write_str(stuts_send1TO0.encode("utf-8"))
        stuts_send1TO0 = ""

    ## 抽屉操作
    if status_D:
        # 运动
        if status_C:
            # 开启
            drawerOUT()
        else:
            # 关闭
            drawerIN()
        status_D = 0
        status_C = 0


print("自动退出")