"""
- 在maix摄像头上用串口接收文件，并解析json文件后发送给机械臂
- serial0与机械臂通讯、serial1与电脑通讯
"""
from maix import camera, display, pinmap, uart, app, time
import threading
import os
import json

stuts0  = ""
stuts1  = ""  

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
    串口接收线程（支持接收 header + 固定长度 payload）
    协议：
        发送端先发一行 header 以换行结束：
        JSONBEGIN len=<bytes> chk=<sum8>\n
        随后发送 exactly <bytes> 的原始 JSON 二进制流（UTF-8）
        接收端校验后回 "OK\n" 或 "ERR\n"
    """
    global stuts0, stuts1, serial0, serial1
    buf = bytearray()
    while True:
        data = serial.read()
        if not data:
            time.sleep(0.01)
            continue
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
            header_line = bytes(buf[:nlpos])  # 包含 header（不含换行）
            parsed = _parse_header_line(header_line)
            # 移除 header 行（包含换行）
            buf = buf[nlpos+1:]
            if not parsed:
                # 非法 header，继续查找下一个换行（丢弃此行）
                continue
            expected_len, expected_chk = parsed
            serial.write_str(f"HEADER RECEIVED len={expected_len} chk=0x{expected_chk:02X}\n".encode("utf-8"))
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
                    time.sleep(0.01)
            if len(buf) < expected_len:
                # 超时或不完整，告诉发送端出错（接收端可根据需要选择不回复以触发重传）
                try:
                    serial.write_str("ERR\n".encode("utf-8"))
                except Exception:
                    try:
                        serial.write_str("ERR\n")
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
                    serial.write_str("ERR\n".encode("utf-8"))
                except Exception:
                    try:
                        serial.write_str("ERR\n")
                    except Exception:
                        pass
                # 丢弃此 payload，继续
                continue
            # 校验通过 -> 保存文件（根据需要选择路径）
            try:
                # 尝试保存到 /sd 或当前目录
                fname = "/root/JSONS/received.json" if os.path.isdir("/root/JSONS") else "received.json"
                # 若文件较大可分块写入
                with open(fname, "wb") as fw:
                    fw.write(payload)
                # 成功回复
                try:
                    serial.write_str("O_S\n".encode("utf-8"))
                    serial.write_str(fname.encode("utf-8"))
                except Exception:
                    try:
                        serial.write_str("O_S\n")
                        serial.write_str(fname)
                    except Exception:
                        pass
                
                ### 解析JSON做验证
                try:
                    j = json.loads(payload.decode('utf-8'))
                except Exception:
                    j = None

                pts = None
                if isinstance(j, list) and len(j) > 0 and isinstance(j[0], dict) and 'points' in j[0]:
                    pts = j[0]['points']

                if pts and len(pts) > 0:
                    first10 = pts[:10]
                    # 格式化为 "x1,y1;x2,y2;..."
                    try:
                        s = ';'.join("{:.3f},{:.3f}".format(float(p[0]), float(p[1])) for p in first10)
                    except Exception:
                        s = ';'.join("{},{}".format(p[0], p[1]) for p in first10)
                    # 赋值到对应 stuts 并通过该串口立即发送
                    if serial == serial0:
                        stuts0 = s
                    else:
                        stuts1 = s
                    try:
                        serial.write_str((s + "\n").encode("utf-8"))
                        serial.write_str("OK\n".encode("utf-8"))
                    except Exception:
                        try:
                            serial.write_str(s + "\n")
                            serial.write_str("OK\n")
                        except Exception:
                            pass

            except Exception as e:
                # 保存失败
                try:
                    serial.write_str("ERR\n".encode("utf-8"))
                except Exception:
                    try:
                        serial.write_str("ERR\n")
                    except Exception:
                        pass
            # 继续循环，支持缓冲中还有下一个 header/payload
        # end inner while

def re_uart(serial):
    """
    信号收发代码-TEST
    """
    global stuts0, stuts1, serial0, serial1
    while 1:
        # 串口  接收数据
        data = serial.read()
        data = data.decode("utf-8",errors="ignore")
        if data != "" and serial == serial0:  #串口0 赋值
        #   print(data)
            stuts0  = data
            data    = ""
        if data != "" and serial == serial1:  #串口1 赋值
            stuts1  = data
            data    = ""

uart0_thread = threading.Thread(target=re_uart_file, args = (serial0,))
uart0_thread.daemon = True
uart0_thread.start()

uart1_thread = threading.Thread(target=re_uart_file, args = (serial1,))
uart1_thread.daemon = True
uart1_thread.start()


while not app.need_exit():
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

    if stuts0 != "" :
        serial0.write_str(f"uart0:{stuts0}")
        stuts0 = ""

    if stuts1 != "" :
        serial1.write_str(f"uart1:{stuts1}") 
        stuts1 = ""


