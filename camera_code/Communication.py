import threading
import time
import os
import json

# 模块级状态（由接收线程写入，主循环通过 pop_statuses 读取并清空）
stuts0 = ""
stuts1 = ""

def _parse_header_line(line: bytes):
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
    接收文件线程（与原 main.py 中逻辑保持一致）
    接收 header + 固定长度 payload，校验后保存并解析前10点，设置模块级 stuts0/stuts1，
    并通过接收的 serial 立即回送拼接字符串。
    """
    global stuts0, stuts1
    buf = bytearray()
    while True:
        data = serial.read()
        if not data:
            time.sleep(0.01)
            continue
        if isinstance(data, str):
            chunk = data.encode('utf-8', errors='ignore')
        else:
            chunk = data
        buf.extend(chunk)

        while True:
            nlpos = buf.find(b'\n')
            if nlpos == -1:
                break
            header_line = bytes(buf[:nlpos])
            parsed = _parse_header_line(header_line)
            buf = buf[nlpos+1:]
            if not parsed:
                continue
            expected_len, expected_chk = parsed
            # 可选回显 header 已接收
            try:
                serial.write_str(f"HEADER RECEIVED len={expected_len} chk=0x{expected_chk:02X}\n".encode("utf-8"))
            except Exception:
                pass

            start_time = time.time()
            timeout = max(5, expected_len / 20000)
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
                try:
                    serial.write_str("ERR\n".encode("utf-8"))
                except Exception:
                    pass
                buf = bytearray()
                break
            payload = bytes(buf[:expected_len])
            buf = buf[expected_len:]
            chk = sum(payload) & 0xFF
            if chk != (expected_chk & 0xFF):
                try:
                    serial.write_str("ERR\n".encode("utf-8"))
                except Exception:
                    pass
                continue
            # 保存文件
            try:
                fname = "/root/JSONS/received.json" if os.path.isdir("/root/JSONS") else "received.json"
                with open(fname, "wb") as fw:
                    fw.write(payload)
                # 简短回执
                try:
                    serial.write_str("O_S\n".encode("utf-8"))
                    serial.write_str(fname.encode("utf-8"))
                except Exception:
                    pass

                # 解析 JSON，按指定结构取 points
                try:
                    j = json.loads(payload.decode('utf-8'))
                except Exception:
                    j = None

                pts = None
                if isinstance(j, list) and len(j) > 0 and isinstance(j[0], dict) and 'points' in j[0]:
                    pts = j[0]['points']

                if pts and len(pts) > 0:
                    first10 = pts[:10]
                    try:
                        s = ';'.join("{:.3f},{:.3f}".format(float(p[0]), float(p[1])) for p in first10)
                    except Exception:
                        s = ';'.join("{},{}".format(p[0], p[1]) for p in first10)
                    # 设置模块状态并回送
                    if serial is not None:
                        try:
                            serial.write_str((s + "\n").encode("utf-8"))
                            serial.write_str("OK\n".encode("utf-8"))
                        except Exception:
                            try:
                                serial.write_str(s + "\n")
                                serial.write_str("OK\n")
                            except Exception:
                                pass
                    # 根据接收的 serial 判断是哪个口，尽量不依赖外部 serial 对象相等比较。
                    # 主程序会读取模块状态并发送到对应口；这里只把状态写入模块全局（覆盖）
                    # 无法准确区分 serial0/serial1 时，主程序应通过 start_threads 传入标识或直接让通信模块写回。
                    # 为简单起见，把信息同时写入两个状态变量（主循环再发到两个口）
                    stuts0 = s
                    stuts1 = s

            except Exception:
                try:
                    serial.write_str("ERR\n".encode("utf-8"))
                except Exception:
                    pass
            # 继续处理缓冲中的下一个报文
        # 内层 while 结束

def re_uart_echo(serial):
    """
    简单回显线程：把接收到的文本读出并保存到模块状态（测试用）
    """
    global stuts0, stuts1
    while True:
        data = serial.read()
        if not data:
            time.sleep(0.01)
            continue
        try:
            s = data.decode("utf-8", errors="ignore")
        except Exception:
            s = str(data)
        # 写入状态（供主循环读取）
        stuts0 = s
        stuts1 = s

def start_threads(serial0, serial1, use_echo=False):
    """
    启动两个接收线程（分别绑定到传入的 serial 对象）
    """
    t0 = threading.Thread(target=re_uart_file if not use_echo else re_uart_echo, args=(serial0,))
    t0.daemon = True
    t0.start()
    t1 = threading.Thread(target=re_uart_file if not use_echo else re_uart_echo, args=(serial1,))
    t1.daemon = True
    t1.start()

def pop_statuses():
    """
    返回 (s0, s1) 并清空模块内缓存
    """
    global stuts0, stuts1
    s0, s1 = stuts0, stuts1
    stuts0, stuts1 = "", ""
    return s0, s1