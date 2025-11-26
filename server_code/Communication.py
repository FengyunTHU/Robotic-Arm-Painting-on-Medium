"""
- 电脑串口发送 - COM9 115200 波特率
- 串口号不确定需要调整
"""

import argparse
import serial
import time
import os
import sys
import threading

def calc_sum8(b: bytes) -> int:
    return sum(b) & 0xFF

def ping_port(port="COM9", baud=115200, message="PING\n", timeout=5.0) -> int:
    """
    - 代码的检测函数，用于通过串口发送简单的消息并等待回复。
    """
    try:
        ser = serial.Serial(port, baud, timeout=0.1)
    except Exception as e:
        print(f"open serial failed: {e}")
        return 1
    try:
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        ser.write(message.encode("utf-8"))
        ser.flush()
        deadline = time.time() + timeout
        resp = bytearray()
        while time.time() < deadline:
            chunk = ser.read(256)
            if chunk:
                resp.extend(chunk)
                # 如果收到换行就可以结束
                if b"\n" in resp:
                    break
            else:
                time.sleep(0.02)
        if resp:
            try:
                text = resp.decode("utf-8", errors="ignore").strip()
            except:
                text = str(resp)
            print("收到回复:", text)
            return 0
        else:
            print("超时未收到回复")
            return 2
    finally:
        try:
            ser.close()
        except:
            pass


def interactive_session(ser, prompt=">> "):
    """
    进入交互监听模式（无超时），直到命令行输入 exit 为止：
        - 后台线程持续打印串口收到的数据
        - 主线程读取命令行输入并发送到串口
        - 输入 'exit' (忽略大小写) 退出并返回
    """
    stop_event = threading.Event()

    def reader_loop():
        while not stop_event.is_set():
            try:
                data = ser.read(256)
            except Exception:
                data = b""
            if data:
                try:
                    print("<<", data.decode("utf-8", errors="ignore").rstrip())
                except Exception:
                    print("<<", data)
            else:
                time.sleep(0.05)

    rd_thread = threading.Thread(target=reader_loop, daemon=True)
    rd_thread.start()

    try:
        while True:
            try:
                cmd = input(prompt)
            except (EOFError, KeyboardInterrupt):
                cmd = "exit"
            if cmd is None:
                continue
            cmd_strip = cmd.strip()
            if cmd_strip.lower() == "exit":
                stop_event.set()
                rd_thread.join(timeout=1.0)
                try:
                    ser.write(b"EXIT")
                    ser.flush()
                except Exception:
                    pass
                print("已退出交互模式")
                return
            # 发送命令到串口（附加换行）
            outb = (cmd_strip).encode("utf-8")
            try:
                ser.write(outb)
                ser.flush()
            except Exception as e:
                print("发送失败:", e)
                stop_event.set()
                rd_thread.join(timeout=1.0)
                return
    finally:
        stop_event.set()
        rd_thread.join(timeout=1.0)


def send_json(port: str, baud: int, filepath: str, retries: int=3, resp_timeout: float=10.0, inter_chunk_delay: float=0.01) -> bool:
    """
    - 使用串口流式发送JSON文件，并进行系列检查操作
    """
    if not os.path.exists(filepath):
        print("file not found:", filepath)
        return False
    with open(filepath, "rb") as f:
        payload = f.read()
    length = len(payload)
    chk = calc_sum8(payload)
    # 用 hex 形式发送 chk，maix 端能解析 0x.. 或 十进制
    header = f"JSONBEGIN len={length} chk=0x{chk:02X}\n".encode("utf-8")
    try:
        ser = serial.Serial(port, baud, timeout=0.1)
    except Exception as e:
        print("open serial failed:", e)
        return False

    try:
        for attempt in range(1, retries+1):
            print(f"[{attempt}/{retries}] send header len={length} chk=0x{chk:02X}")
            ser.write(header)
            ser.flush()
            # 小块发送 payload（可按需调整 chunk）
            CHUNK = 4096
            off = 0
            while off < length:
                tosend = payload[off:off+CHUNK]
                ser.write(tosend)
                ser.flush()
                off += len(tosend)
                time.sleep(inter_chunk_delay)
            # 等待响应 OK/ERR
            deadline = time.time() + resp_timeout
            resp = b""
            while time.time() < deadline:
                r = ser.read(256) # 读取小块回复
                if r:
                    print("收到回复片段:", r)
                    resp += r
                    if b"OK" in resp:
                        print("收到 OK，发送成功")
                        # ser.close()
                        interactive_session(ser) # 监听
                        return True
                    if b"ERR" in resp:
                        print("收到 ERR，重试")
                        break
                else:
                    time.sleep(0.05)
            print("未收到 OK，等待或重试")
        print("发送失败，超出重试次数")
    finally:
        try:
            ser.close()
        except:
            pass
    return False

if __name__ == "__main__":
    p = argparse.ArgumentParser(description="Send JSON file to Maix via serial using JSONBEGIN protocol")
    p.add_argument("--port", "-p", required=True, help="COM port, e.g. COM3")
    p.add_argument("--baud", "-b", type=int, default=115200)
    p.add_argument("--file", "-f", required=True, help="JSON file to send")
    p.add_argument("--retries", "-r", type=int, default=3)
    p.add_argument("--timeout", "-t", type=float, default=10.0)
    args = p.parse_args()
    ok = send_json(args.port, args.baud, args.file, retries=args.retries, resp_timeout=args.timeout)
    sys.exit(0 if ok else 1)