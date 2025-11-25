# ...existing code...
import time, threading, queue

img_stuts = 0         # 摄像头下发指令编码
zhilin   = 0          # 摄像头下发16进制指令信息

# 轨迹执行队列（接收线程解析到完整轨迹后放入）
traj_queue = queue.Queue()

# 可配置：默认 Z 与姿态（根据机械臂实际需求调整）
# 绘画初始点P7
DEFAULT_Z = 0.0
DEFAULT_RXRYRZ = (0.0, 0.0, 0.0)
MOVE_V = 100

# 位姿点格式： {"pose":[x, y, z, rx, ry, rz]}
# my_point_2 = {"pose":[245, 110, 250, 180, 0, 0]}
# MovL(my_point_2)


def execute_traj_worker():
    """后台线程：从队列取出轨迹并依次运动到每个点，保持可中断/继续接收"""
    while True:
        pts = traj_queue.get()  # blocking
        if pts is None:
            break
        try:
            for p in pts:
                # p 可能是 [x,y] 或 [x,y,z]
                try:
                    x = float(p[0])
                    y = float(p[1])
                    z = float(p[2]) if len(p) >= 3 else DEFAULT_Z
                except Exception:
                    # 跳过非法点
                    continue
                # 构造位姿，根据你机器人接口调整格式
                P_target = [x, y, z, DEFAULT_RXRYRZ[0], DEFAULT_RXRYRZ[1], DEFAULT_RXRYRZ[2]]
                # 发送运动指令（MovJ/MovL 根据需求）
                try:
                    MovJ(P_target, {"user": 0, "v": MOVE_V})
                except Exception:
                    # 若 MovJ 不可用或抛错，可做降级或重试
                    pass
                # 允许短暂让步，避免完全阻塞（也能让其它线程运行）
                time.sleep(0.01)
        finally:
            traj_queue.task_done()

# 启动执行线程
worker_thread = threading.Thread(target=execute_traj_worker, daemon=True)
worker_thread.start()

def receiveThread(socket1):
    global img_stuts, zhilin
    buf = ""  # 文本缓冲，可能包含多行
    # 状态机用于解析一条轨迹
    in_traj = False
    expect_n = 0
    collected = []
    while True:
        len_js = 0
        recBuf = 0
        err, data = TCPRead(socket1)      # 假设 TCPRead 非阻塞式或短阻塞
        if err != 0 or not data:
            time.sleep(0.01)
            continue
        try:
            chunk = data.decode('utf-8', errors='ignore')
        except Exception:
            chunk = str(data)
        buf += chunk
        # 按行处理
        while '\n' in buf:
            line, buf = buf.split('\n', 1)
            line = line.strip()
            if not line:
                continue
            # 保留原有命令处理
            if 'Initialize' in line and img_stuts == 0:
                img_stuts = 1
                TCPWrite(socket1, "yunxing")
                continue
            if line.startswith('biao') and img_stuts == 0 and len(line) >= 16:
                try:
                    x_zb = int(line[4:8])
                    y_zb = int(line[8:12])
                    z_zb = int(line[12:16])
                except Exception:
                    continue
                img_stuts = 2
                if x_zb > 0:
                    print(x_zb)
                if z_zb < 0:
                    print(y_zb, z_zb)
                TCPWrite(socket1, "yunxing")
                continue

            # 轨迹协议解析（START/点行/END）
            parts = line.split()
            if parts and parts[0] == 'START':
                # 格式：START <idx> <n>
                in_traj = True
                collected = []
                expect_n = int(parts[2]) if len(parts) >= 3 else 0
                # 可回执确认开始接收
                TCPWrite(socket1, "START_ACK")
                continue
            if in_traj:
                # 检查是否 END 行
                if parts and parts[0] == 'END':
                    # 完整轨迹接收完毕（也可核对 idx）
                    in_traj = False
                    # 若实际收到的点数少于期望也可以处理或丢弃
                    # 把 collected 放入执行队列（后台线程执行）
                    if len(collected) > 0:
                        traj_queue.put(collected)
                        TCPWrite(socket1, "TRAJ_RECEIVED")
                    else:
                        TCPWrite(socket1, "TRAJ_EMPTY")
                    collected = []
                    expect_n = 0
                    continue
                # 否则当作点行，格式 x,y 或 x,y,z
                try:
                    coords = [c.strip() for c in line.split(',') if c.strip()!='']
                    if len(coords) >= 2:
                        # 转换为数值列表保留原格式
                        pt = [float(coords[0]), float(coords[1])]
                        if len(coords) >= 3:
                            pt.append(float(coords[2]))
                        collected.append(pt)
                except Exception:
                    # 忽略无法解析的点
                    pass
                # 可选择在收到期望点数后自动结束（如果发送端没有END）
                if expect_n > 0 and len(collected) >= expect_n:
                    in_traj = False
                    traj_queue.put(collected)
                    TCPWrite(socket1, "TRAJ_RECEIVED")
                    collected = []
                    expect_n = 0
                continue
