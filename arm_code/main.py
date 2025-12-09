# version: Python3
import time ,threading  # 导入库函数
import json
import queue
import csv ## 创建csv文件


## 在下面俩个文件夹中都保存了csv
PATH1:str = "/dobot/userdata/user_project/process/trajectory/"
PATH2:str = "/dobot/userdata/project/process/trajectory/"

## user:0, tool:0, ecokey:0.
HEAD:list = ['j1','j2','j3','j4','j5','j6','x','y','z','Rx','Ry','Rz','user','tool','ecoKey']


img_stuts = 0         # 摄像头下发指令编码
zhilin   = 0         # 摄像头下发16进制指令信息
stuts_add = False
Path_id = 0

# DEFAULT是P7

traj_queue = queue.Queue()

def execute_traj_worker(ori_angle, ori_pose):
    """
    后台线程：从 traj_queue 取出一条轨迹（点列表），依次移动到每个点以“绘制”轨迹。
    这里使用 MovJ 作示例，如需直线插补请用 MovL/相应 API。
    点格式支持 [x,y] 或 [x,y,z]，若无 z 使用 0.0。
    """
    DEFAULT_XYZ = ori_pose["pose"][0:3]
    DEFAULT_RXRYRZ = ori_pose["pose"][3:6]
    assert list(DEFAULT_XYZ) == [276.0, 123.0, 227.0]
    assert list(DEFAULT_RXRYRZ) == [-180.0, 0.0, 0.0]
    DEFAULT_JOINT = ori_angle["joint"] # [j1,j2,j3,j4,j5,j6] of P7

    MOVE_V = 50
    while True:
        pts = traj_queue.get()  # 阻塞直到有轨迹
        if pts is None:
            traj_queue.task_done()
            break
        try:
            Path_id+=1
            file1 = PATH1 + f"x{Path_id}.csv"
            file2 = PATH2 + f"x{Path_id}.csv"
            # 依次运动到每个点
            with open(file1, 'w', newline='') as f1, open(file2, 'w', newline='') as f2:
                writer1 = csv.writer(f1)
                writer2 = csv.writer(f2)
                # 写入表头
                writer1.writerow(HEAD)
                writer2.writerow(HEAD)
                for p in pts:
                    x_ = float(p[0]); y_ = float(p[1])
                    z = float(p[2]) if (isinstance(p, (list,tuple)) and len(p) >= 3) else DEFAULT_XYZ[2]
                    # 构造行数据：关节角度 + 位置 + 姿态 + user/tool/ecoKey
                    # 反解算joint
                    errid, jointpoint = InverseKin({"pose":[x_+DEFAULT_XYZ[0], y_+DEFAULT_XYZ[1], z]+list[DEFAULT_RXRYRZ]})
                    if errid == 0:
                        row = jointpoint["joint"] + [x_+DEFAULT_XYZ[0], y_+DEFAULT_XYZ[1], z] + list(DEFAULT_RXRYRZ) + [0, 0, 0]
                        writer1.writerow(row)
                        writer2.writerow(row)

                    # time.sleep(2.0)
            print(f"成功写入{file1},{file2}-----------------")
        finally:
            traj_queue.task_done()


def receiveThread(socket1):
    """
    接收线程：读取 socket 数据，支持两类内容
      1) 原有命令处理（Initialize / biao ...）
      2) 轨迹接收协议（可由上位机按行发送）:
         - START <idx> <n>
         - x,y 或 x,y,z  （每行一个点）
         - END <idx>
       完整轨迹接收完后把点列表放入 traj_queue，由执行线程依次绘制/运动。
       同时保持原有对 Initialize 和 biao 的处理与回执。
    """
    global img_stuts, zhilin, stuts_add
    print("启动接收线程")
    buf = ""           # 文本缓冲
    in_traj = False
    expect_n = 0
    collected = []
    while True:
        len_js = 0
        recBuf = 0
        err, data = TCPRead(socket1)
        if err != 0 or not data:
            time.sleep(0.01)
            continue
        try:
            chunk = data.decode('utf-8', errors='ignore')
        except Exception:
            chunk = str(data)
        buf += chunk
        print("收到数据:", chunk.strip())
        # 按行解析
        while '\n' in buf:
            line, buf = buf.split('\n', 1)
            line = line.strip()
            if not line:
                continue

            # 轨迹协议解析 START/点/END
            parts = line.split()
            if parts and parts[0] == 'START':
                in_traj = True
                collected = []
                expect_n = int(parts[2]) if len(parts) >= 3 else 0
                TCPWrite(socket1, "START_ACK")
                continue
            if in_traj:
                # END 行
                if parts and parts[0] == 'END':
                    in_traj = False
                    # 将收集到的点入队执行
                    if collected:
                        traj_queue.put(collected)
                        TCPWrite(socket1, "TRAJ_RECEIVED")
                    else:
                        TCPWrite(socket1, "TRAJ_EMPTY")
                    collected = []
                    expect_n = 0
                    stuts_add = True
                    continue
                # 点行 x,y 或 x,y,z
                try:
                    coords = [c.strip() for c in line.split(',') if c.strip()!='']
                    if len(coords) >= 2:
                        pt = [float(coords[0]), float(coords[1])]
                        if len(coords) >= 3:
                            pt.append(float(coords[2]))
                        collected.append(pt)
                except Exception:
                    # 忽略解析错误
                    pass
                # 若发送方没有 END，但已达到期望点数则自动结束并入队
                if expect_n > 0 and len(collected) >= expect_n:
                    in_traj = False
                    traj_queue.put(collected)
                    TCPWrite(socket1, "TRAJ_RECEIVED")
                    collected = []
                    expect_n = 0
                    stuts_add = True
                continue
            # 其他非轨迹内容，直接回显并继续监听
            try:
                TCPWrite(socket1, line)  # 回发原行，保持监听
            except Exception:
                pass


# 网口初始化
err, socket1 = TCPCreate(True, "192.168.5.1", 5200)   #视觉系统
TCPStart(socket1, 0)

tcp_js1 = threading.Thread(target=receiveThread,args=(socket1, ))
tcp_js1.daemon = True
tcp_js1.start()
print("启动接收线程完成")

## 运动至初始点位
assert P7 is not None, "P7 未定义，请根据实际机械臂型号修改代码"
status = CheckMovJ(P7)
if status == 0:
    MovJ(P7)

time.sleep(2.0)
ori_point_angle = GetAngle()
ori_point_pose = GetPose()

while True:
    if stuts_add:
        execute_traj_worker(ori_angle=ori_point_angle, ori_pose=ori_point_pose)
        stuts_add = False
