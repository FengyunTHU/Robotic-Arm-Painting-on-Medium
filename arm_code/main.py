# version: Python3
import time ,threading  # 导入库函数
import json
import queue
import csv ## 创建csv文件
from math import sin,cos,tan,pi


## 在下面俩个文件夹中都保存了csv
PATH1:str = "/dobot/userdata/user_project/process/trajectory/"
PATH2:str = "/dobot/userdata/project/process/trajectory/"

## user:0, tool:0, ecokey:0.
HEAD:list = ['j1','j2','j3','j4','j5','j6','x','y','z','Rx','Ry','Rz','user','tool','ecoKey']


stuts_add = False
Path_id = 0

# DEFAULT是P7


traj_queue = queue.Queue()
COLOR_CSV:dict = dict() # 存储id、csv路径和color
color_queue = queue.Queue()

"""
x正方向：内部
y负方向：右侧
z正方向：上侧
"""

### 松开
def SnapOpen():
    SetParallelGripper(15)
    time.sleep(0.5)

### 夹取
def SnapClose():
    SetParallelGripper(7)
    time.sleep(0.5)


### 进行液体吸取的函数
def Input_liquid(distance, target:str="Y", angle:float=pi/6) -> bool:
    ## angle是与水平方向的夹角
    current_angle = GetAngle()
    current_pose = GetPose()

    X, Y, Z, RX, RY, RZ = current_pose["pose"][0:6]
    ## 向y正方向移动
    YR = Y + distance*cos(angle)
    ZR = Z - distance*sin(angle)
    MovL({"pose":[X,YR,ZR,RX,RY,RZ]},{"v":10})
    time.sleep(2.0)
    MovL({"pose":[X,Y,Z,RX,RY,RZ]},{"v":10})
    return True


### 执行绘画运动的函数
def execute_traj_worker(ori_angle, ori_pose):
    """
    从queue中不断取出点构架轨迹csv。依次进行csv绘制。同一轨迹的颜色应当相同。
    """
    global COLOR_CSV, Path_id,traj_queue,color_queue
    DEFAULT_XYZ = ori_pose["pose"][0:3]
    DEFAULT_RXRYRZ = ori_pose["pose"][3:6]
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
                    errid, jointpoint = InverseKin({"pose":[x_+DEFAULT_XYZ[0], y_+DEFAULT_XYZ[1], z]+DEFAULT_RXRYRZ})
                    if errid == 0:
                        row = jointpoint["joint"] + [x_+DEFAULT_XYZ[0], y_+DEFAULT_XYZ[1], z] + DEFAULT_RXRYRZ + [0, 0, 0]
                        writer1.writerow(row)
                        writer2.writerow(row)

                    # time.sleep(2.0)
                print("LEN:",color_queue)
                COLOR_CSV[Path_id] = {"csv":(file1,file2),"color":color_queue.get()}
            print(f"成功写入{file1},{file2}-----------------")
        finally:
            traj_queue.task_done()


def run(pathid:int):
    file01,file02 = COLOR_CSV[pathid]["csv"]
    color = COLOR_CSV[pathid]["color"]
    GetLiquid(color) # 获取对应颜色的菌液
    time.sleep(1.0)
    ## 执行绘画
    StartPath(f"x{pathid}.csv")
    time.sleep(1.0)


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
    global stuts_add, COLOR_CSV, Path_id,traj_queue,color_queue
    print("启动接收线程")
    buf = ""           # 文本缓冲
    in_traj = False
    expect_n = 0
    collected = []
    color = None
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
                color = None
                expect_n = int(parts[2]) if len(parts) >= 3 else 0
                TCPWrite(socket1, "START_ACK")
                continue
            if in_traj:
                # COLOR
                if parts and parts[0] == 'COLOR':
                    color = parts[1] if len(parts) >= 2 else "default"
                    color_queue.put(color)
                    TCPWrite(socket1, "COLOR_ACK")
                    continue
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
                    color = None
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


def MovJ_self(point) -> bool:
    assert point is not None, "点未定义"
    status =  CheckMovJ(point)
    if status == 0:
        MovJ(point,{"v":10})
        return True
    else:
        return False
    
def MovL_self(point) -> bool:
    assert point is not None, "点未定义"
    status =  CheckMovL(point)
    if status == 0:
        MovL(point,{"v":10})
        return True
    else:
        return False
    
### 完整夹取
def Snap():
    time.sleep(5.0) # 等待构建完
    MovJ_self(P12)
    time.sleep(1.0)
    MovJ_self(P9)
    SnapOpen()
    time.sleep(1.0)
    MovL_self(P10)
    SnapClose()
    time.sleep(1.0)
    MovJ_self(P9)
    time.sleep(1.0)

### 完整取液
def GetLiquid(color):
    MovJ_self(P11)
    Input_liquid(32.0,angle=pi/6)
    time.sleep(2.0)
    MovJ_self(P13)
    time.sleep(2.0)
    MovJ_self(P14)
    return 


# 网口初始化
err, socket1 = TCPCreate(True, "192.168.5.1", 5200)   #视觉系统
TCPStart(socket1, 0)

tcp_js1 = threading.Thread(target=receiveThread,args=(socket1, ))
tcp_js1.daemon = True
tcp_js1.start()
print("启动接收线程完成")

# ## 运动至初始点位
# assert P7 is not None, "P7 未定义，请根据实际机械臂型号修改代码"
# status = CheckMovJ(P7)
# if status == 0:
#     MovJ(P7)
time.sleep(2.0)
ori_point_angle = GetAngle()
ori_point_pose = GetPose()

traj_thread = threading.Thread(target=execute_traj_worker, args=({"joint":[0,0,0,0,0,0]}, {"pose":[260.543304,-75.82502,390,176.204605,-3.127154,90.00]}))
traj_thread.daemon = True
traj_thread.start()

while True:
    if stuts_add: # 
        # execute_traj_worker(ori_angle=ori_point_angle, ori_pose=ori_point_pose) ## 构建csv
        ## 进行夹取
        Snap()
        stuts_add = False

    ## 检测有没有csv文件已经出现，且和PathID数值相等
    if len(COLOR_CSV) >= Path_id:
        opentuts = False
        for idx in range(len(COLOR_CSV)):
            file1,file2 = COLOR_CSV[idx+1]["csv"]
            colorx = COLOR_CSV[idx+1]["color"]
            if idx == 0:
                GetLiquid(colorx)
            ## 执行运动
            if not opentuts:
                TCPWrite(socket1, "OPEN")
                opentuts = True
                time.sleep(13.0) ## 等待开启
            run(idx+1)
            time.sleep(2.0)
            if idx+2 <= Path_id:
                colorx_later = COLOR_CSV[idx+2]["color"]
                if colorx_later != colorx:
                    ## 切换颜色
                    TCPWrite(socket1, "CLOSE")
                    opentuts = False
                    time.sleep(13.0)
                    GetLiquid(colorx_later)