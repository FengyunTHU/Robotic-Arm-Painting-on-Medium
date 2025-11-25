# version: Python3
import time ,threading  # 导入库函数
img_stuts = 0         # 摄像头下发指令编码
zhilin   = 0         # 摄像头下发16进制指令信息
def receiveThread(socket1):
    global img_stuts, zhilin
    while 1:
        len_js    = 0
        recBuf    = 0
        err, recBuf = TCPRead(socket1)
        recBuf = recBuf.decode('utf-8', errors='ignore')
        print(recBuf)
    
        if 'Initialize' in recBuf and img_stuts == 0 :    #
            img_stuts  =  1  #回到初始状态 
            TCPWrite( socket1, "yunxing" )
    
        if recBuf[:4] == 'biao' and img_stuts == 0 and len(recBuf) >= 16:    #
            x_zb = int(recBuf[4:8])
            y_zb = int(recBuf[8:12])
            z_zb = int(recBuf[12:16])
            img_stuts  =  2  #回到初始状态 
            if x_zb > 0:
                print(x_zb)
            if z_zb < 0:
                print(y_zb, z_zb)
            TCPWrite( socket1, "yunxing" )  
# 网口初始化
err, socket1 = TCPCreate( True, "192.168.5.1", 5200)   #视觉系统
TCPStart(socket1, 0)

tcp_js1 = threading.Thread(target=receiveThread,args=(socket1, ))
tcp_js1.daemon = True
tcp_js1.start()

while 1:
    if img_stuts == 1:  # 起始位置
        MovJ(P1, {"user": 0, "v": 100})
        img_stuts = 0

    if img_stuts == 2  :
        MovJ(P2, {"user": 0, "v": 100})
        img_stuts = 0








