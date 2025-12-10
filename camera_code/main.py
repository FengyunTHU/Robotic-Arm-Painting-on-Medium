"""使用maix串口向舵机发送高低电平"""


from maix import camera, display, pinmap, uart, app, time, gpio, err
import threading
from enum import Enum

class ServoStatus(Enum):
    HIGH = 1
    LOW = 0

status = ServoStatus.LOW ## 初始为低电平
pin_name = "A18"
gpio_name = "GPIOA18"
## 设置A18为GPIOA18
err.check_raise(pinmap.set_pin_function(pin_name, gpio_name), "set pin failed")# MaixCAM的引脚是3.3V耐受，请勿输入5V电压。
servo = gpio.GPIO(gpio_name, gpio.Mode.OUT)
servo.value(0)
print("aaa")

while 1:
    servo.toggle()
    time.sleep_ms(1000)