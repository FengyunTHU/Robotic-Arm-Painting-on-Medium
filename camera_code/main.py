"""使用maix串口向舵机发送高低电平"""


from maix import camera, display, pinmap, uart, app, time, gpio, err
import threading
from enum import Enum


### 18|->启动终止。1启动、0终止
### 19|->正转反转。1正转、0反转
pin_name = "A18"
gpio_name = "GPIOA18"
pin_name2 = "A19"
gpio_name2 = "GPIOA19"
## 设置A18为GPIOA18
err.check_raise(pinmap.set_pin_function(pin_name, gpio_name), "set pin failed")# MaixCAM的引脚是3.3V耐受，请勿输入5V电压。
err.check_raise(pinmap.set_pin_function(pin_name2, gpio_name2), "set pin failed")
servo = gpio.GPIO(gpio_name, gpio.Mode.OUT)
servo2 = gpio.GPIO(gpio_name2, gpio.Mode.OUT)
## 0为低电平、1为高电平
servo.value(0)
servo2.value(0)
time.sleep(2.0)
servo.value(1)
time.sleep(7.0)
servo.value(0)
servo2.value(0)
# time.sleep(2.0)
# servo2.value(0)
# servo.value(1)
# time.sleep(7.0)
# servo.value(0)