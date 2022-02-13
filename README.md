Based on a SOP8 MCU (AVR ATtiny13A), decode NEC format IR remote control command. The MCU will output a LOW pluse while reconginized a command from IR remote command. 


/* ========================================================================
                四通道学习型红外遥控接收器（NEC格式）
   ========================================================================
                         _____   _____
            Reset  PB5 -|1    \_/    8|- VCC
          Key3 IO  PB3 -|2           7|- PB2   Key2 IO
     BEEP/Key4 IO  PB4 -|3   Tiny13  6|- PB1   INT0 Ir input
                   GND -|4___________5|- PB0   Key1 IO

    加热器上的轻触开关一般都是共地的，为了把这些按键改装成为遥控功能的，
    请把 Key IO 直接连接到各个按键上，当CPU解码出正确的红外遥控器指令时，
    就在对应的 Key IO 上输出一个固定200毫秒宽的低电平，模拟按键操作。
    注意：通过红外遥控，仅支持单按操作，不支持长按操作。
    如果CPU检测到管脚上持续5秒以上的低电平，则启动对应通道的学习过程
    在学习过程中，按任意键将退出学习过程，
    在学习过程中，超过8秒没有收到合法红外指令将结束学习过程。
    如果减少一个通道，则可以增加LED或者蜂鸣器指示学习状态。
    仅支持最常用的NEC格式的红外编码，每个按键指令有4字节的数据。
    
