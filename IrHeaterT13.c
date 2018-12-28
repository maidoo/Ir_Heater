// Compiler: WinAVR-20100110
// OPT = s
// Application builded at : 20110127
// Target : T13
// Crystal: 1.2000Mhz (使用片内RC振荡器，出厂默认值)

// 麦兜-CDMA650   maidoo@163.com
// copyrights, 2010, maidoo@163.com


/* ========================================================================
                四通道学习型红外遥控接收器（NEC格式）
   ========================================================================
                         _____   _____
            Reset  PB5 -|1    \_/    8|- VCC
          Key3 IO  PB3 -|2           7|- PB2   Key2 IO
     BEEP/Key4 IO  PB4 -|3   Tiny13  6|- PB1   INT0红外输入
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
    NEC指令格式请参考：
    http://www.sbprojects.com/knowledge/ir/nec.htm
    如果要所有红外格式通吃，请使用仅支持一个指令的《learn-t13.c》项目。
   ======================================================================== */

#include <avr/io.h>
#include <avr/eeprom.h>
#include <util/delay_basic.h>
#include <avr/interrupt.h>

/* --------------------------------------------------
   是否需要蜂鸣器指示进入学习状态、按键提示等
   1:  配备一个有源蜂鸣器，输出短暂直流电平驱动其工作
   0:  没有蜂鸣器，该IO可用于模拟按键
   -------------------------------------------------- */
#define __WITH_BEEP             1
/* -------------------------------------------------- */


#if __WITH_BEEP

    // 输出的模拟按键的个数
    #define TOTAL_OUTPUT_COUNT  3

    // 所有按键的MASK码
    #define KEYS_MASK           0x0D

#else
    #define TOTAL_OUTPUT_COUNT  4
    #define KEYS_MASK           0x1D
#endif

#define IO_BEEP                 0x10

// 4个按键对应的IO管脚
const unsigned char OutPinMask [TOTAL_OUTPUT_COUNT] = {0x01, 0x04, 0x08
#if !(__WITH_BEEP)
                                                , IO_BEEP
#endif
                                                };

// 按键高电平有效时的判断表达式
// #define KEY_PRESSED             ( PINB & KEYS_MASK )

// 按键低电平有效时的判断表达式
#define KEY_PRESSED             ( ~(PINB) & KEYS_MASK )


// 延时参数，影响输出脉冲宽度，16bits，最大为 65535
#define OUT_PULSE_WIDTH         24321

// 延时参数，影响进入学习状态前按住按钮所需的时间
// 从255递减到该值的时间
//      195: 约3秒钟
//      155: 约5秒钟
#define WAIT_LEARNING           155


// 红外指令解码的状态机
typedef enum{
             IR_idle,
             IR_waitstart,
             IR_getaddrlow,
             IR_getaddrhigh,
             IR_getdata,
             IR_getdatainv
            }_IRstate;

volatile _IRstate IRstate = IR_idle;

//定义位操作
#define SET_BIT(reg,bitmask)    ( reg |=  (bitmask) )
#define CLR_BIT(reg,bitmask)    ( reg &= ~(bitmask) )
#define INV_BIT(reg,bitmask)    ( reg ^=  (bitmask) )
#define GET_BIT(reg,bitmask)    ( reg &   (bitmask) )

#define GET_PIN(bitmask)        ( PINB  &   (bitmask) )
#define OUT_HIGH(bitmask)       ( PORTB |=  (bitmask) )
#define OUT_LOW(bitmask)        ( PORTB &= ~(bitmask) )
#define OUT_INV(bitmask)        ( PORTB ^=  (bitmask) )


// 保存EEPROM的起始地址，根据datasheet描述的缺陷，不要从0地址开始使用
#define EE_MARGIN               0x10

#define START_T0                ( TCCR0B = 0x04 )
#define STOP_T0                 ( TCCR0B = 0x00 )
#define ENABLE_INT0             ( GIMSK  = 0x40 )
#define DISABLE_INT0            ( GIMSK  = 0x00 )

volatile unsigned char IR_Ready;
volatile unsigned char TimerCount;

//volatile register unsigned char TimerCount asm("r15");

// NEC格式，每个红外指令有4个字节
typedef struct {
                unsigned char addrl;        // 地址低位
                unsigned char addrh;        // 地址高位
                unsigned char data;         // 指令码
                unsigned char datainv;      // 指令反码
                } _IRCMD;

volatile _IRCMD LearnedCMD [TOTAL_OUTPUT_COUNT];
volatile _IRCMD curCMD;

// unsigned char addrH           = 0xff;
// unsigned char addrL           = 0xff;
// unsigned char dataH           = 0xff;
// unsigned char dataL           = 0xff;


//#pragma interrupt_handler timer0_ovf_isr:4
//void timer0_ovf_isr(void)
//----------------------------------------------------------------------------
// INTERUPT HANDLER (TIMER/COUNTER0 OVERFLOW)
//----------------------------------------------------------------------------
ISR(TIM0_OVF_vect)
{
    // INV_BIT(IO_LED);

    // 在内部默认的1.2MHz RC振荡器配置下，再256分频后
    // T0计满0xFF后的溢出周期约为53毫秒
    // 2011-1-15, 室温5摄氏度，实测周期为57.8毫秒

    if (! (-- TimerCount)) {
        IRstate = IR_idle;
        STOP_T0;
        // MCUCR = 0x02;  // INT0下降沿触发
    }
}


// Attiny13缺省RC时钟1.2MHz，256分频后周期为206微秒，
// T0每206us计数加1，以下定义各时间段的计数值
//#define msec_15     0x48
//#define msec_12p5   0x3C
//#define msec_9      0x2B
//#define msec_2p5    0x0C
//#define msec_1p68   0x08
//#define msec_0p9    0x04

// Attiny13缺省RC时钟1.2MHz，256分频后周期为206微秒，
// T0每206us计数加1，以下定义各时间段的计数值
// 20130627, ATtiny13A比ATtiny13V的内置RC振荡器频率略低些，放宽这里的参数
// 以支持13A。正常情况下，TCNT0将采集到：
//             引导码=13.5mS; 逻辑1=2.25mS; 逻辑0=1.12mS; 重复码=11.25mS
// 以下定义各时间段的TCNT0的计数值
#define msec_15     0x48
#define msec_12p5   0x38
#define msec_9      0x2B
#define msec_2p5    0x0F
#define msec_1p68   0x08
#define msec_0p9    0x04


// When the IR receive pin goes low and interrupt is generated
// IR is collected by starting timer 2 in the first falling edge of the pin
// then on every other falling edge, the timer value is saved and the timer restarted .
// the captured time is then used to get the IR data
// a "start of data" is 13.5Msec,a "1" is 2.25Msec,a "0" is 1.12 msec and a "repeat" is 11.25msec.
// the counter increments at ~64 Usec
// I allow a fairly large tolerance to time jitter but there are no false triggers seen.
// #pragma interrupt_handler int0_isr:2
// void int0_isr(void)
//----------------------------------------------------------------------------
// INTERUPT EXTERMNAL INTERUPT 0
//----------------------------------------------------------------------------
ISR(INT0_vect)
{
 //external interupt on INT0
    static unsigned char bits;
    unsigned char time;

    switch(IRstate)
    {
        case IR_idle:
            TCNT0 = 0;
            START_T0;
            IRstate = IR_waitstart;

            // 最后一个脉冲后2个溢出周期(53ms)结束
            TimerCount = 2;
            break;

        case IR_waitstart:
            time = TCNT0;
            TCNT0 = 0;
            // greater than 12.5Msec & less than 15 msec = start code
            if ((time > msec_12p5)&&(time < msec_15)) {
                curCMD.addrl=0;
                curCMD.addrh=0;
                curCMD.data=0;
                curCMD.datainv=0;
                bits=0x01;
                IRstate=IR_getaddrlow;
            }
            // less than 12.5Msec  and greater than 9 msec =Repeat code
            //else if ((time > msec_9)&&(time <  msec_12p5))
            //  {
            //    // IR_repeat++;
            //    IRstate=IR_idle;
            //}
            else {
                // too short, bad data just go to idle
                IRstate = IR_idle;
            }
            break;

        case IR_getaddrlow:
            time = TCNT0;
            TCNT0 = 0;
            // if  > 2.5msec or shorter than .9Msec bad data, go to idle
            if ((time>msec_2p5)||(time<msec_0p9)) {
                IRstate=IR_idle;
                break;
            }
            if (time>msec_1p68) {
                // greater than 1.68Msec is a 1
                curCMD.addrl|= bits;
            }
            bits=bits<<1;
            if (!bits) {
                IRstate=IR_getaddrhigh;
                bits=0x01;
            }
            break;

        case IR_getaddrhigh:
            time = TCNT0;
            TCNT0 = 0;
            if ((time>msec_2p5)||(time<msec_0p9)) {
                IRstate=IR_idle;
                break;
            }
            if (time>msec_1p68) {
                // greater than 1.68Msec is a 1
                curCMD.addrh|= bits;
            }
            bits=bits<<1;
            if (!bits) {
                IRstate=IR_getdata;;
                bits=0x01;
            }
            break;

        case IR_getdata:
            time = TCNT0;
            TCNT0 = 0;
            if ((time>msec_2p5)||(time<msec_0p9)) {
                IRstate=IR_idle;
                break;
            }
            if (time>msec_1p68) {
                curCMD.data|= bits;
            }
            bits=bits<<1;
            if (!bits) {
                IRstate=IR_getdatainv;
                bits=0x01;
            }
            break;

        case IR_getdatainv:
            time = TCNT0;
            TCNT0 = 0;
            // if  > 2.5msec or shorter than .9Msec bad data , go to idle
            if ((time>msec_2p5)||(time<msec_0p9)) {
                IRstate=IR_idle;
                break;
            }
            if (time>msec_1p68) {
                curCMD.datainv|= bits;
            }
            bits=bits<<1;
            // we have it all , now we make sure it is a NEC code from
            if (!bits) {
                // 完整的一帧数据结束
                IR_Ready++;
                IRstate=IR_idle;
            }
            break;
        default:
            IRstate=IR_idle;
            break;
    }
}


/*
void shift_out(unsigned char val)
{
    unsigned char i,j;
    for (i=0; i<=7; i++) {
        CLR_BIT(PORTB,0);     // clk out low

        if (GET_BIT(val,i)){  // data out
            SET_BIT(PORTB,2);     } else{
            CLR_BIT(PORTB,2);
        }
        //delay(10);
        SET_BIT(PORTB,0);    // clk out high
        //delay(10);
    }
}
*/

static void port_init(void)
{
    // PB1做INT0输入，带上拉电阻；其他都作为按键输入，带上拉电阻，
    DDRB  = 0x00;    // xx00 0000
    PORTB = 0x3F;    // xx11 1111

#if __WITH_BEEP
    PORTB &= ~IO_BEEP;
    DDRB  |=  IO_BEEP;
#endif

}

//TIMER0 initialize - prescale:256
// WGM: Normal
static void timer0_init(void)
{
    //TCCR0B = 0x00; //stop
    STOP_T0;
    OCR0A = 0xEA;
    OCR0B = 0xEA;
    // TCNT0 = 0x16;    //set count
    TCCR0A = 0x00;
    //TCCR0B = 0x04; //start timer
}


//call this routine to initialize all peripherals
static void init_devices(void)
{
    //stop errant interrupts until set up
    cli();         //disable all interrupts

    port_init();
    timer0_init();


    MCUCR = 0x02;  // INT0下降沿触发
    TIMSK0 = 0x02; //timer interrupt sources

    // GIMSK = 0x40;  //interrupt sources
    ENABLE_INT0;

    sei();         //re-enable interrupts
    //all peripherals are now initialized
}

void load_learned_instruction(void)
{
    unsigned char i;

    for (i=0; i< sizeof(LearnedCMD); i++) {
        *((unsigned char *) &LearnedCMD + i) = eeprom_read_byte(EE_MARGIN + i);
    }
}

//====================================================
void main(void) __attribute__((noreturn));
void main(void)
{
    unsigned char i;
    unsigned char Key_State;
    unsigned char cur_key_index = 0;

    init_devices();

    // 先读取学习到的指令数据
    load_learned_instruction();

    while(1)
    {
    // ------------ 按键按下 --------------------------------
        Key_State = KEY_PRESSED;
        if (Key_State) {

            // 关INT0中断，暂停处理红外信号。否则处理红外中断，
            // 会修改TimerCount，影响定时器
            DISABLE_INT0;

            // 启动定时器查看按钮按下的时长
            TimerCount = 0xFF;

            // TCNT0 = 0;
            START_T0;

            // 先搞清楚是按了哪个键
            for (i=0; i<=TOTAL_OUTPUT_COUNT; i++) {
                if (Key_State == OutPinMask[i]) {
                    cur_key_index = i;
                    break;
                }
            }

            // 然后死循环等按键放开
#if __WITH_BEEP
            while ((KEY_PRESSED) && (WAIT_LEARNING <= TimerCount));
#else
            while (KEY_PRESSED);
#endif
            // 看看按了多长时间，短的是普通按键操作，原设备处理，CPU不用理会，
            // 超过5秒钟的，那就是要开始对应按键的指令学习了。

            if (WAIT_LEARNING >= TimerCount) {
            // ------------ 按下超过5秒，开始学习过程 ------------

                // 设置8秒种超时，如果还没有收到任何信号，就放弃本次学习
                TimerCount = 160;
                // START_T0;
                // Learning_Mode = 1;

                // 开启INT0中断，允许处理红外信号
                ENABLE_INT0;

#if __WITH_BEEP
                OUT_HIGH(IO_BEEP);
#endif

                while (TimerCount)
                {
                    //INV_BIT(LED);
                    //PORTB ^= _BV(LED);
                    //_delay_loop_2(40000);       // 开启学习期间，闪烁LED

                    //由于收到红外信号后会重新给TimerCount赋值2，
                    //所以无需再判断 IR_Ready 作为退出条件
                    //学习期间，任何按键事件、学习完成都会退出学习状态
                    //if (KEY_PRESSED || IR_Ready) break;
                }

                // SET_BIT(LED);         // 确保学习完毕后LED是熄灭的

                // Learning_Mode = 0;
                // T0在TimerCount归零时自动停止，无需再这里关闭，否则会干扰学习过程
                // STOP_T0;
                // TimerCount = 1;



                if (IR_Ready) {
                    IR_Ready = 0;

                    // 把学习到的指令数据保存到eeprom
                    for (i=0; i<sizeof(_IRCMD); i++) {
                        // 写入EEPROM保存
                        eeprom_update_byte(EE_MARGIN + cur_key_index * sizeof(_IRCMD) + i,
                                        *((unsigned char *) &curCMD + i));

                        // 更新当前RAM里的数据
                        load_learned_instruction();

                        //*((unsigned char *) &(LearnedCMD[cur_key_index]) + i)
                        //                = *((unsigned char *) &curCMD + i);
                    }
                    //LearnedCMD[cur_key_index].addrH = curCMD.addrH;
                    //LearnedCMD[cur_key_index].addrL = curCMD.addrL;
                }
#if __WITH_BEEP
                OUT_LOW(IO_BEEP);
#endif
            }

            // 清理环境
            // 短按按钮时，尽快关闭定时器
            TimerCount = 1;       // 不要设置为0,防止 --TimerCount 后得 0xFF
            ENABLE_INT0;          // 开启INT0中断，允许处理红外信号
        }



    // ------------ 正常模式下，判断解码是否OK， ------------

        if (IR_Ready) {

            DISABLE_INT0;     // 暂停红外，杜绝数据被快速覆盖的可能性
            IR_Ready = 0;


            for (i=0; i<TOTAL_OUTPUT_COUNT; i++) {

                // 找到对应的IO端口，输出低电平脉冲
                if (curCMD.data == LearnedCMD[i].data) {

#if __WITH_BEEP
                    OUT_HIGH(IO_BEEP);
#endif
                    // IO口拉低，然后切换为输出状态
                    OUT_LOW(OutPinMask[i]);
                    SET_BIT(DDRB,OutPinMask[i]);

                    _delay_loop_2(OUT_PULSE_WIDTH);

                    // 完事后，先拉高，在切回输入状态
                    OUT_HIGH(OutPinMask[i]);
                    CLR_BIT(DDRB,OutPinMask[i]);

#if __WITH_BEEP
                    OUT_LOW(IO_BEEP);
#endif

                    break;
                }
            }

            ENABLE_INT0;      // 恢复INT0，启动红外接收
        }


    }
}

/* end of file */
