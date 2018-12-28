// Compiler: WinAVR-20100110
// OPT = s
// Application builded at : 20110127
// Target : T13
// Crystal: 1.2000Mhz (ʹ��Ƭ��RC����������Ĭ��ֵ)

// ��-CDMA650   maidoo@163.com
// copyrights, 2010, maidoo@163.com


/* ========================================================================
                ��ͨ��ѧϰ�ͺ���ң�ؽ�������NEC��ʽ��
   ========================================================================
                         _____   _____
            Reset  PB5 -|1    \_/    8|- VCC
          Key3 IO  PB3 -|2           7|- PB2   Key2 IO
     BEEP/Key4 IO  PB4 -|3   Tiny13  6|- PB1   INT0��������
                   GND -|4___________5|- PB0   Key1 IO

    �������ϵ��ᴥ����һ�㶼�ǹ��صģ�Ϊ�˰���Щ������װ��Ϊң�ع��ܵģ�
    ��� Key IO ֱ�����ӵ����������ϣ���CPU�������ȷ�ĺ���ң����ָ��ʱ��
    ���ڶ�Ӧ�� Key IO �����һ���̶�200�����ĵ͵�ƽ��ģ�ⰴ��������
    ע�⣺ͨ������ң�أ���֧�ֵ�����������֧�ֳ���������

    ���CPU��⵽�ܽ��ϳ���5�����ϵĵ͵�ƽ����������Ӧͨ����ѧϰ����
    ��ѧϰ�����У�����������˳�ѧϰ���̣�
    ��ѧϰ�����У�����8��û���յ��Ϸ�����ָ�����ѧϰ���̡�

    �������һ��ͨ�������������LED���߷�����ָʾѧϰ״̬��

    ��֧����õ�NEC��ʽ�ĺ�����룬ÿ������ָ����4�ֽڵ����ݡ�
    NECָ���ʽ��ο���
    http://www.sbprojects.com/knowledge/ir/nec.htm
    ���Ҫ���к����ʽͨ�ԣ���ʹ�ý�֧��һ��ָ��ġ�learn-t13.c����Ŀ��
   ======================================================================== */

#include <avr/io.h>
#include <avr/eeprom.h>
#include <util/delay_basic.h>
#include <avr/interrupt.h>

/* --------------------------------------------------
   �Ƿ���Ҫ������ָʾ����ѧϰ״̬��������ʾ��
   1:  �䱸һ����Դ���������������ֱ����ƽ�����乤��
   0:  û�з���������IO������ģ�ⰴ��
   -------------------------------------------------- */
#define __WITH_BEEP             1
/* -------------------------------------------------- */


#if __WITH_BEEP

    // �����ģ�ⰴ���ĸ���
    #define TOTAL_OUTPUT_COUNT  3

    // ���а�����MASK��
    #define KEYS_MASK           0x0D

#else
    #define TOTAL_OUTPUT_COUNT  4
    #define KEYS_MASK           0x1D
#endif

#define IO_BEEP                 0x10

// 4��������Ӧ��IO�ܽ�
const unsigned char OutPinMask [TOTAL_OUTPUT_COUNT] = {0x01, 0x04, 0x08
#if !(__WITH_BEEP)
                                                , IO_BEEP
#endif
                                                };

// �����ߵ�ƽ��Чʱ���жϱ��ʽ
// #define KEY_PRESSED             ( PINB & KEYS_MASK )

// �����͵�ƽ��Чʱ���жϱ��ʽ
#define KEY_PRESSED             ( ~(PINB) & KEYS_MASK )


// ��ʱ������Ӱ����������ȣ�16bits�����Ϊ 65535
#define OUT_PULSE_WIDTH         24321

// ��ʱ������Ӱ�����ѧϰ״̬ǰ��ס��ť�����ʱ��
// ��255�ݼ�����ֵ��ʱ��
//      195: Լ3����
//      155: Լ5����
#define WAIT_LEARNING           155


// ����ָ������״̬��
typedef enum{
             IR_idle,
             IR_waitstart,
             IR_getaddrlow,
             IR_getaddrhigh,
             IR_getdata,
             IR_getdatainv
            }_IRstate;

volatile _IRstate IRstate = IR_idle;

//����λ����
#define SET_BIT(reg,bitmask)    ( reg |=  (bitmask) )
#define CLR_BIT(reg,bitmask)    ( reg &= ~(bitmask) )
#define INV_BIT(reg,bitmask)    ( reg ^=  (bitmask) )
#define GET_BIT(reg,bitmask)    ( reg &   (bitmask) )

#define GET_PIN(bitmask)        ( PINB  &   (bitmask) )
#define OUT_HIGH(bitmask)       ( PORTB |=  (bitmask) )
#define OUT_LOW(bitmask)        ( PORTB &= ~(bitmask) )
#define OUT_INV(bitmask)        ( PORTB ^=  (bitmask) )


// ����EEPROM����ʼ��ַ������datasheet������ȱ�ݣ���Ҫ��0��ַ��ʼʹ��
#define EE_MARGIN               0x10

#define START_T0                ( TCCR0B = 0x04 )
#define STOP_T0                 ( TCCR0B = 0x00 )
#define ENABLE_INT0             ( GIMSK  = 0x40 )
#define DISABLE_INT0            ( GIMSK  = 0x00 )

volatile unsigned char IR_Ready;
volatile unsigned char TimerCount;

//volatile register unsigned char TimerCount asm("r15");

// NEC��ʽ��ÿ������ָ����4���ֽ�
typedef struct {
                unsigned char addrl;        // ��ַ��λ
                unsigned char addrh;        // ��ַ��λ
                unsigned char data;         // ָ����
                unsigned char datainv;      // ָ���
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

    // ���ڲ�Ĭ�ϵ�1.2MHz RC���������£���256��Ƶ��
    // T0����0xFF����������ԼΪ53����
    // 2011-1-15, ����5���϶ȣ�ʵ������Ϊ57.8����

    if (! (-- TimerCount)) {
        IRstate = IR_idle;
        STOP_T0;
        // MCUCR = 0x02;  // INT0�½��ش���
    }
}


// Attiny13ȱʡRCʱ��1.2MHz��256��Ƶ������Ϊ206΢�룬
// T0ÿ206us������1�����¶����ʱ��εļ���ֵ
//#define msec_15     0x48
//#define msec_12p5   0x3C
//#define msec_9      0x2B
//#define msec_2p5    0x0C
//#define msec_1p68   0x08
//#define msec_0p9    0x04

// Attiny13ȱʡRCʱ��1.2MHz��256��Ƶ������Ϊ206΢�룬
// T0ÿ206us������1�����¶����ʱ��εļ���ֵ
// 20130627, ATtiny13A��ATtiny13V������RC����Ƶ���Ե�Щ���ſ�����Ĳ���
// ��֧��13A����������£�TCNT0���ɼ�����
//             ������=13.5mS; �߼�1=2.25mS; �߼�0=1.12mS; �ظ���=11.25mS
// ���¶����ʱ��ε�TCNT0�ļ���ֵ
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

            // ���һ�������2���������(53ms)����
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
                // ������һ֡���ݽ���
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
    // PB1��INT0���룬���������裻��������Ϊ�������룬���������裬
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


    MCUCR = 0x02;  // INT0�½��ش���
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

    // �ȶ�ȡѧϰ����ָ������
    load_learned_instruction();

    while(1)
    {
    // ------------ �������� --------------------------------
        Key_State = KEY_PRESSED;
        if (Key_State) {

            // ��INT0�жϣ���ͣ��������źš�����������жϣ�
            // ���޸�TimerCount��Ӱ�춨ʱ��
            DISABLE_INT0;

            // ������ʱ���鿴��ť���µ�ʱ��
            TimerCount = 0xFF;

            // TCNT0 = 0;
            START_T0;

            // �ȸ�����ǰ����ĸ���
            for (i=0; i<=TOTAL_OUTPUT_COUNT; i++) {
                if (Key_State == OutPinMask[i]) {
                    cur_key_index = i;
                    break;
                }
            }

            // Ȼ����ѭ���Ȱ����ſ�
#if __WITH_BEEP
            while ((KEY_PRESSED) && (WAIT_LEARNING <= TimerCount));
#else
            while (KEY_PRESSED);
#endif
            // �������˶೤ʱ�䣬�̵�����ͨ����������ԭ�豸����CPU������ᣬ
            // ����5���ӵģ��Ǿ���Ҫ��ʼ��Ӧ������ָ��ѧϰ�ˡ�

            if (WAIT_LEARNING >= TimerCount) {
            // ------------ ���³���5�룬��ʼѧϰ���� ------------

                // ����8���ֳ�ʱ�������û���յ��κ��źţ��ͷ�������ѧϰ
                TimerCount = 160;
                // START_T0;
                // Learning_Mode = 1;

                // ����INT0�жϣ�����������ź�
                ENABLE_INT0;

#if __WITH_BEEP
                OUT_HIGH(IO_BEEP);
#endif

                while (TimerCount)
                {
                    //INV_BIT(LED);
                    //PORTB ^= _BV(LED);
                    //_delay_loop_2(40000);       // ����ѧϰ�ڼ䣬��˸LED

                    //�����յ������źź�����¸�TimerCount��ֵ2��
                    //�����������ж� IR_Ready ��Ϊ�˳�����
                    //ѧϰ�ڼ䣬�κΰ����¼���ѧϰ��ɶ����˳�ѧϰ״̬
                    //if (KEY_PRESSED || IR_Ready) break;
                }

                // SET_BIT(LED);         // ȷ��ѧϰ��Ϻ�LED��Ϩ���

                // Learning_Mode = 0;
                // T0��TimerCount����ʱ�Զ�ֹͣ������������رգ���������ѧϰ����
                // STOP_T0;
                // TimerCount = 1;



                if (IR_Ready) {
                    IR_Ready = 0;

                    // ��ѧϰ����ָ�����ݱ��浽eeprom
                    for (i=0; i<sizeof(_IRCMD); i++) {
                        // д��EEPROM����
                        eeprom_update_byte(EE_MARGIN + cur_key_index * sizeof(_IRCMD) + i,
                                        *((unsigned char *) &curCMD + i));

                        // ���µ�ǰRAM�������
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

            // ������
            // �̰���ťʱ������رն�ʱ��
            TimerCount = 1;       // ��Ҫ����Ϊ0,��ֹ --TimerCount ��� 0xFF
            ENABLE_INT0;          // ����INT0�жϣ�����������ź�
        }



    // ------------ ����ģʽ�£��жϽ����Ƿ�OK�� ------------

        if (IR_Ready) {

            DISABLE_INT0;     // ��ͣ���⣬�ž����ݱ����ٸ��ǵĿ�����
            IR_Ready = 0;


            for (i=0; i<TOTAL_OUTPUT_COUNT; i++) {

                // �ҵ���Ӧ��IO�˿ڣ�����͵�ƽ����
                if (curCMD.data == LearnedCMD[i].data) {

#if __WITH_BEEP
                    OUT_HIGH(IO_BEEP);
#endif
                    // IO�����ͣ�Ȼ���л�Ϊ���״̬
                    OUT_LOW(OutPinMask[i]);
                    SET_BIT(DDRB,OutPinMask[i]);

                    _delay_loop_2(OUT_PULSE_WIDTH);

                    // ���º������ߣ����л�����״̬
                    OUT_HIGH(OutPinMask[i]);
                    CLR_BIT(DDRB,OutPinMask[i]);

#if __WITH_BEEP
                    OUT_LOW(IO_BEEP);
#endif

                    break;
                }
            }

            ENABLE_INT0;      // �ָ�INT0�������������
        }


    }
}

/* end of file */
