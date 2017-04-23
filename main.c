/*
 * File:   main.c
 * Author: Luke
 *
 * Created on 6. Dezember 2016, 20:26
 */

#include <xc.h>
#include <stdint.h>
#include "morse.h"


// PIC16F18325 Configuration Bit Settings
// CONFIG1
#pragma config FEXTOSC = OFF    // FEXTOSC External Oscillator mode Selection bits (Oscillator not enabled)
//#pragma config RSTOSC = HFINT32 // Power-up default value for COSC bits (HFINTOSC with 2x PLL (32MHz))
#pragma config RSTOSC = HFINT1  // Power-up default value for COSC bits (HFINTOSC (1MHz))
#pragma config CLKOUTEN = OFF   // Clock Out Enable bit (CLKOUT function is disabled; I/O or oscillator function on OSC2)
#pragma config CSWEN = ON       // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)

// CONFIG2
#pragma config MCLRE = ON       // Master Clear Enable bit (MCLR/VPP pin function is MCLR; Weak pull-up enabled)
#pragma config PWRTE = ON       // Power-up Timer Enable bit (PWRT enabled)
#pragma config WDTE = ON        // Watchdog Timer Enable bits (WDT enabled, SWDTEN is ignored)
#pragma config LPBOREN = OFF    // Low-power BOR enable bit (ULPBOR disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bits (Brown-out Reset enabled, SBOREN bit ignored)
#pragma config BORV = HIGH      // Brown-out Reset Voltage selection bit (Brown-out voltage (Vbor) set to 2.7V)
#pragma config PPS1WAY = OFF    // PPSLOCK bit One-Way Set Enable bit (The PPSLOCK bit can be set and cleared repeatedly (subject to the unlock sequence))
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable bit (Stack Overflow or Underflow will cause a Reset)
#pragma config DEBUG = OFF      // Debugger enable bit (Background debugger disabled)

// CONFIG3
#pragma config WRT = ALL        // User NVM self-write protection bits (0000h to 1FFFh write protected, no addresses may be modified)
#pragma config LVP = ON         // Low Voltage Programming Enable bit (Low Voltage programming enabled. MCLR/VPP pin function is MCLR. MCLRE configuration bit is ignored.)

// CONFIG4
#pragma config CP = ON          // User NVM Program Memory Code Protection bit (User NVM code protection enabled)
#pragma config CPD = ON         // Data NVM Memory Code Protection bit (Data NVM code protection enabled)

//CPU Frequency
#define _XTAL_FREQ 1000000

//Sum of 8 temperature reading with an LMT86 temperature sensor, a 2.048V voltage reference and 10 bits of resolution
#define TEMPERATURE_20 7533
#define TEMPERATURE_21 7489
#define TEMPERATURE_22 7449
#define TEMPERATURE_23 7405
#define TEMPERATURE_24 7361
#define TEMPERATURE_25 7317
#define TEMPERATURE_26 7273
#define TEMPERATURE_27 7233
#define TEMPERATURE_28 7189
#define TEMPERATURE_29 7145
#define TEMPERATURE_30 7101
#define TEMPERATURE_31 7057
#define TEMPERATURE_32 7017
#define TEMPERATURE_33 6973
#define TEMPERATURE_34 6929
#define TEMPERATURE_35 6885
#define TEMPERATURE_36 6841
#define TEMPERATURE_37 6797
#define TEMPERATURE_38 6753
#define TEMPERATURE_39 6709
#define TEMPERATURE_40 6665
#define TEMPERATURE_41 6622
#define TEMPERATURE_42 6578
#define TEMPERATURE_43 6534
#define TEMPERATURE_44 6490
#define TEMPERATURE_45 6446
#define TEMPERATURE_46 6402
#define TEMPERATURE_47 6358
#define TEMPERATURE_48 6314
#define TEMPERATURE_49 6270
#define TEMPERATURE_50 6226
#define TEMPERATURE_51 6182
#define TEMPERATURE_52 6138
#define TEMPERATURE_53 6094
#define TEMPERATURE_54 6050
#define TEMPERATURE_55 6006
#define TEMPERATURE_56 5962
#define TEMPERATURE_57 5918
#define TEMPERATURE_58 5874
#define TEMPERATURE_59 5830
#define TEMPERATURE_60 5786

#define FAN_TURN_ON_TEMPERATURE TEMPERATURE_32
#define FAN_TURN_OFF_TEMPERATURE TEMPERATURE_30
#define FAN_STARTUP_PERIOD 8
    
#define LED_TRIS TRISAbits.TRISA4
#define FAN_TRIS TRISCbits.TRISC5
#define KEY_POS_TRIS TRISCbits.TRISC4
#define KEY_NEG_TRIS TRISCbits.TRISC3

#define LED_MASK 0b00010000
#define FAN_MASK 0b00100000
#define KEY_POS_MASK 0b00010000
#define KEY_NEG_MASK 0b00001000

typedef enum adc_channel
{
    ADC_CHANNEL_TEMPERATURE,
    ADC_CHANNEL_SPEED
} adc_channel_t;

//Function prototypes
static void led_on(void);
static void led_off(void);
static void led_toggle(void);
static void fan_on(void);
static void fan_off(void);
static void key_on(void);
static void key_off(void);
static void adc_init(void);
static void adc_set_channel(adc_channel_t channel);
static inline void adc_start_conversion(void);
static uint16_t adc_get_result();

static void timer_init(void);
static void init();

//Global variables
uint8_t portA;
uint8_t portC;
uint16_t position;
uint8_t interrupt_count;
uint16_t adc_sum_temperature;
uint16_t adc_sum_pot;
uint16_t fan_startup;

const char text[] = "HB9MPU JN47HD 10368050KHZ--__________--";
uint8_t bitstring[BITSTRING_LENGTH_BYTES];
uint16_t bitlength;

static void led_on(void)
{
    portA |= LED_MASK;
    PORTA = portA;
}

static void led_off(void)
{
    portA &= ~LED_MASK;
    PORTA = portA;
}

static void led_toggle(void)
{
    portA ^= LED_MASK;
    PORTA = portA;
}

static void fan_on(void)
{
    portC |= FAN_MASK;
    PORTC = portC;
}

static void fan_off(void)
{
    portC &= ~FAN_MASK;
    PORTC = portC;
}

static void key_on(void)
{
    portC |= KEY_POS_MASK;
    portC &= ~KEY_NEG_MASK;
    PORTC = portC;
}

static void key_off(void)
{
    portC &= ~KEY_POS_MASK;
    portC |= KEY_NEG_MASK;
    PORTC = portC;
}

void interrupt ISR(void)
{ 
    uint16_t adc_value;
    
    //Take care of morse code every 4th time
    if((interrupt_count&0b11)==0b11)
    {
        //Set output
        if(getBit(&bitstring[0], position))
        {
            key_on();
        }
        else
        {
            key_off();
        }
        //Increment position
        ++position;
        if(position==bitlength)
        {
            position = 0;
        }
        //Set period
        PR2 = 145 - (adc_sum_pot>>5);
        adc_sum_pot = 0;
    }
    
    //Take care of fan every 16th time
    if((interrupt_count&0b1111)==0b1001)
    {
        if(fan_startup)
        {
            --fan_startup;
        }
        else
        {
            //Turn fan on or off
            if(adc_sum_temperature<FAN_TURN_ON_TEMPERATURE)
                fan_on();
            if(adc_sum_temperature>FAN_TURN_OFF_TEMPERATURE)
                fan_off();
        }
        adc_sum_temperature = 0;    
    }
   
    
    //Iterate between measuring temperature and speed
    if(interrupt_count&0b1)
    {
        adc_start_conversion();
        adc_sum_pot += adc_get_result();
        adc_set_channel(ADC_CHANNEL_TEMPERATURE);
    }
    else
    {
        adc_start_conversion();
        adc_sum_temperature += adc_get_result();
        adc_set_channel(ADC_CHANNEL_SPEED);
    }

    //Increment interrupt count
    ++interrupt_count;
    
    //Clear interrupt flag
    PIR1bits.TMR2IF = 0;
}

static void timer_init(void)
{
    //Prescaler = 64
    T2CONbits.T2CKPS = 0b11;
    //Postscaler = 1
    T2CONbits.T2OUTPS = 0b0000;
    //Period = 122
    // 1 = 1MHz / 4 / 64 / 4 = 1.024ms
    PR2 = 122;
    //Zero timer 2
    TMR2 = 0x00;
    //Clear Interrupt flag
    PIR1bits.TMR2IF = 0;
    //Enable general interrupts
    INTCONbits.GIE = 1;
    //Enable peripheral interrupts
    INTCONbits.PEIE = 1;
    //Enable timer2 interrupts
    PIE1bits.TMR2IE = 1;
    //Turn timer 2 on
    T2CONbits.TMR2ON = 1;    
}

static void adc_init(void)
{
    //Enable fixed voltage reference
    FVRCONbits.FVREN = 1;
    //Reference voltage 2.048V
    FVRCONbits.ADFVR = 0b10;
    //Turn ADC on
    ADCON0bits.ADON = 1;
    //Conversion Clock = fosc/2
    ADCON1bits.ADCS = 0b000;
    //Negative reference = AVSS RA2 (pin 11)
    ADCON1bits.ADNREF = 1;
    //Output format right-justified
    ADCON1bits.ADFM = 1;
}

static void adc_set_channel(adc_channel_t channel)
{
    switch(channel)
    {
        case ADC_CHANNEL_TEMPERATURE:
            //ANC0 (pin 10) as source
            ADCON0bits.CHS = 0b010000;
            //Positive reference = Fixed Voltage Reference
            ADCON1bits.ADPREF = 0b11;
            break;
        case ADC_CHANNEL_SPEED:
            //ANC1 (pin 9) as source
            ADCON0bits.CHS = 0b010001;
            //Positive reference = VDD
            ADCON1bits.ADPREF = 0b00;
            break;
    }
}

static inline void adc_start_conversion(void)
{
    //Start a conversion
    ADCON0bits.GO = 1;
}

static uint16_t adc_get_result()
{
    uint16_t adc_value;
    //Wait for the result to be ready
    while(ADCON0bits.GO_nDONE)
    {
        __delay_us(25);
    }
    //Read result of ADC conversion
    adc_value = ADRESH;
    adc_value <<= 8;
    adc_value |= ADRESL;
    return adc_value;
}

static void init(void)
{
    //Configure watchdog timer (WDT)
    // 1 second timeout period
    WDTCONbits.WDTPS4 = 0;
    WDTCONbits.WDTPS3 = 1;
    WDTCONbits.WDTPS2 = 0;
    WDTCONbits.WDTPS1 = 1;
    WDTCONbits.WDTPS0 = 0;
    //Clear WDT
    CLRWDT();
    
    //LED_EN as output
    LED_TRIS = 0;
    //Turn LED on
    led_on();
    
    //FAN_EN as output
    FAN_TRIS = 0;
    //Turn fan on
    fan_on();
    fan_startup = FAN_STARTUP_PERIOD;
    
    //KEY_SIG+ as output
    KEY_POS_TRIS = 0;
    
    //KEY_SIG- as output
    KEY_NEG_TRIS = 0;
    
    //Temperature as analog input
    TRISCbits.TRISC0 = 1;
    ANSELCbits.ANSC0 = 1;
    
    //Pot as analog input
    TRISCbits.TRISC1 = 1;
    ANSELCbits.ANSC1 = 1;
    
    //Prepare morse code
    bitlength = compile(&text[0], &bitstring[0]);
    position = 0;
    
    //General ADC setup
    adc_init();
    adc_set_channel(ADC_CHANNEL_SPEED);

    //Set up timer 2 to generate periodic interrupts
    timer_init();
}

void main(void) 
{
    init();

    while(1)
    {
        __delay_ms(200);
        CLRWDT(); 
    }   
    return;
}
