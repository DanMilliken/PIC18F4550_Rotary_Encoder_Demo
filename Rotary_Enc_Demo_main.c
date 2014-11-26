/******************************************************************************
 *
 * PIC18F4550_Rotary_Encoder_Demo
 *
 * Author: Dan Milliken
 *
 * Date: 2014-11-26
 * 
 * Project: PIC18F4550_Rotary_Enc_Demo
 *
 * Description: Demonstrates reading a rotary encoder and using the value to
 * output a character value to a 16x2 LCD.
 *
 * License: Licensed under the Creative Commons Attribution-ShareAlike 4.0
 * International License (http://creativecommons.org/licenses/by-sa/4.0/)
 *
*******************************************************************************/

#if defined(__XC)
    #include <xc.h>         /* XC8 General Include File */
#elif defined(HI_TECH_C)
    #include <htc.h>        /* HiTech General Include File */
#endif

#include <stdint.h>        /* For uint8_t definition */
#include <stdio.h>         /* C standard IO */
#include <conio.h>         /* Console IO */
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>       /* For true/false definition */
#include <delays.h>
#include <portb.h>         /* Port B interrupt routines */

#define _XTAL_FREQ  20000000

#pragma config CPUDIV = OSC1_PLL2
#pragma config PLLDIV = 1
#pragma config USBDIV = 1
#pragma config LVP = OFF        // disable low voltage programming
#pragma config FCMEN = OFF      // disable fail safe clock monitor
#pragma config IESO = OFF       // disable internal/external oscillator switchover
#pragma config BOR = OFF        // disable brown out reset
#pragma config PWRT = ON        // enable power up timer
#pragma config WDT = OFF        // disable watchdog timer
#pragma config FOSC = HS        // external crystal
#pragma config PBADEN = OFF     // Port B use digital I/O
#pragma config DEBUG = ON       // Background debugger enable
#pragma config ICPRT = ON       // In-Circuit Debug/Programming enabled
// Turn on code protection
#pragma config CONFIG5L = 0xF
#pragma config CONFIG5H = 0xC0
#pragma config CONFIG6L = 0xF
#pragma config CONFIG6H = 0xE0
#pragma config CONFIG7L = 0xF
#pragma config CONFIG7H = 0x40

/* CTRL_PORT defines for LCD. */
#undef DATA_PORT
#undef TRIS_DATA_PORT
#undef E_PIN
#undef TRIS_E
#undef RW_PIN
#undef TRIS_RW
#undef RS_PIN
#undef TRIS_RS
#define DATA_PORT      		PORTA
#define TRIS_DATA_PORT 		TRISA
#define E_PIN    LATEbits.LATE0  		/* PORT for E  */
#define TRIS_E   TRISEbits.TRISE0    	/* TRIS for E  */
#define RW_PIN   LATEbits.LATE1   		/* PORT for RW */
#define TRIS_RW  TRISEbits.TRISE1    	/* TRIS for RW */
#define RS_PIN   LATEbits.LATE2   		/* PORT for RS */
#define TRIS_RS  TRISEbits.TRISE2    	/* TRIS for RS */

// data declaration prototypes
struct menu;

// function prototypes
void UART_write(unsigned char c);
void UART_init(void);
void timer2_init(void);
void interrupt ISR(void);
unsigned long GetSystemTime(void);

void DisplayMenu(struct menu *mnu);
void NavigateMenu(struct menu *mnu);
struct menu* GetUserSelection(struct menu *mnu);

void DisplayStringLCD();

// globals
unsigned long system_time = 0; // ms - SYSTEM RUNNING TIME LIMIT: 49 days
bool getchar_active = false;
unsigned char encoder_value = 0;
bool encoder_changed = false;

/* Options for menu system */
#define MAX_TITLE     100
#define MAX_SUBMENUS  10

struct menu
{
    char title[MAX_TITLE];
    void (*command)();
    short num_submenus;
    struct menu *submenu[MAX_SUBMENUS];
};

struct menu menu_lcd_control_display_string = { "Display a string", DisplayStringLCD, 0 };
struct menu menu_lcd_control = { "LCD Control", NULL, 1, { &menu_lcd_control_display_string } };
struct menu menu_main = { "Main Menu", NULL, 1, { &menu_lcd_control } };

/* delay functions for LCD */
void DelayFor18TCY(void) { _delay(18); };         // delay for 18 Tcy, 3.6us
void DelayPORXLCD(void) { __delay_ms(15); };      // delay at least 15ms
void DelayXLCD(void) { __delay_ms(5); };          // delay at least 5ms
#assert _XTAL_FREQ == 20000000                    // Delay45ms based on 20MHz clock
void Delay45ms(void) { Delay1KTCYx(225); };       // delay 45ms

void DisplayMenu(struct menu *mnu)
{
    char output[MAX_TITLE];

    strcpy(output, "\n");
    strncat(output, mnu->title, MAX_TITLE);
    strncat(output, "\n", MAX_TITLE);
    cputs(output);

    for(int i=0; i<mnu->num_submenus; i++)
    {
        strcpy(output, "");
        sprintf(output, "%d", i+1);
        strcat(output, ". ");
        strcat(output, mnu->submenu[i]->title);
        strcat(output, "\n");
        cputs(output);
    }
    return;
}

struct menu* GetUserSelection(struct menu *mnu)
{
    struct menu* new_menu = NULL;
    char selection[83];
    int choice = 0;

    do
    {
        cputs("Make a selection: ");
        cgets(selection);
        choice = atoi(selection);
    } while (!(choice >= 1 && choice <= mnu->num_submenus));

    new_menu = mnu->submenu[choice-1];

    return new_menu;
}

void NavigateMenu(struct menu *mnu)
{
    while(0 != mnu->num_submenus)
    {
        DisplayMenu(mnu);
        mnu = GetUserSelection(mnu);
    }
    mnu->command();
    return;
}

void EnterCriticalSection(void)
{
    INTCONbits.GIE = 0;
    return;
}

void ExitCriticalSection(void)
{
    INTCONbits.GIE = 1;
    return;
}

unsigned char BusyLCD(void)
{
    __delay_us(80);             // Must wait at least 80us after last instruction
                                // to check busy flag.
    EnterCriticalSection();
    RW_PIN = 1;                 // Set the control bits for read
    RS_PIN = 0;
    E_PIN = 1;                  // Clock in the command
    _delay(2);                  // Data setup time: Max 320ns Actual 400ns
    if(DATA_PORT&0x08)
    {
        _delay(1);              // Remainder of enable pulse width: Min 480ns Actual 600ns
        E_PIN = 0;              // Reset clock line
        _delay(3);              // Data hold time and enable cycle time: Total 1200ns
        E_PIN = 1;              // Clock out other nibble
        _delay(3);              // Data setup time: Max 320ns Actual 400ns
        E_PIN = 0;
        _delay(3);              // Data hold time and enable cycle time: Total 1200ns
        RW_PIN = 0;             // Reset control line
        ExitCriticalSection();
        return 1;               // Return TRUE
    }
    else                            // Busy bit is low
    {
        _delay(1);              // Remainder of enable pulse width: Min 480ns Actual 600ns
        E_PIN = 0;              // Reset clock line
        _delay(3);              // Data hold time and enable cycle time: Total 1200ns
        E_PIN = 1;              // Clock out other nibble
        _delay(3);              // Data setup time: Max 320ns Actual 400ns
        E_PIN = 0;
        _delay(3);              // Data hold time and enable cycle time: Total 1200ns
        RW_PIN = 0;             // Reset control line
        ExitCriticalSection();
        return 0;               // Return FALSE
    }
}

void WriteCmdLCD(unsigned char cmd)
{
    EnterCriticalSection();
    TRIS_DATA_PORT &= 0xf0;
    RS_PIN = 0;
    RW_PIN = 0;                     // Set control signals for command

    DATA_PORT &= 0xf0;
    DATA_PORT |= (cmd>>4)&0x0f;
    E_PIN = 1;                      // Clock command in
    DelayFor18TCY();
    E_PIN = 0;

    DATA_PORT &= 0xf0;
    DATA_PORT |= cmd&0x0f;
    DelayFor18TCY();
    E_PIN = 1;                      // Clock command in
    DelayFor18TCY();
    E_PIN = 0;

    TRIS_DATA_PORT |= 0x0f;
    ExitCriticalSection();

    return;
}

void WriteDataLCD(char data)
{
    EnterCriticalSection();
    TRIS_DATA_PORT &= 0xf0;
    RS_PIN = 1;                     // Set control bits
    RW_PIN = 0;

    DATA_PORT &= 0xf0;
    DATA_PORT |= ((data>>4)&0x0f);
    E_PIN = 1;                      // Clock nibble into LCD
    _delay(1);                      // Data setup time: Max 80ns Actual 200ns
    _delay(2);                      // Remainder of enable pulse width: Min 460ns Actual 600ns
    E_PIN = 0;
    _delay(3);                      // Data hold time and enable cycle time: Total 1200ns

    DATA_PORT &= 0xf0;
    DATA_PORT |= (data&0x0f);
    E_PIN = 1;                      // Clock nibble into LCD
    _delay(1);                      // Data setup time: Max 80ns Actual 200ns
    _delay(2);                      // Remainder of enable pulse width: Min 460ns Actual 600ns
    E_PIN = 0;
    _delay(3);                      // Data hold time and enable cycle time: Total 1200ns

    TRIS_DATA_PORT |= 0x0f;
    ExitCriticalSection();

    return;
}

void SetDDRamAddr(unsigned char DDaddr)
{
    EnterCriticalSection();
    TRIS_DATA_PORT &= 0xf0;                 // Make port output
    DATA_PORT &= 0xf0;                      // and write upper nibble
    DATA_PORT |= (((DDaddr | 0b10000000)>>4) & 0x0f);

    RW_PIN = 0;                             // Set control bits
    RS_PIN = 0;
    DelayFor18TCY();
    E_PIN = 1;                              // Clock the cmd and address in
    DelayFor18TCY();
    E_PIN = 0;
    DATA_PORT &= 0xf0;                      // Write lower nibble
    DATA_PORT |= (DDaddr&0x0f);
    DelayFor18TCY();
    E_PIN = 1;                              // Clock the cmd and address in
    DelayFor18TCY();
    E_PIN = 0;

    TRIS_DATA_PORT |= 0x0f;                 // Make port input
    ExitCriticalSection();

    return;
}

void InitLCD(unsigned char lcdtype)
{
    // The data bits must be either a 8-bit port or the upper or
    // lower 4-bits of a port. These pins are made into inputs
    DATA_PORT &= 0xf0;
    TRIS_DATA_PORT &= 0xF0;
    TRIS_RW = 0;                    // All control signals made outputs
    TRIS_RS = 0;
    TRIS_E = 0;
    RW_PIN = 0;                     // R/W pin made low
    RS_PIN = 0;                     // Register select pin made low
    E_PIN = 0;                      // Clock pin made low

    // Delay for 45ms to allow for LCD Power on reset
    Delay45ms();

    //-------------------reset procedure through software----------------------
    WriteCmdLCD(0x30);
    __delay_us(37);

    WriteCmdLCD(0x30);
    __delay_us(37);

    WriteCmdLCD(0x32);
    __delay_us(37);

    // Set data interface width, # lines, font
    while(BusyLCD());              // Wait if LCD busy
    WriteCmdLCD(lcdtype);          // Function set cmd

    // Turn the display on then off
    while(BusyLCD());              // Wait if LCD busy
    WriteCmdLCD(DOFF&CURSOR_OFF&BLINK_OFF);        // Display OFF/Blink OFF
    while(BusyLCD());              // Wait if LCD busy
    WriteCmdLCD(DON&CURSOR_ON&BLINK_ON);           // Display ON/Blink ON

    // Clear display
    while(BusyLCD());              // Wait if LCD busy
    WriteCmdLCD(0x01);             // Clear display

    // Set entry mode inc, no shift
    while(BusyLCD());              // Wait if LCD busy
    WriteCmdLCD(0b00000110);       // Shift cursor right, increment DRAM. Don't shift display.

    // Set DD Ram address to 0
    while(BusyLCD());              // Wait if LCD busy
    SetDDRamAddr(0x00);            // Set Display data ram address to 0

    return;
}

void ClearLCD()
{
    // Clear the LCD by writing "20H" to all DDRAM addresses
    while(BusyLCD());              // Wait if LCD busy
    SetDDRamAddr(0x00);            // Set Display data ram address to 0

    for(int i=0; i<16; i++)
    {
        while(BusyLCD());          // Wait if LCD busy
        WriteDataLCD(' ');         // Write character to LCD
    }

    while(BusyLCD());              // Wait if LCD busy
    SetDDRamAddr(0x40);            // Set Display data ram address to second line

    for(int i=0; i<16; i++)
    {
        while(BusyLCD());          // Wait if LCD busy
        WriteDataLCD(' ');         // Write character to LCD
    }

    while(BusyLCD());              // Wait if LCD busy
    SetDDRamAddr(0x00);            // Set Display data ram address to 0

    return;
}

void putsLCD(char *buffer)
{
    ClearLCD();

    while(*buffer)               // Write data to LCD up to null
    {
        while(BusyLCD());        // Wait while LCD is busy
        if(*buffer=='\n')
          SetDDRamAddr(0x40);    // Set Display data ram address to second line
        else
          WriteDataLCD(*buffer); // Write character to LCD
        buffer++;                // Increment buffer
    }

    return;
}

void LCD_Display_Encoder_Value(unsigned char value)
{
    /**** 
     * Display format:
    "Value: ' '"
    "0xFF: 0b00000000"
     ****/

    // 64 characters per line + newline + null terminator
    unsigned char message[66];

    memset(message,0,66);

    if(0==value)
        sprintf(message,"Value: null\n0x%2X: 0b", value);
    else
        sprintf(message,"Value: '%c'\n0x%2X: 0b", value,value);

    for(int i=7; i>=0; i--)
        if((value >> i)&1 == 1)
            strcat(message,"1");
        else
            strcat(message,"0");

    putsLCD(message);

    return;
}

int main(void)
{
    INTCONbits.PEIE = 0; // enable peripheral interrupts
    INTCONbits.GIE = 0;  // enable interrupts

    UART_init();   // initialize the UART module
    printf("\n*** System startup ***\n");

    timer2_init(); // initialize the system time
    printf("%ul: System clock started\n", GetSystemTime());

    InitLCD(FOUR_BIT);    // initialize the LCD: 4-bit, 2 line, 5x11 dots per character
    printf("%ul: LCD initialized.\n", GetSystemTime());

    INTCONbits.PEIE = 0; // enable peripheral interrupts
    INTCONbits.GIE = 0;  // enable interrupts

    printf("%ul: Opening port B.\n", GetSystemTime());
    OpenPORTB(PORTB_CHANGE_INT_ON & PORTB_PULLUPS_ON);
    printf("%ul: Port B Open. Opening INT...\n", GetSystemTime());
    OpenRB0INT(PORTB_CHANGE_INT_ON & FALLING_EDGE_INT & PORTB_PULLUPS_ON);
    printf("%ul: Port B interrupts initialized.\n", GetSystemTime());

    RCONbits.IPEN = 0; // disable priority levels.
    INTCONbits.RBIE = 0;
    INTCONbits.PEIE = 1; // enable peripheral interrupts
    INTCONbits.GIE = 1;  // enable interrupts

    LCD_Display_Encoder_Value(encoder_value);

    while(1)
    {
        if(true == encoder_changed)
        {
            _delay3(833);     // delay 500us
            encoder_value = PORTBbits.RB1 ? encoder_value + 1 : encoder_value - 1;
            INTCONbits.INT0IE=0;
            LCD_Display_Encoder_Value(encoder_value);
            INTCONbits.INT0IE=1;
            Delay10KTCYx(75);  // delay 150ms
            encoder_changed = false;
        }
    }

    while(1)
    {
        NavigateMenu(&menu_main);
    }

}

void DisplayStringLCD()
{
    char disp_string[16];

    cputs("Enter a string to display: ");
    cgets(disp_string);
    putsLCD(disp_string);

    return;
}

void timer2_init(void)
{
    INTCONbits.PEIE = 1; // enable peripheral interrupts

    T2CONbits.T2CKPS1 = 0;   // Prescalar = 1:4
    T2CONbits.T2CKPS0 = 1;

    T2CONbits.TOUTPS3 = 1;   // Postscalar = 1:9
    T2CONbits.TOUTPS2 = 0;
    T2CONbits.TOUTPS1 = 0;
    T2CONbits.TOUTPS0 = 0;

    PR2 = 139;               // Period register 139

    T2CONbits.TMR2ON = 1;    // Timer2 ON

    PIR1bits.TMR2IF = 0;     // Timer2 flag clear
    PIE1bits.TMR2IE = 0;     // Timer2 interrupt enable
}

void UART_init(void)
{
    TXSTAbits.BRGH = 1; // high baud rate
    TXSTAbits.SYNC = 0; // asynchronous mode
    TXSTAbits.TX9  = 0; // 8-bit transmission
    RCSTAbits.CREN = 1; // continuous receive enable

    SPBRG = 129;        // 9600 baud @ 20MHz

    PIE1bits.RCIE  = 1;
    RCSTAbits.SPEN = 1;
    TXSTAbits.TXEN = 1; // enable transmitter

    return;
}

void UART_write(unsigned char c) {
    while (!TXSTAbits.TRMT);
    TXREG = c;

    return;
}

// Override putch called by printf
void putch(unsigned char byte)
{
    UART_write(byte);
    if ('\n' == byte)
        UART_write('\r');
    return;
}

unsigned char getch()
{
    getchar_active = true;
    /* retrieve one byte */
    while(getchar_active) /* set when register is not empty */
        continue;
    return RCREG;
}

unsigned char getche(void)
{
    unsigned char c;
    putch(c = getch());
    return c;
}

unsigned long GetSystemTime()
{
    unsigned long TIME;
    INTCONbits.TMR0IE = 0;   // Timer0 interrupt disable
    TIME = system_time;
    INTCONbits.TMR0IE = 1;   // Timer0 interrupt enable
    return TIME;
}

void ProcessUART(unsigned char byte)
{
    return;
}

//#define DEBUG_INTERRUPTS

void int_debug(const char * msg)
{
#ifdef DEBUG_INTERRUPTS
    printf(msg);
#endif
    return;
}

void interrupt ISR(void)
{
    int_debug("Interrupt! Source(s): ");

    // AUSART Receive Interrupt Flag bit
    if (RCIE && RCIF)
    {
        int_debug("RCIF ");
        getchar_active = false;
        RCIF = 0;
        ProcessUART(RCREG);
    }

    // Timer 2 overflow interrupt
    if (1 == PIR1bits.TMR2IF)
    {
        int_debug("TMR2IF ");
        PIR1bits.TMR2IF = 0;
        system_time = system_time + 1;
    }

    if (1 == INTCONbits.INT0IF)
    {
        int_debug("INT0IF ");
        INTCONbits.INT0IF = 0;
        if(!encoder_changed) // Check if still processing the last interrupt
        {
            encoder_changed = true;
        }
    }

    if (1 == INTCONbits.RBIF)
        int_debug("RBIF ");
    if (1 == INTCON3bits.INT1IF)
        int_debug("INT1IF ");
    if (1 == INTCON3bits.INT2IF)
        int_debug("INT2IF ");
    if (1 == PIR1bits.SPPIF)
        int_debug("SPPIF ");
    if (1 == PIR1bits.ADIF)
        int_debug("ADIF ");
    if (1 == PIR1bits.TXIF)
        int_debug("TXIF ");
    if (1 == PIR1bits.SSPIF)
        int_debug("SSPIF ");
    if (1 == PIR1bits.CCP1IF)
        int_debug("CCP1IF ");
    if (1 == PIR1bits.TMR1IF)
        int_debug("TMR1IF ");
    if (1 == PIR2bits.OSCFIF)
        int_debug("OSCFIF ");
    if (1 == PIR2bits.CMIF)
        int_debug("CMIF ");
    if (1 == PIR2bits.USBIF)
        int_debug("USBIF ");
    if (1 == PIR2bits.EEIF)
        int_debug("EEIF ");
    if (1 == PIR2bits.BCLIF)
        int_debug("BCLIF ");
    if (1 == PIR2bits.HLVDIF)
        int_debug("HLVDIF ");
    if (1 == PIR2bits.TMR3IF)
        int_debug("TMR3IF ");
    if (1 == PIR2bits.CCP2IF)
        int_debug("CCP2IF ");

    int_debug("\n");

    return;
}
