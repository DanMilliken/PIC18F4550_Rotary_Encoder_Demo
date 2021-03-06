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
void uart_write(unsigned char c);
void uart_init(void);
void timer2_init(void);
void interrupt isr(void);
unsigned long get_system_time(void);

/* Options for menu system */
#define MAX_TITLE     100
#define MAX_SUBMENUS  10

// menu system functions
void menu_display(struct menu *mnu);
void menu_navigate(struct menu *mnu);
struct menu* menu_get_user_selection(struct menu *mnu);

// user functions for menu system
void display_string_on_lcd();

// globals
unsigned long system_time = 0; // ms - SYSTEM RUNNING TIME LIMIT: 49 days
bool getchar_active = false;
unsigned char encoder_value = 0;
bool encoder_changed = false;

struct menu
{
    char title[MAX_TITLE];
    void (*command)();
    short num_submenus;
    struct menu *submenu[MAX_SUBMENUS];
};

struct menu menu_lcd_control_display_string = { "Display a string", display_string_on_lcd, 0 };
struct menu menu_lcd_control = { "LCD Control", NULL, 1, { &menu_lcd_control_display_string } };
struct menu menu_main = { "Main Menu", NULL, 1, { &menu_lcd_control } };

/* delay functions for LCD */
#assert _XTAL_FREQ == 20000000                    // lcd_delay_100_ms is based on 20MHz clock
void lcd_delay_100_ms(void) { Delay1KTCYx(500); };       // delay 100ms

void menu_display(struct menu *mnu)
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

struct menu* menu_get_user_selection(struct menu *mnu)
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

void menu_navigate(struct menu *mnu)
{
    while(0 != mnu->num_submenus)
    {
        menu_display(mnu);
        mnu = menu_get_user_selection(mnu);
    }
    mnu->command();
    return;
}

void enter_critical_section(void)
{
    INTCONbits.GIE = 0;
    return;
}

void exit_critical_section(void)
{
    INTCONbits.GIE = 1;
    return;
}

unsigned char lcd_busy_check(void)
{
    __delay_us(80);             // Must wait at least 80us after last instruction
                                // to check busy flag.
    enter_critical_section();
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
        exit_critical_section();
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
        exit_critical_section();
        return 0;               // Return FALSE
    }
}

void lcd_write_cmd(unsigned char cmd)
{
    enter_critical_section();
    TRIS_DATA_PORT &= 0xf0;
    RS_PIN = 0;
    RW_PIN = 0;                     // Set control signals for command

    DATA_PORT &= 0xf0;
    DATA_PORT |= (cmd>>4)&0x0f;
    E_PIN = 1;                      // Clock command in
    _delay(1);                      // Data setup time: Max 80ns Actual 200ns
    _delay(2);                      // Remainder of enable pulse width: Min 460ns Actual 600ns
    E_PIN = 0;
    _delay(3);                      // Data hold time and enable cycle time: Total 1200ns

    DATA_PORT &= 0xf0;
    DATA_PORT |= cmd&0x0f;
    E_PIN = 1;                      // Clock command in
    _delay(1);                      // Data setup time: Max 80ns Actual 200ns
    _delay(2);                      // Remainder of enable pulse width: Min 460ns Actual 600ns
    E_PIN = 0;
    _delay(3);                      // Data hold time and enable cycle time: Total 1200ns

    TRIS_DATA_PORT |= 0x0f;
    exit_critical_section();

    return;
}

void lcd_set_ddram_address(unsigned char addr)
{
    enter_critical_section();
    TRIS_DATA_PORT &= 0xf0;                 // Make port output
    RW_PIN = 0;                             // Set control bits
    RS_PIN = 0;

    DATA_PORT &= 0xf0;                      // and write upper nibble
    DATA_PORT |= (((addr | 0b10000000)>>4) & 0x0f);
    E_PIN = 1;                              // Clock the cmd and address in
    _delay(1);                      // Data setup time: Max 80ns Actual 200ns
    _delay(2);                      // Remainder of enable pulse width: Min 460ns Actual 600ns
    E_PIN = 0;
    _delay(3);                      // Data hold time and enable cycle time: Total 1200ns

    DATA_PORT &= 0xf0;                      // Write lower nibble
    DATA_PORT |= (addr&0x0f);
    E_PIN = 1;                              // Clock the cmd and address in
    _delay(1);                      // Data setup time: Max 80ns Actual 200ns
    _delay(2);                      // Remainder of enable pulse width: Min 460ns Actual 600ns
    E_PIN = 0;
    _delay(3);                      // Data hold time and enable cycle time: Total 1200ns

    TRIS_DATA_PORT |= 0x0f;                 // Make port input
    exit_critical_section();

    return;
}

void lcd_write_data(char data)
{
    enter_critical_section();
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
    exit_critical_section();

    return;
}

void lcd_init(unsigned char lcdtype)
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

    // Delay for 100ms to allow for LCD Power on reset
    lcd_delay_100_ms();

    //-------------------reset procedure through software----------------------
    lcd_write_cmd(0x30);
    __delay_ms(5);      // Delay more than 4.1ms

    lcd_write_cmd(0x30);
    __delay_us(100);     // Delay more than 100us

    lcd_write_cmd(0x30);
    __delay_us(100);     // Delay more than 100us

    lcd_write_cmd(0x20);
    __delay_us(100);     // Delay more than 100us

    // Set data interface width, # lines, font
    while(lcd_busy_check());              // Wait if LCD busy
    lcd_write_cmd(lcdtype);          // Function set cmd

    // Turn the display on then off
    while(lcd_busy_check());              // Wait if LCD busy
    lcd_write_cmd(DOFF&CURSOR_OFF&BLINK_OFF);        // Display OFF/Blink OFF
    while(lcd_busy_check());              // Wait if LCD busy
    lcd_write_cmd(DON&CURSOR_ON&BLINK_ON);           // Display ON/Blink ON

    // Clear display
    while(lcd_busy_check());              // Wait if LCD busy
    lcd_write_cmd(0x01);             // Clear display

    // Set entry mode inc, no shift
    while(lcd_busy_check());              // Wait if LCD busy
    lcd_write_cmd(0b00000110);       // Shift cursor right, increment DRAM. Don't shift display.

    // Set DD Ram address to 0
    while(lcd_busy_check());              // Wait if LCD busy
    lcd_set_ddram_address(0x00);            // Set Display data ram address to 0

    return;
}

void lcd_clear_display()
{
    // Clear the LCD by writing "20H" to all DDRAM addresses
    while(lcd_busy_check());              // Wait if LCD busy
    lcd_set_ddram_address(0x00);            // Set Display data ram address to 0

    for(int i=0; i<16; i++)
    {
        while(lcd_busy_check());          // Wait if LCD busy
        lcd_write_data(' ');         // Write character to LCD
    }

    while(lcd_busy_check());              // Wait if LCD busy
    lcd_set_ddram_address(0x40);            // Set Display data ram address to second line

    for(int i=0; i<16; i++)
    {
        while(lcd_busy_check());          // Wait if LCD busy
        lcd_write_data(' ');         // Write character to LCD
    }

    while(lcd_busy_check());              // Wait if LCD busy
    lcd_set_ddram_address(0x00);            // Set Display data ram address to 0

    return;
}

void lcd_put_string(char *buffer)
{
    lcd_clear_display();

    while(*buffer)               // Write data to LCD up to null
    {
        while(lcd_busy_check());        // Wait while LCD is busy
        if(*buffer=='\n')
          lcd_set_ddram_address(0x40);    // Set Display data ram address to second line
        else
          lcd_write_data(*buffer); // Write character to LCD
        buffer++;                // Increment buffer
    }

    return;
}

void display_encoder_value(unsigned char value)
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

    lcd_put_string(message);

    return;
}

void int0_init()
{
    TRISB |= 0b00000001;     // Set RB0 to input
    ADCON1bits.PCFG = 0x0F;  // AN12:AN0 to digital
    INTCON2bits.INTEDG0 = 0; // interrupt on falling edge
    INTCONbits.INT0IF = 0;   // clear the flag
    INTCONbits.INT0IE = 1;

    return;
}
int main(void)
{
    INTCONbits.PEIE = 0; // enable peripheral interrupts
    INTCONbits.GIE = 0;  // enable interrupts

    uart_init();   // initialize the UART module
    printf("\n*** System startup ***\n");

    timer2_init(); // initialize the system time
    printf("%ul: System clock started\n", get_system_time());

    lcd_init(FOUR_BIT);    // initialize the LCD: 4-bit, 2 line, 5x11 dots per character
    printf("%ul: LCD initialized.\n", get_system_time());

    INTCONbits.PEIE = 0; // enable peripheral interrupts
    INTCONbits.GIE = 0;  // enable interrupts

    int0_init();           // initialize INT0 interrupt
    TRISB |= 0b00000010;   // Set RB1 to input
    printf("%ul: Port B interrupts initialized.\n", get_system_time());

    RCONbits.IPEN = 0; // disable priority levels.
    INTCONbits.RBIE = 0;
    INTCONbits.PEIE = 1; // enable peripheral interrupts
    INTCONbits.GIE = 1;  // enable interrupts

    display_encoder_value(encoder_value);

    while(1)
    {
        if(true == encoder_changed)
        {
            display_encoder_value(encoder_value);
            encoder_changed = false;
        }
    }

    while(1)
    {
        menu_navigate(&menu_main);
    }

}

void display_string_on_lcd()
{
    char disp_string[16];

    cputs("Enter a string to display: ");
    cgets(disp_string);
    lcd_put_string(disp_string);

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

void uart_init(void)
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

void uart_write(unsigned char c) {
    while (!TXSTAbits.TRMT);
    TXREG = c;

    return;
}

// Override putch called by printf
void putch(unsigned char byte)
{
    uart_write(byte);
    if ('\n' == byte)
        uart_write('\r');
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

unsigned long get_system_time()
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

void interrupt isr(void)
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
        INTCONbits.TMR0IE = 0;
        PIR1bits.TMR2IF = 0;
        system_time += 1;
        INTCONbits.TMR0IE = 1;
    }

    // INT0 (RB0) external interrupt
    if (1 == INTCONbits.INT0IF)
    {
        int_debug("INT0IF ");
        INTCONbits.INT0IE = 0;
        INTCONbits.INT0IF = 0;
        encoder_value += PORTBbits.RB1 ? 1 : -1;
        encoder_changed = true;
        INTCONbits.INT0IE = 1;
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
