/*
 * ps2mac.c
 *
 *  This program is the interface code for AVR with a PS2 keyboard.
 *  It implements a PS2 keyboard interface and a Mac Plus serial keyboard interface.
 *  The code configures the keyboard, accepts scan codes, converts the AT scan codes
 *  to make/break codes for the Mac.
 *
 * +-----+            +-----+            +----------+
 * |     |            |     |            |          |
 * |     |            |     |            |          |
 * |     |            |     |            |          |
 * |     +--< Data >--+     +--< Data >--+ PS2      |
 * | Mac |            | AVR |            | keyboard |
 * |     +--< CLK ]---+     +--< CLK ]---+          |
 * |     |            |     |            |          |
 * |     |            |     |            |          |
 * |     |            |     |            |          |
 * +-----+            +-----+            +----------+
 *
 *
 * ATtiny85 AVR IO
 *
 * | Function   | AVR | Pin | I/O     |
 * |------------|-----|-----|---------|
 * | Reset      | PB5 | 1   | Pull up |
 * | PS2 clock  | PB3 | 2   | in/out  |
 * | PS2 data   | PB4 | 3   | in/out  |
 * | KBD Clock  | PB0 | 5   | out     |
 * | KBD data   | PB1 | 6   | in/out  |
 * | Test point | PB2 | 7   | out     |
 *
 * Port B bit assignment
 *
 *  b7 b6 b5 b4 b3 b2 b1 b0
 *        |  |  |  |  |  |
 *        |  |  |  |  |  +--- 'o' Mac keyboard clock
 *        |  |  |  |  +------ 'i' Mac keyboard data
 *        |  |  |  +--------- 'o' Test point
 *        |  |  +------------ 'i' PS2 clock
 *        |  +--------------- 'i' PS2 data
 *        +------------------ 'i' ^Reset
 *
 * Note: all references to data sheet are for ATtine85 Rev. 2586Q–AVR–08/2013
 *
 * Data transfer schema uses a double buffer: input buffer from PS2 holding scan codes,
 * output buffer holding converted keyboard make/break codes for the Mac.
 * The main loop reads from the PS2 buffer, processes the scan codes, and stores them in
 * the output keyboard buffer.
 *
 * [Mac Linux M68000](http://www.mac.linux-m68k.org/devel/plushw.php)
 *
 * TODO:
 * 1) keyboard error handling and recovery
 *
 */

#include    <stdint.h>
#include    <stdlib.h>

#include    <avr/io.h>
#include    <avr/interrupt.h>
#include    <avr/wdt.h>
#include    <util/delay.h>

// IO port B initialization
#define     PB_DDR_INIT     0b00000101  // Port data direction
#define     PB_PUP_INIT     0b00100010  // Port input pin pull-up in Mac data line
#define     PB_INIT         0b00000001  // Port initial values

#define     TEST_POINT      0b00000100  // Set and reset mask

// Pin change interrupt setting
#define     GIMSK_INIT      0x20        // Enable pin change sensing on PB
#define     PCMSK_INIT      0b00001000  // Enable pin change interrupt on PB3
#define     PCINT_PB3       PCMSK_INIT  // Pin change interrupt on PB3

// Timer0 initialization
/* Using Timer0 for long time-out value measurements
 * in the 100s of milliseconds.
 * System clock = 8MHz
 * Divide Timer0 scaler by 1,024 = 7.8KHz (128uSec clock)
 * Timer0 overflow occurs every ~33mSec
 *
 * ~200mSec keyboard time out with a count of 6
 *
 */
#define     TCCR0A_INIT     0b00000000  // Normal mode, no compare match
#define     TCCR0B_INIT     0b00000101  // Mode-0 timer, Clk/1024
#define     TIMSK_INIT      0b00000010  // Enable Timer0 overflow interrupt

#define     KBDRD_TOV       6           // See explanation above

// PS2 control line masks
#define     PS2_CLOCK       0b00001000
#define     PS2_DATA        0b00010000

// Buffers
#define     PS2_BUFF_SIZE   32          // PS2 input buffer
#define     KEY_BUFF_SIZE   32          // Key code output buffer

// Host to Keyboard commands
#define     PS2_HK_LEDS     0xED        // Set Status Indicators, next byte LED bit mask
#define     PS2_HK_ECHO     0xEE        // Echo
#define     PS2_HK_INVALID  0xEF        // Invalid Command
#define     PS2_HK_ALTCODE  0xF0        // Select Alternate Scan Codes, next byte Scan code set
#define     PS2_HK_INVALID2 0xF1        // Invalid Command
#define     PS2_HK_TMDELAY  0xF3        // Set Typematic Rate/Delay, next byte Encoded rate/delay
#define     PS2_HK_ENABLE   0xF4        // Enable
#define     PS2_HK_DISABLE  0xF5        // Default Disable
#define     PS2_HK_DEFAULT  0xF6        // Set Default
#define     PS2_HK_SET1     0xF7        // Set All Keys - Typematic
#define     PS2_HK_SET2     0xF8        // Set All Keys - Make/Break
#define     PS2_HK_SET3     0xF8        // Set All Keys - Make
#define     PS2_HK_SET4     0xFA        // Set All Keys - Typematic/Make/Break
#define     PS2_HK_SET5     0xFB        // Set All Key Type - Typematic, next byte Scan code
#define     PS2_HK_SET6     0xFC        // Set All Key Type - Make/Break, next byte Scan code
#define     PS2_HK_SET7     0xFD        // Set All Key Type - Make, next byte Scan code
#define     PS2_HK_RESEND   0xFE        // Resend
#define     PS2_HK_RESET    0xFF        // Reset

#define     PS2_HK_SCRLOCK  1           // Scroll lock - mask 1 on/0 off
#define     PS2_HK_NUMLOCK  2           // Num lock   - mask 1 on/0 off
#define     PS2_HK_CAPSLOCK 4           // Caps lock  - mask 1 on/0 off

#define     PS2_HK_TYPEMAT  0b01111111  // 1Sec delay, 2Hz repetition

// Keyboard to Host commands
#define     PS2_KH_ERR23    0x00        // Key Detection Error/Overrun (Code Sets 2 and 3)
#define     PS2_KH_BATOK    0xAA        // BAT Completion Code
#define     PS2_KH_ERR      0xFC        // BAT Failure Code
#define     PS2_KH_ECHO     0xEE        // Echo
#define     PS2_KH_BREAK    0xF0        // Break (key-up)
#define     PS2_KH_ACK      0xFA        // Acknowledge (ACK)
#define     PS2_KH_RESEND   0xFE        // Resend
#define     PS2_KH_ERR1     0xFF        // Key Detection Error/Overrun (Code Set 1)

#define     PS2_SCAN_CAPS   0x3a        // Caps lock scan code
#define     PS2_SCAN_SCROLL 0x46        // Scroll lock scan code
#define     PS2_SCAN_NUM    0x45        // Num lock scan code
#define     PS2_LAST_CODE   59          // Last (largest scan code)

// Mac Plus / keyboard definitions
#define     MAC_DATA_LINE   0b00000010  // Mac serial interface register bit masks
#define     MAC_CLOCK_LINE  0b00000001

#define     MAC_KBD_TOV     20          // In multiples of 10uSec (20=200uSec)

#define     MAC_KBD_INQ     0x10        // Key Transition code or Null ($7B)
#define     MAC_KBD_INST    0x14        // Key Transition code or Null ($7B)
#define     MAC_KBD_MODEL   0x16        // Bit 0:    1
                                        // Bits 1-3: keyboard model number, 1-8
                                        //           M0110:  0x09  00001001 : model number 4 (100)
                                        //           M0110A: 0x0B  00001011 : model number 5 (101)
                                        // Bits 4-6: next device number, 1-8
                                        // Bit 7:    1 if another device connected
#define     MAC_KBD_TEST    0x36        // ACK ($7D) or NAK ($77)

#define     KBD_MAC_NULL    0x7B        // No key press indicator/response
#define     KBD_MAC_ACK     0x7D        // Acknowledge test
#define     KBD_MAC_NAK     0x77        // Test error response
#define     KBD_MAC_MODEL   0b00001001  // See MAC_KBD_MODEL command

/****************************************************************************
  Types
****************************************************************************/
typedef enum
{
    PS2_IDLE,
    PS2_DATA_BITS,
    PS2_PARITY,
    PS2_STOP,
    PS2_RX_ERR_START,
    PS2_RX_ERR_OVERRUN,
    PS2_RX_ERR_PARITY,
    PS2_RX_ERR_STOP,
} ps2_state_t;

/****************************************************************************
  Function prototypes
****************************************************************************/
void reset(void) __attribute__((naked)) __attribute__((section(".init3")));
void ioinit(void);

int  ps2_send(uint8_t);
int  ps2_recv_x(void);      // Blocking
int  ps2_recv(void);        // Non-blocking

void kbd_test_led(void);
int  kdb_led_ctrl(uint8_t);
int  kbd_code_set(int);
int  kbd_typematic_set(uint8_t);

int  read_key(void);
int  write_key(uint8_t key_code);

void set_test_point(void);
void clr_test_point(void);
void pulse_test_point(void);

int  read_mac_interface(void);
int  write_mac_interface(uint8_t byte);

uint8_t get_global_time(void);

/* Scan code translation table
 */
uint8_t scan_code_xlate[] =
{
        KBD_MAC_NULL,   // 0
        0x25,
        0x27,
        0x29,
        0x2B,
        0x2F,
        0x2D,
        0x35,
        0x39,
        0x33,
        0x3b,           // 10
        0x37,
        0x31,
        0x67,
        0x61,
        0x19,
        0x1B,
        0x1D,
        0x1F,
        0x23,
        0x21,           // 20
        0x41,
        0x45,
        0x3F,
        0x47,
        0x43,
        0x3D,
        0x49,
        0x75,
        0x01,
        0x03,           // 30
        0x05,
        0x07,
        0x0B,
        0x09,
        0x4D,
        0x51,
        0x4B,
        0x53,
        0x4F,
        KBD_MAC_NULL,   // 40
        0x71,
        0x55,
        0x0D,
        0x0F,
        0x11,
        0x13,
        0x17,
        0x5B,
        0x5D,
        0x57,           // 50
        0x5F,
        0x59,
        0x71,
        KBD_MAC_NULL,
        KBD_MAC_NULL,
        0x63,
        0x73,
        0x6F,           // 58
};

/****************************************************************************
  Globals
****************************************************************************/
// Circular buffer holding PS2 scan codes
uint8_t      ps2_scan_codes[PS2_BUFF_SIZE];
int          ps2_buffer_out = 0;
volatile int ps2_buffer_in = 0;
volatile int ps2_scan_code_count = 0;

// Variable maintaining state of bit stream from PS2
volatile ps2_state_t ps2_rx_state = PS2_IDLE;
volatile uint8_t  ps2_rx_data_byte = 0;
volatile int      ps2_rx_bit_count = 0;
volatile int      ps2_rx_parity = 0;

// Key code output buffer
uint8_t      key_codes[KEY_BUFF_SIZE];
volatile int key_code_count = 0; //KEY_BUFF_SIZE;
volatile int key_buffer_out = 0;
volatile int key_buffer_in = 0;

// Keyboard status
volatile uint8_t    kbd_lock_keys = 0;;

// Global time base
volatile uint8_t    global_counter = 0;

/* ----------------------------------------------------------------------------
 * main() control functions
 *
 */
int main(void)
{
    int     scan_code;
    uint8_t wait_start_time;
    int     code_wait_state = 0;
    uint8_t kdb_lock_state = 0;

    // Initialize IO devices
    ioinit();

    // Wait enough time for keyboard to complete self test
    _delay_ms(1000);

    // light LEDs in succession
    kbd_test_led();

    // set typematic delay and rate
    kbd_typematic_set(PS2_HK_TYPEMAT);

    // change code set to "1" so code set translation does not needs to take place on the AVR
    kbd_code_set(1);

    sei();

    /* Loop forever. receive key strokes from the keyboard,
     * filter out the unwanted keys, translate to Mac scan codes and store in output buffer.
     * Send scan codes from output buffer upon request from Mac's serial interface.
     */
    while ( 1 )
    {
        scan_code = ps2_recv();

        /* Only pass make and break codes for keys in range 1 to 59, scan code 91 is mapped to 59 (F1 key)
         * Handle 'E0' modifier for keypad by removing the 'E0' which
         * will effectively reduce any keyboard to one that is equivalent to an 83 key keyboard.
         * Discard PrtScrn E0,2A,E0,37 and E0,B7,E0,AA.
         * Discard E1 sequence of Pause/Break
         */
        if  ( scan_code != -1 )
        {
            /* Handle 'E1' scan code case for Pause/Break key
             */
            if ( scan_code == 0xe1 )
            {
                scan_code = ps2_recv_x();

                /* Get the next (3rd) byte and discard sequence
                 */
                if ( scan_code == 0x1d )
                {
                    ps2_recv_x();
                    continue;
                }
                else if ( scan_code == 0x9d )
                {
                    ps2_recv_x();
                    continue;
                }
            }

            /* Handle 'E0' scan code cases
             */
            if ( scan_code == 0xe0 )
            {
                scan_code = ps2_recv_x();

                /* Only keep the "Windows" key as a Mac Command key
                 * and the Right Ctrl key (both make and break codes)
                 */
                if ( scan_code == 0x1d ||
                     scan_code == 0x9d    )
                {
                    // Do nothing.
                }
                else if ( scan_code == 0x5b )
                {
                    scan_code = 0x3b;
                }
                else if ( scan_code == 0xdb    )
                {
                    scan_code = 0xbb;
                }
                else
                {
                    continue;
                }
            }

            /* Special Caps Lock handling as a toggle of keyboard lock status,
             * and only need to support caps-lock.
             * Only toggle on CapLock PS2 'make' code and discard the 'break' code.
             */
            if ( scan_code == (PS2_SCAN_CAPS | 0x80) )
            {
                continue;
            }
            else if ( scan_code == PS2_SCAN_CAPS )
            {
                kbd_lock_keys ^= PS2_HK_CAPSLOCK;
                if ( kbd_lock_keys & PS2_HK_CAPSLOCK )
                    scan_code = PS2_SCAN_CAPS;
                else
                    scan_code = PS2_SCAN_CAPS | 0x80;
            }

            /* Remove unwanted scan codes
             */
            switch ( ((uint8_t)scan_code & 0x7f) & 0x7f )
            {
                case 1:         // Esc
                case 41:        // '
                case 55:        // Keypad *
                case 56:        // Alt keys
                    continue;
            }

            /* Discard all codes that are out of range
             */
            if  ( ((uint8_t)scan_code & 0x7f) > PS2_LAST_CODE || scan_code == 0 )
            {
                continue;
            }

            /* Store processed scan code in the key code output buffer 'key_codes[]'
             */
            write_key((uint8_t)scan_code);
        }

        /* Update indicator LEDs if required.
         * Do this only if there is no pending scan code in the buffer
         * so that host-to-keyboard communication does not interfere with scan code exchange.
         */
        else if ( kdb_lock_state != kbd_lock_keys )
        {
            kdb_led_ctrl(kbd_lock_keys);
            kdb_lock_state = kbd_lock_keys;
        }

        /* Poll the Mac serial interface for incoming requests
         * and process them here.
         *
         * There are two states governed by 'code_wait_state':
         * is '1': an Inquiry command was received from the host and the keyboard controller
         *         has~250mSec to wait for a key press before returning a code or a NULL.
         * is '0': normal loop, waiting for a command from the host.
         */
        if ( code_wait_state )
        {
            scan_code = read_key();

            if ( scan_code != KBD_MAC_NULL ||
                 (uint8_t)(get_global_time() - wait_start_time) >= KBDRD_TOV )
            {
                write_mac_interface((uint8_t) scan_code);
                code_wait_state = 0;
            }
        }
        else
        {
            //pulse_test_point();
            switch ( read_mac_interface() )
            {
                case -1:
                    // Do nothing. No request received.
                    break;

                case MAC_KBD_INQ:
                    code_wait_state = 1;
                    wait_start_time = get_global_time();
                    break;

                case MAC_KBD_INST:
                    scan_code = read_key();
                    write_mac_interface((uint8_t) scan_code);
                    code_wait_state = 0;
                    break;

                case MAC_KBD_MODEL:
                    write_mac_interface(KBD_MAC_MODEL);
                    // "Specification" suggest that keyboard system needs to reset itself.
                    ps2_buffer_out = 0;
                    ps2_buffer_in = 0;
                    ps2_scan_code_count = 0;

                    key_code_count = 0;
                    key_buffer_out = 0;
                    key_buffer_in = 0;

                    kbd_lock_keys &= ~PS2_HK_CAPSLOCK;
                    code_wait_state = 0;
                    break;

                case MAC_KBD_TEST:
                    write_mac_interface(KBD_MAC_ACK);
                    code_wait_state = 0;
                    break;

                default:
                    /* TODO: Something went wrong and an unidentified command byte was received.
                     */
                    code_wait_state = 0;
                    break;
            }
        }
    }

    return 0;
}

/* ----------------------------------------------------------------------------
 * reset()
 *
 *  Clear SREG_I on hardware reset.
 *  source: http://electronics.stackexchange.com/questions/117288/watchdog-timer-issue-avr-atmega324pa
 */
void reset(void)
{
     cli();
    // Note that for newer devices (any AVR that has the option to also
    // generate WDT interrupts), the watchdog timer remains active even
    // after a system reset (except a power-on condition), using the fastest
    // prescaler value (approximately 15 ms). It is therefore required
    // to turn off the watchdog early during program startup.
    MCUSR = 0; // clear reset flags
    wdt_disable();
}

/* ----------------------------------------------------------------------------
 * ioinit()
 *
 *  Initialize IO interfaces.
 *  Timer and data rates calculated based on the default 8MHz internal clock.
 *
 */
void ioinit(void)
{
    // Reconfigure system clock scaler to 8MHz
    CLKPR = 0x80;   // change clock scaler (ATtiny85 sec 6.5.2 p.32)
    CLKPR = 0x00;

    // Initialize general IO PB pins
    DDRB  = PB_DDR_INIT;
    PORTB = PB_INIT | PB_PUP_INIT;

    // Pin change interrupt setting
    GIMSK = GIMSK_INIT;
    PCMSK = PCMSK_INIT;

    // Timer0 setup
    TCNT0 = 0;
    TCCR0A = TCCR0A_INIT;
    TCCR0B = TCCR0B_INIT;
    TIMSK = TIMSK_INIT;
}

/* ----------------------------------------------------------------------------
 * ps2_send()
 *
 *  Send a command to the PS2 keyboard
 *  1)   Bring the Clock line low for at least 100 microseconds.
 *  2)   Bring the Data line low.
 *  3)   Release the Clock line.
 *
 *  4)   Set/reset the Data line to send the first data bit
 *  5)   Wait for the device to bring Clock low.
 *  6)   Repeat steps 5-7 for the other seven data bits and the parity bit
 *  7)   Wait for the device to bring Clock high.
 *
 *  8)   Release the Data line.
 *  9)   Wait for the device to bring Data low.
 *  10)  Wait for the device to bring Clock  low.
 *  11)  Wait for the device to release Data and Clock
 *
 *  param: command byte
 *  return: -1 transmit error, 0 ok
 */
int ps2_send(uint8_t byte)
{
    int     ps2_tx_parity = 1;
    int     ps2_tx_bit_count = 0;
    uint8_t ps2_data_bit;
    int     result;

    // Disable pin-change interrupts and reset receiver state so receive ISR does not run
    cli();
    //PCMSK &= ~PCINT_PB3;

    ps2_rx_state = PS2_IDLE;
    ps2_rx_data_byte = 0;
    ps2_rx_bit_count = 0;
    ps2_rx_parity = 0;

    // Follow byte send steps
    DDRB |= PS2_CLOCK;
    PORTB &= ~PS2_CLOCK;
    _delay_us(100);

    DDRB |= PS2_DATA;
    PORTB &= ~PS2_DATA;

    DDRB &= ~PS2_CLOCK;
    PORTB |= PS2_CLOCK;

    while ( ps2_tx_bit_count < 10 )
    {
        // This will repeat 8 bits of data, one parity bit, and one stop bit for transmission
        if ( ps2_tx_bit_count < 8 )
        {
            ps2_data_bit = byte & 0x01;
            ps2_tx_parity += ps2_data_bit;
        }
        else if ( ps2_tx_bit_count == 8 )
        {
            ps2_data_bit = (uint8_t)ps2_tx_parity & 0x01;
        }
        else
        {
            ps2_data_bit = 1;
        }

        do {} while ( (PINB & PS2_CLOCK) );

        if ( ps2_data_bit )
            PORTB |= PS2_DATA;
        else
            PORTB &= ~PS2_DATA;

        do {} while ( !(PINB & PS2_CLOCK) );

        ps2_tx_bit_count++;
        byte = byte >> 1;
    }

    // Restore data line to receive mode
    DDRB &= ~PS2_DATA;
    PORTB |= PS2_DATA;

    // Check here for ACK pulse and line to idle
    do {} while ( (PINB & PS2_CLOCK) );
    result = -1 * (int)(PINB & PS2_DATA);

    // Wait for clock to go high before enabling interrupts
    do {} while ( !(PINB & PS2_CLOCK) );

    sei();
    //PCMSK |= PCINT_PB3;

    // Allow keyboard to recover before exiting,
    // so that another ps2_send() is spaced in time from this call.
    _delay_ms(20);

    return result;
}

/* ----------------------------------------------------------------------------
 * ps2_recv_x()
 *
 *  Get a byte from the PS2 input buffer and block until byte is available
 *
 *  param:  none
 *  return: data byte value
 *
 */
int ps2_recv_x(void)
{
    int     result;

    do
    {
        result = ps2_recv();
    } while ( result == -1 );

    return result;
}

/* ----------------------------------------------------------------------------
 * ps2_recv()
 *
 *  Get a byte from the PS2 input buffer
 *
 *  param:  none
 *  return: -1 if buffer empty, otherwise data byte value
 *
 */
int ps2_recv(void)
{
    int     result = -1;

    if ( ps2_scan_code_count > 0 )
    {
        result = (int)ps2_scan_codes[ps2_buffer_out];
        ps2_scan_code_count--;
        ps2_buffer_out++;
        if ( ps2_buffer_out == PS2_BUFF_SIZE )
            ps2_buffer_out = 0;
    }

    return result;
}

/* ----------------------------------------------------------------------------
 * ps2_test_led()
 *
 *  Simple light test for keyboard LEDs.
 *  The function discards the keyboard's response;
 *  if something is wrong LEDs will not light up in succession.
 *
 *  param:  none
 *  return: none
 */
void kbd_test_led(void)
{
    kdb_led_ctrl(PS2_HK_SCRLOCK);

    _delay_ms(200);

    kdb_led_ctrl(0);
    kdb_led_ctrl(PS2_HK_CAPSLOCK);

    _delay_ms(200);

    kdb_led_ctrl(0);
    kdb_led_ctrl(PS2_HK_NUMLOCK);

    _delay_ms(200);

    kdb_led_ctrl(0);
    kdb_led_ctrl(PS2_HK_CAPSLOCK);

    _delay_ms(200);

    kdb_led_ctrl(0);
    kdb_led_ctrl(PS2_HK_SCRLOCK);

    _delay_ms(200);

    kdb_led_ctrl(0);
}

/* ----------------------------------------------------------------------------
 * kdb_led_ctrl()
 *
 *  Function for setting LED state to 'on' or 'off'
 *
 *  param:  LED bits, b0=Scroll lock b1=Num lock b2=Caps Lock
 *  return: none
 */
int kdb_led_ctrl(uint8_t state)
{
    int temp_scan_code;

    state &= 0x07;

    ps2_send(PS2_HK_LEDS);
    temp_scan_code = ps2_recv_x();

    if ( temp_scan_code == PS2_KH_ACK )
    {
        ps2_send(state);
        temp_scan_code = ps2_recv_x();
    }

    return temp_scan_code;
}

/* ----------------------------------------------------------------------------
 * kbd_code_set()
 *
 *  The function requests the keyboard to change its scan code set,
 *  Legal values are 1, 2 or 3.
 *
 *  param:  scan code set identifier
 *  return: PS2_KH_ACK no errors, other keyboard response if error
 */
int kbd_code_set(int set)
{
    int temp_scan_code;

    if ( set < 1 || set > 3 )
        return PS2_KH_RESEND;

    ps2_send(PS2_HK_ALTCODE);
    temp_scan_code = ps2_recv_x();

    if ( temp_scan_code == PS2_KH_ACK )
    {
        ps2_send((uint8_t)set);
        temp_scan_code = ps2_recv_x();
    }

    return temp_scan_code;
}

/* ----------------------------------------------------------------------------
 * kbd_typematic_set()
 *
 *  The function sets the keyboard typematic rate and delay.
 *
 *  Bit/s   Meaning
 *  ....... ...................................................
 *  0 to 4  Repeat rate (00000b = 30 Hz, ..., 11111b = 2 Hz)
 *  5 to 6  Delay before keys repeat (00b = 250 ms, 01b = 500 ms, 10b = 750 ms, 11b = 1000 ms)
 *     7    Must be zero
 *
 *  param:  typematic rate and delay
 *  return: PS2_KH_ACK no errors, other keyboard response if error
 */
int kbd_typematic_set(uint8_t configuration)
{
    int temp_scan_code;

    configuration &= 0x7f;

    ps2_send(PS2_HK_TMDELAY);
    temp_scan_code = ps2_recv_x();

    if ( temp_scan_code == PS2_KH_ACK )
    {
        ps2_send(configuration);
        temp_scan_code = ps2_recv_x();
    }

    return temp_scan_code;
}

/* ----------------------------------------------------------------------------
 * read_key()
 *
 *  Get a byte from the output buffer
 *
 *  param:  none
 *  return: KBD_MAC_NULL if buffer empty, otherwise data byte value of key code
 *
 */
int read_key(void)
{
    int     result = KBD_MAC_NULL;

    if ( key_code_count > 0 )
    {
        result = (int)key_codes[key_buffer_out];
        key_code_count--;
        key_buffer_out++;
        if ( key_buffer_out == KEY_BUFF_SIZE )
            key_buffer_out = 0;
    }

    return result;
}

/* ----------------------------------------------------------------------------
 * write_key()
 *
 *  Translate and write a Mac scan code byte to the output buffer
 *
 *  param:  key code to translate
 *  return: -1 if buffer is full, otherwise value of key code
 *
 */
int write_key(uint8_t key_code)
{
    int     result = -1;
    int     code = 0;
    uint8_t make_break = 0;

    code = ((int)(key_code & 0x7f)) - 1;

    if ( code < 0 || code > (PS2_LAST_CODE - 1) )
        return -1;

    make_break = key_code & 0x80;

    if ( key_code_count < KEY_BUFF_SIZE )
    {
        key_codes[key_buffer_in] = scan_code_xlate[code] | make_break;
        result = (int) key_codes[key_buffer_in];
        key_code_count++;
        key_buffer_in++;
        if ( key_buffer_in == KEY_BUFF_SIZE )
            key_buffer_in = 0;
    }

    return result;
}

/* ----------------------------------------------------------------------------
 * set_test_point()
 *
 *  Set test point PB2 to '1'
 *
 *  param:  none
 *  return: none
 *
 */
void set_test_point(void)
{
    PORTB |= TEST_POINT;
}

/* ----------------------------------------------------------------------------
 * clr_test_point()
 *
 *  Clear test point PB2 to '0'
 *
 *  param:  none
 *  return: none
 *
 */
void clr_test_point(void)
{
    PORTB &= ~TEST_POINT;
}

/* ----------------------------------------------------------------------------
 * pulse_test_point()
 *
 *  10uSec high pulse on the test point output
 *
 *  param:  none
 *  return: none
 *
 */
void pulse_test_point(void)
{
    set_test_point();
    _delay_us(10);
    clr_test_point();
}

/* ----------------------------------------------------------------------------
 * read_mac_interface()
 *
 *  Read the Mac serial keyboard interface
 *
 *  param:  none
 *  return: -1 interface times out, Mac's command byte
 *
 */
int read_mac_interface(void)
{
    int     i;
    int     tov = 0;
    uint8_t byte = 0;

    /* Receive procedure (http://www.mac.linux-m68k.org/devel/plushw.php):
     * 1. Wait for Mac to be 'ready' - Data line='0'
     *    - initiate time out counter here
     * 2. Loop over 8-bits, MSB first
     *    a. Wait 140uSec
     *    b. Set clock to '0'
     *    c. Wait 180uSec
     *    d. Set clock to '1'
     *    e. Wait 80uSec
     *    f. Sample data bit
     */

    while ( (PINB & MAC_DATA_LINE) == 1 )
    {
        _delay_us(10);
        tov++;
        if ( tov == MAC_KBD_TOV )
            return -1;
    }

    for ( i = 0; i < 8; i++ )
    {
        _delay_us(140);

        PORTB &= ~MAC_CLOCK_LINE;

        _delay_us(180);

        PORTB |= MAC_CLOCK_LINE;

        _delay_us(80);

        byte = byte << 1;
        if ( PINB & MAC_DATA_LINE )
            byte |= 1;
    }

    return (int) byte;
}

/* ----------------------------------------------------------------------------
 * write_mac_interface()
 *
 *  Write a bytes to the Mac serial keyboard interface
 *
 *  param:  Byte to send
 *  return: -1 interface times out, echo data byte
 *
 */
int write_mac_interface(uint8_t byte)
{
    int     i;
    int     tov = 0;

    /* Transmit procedure (http://www.mac.linux-m68k.org/devel/plushw.php):
     * 1. Wait for Mac to be 'ready' - Data line='1'
     *    - initiate time out counter here
     * 2. Set data line to output
     * 3. Loop over 8-bits, MSB first
     *    a. Wait 130uSec
     *    b. Set data bit
     *    c. Wait 40uSec
     *    d. Set clock to '0'
     *    e. Wait 160uSec
     *    f. Set clock to '1'
     * 4. Set data line to input
     */

    while ( (PINB & MAC_DATA_LINE) == 0 )
    {
        _delay_us(10);
        tov++;
        if ( tov == MAC_KBD_TOV )
            return -1;
    }

    DDRB |= MAC_DATA_LINE;

    for ( i = 0; i < 8; i++ )
    {
        _delay_us(130);

        if ( byte & 0b10000000 )
            PORTB |= MAC_DATA_LINE;
        else
            PORTB &= ~MAC_DATA_LINE;

        _delay_us(40);

        PORTB &= ~MAC_CLOCK_LINE;

        _delay_us(160);

        PORTB |= MAC_CLOCK_LINE;

        byte = byte << 1;
    }

    DDRB &= ~MAC_DATA_LINE;

    return byte;
}

/* ----------------------------------------------------------------------------
 * get_global_time()
 *
 *  Return the value of the global timer tick counter
 *
 *  param:  none
 *  return: counter value
 *
 */
uint8_t get_global_time(void)
{
    return global_counter;
}

/* ----------------------------------------------------------------------------
 * This ISR will trigger when PB3 changes state.
 * PB3 is connected to the PS2 keyboard clock line, and PB4 to the data line.
 * ISR will check PB3 state and determine if it is '0' or '1',
 * as well as track clock counts and input bits from PB4.
 * Once input byte is assembled is will be added to a circular buffer.
 *
 */
ISR(PCINT0_vect)
{
    uint8_t         ps2_data_bit;

    if ( (PINB & PS2_CLOCK) == 0 )
    {
        ps2_data_bit = (PINB & PS2_DATA) >> 4;

        switch ( ps2_rx_state )
        {
            /* Do nothing if an error was already signaled
             * let the main loop handle the error
             */
            case PS2_RX_ERR_START:
            case PS2_RX_ERR_OVERRUN:
            case PS2_RX_ERR_PARITY:
            case PS2_RX_ERR_STOP:
                break;

            /* If in idle, then check for valid start bit
             */
            case PS2_IDLE:
                if ( ps2_data_bit == 0 )
                {
                    ps2_rx_data_byte = 0;
                    ps2_rx_bit_count = 0;
                    ps2_rx_parity = 0;
                    ps2_rx_state = PS2_DATA_BITS;
                }
                else
                    ps2_rx_state = PS2_RX_ERR_START;
                break;

            /* Accumulate eight bits of data LSB first
             */
            case PS2_DATA_BITS:
                ps2_rx_parity += ps2_data_bit;
                ps2_data_bit = ps2_data_bit << ps2_rx_bit_count;
                ps2_rx_data_byte += ps2_data_bit;
                ps2_rx_bit_count++;
                if ( ps2_rx_bit_count == 8 )
                    ps2_rx_state = PS2_PARITY;
                break;

            /* Evaluate the parity and signal error if it is wrong
             */
            case PS2_PARITY:
                if ( ((ps2_rx_parity + ps2_data_bit) & 1) )
                    ps2_rx_state = PS2_STOP;
                else
                    ps2_rx_state = PS2_RX_ERR_PARITY;
                break;

            /* Check for valid stop bit
             */
            case PS2_STOP:
                if ( ps2_data_bit == 1 )
                {
                    if ( ps2_scan_code_count < PS2_BUFF_SIZE )
                    {
                        ps2_scan_codes[ps2_buffer_in] = ps2_rx_data_byte;
                        ps2_scan_code_count++;
                        ps2_buffer_in++;
                        if ( ps2_buffer_in == PS2_BUFF_SIZE )
                            ps2_buffer_in = 0;
                        ps2_rx_state = PS2_IDLE;
                    }
                    else
                        ps2_rx_state = PS2_RX_ERR_OVERRUN;
                }
                else
                    ps2_rx_state = PS2_RX_ERR_STOP;
                break;
        }
    }
}

/* ----------------------------------------------------------------------------
 * This ISR will trigger approximately every 33mSec when Timer0 overflows.
 * The ISR increments a global 8-bit time variable that will overflow (cycle back through 00)
 * approximately every 8.4 seconds.
 *
 */
ISR(TIMER0_OVF_vect)
{
    global_counter++;
}
