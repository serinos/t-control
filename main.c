/* Temperature control system - O.S.
 * Works with MSP430G2x53 Launchpad and UsluKukla (44 Booster)
 * Basic flow explained in appended flowchart
 *
 * Usage: IMPORTANT: Short the second and third heads of JP101 to use the thermistor.
 *        IMPORTANT: Short the heads of JP201 for the buzzer to work for threshold warnings
 *        There are three "modes": Thermistor reading, potentiometer reading, and off.
 *        The modes are switched revolvingly using S101 on UK by pressing the button briefly
 *        Readings can be either in mV or 100xCelcius on the 7Seg, use READ_VOLTAGE_OR_DEG to compile accordingly
 *        The potentiometer can be used to set threshold, give TEMP_THRESHOLD_TOGGLE 1 to include this feature
 *        When the voltage set by the potentiometer is exceeded, the buzzer vibrates to give an alarm
 *        The buzzer routine can be used as a driver for a motor, if one is mounted on H202; it is connected to P1.6
 *
 * Notes: 7Seg will not show a minus sign, no info given on the screen for the sign the temperature goes below 0 Celcius.
 *        Also, if the temperature exceeds 99.99 Celcius, the hundreds digit will not show on the screen
 *        This can be solved by printing values in Celcius proper in XXX.X format using the dots of the 7Seg
 *        (The decimal dots are activated by flipping the least sig.fig. of a char to 1)
 *        If voltage of P1.3 is zeroed out, the 7Seg gets scrambled if the code is compiled with READ_VOLTAGE_OR_DEG 0
 *        The code is not optimized for low power usage
 */

#include <msp430.h>
#include <math.h>                              // Need log()

#define READ_VOLTAGE_OR_DEG 1        // Toggle for troubleshoot: 1 for milivoltage, 0 for temp in 100x celcius for 7Seg display
#define TEMP_THRESHOLD_TOGGLE 1      // 1: Use potentiometer as a temperature threshold toggle; 0: Do not
#define BUZZER_LIMIT 5     // No. of consec times the buzzer is run after a reading exceeds threshold, before getting neglected

#define BEEP_TIME 1500                         // Beep time in cycles
#define SOUND_DELAY 20                         // Tune here for different freqs of sound
#define WAIT_TIME 200                          // Wait time cycles
#define TIMER_MOD_COEFF 1/(2*SOUND_DELAY)     // Readjustment for matching desired time in ms for beep duration
#define BEEP_TIME_MOD BEEP_TIME*TIMER_MOD_COEFF  // Beep time in cycles, corrected
#define VOLTAGE_COEFF 2.96                   // Coeff for converting reading to voltage times 1000, theo:3.22, exper: 2.95
#define EMPTY_X 10                       // Refers to the all-clear character

// Thermistor parameters for Steinhart-Hart Eqn
#define L_1 13.69
#define L_2 9.21
#define L_3 5.125
#define Y_1 -0.01818
#define Y_2 0.04
#define Y_3 0.006451
#define G_2 ((Y_2-Y_1)/(L_2-L_1))
#define G_3 ((Y_3-Y_1)/(L_3-L_1))
#define C_COEFF (((G_3-G_2)/(L_3-L_2))/(L_1+L_2+L_3))
#define B_COEFF (G_2 - C_COEFF*(L_1*L_1 + L_1*L_2 + L_2*L_2))
#define A_COEFF (Y_1 - L_1*(B_COEFF + L_1*L_1*C_COEFF))
// These coeffs seem not to behave as expected for the given thermistor, would advise recalibrating

int degree_conv(double voltage);
void buzz();
int threshold_check(unsigned int input);
void write_4digit(int number, int delay);
void inject_7seg(unsigned int d_1, unsigned int d_2,
                 unsigned int d_3, unsigned int d_4,
                 int delay);
void write_7seg(int data, int index);
unsigned int p1_samples[3] = {0,0,0};  //From the ADC readings, p1_samples[0] holds P1.5, p1_samples[2] holds P1.3
const unsigned int index[11] = { // 7-Seg Mapping; Order of bits for 7-Seg is 0bABCDEFGP
                                 // Using volatile results in glitches, so use const
    0b11111100, // 0
    0b01100000, // 1
    0b11011010, // 2
    0b11110010, // 3
    0b01100110, // 4
    0b10110110, // 5
    0b10111110, // 6
    0b11100000, // 7
    0b11111110, // 8
    0b11110110, // 9
    0b00000000, // all-clear
};


int main(void) {
     int reading = 0, reading_pot = 0;
     unsigned int i = 0, buzz_ctr = 0, button = 0, button_ctr = 0;
     double voltage = 0, voltage_pot = 0, voltage_act = 0, voltage_pot_act = 0;
     int degree = 0;
     WDTCTL = WDTPW + WDTHOLD;                 // Stop the watch-dog timer
     ADC10CTL1 = INCH_5 + CONSEQ_1;            // Will read starting from P1.5 downward
     ADC10CTL0 = ADC10SHT_2 + MSC + ADC10ON + ADC10IE;
     ADC10DTC1 = 0x03;                         // 3 conversions
     ADC10AE0 |= 0x38;                         // P1.5,4,3 ADC10 option select
     ADC10SA = (unsigned int)p1_samples;       // Data from ADC is to be stored at p1_samples
     P1DIR |= 0x41;                            // Set P1.0 and P1.6 to output
     P2DIR &= 0xFD;                            // Set P2.1 to input (S101 on UK feeds P2.1)
     P2DIR |= 0x19;                            // Set P2.0,3,4 to output (for pin 14/11/12 of IC102)
     P1OUT &= 0xBE;                            // Clear P1.0 and P1.6 to zero

     DCOCTL = 0;                 // Select lowest DCOx and MODx settings
     BCSCTL1 = CALBC1_16MHZ;     // Set range
     DCOCTL = CALDCO_16MHZ;      // 1 cycle = 1s/16MHz = 62.5ns

    for (;;) {
        // Button sense
        button = P2IN & 0x02;
        if (button == 0x00) {                 // Check if P2.1 is zero (S101 pressed)
            button_ctr += 1;
        }
        while (button == 0x00) {
            for(i = WAIT_TIME; i > 0; i--);
            button = P2IN & 0x02;
        }
        if (button_ctr % 3 == 2) {
            inject_7seg(EMPTY_X, EMPTY_X, EMPTY_X, EMPTY_X, WAIT_TIME);
            continue;
        }

        // Temperature checking subroutine
        ADC10CTL0 &= ~ENC;
        while (ADC10CTL1 & BUSY);               // Wait if ADC10 core is active
        ADC10SA = 0x200;                        // Data buffer start
        P1OUT |= 0x01;                          // P1.0 set ON, signaling data acquisition
        ADC10CTL0 |= ENC + ADC10SC;             // Sampling and conversion start
               // Not using interrupts here makes data acquisition significantly faster but makes readings slightly shaky (+- 10mV experimentally)
               // Talking about __bis_SR_register(CPUOFF + GIE)
        reading = p1_samples[2];                // Reading voltage step of P1.3
        reading_pot = p1_samples[0];            // Reading voltage step of P1.5 (potentiometer subcircuit)
        P1OUT &= ~0x01;                         // P1.0 set OFF, signaling end of data acquisition
        voltage = reading * VOLTAGE_COEFF;      // Unit: mV
        voltage_pot = reading_pot * VOLTAGE_COEFF;

        // Button sense, potentiometer mode - show values corresponding to the potentiometer on 7Seg
        if (button_ctr % 3 == 1) {
            if(READ_VOLTAGE_OR_DEG)
                write_4digit(voltage_pot, WAIT_TIME);
            else {
                voltage_pot_act = voltage_pot/1000;
                degree = degree_conv(voltage_pot_act);
                write_4digit(degree, WAIT_TIME);
            }
            continue;
        }

        // Temperature threshold control
        if ((TEMP_THRESHOLD_TOGGLE) && (reading > reading_pot) && (buzz_ctr < BUZZER_LIMIT)) {
            buzz();
            buzz_ctr += 1;
        }
        else if (reading > reading_pot) buzz_ctr += 1;
        else buzz_ctr = 0;

        // 7SEG printing - Temperature reading off anyway, will just use reading to display
        if(READ_VOLTAGE_OR_DEG)
            write_4digit(voltage, WAIT_TIME);
        else {
            voltage_act = voltage/1000;
            degree = degree_conv(voltage_act);
            write_4digit(degree, WAIT_TIME);
        }
    }
}


int degree_conv(double voltage) { // Tuned for B57891M103J thermistor, subcircuit A2 at page 2, unit V for voltage
    double degree = 0;
    double resistance_therm = 10000*voltage/(3.3-voltage);  // Bias resistance 10Kohm, feed 3.3V

    // Implementing the Steinhart-Hart Eqn:
    double res_log = log(resistance_therm);
    degree = 1/(A_COEFF + B_COEFF*res_log + C_COEFF*res_log*res_log*res_log);
    return degree*100; // Return 100 times the degree as int
}


void buzz() {              // Works the buzzer and P1.0 for BEEP_TIME duration
    int i = 0, j = 0;
    for (i=BEEP_TIME_MOD; i > 0; i--) {
        P1OUT |= 0x41;                      // Light up P1.0 and P1.6
        for (j=SOUND_DELAY; j > 0; j--);
        P1OUT &= 0xBE;                      // Clear up P1.0 and P1.6
        for (j=SOUND_DELAY; j > 0; j--);
    }
}


void write_4digit(int number, int delay) { // Decimal to 7Seg, truncates after the 4 least significant digits
    unsigned int d_4 = number % 10;
    unsigned int d_3 = (number / (10)) % 10;
    unsigned int d_2 = (number / (100)) % 10;
    unsigned int d_1 = (number / (1000)) % 10;

    inject_7seg(d_1, d_2, d_3, d_4, WAIT_TIME);
}


void inject_7seg(unsigned int d_1, unsigned int d_2,
                 unsigned int d_3, unsigned int d_4,
                 int delay) { // Directly write characters from index[] onto 7Seg screen
                                                             // Write from left to right
    int i = 0;
    for(i = delay*0.4; i > 0; i--) {
        write_7seg(index[d_1],4);
        __delay_cycles(10000);
        write_7seg(index[d_2],3);
        __delay_cycles(10000);
        write_7seg(index[d_3],2);
        __delay_cycles(10000);
        write_7seg(index[d_4],1);
        __delay_cycles(10000);
    }
}


void write_7seg(int data, int index) {
    unsigned int sck=0x08;   // P2.3
    unsigned int rck=0x10;   // P2.4
    unsigned int bit=0, i=0; // bit: Bit Holder for shifting

    for(i = 4; i > 0; i--) { // Fill lower 4 bits of IC102 with 0
        P2OUT=0;
        __delay_cycles(1);
        P2OUT=sck;
        __delay_cycles(1);
        P2OUT=0;
        __delay_cycles(1);
    }

    for(i = 4; i > 0; i--) { // Fill upper 4 bits of IC102

        if(index == 1) { //Write 1 to Operated Digit C
            P2OUT = 1;
            __delay_cycles(1);
            P2OUT = sck|1;
            __delay_cycles(1);
            P2OUT = 1;
            __delay_cycles(1);
        } else {          // Write 0 to unoperated digits C
            P2OUT = 0;
            __delay_cycles(1);
             P2OUT = sck|0;
            __delay_cycles(1);
            P2OUT = 0;
            __delay_cycles(1);
        }
        index = index-1;
    }

    for(i = 8; i > 0; i--) { // Write 7Seg message into IC102, push the existing message to IC104
        bit = data & 1; // Grab 1st Bit of Data
        __delay_cycles(1);
        P2OUT = bit; // Write bit Value to Output
        P2OUT = sck|bit; // SCK is on, Write to Register
        __delay_cycles(1);
        P2OUT = bit; // SCK is off, Save Register Value
        __delay_cycles(1);
        data = data >> 1; // Bit-shift right by 1 bit for the next cycle
    }

    P2OUT = rck; // Register done send data to 7Seg
    __delay_cycles(1);
}