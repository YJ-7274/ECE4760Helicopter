/**
 * V. Hunter Adams (vha3@cornell.edu)

    This is an experiment with the multicore capabilities on the
    RP2040. The program instantiates a timer interrupt on each core.
    Each of these timer interrupts writes to a separate channel
    of the SPI DAC and does DDS of two sine waves of two different
    frequencies. These sine waves are amplitude-modulated to "beeps."

    No spinlock is required to mediate the SPI writes because of the
    SPI buffer on the RP2040. Spinlocks are used in the main program
    running on each core to lock the other out from an incrementing
    global variable. Experimentation shows that a short delay is
    required between unlocking and locking the spinlock for the second
    core to reliably lock the spinlock before the first re-locks it.

    Note that globals are visible from both cores. Note also that GPIO
    pin mappings performed on core 0 can be utilized from core 1.
    Creation of an alarm pool is required to force a timer interrupt to
    take place on core 1 rather than core 0.

    GPIO 5 (pin 7) Chip select
    GPIO 6 (pin 9) SCK/spi0_sclk
    GPIO 7 (pin 10) MOSI/spi0_tx
    GPIO 2 (pin 4) GPIO output for timing ISR on core 0
    GPIO 3 (pin 5) GPIO output for timing ISR on core 1
    3.3v (pin 36) -> VCC on DAC 
    GND (pin 3)  -> GND on DAC 

 */

#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/sync.h"
#include "hardware/spi.h"
#include "hardware/timer.h"
#include "hardware/irq.h"

// === the fixed point macros ========================================
typedef signed int fix15 ;
#define multfix15(a,b) ((fix15)((((signed long long)(a))*((signed long long)(b)))>>15))
#define float2fix15(a) ((fix15)((a)*32768.0)) 
#define fix2float15(a) ((float)(a)/32768.0)
#define absfix15(a) abs(a) 
#define int2fix15(a) ((fix15)(a << 15))
#define fix2int15(a) ((int)(a >> 15))
#define char2fix15(a) (fix15)(((fix15)(a)) << 15)
#define divfix(a,b) (fix15)( (((signed long long)(a)) << 15) / (b))

// Alarm infrastructure that we'll be using
#define ALARM_NUM_0 0
#define ALARM_NUM_1 1
#define ALARM_IRQ_0 TIMER_IRQ_0
#define ALARM_IRQ_1 TIMER_IRQ_1

//DDS parameters
#define two32 4294967296.0 // 2^32 
#define Fs 50000
#define DELAY 20 // 1/Fs (in microseconds)


// the DDS units - core 1
volatile unsigned int phase_accum_main_1;
volatile unsigned int phase_incr_main_1 = (800.0*two32)/Fs ;
// the DDS units - core 2
volatile unsigned int phase_accum_main_0;
volatile unsigned int phase_incr_main_0 = (400.0*two32)/Fs ;


// DDS sine table
#define sine_table_size 256
fix15 sin_table[sine_table_size] ;


// Values output to DAC
int DAC_output_0 ;
int DAC_output_1 ;


// Amplitude modulation parameters and variables
fix15 max_amplitude = int2fix15(1) ;
fix15 attack_inc ;
fix15 decay_inc ;
fix15 current_amplitude_0 = 0 ;
fix15 current_amplitude_1 = 0 ;
#define ATTACK_TIME             250
#define DECAY_TIME              250
#define SUSTAIN_TIME            10000
#define BEEP_DURATION           10500
#define BEEP_REPEAT_INTERVAL    50000


//SPI data
uint16_t DAC_data_1 ; // output value
uint16_t DAC_data_0 ; // output value


//DAC parameters
// A-channel, 1x, active
#define DAC_config_chan_A 0b0011000000000000
// B-channel, 1x, active
#define DAC_config_chan_B 0b1011000000000000


//SPI configurations
#define PIN_MISO 4
#define PIN_CS   5
#define PIN_SCK  6
#define PIN_MOSI 7
#define LDAC     8
#define SPI_PORT spi0


// GPIO's for ISR timing
#define ISR_0 2
#define ISR_1 3

// Two variables to store core number
volatile int corenum_0  ;
volatile int corenum_1  ;

// Global counter for spinlock experimenting
volatile int global_counter = 0 ;

// Counter spinlock
int spinlock_num_count ;
spin_lock_t *spinlock_count ;

// State machine variables
volatile unsigned int STATE_0 = 0 ;
volatile unsigned int count_0 = 0 ;
volatile unsigned int STATE_1 = 0 ;
volatile unsigned int count_1 = 0 ;

// This one is called on core 0
static void alarm_irq_0(void) {

    // Assert GPIO for timing interrupt
    gpio_put(ISR_0, 1) ;

    // Clear the alarm irq
    hw_clear_bits(&timer_hw->intr, 1u << ALARM_NUM_0);

    // Reset the alarm register
    timer_hw->alarm[ALARM_NUM_0] = timer_hw->timerawl + DELAY ;

    if (STATE_0 == 0) {
        // DDS phase and sine table lookup
        phase_accum_main_0 += phase_incr_main_0  ;
        DAC_output_0 = fix2int15(multfix15(current_amplitude_0,
            sin_table[phase_accum_main_0>>24])) + 2048 ;

        // Ramp up amplitude
        if (count_0 < ATTACK_TIME) {
            current_amplitude_0 = (current_amplitude_0 + attack_inc) ;
        }
        // Ramp down amplitude
        else if (count_0 > BEEP_DURATION - DECAY_TIME) {
            current_amplitude_0 = (current_amplitude_0 - decay_inc) ;
        }

        DAC_data_0 = (DAC_config_chan_B | (DAC_output_0 & 0xffff))  ;

        // SPI write (no spinlock b/c of SPI buffer)
        spi_write16_blocking(SPI_PORT, &DAC_data_0, 1) ;

        // Increment the counter
        count_0 += 1 ;

        // State transition?
        if (count_0 == BEEP_DURATION) {
            STATE_0 = 1 ;
            count_0 = 0 ;
        }
    }

    // State transition?
    else {
        count_0 += 1 ;
        if (count_0 == BEEP_REPEAT_INTERVAL) {
            current_amplitude_0 = 0 ;
            STATE_0 = 0 ;
            count_0 = 0 ;
        }
    }

    // retrieve core number of execution
    corenum_0 = get_core_num() ;

    // De-assert GPIO for timing interrupt
    gpio_put(ISR_0, 0) ;

}

// This one is called on core 1
static void alarm_irq_1(void) {

    // Assert GPIO for timing interrupt
    gpio_put(ISR_1, 1) ;

    // Clear the alarm irq
    hw_clear_bits(&timer_hw->intr, 1u << ALARM_NUM_1);

    // Reset the alarm register
    timer_hw->alarm[ALARM_NUM_1] = timer_hw->timerawl + DELAY ;

    if (STATE_1 == 0) {
        // DDS phase and sine table lookup
        phase_accum_main_1 += phase_incr_main_1  ;
        DAC_output_1 = fix2int15(multfix15(current_amplitude_1,
            sin_table[phase_accum_main_1>>24])) + 2048 ;

        // Ramp up amplitude
        if (count_1 < ATTACK_TIME) {
            current_amplitude_1 = (current_amplitude_1 + attack_inc) ;
        }
        // Ramp down amplitude
        else if (count_1 > BEEP_DURATION - DECAY_TIME) {
            current_amplitude_1 = (current_amplitude_1 - decay_inc) ;
        }

        // Mask with DAC control bits
        DAC_data_1 = (DAC_config_chan_A | (DAC_output_1 & 0xffff))  ;

        // SPI write (no spinlock b/c of SPI buffer)
        spi_write16_blocking(SPI_PORT, &DAC_data_1, 1) ;

        // Increment the counter
        count_1 += 1 ;

        // State transition?
        if (count_1 == BEEP_DURATION) {
            STATE_1 = 1 ;
            count_1 = 0 ;
        }
    }

    // State transition?
    else {
        count_1 += 1 ;
        if (count_1 == BEEP_REPEAT_INTERVAL) {
            current_amplitude_1 = 0 ;
            STATE_1 = 0 ;
            count_1 = 0 ;
        }
    }

    // retrieve core number of execution
    corenum_1 = get_core_num() ;

    // De-assert GPIO for timing interrupt
    gpio_put(ISR_1, 0) ;

}

void core1_entry() {

    // Enable the interrupt for the alarm (we're using Alarm 1)
    hw_set_bits(&timer_hw->inte, 1u << ALARM_NUM_1) ;
    // Associate an interrupt handler with the ALARM_IRQ
    irq_set_exclusive_handler(ALARM_IRQ_1, alarm_irq_1) ;
    // Enable the alarm interrupt
    irq_set_enabled(ALARM_IRQ_1, true) ;
    // Write the lower 32 bits of the target time to the alarm register, arming it.
    timer_hw->alarm[ALARM_NUM_1] = timer_hw->timerawl + DELAY ;

    while (1) {

        // Lock spinlock (without disabling interrupts)
        spin_lock_unsafe_blocking(spinlock_count) ;
        // Increment global counter variable
        for (int i=0; i<10; i++) {
            global_counter += 1 ;
            sleep_ms(250) ;
            printf("Core 1: %d, ISR core: %d\n", global_counter, corenum_1) ;
        }
        printf("\n\n") ;
        // Unlock spinlock
        spin_unlock_unsafe(spinlock_count) ;
        // A short delay to make sure the other core unlocks the spinlock
        // before this one unlocks it again (experimentation shows that
        // this is necessary)
        sleep_ms(1);

    }
}

int main() {
    // Initialize stdio/uart
    stdio_init_all();
    printf("Hello, multicore!\n");

    // Initialize SPI channel (channel, baud rate set to 20MHz)
    spi_init(SPI_PORT, 20000000) ;
    // Format (channel, data bits per transfer, polarity, phase, order)
    spi_set_format(SPI_PORT, 16, 0, 0, 0);

    // Map SPI signals to GPIO ports
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS, GPIO_FUNC_SPI) ;

    // Map LDAC pin to GPIO port, hold it low
    gpio_init(LDAC) ;
    gpio_set_dir(LDAC, GPIO_OUT) ;
    gpio_put(LDAC, 0) ;

    // Setup the ISR-timing GPIO
    gpio_init(ISR_0) ;
    gpio_set_dir(ISR_0, GPIO_OUT);
    gpio_put(ISR_0, 0) ;
    gpio_init(ISR_1) ;
    gpio_set_dir(ISR_1, GPIO_OUT);
    gpio_put(ISR_1, 0) ;

    // set up increments for calculating bow envelope
    attack_inc = divfix(max_amplitude, int2fix15(ATTACK_TIME)) ;//max_amplitude/(float)ATTACK_TIME ;
    decay_inc =  divfix(max_amplitude, int2fix15(DECAY_TIME)) ;//max_amplitude/(float)DECAY_TIME ;

    // === build the sine lookup table =======
    // scaled to produce values between 0 and 4096
    int ii;
    for (ii = 0; ii < sine_table_size; ii++){
         sin_table[ii] = float2fix15(2047*sin((float)ii*6.283/(float)sine_table_size));
    }

    // Claim and initialize a spinlock
    spinlock_num_count = spin_lock_claim_unused(true) ;
    spinlock_count = spin_lock_init(spinlock_num_count) ;

    // Launch core 1
    multicore_launch_core1(core1_entry);

    // Desyncrhonize the beeps
    sleep_ms(500) ;

     // Enable the interrupt for the alarm (we're using Alarm 1)
    hw_set_bits(&timer_hw->inte, 1u << ALARM_NUM_0) ;
    // Associate an interrupt handler with the ALARM_IRQ
    irq_set_exclusive_handler(ALARM_IRQ_0, alarm_irq_0) ;
    // Enable the alarm interrupt
    irq_set_enabled(ALARM_IRQ_0, true) ;
    // Write the lower 32 bits of the target time to the alarm register, arming it.
    timer_hw->alarm[ALARM_NUM_0] = timer_hw->timerawl + DELAY ;


    while(1) {

        // Lock spinlock (without disabling interrupts)
        spin_lock_unsafe_blocking(spinlock_count) ;
        // Increment global counter
        for (int i=0; i<10; i++) {
            global_counter += 1 ;
            sleep_ms(250) ;
            printf("Core 0: %d, ISR core: %d\n", global_counter, corenum_0) ;
        }
        printf("\n\n") ;
        // Unlock spinlock
        spin_unlock_unsafe(spinlock_count) ;
        // A short delay to make sure the other core locks before this
        // one locks the spinlock again (experimentation shows that
        // this is necessary)
        sleep_ms(1);

    }
    return 0 ;
}