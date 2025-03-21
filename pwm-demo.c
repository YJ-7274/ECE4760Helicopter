/**
 * V. Hunter Adams (vha3@cornell.edu)
 * PWM demo code with serial input
 * 
 * This demonstration sets a PWM duty cycle to a
 * user-specified value.
 * 
 * HARDWARE CONNECTIONS
 *   - GPIO 4 ---> PWM output
 **  - GPIO 16 ---> VGA Hsync
 *  - GPIO 17 ---> VGA Vsync
 *  - GPIO 18 ---> 330 ohm resistor ---> VGA Red
 *  - GPIO 19 ---> 330 ohm resistor ---> VGA Green
 *  - GPIO 20 ---> 330 ohm resistor ---> VGA Blue
 *  - RP2040 GND ---> VGA GND
 *  - GPIO 8 ---> MPU6050 SDA
 *  - GPIO 9 ---> MPU6050 SCL
 *  - 3.3v ---> MPU6050 VCC
 *  - RP2040 GND ---> MPU6050 GND
 */

 #include <stdio.h>
 #include <stdlib.h>
 #include <math.h>
 #include <string.h>
 
 #include "pico/stdlib.h"
 #include "pico/multicore.h"
 
 #include "hardware/pwm.h"
 #include "hardware/irq.h"
 #include "hardware/dma.h"
 #include "hardware/adc.h"
 #include "hardware/pio.h"
 #include "hardware/i2c.h"
 // Include custom libraries
 #include "vga16_graphics.h"
 #include "mpu6050.h"
 #include "pt_cornell_rp2040_v1_3.h"
 
 
 // Arrays in which raw measurements will be stored
 fix15 acceleration[3], filter_accel[3], gyro[3], accel_angle, gyro_angle_delta, complementary_angle, error_tracker;
 
 // character array
 char screentext[40];
 
 // draw speed
 int threshold = 10 ;
 
 // Some macros for max/min/abs
 #define min(a,b) ((a<b) ? a:b)
 #define max(a,b) ((a<b) ? b:a)
 #define abs(a) ((a>0) ? a:-a)
 
 // semaphore
 static struct pt_sem vga_semaphore ;
 
 // PWM wrap value and clock divide value
 // For a CPU rate of 125 MHz, this gives
 // a PWM frequency of 1 kHz.
 #define WRAPVAL 5000
 #define CLKDIV 25.0f
 
 // GPIO we're using for PWM
 #define PWM_OUT 4
 
 // Variable to hold PWM slice number
 uint slice_num ;
 
 // PWM duty cycle
 volatile int control;
 volatile int control_filtered;
 volatile int old_control;
 
 // Control variables
 volatile float target_angle = 0.0f;  // Target angle in degrees
 volatile float Kp = 20.0f;           // Proportional gain (adjust based on system response)
 volatile float Kd = 40.0f;           // Derivative Term (2-3x Kp) ADJUST!
 volatile float Ki = 10.0f;           // Integral Term (same order as Kp) ADJSUT! 

 // PWM interrupt service routine
 void on_pwm_wrap() {
     // Clear the interrupt flag that brought us here
     pwm_clear_irq(pwm_gpio_to_slice_num(PWM_OUT));
     mpu6050_read_raw(acceleration, gyro);
     
 
     filter_accel[0] = filter_accel[0] + ((acceleration[0] - filter_accel[0])>>6);
     filter_accel[1] = filter_accel[1] + ((acceleration[1] - filter_accel[1])>>6);
     filter_accel[2] = filter_accel[2] + ((acceleration[2] - filter_accel[2])>>6);
 
     // SMALL ANGLE APPROXIMATION
     accel_angle = multfix15(float2fix15(atan2(filter_accel[1], filter_accel[2]) + M_PI), oneeightyoverpi) - float2fix15(90.0);
 
     // Gyro angle delta (measurement times timestep) (15.16 fixed point)
     gyro_angle_delta = multfix15(gyro[0], zeropt001) ;
 
     // Complementary angle (degrees - 15.16 fixed point)
     complementary_angle = multfix15(complementary_angle + gyro_angle_delta, zeropt999) + multfix15(accel_angle, zeropt001);
 
     // P CONTROL LOGIC
     float integral_sum = 0.0f;
     float prev_error = 0.0f;
    
     float current_angle = fix2float15(complementary_angle); // Convert to degrees
     float delta_t = 1.0f;
     float error = target_angle - current_angle;
    
     //PID Calculations     
     float proportional = Kp * error;
     integral_sum += Ki* error * delta_t; 
     float derivative = Kd * (error - prev_error) / delta_t;

     float output = proportional + integral_sum + derivative;
     error_tracker = float2fix15(error);
     int control = (int)output;
     control_filtered = control;
     control_filtered = control_filtered + ((control - control_filtered)>>2);
 
     // Clamp duty cycle between 0 and 5000
     if (control > 3500.0f) control = 3500.0f;
     else if (control < 0.0f) control = 0.0f;
     
 
     // Update duty cycle
     if (control!=old_control) {
         old_control = control ;
         pwm_set_chan_level(slice_num, PWM_CHAN_A, control);
     }
 
     // Signal VGA to draw
     PT_SEM_SIGNAL(pt, &vga_semaphore);
 }
 
 // User input thread
 static PT_THREAD (protothread_serial(struct pt *pt))
 {
     PT_BEGIN(pt) ;
     static float input_angle;
     static float input_kp;
     static float input_kd;
     static float input_ki;
     while(1) {
         sprintf(pt_serial_out_buffer, "Enter target angle (degrees): ");
         serial_write;
         serial_read;
         sscanf(pt_serial_in_buffer, "%f", &input_angle);
         target_angle = input_angle; // Update global target
         
         sprintf(pt_serial_out_buffer, "Enter PID P Constant: ");
         serial_write;
         serial_read;
         sscanf(pt_serial_in_buffer, "%f", &input_kp);
         Kp = input_kp;

         sprintf(pt_serial_in_buffer, "%f", "Enter PID Kd");
         serial_write;
         serial_read;
         sscanf(pt_serial_in_buffer, "%f", &input_kd)
         Kd = input_kd;

         sprintf(pt_serial_in_buffer, "%f", "Enter PID Ki");
         serial_write;
         serial_read;
         sscanf(pt_serial_in_buffer, "%f", &input_ki)
         Ki = input_ki;
        
     }
     
     PT_END(pt) ;
 }
 
 // Thread that draws to VGA display
 static PT_THREAD (protothread_vga(struct pt *pt))
 {
     // Indicate start of thread
     PT_BEGIN(pt) ;
 
     // We will start drawing at column 81
     static int xcoord = 81 ;
     
     // Rescale the measurements for display
     static float OldRange = 500. ; // (+/- 250)
     static float NewRange = 150. ; // (looks nice on VGA)
     static float OldMin = -250. ;
     static float OldMax = 250. ;
 
     // Control rate of drawing
     static int throttle ;
 
     // Counter for beam angle refresh
     static int beam_refresh_counter = 0;
     static const int BEAM_REFRESH_INTERVAL = 20; // Adjust this value to control refresh rate
 
     // Draw the static aspects of the display
     setTextSize(1) ;
     setTextColor(WHITE);
     // Draw bottom plot
     drawHLine(75, 430, 5, CYAN) ;
     drawHLine(75, 355, 5, CYAN) ;
     drawHLine(75, 280, 5, CYAN) ;
     drawVLine(80, 280, 150, CYAN) ;
     sprintf(screentext, "0") ;
     setCursor(50, 350) ;
     writeString(screentext) ;
     sprintf(screentext, "180") ;
     setCursor(50, 280) ;
     writeString(screentext) ;
     sprintf(screentext, "-180") ;
     setCursor(50, 425) ;
     writeString(screentext) ;
 
     // Draw top plot
     drawHLine(75, 230, 5, CYAN) ;
     drawHLine(75, 155, 5, CYAN) ;
     drawHLine(75, 80, 5, CYAN) ;
     drawVLine(80, 80, 150, CYAN) ;
     sprintf(screentext, "0") ;
     setCursor(50, 150) ;
     writeString(screentext) ;
     sprintf(screentext, "3500") ;
     setCursor(45, 75) ;
     writeString(screentext) ;
     sprintf(screentext, "-3500") ;
     setCursor(45, 225) ;
     writeString(screentext) ;
     while (true) {
         // Wait on semaphore
         PT_SEM_WAIT(pt, &vga_semaphore);
         // Increment beam refresh counter
         beam_refresh_counter++;
 
         // Update beam angle display only if counter reaches the interval
         if (beam_refresh_counter >= BEAM_REFRESH_INTERVAL) {
             beam_refresh_counter = 0; // Reset counter
 
             // Clear the previous beam angle display
             fillRect(120, 50, 100, 50, BLACK);
             setCursor(50, 50);
             sprintf(screentext, "Beam Angle:  %d", fix2int15(complementary_angle));
             writeString(screentext);
 
             fillRect(120, 60, 100, 50, BLACK);
             setCursor(50, 60);
             sprintf(screentext, "Error Angle:  %d", fix2int15(error_tracker));
             writeString(screentext);
         }
         // setCursor(50, 75)
         // sprintf(screentext, "Low Pass Motor:  %5.1f", fix2float15(complementary_angle));
         // // Increment drawspeed controller
         throttle += 1 ;
         // If the controller has exceeded a threshold, draw
         if (throttle >= threshold) { 
             // Zero drawspeed controller
             throttle = 0 ;
 
             // Erase a column
             drawVLine(xcoord, 0, 480, BLACK) ;
 
             // Draw bottom plot (multiply by 120 to scale from +/-2 to +/-250)
             drawPixel(xcoord, 430 - (int)(NewRange*((float)((fix2float15(complementary_angle )*250.0/180.0)-OldMin)/OldRange)), WHITE) ;
 
             // Draw top plot
             drawPixel(xcoord, 230 - (int)(NewRange*((float)((control_filtered / 7.0f) -OldMin)/OldRange)), RED) ;
             // Update horizontal cursor
             if (xcoord < 609) {
                 xcoord += 1 ;
             }
             else {
                 xcoord = 81 ;
             }
         }
     }
     // Indicate end of thread
     PT_END(pt);
 }
 
 void core1_entry() {
     pt_add_thread(protothread_vga) ;
     pt_schedule_start ;
 }
 
 int main() {
 
     // Initialize stdio
     stdio_init_all();
     initVGA();
     ////////////////////////////////////////////////////////////////////////
     ///////////////////////// I2C CONFIGURATION ////////////////////////////
     i2c_init(I2C_CHAN, I2C_BAUD_RATE) ;
     gpio_set_function(SDA_PIN, GPIO_FUNC_I2C) ;
     gpio_set_function(SCL_PIN, GPIO_FUNC_I2C) ;
 
     // MPU6050 initialization
     mpu6050_reset();
     mpu6050_read_raw(acceleration, gyro);
 
     ////////////////////////////////////////////////////////////////////////
     ///////////////////////// PWM CONFIGURATION ////////////////////////////
     ////////////////////////////////////////////////////////////////////////
     // Tell GPIO PWM_OUT that it is allocated to the PWM
     gpio_set_function(PWM_OUT, GPIO_FUNC_PWM);
 
     // Find out which PWM slice is connected to GPIO PWM_OUT (it's slice 2)
     slice_num = pwm_gpio_to_slice_num(PWM_OUT);
 
     // Mask our slice's IRQ output into the PWM block's single interrupt line,
     // and register our interrupt handler
     pwm_clear_irq(slice_num);
     pwm_set_irq_enabled(slice_num, true);
     irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
     irq_set_enabled(PWM_IRQ_WRAP, true);
 
     // This section configures the period of the PWM signals
     pwm_set_wrap(slice_num, WRAPVAL) ;
     pwm_set_clkdiv(slice_num, CLKDIV) ;
 
     //invert
     pwm_set_output_polarity (slice_num, 1, 1);
 
     // This sets duty cycle
     pwm_set_chan_level(slice_num, PWM_CHAN_A, 3125);
 
     // Start the channel
     pwm_set_mask_enabled((1u << slice_num));
 
     ////////////////////////////////////////////////////////////////////////
     ///////////////////////////// ROCK AND ROLL ////////////////////////////
     ////////////////////////////////////////////////////////////////////////
     multicore_reset_core1();
     multicore_launch_core1(core1_entry);
     pt_add_thread(protothread_serial) ;
     pt_schedule_start ;
 
 }
 