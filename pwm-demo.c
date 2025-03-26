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
 // Button GPIO
 #define BUTTON_PIN 2
 
 // Variable to hold PWM slice number
 uint slice_num ;
 
 // PWM duty cycle
 volatile int control;
 volatile int control_filtered;
 volatile int old_control;
 
 // Control variables
 volatile float target_angle = 0.0f;  // Target angle in degrees
 volatile float prev_error = 0.0f;
 volatile float prev_error2 = 0.0f;
 volatile float prev_error3 = 0.0f;
 volatile float prev_error4 = 0.0f;
 volatile float Kp = 200.0f;           // Proportional gain (adjust based on system response)
 volatile float Kd = 18000.0f;         // Derivative Term (10^(2-3)x Kp) ADJUST!
 volatile float Ki = 1.5f;           // Integral Term (same order as Kp) ADJSUT! 
 volatile float derivative = 0.0f;
 volatile float integral_sum = 0.0f;
 
 bool button_pressed  = false;
 volatile int timer = 0;
 
 // PWM interrupt service routine
 void on_pwm_wrap() {
     // Clear the interrupt flag that brought us here
     pwm_clear_irq(pwm_gpio_to_slice_num(PWM_OUT));
     mpu6050_read_raw(acceleration, gyro);
 
     filter_accel[0] = filter_accel[0] + ((acceleration[0] - filter_accel[0])>>6);
     filter_accel[1] = filter_accel[1] + ((acceleration[1] - filter_accel[1])>>6);
     filter_accel[2] = filter_accel[2] + ((acceleration[2] - filter_accel[2])>>6);
 
     // SMALL ANGLE APPROXIMATION
     accel_angle = multfix15(float2fix15(atan2(fix2float15(filter_accel[1]), fix2float15(filter_accel[2])) + M_PI), oneeightyoverpi) - float2fix15(90.0f);
 
     // Gyro angle delta (measurement times timestep) (15.16 fixed point)
     gyro_angle_delta = multfix15(gyro[0], zeropt001) ;
 
     // Complementary angle (degrees - 15.16 fixed point)
     complementary_angle = multfix15(complementary_angle + gyro_angle_delta, zeropt999) + multfix15(accel_angle, zeropt001);
     
     // P CONTROL LOGIC
     
     float delta_t = 0.01f;
 
     float current_angle = fix2float15(complementary_angle); // Convert to degrees
     float error = target_angle - current_angle;
     
     float proportional = (Kp * error);
     
     derivative = Kd * ((error - prev_error4));
     prev_error4 = prev_error3;
     prev_error3 = prev_error2;
     prev_error2 = prev_error;
     prev_error = error;
     
     if (target_angle != 0){
         integral_sum += error * delta_t; 
         if (integral_sum * Ki > 2000.0f){
             integral_sum = 2000.0f / Ki;
         }
     }
     float output = proportional + (Ki * integral_sum) + derivative;
     // float output = proportional + derivative;
 
     //float output = Kp * error ;
     error_tracker = float2fix15(error);
     control = (int)output;
     if (target_angle == 0){
         control = 0.0f;
     }
     // Clamp duty cycle between 0 and 5000
     if (control > 2000.0f) control = 2000.0f;
     else if (control < 0.0f) control = 0.0f;
     control_filtered = control;
     control_filtered = control_filtered + ((control - control_filtered)>>6);
     
 
     // Update duty cycle
     if (control!=old_control) {
         old_control = control ;
         pwm_set_chan_level(slice_num, PWM_CHAN_A, control);
     }
 
     // Signal VGA to draw
     if (timer % 5 == 0){
         PT_SEM_SIGNAL(pt, &vga_semaphore);
     }
 
     timer++;
 
 }
 
 static PT_THREAD (protothread_control(struct pt *pt))
 {
     PT_BEGIN(pt) ;
     while(1) {
         if (gpio_get(BUTTON_PIN)) {
             PT_YIELD_usec(10000);
             if (button_pressed) {
                 sprintf(pt_serial_out_buffer, "Button Pressed");
                 timer = 0;
                 while(timer < 16000){
                     PT_YIELD_usec(10000);
                     if (timer < 5000){
                         target_angle = 90;
                     }
                     else if (timer < 10000){
                         target_angle = 120.0f;
                     }
                     else if (timer < 15000){
                         target_angle = 60.0f;
                     }
                     else{
                         target_angle = 90.0f;
                     }
                 }
             }
             button_pressed = false;
         }
         else{
             target_angle = 0.0f;
             button_pressed = true;
         }
     }
     PT_END(pt) ;
 }
 
 // User input thread
 static PT_THREAD (protothread_serial(struct pt *pt))
 {
     PT_BEGIN(pt) ;
     static float input_angle;
     static float input_kp, input_ki, input_kd;
     char command[100];
     while(1) {
         
         sprintf(pt_serial_out_buffer, "Enter command (e.g., 'set angle 90' or 'set kp 30'): ");
         serial_write;
         serial_read;
         
         // Read the user input
         sscanf(pt_serial_in_buffer, "%s %f", command, &input_angle); // Read the command
         
         // Check for 'set angle' command
         if (strcmp(command, "set") == 0 && strstr(pt_serial_in_buffer, "angle") != NULL) {
             sscanf(pt_serial_in_buffer, "set angle %f", &input_angle);
             target_angle = input_angle;  // Update the global target angle
             sprintf(pt_serial_out_buffer, "Target angle set to: %.2f degrees\n", target_angle);
             serial_write;
         }
         
         // Check for 'set kp' command
         else if (strcmp(command, "set") == 0 && strstr(pt_serial_in_buffer, "kp") != NULL) {
             sscanf(pt_serial_in_buffer, "set kp %f", &input_kp);
             Kp = input_kp;  // Update the global Kp value
             sprintf(pt_serial_out_buffer, "Kp set to: %.2f\n", Kp);
             serial_write;
         }
 
         else if (strcmp(command, "set") == 0 && strstr(pt_serial_in_buffer, "ki") != NULL) {
             sscanf(pt_serial_in_buffer, "set ki %f", &input_ki);
             Ki = input_ki;  // Update the global Kp value
             sprintf(pt_serial_out_buffer, "Ki set to: %.2f\n", Ki);
             serial_write;
         }
 
         else if (strcmp(command, "set") == 0 && strstr(pt_serial_in_buffer, "kd") != NULL) {
             sscanf(pt_serial_in_buffer, "set kd %f", &input_kd);
             Kd = input_kd;  // Update the global Kp value
             sprintf(pt_serial_out_buffer, "Kd set to: %.2f\n", Kd);
             serial_write;
         }
         
         else {
             sprintf(pt_serial_out_buffer, "Invalid command. Please use 'set angle' or 'set kp/ki/kd'.\n");
             serial_write;
         }
         
         serial_read;  // Read input after printing the response
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
     sprintf(screentext, "90") ;
     setCursor(50, 350) ;
     writeString(screentext) ;
     sprintf(screentext, "180") ;
     setCursor(50, 280) ;
     writeString(screentext) ;
     sprintf(screentext, "0") ;
     setCursor(50, 425) ;
     writeString(screentext) ;
 
     // Draw top plot
     drawHLine(75, 230, 5, CYAN) ;
     drawHLine(75, 155, 5, CYAN) ;
     drawHLine(75, 80, 5, CYAN) ;
     drawVLine(80, 80, 150, CYAN) ;
     sprintf(screentext, "1250") ;
     setCursor(50, 150) ;
     writeString(screentext) ;
     sprintf(screentext, "2500") ;
     setCursor(45, 75) ;
     writeString(screentext) ;
     sprintf(screentext, "0") ;
     setCursor(45, 225) ;
     writeString(screentext) ;
     while (true) {
         // Wait on semaphore
         // for(int i = 0; i < 5; i++){
             PT_SEM_WAIT(pt, &vga_semaphore);
         // }
         // Increment beam refresh counter
         beam_refresh_counter++;
 
         // Update beam angle display only if counter reaches the interval
         if (beam_refresh_counter >= BEAM_REFRESH_INTERVAL) {
             beam_refresh_counter = 0; // Reset counter
 
             // Clear the previous beam angle display
             fillRect(120, 50, 100, 10, BLACK);
             setCursor(50, 50);
             sprintf(screentext, "Beam Angle:  %d", fix2int15(complementary_angle));
             writeString(screentext);
 
             fillRect(120, 60, 100, 10, BLACK);
             setCursor(50, 60);
             sprintf(screentext, "Error Angle:  %f", fix2float15(error_tracker));
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
             drawPixel(xcoord, 430 - (fix2int15(complementary_angle) * 150 / 180), WHITE) ;
 
             // Draw top plot
             drawPixel(xcoord, (230 - control_filtered * 150 / 2500), RED) ;
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
 
     //Button pin 
     gpio_init(BUTTON_PIN);
     gpio_set_dir(BUTTON_PIN, GPIO_IN);  // Set the pin as an input
     gpio_pull_up(BUTTON_PIN); 
 
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
     pwm_set_chan_level(slice_num, PWM_CHAN_A, 0);
 
     // Start the channel
     pwm_set_mask_enabled((1u << slice_num));
 
     ////////////////////////////////////////////////////////////////////////
     ///////////////////////////// ROCK AND ROLL ////////////////////////////
     ////////////////////////////////////////////////////////////////////////
     multicore_reset_core1();
     multicore_launch_core1(core1_entry);
     pt_add_thread(protothread_serial);
     pt_add_thread(protothread_control);
     pt_schedule_start ;
 
 }
 