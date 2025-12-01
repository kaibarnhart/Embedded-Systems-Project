#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "eecs388_lib.h"

//
// Task 1: Lidar distance parsing + LED control
//
void auto_brake(int devid)
{
    uint8_t buf[9];
    uint16_t distance = 0;

    // Wait for the TFmini 0x59 0x59 header
    while (1) {
        if (ser_read(devid) == 0x59) {          // first header byte
            if (ser_read(devid) == 0x59) {      // second header byte
                buf[0] = 0x59;
                buf[1] = 0x59;
                break;                          // we got the header
            }
        }
    }

    // Read the next 7 bytes of the frame
    for (int i = 2; i < 9; i++) {
        buf[i] = ser_read(devid);
    }

    // Distance is in buf[2] + buf[3] (little endian)
    distance = (buf[3] << 8) | buf[2];
    printf("Distance: %d cm\n", distance);

    // Basic LED behavior depending on distance
    if (distance > 200) {
        gpio_write(RED_LED,   OFF);
        gpio_write(GREEN_LED, ON);
        gpio_write(BLUE_LED,  OFF);
    }
    else if (distance > 100) {
        gpio_write(RED_LED,   ON);
        gpio_write(GREEN_LED, ON);
        gpio_write(BLUE_LED,  OFF);
    }
    else if (distance > 60) {
        gpio_write(RED_LED,   ON);
        gpio_write(GREEN_LED, OFF);
        gpio_write(BLUE_LED,  OFF);
    }
    else {
        // Under 60 cm = danger zone → blink red only
        gpio_write(GREEN_LED, OFF);
        gpio_write(BLUE_LED,  OFF);

        gpio_write(RED_LED, ON);
        delay(100);
        gpio_write(RED_LED, OFF);
        delay(100);
    }
}

//
// Task 2: Read steering angle from Raspberry Pi
//
int read_angle(int devid)
{
    char buf[16];
    int angle = 0;

    // Only read if serial has something ready
    if (ser_isready(devid)) {
        int ret = ser_readline(devid, 16, buf);
        if (ret > 0) {
            sscanf(buf, "%d", &angle);   // convert text → int
        }
    }

    // Clamp angle range so servo doesn’t freak out
    if (angle < 0)   angle = 0;
    if (angle > 180) angle = 180;

    return angle;
}

//
// Task 3: Move servo using PWM pulses
//
void steering(int gpio, int pos)
{
    // Map 0–180° → 1000–2000 microseconds
    int pulse_width = 1000 + (pos * 1000 / 180);

    // Send ~80 pulse cycles (smooth movement)
    for (int i = 0; i < 80; i++) {
        gpio_write(gpio, ON);
        delay_usec(pulse_width);

        gpio_write(gpio, OFF);
        delay_usec(20000 - pulse_width);   // 20ms standard servo frame
    }
}

//
// Main program
//
int main()
{
    printf("Program started.\n");

    ser_setup(0); // UART0 = Lidar
    ser_setup(1); // UART1 = Pi (steering angle data)

    gpio_mode(PIN_19, OUTPUT);  // Servo pin
    gpio_mode(RED_LED, OUTPUT);
    gpio_mode(GREEN_LED, OUTPUT);
    gpio_mode(BLUE_LED, OUTPUT);

    printf("Setup complete. Running loop...\n");

    while (1) {
        auto_brake(0);            // Task 1: braking logic
        int angle = read_angle(1); // Task 2: angle from Pi
        printf("angle=%d\n", angle);
        steering(PIN_19, angle);   // Task 3: control servo
    }

    return 0;
}
