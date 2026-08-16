#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/mman.h>
#include <sys/neutrino.h>
#include <hw/inout.h>
#include <unistd.h>
#include <time.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>

/* ============================================================
 * GPIO
 * ============================================================ */

#define GPIO_BASE   0xFE200000
#define BLOCK_SIZE  4096

#define GPFSEL0     (0x00 / 4)
#define GPFSEL1     (0x04 / 4)
#define GPFSEL2     (0x08 / 4)

#define GPSET0      (0x1C / 4)
#define GPCLR0      (0x28 / 4)
#define GPLEV0      (0x34 / 4)


/* ============================================================
 * L298N MOTOR DRIVER
 *
 * Motor A:
 * ENA = GPIO18
 * IN1 = GPIO23
 * IN2 = GPIO24
 *
 * Motor B:
 * ENB = GPIO13
 * IN3 = GPIO25
 * IN4 = GPIO8
 * ============================================================ */

#define ENA_GPIO    18
#define IN1_GPIO    23
#define IN2_GPIO    24

#define ENB_GPIO    13
#define IN3_GPIO    25
#define IN4_GPIO    8


#define GPIO_MASK(pin)   (1u << (pin))

#define ENA    GPIO_MASK(ENA_GPIO)
#define IN1    GPIO_MASK(IN1_GPIO)
#define IN2    GPIO_MASK(IN2_GPIO)

#define ENB    GPIO_MASK(ENB_GPIO)
#define IN3    GPIO_MASK(IN3_GPIO)
#define IN4    GPIO_MASK(IN4_GPIO)


/* ============================================================
 * HC-SR04
 *
 * TRIG = GPIO17
 * ECHO = GPIO27
 * ============================================================ */

#define TRIG_GPIO   17
#define ECHO_GPIO   27

#define TRIG        GPIO_MASK(TRIG_GPIO)
#define ECHO_PIN    GPIO_MASK(ECHO_GPIO)


/* ============================================================
 * SERIAL
 * ============================================================ */

#define SERIAL_PORT "/dev/ser1"


/* ============================================================
 * ULTRASONIC DISTANCE
 * ============================================================ */

double get_distance(volatile uint32_t *gpio)
{
    struct timespec start;
    struct timespec end;

    double timeout_sec = 0.03;


    /* Make sure TRIG is LOW */

    gpio[GPCLR0] = TRIG;

    usleep(2);


    /* 10 us trigger pulse */

    gpio[GPSET0] = TRIG;

    usleep(10);

    gpio[GPCLR0] = TRIG;


    /* --------------------------------------------------------
     * Wait for ECHO HIGH
     * -------------------------------------------------------- */

    clock_gettime(
        CLOCK_MONOTONIC,
        &start
    );


    while (!(gpio[GPLEV0] & ECHO_PIN))
    {
        clock_gettime(
            CLOCK_MONOTONIC,
            &end
        );

        double t =
            (end.tv_sec - start.tv_sec) +
            (end.tv_nsec - start.tv_nsec) / 1e9;

        if (t > timeout_sec)
        {
            return 999;
        }
    }


    /* Echo HIGH started */

    clock_gettime(
        CLOCK_MONOTONIC,
        &start
    );


    /* --------------------------------------------------------
     * Wait for ECHO LOW
     * -------------------------------------------------------- */

    while (gpio[GPLEV0] & ECHO_PIN)
    {
        clock_gettime(
            CLOCK_MONOTONIC,
            &end
        );

        double t =
            (end.tv_sec - start.tv_sec) +
            (end.tv_nsec - start.tv_nsec) / 1e9;

        if (t > timeout_sec)
        {
            return 999;
        }
    }


    /* Echo LOW */

    clock_gettime(
        CLOCK_MONOTONIC,
        &end
    );


    double t =
        (end.tv_sec - start.tv_sec) +
        (end.tv_nsec - start.tv_nsec) / 1e9;


    /* Distance in cm */

    return (t * 34300.0) / 2.0;
}


/* ============================================================
 * MOTOR DIRECTION
 *
 * Both motors forward
 * ============================================================ */

void motor_forward(volatile uint32_t *gpio)
{
    /*
     * Motor A:
     * IN1 = HIGH
     * IN2 = LOW
     *
     * Motor B:
     * IN3 = HIGH
     * IN4 = LOW
     */

    gpio[GPCLR0] =
        IN2 |
        IN4;

    gpio[GPSET0] =
        IN1 |
        IN3;
}


/* ============================================================
 * STOP BOTH MOTORS
 * ============================================================ */

void motor_stop(volatile uint32_t *gpio)
{
    gpio[GPCLR0] =
        ENA |
        ENB;
}


/* ============================================================
 * SOFTWARE PWM
 *
 * PWM period = 10 ms
 * PWM frequency = 100 Hz
 *
 * ENA and ENB are controlled independently.
 * ============================================================ */

void motor_pwm(
    volatile uint32_t *gpio,
    int duty)
{
    if (duty <= 0)
    {
        gpio[GPCLR0] = ENA | ENB;

        usleep(10000);

        return;
    }


    if (duty > 100)
    {
        duty = 100;
    }


    int on_time =
        (10000 * duty) / 100;

    int off_time =
        10000 - on_time;


    /* Both enable pins HIGH */

    gpio[GPSET0] =
        ENA |
        ENB;

    usleep(on_time);


    /* Both enable pins LOW */

    gpio[GPCLR0] =
        ENA |
        ENB;

    usleep(off_time);
}


/* ============================================================
 * GPIO SETUP
 * ============================================================ */

void gpio_setup(volatile uint32_t *gpio)
{
    /* --------------------------------------------------------
     * GPIO8 -> OUTPUT
     *
     * GPFSEL0 bits 24-26
     * -------------------------------------------------------- */

    gpio[GPFSEL0] &= ~(7 << 24);
    gpio[GPFSEL0] |=  (1 << 24);


    /* --------------------------------------------------------
     * GPIO13 -> OUTPUT
     *
     * GPFSEL1 bits 9-11
     * -------------------------------------------------------- */

    gpio[GPFSEL1] &= ~(7 << 9);
    gpio[GPFSEL1] |=  (1 << 9);


    /* --------------------------------------------------------
     * GPIO17 -> OUTPUT
     *
     * GPFSEL1 bits 21-23
     * -------------------------------------------------------- */

    gpio[GPFSEL1] &= ~(7 << 21);
    gpio[GPFSEL1] |=  (1 << 21);


    /* --------------------------------------------------------
     * GPIO18 -> OUTPUT
     *
     * GPFSEL1 bits 24-26
     * -------------------------------------------------------- */

    gpio[GPFSEL1] &= ~(7 << 24);
    gpio[GPFSEL1] |=  (1 << 24);


    /* --------------------------------------------------------
     * GPIO23 -> OUTPUT
     *
     * GPFSEL2 bits 9-11
     * -------------------------------------------------------- */

    gpio[GPFSEL2] &= ~(7 << 9);
    gpio[GPFSEL2] |=  (1 << 9);


    /* --------------------------------------------------------
     * GPIO24 -> OUTPUT
     *
     * GPFSEL2 bits 12-14
     * -------------------------------------------------------- */

    gpio[GPFSEL2] &= ~(7 << 12);
    gpio[GPFSEL2] |=  (1 << 12);


    /* --------------------------------------------------------
     * GPIO25 -> OUTPUT
     *
     * GPFSEL2 bits 15-17
     * -------------------------------------------------------- */

    gpio[GPFSEL2] &= ~(7 << 15);
    gpio[GPFSEL2] |=  (1 << 15);


    /* --------------------------------------------------------
     * GPIO27 -> INPUT
     *
     * GPFSEL2 bits 21-23
     * -------------------------------------------------------- */

    gpio[GPFSEL2] &= ~(7 << 21);


    /* --------------------------------------------------------
     * Initial motor direction
     * -------------------------------------------------------- */

    motor_forward(gpio);


    /* Start with both motors stopped */

    motor_stop(gpio);
}


/* ============================================================
 * MAIN
 * ============================================================ */

int main(void)
{
    int fd;

    struct termios options;


    /* ========================================================
     * QNX I/O PRIVILEGE
     * ======================================================== */

    if (ThreadCtl(
            _NTO_TCTL_IO,
            0) == -1)
    {
        perror("ThreadCtl");

        return EXIT_FAILURE;
    }


    /* ========================================================
     * MAP GPIO
     * ======================================================== */

    void *gpio_ptr =
        mmap_device_memory(
            NULL,
            BLOCK_SIZE,
            PROT_READ |
            PROT_WRITE |
            PROT_NOCACHE,
            0,
            GPIO_BASE
        );


    if (gpio_ptr == MAP_FAILED)
    {
        perror("GPIO mmap");

        return EXIT_FAILURE;
    }


    volatile uint32_t *gpio =
        (volatile uint32_t *)gpio_ptr;


    /* ========================================================
     * GPIO SETUP
     * ======================================================== */

    gpio_setup(gpio);


    /* ========================================================
     * SERIAL PORT
     * ======================================================== */

    fd =
        open(
            SERIAL_PORT,
            O_RDWR |
            O_NOCTTY
        );


    if (fd == -1)
    {
        perror("Serial open");

        return EXIT_FAILURE;
    }


    tcgetattr(
        fd,
        &options
    );


    cfsetispeed(
        &options,
        B9600
    );

    cfsetospeed(
        &options,
        B9600
    );


    options.c_cflag |=
        CLOCAL |
        CREAD;


    tcsetattr(
        fd,
        TCSANOW,
        &options
    );


    /* ========================================================
     * DISTANCE FILTER
     * ======================================================== */

    double d_filtered = 50.0;


    /* ========================================================
     * SYSTEM INFORMATION
     * ======================================================== */

    printf("\n");
    printf("========================================\n");
    printf("       QNX SMART VEHICLE SYSTEM\n");
    printf("========================================\n");

    printf("\nL298N MOTOR A\n");
    printf("ENA = GPIO18\n");
    printf("IN1 = GPIO23\n");
    printf("IN2 = GPIO24\n");

    printf("\nL298N MOTOR B\n");
    printf("ENB = GPIO13\n");
    printf("IN3 = GPIO25\n");
    printf("IN4 = GPIO8\n");

    printf("\nHC-SR04\n");
    printf("TRIG = GPIO17\n");
    printf("ECHO = GPIO27\n");

    printf("\nMPU6050 = NOT USED\n");

    printf("\n========================================\n");
    printf("System Started...\n");
    printf("========================================\n\n");


    /* ========================================================
     * MAIN LOOP
     * ======================================================== */

    while (1)
    {
        /* ----------------------------------------------------
         * Measure distance
         * ---------------------------------------------------- */

        double d_raw =
            get_distance(gpio);


        /* ----------------------------------------------------
         * Invalid reading
         * ---------------------------------------------------- */

        if (d_raw == 999)
        {
            d_raw =
                d_filtered;
        }


        /* ----------------------------------------------------
         * Low-pass filter
         * ---------------------------------------------------- */

        d_filtered =
            0.4 * d_filtered +
            0.6 * d_raw;


        /* Limit maximum */

        if (d_filtered > 200)
        {
            d_filtered = 200;
        }


        /* Limit minimum */

        if (d_filtered < 2)
        {
            d_filtered = 2;
        }


        double distance =
            d_filtered;


        /* ----------------------------------------------------
         * Immediate obstacle override
         * ---------------------------------------------------- */

        if ((d_raw < 20) &&
            (d_raw != 999))
        {
            distance =
                d_raw;
        }


        printf(
            "Raw: %.2f cm | Distance: %.2f cm\n",
            d_raw,
            distance
        );


        /* ====================================================
         * OBSTACLE DECISION
         * ==================================================== */

        int duty;

        char msg[120];


        /* ----------------------------------------------------
         * < 20 cm
         *
         * STOP
         * ---------------------------------------------------- */

        if (distance < 20)
        {
            duty = 0;

            sprintf(
                msg,
                "A1206: SPEED=0 | OBSTACLE\n"
            );
        }


        /* ----------------------------------------------------
         * 20-40 cm
         *
         * 50% PWM
         * ---------------------------------------------------- */

        else if (distance < 40)
        {
            duty = 50;

            sprintf(
                msg,
                "A1206: SPEED=50%% | TRAFFIC\n"
            );
        }


        /* ----------------------------------------------------
         * > 40 cm
         *
         * 100% PWM
         * ---------------------------------------------------- */

        else
        {
            duty = 100;

            sprintf(
                msg,
                "A1206: SPEED=100%% | CLEAR\n"
            );
        }


        /* ====================================================
         * PWM CONTROL
         *
         * 20 PWM periods
         * ==================================================== */

        for (int i = 0; i < 20; i++)
        {
            motor_pwm(
                gpio,
                duty
            );
        }


        /* ====================================================
         * SERIAL OUTPUT
         * ==================================================== */

        write(
            fd,
            msg,
            strlen(msg)
        );


        /* ====================================================
         * CONSOLE OUTPUT
         * ==================================================== */

        printf(
            "Dist: %.2f cm | %s",
            distance,
            msg
        );
    }


    close(fd);

    return EXIT_SUCCESS;
}
