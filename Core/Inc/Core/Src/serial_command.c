
#include "serial_command.h"
#include "motor_control.h"
#include <stdio.h>

#define PID_KP     16
#define PID_KI     8
#define PID_KD     0
#define MAX_SPEED  6000

static uint32_t input_num = 0;
static uint8_t input_sign = 0;
static uint8_t char_index = 0;


void serial_command(UART_HandleTypeDef *huart, uint8_t *direction, uint16_t *speed)
{
    uint8_t b = 0;
    if (HAL_UART_Receive(huart, &b, 1, 0) == HAL_OK)
    {
        if ('0' <= b && b <= '9')
        {
            input_num = input_num * 10 + (b - '0');
            HAL_UART_Transmit(huart, &b, 1, 0);
            char_index++;
        }
        else if (b == '\n')
        {
            if (input_num == 0)
            {
                *speed = 0;
                init_pwm();
                printf("\nstop motor\n");
            }
            else {
                if (*speed == 0 || (input_sign == 1 && *direction == 0) || (input_sign == 0 && *direction == 1))
                {
                    *direction = input_sign;
                    PID_init(PID_KP, PID_KI, PID_KD, *direction, 40);
                }
                *speed = input_num;
                *direction = input_sign;
                if (*speed > MAX_SPEED)
                {
                    *speed = MAX_SPEED;
                }
                if (input_sign == 0)
                {
                    printf("\nset speed to %d\n", *speed);
                }
                else
                {
                    printf("\nset speed to %d\n", -(*speed));
                }
            }
            input_num = 0;
            input_sign = 0;
            char_index = 0;
        }
        else if (b == 27) // ESC
        {
            input_num = 0;
            uint8_t const clear_line[6] = "\r    ";
            HAL_UART_Transmit(huart, clear_line, 1, 10);
            // send spaces by group of 4
            for (uint8_t i = 0; i < ((char_index >> 2) + 1); i++)
            {
                HAL_UART_Transmit(huart, clear_line+1, 4, 10);
            }
            HAL_UART_Transmit(huart, clear_line, 1, 10);
            input_sign = 0;
            char_index = 0;
        }
        else if (b == 8 && char_index) // BACKSPACE
        {
            input_num /= 10;
            uint8_t const clear_char[4] = "\b \b";
            HAL_UART_Transmit(huart, clear_char, 3, 10);
            char_index--;
            if (char_index == 0)
            {
                input_sign = 0;
            }
        }
        else if (b == '-' && char_index == 0)
        {
            input_sign = 1;
            HAL_UART_Transmit(huart, &b, 1, 0);
            char_index++;
        }
    }
}
