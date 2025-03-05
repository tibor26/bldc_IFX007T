

#include "motor_control.h"


void update_pwm(uint8_t step, uint8_t duty_cycle);
uint8_t get_hall(void);

#define COMMUTATION_TIMEOUT  500
#define PID_SAMPLE_TIME      100
#define F_PI_CONTROLLER      1
#define PID_POWER_MAX        100
#define PID_POWER_MIN        20

static uint8_t hall_old = 0;
static uint8_t motor_run = 0;
static uint32_t timeout = 0;
static uint32_t PID_next_update = 0;
static int32_t PID_Proportional = 0;
static int32_t PID_Integral = 0;
static int16_t last_speed;
static int32_t PID_Kp = 0;
static int32_t PID_Ki = 0;
static int32_t PID_Kd = 0;
static int32_t PID_out = 0;  // for debug
static uint32_t Hall_Pulses = 0;
static uint32_t CurrentSpeed = 0;

/**
* The first index switches between normal mode and fast mode (field weakening range)
* The second index switches the direction between forward and backward
* The third index for the pattern is the Hallsensor input as a dezimal value (e.g 0b00000101 is 5 dez)
* The value at the index poition delivers the commutation state for the next step.
* HallPattern[FieldWeakening][direction][hall_state]
*/
static const uint8_t HallPattern[2][2][7] = {
{
    { 9, 4, 0, 5, 2, 3, 1 },
    /* Normal foreward */
    { 9, 1, 3, 2, 5, 0, 4 }  /* Normal backward */
},
{
    { 9, 5, 1, 0, 3, 4, 2 },
    /* Fast foreward */
    { 9, 0, 2, 1, 4, 5, 3 }  /* Fast backward */
}
};

void commutation(uint8_t direction, uint8_t duty_cycle, uint8_t field_weakening)
{
    if (duty_cycle)
    {
        if (motor_run)
        {
            uint8_t hall_new = get_hall();
            if (hall_new != hall_old)
            {
                hall_old = hall_new;
                Hall_Pulses += 1;
                update_pwm(HallPattern[field_weakening][direction][hall_new], duty_cycle);
                timeout = HAL_GetTick() + COMMUTATION_TIMEOUT;
            }
            else if (HAL_GetTick() > timeout)
            {
                init_pwm();
            }
        }
        else
        {
            motor_run = 1;
            hall_old = get_hall();
            update_pwm(HallPattern[field_weakening][direction][hall_old], duty_cycle);
            timeout = HAL_GetTick() + COMMUTATION_TIMEOUT;
        }
    }
    else if (motor_run)
    {
        init_pwm();
    }

    return;
}


uint8_t get_hall(void)
{
    return (HAL_GPIO_ReadPin(HALL_U_GPIO_Port, HALL_U_Pin) << 2) | (HAL_GPIO_ReadPin(HALL_V_GPIO_Port, HALL_V_Pin) << 1) | HAL_GPIO_ReadPin(HALL_W_GPIO_Port, HALL_W_Pin);
}

void init_pwm(void)
{
    HAL_GPIO_WritePin(INHU_GPIO_Port, INHU_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(INHV_GPIO_Port, INHV_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(INHW_GPIO_Port, INHW_Pin, GPIO_PIN_RESET);
    TIM17->CCR1 = 0;
    TIM3->CCR3 = 0;
    TIM3->CCR2 = 0;
    motor_run = 0;
}

/**
 * Commutation table for brushless motors (the "6 step Process")
 * Inhibit Pin = High means active -> Inhibit Pin = Low means output is floating
*/
void update_pwm(uint8_t step, uint8_t duty_cycle)
{
    switch (step) {
    case 0:
        HAL_GPIO_WritePin(INHU_GPIO_Port, INHU_Pin, GPIO_PIN_SET); //PWM
        HAL_GPIO_WritePin(INHV_GPIO_Port, INHV_Pin, GPIO_PIN_SET); //GND
        HAL_GPIO_WritePin(INHW_GPIO_Port, INHW_Pin, GPIO_PIN_RESET); //Floating
        TIM17->CCR1 = duty_cycle;
        TIM3->CCR3 = 0;
        TIM3->CCR2 = 0;
        break;

    case 1:
        HAL_GPIO_WritePin(INHU_GPIO_Port, INHU_Pin, GPIO_PIN_SET); //PWM
        HAL_GPIO_WritePin(INHV_GPIO_Port, INHV_Pin, GPIO_PIN_RESET); //Floating
        HAL_GPIO_WritePin(INHW_GPIO_Port, INHW_Pin, GPIO_PIN_SET); //GND
        TIM17->CCR1 = duty_cycle;
        TIM3->CCR3 = 0;
        TIM3->CCR2 = 0;
        break;

    case 2:
        HAL_GPIO_WritePin(INHU_GPIO_Port, INHU_Pin, GPIO_PIN_RESET); //Floating
        HAL_GPIO_WritePin(INHV_GPIO_Port, INHV_Pin, GPIO_PIN_SET); //PWM
        HAL_GPIO_WritePin(INHW_GPIO_Port, INHW_Pin, GPIO_PIN_SET); //GND
        TIM17->CCR1 = 0;
        TIM3->CCR3 = duty_cycle;
        TIM3->CCR2 = 0;
        break;

    case 3:
        HAL_GPIO_WritePin(INHU_GPIO_Port, INHU_Pin, GPIO_PIN_SET); //GND
        HAL_GPIO_WritePin(INHV_GPIO_Port, INHV_Pin, GPIO_PIN_SET); //PWM
        HAL_GPIO_WritePin(INHW_GPIO_Port, INHW_Pin, GPIO_PIN_RESET); //Floating
        TIM17->CCR1 = 0;
        TIM3->CCR3 = duty_cycle;
        TIM3->CCR2 = 0;
        break;

    case 4:
        HAL_GPIO_WritePin(INHU_GPIO_Port, INHU_Pin, GPIO_PIN_SET); //GND
        HAL_GPIO_WritePin(INHV_GPIO_Port, INHV_Pin, GPIO_PIN_RESET); //Floating
        HAL_GPIO_WritePin(INHW_GPIO_Port, INHW_Pin, GPIO_PIN_SET); //PWM
        TIM17->CCR1 = 0;
        TIM3->CCR3 = 0;
        TIM3->CCR2 = duty_cycle;
        break;

    case 5:
        HAL_GPIO_WritePin(INHU_GPIO_Port, INHU_Pin, GPIO_PIN_RESET); //Floating
        HAL_GPIO_WritePin(INHV_GPIO_Port, INHV_Pin, GPIO_PIN_SET); //GND
        HAL_GPIO_WritePin(INHW_GPIO_Port, INHW_Pin, GPIO_PIN_SET); //PWM
        TIM17->CCR1 = 0;
        TIM3->CCR3 = 0;
        TIM3->CCR2 = duty_cycle;
        break;

    default:
        break;
    }
}

void PID_control(uint8_t direction, uint16_t speed)
{
    uint32_t now = HAL_GetTick();
    if (now >= PID_next_update)  // update every 100ms
    {
        PID_next_update = now + PID_SAMPLE_TIME;

        // RPM Calculation:
        // 1 turn = 6*3=18 hall transitions
        // RPM = (hall transitions per 100ms / 18) * 60000ms / 100ms
        //     = (hall transitions per 100ms) * 600 / 18
        CurrentSpeed = (Hall_Pulses * 600) / 18 ;
        Hall_Pulses = 0;

        int32_t error = speed - CurrentSpeed;

        // Proportional
        PID_Proportional = PID_Kp * error;
        int32_t output = PID_Proportional;

        // Integral
        PID_Integral += PID_Ki * error;
        if (PID_Integral > PID_POWER_MAX * 1000)
        {
            PID_Integral = PID_POWER_MAX * 1000;
        }
        else if (PID_Integral < PID_POWER_MIN * 1000)
        {
            PID_Integral = PID_POWER_MIN * 1000;
        }

        // Derivative
#if (F_PI_CONTROLLER != 1)
        int32_t dSpeed = (CurrentSpeed - LastSpeed);
        output -= PID_Kd * dSpeed;
        LastSpeed = CurrentSpeed;
#endif

        output = (output + PID_Integral) / 1000;

        // limit output
        if (output > PID_POWER_MAX)
        {
            output = PID_POWER_MAX;
        }
        else if (output < PID_POWER_MIN)
        {
            output = PID_POWER_MIN;
        }

        PID_out = output;
        //PID_out = speed;
    }
    commutation(direction, PID_out, 0);
}


void PID_init(int32_t kp, int32_t ki, int32_t kd, uint8_t direction, uint8_t output_init)
{
    CurrentSpeed = 0;
    last_speed = 0;
    Hall_Pulses = 0;
    if (output_init > PID_POWER_MAX)
    {
        output_init = PID_POWER_MAX;
    }
    else if (output_init < PID_POWER_MIN)
    {
        output_init = PID_POWER_MIN;
    }
    PID_Integral = output_init * 1000;
    PID_Kp = kp;
    PID_Ki = ki;
    PID_Kd = kd;
    PID_next_update = 0;
    commutation(direction, output_init, 0);
    PID_out = output_init;
}
