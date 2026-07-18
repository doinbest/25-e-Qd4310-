#include "drug.h"
#include "control.h"

uint8_t drug_state = 0;


#define DRUG_PLACE_TIME_MS 250
#define DRUG_REMOVE_TIME_MS 250

/*
 * 药品等待状态号
 * 注意：这些状态号不要和 control.c 里的路线 case 冲突
 */
#define TASK_WAIT_NEAR_REMOVE 80
#define TASK_WAIT_MIDDLE_REMOVE 81
#define TASK_WAIT_FAR_REMOVE 82

uint8_t Drug_IsPresent(void)
{
    drug_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15);

    if (drug_state == GPIO_PIN_SET)
    {
        return 1; // 有药品，遮挡住，高电平
    }
    else
    {
        return 0; // 无药品，低电平
    }
}

/*
 * 等待放药：连续检测到药品 500ms，返回 1
 */
uint8_t Drug_WaitPlace(void)
{
    static uint32_t start_tick = 0;
    static uint8_t timing = 0;

    if (Drug_IsPresent())
    {
        if (timing == 0)
        {
            timing = 1;
            start_tick = HAL_GetTick();
        }

        if (HAL_GetTick() - start_tick >= DRUG_PLACE_TIME_MS)
        {
            timing = 0;
            start_tick = 0;
            return 1;
        }
    }
    else
    {
        timing = 0;
        start_tick = 0;
    }

    return 0;
}

/*
 * 等待取药：连续 1000ms 没有检测到药品，返回 1
 */
uint8_t Drug_WaitRemove(void)
{
    static uint32_t start_tick = 0;
    static uint8_t timing = 0;
    static uint8_t drug_seen = 0;

    /*
     * 到达病房等待阶段，先确认药品确实还在车上。
     * 只有见过一次有药品，才允许后面判断“药品被取走”。
     */
    if (Drug_IsPresent())
    {
        drug_seen = 1;

        timing = 0;
        start_tick = 0;

        return 0;
    }

    /*
     * 如果到达病房后从来没有检测到药品，
     * 不允许直接认为药品已经被取走。
     */
    if (drug_seen == 0)
    {
        timing = 0;
        start_tick = 0;

        return 0;
    }

    /*
     * 已经确认过药品在车上，现在连续无药 1000ms，
     * 才认为人工取药完成。
     */
    if (Drug_IsPresent() == 0)
    {
        if (timing == 0)
        {
            timing = 1;
            start_tick = HAL_GetTick();
        }

        if (HAL_GetTick() - start_tick >= DRUG_REMOVE_TIME_MS)
        {
            timing = 0;
            start_tick = 0;
            drug_seen = 0;

            return 1;
        }
    }
    else
    {
        timing = 0;
        start_tick = 0;
    }

    return 0;
}
/*
 * 药品检测主状态机
 */
void drug_scan(void)
{
    static uint8_t mission_started = 0;

    /*
     * 先读一次药品状态，方便 OLED 显示 drug_state
     */
    Drug_IsPresent();

    /*
     * 目标数字还没有识别出来，不允许发车
     * 防止药品先放上后，car_go 直接变 1，导致 MaixCAM 不再识别数字
     */
    // if (target_number == 0)
    // {
    //     car_go = 0;
    //     return;
    // }

    /*
     * 1. 起点等待放药
     */
    if (mission_started == 0)
    {
        car_go = 0;

        if (task_state == 0)
        {
            if (Drug_WaitPlace())
            {
                mission_started = 1;

                /*
                 * 药品确认放上后，正式发车前清零
                 */
                Encoder_ResetDistance();
                JY61p_HardwareZeroYaw();

                car_go = 1;
            }
        }

        return;
    }

    /*
     * 2. 到达病房后等待取药
     */
    if (task_state == TASK_WAIT_NEAR_REMOVE ||
        task_state == TASK_WAIT_MIDDLE_REMOVE ||
        task_state == TASK_WAIT_FAR_REMOVE)
    {
        car_go = 0;

        Location_flag = 0;
        Angle_flag = 0;
        Gray_flag = 0;

        Motor_SetSpeed(MOTOR_LEFT, 0);
        Motor_SetSpeed(MOTOR_RIGHT, 0);

        if (Drug_WaitRemove())
        {
            car_go = 1;

            if (task_state == TASK_WAIT_NEAR_REMOVE)
            {
                task_state = 3; // 近端：开始倒退出病房
            }
            else if (task_state == TASK_WAIT_MIDDLE_REMOVE)
            {
                task_state = 5; // 中端：开始倒退出病房
            }
            else if (task_state == TASK_WAIT_FAR_REMOVE)
            {
                task_state = 22; // 远端：开始倒退出病房
            }
        }

        return;
    }
}