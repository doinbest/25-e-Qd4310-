#include "No_Mcu_Ganv_Grayscale_Sensor_Config.h"

void Get_Analog_value(unsigned short *result)
{
    unsigned char i;
    unsigned char j;
    unsigned int analog_sum = 0;

    for (i = 0; i < 8; i++)
    {
        Switch_Address_0(!(i & 0x01));
        Switch_Address_1(!(i & 0x02));
        Switch_Address_2(!(i & 0x04));

        for (j = 0; j < 8; j++)
        {
            analog_sum += Get_adc_of_user();
        }

        if (!Direction)
        {
            result[i] = analog_sum / 8U;
        }
        else
        {
            result[7U - i] = analog_sum / 8U;
        }
        analog_sum = 0;
    }
}

static unsigned short Clamp_To_Bits(long value, unsigned short bits)
{
    if (value <= 0)
    {
        return 0;
    }
    if (value >= (long)bits)
    {
        return bits;
    }
    return (unsigned short)value;
}

static void Update_Line_Digital(No_MCU_Sensor *sensor)
{
    uint8_t digital = 0;
    uint8_t i;

    for (i = 0; i < 8; i++)
    {
        if (sensor->Line_is_low[i] != 0)
        {
            if (sensor->Analog_value[i] <= sensor->Line_threshold[i])
            {
                digital |= (uint8_t)(1U << i);
            }
        }
        else
        {
            if (sensor->Analog_value[i] >= sensor->Line_threshold[i])
            {
                digital |= (uint8_t)(1U << i);
            }
        }
    }

    sensor->Digtal = digital;
}

static void Update_Line_Normal(No_MCU_Sensor *sensor)
{
    uint8_t i;
    unsigned short bits = (unsigned short)sensor->bits;

    for (i = 0; i < 8; i++)
    {
        long value;

        if (sensor->Normal_factor[i] <= 0.0)
        {
            sensor->Normal_value[i] = 0;
            continue;
        }

        if (sensor->Line_is_low[i] != 0)
        {
            value = (long)(((double)sensor->Analog_value[i] -
                            (double)sensor->Calibrated_line[i]) *
                           sensor->Normal_factor[i]);
        }
        else
        {
            value = (long)(((double)sensor->Calibrated_line[i] -
                            (double)sensor->Analog_value[i]) *
                           sensor->Normal_factor[i]);
        }

        sensor->Normal_value[i] = Clamp_To_Bits(value, bits);
    }
}

void No_MCU_Ganv_Sensor_Init_Frist(No_MCU_Sensor *sensor)
{
    memset(sensor->Analog_value, 0, sizeof(sensor->Analog_value));
    memset(sensor->Normal_value, 0, sizeof(sensor->Normal_value));
    memset(sensor->Calibrated_white, 0, sizeof(sensor->Calibrated_white));
    memset(sensor->Calibrated_line, 0, sizeof(sensor->Calibrated_line));
    memset(sensor->Line_threshold, 0, sizeof(sensor->Line_threshold));
    memset(sensor->Line_is_low, 0, sizeof(sensor->Line_is_low));

    for (int i = 0; i < 8; i++)
    {
        sensor->Normal_factor[i] = 0.0;
    }

    sensor->bits = 0.0;
    sensor->Digtal = 0;
    sensor->Time_out = 0;
    sensor->Tick = 0;
    sensor->ok = 0;
}

void No_MCU_Ganv_Sensor_Init(No_MCU_Sensor *sensor,
                             unsigned short *Calibrated_white,
                             unsigned short *Calibrated_line)
{
    No_MCU_Ganv_Sensor_Init_Frist(sensor);

    if (Sensor_ADCbits == _8Bits)
    {
        sensor->bits = 255.0;
    }
    else if (Sensor_ADCbits == _10Bits)
    {
        sensor->bits = 1024.0;
    }
    else if (Sensor_ADCbits == _12Bits)
    {
        sensor->bits = 4096.0;
    }
    else if (Sensor_ADCbits == _14Bits)
    {
        sensor->bits = 16384.0;
    }

    if (Sensor_Edition == Class)
    {
        sensor->Time_out = 1;
    }
    else
    {
        sensor->Time_out = 10;
    }

    for (int i = 0; i < 8; i++)
    {
        double diff;

        sensor->Calibrated_white[i] = Calibrated_white[i];
        sensor->Calibrated_line[i] = Calibrated_line[i];
        sensor->Line_threshold[i] = (unsigned short)(((uint32_t)Calibrated_white[i] +
                                                      (uint32_t)Calibrated_line[i]) /
                                                     2U);
        sensor->Line_is_low[i] = (Calibrated_line[i] < Calibrated_white[i]) ? 1U : 0U;

        if (Calibrated_white[i] == Calibrated_line[i])
        {
            sensor->Normal_factor[i] = 0.0;
            continue;
        }

        diff = (Calibrated_white[i] > Calibrated_line[i]) ?
                   ((double)Calibrated_white[i] - (double)Calibrated_line[i]) :
                   ((double)Calibrated_line[i] - (double)Calibrated_white[i]);
        sensor->Normal_factor[i] = sensor->bits / diff;
    }

    sensor->ok = 1;
}

void No_Mcu_Ganv_Sensor_Task_Without_tick(No_MCU_Sensor *sensor)
{
    Get_Analog_value(sensor->Analog_value);
    Update_Line_Digital(sensor);
    Update_Line_Normal(sensor);
}

void No_Mcu_Ganv_Sensor_Task_With_tick(No_MCU_Sensor *sensor)
{
    if (sensor->Tick >= sensor->Time_out)
    {
        Get_Analog_value(sensor->Analog_value);
        Update_Line_Digital(sensor);
        Update_Line_Normal(sensor);
        sensor->Tick = 0;
    }
}

void Task_tick(No_MCU_Sensor *sensor)
{
    sensor->Tick++;
}

unsigned char Get_Digtal_For_User(No_MCU_Sensor *sensor)
{
    return sensor->Digtal;
}

unsigned char Get_Normalize_For_User(No_MCU_Sensor *sensor, unsigned short *result)
{
    if (!sensor->ok)
    {
        return 0;
    }

    memcpy(result, sensor->Normal_value, sizeof(sensor->Normal_value));
    return 1;
}

unsigned char Get_Anolog_Value(No_MCU_Sensor *sensor, unsigned short *result)
{
    memcpy(result, sensor->Analog_value, sizeof(sensor->Analog_value));
    return 1;
}
