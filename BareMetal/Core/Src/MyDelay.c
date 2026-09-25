#include "MyDelay.h"

/**
 * @brief  ���뼶��ʱ����
 * @param  ms: Ҫ��ʱ�ĺ�����
 */
void MyDelay_ms(uint32_t ms)
{
    HAL_Delay(ms); // ֱ�ӵ��� STM32 HAL �⼫���ȶ��ĺ�����ʱ
}

/**
 * @brief  Microsecond delay function
 * @param  us: Delay time in microseconds
 */
void MyDelay_us(uint32_t us)
{
    uint32_t delay = (SystemCoreClock / 4000000) * us;
    while (delay--)
    {
        __NOP();
    }
}
