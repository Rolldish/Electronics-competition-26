#include "ADC.h"
volatile bool gCheckADC;        //ADC采集成功标志位
//读取ADC的数据
unsigned int adc_getValue(void)
{
        unsigned int gAdcResult = 0;

        //软件触发ADC开始转换
        DL_ADC12_startConversion(ADC12_0_INST);
        //如果当前状态为正在转换中则等待转换结束
        while (false == gCheckADC) {
            __WFE();
        }
        //获取数据
        gAdcResult = DL_ADC12_getMemResult(ADC12_0_INST, ADC12_0_ADCMEM_ADC_CH0);

        //清除标志位
        gCheckADC = false;

        return gAdcResult;
}
//ADC中断服务函数
void ADC12_0_INST_IRQHandler(void)
{
        //查询并清除ADC中断
        switch (DL_ADC12_getPendingInterrupt(ADC12_0_INST))
        {
              //检查是否完成数据采集
              case DL_ADC12_IIDX_MEM0_RESULT_LOADED:
                        gCheckADC = true;//将标志位置1
                        break;
              default:
                        break;
        }
}