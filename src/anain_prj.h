//#ifndef ANAIN_PRJ_H_INCLUDED
//#define ANAIN_PRJ_H_INCLUDED
//
//
///* Here we specify how many samples are combined into one filtered result. Following values are possible:
//*  - NUM_SAMPLES = 1: Most recent raw value is returned
//*  - NUM_SAMPLES = 3: Median of last 3 values is returned
//*  - NUM_SAMPLES = 9: Median of last 3 medians is returned
//*  - NUM_SAMPLES = 12: Average of last 4 medians is returned
//*/
//#define NUM_SAMPLES 12
//
////#define SAMPLE_TIME ADC_SMPR_SMP_7DOT5CYC //Sample&Hold time for each pin. Increases sample time, might increase accuracy
//#define SAMPLE_TIME ADC_SMPR_SMP_15CYC //Sample&Hold time for each pin. Increases sample time, might increase accuracy
//
////Here you specify a list of analog inputs, see main.cpp on how to use them
//#define ANA_IN_LIST \
//   ANA_IN_ENTRY(pp,    GPIOA, 0) \
//   ANA_IN_ENTRY(tempi, GPIOB, 2) /*internal temp sensor*/ \
//   ANA_IN_ENTRY(temp1, GPIOC, 0) \
//   ANA_IN_ENTRY(temp2, GPIOC, 1) \
//   ANA_IN_ENTRY(temp3, GPIOC, 2) \
//   ANA_IN_ENTRY(udc,   GPIOC, 3) \
//   ANA_IN_ENTRY(button,GPIOC, 4) \
//   ANA_IN_ENTRY(lockfb,GPIOC, 5) \
//   ANA_IN_ENTRY(ipropi,GPIOA, 3) \
//
//#endif // ANAIN_PRJ_H_INCLUDED
