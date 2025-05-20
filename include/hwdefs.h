#ifndef HWDEFS_H_INCLUDED
#define HWDEFS_H_INCLUDED


//Common for any config

//#define RCC_CLOCK_SETUP()   rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ])

//#define CONTACT_LOCK_TIMER  TIM3
//#define CONTACT_LOCK_PERIOD 4096

//#define LOCK1_CHAN          TIM_OC1
//#define LOCK2_CHAN          TIM_OC2
//#define CONTACT1_CHAN       TIM_OC3
//#define CONTACT2_CHAN       TIM_OC4

//#define CP_TIMER            TIM2

//Address of parameter block in flash

#define FLASH_PAGE_SIZE 1024
#define PARAM_BLKSIZE FLASH_PAGE_SIZE
#define PARAM_BLKNUM  1   //last block of 1k
#define CAN1_BLKNUM   2
#define CAN2_BLKNUM   4


#endif // HWDEFS_H_INCLUDED
