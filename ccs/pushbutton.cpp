//
///* Pushbutton evaluation */
//
//#include "ccs32_globals.h"
//
///* public interface */
//static uint16_t pushbutton_tButtonReleaseTime;
//static uint16_t pushbutton_tButtonPressTime;
//
//static uint8_t pushbutton_nNumberOfButtonPresses;
//static uint8_t pushbutton_buttonSeriesCounter;
//static uint16_t pushbutton_accumulatedButtonDigits;
//
///* local defines and variables */
//#define PUSHBUTTON_NUMBER_OF_ENTRIES 4
//static uint8_t blButtonPressedOld;
//static uint8_t buttonSeriesEntries[PUSHBUTTON_NUMBER_OF_ENTRIES];
//
///* functions */
//
//static void pushbutton_processPushButtonSeries(void) {
//  uint8_t i;
//  uint16_t tmp16;
//  for (i=0; i<PUSHBUTTON_NUMBER_OF_ENTRIES-1; i++) {
//	  buttonSeriesEntries[i]=buttonSeriesEntries[i+1];
//  }
//  buttonSeriesEntries[PUSHBUTTON_NUMBER_OF_ENTRIES-1]=pushbutton_nNumberOfButtonPresses;
//  tmp16 = buttonSeriesEntries[0];
//  tmp16*=10;
//  tmp16 += buttonSeriesEntries[1];
//  tmp16*=10;
//  tmp16 += buttonSeriesEntries[2];
//  tmp16*=10;
//  tmp16 += buttonSeriesEntries[3];
//  pushbutton_buttonSeriesCounter++;
//  if (pushbutton_buttonSeriesCounter==PUSHBUTTON_NUMBER_OF_ENTRIES) {
//	  pushbutton_accumulatedButtonDigits = tmp16;
//  }
//}
//
//bool pushbutton_isPressed500ms()
//{
//    if(Param::GetInt(Param::AllowUnlock) == 1) //check unlocking is allowed and thus vehicle is not in lockstate, read button
//    {
//        return pushbutton_tButtonPressTime>(PUSHBUTTON_CYCLES_PER_SECOND/2);
//    }
//    else
//    {
//        return 0;
//    }
//}
//
//int pushButton_getAccumulatedDigits()
//{
//   return pushbutton_accumulatedButtonDigits;
//}
//
//void pushbutton_handlePushbutton(void) {
//	uint8_t blButtonPressed;
//	uint8_t i;
//	//debugvalue = AnaIn::button.Get();
//	blButtonPressed = AnaIn::button.Get()<1000;
//	if (blButtonPressed) {
//		if (!blButtonPressedOld) {
//			pushbutton_tButtonReleaseTime=0;
//			pushbutton_nNumberOfButtonPresses++;
//			pushbutton_tButtonPressTime=0;
//		}
//		if (pushbutton_tButtonPressTime<PUSHBUTTON_CYCLES_PER_SECOND*60) pushbutton_tButtonPressTime++;
//	} else {
//        /* button is no pressed */
//		if (pushbutton_tButtonReleaseTime<PUSHBUTTON_CYCLES_PER_SECOND*60) pushbutton_tButtonReleaseTime++;
//		if (pushbutton_tButtonReleaseTime==PUSHBUTTON_CYCLES_PER_SECOND/2) {
//			/* not pressed for 0.5s -> evaluate the series */
//			//debugvalue=pushbutton_nNumberOfButtonPresses;
//			pushbutton_processPushButtonSeries();
//			pushbutton_nNumberOfButtonPresses=0;
//		}
//		if (pushbutton_tButtonReleaseTime==PUSHBUTTON_CYCLES_PER_SECOND*5) {
//			/* 5s not pressed -> reset the series */
//			for (i=0; i<PUSHBUTTON_NUMBER_OF_ENTRIES; i++) {
//				buttonSeriesEntries[i]=0;
//				pushbutton_buttonSeriesCounter=0;
//				pushbutton_accumulatedButtonDigits = 0;
//				pushbutton_tButtonPressTime=0;
//			}
//		}
//	}
//	blButtonPressedOld = blButtonPressed;
//}
