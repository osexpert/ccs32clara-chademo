#include <stdint.h>
#include "chademoCharger.h"
#include <libopencm3/stm32/can.h>
#include "params.h"
#include "digio.h"
#include "hwinit.h"



//
//#include <stdint.h>
//#include "hwdefs.h"
//#include "my_math.h"
//#include "printf.h"
//#include <libopencm3/stm32/can.h>
//#include <libopencm3/stm32/gpio.h>
//#include <libopencm3/stm32/rcc.h>
//#include <libopencm3/stm32/rtc.h>
//#include <libopencm3/cm3/common.h>
//#include <libopencm3/cm3/nvic.h>
//#include "stm32_can.h"
//
//#define MAX_INTERFACES        2
//#define IDS_PER_BANK          4
//#define EXT_IDS_PER_BANK      2
//
//#ifndef CAN_PERIPH_SPEED
//#define CAN_PERIPH_SPEED 36
//#endif // CAN_PERIPH_SPEED
//


#ifndef SENDBUFFER_LEN
#define SENDBUFFER_LEN 20
#endif // SENDBUFFER_LEN

//class Stm32Can : public CanHardware
//{
//public:
//    Stm32Can(uint32_t baseAddr, enum baudrates baudrate, bool remap = false);
//    void SetBaudrate(enum baudrates baudrate);
//    void Send(uint32_t canId, uint32_t data[2], uint8_t len);
//    void HandleTx();
//    void HandleMessage(int fifo);
//    static Stm32Can* GetInterface(int index);
//
//private:
struct SENDBUFFER
{
    uint32_t id;
    uint32_t len;
    uint32_t data[2];
};

SENDBUFFER sendBuffer[SENDBUFFER_LEN];
int sendCnt;



/** \brief Send a user defined CAN message
 *
 * \param canId uint32_t
 * \param data[2] uint32_t
 * \param len message length
 * \return void
 *
 */
void Stm32CanSend(uint32_t canId, uint32_t data[2], uint8_t len)
{
    can_disable_irq(CAN1, CAN_IER_TMEIE);

    if (can_transmit(CAN1, canId, canId > 0x7FF, false, len, (uint8_t*)data) < 0 && sendCnt < SENDBUFFER_LEN)
    {
        // enqueue in send buffer if all TX mailboxes are full
        sendBuffer[sendCnt].id = canId;
        sendBuffer[sendCnt].len = len;
        sendBuffer[sendCnt].data[0] = data[0];
        sendBuffer[sendCnt].data[1] = data[1];
        sendCnt++;
    }

    if (sendCnt > 0)
    {
        can_enable_irq(CAN1, CAN_IER_TMEIE);
    }
}

extern ChademoCharger* chademoCharger;

void Stm32CanHandleMessage(int fifo)
{
    uint32_t id;
    bool ext, rtr;
    uint8_t length, fmi;
    uint32_t data[2];

    while (can_receive(CAN1, fifo, true, &id, &ext, &rtr, &fmi, &length, (uint8_t*)data, 0) > 0)
    {
        chademoCharger->HandleChademoMessage(id, (uint8_t*)data, length);
        //lastRxTimestamp = time_value;
    }
}

void Stm32CanHandleTx()
{
    SENDBUFFER* b = sendBuffer; //alias

    while (sendCnt > 0 && can_transmit(CAN1, b[sendCnt - 1].id, b[sendCnt - 1].id > 0x7FF, false, b[sendCnt - 1].len, (uint8_t*)b[sendCnt - 1].data) >= 0)
        sendCnt--;

    if (sendCnt == 0)
    {
        can_disable_irq(CAN1, CAN_IER_TMEIE);
    }
}




/* Interrupt service routines */
extern "C" void can1_rx0_isr(void)
{
    Stm32CanHandleMessage(0);
}

extern "C" void can1_rx1_isr()
{
    Stm32CanHandleMessage(1);
}

extern "C" void can1_tx_isr()
{
    Stm32CanHandleTx();
}





void ChademoCharger::CanSend(int id, int len, uint8_t* data)
{
    Stm32CanSend(id, (uint32_t*)data, len);
};




void ChademoCharger::SetSwitchD2(bool set)
{
    if (set)
        DigIo::switch_d2_out_inverted.Clear(); // 0: on
    else
        DigIo::switch_d2_out_inverted.Set(); // 1: off
};

void ChademoCharger::SetSwitchD1(bool set)
{
    if (set)
        DigIo::switch_d1_out.Set();
    else
        DigIo::switch_d1_out.Clear();
};

bool ChademoCharger::IsChargerLive()
{
    return ccsPowerRelayOnTrigger;
};

bool ChademoCharger::GetSwitchK()
{
    return DigIo::switch_k_in_inverted.Get() == false; // inverted...0 = on
};

bool ChademoCharger::IsChargingStoppedByAdapter()
{
    return stopButtonTrigger;
};

void ChademoCharger::NotifyCarContactorsClosed()
{
    printf("cha: Adapter contactor closed\r\n");

    // close our contactor too. it is not opened until power off
    DigIo::contactor_out.Set();
    // DigIo::external_led_out.Clear(); led on. pointless?
};

