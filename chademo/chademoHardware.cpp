#include <stdint.h>
#include "chademoCharger.h"
#include <libopencm3/stm32/can.h>
#include "params.h"
#include "digio.h"
#include "hwinit.h"
#include "main.h"

extern volatile uint32_t system_millis;

extern global_data _global;

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
    // TODO: OR....could check that voltage delivered > 1 ? (precharge started)
    return _global.ccsPowerRelayOnTrigger_prechargeDone;
};

bool ChademoCharger::GetSwitchK()
{
    return DigIo::switch_k_in_inverted.Get() == false; // inverted...0 = on
};

bool ChademoCharger::IsChargingStoppedByAdapter()
{
    return _global.stopButtonEvent;
};

void ChademoCharger::NotifyCarContactorsClosed()
{
    printf("[cha] Adapter contactor closed\r\n");

    // close our contactor too. it is not opened until power off
    DigIo::contactor_out.Set();
    // DigIo::external_led_out.Clear(); led on. pointless?
};

extern ChademoCharger* chademoCharger;

/* Interrupt service routines */
extern "C" void can1_rx0_isr(void)
{
    uint32_t id;
    bool ext, rtr;
    uint8_t length, fmi;
    uint32_t data[2];

    while (can_receive(CAN1,
        0, // fifo
        true,  // release
        &id,
        &ext,
        &rtr,
        &fmi, // ID of the matched filter
        &length,
        (uint8_t*)data,
        0 // timestamp
    ) > 0 && length == 8)
    {
        chademoCharger->HandleCanMessage(id, data);
        //lastRxTimestamp = time_value;
    }
}

//#ifndef SENDBUFFER_LEN
//#define SENDBUFFER_LEN 16  // must be power of 2!
//#endif
//
//#define SENDBUFFER_MASK (SENDBUFFER_LEN - 1)
//
//typedef struct {
//    uint32_t id;
//    uint8_t len;
//    uint32_t data[2];
//} SENDBUFFER;
//
//static volatile uint8_t txHead = 0;
//static volatile uint8_t txTail = 0;
//static SENDBUFFER sendBuffer[SENDBUFFER_LEN];


/** \brief Send a user defined CAN message
 *
 * \param canId uint32_t
 * \param data[2] uint32_t
 * \param len message length
 * \return void
 *
 */
//void Stm32CanSend(uint32_t canId, uint32_t data[2], uint8_t len)
//{
//    can_disable_irq(CAN1, CAN_IER_TMEIE);
//
//    // Try to send directly (but only if q is empty, want fifo)
//    if (txHead == txTail && can_transmit(CAN1, canId, canId > 0x7FF, false, len, (uint8_t*)data) >= 0) {
//        // Sent immediately, and ring buffer is empty, no need to enable IRQ
//
//        if (canId == 0x108)
//        {
//            _global.cha108++;
//            _global.cha108dur = system_millis - _global.cha108last;
//            _global.cha108last = system_millis;
//        }
//        else if (canId == 0x109)
//        {
//            _global.cha109++;
//            _global.cha109dur = system_millis - _global.cha109last;
//            _global.cha109last = system_millis;
//        }
//
//        return;
//    }
//
//    // Queue if failed or buffer already has messages
//    uint8_t nextHead = (txHead + 1) & SENDBUFFER_MASK;
//    if (nextHead == txTail) {
//        // car stop reading messages if we get stuck during rundown...
//        //printf("tx Buffer full...\r\n");
//        return;
//    }
//
//    sendBuffer[txHead].id = canId;
//    sendBuffer[txHead].len = len;
//    sendBuffer[txHead].data[0] = data[0];
//    sendBuffer[txHead].data[1] = data[1];
//    txHead = nextHead;
//
//    can_enable_irq(CAN1, CAN_IER_TMEIE); // Enable interrupt only if queue has data
//}
//
//void ChademoCharger::CanSend(int id, int len, uint8_t* data)
//{
//    Stm32CanSend(id, (uint32_t*)data, len);
//};
//



//void Stm32CanHandleMessage(int fifo)
//{
//    uint32_t id;
//    bool ext, rtr;
//    uint8_t length, fmi;
//    uint32_t data[2];
//
//    while (can_receive(CAN1, fifo, true, &id, &ext, &rtr, &fmi, &length, (uint8_t*)data, 0) > 0)
//    {
//        chademoCharger->HandleChademoMessage(id, (uint8_t*)data, length);
//        //lastRxTimestamp = time_value;
//    }
//}

//void Stm32CanHandleTx()
//{
//    SENDBUFFER* b = sendBuffer; //alias
//
//    while (sendCnt > 0 && can_transmit(CAN1, b[sendCnt - 1].id, b[sendCnt - 1].id > 0x7FF, false, b[sendCnt - 1].len, (uint8_t*)b[sendCnt - 1].data) >= 0)
//        sendCnt--;
//
//    if (sendCnt == 0)
//    {
//        can_disable_irq(CAN1, CAN_IER_TMEIE);
//    }
//}
//void Stm32CanHandleTx()
//{
//    SENDBUFFER* b = sendBuffer; //alias
//
//    while (txTail != txHead)
//    {
//        SENDBUFFER* msg = &b[txTail];
//        if (can_transmit(CAN1, msg->id, msg->id > 0x7FF, false, msg->len, (uint8_t*)msg->data) < 0)
//            break;  // Hardware TX mailbox full, exit and wait for next IRQ
//
//        if (msg->id == 0x108)
//        {
//            _global.cha108++;
//            _global.cha108dur = system_millis - _global.cha108last;
//            _global.cha108last = system_millis;
//        }
//        else if (msg->id == 0x109)
//        {
//            _global.cha109++;
//            _global.cha109dur = system_millis - _global.cha109last;
//            _global.cha109last = system_millis;
//        }
//
//        txTail = (txTail + 1) & SENDBUFFER_MASK;
//    }
//
//    if (txTail == txHead)
//    {
//        // Buffer empty, disable TX empty interrupt
//        can_disable_irq(CAN1, CAN_IER_TMEIE);
//    }
//}


