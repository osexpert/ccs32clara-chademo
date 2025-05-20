





#define MSG_CAR_100		0x100
#define MSG_CAR_101		0x101
#define MSG_CAR_102		0x102

// Send every 100ms +- 10%
#define MSG_CHARGER_108			0x108

// Send every 100ms +- 10%
// Message 109 is the "reply" to the 102 message from the car. So while all messages has this 100ms requirement, 109 may have stricter requirements...basically as fast as 102...
#define MSG_CHARGER_109			0x109


// 100 -> charger
// car <- 108
// 101 -> charger
// 102 -> charger
// car <- 109
// 102 -> charger
// car <- 109


// Message 0x102, byte[4]
#define CAR_FAULT_OVER_VOLT		1 //over voltage
#define CAR_FAULT_UNDER_VOLT	2 //Under voltage
#define CAR_FAULT_DEV_AMPS		4 //current mismatch
#define CAR_FAULT_OVER_TEMP		8 //over temperature
#define CAR_FAULT_DEV_VOLT		16 //voltage mismatch (how does this relate to over and under volt flags?)

// Message 0x102, byte[5]
#define CAR_STATUS_READY_TO_CHARGE	1 // charging enabled/allowed
#define CAR_STATUS_NOT_IN_PARK	2 // shifter not in safe state (0: park 1: other)
#define CAR_STATUS_ERROR	4 // car did something dumb (fault caused by the car or the charger and detected by the car)
#define CAR_STATUS_CONTACTOR_OPEN	8 // main contactor open (Special: 0: During contact sticking detection, 1: Contact sticking detection completed)
#define CAR_STATUS_STOP_CHARGING	16 // charger stop before even charging (or car request to stop charging) TODO: why do we have both CAR_STATUS_READY_TO_CHARGE and CAR_STATUS_STOP_CHARGING?
#define CAR_STATUS_V2X_COMPATIBLE   128 // car is V2X compatible (can deliver power to grid)

// Message 0x109, byte[5]
#define CHARGER_STATUS_CHARGING		1 // 0: standby 1: charging (power transfer from charger)
#define CHARGER_STATUS_ERROR 2 // something went wrong (fault caused by (or inside) the charger)
#define CHARGER_STATUS_PLUG_LOCKED	4 // connector is currently locked (electromagnetic lock, plug locked into the car)
#define CHARGER_STATUS_INCOMPAT	8 // parameters between vehicle and charger not compatible (battery incompatible?)
#define CHARGER_STATUS_CAR_ERROR		16 // problem with the car, such as improper connection (or something wrong with the battery?)
#define CHARGER_STATUS_CHARGING_STOPPED		32 //charger is stopped (charger shutdown or end of charging) TODO: is this also ititially set to stop, before charging??


typedef enum {
    STATE_IDLE,
    STATE_HANDSHAKE,
    STATE_READY,
    STATE_CHARGING,
    STATE_STOP,
    STATE_FAULT
} ChargerState;

ChargerState currentState = STATE_IDLE;

// Input & output states
bool carContactorControl = false;
bool twelveVoltOutToCar = false;

// EV requested parameters
uint16_t requested_voltage = 0;
uint16_t requested_current = 0;

// Charger capability limits
#define MAX_VOLTAGE     500
#define MAX_CURRENT     125

// Timeout timers
uint32_t handshake_timer = 0;
uint32_t charging_timer = 0;

// Constants
#define HANDSHAKE_TIMEOUT_MS 5000
#define CHARGING_TIMEOUT_MS  60000


void update_state_machine(uint32_t delta_ms) 
{
    bool car_ready_signal = read_gpio_input(CAR_READY_PIN); // e.g., 12V present

    switch (currentState)
    {
        case STATE_IDLE:
            twelveVoltOutToCar = false;
            carContactorControl = false;
            break;

        case STATE_HANDSHAKE:
            twelveVoltOutToCar = true;
            handshake_timer += delta_ms;
            if (handshake_timer > HANDSHAKE_TIMEOUT_MS) {
                currentState = STATE_FAULT;
            }
            break;

        case STATE_READY:
            if (!car_ready_signal) {
                currentState = STATE_FAULT;
            }
            break;

        case STATE_CHARGING:
            carContactorControl = true;
            charging_timer += delta_ms;
            if (!car_ready_signal || charging_timer > CHARGING_TIMEOUT_MS) {
                currentState = STATE_STOP;
            }
            break;

        case STATE_STOP:
            carContactorControl = false;
            currentState = STATE_IDLE;
            break;

        case STATE_FAULT:
            twelveVoltOutToCar = false;
            carContactorControl = false;
            break;
    }

    // Apply outputs
    write_gpio_output(TWELVE_V_TO_CAR_PIN, twelveVoltOutToCar);
    write_gpio_output(CAR_CONTACTOR_CONTROL_PIN, carContactorControl);
}

void handle_message_100(can_message_t msg)
{
    // MinimumChargeCurrent (byte 0, 8 bits, little endian)
    // eg. Don't bother starting or continuing charging if you can't provide at least this much current
    // OR: it means the EV wants at least #.0A current to begin or maintain charging.
    // Hmm....but 102 already have askingAmps....
    uint8_t MinimumChargeCurrent = msg.data[0];  // 1A per unit
    // Leaf: 6A typical...

    // Maximum voltage value at the vehicle
    // connector contacts at which the charging station
    // stops charging to protect the vehicle battery
    uint16_t max_charge_voltage =  msg.data[4] | msg.data[5] << 8; // or is the correct name max_battery_voltage??????? or m_thresholdVoltage??
    // Leaf 40kwt: 435v typically. But why so high???? Battery max voltage is typically 395v fully charged.

    // In the first messages, soc is 240, at least in the trace i have seen...
    uint8_t soc_percent = msg.data[6]; // 0-100
}



// best
int approx_mul_0_11(int x) {
    return (x * 45) / 409;  // Closest integer-only approximation
}

// second best
int approx_mul_0_11(int x) {
    return (x * 14) >> 7; // equivalent to (x * 14) / 128
}

void handle_message_101(can_message_t msg)
{
    // max charging time 10s
    msg.data[1] = 0xFF; //not using 10 second increment mode

    // max charging time mins
    msg.data[2] = 90; //ask for how long of a charge? It will be forceably stopped if we hit this time

    // estimated charging time mins
    msg.data[3] = 60; //how long we think the charge will actually take

    // 0,11 kWh increments
    uint16_t battery_capacity_kwt = (msg.data[5] | msg.data[6] << 8) * 0.11;
    // Leaf 40 38.94 typical
}

void handle_message_102(can_message_t msg)
{
    uint8_t version = msg.data[0] = 2; // 1: v0.9, 2: v1.0

    // This is the charging voltage? I think so...
    uint16_t target_volt = msg.data[1] | msg.data[2] << 8;
    // Leaf 410 typical

    uint8_t askingAmps = msg.data[3];
    uint8_t faults = msg.data[4];
    uint8_t status = msg.data[5];
    //uint8_t kiloWattHours = msg.data[6];
    uint8_t chargingSpeedPercent = msg.data[6]; // ??? chargingRate
    // Leaf 72% typical. what does it mean???
}

void handle_message_108(can_message can)
{
    data[0] = 1; // EVContactorWeldingDetection: Supporting vehicle welding detection

    // max
    data[1] = availableOutputVoltage & 0xff;
    data[2] = availableOutputVoltage >> 8;

    // max
    data[3] = availableOutputCurrent;

    // Threshold voltage for terminating the charging process to protect the car battery
    // BUT WHY IS THE CHARGER SENDING THIS TO THE CAR?????
    data[4] = m_thresholdVoltage & 0xff;
    data[5] = m_thresholdVoltage >> 8;

    sendFrame(0x108, data);
}

void handle_message_109(can_message can)
{
    
    data[0] = 2; // ControlProtocolNumberQC, chademo v1.0

    data[1] = outputVoltage & 0xff;
    data[2] = outputVoltage >> 8;

    data[3] = outputCurrent;

    // fault
    data[4] = 0;
    // status
    data[5] = (statusChargerStopControl ? 0x20 : 0) |
        (statusStation ? 0x01 : 0) |
        (statusVehicleConnectorLock ? 4 : 0);

    data[6] = 0; // remaining charge time 10s ????
    data[7] = 60; // remaining charge time min
}

void prepare_and_send_charger_messages() {
    can_message_t msg;

    // --- 0x108: Status ---
    msg.id = 0x108;
    msg.dlc = 8;
    memset(msg.data, 0, 8);

    switch (currentState) 
    {
        case STATE_IDLE:      msg.data[0] = 0x00; break;
        case STATE_HANDSHAKE: msg.data[0] = 0x01; break;
        case STATE_READY:     msg.data[0] = 0x02; break;
        case STATE_CHARGING:  msg.data[0] = 0x03; break;
        case STATE_STOP:      msg.data[0] = 0x04; break;
        case STATE_FAULT:     msg.data[0] = 0xFF; msg.data[1] = 0x01; break;
    }

    send_can_message(&msg);

    // --- 0x109: Output Settings ---
    msg.id = 0x109;
    msg.dlc = 8;
    memset(msg.data, 0, 8);

    if (currentState == STATE_READY || currentState == STATE_CHARGING) {
        msg.data[0] = requested_voltage & 0xFF;
        msg.data[1] = (requested_voltage >> 8) & 0xFF;
        msg.data[2] = requested_current & 0xFF;
        msg.data[3] = (requested_current >> 8) & 0xFF;
    }

    send_can_message(&msg);
}

void handle_ev_message_0x101(const can_message_t* msg) {
    if (msg->dlc < 4) return;

    uint16_t req_voltage = msg->data[0] | (msg->data[1] << 8);
    uint16_t req_current = msg->data[2] | (msg->data[3] << 8);

    // Apply limits
    requested_voltage = (req_voltage <= MAX_VOLTAGE) ? req_voltage : MAX_VOLTAGE;
    requested_current = (req_current <= MAX_CURRENT) ? req_current : MAX_CURRENT;
}

void handle_ev_message_0x100(const can_message_t* msg) {
    if (msg->dlc < 1) return;

    uint8_t command = msg->data[0];

    switch (command) {
    case 0x01: // EV requests handshake
        if (currentState == STATE_IDLE) {
            currentState = STATE_HANDSHAKE;
            handshake_timer = 0;
        }
        break;
    case 0x02: // EV ready
        if (currentState == STATE_HANDSHAKE) {
            currentState = STATE_READY;
            handshake_timer = 0;
        }
        break;
    case 0x03: // Start charging
        if (currentState == STATE_READY) {
            currentState = STATE_CHARGING;
            charging_timer = 0;
        }
        break;
    case 0x04: // Stop charging
        if (currentState == STATE_CHARGING || currentState == STATE_READY) {
            currentState = STATE_STOP;
        }
        break;
    case 0xFF: // Fault
        currentState = STATE_FAULT;
        break;
    }
}



#define CYCLE_INTERVAL_MS 100

int main() 
{
    while (1) 
    {
        // Poll or receive CAN messages here
        can_message_t msg;
        if (receive_can_message(&msg)) 
        {
            switch (msg.id) 
            {
                case 0x100: handle_ev_message_0x100(&msg); break;
                case 0x101: handle_ev_message_0x101(&msg); break;
                // case 0x102: Optional extended info
            }
        }

        update_state_machine(CYCLE_INTERVAL_MS);
        prepare_and_send_charger_messages();
        delay_ms(CYCLE_INTERVAL_MS);
    }
}