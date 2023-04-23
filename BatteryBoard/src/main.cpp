// #include "BPSCANInterface.h"
// #include "BPSRelayController.h"
#include "DigitalOut.h"
// #include "PowerAuxCANInterface.h"
#include "Printing.h"
#include "ThisThread.h"
#include "log.h"
#include "pindef.h"
#include <mbed.h>
#include <rtos.h>

#define LOG_LEVEL          LOG_DEBUG
#define MAIN_LOOP_PERIOD   1s
#define ERROR_CHECK_PERIOD 100ms
#define FLASH_PERIOD       500ms
#define PRECHARGE_PAUSE    100ms

// PowerAuxCANInterface vehicle_can_interface(MAIN_CAN_RX, MAIN_CAN_TX,
                                        //    MAIN_CAN_STBY);
// BPSCANInterface bps_can_interface(BMS_CAN1_RX, BMS_CAN1_TX, BMS_CAN1_STBY);

bool flashHazards, flashLSignal, flashRSignal = false;
Thread signalFlashThread;

AnalogIn fan_tach(FanTach);
AnalogIn brake_light_current(BRAKE_LIGHT_CURRENT);
AnalogIn headlight_current(DRL_CURRENT);
AnalogIn bms_strobe_current(BMS_STROBE_CURRENT);
AnalogIn left_turn_current(LEFT_TURN_CURRENT);
AnalogIn right_turn_current(RIGHT_TURN_CURRENT);
Thread peripheral_error_thread;

BPSRelayController bps_relay_controller(HORN_EN, DRL_EN, AUX_PLUS,
                                        BMS_STROBE_EN);

// void peripheral_error_handler() {
//     PowerAuxError msg;
//     while (true) {
//         msg.bps_strobe_error = (bms_strobe_current.read_u16() < 1000 &&
//                                 bps_relay_controller.bps_fault_indicator_on());
//         msg.brake_light_error =
//             (brake_light_current.read_u16() < 1000 && brake_lights.read());
//         msg.fan_error = (fan_tach.read_u16() < 1000);
//         msg.left_turn_error =
//             (left_turn_current.read_u16() < 1000 && leftTurnSignal.read());
//         msg.right_turn_error =
//             (right_turn_current.read_u16() < 1000 && rightTurnSignal.read());
//         msg.bps_error = bps_relay_controller.bps_has_fault();

//         vehicle_can_interface.send(&msg);
//         ThisThread::sleep_for(ERROR_CHECK_PERIOD);
//     }
// }

DigitalOut chargeRelay(CHARGE_RELAY);
DigitalOut vbus(BUS_12V);
Thread precharge_check;
bool allow_precharge = true;


void start_precharge() { //Enables switch to start precharging
    chargeRelay = true;
}


void battery_precharge() { 
    while (true) {
        int relay_status = chargeRelay.read();
        int vbus_status = vbus.read();

        if(relay_status && vbus_status && allow_precharge) {
            allow_precharge = false;
            start_precharge();
            continue;
        }
        if(!relay_status || !vbus) {
            bool dont_allow_charge = false;
            chrono::steady_clock::time_point start = chrono::steady_clock::now();
            while(chrono::duration_cast<chrono::seconds>(chrono::steady_clock::now() - start).count() < 30) {
                if(relay_status || vbus) {
                    dont_allow_charge = true;
                    break;
                }
                ThisThread::sleep_for(PRECHARGE_PAUSE);
            }
            if(!dont_allow_charge) {
                allow_precharge = true;
            }
            continue;
        }
    }
    
}


int main() {
    log_set_level(LOG_LEVEL);
    log_debug("Start main()");

    // signalFlashThread.start(signalFlashHandler);
    // peripheral_error_thread.start(peripheral_error_handler);
    precharge_check.start(battery_precharge);

    while (true) {
        log_debug("Main thread loop");

        ThisThread::sleep_for(MAIN_LOOP_PERIOD);
    }
}

// void PowerAuxCANInterface::handle(ECUPowerAuxCommands *can_struct) {
//     can_struct->log(LOG_INFO);

//     brake_lights = can_struct->brake_lights;

//     flashLSignal = can_struct->left_turn_signal;
//     flashRSignal = can_struct->right_turn_signal;
//     flashHazards = can_struct->hazards;

//     signalFlashThread.flags_set(0x1);
// }

// void BPSCANInterface::handle(BPSPackInformation *can_struct) {
//     can_struct->log(LOG_INFO);

//     bps_relay_controller.update_state(can_struct);

//     vehicle_can_interface.send(can_struct);
// }

// void BPSCANInterface::handle(BPSError *can_struct) {
//     can_struct->log(LOG_INFO);

//     bps_relay_controller.update_state(can_struct);

//     vehicle_can_interface.send(can_struct);
// }

// void BPSCANInterface::handle(BPSCellVoltage *can_struct) {
//     can_struct->log(LOG_INFO);

//     vehicle_can_interface.send(can_struct);
// }

// void BPSCANInterface::handle(BPSCellTemperature *can_struct) {
//     can_struct->log(LOG_INFO);

//     vehicle_can_interface.send(can_struct);
// }
