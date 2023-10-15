#ifndef ECU_CAN_INTERFACE_H
#define ECU_CAN_INTERFACE_H

#include "MainCANInterface.h"
#include "pindef.h"
// TODO: change the NC, NC in the instantiation of MainCANInteface to rx and tx pins for serial 
class ECUCANInterface : public MainCANInterface {
  public:
    ECUCANInterface(PinName rd, PinName td, PinName standby_pin)
        : MainCANInterface(rd, td, standby_pin, PI_UART_TX, PI_UART_RX) {}
    
    void handle(MotorControllerPowerStatus *can_struct) override;
    void handle(BPSPackInformation *can_struct) override;
    // take the handle functions from main can interface and override them here
    void send_to_pi(CANMessage *message, uint16_t message_id);
};

#endif
