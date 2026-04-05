#include "input/rc_input.h"

#include "sensor/sensor_common.h"

void RcReceiver_Init(void)
{
    /*
     * FC_F411.ioc does not define RC input-capture timers/pins.
     * Keep the controller in a safe disarmed state until RC input
     * is implemented for this board configuration.
     */
    RC_Raw_Roll = 1500;
    RC_Raw_Pitch = 1500;
    RC_Raw_Yaw = 1500;
    RC_Raw_Throttle = 1000;
    RC_Raw_SW_Arm = 1000;
    RC_Raw_SW_Mode = 1000;
}
