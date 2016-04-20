/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

/*
 * control_stabilize.pde - init and run calls for stabilize flight mode
 */








   //---------------------------------------------------------------------------------------------------------------------------------------------
    //kd adc variables  conv factor is 3 since lean angle is in centi degrees so a 30 degree lean would be 3000, and the adc pulls in max 1023, so 3*1023 would give a lean angle of 30 degrees
    float adcroll = 0, adcpitch = 0, criticalfactor = 1, convfactor = 3;

    //kd Analog In setup, may not be complete
    AP_HAL::AnalogSource* ch1;
    AP_HAL::AnalogSource* ch2;
    AP_HAL::AnalogSource* ch3;

    void setup (void) {
        hal.console->printf("Starting AP_HAL::AnalogIn test\r\n");
        //ch = hal.analogin->channel(13);
        ch1 = hal.analogin->channel(15);
        ch2 = hal.analogin->channel(13);
        ch3 = hal.analogin->channel(14);
    }
    //--------------------------------------------------------------------------------------------------------------------------------------------- 



// stabilize_init - initialise stabilize controller
bool Copter::stabilize_init(bool ignore_checks)
{
    
   
    
    // if landed and the mode we're switching from does not have manual throttle and the throttle stick is too high
    if (motors.armed() && ap.land_complete && !mode_has_manual_throttle(control_mode) && (get_pilot_desired_throttle(channel_throttle->control_in) > get_non_takeoff_throttle())) {
        return false;
    }
    // set target altitude to zero for reporting
    pos_control.set_alt_target(0);

    return true;
}

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
void Copter::stabilize_run()
{

    float target_roll, target_pitch;
    float target_yaw_rate;
    float pilot_throttle_scaled;

    // if not armed set throttle to zero and exit immediately
    if (!motors.armed() || ap.throttle_zero || !motors.get_interlock()) {
        motors.set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
        return;
    }

    motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // convert pilot input to lean angles
    // To-Do: convert get_pilot_desired_lean_angles to return angles as floats
    get_pilot_desired_lean_angles(channel_roll->control_in, channel_pitch->control_in, target_roll, target_pitch, aparm.angle_max);

    
    //-------------------------------------------------------------------------------------------------------------------------------------------
    //kd should just change target_roll, and target_pitch with adc input  (need to know a good scaling factor). 
    adcpitch = ch1->voltage_average(); 
    target_pitch = target_pitch + adcpitch*convfactor*criticalfactor;
    //target_roll = target_roll + adcroll*cfactor;
    //---------------------------------------------------------------------------------------------------------------------------------------------
    
    // get pilot's desired yaw rate
    target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->control_in);

    // get pilot's desired throttle
    pilot_throttle_scaled = get_pilot_desired_throttle(channel_throttle->control_in);

    // call attitude controller
    attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

    // body-frame rate controller is run directly from 100hz loop

    // output pilot's throttle
    attitude_control.set_throttle_out(pilot_throttle_scaled, true, g.throttle_filt);
}
