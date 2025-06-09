#include "Copter.h"

#if MODE_DRAWSTAR_ENABLED == ENABLED

static Vector3p guided_pos_target_cm; 
bool guided_pos_terrain_alt_1;   
static Vector3f guided_vel_target_cms; 
static Vector3f guided_accel_target_cmss;  
static uint32_t update_time_ms;

struct {
    uint32_t update_time_ms;
    Quaternion attitude_quat;
    Vector3f ang_vel;
    float yaw_rate_cds;
    float climb_rate_cms;   // climb rate in cms.  Used if use_thrust is false
    float thrust;           // thrust from -1 to 1.  Used if use_thrust is true
    bool use_yaw_rate;
    bool use_thrust;
} static guided_angle_state;

struct Guided_Limit {
    uint32_t timeout_ms;  // timeout (in seconds) from the time that guided is invoked
    float alt_min_cm;   // lower altitude limit in cm above home (0 = no limit)
    float alt_max_cm;   // upper altitude limit in cm above home (0 = no limit)
    float horiz_max_cm; // horizontal position limit in cm from where guided mode was initiated (0 = no limit)
    uint32_t start_time;// system time in milliseconds that control was handed to the external computer
    Vector3f start_pos; // start position as a distance from home in cm.  used for checking horiz_max limit
} guided_limit_1;

// init - initialise guided controller
bool ModeDrawStar::init(bool ignore_checks)
{
    // start in velaccel control mode
    // velaccel_control_start();
    guided_vel_target_cms.zero();
    guided_accel_target_cmss.zero();

    // clear pause state when entering guided mode
    _paused = false;

    path_num = 0;
    generate_path();
    pos_control_start();

    return true;
}

void ModeDrawStar::generate_path()
{
    // float radius_cm = 100.0;

    float radius_cm = g2.star_radius_cm;

    wp_nav->get_wp_stopping_point(path[0]);

    path[1] = path[0] + Vector3f(1.0f, 0, 0) * radius_cm;
    path[2] = path[0] + Vector3f(-cosf(radians(36.0f)), -sinf(radians(36.0f)), 0) * radius_cm;
    path[3] = path[0] + Vector3f(sinf(radians(18.0f)), cosf(radians(18.0f)), 0) * radius_cm;
    path[4] = path[0] + Vector3f(sinf(radians(18.0f)), -cosf(radians(18.0f)), 0) * radius_cm;
    path[5] = path[0] + Vector3f(-cosf(radians(36.0f)), sinf(radians(36.0f)), 0) * radius_cm;
    path[6] = path[1];
}

// initialise guided mode's position controller
void ModeDrawStar::pos_control_start()
{
    // initialise position controller
    pva_control_start();
}

// initialise position controller
void ModeDrawStar::pva_control_start()
{
    // initialise horizontal speed, acceleration
    pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
    pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());

    // initialize vertical speeds and acceleration
    pos_control->set_max_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());
    pos_control->set_correction_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());

    // initialise velocity controller
    pos_control->init_z_controller();
    pos_control->init_xy_controller();

    wp_nav->set_wp_destination(path[0], false);

    // initialise yaw
    auto_yaw.set_mode_to_default(false);

    // initialise terrain alt
    guided_pos_terrain_alt_1 = false;
}

// run - runs the guided controller
// should be called at 100hz or more
void ModeDrawStar::run()
{
    if(path_num < 6){
        if(wp_nav->reached_wp_destination()){
            path_num ++;
            wp_nav->set_wp_destination(path[path_num], false);
        }
    }

    pos_control_run();
 }

// initialise guided mode's velocity and acceleration controller
void ModeDrawStar::velaccel_control_start()
{
    // set guided_mode to velocity controller
    guided_mode = SubMode::VelAccel;

    // initialise position controller
    pva_control_start();
}

// pos_control_run - runs the guided position controller
// called from guided_run
void ModeDrawStar::pos_control_run()
{
    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        // do not spool down tradheli when on the ground with motor interlock enabled
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        return;
    }

    // calculate terrain adjustments
    float terr_offset = 0.0f;
    if (guided_pos_terrain_alt_1 && !wp_nav->get_terrain_offset(terr_offset)) {
        // failure to set destination can only be because of missing terrain data
        copter.failsafe_terrain_on_event();
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // send position and velocity targets to position controller
    guided_accel_target_cmss.zero();
    guided_vel_target_cms.zero();

    // stop rotating if no updates received within timeout_ms
    if (millis() - update_time_ms > get_timeout_ms()) {
        if ((auto_yaw.mode() == AutoYaw::Mode::RATE) || (auto_yaw.mode() == AutoYaw::Mode::ANGLE_RATE)) {
            auto_yaw.set_mode(AutoYaw::Mode::HOLD);
        }
    }

    float pos_offset_z_buffer = 0.0; // Vertical buffer size in m
    if (guided_pos_terrain_alt_1) {
        pos_offset_z_buffer = MIN(copter.wp_nav->get_terrain_margin() * 100.0, 0.5 * fabsF(guided_pos_target_cm.z));
    }
    pos_control->input_pos_xyz(guided_pos_target_cm, terr_offset, pos_offset_z_buffer);

    // run position controllers
    pos_control->update_xy_controller();
    pos_control->update_z_controller();

    // call attitude controller with auto yaw
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());
}

const Vector3p &ModeDrawStar::get_target_pos() const
{
    return guided_pos_target_cm;
}

const Vector3f& ModeDrawStar::get_target_vel() const
{
    return guided_vel_target_cms;
}

const Vector3f& ModeDrawStar::get_target_accel() const
{
    return guided_accel_target_cmss;
}

uint32_t ModeDrawStar::wp_distance() const
{
    switch(guided_mode) {
    case SubMode::WP:
        return wp_nav->get_wp_distance_to_destination();
    case SubMode::Pos:
        return get_horizontal_distance_cm(inertial_nav.get_position_xy_cm(), guided_pos_target_cm.tofloat().xy());
    case SubMode::PosVelAccel:
        return pos_control->get_pos_error_xy_cm();
    default:
        return 0;
    }
}

int32_t ModeDrawStar::wp_bearing() const
{
    switch(guided_mode) {
    case SubMode::WP:
        return wp_nav->get_wp_bearing_to_destination();
    case SubMode::Pos:
        return get_bearing_cd(inertial_nav.get_position_xy_cm(), guided_pos_target_cm.tofloat().xy());
    case SubMode::PosVelAccel:
        return pos_control->get_bearing_to_target_cd();
    case SubMode::TakeOff:
    case SubMode::Accel:
    case SubMode::VelAccel:
    case SubMode::Angle:
        // these do not have bearings
        return 0;
    }
    // compiler guarantees we don't get here
    return 0.0;
}

float ModeDrawStar::crosstrack_error() const
{
    switch (guided_mode) {
    case SubMode::WP:
        return wp_nav->crosstrack_error();
    case SubMode::Pos:
    case SubMode::TakeOff:
    case SubMode::Accel:
    case SubMode::VelAccel:
    case SubMode::PosVelAccel:
        return pos_control->crosstrack_error();
    case SubMode::Angle:
        // no track to have a crosstrack to
        return 0;
    }
    // compiler guarantees we don't get here
    return 0;
}

// return guided mode timeout in milliseconds. Only used for velocity, acceleration, angle control, and angular rates
uint32_t ModeDrawStar::get_timeout_ms() const
{
    return MAX(copter.g2.guided_timeout, 0.1) * 1000;
}

// pause guide mode
bool ModeDrawStar::pause()
{
    _paused = true;
    return true;
}

// resume guided mode
bool ModeDrawStar::resume()
{
    _paused = false;
    return true;
}

#endif
