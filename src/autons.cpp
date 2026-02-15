#include "main.h"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 110;
const int TURN_SPEED = 80;
const int SWING_SPEED = 127;            //speed: 110

///
// Constants
///
void default_constants() {
  // P, I, D, and Start I 
  //11 0 10     change k and d Fwd/rev constants, used for odom and non odom motions
  chassis.pid_drive_constants_forward_set(7.0, 0.0, 32.0);           //6 0 2 
  chassis.pid_drive_constants_backward_set(7.0, 0.0, 32.0);           //9 0 12

  chassis.pid_heading_constants_set(3.0, 0.0, 20.0);        //Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(3.0, 0.05, 20.0, 15.0);     // p i d start_i 3.0, 0.05, 20.0, 15.0           p i d angle. Turn in place constants
  chassis.pid_swing_constants_set(3.0, 0.0, 27.0);           // Swing constants
  chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions

  // Exit conditions
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(3_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(0.9);

  chassis.odom_look_ahead_set(7_in);               // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);     // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);       // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}

///
// Drive Example
///
void drive_example()               //right side 7 bloacks
{
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency
  chassis.odom_theta_set(0); 
  
  matchLoad.set(false);
  wing.set(true);
  intakeLeft.move(127);
  intakeRight.move(127);
  tripleUp.set(true);
  tripleDown.set(false);

  chassis.pid_drive_set(26_in, 110, true); 
  pros::delay(380);
  matchLoad.set(true); 
  chassis.pid_wait_quick();

  chassis.pid_turn_set(100_deg, 110);  
  chassis.pid_wait_quick();

  chassis.pid_drive_set(36_in, 110, true);
  chassis.pid_wait_quick();

  chassis.pid_turn_set(155_deg, 110);  
  chassis.pid_wait_quick();

  chassis.pid_drive_set(-20_in, 110, true);
  pros::delay(700);

  tripleUp.set(true);
  tripleDown.set(true); 
  pros::delay(1300);      //애가 스코어시간임
  tripleDown.set(false);
  chassis.pid_drive_set(31_in, 110, true);
  pros::delay(1200);
  chassis.pid_drive_set(-31_in, 110, true);
  pros::delay(400);
  tripleUp.set(true);
  tripleDown.set(true); 
  chassis.pid_wait_quick();

  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency
  chassis.odom_theta_set(0); 
  intakeLeft.move(0);
  intakeRight.move(0);
  chassis.pid_wait_quick();

  chassis.pid_drive_set(15_in, 110, true);
  chassis.pid_wait_quick();

  chassis.pid_turn_set(40_deg, 110);
  wing.set(false); 
  chassis.pid_wait_quick();

  chassis.pid_drive_set(-17_in, 110, true);
  chassis.pid_wait_quick();

  chassis.pid_turn_set(0_deg, 110);
  chassis.pid_wait_quick();

  chassis.pid_drive_set(-20_in, 60, true);
  chassis.pid_wait_quick();

  
}

///
// Turn Example
///
void turn_example() {                 //left side split

  //split left auto
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the current position, you can start at a specific position with this
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency
  
  matchLoad.set(false);
  wing.set(false);
  intakeLeft.move(127);
  intakeRight.move(127);
  tripleUp.set(true);
  tripleDown.set(false);
  
  chassis.pid_drive_set(26_in, 110, true); 
  pros::delay(380);
  matchLoad.set(true); 
  chassis.pid_wait_quick();

  chassis.pid_turn_set(-25_deg, 110);  
  pros::delay(100);
  matchLoad.set(false); 
  chassis.pid_wait_quick();

  chassis.pid_drive_set(26_in, 110, true);  
  pros::delay(450);
  matchLoad.set(true); 
  pros::delay(700);
  chassis.pid_drive_set(-10_in, 110, true); 
  chassis.pid_wait_quick(); 

  chassis.pid_turn_set(25_deg, 110);
  chassis.pid_wait_quick(); 
  chassis.pid_drive_set(-15_in, 110, true); 
  chassis.pid_wait_quick();
  chassis.pid_turn_set(115_deg, 110);
  chassis.pid_wait_quick(); 

  chassis.pid_drive_set(-10_in, 110, true); 
  pros::delay(500);
  chassis.pid_swing_set(ez::LEFT_SWING, 205_deg, -30, -110);
  pros::delay(1000);

  tripleUp.set(true);
  tripleDown.set(true);
  chassis.pid_drive_set(-5_in, 110, true); 
  pros::delay(950);
  tripleUp.set(true);
  tripleDown.set(false); 
  chassis.pid_wait_quick();

  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the current position, you can start at a specific position with this
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency
  
  chassis.pid_drive_set(32_in, 110, true);
  pros::delay(1300);
  chassis.pid_drive_set(-14_in, 110, true);
  chassis.pid_wait_quick();

  chassis.pid_turn_set(50_deg, 110);
  chassis.pid_wait_quick();

  chassis.pid_drive_set(-52_in, 110, true);
  chassis.pid_wait_quick();

  
  intakeLeft.move(90);
  intakeRight.move(90);
  tripleUp.set(false);
  tripleDown.set(false);
  pros::delay(1300);
  chassis.pid_wait_quick();

}

///
// Combining Turn + Drive
///
void drive_and_turn() {                                 //NOTHING
  wing.set(true);
  chassis.pid_drive_set(5.5_in, 127, true);  
  chassis.pid_wait_quick();
  
}

///
// Wait Until and Changing Max Speed
///
void wait_until_change_speed() {        //skills, but starts from left side

  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the current position, you can start at a specific position with this
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency
  
  matchLoad.set(false);
  intakeLeft.move(127);
  intakeRight.move(127);
  tripleUp.set(true);
  tripleDown.set(false);
  wing.set(true);

  chassis.pid_drive_set(26_in, 110, true); 
  chassis.pid_wait_quick();

  chassis.pid_turn_set(100_deg, 110); 
  matchLoad.set(true); 
  chassis.pid_wait_quick();

  chassis.pid_drive_set(35_in, 110, true);
  chassis.pid_wait_quick();

  chassis.pid_turn_set(155_deg, 110);  
  chassis.pid_wait_quick();

  chassis.pid_drive_set(-20_in, 110, true);
  pros::delay(700);

  tripleUp.set(true);
  tripleDown.set(true); 
  pros::delay(1300);      //애가 스코어시간임
  tripleDown.set(false);
  chassis.pid_drive_set(31_in, 110, true);
  pros::delay(2200);          //메치로더

  chassis.pid_drive_set(-16_in, 110, true);
  chassis.pid_wait_quick();

  intakeLeft.move(0);
  intakeRight.move(0);
  chassis.pid_turn_set(200_deg, 110);
  chassis.pid_wait_quick();

  chassis.pid_drive_set(-18_in, 110, true);
  chassis.pid_wait_quick();
  
  chassis.pid_turn_set(154_deg, 110);
  chassis.pid_wait_quick();

  matchLoad.set(false);
  chassis.pid_drive_set(-65_in, 110, true);
  chassis.pid_wait_quick();

  chassis.pid_turn_set(-115_deg, 110);
  chassis.pid_wait_quick();
  
  chassis.pid_drive_set(12_in, 110, true);
  chassis.pid_wait_quick();

  chassis.pid_turn_set(-25_deg, 110);
  chassis.pid_wait_quick();

  chassis.pid_drive_set(-12_in, 110, true);
  pros::delay(900);

  tripleUp.set(true);
  tripleDown.set(true); 
  intakeLeft.move(127);
  intakeRight.move(127);
  pros::delay(1000);      //애가 스코어시간임(2번째)
  chassis.pid_wait_quick();

  tripleDown.set(false);
  matchLoad.set(true); 
  chassis.pid_drive_set(31_in, 110, true);
  pros::delay(2200);
  chassis.pid_drive_set(-31_in, 110, true);
  pros::delay(800);
  tripleUp.set(true);
  tripleDown.set(true);
  pros::delay(1700);
  chassis.pid_drive_set(15_in, 110, true);
  chassis.pid_wait_quick();

  chassis.pid_turn_set(63_deg, 110);
  chassis.pid_wait_quick();

  chassis.pid_drive_set(-86_in, 110, true);
  chassis.pid_wait_quick();

  matchLoad.set(true);
  chassis.pid_turn_set(25_deg, 110);
  chassis.pid_wait_quick();

}



void swing_example() {                              

  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the current position, you can start at a specific position with this
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency
  
  wing.set(true);
  in.move(-127); 
  basket.move(127);
  everything.move(127); 

  chassis.pid_drive_set(22_in, 100, true); 
  chassis.pid_wait_quick();

  chassis.pid_swing_set(ez::LEFT_SWING, 70_deg, 127, 1);          
  chassis.pid_wait_quick();

  chassis.pid_drive_set(-35_in, 127, true);                   //go back to the goal
  chassis.pid_wait_quick();

  chassis.pid_swing_set(ez::RIGHT_SWING, 210_deg, 127, -127);
  chassis.pid_wait_quick();
  
  in.move(-100); 
  basket.move(100);
  everything.move(100);

  chassis.pid_drive_set(14_in, 100, true);                    //gets match loader
  chassis.pid_wait();

  chassis.pid_drive_set(-15_in, 127, true);
  chassis.pid_wait_quick();

  chassis.pid_swing_set(ez::RIGHT_SWING, 20_deg, 127, -127);        //25 deg
  chassis.pid_wait_quick();
  
  in.move(-127); 
  basket.move(-127);
  chassis.pid_drive_set(13_in, 127, true);
  chassis.pid_wait_quick();

  everything.move(127);  
  chassis.pid_wait_quick();
  pros::delay(15000);  
  
  /** 
  chassis.pid_drive_set(-9_in, 127, true);
  chassis.pid_wait_quick();

  chassis.pid_swing_set(ez::LEFT_SWING, -35_deg, 127, -127);        
  chassis.pid_wait_quick();

  chassis.pid_drive_set(17_in, 127, true);
  chassis.pid_wait_quick();

  wing.set(false);   
  chassis.pid_swing_set(ez::LEFT_SWING, 22_deg, 127, -127);     
  chassis.pid_wait_quick();

  chassis.pid_drive_set(20_in, 127, true);
  chassis.pid_wait_quick(); 
  */

  
}

///
// Motion Chaining
///
void motion_chaining() {              //odom

  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the current position, you can start at a specific position with this
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency
  
  chassis.pid_drive_set(20_in, 110, true);
  chassis.pid_wait();
  chassis.pid_turn_set(65_deg, 110);
  chassis.pid_wait();

}

///
// Auto that tests everything
///
void combining_movements() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, -45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Interference example
///
void tug(int attempts) {
  for (int i = 0; i < attempts - 1; i++) {
    // Attempt to drive backward
    printf("i - %i", i);
    chassis.pid_drive_set(-12_in, 127);
    chassis.pid_wait();

    // If failsafed...
    if (chassis.interfered) {
      chassis.drive_sensor_reset();
      chassis.pid_drive_set(-2_in, 20);
      pros::delay(1000);
    }
    // If the robot successfully drove back, return
    else {
      return;
    }
  }
}

// If there is no interference, the robot will drive forward and turn 90 degrees.
// If interfered, the robot will drive forward and then attempt to drive backward.
void interfered_example() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  if (chassis.interfered) {
    tug(3);
    return;
  }

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Odom Drive PID
///
void odom_drive_example() {
  // This works the same as pid_drive_set, but it uses odom instead!
  // You can replace pid_drive_set with pid_odom_set and your robot will
  // have better error correction.

  chassis.pid_odom_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit
///
void odom_pure_pursuit_example() {
  // Drive to 0, 30 and pass through 6, 10 and 0, 20 on the way, with slew
  chassis.pid_odom_set({{{6_in, 10_in}, fwd, DRIVE_SPEED},
                        {{0_in, 20_in}, fwd, DRIVE_SPEED},
                        {{0_in, 30_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  // Drive to 0, 0 backwards
  chassis.pid_odom_set({{0_in, 0_in}, rev, DRIVE_SPEED},
                       true);         //odom driving
  chassis.pid_wait();
}

///
// Odom Pure Pursuit Wait Until
///
void odom_pure_pursuit_wait_until_example() {
  chassis.pid_odom_set({{{0_in, 24_in}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait_until_index(1);  // Waits until the robot passes 12, 24
  // Intake.move(127);  // Set your intake to start moving once it passes through the second point in the index
  chassis.pid_wait();
  // Intake.move(0);  // Turn the intake off
}

///
// Odom Boomerang
///
void odom_boomerang_example() {
  chassis.pid_odom_set({{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Boomerang Injected Pure Pursuit
///
void odom_boomerang_injected_pure_pursuit_example() {
  chassis.pid_odom_set({{{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Calculate the offsets of your tracking wheels
///
void measure_offsets() {
  // Number of times to test
  int iterations = 10;

  // Our final offsets
  double l_offset = 0.0, r_offset = 0.0, b_offset = 0.0, f_offset = 0.0;

  // Reset all trackers if they exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->reset();
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->reset();
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->reset();
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->reset();
  
  for (int i = 0; i < iterations; i++) {
    // Reset pid targets and get ready for running an auton
    chassis.pid_targets_reset();
    chassis.drive_imu_reset();
    chassis.drive_sensor_reset();
    chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
    chassis.odom_xyt_set(0_in, 0_in, 0_deg);
    double imu_start = chassis.odom_theta_get();
    double target = i % 2 == 0 ? 90 : 270;  // Switch the turn target every run from 270 to 90

    // Turn to target at half power
    chassis.pid_turn_set(target, 63, ez::raw);
    chassis.pid_wait();
    pros::delay(250);

    // Calculate delta in angle
    double t_delta = util::to_rad(fabs(util::wrap_angle(chassis.odom_theta_get() - imu_start)));

    // Calculate delta in sensor values that exist
    double l_delta = chassis.odom_tracker_left != nullptr ? chassis.odom_tracker_left->get() : 0.0;
    double r_delta = chassis.odom_tracker_right != nullptr ? chassis.odom_tracker_right->get() : 0.0;
    double b_delta = chassis.odom_tracker_back != nullptr ? chassis.odom_tracker_back->get() : 0.0;
    double f_delta = chassis.odom_tracker_front != nullptr ? chassis.odom_tracker_front->get() : 0.0;

    // Calculate the radius that the robot traveled
    l_offset += l_delta / t_delta;
    r_offset += r_delta / t_delta;
    b_offset += b_delta / t_delta;
    f_offset += f_delta / t_delta;
  }

  // Average all offsets
  l_offset /= iterations;
  r_offset /= iterations;
  b_offset /= iterations;
  f_offset /= iterations;

  // Set new offsets to trackers that exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->distance_to_center_set(l_offset);
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->distance_to_center_set(r_offset);
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->distance_to_center_set(b_offset);
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->distance_to_center_set(f_offset);
}

// . . .
// Make your own autonomous functions here!
// . . .