#include "main.h"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 110;
const int TURN_SPEED = 80;
const int SWING_SPEED = 110;

///
// Constants
///
void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_set(8.0, 0.0, 28.0);            //4 0 1     change k and d Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(3.0, 0.0, 20.0);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(4.0, 0.05, 93.0, 15.0);     //2.0, 0.05, 60.0, 15.0           p i d angle. Turn in place constants
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants
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
void drive_example()  
{
  piston.set(false);
  in.move(0); 
  basket.move(127);
  everything.move(127); 

  chassis.pid_drive_set(28_in, 110, true);  
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, SWING_SPEED, 5);
  chassis.pid_wait();

  chassis.pid_drive_set(5_in, 70, true);  
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(-10_in, 110, true);  
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 100_deg, SWING_SPEED, 5);
  chassis.pid_wait();
  
  chassis.pid_drive_set(38_in, 110, true);        //maybe 40 if it goes to less
  chassis.pid_wait();

  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();

  piston.set(true);
  chassis.pid_wait_quick_chain();

  everything.move(65); 
  chassis.pid_drive_set(26_in, 40, true);         //first time driving towards match loader
  chassis.pid_wait();
 
  /**for (int i = 0; i < 5; i++)
  {
    chassis.pid_drive_set(-5_in, 70, true);         
    chassis.pid_wait_quick_chain();
    chassis.pid_drive_set(5_in, 70, true);         
    chassis.pid_wait_quick_chain();
  } */

  chassis.pid_drive_set(-18_in, 110, true);  
  chassis.pid_wait();

  piston.set(false);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
  
  chassis.pid_drive_set(10_in, 110, true);                    //going close to the goal
  chassis.pid_wait();
  
  in.move(0);
  basket.move(0);
  everything.move(0); 
  chassis.pid_wait();

  in.move(-127);
  basket.move(-127);
  everything.move(127); 
  chassis.pid_wait();

  pros::delay(20000);
  chassis.pid_wait();
}

///
// Turn Example
///
void turn_example() {                 //second, left

  piston.set(false);
  in.move(0); 
  basket.move(127);
  everything.move(95); 

  chassis.pid_drive_set(28_in, 110, true);          
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING, -30_deg, SWING_SPEED, 5);
  chassis.pid_wait();

  chassis.pid_drive_set(17_in, 45, true);                  //approaching the balls slowly
  chassis.pid_wait(); 
  pros::delay(400);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(-17_in, 110, true);  
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, -120_deg, SWING_SPEED, 5);
  chassis.pid_wait();
  
  chassis.pid_drive_set(26_in, 110, true);        //going towards the goal but doesn't face it
  chassis.pid_wait();

  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();

  piston.set(true);
  chassis.pid_wait_quick_chain();

  everything.move(65); 
  chassis.pid_drive_set(25_in, 40, true);         //eating ballls from match loader
  chassis.pid_wait();
 
  /**for (int i = 0; i < 5; i++)
  {
    chassis.pid_drive_set(-5_in, 70, true);         
    chassis.pid_wait_quick_chain();
    chassis.pid_drive_set(5_in, 70, true);         
    chassis.pid_wait_quick_chain();
  } */

  chassis.pid_drive_set(-18_in, 110, true);
  pros::delay(600);
  chassis.pid_wait();

  piston.set(false);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
  
  chassis.pid_drive_set(10_in, 110, true);                    //going close to the goal
  chassis.pid_wait();
  
  in.move(0);
  basket.move(0);
  everything.move(0); 
  chassis.pid_wait();

  in.move(-127);
  basket.move(-127);
  everything.move(127); 
  chassis.pid_wait();

  pros::delay(20000);
  chassis.pid_wait();

}

///
// Combining Turn + Drive
///
void drive_and_turn() {

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
  
  chassis.pid_turn_set(180_deg, TURN_SPEED);
chassis.pid_wait();

chassis.pid_turn_set(270_deg, TURN_SPEED);
chassis.pid_wait();

chassis.pid_turn_set(0_deg, TURN_SPEED);
chassis.pid_wait();
  

}

///
// Wait Until and Changing Max Speed
///
void wait_until_change_speed() {        //skills right side
  wing.set(true);
  piston.set(false);
  in.move(-127); 
  basket.move(127);
  everything.move(127); 

  chassis.pid_drive_set(20_in, 100, true);  
  chassis.pid_wait_quick();

  piston.set(true);
  chassis.pid_wait_quick();

  chassis.pid_swing_set(ez::LEFT_SWING, 90_deg, SWING_SPEED, 5);
  chassis.pid_wait_quick();
  
  chassis.pid_drive_set(12_in, 100, true);  
  chassis.pid_wait_quick();
  pros::delay(1200);                                            // first match loader

  chassis.pid_drive_set(-15_in, 110, true);  
  chassis.pid_wait_quick();

  chassis.pid_swing_set(ez::LEFT_SWING, -90_deg, SWING_SPEED, 5);
  chassis.pid_wait_quick();

  chassis.pid_drive_set(50_in, 90, true);  
  chassis.pid_wait_quick();

  chassis.pid_drive_set(40_in, 60, true);  
  chassis.pid_wait_quick();                                               //crosses middle
  
  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, SWING_SPEED, 5);
  chassis.pid_wait_quick();

  chassis.pid_swing_set(ez::RIGHT_SWING, -90_deg, SWING_SPEED, 5);
  chassis.pid_wait_quick();

  chassis.pid_drive_set(13_in, 80, true);       //15inch 
  chassis.pid_wait_quick();
  
  pros::delay(1200);                                    // second match loader                          

  chassis.pid_drive_set(-12_in, 110, true);  
  chassis.pid_wait_quick();

  piston.set(false);
  chassis.pid_wait_quick();

  chassis.pid_turn_set(90_deg, 80);
  chassis.pid_wait();

  chassis.pid_drive_set(19_in, 60, true);  
  everything.move(127);
  basket.move(-127);
  chassis.pid_wait_quick();

  in.move(-127);
  pros::delay(4000);

  chassis.pid_swing_set(ez::RIGHT_SWING, 175_deg, SWING_SPEED, 5);
  chassis.pid_wait();

  chassis.pid_drive_set(50_in, 80, true);  
  chassis.pid_wait_quick();

  chassis.pid_drive_set(48_in, 90, true);  
  chassis.pid_wait_quick();   

  chassis.pid_turn_set(-90_deg, 90);
  chassis.pid_wait_quick();

  piston.set(true);
  chassis.pid_wait_quick();

  in.move(-127); 
  basket.move(127);
  everything.move(127); 

  chassis.pid_drive_set(23.5_in, 60, true);                    //third match loader     
  chassis.pid_wait_quick();

  pros::delay(1200);

  chassis.pid_drive_set(-15_in, 110, true);
  chassis.pid_wait_quick();

  chassis.pid_swing_set(ez::LEFT_SWING, 84_deg, SWING_SPEED, 5);
  chassis.pid_wait_quick();

  chassis.pid_drive_set(40_in, 100, true);  
  chassis.pid_wait_quick();

  chassis.pid_drive_set(40_in, 100, true);  
  chassis.pid_wait_quick(); 

  chassis.pid_swing_set(ez::LEFT_SWING, 180_deg, SWING_SPEED, 5);
  chassis.pid_wait_quick();

  chassis.pid_swing_set(ez::RIGHT_SWING, 90_deg, 90, 5);
  chassis.pid_wait_quick();

  chassis.pid_turn_set(90_deg, 110);
  chassis.pid_wait_quick();

  chassis.pid_drive_set(24.5_in, 70, true);  
  chassis.pid_wait_quick(); 

  pros::delay(1200);                                    // fourth match loader
  chassis.pid_wait_quick();

  chassis.pid_drive_set(-20_in, 70, true);  
  chassis.pid_wait_quick(); 

  chassis.pid_turn_set(-98_deg, 90);                  //faces fourth goal
  piston.set(false);
  chassis.pid_wait_quick();

  chassis.pid_drive_set(10_in, 90, true);  
  everything.move(127);
  basket.move(-127);
  chassis.pid_wait_quick(); 

  in.move(-127);
  pros::delay(4000);

  everything.move(0);
  basket.move(0);
  in.move(0);
  pod.set(true);
  wing.set(false);
  chassis.pid_wait_quick();

  chassis.pid_drive_set(-10_in, 70, true);  
  chassis.pid_wait_quick();

  chassis.pid_turn_set(20_deg, 90);                  //faces parking zone
  piston.set(false);
  chassis.pid_wait_quick();

  chassis.pid_drive_set(-15_in, 127, true);  
  chassis.pid_wait_quick();

  chassis.pid_drive_set(100_in, 127, true);  
  chassis.pid_wait_quick();

 

  pros::delay(1000000);


  chassis.pid_turn_set(25_deg, 40);
  chassis.pid_wait();

  chassis.pid_drive_set(16.5_in, 30, true);  
  chassis.pid_wait();

  chassis.pid_drive_set(-10_in, 110, true);  
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 100_deg, SWING_SPEED, 5);
  chassis.pid_wait();
  
  chassis.pid_drive_set(34_in, 110, true);        //to the goal but not facing
  chassis.pid_wait();

  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();

  piston.set(true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(21_in, 55, true);         //first time driving towards match loader
  chassis.pid_wait();
  pros::delay(1000);

  chassis.pid_drive_set(-18_in, 110, true);  
  chassis.pid_wait();

  piston.set(false);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
  
  chassis.pid_drive_set(9_in, 90, true);                    //going close to the goal
  chassis.pid_wait();

  in.move(-127);
  basket.move(-127);
  everything.move(100); 
  chassis.pid_wait();

  pros::delay(5500);
  chassis.pid_wait();
  
  in.move(0);
  basket.move(0);
  everything.move(0); 
  chassis.pid_wait_quick();

  chassis.pid_drive_set(-10_in, 110, true);
  chassis.pid_wait();

  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(15.5_in, 110, true);          //facing the left
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, SWING_SPEED, 5);
  chassis.pid_wait();

  chassis.pid_drive_set(70_in, 80, true);   
  chassis.pid_wait_quick_chain();

/** 
  in.move(-127); 
  basket.move(127);
  everything.move(127); 
  chassis.pid_wait();

  
  chassis.pid_turn_set(5_deg, TURN_SPEED);          //turns to get the far side balls
  chassis.pid_wait();

  chassis.pid_drive_set(35_in, 30, true);   
  chassis.pid_wait();
  **/

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(21_in, 100, true);             //second time driving towards goals but not facing it   
  chassis.pid_wait();

  in.move(-127);           
  basket.move(127);
  everything.move(127);
  chassis.pid_wait();

  piston.set(true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0_deg, TURN_SPEED);            //facing the match loaders
  chassis.pid_wait();

  chassis.pid_drive_set(18_in, 65, true);               //getting into the match loaders
  chassis.pid_wait();  

  everything.move(127);
  pros::delay(700);
  chassis.pid_wait();
 
  chassis.pid_drive_set(-10_in, 70, true);         
  chassis.pid_wait();
  
  chassis.pid_drive_set(10_in, 70, true);         
  chassis.pid_wait();

  chassis.pid_drive_set(-18_in, 110, true);         //wiggles, still faces match loader
  chassis.pid_wait();

  piston.set(false);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();
  
  chassis.pid_drive_set(10_in, 90, true);                    //going close to the goal
  chassis.pid_wait();
  
  in.move(0);
  basket.move(0);
  everything.move(0); 
  chassis.pid_wait();

  in.move(-127);
  basket.move(-127);
  everything.move(127); 
  chassis.pid_wait();

  pros::delay(1200);
  chassis.pid_wait();                             //SCORING makes ure that the balls are secured(the further side right in reference frame of the strting point)
  
  in.move(0);
  basket.move(0);
  everything.move(0); 
  chassis.pid_wait();

  chassis.pid_drive_set(-15_in, 110, true);            //still faces the goal       
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING, -90_deg, SWING_SPEED, 5);      //faces the far right side
  chassis.pid_wait();

  in.move(-127); 
  basket.move(127);
  everything.move(127); 
  chassis.pid_wait();                                   //anti jam the balls

  in.move(0); 
  basket.move(0);
  everything.move(0); 
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(55_in, 80, true);
  chassis.pid_wait();

  chassis.pid_turn_set(215_deg, TURN_SPEED);        //gets the far left balls in the left side
  chassis.pid_wait();

  in.move(-127); 
  basket.move(127);
  everything.move(127); 
  chassis.pid_wait();

  chassis.pid_drive_set(20_in, 30, true);                            
  chassis.pid_wait();

  chassis.pid_drive_set(-20_in, 30, true);                            
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, SWING_SPEED, 5);      //faces far side left but it's on the left side
  chassis.pid_wait();

  in.move(0); 
  basket.move(0);
  everything.move(0); 
  chassis.pid_wait();

  chassis.pid_drive_set(12_in, 40, true);
  chassis.pid_wait();

  chassis.pid_turn_set(-90_deg, TURN_SPEED);          
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, SWING_SPEED, 5);      //faces the far left side
  chassis.pid_wait();

  piston.set(true);
  chassis.pid_wait();

  chassis.pid_drive_set(30_in, 60, true);
  chassis.pid_wait();








  chassis.pid_swing_set(ez::LEFT_SWING, -90_deg, SWING_SPEED, 5);      //faces the far left side
  chassis.pid_wait();
  
  piston.set(true);
  chassis.pid_wait();

  chassis.pid_turn_set(225_deg, TURN_SPEED);          //faces middle goal
  chassis.pid_wait();

  chassis.pid_drive_set(38_in, 40, true);                            
  chassis.pid_wait();

  in.move(68);                                       //score middle goal
  basket.move(-68);
  everything.move(68); 

  pros::delay(4200);
  chassis.pid_wait();

  in.move(-127); 
  basket.move(127);
  everything.move(127); 
  chassis.pid_wait();

  chassis.pid_drive_set(-10_in, 100, true);        
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(135_deg, TURN_SPEED);          //faces lower goal
  chassis.pid_wait();

  chassis.pid_drive_set(10_in, 100, true);        
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(30_in, 30, true);           //drive towards lower goal 
  chassis.pid_wait();




  

}

///
// Swing Example
///
void swing_example() {
  chassis.pid_drive_set(10_in, DRIVE_SPEED, true);
  chassis.pid_wait();

/** 
  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  */


}

///
// Motion Chaining
///
void motion_chaining() {
  // Motion chaining is where motions all try to blend together instead of individual movements.
  // This works by exiting while the robot is still moving a little bit.
  // To use this, replace pid_wait with pid_wait_quick_chain.
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // Your final motion should still be a normal pid_wait
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
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