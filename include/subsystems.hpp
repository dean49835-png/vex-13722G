#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

// inline pros::Motor intake(1);
//inline pros::adi::DigitalIn limit_switch('H');
inline pros::Motor in(16);
inline pros::Motor basket(15);
inline pros::Motor everything(14);

void set_in(int input);
void intake_opcontrol();

void set_basket(int input);
void intake_opcontrol();

void set_everything(int input);
void intake_opcontrol();

inline ez::Piston piston('H', true); 