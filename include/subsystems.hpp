#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

// inline pros::Motor intake(1);
//inline pros::adi::DigitalIn limit_switch('H');
inline pros::Motor in(16);                                              //midlle roller
inline pros::Motor basket(15);                                          //big roller + basket
inline pros::Motor everything(14);                                      //intake

inline pros::Motor intakeLeft(-10);                                      //intake left
inline pros::Motor intakeRight(8);                                      //intake right


void set_in(int input);
void intake_opcontrol();

void set_basket(int input);
void intake_opcontrol();

void set_everything(int input);
void intake_opcontrol();

inline ez::Piston tripleDown('H', true); 
inline ez::Piston tripleUp('G', true); 
inline ez::Piston matchLoad('F', true); 
inline ez::Piston wing('D', false); 
inline ez::Piston odomPull('E', true); 

inline ez::Piston pod('B', false); 