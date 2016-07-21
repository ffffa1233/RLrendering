#include <string.h>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>

#include <rlglue/Environment_common.h>
#include <rlglue/utils/C/RLStruct_util.h>
#include <rlglue/utils/C/TaskSpec_Parser.h>
#include <rlglue/RL_glue.h>

#include "Framework/Test.h"
#include "Tests/Pendulum1.h"

static observation_t this_observation;
static reward_observation_terminal_t this_reward_observation;

Pendulum1 pendulum1;

b2Body* verticalBody;
b2Body* horizonBody;
b2Body* lineBody;

int calculate_reward(double angle, double angleVel, double vel);
int check_terminal(double angle, double angleVel, double vel);

const char *env_init(){
	const char *task_spec_string = "[VERSION RL-GLue-3.0] [PROBLEMTYPE episodic] "
									"[OBSERVATIONS angle(-90 90) angle velocity(-50  50) velocity(-50 50) ] "
									"[ACTIONS (-3 3) ] [REWARDS 100 0 ] "
									"[EXTRA environment(C/C++) by PSC, CBH]";

	allocateRLStruct(&this_observation,0,3,0);
	this_reward_observation.observation = &this_observation;
	this_reward_observation.reward = 0;
	this_reward_observation.terminal = 0;

	return task_spec_string;
}

const observation_t *env_start(){
	double angle = 0.0;
	double angleVel = 0.0;
	double vel = 0.0;

	angle = pendulum1.get_angle();
	angleVel = pendulum1.get_angleVelocity() + 3;
	vel = pendulum1.get_velocity() + 50;

	this_observation.doubleArray[0] = angle;
	this_observation.doubleArray[1] = angleVel;
	this_observation.doubleArray[2] = vel;
	return &this_observation;
 }

const reward_observation_terminal_t *env_step(const action_t *this_action){
	double the_reward = 0.0;
	int force = 0;
	int terminal = 0;

	double angle = 0.0;
	double angleVel = 0.0;
	double vel = 0.0;

	force = this_action->intArray[0];

	pendulum1.apply_force(force);

	angle = pendulum1.get_angle();
	angleVel = pendulum1.get_angleVelocity() + 3;
	vel = pendulum1.get_velocity() + 50;

	terminal = check_terminal(angle, angleVel, vel);
	the_reward = calculate_reward(angle, angleVel, vel);

	this_reward_observation.observation->doubleArray[0] = angle;
	this_reward_observation.observation->doubleArray[1] = angleVel;
	this_reward_observation.observation->doubleArray[2] = vel;

	this_reward_observation.reward = the_reward;
	this_reward_observation.terminal = terminal;
	
	return &this_reward_observation;
}

void env_cleanup(){
	clearRLStruct(&this_observation);
}

const char* env_message(const char* inMessage){
	return inMessage;
}

int calculate_reward(double angle, double angleVel, double vel){
    if((int)angle<=94 && (int)angle>=86 && (int)angleVel==3 && (int)vel==50){
        return 100;
    }
    return 0;
}
  
int check_terminal(double angle, double angleVel, double vel){
	//printf("check terminal angle : %lf, angleVel : %lf, vel : %lf\n",angle, angleVel, vel);
    if((int)angle<=94 && (int)angle>=86 && (int)angleVel==3 && (int)vel==50){
        return 1;
    }
    if((int)angle<=20 || (int)angle>=160){
    	return 1;
    }
    return 0;
}
