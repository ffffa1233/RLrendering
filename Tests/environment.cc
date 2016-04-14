#include <string.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>
#include <rlglue/Environment_common.h>
#include <rlglue/utils/C/RLStruct_util.h>
#include <rlglue/utils/C/TaskSpec_Parser.h>
#include <rlglue/RL_glue.h>

#include "../Framework/Test.h"
#include "rltest.h"

static observation_t this_observation;
static reward_observation_terminal_t this_reward_observation;

b2Body *bodies;
rltest rl;

int calculate_reward(double current_state, double current_velocity);
int check_terminal(double current_state, double current_velocity);

const char *env_init(){
	const char *task_spec_string = "[VERSION RL-GLue-3.0] [PROBLEMTYPE episodic] "
									"[OBSERVATIONS DOUBLE (-10.0 ~ 40.0)] "
									"[ACTIONS INT (0,1)] [REWARDS(-5.0 5.0)] "
									"[EXTRA environment(C/C++) by PSC, CBH]";

	allocateRLStruct(&this_observation,0,1,0);
	this_reward_observation.observation = &this_observation;
	this_reward_observation.reward = 0;
	this_reward_observation.terminal = 0;

	return task_spec_string;
}

const observation_t *env_start(){
	double posx = 0.0;
	double vel = 0.0;
	posx = rl.get_position();
	vel = rl.get_velocity()+50;

	this_observation.doubleArray[0] = posx;
	this_observation.doubleArray[1] = vel;

	return &this_observation; 
}

const reward_observation_terminal_t *env_step(const action_t *this_action){
	double the_reward = 0.0;
	int force = 0;
	double posx = 0.0;
	double vel = 0.0;
	int terminal = 0;

	force = this_action->intArray[0];

	rl.apply_force(force);
	posx = rl.get_position(); 
	vel = rl.get_velocity();
	vel += 50;
	terminal = check_terminal(posx, vel);
	
	printf("force %d, posx %lf, vel %lf, terminal %d\n",force, posx, vel, terminal);
	
	the_reward = calculate_reward(posx, vel);
	this_reward_observation.observation->doubleArray[0] = posx;
	this_reward_observation.observation->doubleArray[1] = vel;
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

int calculate_reward(double current_state, double current_velocity){
    if((int)current_state==45 && (int)current_velocity==50){
        return 100;
    }
    return 0;
}
  
int check_terminal(double current_state, double current_velocity){
    if((int)current_state==45 && (int)current_velocity==50){
        return 1;
    }
    if(current_state > 51.0 || current_state < 0.0){
        return 1;
    }
    return 0;
}
