#include "environment_pendulum2.h"

#include "Framework/Test.h"
#include "Tests/Pendulum2.h"

Pendulum2 pendulum2;

b2Body* verticalBody;
b2Body* horizonBody;
b2Body* lineBody;

b2Body* verticalBody2;

static observation_t this_observation;
static reward_observation_terminal_t this_reward_observation;

void env_reset(){
	pendulum2.reset();}


const char *env_init(){
	const char *task_spec_string = "[VERSION RL-GLue-3.0] [PROBLEMTYPE episodic] "
									"[OBSERVATIONS angle(-90 90) angle velocity(-50  50) velocity(-50 50) ] "
									"[ACTIONS (-3 3) ] [REWARDS 100 0 ] "
									"[EXTRA environment(C/C++) by PSC, CBH]";

	allocateRLStruct(&this_observation,0,5,0);
	this_reward_observation.observation = &this_observation;
	this_reward_observation.reward = 0;
	this_reward_observation.terminal = 0;

	return task_spec_string;
}

const observation_t *env_start(){
	double angle_footleg, angleVelocity_footleg, velocity_foot;
	double angle_footbody, angleVelocity_footbody;

	angle_footleg = 0.0;
	angleVelocity_footleg = 0.0;
	velocity_foot = 0.0;
	angle_footbody = 0.0;
	angleVelocity_footbody = 0.0;

	angle_footleg = pendulum2.get_angle_footleg();
	angleVelocity_footleg = pendulum2.get_angleVelocity_footleg() + 2;
	velocity_foot = pendulum2.get_velocity_foot() + 7;
	angle_footbody = pendulum2.get_angle_footbody();
	angleVelocity_footbody = pendulum2.get_angleVelocity_footbody() + 2;

	this_observation.doubleArray[0] = angle_footleg/18;
	this_observation.doubleArray[1] = angleVelocity_footleg;
	this_observation.doubleArray[2] = velocity_foot;
	this_observation.doubleArray[3] = angle_footbody/18;
	this_observation.doubleArray[4] = angleVelocity_footbody;

	return &this_observation;
 }

void env_step1(const action_t *this_action){

	int force_foot, force_waist, force_heap;
	force_foot = 0;
	force_waist = 0;
	force_heap = 0;

	force_foot = this_action->intArray[0];
	force_waist = this_action->intArray[1];
	force_heap = this_action->intArray[2];

//printf("foot, waist, heap %d, %d, %d\n",force_foot, force_waist, force_heap);
	pendulum2.apply_force_foot(force_foot);
	pendulum2.apply_force_waist(force_waist);
	pendulum2.apply_force_heap(force_heap);


}

const reward_observation_terminal_t *env_step2() {
	int terminal = 0;
	int the_reward = 0;
	
	double angle_footleg, angleVelocity_footleg, velocity_foot;
	double angle_footbody, angleVelocity_footbody;

	angle_footleg = 0.0;
	angleVelocity_footleg = 0.0;
	velocity_foot = 0.0;
	angle_footbody = 0.0;
	angleVelocity_footbody = 0.0;

	angle_footleg = pendulum2.get_angle_footleg();
	angleVelocity_footleg = pendulum2.get_angleVelocity_footleg() + 2;
	velocity_foot = pendulum2.get_velocity_foot() + 7;
	angle_footbody = pendulum2.get_angle_footbody();
	angleVelocity_footbody = pendulum2.get_angleVelocity_footbody() + 2;

	
	terminal = check_terminal(angle_footleg, angleVelocity_footleg, velocity_foot, 
					angle_footbody, angleVelocity_footbody);
	
	the_reward = calculate_reward(angle_footleg, angleVelocity_footleg, velocity_foot, 
					angle_footbody, angleVelocity_footbody);

	this_reward_observation.observation->doubleArray[0] = angle_footleg/18;
	this_reward_observation.observation->doubleArray[1] = angleVelocity_footleg;
	this_reward_observation.observation->doubleArray[2] = velocity_foot;
	this_reward_observation.observation->doubleArray[3] = angle_footbody/18;
	this_reward_observation.observation->doubleArray[4] = angleVelocity_footbody;

	this_reward_observation.reward = the_reward;
	this_reward_observation.terminal = terminal;

	if(the_reward==100){
		printf("success : %lf, %lf\n",angle_footleg, angle_footbody);
	}
	
	return &this_reward_observation;
}

void env_cleanup(){
	clearRLStruct(&this_observation);
}

const char* env_message(const char* inMessage){
	return inMessage;
}

int calculate_reward(double angle_footleg, double angleVelocity_footleg, double velocity_foot, 
					double angle_footbody, double angleVelocity_footbody){
	
    if( 
    	(int)angle_footleg<=94 && (int)angle_footleg>=86 
    	&&
    	(int)angleVelocity_footleg==2
    	&& 
    	(int)velocity_foot==7
    	&&
    	(int)angle_footbody<=94 && (int)angle_footbody>=86
    	&&
    	(int)angleVelocity_footbody==2 
    ){
        return 100;
    }
    /*
    if(angle_footleg>=80 && angle_footleg<=100 && angle_footbody >=80 && angle_footbody <= 100){
    	return 100;
     }*/
    return 0;
}

int check_terminal(double angle_footleg, double angleVelocity_footleg, double velocity_foot, 
					double angle_footbody, double angleVelocity_footbody){

	if( 
    	(int)angle_footleg<=94 && (int)angle_footleg>=86 
    	&&
    	(int)angleVelocity_footleg==2
    	&& 
    	(int)velocity_foot==7
    	&&
    	(int)angle_footbody<=94 && (int)angle_footbody>=86
    	&&
    	(int)angleVelocity_footbody==2 
    ){
        return 1;
    }
    /*if(angle_footleg>=80 && angle_footleg<=100 && angle_footbody >=80 && angle_footbody <= 100){
    	return 1;
     }*/
    if((int)angle_footleg<=45 || (int)angle_footleg>=135){
    	return 1;
    }
    if((int)angle_footbody<=20 || (int)angle_footbody>=160){
    	return 1;
    }
    return 0;
}
