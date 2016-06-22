#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <assert.h>
#include <math.h>
#include <rlglue/Agent_common.h>
#include <rlglue/utils/C/RLStruct_util.h>
#include <rlglue/utils/C/TaskSpec_Parser.h>
#include <rlglue/RL_glue.h>

action_t this_action;
action_t last_action;
observation_t *last_observation = 0;

double* value_function=0;
double sarsa_stepsize = 0.1;
double sarsa_epsilon = 0.5;
double sarsa_gamma = 0.5;

int extends = 1;

int numActions=0;
int numAngles=0;
int numAngleVelocity=0;
int numVelocity=0;

int policy_frozen=0;
int exploring_frozen=0;

int randInRange(int max);
int egreedy(double angle, double angleVelocity, double velocity);
int calculateArrayIndex(double theAngle, double theAngleVelocity, 
						double theVelocity, int theAction);
void save_value_function(const char *fileName);
void load_value_function(const char *fileName);

void agent_init(const char* task_spec){
	allocateRLStruct(&this_action,1,0,0);
	allocateRLStruct(&last_action,1,0,0);

	last_observation=allocateRLStructPointer(0,3,0);

	numActions = 7; // -3 ~ 3
	numAngles = 181*extends; // -90 ~ 90
	numAngleVelocity = 101*extends; // -50 ~ 50
	numVelocity = 101*extends; // -50 ~ 50

	srand(time(0));
	
	value_function=(double *)calloc(numActions*numAngles*
									numAngleVelocity*numVelocity, 
									sizeof(double));
	
}

const action_t *agent_start(const observation_t *this_observation){
	int theIntAction = egreedy(this_observation->doubleArray[0], 
								this_observation->doubleArray[1],
								this_observation->doubleArray[2]);
	this_action.intArray[0] = theIntAction;
	
	replaceRLStruct(&this_action, &last_action);
	replaceRLStruct(this_observation, last_observation);

	return &this_action;
}

const action_t *agent_step(double reward, const observation_t *this_observation){
	double newAngle = this_observation->doubleArray[0];
	double newAngleVelocity = this_observation->doubleArray[1];
	double newVelocity = this_observation->doubleArray[2];

	double lastAngle = last_observation->doubleArray[0];
	double lastAngleVelocity = last_observation->doubleArray[1];
	double lastVelocity = last_observation->doubleArray[2];

	int lastAction = last_action.intArray[0];
	int newAction = egreedy(newAngle, newAngleVelocity, newVelocity);
	
	double Q_sa = value_function[
					calculateArrayIndex(lastAngle,lastAngleVelocity, 
										lastVelocity, lastAction)
					];
	double Q_sprime_aprime = value_function[
							calculateArrayIndex(newAngle, newAngleVelocity, 
												newVelocity, newAction)
							];
	
	double new_Q_sa=Q_sa + sarsa_stepsize * (reward + sarsa_gamma * Q_sprime_aprime - Q_sa);

	if(!policy_frozen){
		if(value_function[calculateArrayIndex(lastAngle,
							lastAngleVelocity,lastVelocity, lastAction)] < new_Q_sa){
			value_function[calculateArrayIndex(lastAngle,
							lastAngleVelocity,lastVelocity, lastAction)] = new_Q_sa;
		}
	}
	this_action.intArray[0]=newAction;

	replaceRLStruct(&this_action, &last_action);
	replaceRLStruct(this_observation, last_observation);

	return &this_action;	
}

void agent_end(double reward){
	double lastAngle = last_observation->doubleArray[0];
	double lastAngleVelocity = last_observation->doubleArray[1];
	double lastVelocity = last_observation->doubleArray[2];

	int lastAction=last_action.intArray[0];

	double Q_sa = value_function[
					calculateArrayIndex(lastAngle,lastAngleVelocity, 
										lastVelocity, lastAction)
					];

	double new_Q_sa=Q_sa + sarsa_stepsize * (reward - Q_sa);
	
	if(!policy_frozen){
		if(value_function[calculateArrayIndex(lastAngle,
							lastAngleVelocity,lastVelocity, lastAction)] < new_Q_sa){
			value_function[calculateArrayIndex(lastAngle,
							lastAngleVelocity,lastVelocity, lastAction)] = new_Q_sa;
		}
	}

	clearRLStruct(&last_action);
	clearRLStruct(last_observation);
}

void agent_cleanup(){
	clearRLStruct(&this_action);
	clearRLStruct(&last_action);
	
	freeRLStructPointer(last_observation);

	if(value_function!=0){
		free(value_function);
		value_function=0;
	}
}

const char* agent_message(const char* inMessage){
	static char buffer[128];

	if(strcmp(inMessage,"freeze learning")==0){printf("agent_message freeze\n");
		policy_frozen=1;
		return "message understood, policy frozen";
	}

	if(strcmp(inMessage,"unfreeze learning")==0){printf("agent_message unfreeze\n");
		policy_frozen=0;
		return "message understood, policy unfrozen";
	}

	if(strcmp(inMessage,"freeze exploring")==0){
		exploring_frozen=1;
		return "message understood, exploring frozen";
	}

	if(strcmp(inMessage,"unfreeze exploring")==0){
		exploring_frozen=0;
		return "message understood, exploring unfrozen";
	}

	if(strncmp(inMessage,"save_policy",11)==0){
		strncpy(buffer,inMessage+12,sizeof(buffer));
		printf("Saving value function...");
		save_value_function(buffer);
		printf("Saved.\n");
		return "message understood, saving policy";
	}

	if(strncmp(inMessage,"load_policy",11)==0){
		strncpy(buffer,inMessage+12,sizeof(buffer));
		printf("Loading value function...");
		load_value_function(buffer);
		printf("Loaded.\n");
		return "message understood, loading policy";
	}
	
	return "agent.c does not understand your message.";
}

void save_value_function(const char *fileName){
	FILE *fp;
	fp=fopen(fileName, "wb");

	fwrite(value_function,sizeof(double),numActions*numAngles*numAngleVelocity*numVelocity,fp);
	fclose(fp);
}

void load_value_function(const char *fileName){
	FILE *fp;
	fp=fopen(fileName, "rb");

	fread(value_function,sizeof(double),numActions*numAngles*numAngleVelocity*numVelocity,fp);
	fclose(fp);
}

int egreedy(double angle, double angleVelocity, double velocity){
	int maxIndex = 0;
	int a = 1;
	int randFrequency = (int)(1.0f/sarsa_epsilon);

	for(a=1;a<numActions;a++){
		if(value_function[calculateArrayIndex(angle, angleVelocity, velocity,a)] > 
			value_function[calculateArrayIndex(angle,angleVelocity, velocity,maxIndex)]){
			maxIndex = a;
		}
	}
	if(value_function[calculateArrayIndex(angle, angleVelocity, velocity,maxIndex)]==0.0){
		return randInRange(numActions-1);
	}
	return maxIndex;
}

int randInRange(int max){
	double r, x;
	r = ((double)rand() / ((double)(RAND_MAX)+(double)(1)));
	x = (r * (max+1));
	return (int)x;
}

int calculateArrayIndex(double theAngle, double theAngleVelocity, 
						double theVelocity, int theAction){
	return ((int)(theAngle*extends))*numAngleVelocity*numVelocity*numActions
			+((int)(theAngleVelocity*extends))*numVelocity*numActions
			+((int)(theVelocity*extends))*numActions+theAction;
}
