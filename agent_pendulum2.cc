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

double* value_function_foot = 0;
double* value_function_waist = 0;
double* value_function_heap = 0;

double sarsa_stepsize = 0.9;
double sarsa_epsilon = 0.5;
double sarsa_gamma = 0.5;

int extends = 1;

int numActions_foot = 0;
int numActions_waist = 0;
int numActions_heap = 0;

int numAngles_footleg = 0;
int numAngles_footbody = 0;
int numAngleVelocity_footleg = 0;
int numAngleVelocity_footbody = 0;
int numVelocity_foot = 0;

int policy_frozen=0;
int exploring_frozen=0;

int randInRange(int max);

int egreedy(double angle_footleg, double angleVelocity_footleg, double velocity_foot, double angle_footbody, double angleVelocity_footbody, int tmp);

int calculateArrayIndex_foot(double angle_footleg, double angleVelocity_footleg, 
							double velocity_foot, double angle_footbody, double angleVelocity_footbody, int action);
int calculateArrayIndex_waist(double angle_footleg, double angleVelocity_footleg, 
							double velocity_foot, double angle_footbody, double angleVelocity_footbody, int action);
int calculateArrayIndex_heap(double angle_footleg, double angleVelocity_footleg, 
							double velocity_foot, double angle_footbody, double angleVelocity_footbody, int action);

void save_value_function(const char *fileName);
void load_value_function(const char *fileName);

void agent_init(const char* task_spec){
	allocateRLStruct(&this_action,3,0,0);
	allocateRLStruct(&last_action,3,0,0);

	last_observation=allocateRLStructPointer(0,7,0);

	numActions_foot = 21; // -10 ~ 10
	numActions_waist = 23; // -10 ~ 10
	numActions_heap = 19; // -10 ~ 10

	numAngles_footleg = 11*extends; // -90 ~ 90
	numAngles_footbody = 11*extends; // -90 ~ 90

	numAngleVelocity_footleg = 5*extends; // -2 ~ 2
	numAngleVelocity_footbody = 5*extends; // -2 ~ 2

	numVelocity_foot = 15*extends; // -7 ~ 7

	srand(time(0));
	
	value_function_foot = (double*)calloc(numActions_foot*
								numAngles_footleg*numAngleVelocity_footleg*
								numAngles_footbody*numAngleVelocity_footbody*
								numVelocity_foot, 
								sizeof(double));
	value_function_waist = (double*)calloc(numActions_waist*
								numAngles_footleg*numAngleVelocity_footleg*
								numAngles_footbody*numAngleVelocity_footbody*
								numVelocity_foot, 
								sizeof(double));
	value_function_heap = (double*)calloc(numActions_heap*
								numAngles_footleg*numAngleVelocity_footleg*
								numAngles_footbody*numAngleVelocity_footbody*
								numVelocity_foot, 
								sizeof(double));

	printf("num : %d\n",numActions_heap*
								numAngles_footleg*numAngleVelocity_footleg*
								numAngles_footbody*numAngleVelocity_footbody*
								numVelocity_foot);
}

const action_t *agent_start(const observation_t *this_observation){

/*
	printf("%lf, %lf, %lf, %lf, %lf\n",
		this_observation->doubleArray[0],this_observation->doubleArray[1],this_observation->doubleArray[2],
		this_observation->doubleArray[3],this_observation->doubleArray[4]);
	*/

	int theIntAction_foot = egreedy(this_observation->doubleArray[0], 
								this_observation->doubleArray[1],
								this_observation->doubleArray[2],
								this_observation->doubleArray[3],
								this_observation->doubleArray[4],
								numActions_foot
								);
	int theIntAction_waist = egreedy(this_observation->doubleArray[0], 
								this_observation->doubleArray[1],
								this_observation->doubleArray[2],
								this_observation->doubleArray[3],
								this_observation->doubleArray[4],
								numActions_waist
								);
	int theIntAction_heap = egreedy(this_observation->doubleArray[0], 
								this_observation->doubleArray[1],
								this_observation->doubleArray[2],
								this_observation->doubleArray[3],
								this_observation->doubleArray[4],
								numActions_heap
								);

	this_action.intArray[0] = theIntAction_foot;
	this_action.intArray[1] = theIntAction_waist;
	this_action.intArray[2] = theIntAction_heap;

	replaceRLStruct(&this_action, &last_action);
	replaceRLStruct(this_observation, last_observation);

	return &this_action;
}

const action_t *agent_step(double reward, const observation_t *this_observation){
	double newAngle_footleg, newAngleVelocity_footleg, newVelocity_foot;
	double newAngle_footbody, newAngleVelocity_footbody;

printf("agent step %lf, %lf, %lf, %lf, %lf\n",
		this_observation->doubleArray[0],this_observation->doubleArray[1],this_observation->doubleArray[2],
		this_observation->doubleArray[3],this_observation->doubleArray[4]);

	newAngle_footleg = this_observation->doubleArray[0];
	newAngleVelocity_footleg = this_observation->doubleArray[1];
	newVelocity_foot = this_observation->doubleArray[2];
	newAngle_footbody = this_observation->doubleArray[3];
	newAngleVelocity_footbody = this_observation->doubleArray[4];

	double lastAngle_footleg, lastAngleVelocity_footleg, lastVelocity_foot;
	double lastAngle_footbody, lastAngleVelocity_footbody;

	lastAngle_footleg = last_observation->doubleArray[0];
	lastAngleVelocity_footleg = last_observation->doubleArray[1];
	lastVelocity_foot = last_observation->doubleArray[2];
	lastAngle_footbody = last_observation->doubleArray[3];
	lastAngleVelocity_footbody = last_observation->doubleArray[4];

	int lastAction_foot, lastAction_waist, lastAction_heap;
	int newAction_foot, newAction_waist, newAction_heap;

	lastAction_foot = last_action.intArray[0];
	lastAction_waist = last_action.intArray[1];
	lastAction_heap = last_action.intArray[2];
printf("<<<\n");
	newAction_foot = egreedy(newAngle_footleg, newAngleVelocity_footleg, newVelocity_foot, newAngle_footbody, newAngleVelocity_footbody, numActions_foot);
	printf("<<<2222\n");
	newAction_waist = egreedy(newAngle_footleg, newAngleVelocity_footleg, newVelocity_foot, newAngle_footbody, newAngleVelocity_footbody, numActions_waist);
printf("<<<33333\n");
	newAction_heap = egreedy(newAngle_footleg, newAngleVelocity_footleg, newVelocity_foot, newAngle_footbody, newAngleVelocity_footbody, numActions_heap);

	double Q_sa_foot, Q_sa_waist, Q_sa_heap;
	double Q_sprime_aprime_foot, Q_sprime_aprime_waist, Q_sprime_aprime_heap;
	double new_Q_sa_foot, new_Q_sa_waist, new_Q_sa_heap;
int tes = calculateArrayIndex_foot(lastAngle_footleg, lastAngleVelocity_footleg, lastVelocity_foot, lastAngle_footbody, lastAngleVelocity_footbody, lastAction_foot);
printf("tes   %d\n",tes);
	Q_sa_foot = value_function_foot[
				calculateArrayIndex_foot(lastAngle_footleg, lastAngleVelocity_footleg, lastVelocity_foot, lastAngle_footbody, lastAngleVelocity_footbody, lastAction_foot)
			];
	Q_sprime_aprime_foot = value_function_foot[
				calculateArrayIndex_foot(lastAngle_footleg, lastAngleVelocity_footleg, lastVelocity_foot, lastAngle_footbody, lastAngleVelocity_footbody, lastAction_foot)
		];
	new_Q_sa_foot = Q_sa_foot + sarsa_stepsize * (reward + sarsa_gamma * Q_sprime_aprime_foot - Q_sa_foot);
	
	if(!policy_frozen){
		if(value_function_foot[calculateArrayIndex_foot(lastAngle_footleg, lastAngleVelocity_footleg, lastVelocity_foot, lastAngle_footbody, lastAngleVelocity_footbody, lastAction_foot)] < new_Q_sa_foot){
			value_function_foot[calculateArrayIndex_foot(lastAngle_footleg, lastAngleVelocity_footleg, lastVelocity_foot, lastAngle_footbody, lastAngleVelocity_footbody, lastAction_foot)] = new_Q_sa_foot;
		}
	}
	this_action.intArray[0] = newAction_foot;
//int te = calculateArrayIndex_waist(lastAngle_footleg, lastAngleVelocity_footleg, lastVelocity_foot, lastAngle_footbody, lastAngleVelocity_footbody, lastAction_waist);
//printf("te %d\n",te);
	Q_sa_waist = value_function_waist[
				calculateArrayIndex_waist(lastAngle_footleg, lastAngleVelocity_footleg, lastVelocity_foot, lastAngle_footbody, lastAngleVelocity_footbody, lastAction_waist)
			];
	Q_sprime_aprime_waist = value_function_waist[
				calculateArrayIndex_waist(lastAngle_footleg, lastAngleVelocity_footleg, lastVelocity_foot, lastAngle_footbody, lastAngleVelocity_footbody, lastAction_waist)
		];
	new_Q_sa_waist = Q_sa_waist + sarsa_stepsize * (reward + sarsa_gamma * Q_sprime_aprime_waist - Q_sa_waist);
	
	if(!policy_frozen){
		if(value_function_waist[calculateArrayIndex_waist(lastAngle_footleg, lastAngleVelocity_footleg, lastVelocity_foot, lastAngle_footbody, lastAngleVelocity_footbody, lastAction_waist)] < new_Q_sa_waist){
			value_function_waist[calculateArrayIndex_waist(lastAngle_footleg, lastAngleVelocity_footleg, lastVelocity_foot, lastAngle_footbody, lastAngleVelocity_footbody, lastAction_waist)] = new_Q_sa_waist;
		}
	}
	this_action.intArray[1] = newAction_waist;

	Q_sa_heap = value_function_heap[
				calculateArrayIndex_heap(lastAngle_footleg, lastAngleVelocity_footleg, lastVelocity_foot, lastAngle_footbody, lastAngleVelocity_footbody, lastAction_heap)
			];
	Q_sprime_aprime_heap = value_function_heap[
				calculateArrayIndex_heap(lastAngle_footleg, lastAngleVelocity_footleg, lastVelocity_foot, lastAngle_footbody, lastAngleVelocity_footbody, newAction_heap)
		];
	new_Q_sa_heap = Q_sa_heap + sarsa_stepsize * (reward + sarsa_gamma * Q_sprime_aprime_heap - Q_sa_heap);
	
	if(!policy_frozen){
		if(value_function_heap[calculateArrayIndex_heap(lastAngle_footleg, lastAngleVelocity_footleg, lastVelocity_foot, lastAngle_footbody, lastAngleVelocity_footbody, newAction_heap)] < new_Q_sa_heap){
			value_function_heap[calculateArrayIndex_heap(lastAngle_footleg, lastAngleVelocity_footleg, lastVelocity_foot, lastAngle_footbody, lastAngleVelocity_footbody, newAction_heap)] = new_Q_sa_heap;
		}
	}
	this_action.intArray[2] = newAction_heap;

	replaceRLStruct(&this_action, &last_action);
	replaceRLStruct(this_observation, last_observation);

	return &this_action;	
}

void agent_end(double reward){
	double lastAngle_footleg, lastAngleVelocity_footleg, lastVelocity_foot;
	double lastAngle_footbody, lastAngleVelocity_footbody;

	lastAngle_footleg = last_observation->doubleArray[0];
	lastAngleVelocity_footleg = last_observation->doubleArray[1];
	lastVelocity_foot = last_observation->doubleArray[2];
	lastAngle_footbody = last_observation->doubleArray[3];
	lastAngleVelocity_footbody = last_observation->doubleArray[4];

	int lastAction_foot, lastAction_waist, lastAction_heap;

	lastAction_foot = last_action.intArray[0];
	lastAction_waist = last_action.intArray[1];
	lastAction_heap = last_action.intArray[2];

	double Q_sa_foot, Q_sa_waist, Q_sa_heap;
	double new_Q_sa_foot, new_Q_sa_waist, new_Q_sa_heap;

	Q_sa_foot = value_function_foot[
				calculateArrayIndex_foot(lastAngle_footleg, lastAngleVelocity_footleg, lastVelocity_foot, lastAngle_footbody, lastAngleVelocity_footbody, lastAction_foot)
			];

	new_Q_sa_foot = Q_sa_foot + sarsa_stepsize * (reward - Q_sa_foot);
	
	if(!policy_frozen){
		if(value_function_foot[calculateArrayIndex_foot(lastAngle_footleg, lastAngleVelocity_footleg, lastVelocity_foot, lastAngle_footbody, lastAngleVelocity_footbody, lastAction_foot)] < new_Q_sa_foot){
			value_function_foot[calculateArrayIndex_foot(lastAngle_footleg, lastAngleVelocity_footleg, lastVelocity_foot, lastAngle_footbody, lastAngleVelocity_footbody, lastAction_foot)] = new_Q_sa_foot;
		}
	}

	Q_sa_waist = value_function_waist[
				calculateArrayIndex_waist(lastAngle_footleg, lastAngleVelocity_footleg, lastVelocity_foot, lastAngle_footbody, lastAngleVelocity_footbody, lastAction_waist)
			];
	
	new_Q_sa_waist = Q_sa_waist + sarsa_stepsize * (reward - Q_sa_waist);
	
	if(!policy_frozen){
		if(value_function_waist[calculateArrayIndex_waist(lastAngle_footleg, lastAngleVelocity_footleg, lastVelocity_foot, lastAngle_footbody, lastAngleVelocity_footbody, lastAction_waist)] < new_Q_sa_waist){
			value_function_waist[calculateArrayIndex_waist(lastAngle_footleg, lastAngleVelocity_footleg, lastVelocity_foot, lastAngle_footbody, lastAngleVelocity_footbody, lastAction_waist)] = new_Q_sa_waist;
		}
	}

	Q_sa_heap = value_function_heap[
				calculateArrayIndex_heap(lastAngle_footleg, lastAngleVelocity_footleg, lastVelocity_foot, lastAngle_footbody, lastAngleVelocity_footbody, lastAction_heap)
			];
	
	new_Q_sa_heap = Q_sa_heap + sarsa_stepsize * (reward + sarsa_gamma - Q_sa_heap);
	
	if(!policy_frozen){
		if(value_function_heap[calculateArrayIndex_heap(lastAngle_footleg, lastAngleVelocity_footleg, lastVelocity_foot, lastAngle_footbody, lastAngleVelocity_footbody, lastAction_heap)] < new_Q_sa_heap){
			value_function_heap[calculateArrayIndex_heap(lastAngle_footleg, lastAngleVelocity_footleg, lastVelocity_foot, lastAngle_footbody, lastAngleVelocity_footbody, lastAction_heap)] = new_Q_sa_heap;
		}
	}

	clearRLStruct(&last_action);
	clearRLStruct(last_observation);
}

void agent_cleanup(){
	clearRLStruct(&this_action);
	clearRLStruct(&last_action);
	
	freeRLStructPointer(last_observation);

	if(value_function_foot!=0){
		free(value_function_foot);
		value_function_foot=0;
	}
	if(value_function_waist!=0){
		free(value_function_waist);
		value_function_waist=0;
	}
	if(value_function_heap!=0){
		free(value_function_heap);
		value_function_heap=0;
	}
}

const char* agent_message(const char* inMessage){
	static char buffer[128];

	if(strcmp(inMessage,"freeze learning")==0){printf("agent_message freeze\n");
		policy_frozen=1;
		return "message understood, policy frozen";
	}

	if(strcmp(inMessage,"unfreeze learning")==0){//printf("agent_message unfreeze\n");
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

	fwrite(value_function_foot, sizeof(double), numActions_heap*
								numAngles_footleg*numAngleVelocity_footleg*
								numAngles_footbody*numAngleVelocity_footbody*
								numVelocity_foot, fp);
	fwrite(value_function_waist, sizeof(double), numActions_heap*
								numAngles_footleg*numAngleVelocity_footleg*
								numAngles_footbody*numAngleVelocity_footbody*
								numVelocity_foot, fp);
	fwrite(value_function_heap, sizeof(double), numActions_heap*
								numAngles_footleg*numAngleVelocity_footleg*
								numAngles_footbody*numAngleVelocity_footbody*
								numVelocity_foot, fp);
	fclose(fp);
}

void load_value_function(const char *fileName){
	FILE *fp;
	fp=fopen(fileName, "rb");

	fread(value_function_foot, sizeof(double), numActions_heap*
								numAngles_footleg*numAngleVelocity_footleg*
								numAngles_footbody*numAngleVelocity_footbody*
								numVelocity_foot, fp);
	fread(value_function_waist, sizeof(double), numActions_heap*
								numAngles_footleg*numAngleVelocity_footleg*
								numAngles_footbody*numAngleVelocity_footbody*
								numVelocity_foot, fp);
	fread(value_function_heap, sizeof(double), numActions_heap*
								numAngles_footleg*numAngleVelocity_footleg*
								numAngles_footbody*numAngleVelocity_footbody*
								numVelocity_foot, fp);
	fclose(fp);
}

int egreedy(double angle_footleg, double angleVelocity_footleg, double velocity_foot, double angle_footbody, double angleVelocity_footbody, int tmp){
	int maxIndex = 0;
	int a = 1;
	int randFrequency = (int)(1.0f/sarsa_epsilon);
printf("tmp %d\n",tmp);
	if(tmp == numActions_foot){
		for(a=1;a<tmp;a++){//printf("cal  %d\n",calculateArrayIndex_foot(angle_footleg, angleVelocity_footleg, velocity_foot, angle_footbody, angleVelocity_footbody, a));
			if(value_function_foot[calculateArrayIndex_foot(angle_footleg, angleVelocity_footleg, velocity_foot, angle_footbody, angleVelocity_footbody, a)] > 
				value_function_foot[calculateArrayIndex_foot(angle_footleg, angleVelocity_footleg, velocity_foot, angle_footbody, angleVelocity_footbody, maxIndex)]){
				maxIndex = a;
			}
		}
		if(value_function_foot[calculateArrayIndex_foot(angle_footleg, angleVelocity_footleg, velocity_foot, angle_footbody, angleVelocity_footbody, maxIndex)]==0.0){
			return randInRange(tmp-1);
		}
	}else if(tmp == numActions_waist){
		for(a=1;a<tmp;a++){
			if(value_function_foot[calculateArrayIndex_waist(angle_footleg, angleVelocity_footleg, velocity_foot, angle_footbody, angleVelocity_footbody, a)] > 
				value_function_foot[calculateArrayIndex_waist(angle_footleg, angleVelocity_footleg, velocity_foot, angle_footbody, angleVelocity_footbody, maxIndex)]){
				maxIndex = a;
			}
		}
		if(value_function_foot[calculateArrayIndex_waist(angle_footleg, angleVelocity_footleg, velocity_foot, angle_footbody, angleVelocity_footbody, maxIndex)]==0.0){
			return randInRange(tmp-1);
		}
	}else if(tmp == numActions_heap){
		for(a=1;a<tmp;a++){
			if(value_function_heap[calculateArrayIndex_heap(angle_footleg, angleVelocity_footleg, velocity_foot, angle_footbody, angleVelocity_footbody, a)] > 
				value_function_heap[calculateArrayIndex_heap(angle_footleg, angleVelocity_footleg, velocity_foot, angle_footbody, angleVelocity_footbody, maxIndex)]){
				maxIndex = a;
			}
		}
		if(value_function_heap[calculateArrayIndex_heap(angle_footleg, angleVelocity_footleg, velocity_foot, angle_footbody, angleVelocity_footbody, maxIndex)]==0.0){
			return randInRange(tmp-1);
		}
	}

	return maxIndex;
}

int randInRange(int max){
	double r, x;
	r = ((double)rand() / ((double)(RAND_MAX)+(double)(1)));
	x = (r * (max+1));
	return (int)x;
}

int calculateArrayIndex_foot(double angle_footleg, double angleVelocity_footleg, 
							double velocity_foot, double angle_footbody, double angleVelocity_footbody, int action){
	printf("cal foot %lf, %lf, %lf, %lf, %lf,%d\n",angle_footleg, angleVelocity_footleg, velocity_foot, angle_footbody, angleVelocity_footbody, action);
	return ((int)(angle_footleg*extends))*numAngleVelocity_footleg*numVelocity_foot*numAngles_footbody*numAngleVelocity_footbody*numActions_foot
			+ ((int)(angleVelocity_footleg*extends))*numVelocity_foot*numAngles_footbody*numAngleVelocity_footbody*numActions_foot
			+((int)(velocity_foot*extends))*numAngles_footbody*numAngleVelocity_footbody*numActions_foot
			+((int)(angle_footbody*extends))*numAngleVelocity_footbody*numActions_foot
			+((int)(angleVelocity_footbody*extends))*numActions_foot
			+action;
}

int calculateArrayIndex_waist(double angle_footleg, double angleVelocity_footleg, 
							double velocity_foot, double angle_footbody, double angleVelocity_footbody, int action){
	return ((int)(angle_footleg*extends))*numAngleVelocity_footleg*numVelocity_foot*numAngles_footbody*numAngleVelocity_footbody*numActions_waist
			+ ((int)(angleVelocity_footleg*extends))*numVelocity_foot*numAngles_footbody*numAngleVelocity_footbody*numActions_waist
			+((int)(velocity_foot*extends))*numAngles_footbody*numAngleVelocity_footbody*numActions_waist
			+((int)(angle_footbody*extends))*numAngleVelocity_footbody*numActions_waist
			+((int)(angleVelocity_footbody*extends))*numActions_waist
			+action;
}

int calculateArrayIndex_heap(double angle_footleg, double angleVelocity_footleg, 
							double velocity_foot, double angle_footbody, double angleVelocity_footbody, int action){
	return ((int)(angle_footleg*extends))*numAngleVelocity_footleg*numVelocity_foot*numAngles_footbody*numAngleVelocity_footbody*numActions_heap
			+ ((int)(angleVelocity_footleg*extends))*numVelocity_foot*numAngles_footbody*numAngleVelocity_footbody*numActions_heap
			+((int)(velocity_foot*extends))*numAngles_footbody*numAngleVelocity_footbody*numActions_heap
			+((int)(angle_footbody*extends))*numAngleVelocity_footbody*numActions_heap
			+((int)(angleVelocity_footbody*extends))*numActions_heap
			+action;
}