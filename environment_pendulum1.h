#include <string.h>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>

#include <rlglue/utils/C/RLStruct_util.h>
#include <rlglue/utils/C/TaskSpec_Parser.h>
#include <rlglue/RL_glue.h>




int calculate_reward(double angle, double angleVel, double vel);
int check_terminal(double angle, double angleVel, double vel);

void env_reset();

const char *env_init();
const observation_t *env_start();
void env_step1(const action_t *this_action);
const reward_observation_terminal_t *env_step2();
void env_cleanup();
const char* env_message(const char* inMessage);