#ifndef MAIN_H
#define MAIN_H

void SingleStep(int);

int setup_rlglue_network();
void teardown_rlglue_network(int theConnection);
void runEnvironmentEventLoop(int theConnection);
int check_terminal(double current_state);
#endif
