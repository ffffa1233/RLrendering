#include <assert.h> /* assert  */
#include <unistd.h> /* sleep   */
#include <string.h> /* strlen */ /* I'm sorry about using strlen. */
#include <stdio.h>  /* fprintf */
#include <stdlib.h> /* calloc, getenv, exit */

#include <ctype.h> /* isdigit */
#include <netdb.h> /* gethostbyname */
#include <arpa/inet.h> /* inet_ntoa */

#include <rlglue/Environment_common.h>
#include <rlglue/network/RL_network.h>

/* Include the utility methods*/
#include <rlglue/utils/C/RLStruct_util.h>

//////////////////////////////////
/* Our project specific include */
#include "Custom.h"

extern double angle;
extern double angularVel;
extern double vel;

Pendulum1 pendulum1;
///////////////////////////

static const char* kUnknownMessage = "Unknown Message: %s\n";

static action_t theAction                 = {0};
static rlBuffer theBuffer                 = {0};
static char* theInMessage = 0;
static unsigned int theInMessageCapacity = 0;

/*Added as a global variable because now we must allocate and fill
up the data structures instead of the environment.*/
static observation_t globalObservation		  = {0};

static void onEnvInit(int theConnection) {
 /* CUT-FOR-CUSTOMIZATION:  char* theTaskSpec = 0;*/
  unsigned int theTaskSpecLength = 0;
  unsigned int offset = 0;

   /* CUT-FOR-CUSTOMIZATION: theTaskSpec = env_init();*/

  /* You could give a real RL-Glue task spec or a custom one*/
  /* TheGame conveniently is the same as SkeletonEnvironment.c,
	 our other example environment*/
  char* theTaskSpec = "VERSION RL-Glue-3.0 PROBLEMTYPE episodic TESTTESTTEST"
							 "DISCOUNTFACTOR 1 OBSERVATIONS INTS (0 107) "
							 "ACTIONS INTS (0 3)  REWARDS (-100.0 10.0) "
							"EXTRA SampleMinesEnvironment(C/C++) by TESTPSC";

  if (theTaskSpec != NULL) {
    theTaskSpecLength = strlen(theTaskSpec);
  }

  /* Prepare the buffer for sending data back to the server */
  rlBufferClear(&theBuffer);
  offset = rlBufferWrite(&theBuffer, offset, &theTaskSpecLength, 1, sizeof(int));
  if (theTaskSpecLength > 0) {
    offset = rlBufferWrite(&theBuffer, offset, theTaskSpec, theTaskSpecLength, sizeof(char));
  }
}

static void onEnvStart(int theConnection) {
   	/* CUT-FOR-CUSTOMIZATION: observation_t globalObservation = {0}; */
  	unsigned int offset = 0;

 	/* CUT-FOR-CUSTOMIZATION: globalObservation=env_start(); */


/////////////////////////////////////////////
	/* Allocate space to store the observation (angle, angleVel, vel)*/
	allocateRLStruct(&globalObservation,0,3,0);

	globalObservation.doubleArray[0] = angle;
  globalObservation.doubleArray[1] = angularVel + 3;
  globalObservation.doubleArray[2] = vel + 50;
  ///////////////////////////////////////


	__RL_CHECK_STRUCT(&globalObservation)
  rlBufferClear(&theBuffer);
  offset = rlCopyADTToBuffer(&globalObservation, &theBuffer, offset);
}

static void onEnvStep(int theConnection) {
	static reward_observation_terminal_t ro = {0};
	unsigned int offset = 0;
	/* Create an integer variable to hold the action from the agent*/
  	int theIntAction=0;
	ro.terminal=0;
  ro.reward=0;

	offset = rlCopyBufferToADT(&theBuffer, offset, &theAction);
 	__RL_CHECK_STRUCT(&theAction);

	/*I know to only expect 1 integer action*/
	theIntAction=theAction.intArray[0];

	/*This is our hook into TheGame */
	pendulum1.apply_force(theIntAction);
	
  	/* CUT-FOR-CUSTOMIZATION: ro = env_step(theAction);	*/
  
	/************************** ALL NEW CODE HERE **************************/
	/* Allocate space to store the observation (angle, angleVel, vel)*/
	allocateRLStruct(&globalObservation,0,3,0);
	/* Get the int observation from a global variable we
								extern'd from TheGame.c */
	globalObservation.doubleArray[0] = angle;
  globalObservation.doubleArray[1] = angularVel + 3;
  globalObservation.doubleArray[2] = vel + 50;

	/* TheGame doesn't know about rewards, and it doesn't
		have a "terminal" flag, so we have to write code
					   to make TheGame fit with RL-Glue*/
  if((int)angle<=94 && (int)angle>=86 && (int)angularVel==3 && (int)vel==50){
      ro.reward = 100;
      ro.terminal = 1;
  }
  if((int)angle<=20 || (int)angle>=160){
      ro.reward = 0;
      ro.terminal = 1;
  }

	ro.observation =&globalObservation;

	/************************** NEW CODE OVER **************************/
	
  __RL_CHECK_STRUCT(ro.observation)


  rlBufferClear(&theBuffer);
  offset = 0;
  offset = rlBufferWrite(&theBuffer, offset, &ro.terminal, 1, sizeof(int));
  offset = rlBufferWrite(&theBuffer, offset, &ro.reward, 1, sizeof(double));
  offset = rlCopyADTToBuffer(ro.observation, &theBuffer, offset);
}

static void onEnvCleanup(int theConnection) {
	/*No game specific cleanup to do*/
	/* CUT-FOR-CUSTOMIZATION: env_cleanup();*/
  pendulum1.reset();

	rlBufferClear(&theBuffer);
	
	/* Clean up globalObservation global we created*/
	clearRLStruct(&globalObservation);

	clearRLStruct(&theAction);
	
	/*It's ok to free null pointers, so this is safe */
	free(theInMessage);
	theInMessage = 0;
	theInMessageCapacity = 0;
}


static void onEnvMessage(int theConnection) {
  unsigned int inMessageLength = 0;
  unsigned int outMessageLength = 0;
  char* inMessage = 0;
  /*We set this to a string constant instead of null*/
  char* outMessage = "sample custom codec integration has no messages!";
  unsigned int offset = 0;

  offset = 0;
  offset = rlBufferRead(&theBuffer, offset, &inMessageLength, 1, sizeof(int));
  if (inMessageLength >= theInMessageCapacity) {
    inMessage = (char*)calloc(inMessageLength+1, sizeof(char));
    free(theInMessage);

    theInMessage = inMessage;
    theInMessageCapacity = inMessageLength;
  }

  if (inMessageLength > 0) {
    offset = rlBufferRead(&theBuffer, offset, theInMessage, inMessageLength, sizeof(char));
  }
/*Make sure to null terminate the string */
   theInMessage[inMessageLength]='\0';

	/* CUT-FOR-CUSTOMIZATION: outMessage = env_message(theInMessage);*/

  if (outMessage != NULL) {
   outMessageLength = strlen(outMessage);
  }

  
  /* we want to start sending, so we're going to reset the offset to 0 so we write the the beginning of the buffer */
  rlBufferClear(&theBuffer);
  offset = 0;
  offset = rlBufferWrite(&theBuffer, offset, &outMessageLength, 1, sizeof(int));
  if (outMessageLength > 0) {
    offset = rlBufferWrite(&theBuffer, offset, outMessage, outMessageLength, sizeof(char));
  }
}

void runEnvironmentEventLoop(int theConnection) {
  int envState = 0;

  do { 
    rlBufferClear(&theBuffer);
    rlRecvBufferData(theConnection, &theBuffer, &envState);

    switch(envState) {
    case kEnvInit:
      onEnvInit(theConnection);
      break;

    case kEnvStart:
      onEnvStart(theConnection);
      break;

    case kEnvStep:
      onEnvStep(theConnection);
      break;

    case kEnvCleanup:
      onEnvCleanup(theConnection);
      break;

    case kEnvMessage:
      onEnvMessage(theConnection);
      break;

    case kRLTerm:
      break;

    default:
      fprintf(stderr, kUnknownMessage, envState);
      exit(0);
      break;
    };

    rlSendBufferData(theConnection, &theBuffer, envState);
  } while (envState != kRLTerm);
}

/*This used to be the main method, I've renamed it and cut a bunch of stuff out*/
int setup_rlglue_network() {
	int theConnection = 0;

	char* host = kLocalHost;
	short port = kDefaultPort;

	printf("RL-Glue sample env custom codec integration.\n");

	/* Allocate what should be plenty of space for the buffer - it will dynamically resize if it is too small */
	rlBufferCreate(&theBuffer, 4096);

	theConnection = rlWaitForConnection(host, port, kRetryTimeout);

	printf("\tSample custom env codec :: Connected\n");
	rlBufferClear(&theBuffer);
	rlSendBufferData(theConnection, &theBuffer, kEnvironmentConnection);

	return theConnection;
}

void teardown_rlglue_network(int theConnection){
	rlClose(theConnection);
	rlBufferDestroy(&theBuffer);
}