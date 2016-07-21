#ifndef CUSTOM_H
#define CUSTOM_H
#include <Box2D/Box2D.h>
#include "../Framework/Test.h"
#include "../Framework/Render.h"
#include "../../freeglut/GL/glut.h"
#include <rlglue/RL_glue.h>
/*
	this is implemented in Pendulum.h
*/
class Pendulum1: public Test
{
private:
	/*b2Body* verticalBody;
	b2Body* horizonBody;
	b2Body* lineBody;*/
	double randomAngle;
public:
	Pendulum1();
	void reset();
	void apply_force(int force);
	double get_angle();
	double get_angleVelocity();
	double get_velocity();
	void Step(Settings* settings);

static Test* Create()
	{
		return new Pendulum1;
	}
};


/*
	this is implemented in Custom_Integrated_Env_Codec.cpp
*/
int setup_rlglue_network();
void teardown_rlglue_network(int theConnection);
void runEnvironmentEventLoop(int theConnection);
#endif
