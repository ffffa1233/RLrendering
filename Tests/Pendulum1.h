#ifndef PENDULUM1_H
#define PENDULUM1_H

#define DEGTORAD 0.0174532925199432957f
#define RADTODEG 57.295779513082320876f

#include <stdio.h>
//#include <rlglue/RL_glue.h>

extern b2Body* verticalBody;
extern b2Body* horizonBody;
extern b2Body* lineBody;

//extern int fforce;

class Pendulum1: public Test
{
	const char *task_spec;	

	double randomAngle = (rand()%40)+70;
	int success = 0;
	int fail = 0;

	public:
	Pendulum1() 
	{	
		randomAngle = (rand()%40)+70;

		b2BodyDef myBodyDef;
		myBodyDef.type = b2_dynamicBody;

		b2PolygonShape rectangleShape;
		rectangleShape.SetAsBox(10, 0.5);

		b2FixtureDef rectangleFixtureDef;
		rectangleFixtureDef.shape = &rectangleShape;
		rectangleFixtureDef.density = 1;

		myBodyDef.position.Set(0, 14);
		verticalBody = m_world->CreateBody(&myBodyDef);
		verticalBody->CreateFixture(&rectangleFixtureDef);
		
		verticalBody->SetTransform(b2Vec2(9.95*cos(randomAngle*DEGTORAD), 14-(9.95-9.95*sin(randomAngle*DEGTORAD))), randomAngle*DEGTORAD);

		rectangleShape.SetAsBox(6, 2);
		rectangleFixtureDef.shape = &rectangleShape;
		rectangleFixtureDef.density = 3;

		myBodyDef.position.Set(0, 2);
		horizonBody = m_world->CreateBody(&myBodyDef);
		horizonBody->CreateFixture(&rectangleFixtureDef);

		myBodyDef.type = b2_staticBody;
		myBodyDef.position.Set(0, 0);
		
		b2PolygonShape lineShape;
		lineShape.SetAsEdge(b2Vec2(-10000,0), b2Vec2(10000,0) );

		b2FixtureDef lineFixtureDef;
		lineFixtureDef.shape = &lineShape;
		lineFixtureDef.density = 3;

		lineBody = m_world->CreateBody(&myBodyDef);
		lineBody->CreateFixture(&lineFixtureDef);

		b2RevoluteJointDef upperJointDef;
		upperJointDef.localAnchorA.Set(-9.95, 0);
		upperJointDef.localAnchorB.Set(0, 1.95);

		upperJointDef.bodyA = verticalBody;
		upperJointDef.bodyB = horizonBody;

		m_world->CreateJoint(&upperJointDef);

		printf("\nThis is a RL Test Program Pendulum1(Start)\n");
		
	/*	task_spec = RL_init();

		printf("\nTASK SPEC : %s\n",task_spec);
		printf("Starting offline demo\n------------------------\nWill alternate learning for 25 episodes, then freeze policy and evaluate for 10 episodes.\n\n");
		printf("After Episode\tMean Return\tStandard Deviation\n---------------------------------------------------------------------\n");
*/
		//RL_start();

		//RL_agent_message("load_policy results.dat");
		//RL_agent_message("freeze learning");

	}


	void reset(){
		//printf("reset\n");
		randomAngle = (rand()%40)+70;
		
		verticalBody->SetAngularVelocity(0);
		verticalBody->SetLinearVelocity(b2Vec2(0, 0) );
		verticalBody->SetTransform(b2Vec2(9.95*cos(randomAngle*DEGTORAD), 14-(9.95-9.95*sin(randomAngle*DEGTORAD))), randomAngle*DEGTORAD);

		horizonBody->SetLinearVelocity(b2Vec2(0, 0) );
		horizonBody->SetTransform(b2Vec2(0, 2), 0);

		m_world->ClearForces();
		
		//RL_start();
	}

	
	void Step(Settings* settings)
	{
		//run the default physics and rendering
		Test::Step(settings);
		/*
		int isterminal = 0;
		double reward = 0;
		const reward_observation_action_terminal_t *rl_step_result = 0;
		
		rl_step_result = RL_step();
		isterminal = rl_step_result->terminal;
		reward = rl_step_result->reward;
		if(isterminal==1){
			if(reward==100){
				printf("success angle : %lf, %lf\n",randomAngle,verticalBody->GetAngle()*RADTODEG);
				success++;
			}else{
				printf("fail angle : %lf, %lf\n",randomAngle,verticalBody->GetAngle()*RADTODEG);
				fail++;
			}
			reset();
		}*/
		//apply_force(fforce);

		b2Vec2 pos = horizonBody->GetPosition();
		float angle = verticalBody->GetAngle()*RADTODEG;
		b2Vec2 vel = horizonBody->GetLinearVelocity();
		float angularVel = verticalBody->GetAngularVelocity();

		m_debugDraw.DrawString(5, m_textLine, "position:%.3f,%.3f, angle : %.3f, vel : %.3f, angularvel : %.3f ",pos.x, pos.y, angle, vel.x, angularVel );
		m_textLine += 15;

	}

	public: void apply_force(int force)
	{
		//printf("apply force : %d\n",force);
		horizonBody->ApplyForce(b2Vec2(force*5000-50*5000, 0), horizonBody->GetWorldCenter() );
	}

	public: double get_angle(){
		return verticalBody->GetAngle()*RADTODEG;
	}

	public: double get_angleVelocity()
	{
		return verticalBody->GetAngularVelocity();
	}

	public: double get_velocity()
	{
		return horizonBody->GetLinearVelocity().x;
	}

	//return an object of this class to the testbed
	static Test* Create()
	{
		return new Pendulum1;
	}


};

#endif
