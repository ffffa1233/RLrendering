#ifndef PENDULUM1_H
#define PENDULUM1_H

#include <stdio.h>
#include <rlglue/RL_glue.h>

class Pendulum1: public Test
{
	const char *task_spec;

	b2Body* verticalBody;
	b2Body* horizonBody;
	b2Body* lineBody;

	double init_position = 35.0;
	int success = 0;
	int fail = 0;

	public:
	Pendulum1() 
	{	
		b2BodyDef myBodyDef;
		myBodyDef.type = b2_dynamicBody;

		b2PolygonShape rectangleShape;
		rectangleShape.SetAsBox(0.5, 10);

		b2FixtureDef rectangleFixtureDef;
		rectangleFixtureDef.shape = &rectangleShape;
		rectangleFixtureDef.density = 3;

		myBodyDef.position.Set(0, 15);
		verticalBody = m_world->CreateBody(&myBodyDef);
		verticalBody->CreateFixture(&rectangleFixtureDef);

		rectangleShape.SetAsBox(6, 2);
		rectangleFixtureDef.shape = &rectangleShape;
		rectangleFixtureDef.density = 3;

		myBodyDef.position.Set(0, 3);
		horizonBody = m_world->CreateBody(&myBodyDef);
		horizonBody->CreateFixture(&rectangleFixtureDef);

		myBodyDef.type = b2_staticBody;
		myBodyDef.position.Set(0, 0);
		
		b2PolygonShape lineShape;
		lineShape.SetAsEdge(b2Vec2(-10000,0), b2Vec2(10000,0) );

		b2FixtureDef lineFixtureDef;
		lineFixtureDef.shape = &lineShape;
		lineFixtureDef.density = 1;

		lineBody = m_world->CreateBody(&myBodyDef);
		lineBody->CreateFixture(&lineFixtureDef);

		b2RevoluteJointDef upperJointDef;
		upperJointDef.localAnchorA.Set(0, -9.95);
		upperJointDef.localAnchorB.Set(0, 1.95);
		upperJointDef.lowerAngle = -90 * DEGTORAD;
		upperJointDef.upperAngle = 90 * DEGTORAD;
		upperJointDef.enableLimit = true;
		upperJointDef.bodyA = verticalBody;
		upperJointDef.bodyB = horizonBody;
		
		m_world->CreateJoint(&upperJointDef);

		printf("\nThis is a RL Test Program Pendulum1(Start)\n");
		
		task_spec = RL_init();

		printf("\nTASK SPEC : %s\n",task_spec);
		printf("Starting offline demo\n------------------------\nWill alternate learning for 25 episodes, then freeze policy and evaluate for 10 episodes.\n\n");
		printf("After Episode\tMean Return\tStandard Deviation\n---------------------------------------------------------------------\n");

		RL_start();

		RL_agent_message("load_policy results.dat");
		RL_agent_message("freeze learning");

	}


	void reset(){

		verticalBody->SetAngularVelocity(0);
		verticalBody->SetLinearVelocity(b2Vec2(0, 0) );
		verticalBody->SetTransform(b2Vec2(0, 15), 0);

		horizonBody->SetLinearVelocity(b2Vec2(0, 0) );
		horizonBody->SetTransform(b2Vec2(0, 3), 0);

		m_world->ClearForces();
		
		RL_start();
	}

	
	void Step(Settings* settings)
	{
		//run the default physics and rendering
		Test::Step(settings);
		
		int isterminal = 0;
		double reward = 0;
		const reward_observation_action_terminal_t *rl_step_result = 0;
		
		
		rl_step_result = RL_step();
		isterminal = rl_step_result->terminal;
		reward = rl_step_result->reward;
		if(isterminal==1){
			if(reward==100){
				printf("success position : %lf\n",init_position);
				success++;
			}else{
				printf("fail position : %lf\n",init_position);
				fail++;
			}
			reset();
		}

		b2Vec2 pos = horizonBody->GetPosition();
		float angle = verticalBody->GetAngle()*RADTODEG;
		b2Vec2 vel = horizonBody->GetLinearVelocity();
		float angularVel = verticalBody->GetAngularVelocity();

		m_debugDraw.DrawString(5, m_textLine, "position:%.3f,%.3f, angle : %.3f, vel : %.3f, angularvel : %.3f ",pos.x, pos.y, angle, vel.x, angularVel );
		m_textLine += 15;

	}

	public: void apply_force(int force)
	{
		horizonBody->ApplyForce(b2Vec2(force*100-5*100, 0), horizonBody->GetWorldCenter() );
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
