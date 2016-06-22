#ifndef RLTEST_H
#define RLTEST_H

#define DEGTORAD 0.0174532925199432957f
#define RADTODEG 57.295779513082320876f

#include <stdio.h>
#include <rlglue/RL_glue.h>

extern b2Body* bodies;
class rltest : public Test
{
	const char *task_spec;

	int iter = 1;
	int iter2 = 1;

	bool learning = true;
	bool load = false;

	double init_position = 35.0;
	int success = 0;
	int fail = 0;

	public:
	rltest() 
	{	
		//body definition
		b2BodyDef myBodyDef;
		myBodyDef.type = b2_dynamicBody;

		myBodyDef.position.Set(init_position,0);

		//shape definition
		b2PolygonShape polygonShape;
		polygonShape.SetAsBox(1, 1); //a 2x2 rectangle

		//fixture definition
		b2FixtureDef myFixtureDef;
		myFixtureDef.shape = &polygonShape;
		myFixtureDef.density = 1;

		//create identical bodies in different positions
		bodies = m_world->CreateBody(&myBodyDef);
		bodies->CreateFixture(&myFixtureDef);

		//a static floor to drop things on
		myBodyDef.type = b2_staticBody;
		myBodyDef.position.Set(0,0);
		polygonShape.SetAsEdge( b2Vec2(-1000000,0), b2Vec2(1000000,0) );
		m_world->CreateBody(&myBodyDef)->CreateFixture(&myFixtureDef);

		printf("\nThis is a RL Test Program(Start)\n");
		
		task_spec = RL_init();

		printf("\nTASK SPEC : %s\n",task_spec);
		printf("Starting offline demo\n------------------------\nWill alternate learning for 25 episodes, then freeze policy and evaluate for 10 episodes.\n\n");
		printf("After Episode\tMean Return\tStandard Deviation\n---------------------------------------------------------------------\n");

		RL_start();

		RL_agent_message("load_policy results.dat");
		RL_agent_message("freeze learning");
	}

	void reset(){
		init_position = RandomFloat(0.0f, 50.0f);
		bodies->SetTransform(b2Vec2(init_position, 0), 0);
		bodies->SetLinearVelocity(b2Vec2(0, 0) );
		bodies->SetAngularVelocity(0);

		m_world->ClearForces();
		
		RL_start();
	}

	void Keyboard(unsigned char key){
		switch(key){
			case 's':
				learning = false;
				load = false;
				
				RL_agent_message("save_policy results.dat");
				RL_cleanup();

				reset();

				task_spec = RL_init();
			break;
			case 'l':
				learning = false;
				load = true;
				iter2 = 1;

				RL_agent_message("load_policy results.dat");
				RL_agent_message("freeze learning");
				reset();
			break;
			default:
				Test::Keyboard(key);
			break;
		}
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
		


		/* if user press 's' key, learning will be false, then stop learning, save results.dat */
/*		if(learning){
			rl_step_result = RL_step();
			isterminal = rl_step_result->terminal;
		
			if(isterminal==1){
				reset();
				iter2++;
			}
		}*/

		/*if user press 'l' key, then load results.dat */
/*		if(load){
			rl_step_result = RL_step();
			isterminal = rl_step_result->terminal;
			if(isterminal == 1){
				reset();
				iter2++;
			}

			//RL_agent_message("unfreeze learning");
		}
*/
		//show some information for the dynamic body
		b2Vec2 pos = bodies->GetPosition();
		float angle = bodies->GetAngle();
		b2Vec2 vel = bodies->GetLinearVelocity();
		float angularVel = bodies->GetAngularVelocity();
		m_debugDraw.DrawString(5, m_textLine, "position:%.3f,%.3f, vel : %.3f, success/fail : %d / %d",pos.x, pos.y, vel.x, success, fail);
		m_textLine += 15;
		iter++;
	}

	static void apply_force(int force)
	{
		bodies->ApplyForce(b2Vec2(force*100-5*100, 0), bodies->GetWorldCenter() );
	}

	public: double get_position()
	{
		return bodies->GetPosition().x;
	}

	public: double get_velocity()
	{
		return bodies->GetLinearVelocity().x;
	}

	//return an object of this class to the testbed
	static Test* Create()
	{
		return new rltest;
	}


};

#endif
