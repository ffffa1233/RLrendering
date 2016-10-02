#ifndef PENDULUM2_H
#define PENDULUM2_H

#define DEGTORAD 0.0174532925199432957f
#define RADTODEG 57.295779513082320876f

#include <stdio.h>
#include <rlglue/RL_glue.h>

extern b2Body* verticalBody;
extern b2Body* horizonBody;
extern b2Body* lineBody;

extern b2Body* verticalBody2;

//extern int fforce;

class Pendulum2: public Test
{
	const char *task_spec;	

	double randomAngle = (rand()%40)+70;
	double randomAngle2 = (rand()%80)+50;

	int success = 0;
	int fail = 0;

	b2PrismaticJoint* m_joint;

	public:
	Pendulum2() 
	{	
		randomAngle = (rand()%40)+70;
		randomAngle2 = (rand()%80)+50;

		b2BodyDef myBodyDef;
		myBodyDef.type = b2_dynamicBody;

		b2PolygonShape rectangleShape;
		rectangleShape.SetAsBox(10, 0.5);

		b2FixtureDef rectangleFixtureDef;
		rectangleFixtureDef.shape = &rectangleShape;
		rectangleFixtureDef.density = 1.5;

		verticalBody = m_world->CreateBody(&myBodyDef);
		verticalBody->CreateFixture(&rectangleFixtureDef);
		
		verticalBody->SetTransform(b2Vec2(9.95*cos(randomAngle*DEGTORAD), 9.95*sin(randomAngle*DEGTORAD)+4.05), randomAngle*DEGTORAD);

//////////////////////////////////////////
		rectangleShape.SetAsBox(10, 0.5);

		rectangleFixtureDef.shape = &rectangleShape;
		rectangleFixtureDef.density = 1.5;

		verticalBody2 = m_world->CreateBody(&myBodyDef);
		
		verticalBody2->SetTransform(
						b2Vec2(
							19.95*cos(randomAngle*DEGTORAD)+9.95*cos(randomAngle2*DEGTORAD), 
							19.95*sin(randomAngle*DEGTORAD)+9.95*sin(randomAngle2*DEGTORAD)+4.05
							),
						randomAngle2*DEGTORAD
			);		

		verticalBody2->CreateFixture(&rectangleFixtureDef);

//////////////////////////////////////////

		rectangleShape.SetAsBox(2.5, 2);
		rectangleFixtureDef.shape = &rectangleShape;
		rectangleFixtureDef.density = 0.05;

		myBodyDef.position.Set(0, 2);
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

		b2RevoluteJointDef lowerJointDef;
		lowerJointDef.localAnchorA.Set(-9.95, 0);
		lowerJointDef.localAnchorB.Set(0, 1.95);
//////////////////////////////////////////		
		b2RevoluteJointDef upperJointDef;
		upperJointDef.localAnchorA.Set(-9.95, 0);
		upperJointDef.localAnchorB.Set(9.95, 0);

		upperJointDef.enableMotor = true;
  		upperJointDef.maxMotorTorque = 20;
  		upperJointDef.motorSpeed = 360 * DEGTORAD;

		upperJointDef.bodyA = verticalBody2;
		upperJointDef.bodyB = verticalBody;

		m_world->CreateJoint(&upperJointDef);
//////////////////////////////////////////
		
		lowerJointDef.bodyA = verticalBody;
		lowerJointDef.bodyB = horizonBody;

		m_world->CreateJoint(&lowerJointDef);

		b2PrismaticJointDef pjd;

		b2Vec2 axis(1.0f, 0.0f);
		axis.Normalize();
		pjd.Initialize(lineBody, horizonBody, b2Vec2(0.0f, 0.0f), axis);

		m_joint = (b2PrismaticJoint*)m_world->CreateJoint(&pjd);

		printf("\nThis is a RL Test Program Pendulum2(Start)\n");
		

	}


	void reset(){
		//printf("reset\n");
		randomAngle = (rand()%40)+70;
		randomAngle2 = (rand()%80)+50;
		
		verticalBody->SetAngularVelocity(0);
		verticalBody->SetLinearVelocity(b2Vec2(0, 0) );
		verticalBody->SetTransform(b2Vec2(9.95*cos(randomAngle*DEGTORAD), 14-(9.95-9.95*sin(randomAngle*DEGTORAD))), randomAngle*DEGTORAD);

///////////////////////////////////////////////////////
		verticalBody2->SetAngularVelocity(0);
		verticalBody2->SetLinearVelocity(b2Vec2(0, 0) );
		verticalBody2->SetTransform(
						b2Vec2(
							19.95*cos(randomAngle*DEGTORAD)+9.95*cos(randomAngle2*DEGTORAD), 
							19.95*sin(randomAngle*DEGTORAD)+9.95*sin(randomAngle2*DEGTORAD)+4.05
							),
						randomAngle2*DEGTORAD
			);	
///////////////////////////////////////////////////////

		horizonBody->SetLinearVelocity(b2Vec2(0, 0) );
		horizonBody->SetTransform(b2Vec2(0, 2), 0);

		m_world->ClearForces();

	}

	void Step(Settings* settings)
	{
		//run the default physics and rendering
		Test::Step(settings);

		b2Vec2 pos = horizonBody->GetPosition();
		float angle = verticalBody->GetAngle()*RADTODEG;
		b2Vec2 vel = horizonBody->GetLinearVelocity();
		float angularVel = verticalBody->GetAngularVelocity();

		m_debugDraw.DrawString(5, m_textLine, "position:%.3f,%.3f, angle : %.3f, vel : %.3f, angularvel : %.3f ",
							pos.x, pos.y, angle, vel.x, angularVel );
		m_textLine += 15;

	}

	public: void apply_force_foot(int force_foot)
	{
		//force
		horizonBody->ApplyForce(b2Vec2((force_foot-5)*1, 0), horizonBody->GetWorldCenter() );
	}
	public: void apply_force_waist(int force_waist)
	{
		//torque	
		//SetMotorSpeed((force_waist-18)*10*DEGTORAD);	
		verticalBody2->ApplyForce(b2Vec2((force_waist-11)*1000, 0), verticalBody2->GetWorldCenter()  );
	}
	public: void apply_force_heap(int force_heap)
	{
		//force
		verticalBody->ApplyForce(b2Vec2((force_heap-9)*100, 0), verticalBody->GetWorldPoint(b2Vec2(10, -0.5)));
	}

	public: double get_angle_footleg(){
		return verticalBody->GetAngle()*RADTODEG;
	}
	public: double get_angle_footbody(){
		return verticalBody2->GetAngle()*RADTODEG;
	}

	public: double get_angleVelocity_footleg()
	{
		return verticalBody->GetAngularVelocity();
	}
	public: double get_angleVelocity_footbody()
	{
		return verticalBody2->GetAngularVelocity();
	}

	public: double get_velocity_foot()
	{
		return horizonBody->GetLinearVelocity().x;
	}

	//return an object of this class to the testbed
	static Test* Create()
	{
		return new Pendulum2;
	}


};

#endif
