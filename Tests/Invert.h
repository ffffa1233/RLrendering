#ifndef INVERT_H
#define INVERT_H

#define DEGTORAD 0.0174532925199432957f
#define RADTODEG 57.295779513082320876f

#include <stdio.h>
#include <rlglue/RL_glue.h>

class Invert : public Test
{
	b2Body* upperBody;
	b2Body* lowerBody;
	b2Body* baseBody;
	b2Body* lineBody;

	public:
	Invert() 
	{	
		
		b2BodyDef myBodyDef;
		myBodyDef.type = b2_dynamicBody;

		b2PolygonShape rectangleShape;
		rectangleShape.SetAsBox(0.5, 2);

		b2FixtureDef rectangleFixtureDef;
		rectangleFixtureDef.shape = &rectangleShape;
		rectangleFixtureDef.density = 1;

		myBodyDef.position.Set(0, 6);
		upperBody = m_world->CreateBody(&myBodyDef);
		upperBody->CreateFixture(&rectangleFixtureDef);

		rectangleShape.SetAsBox(1, 3);

		rectangleFixtureDef.shape = &rectangleShape;
		rectangleFixtureDef.density = 1;

		myBodyDef.position.Set(0, 4);
		lowerBody = m_world->CreateBody(&myBodyDef);
		lowerBody->CreateFixture(&rectangleFixtureDef);

		rectangleShape.SetAsBox(5, 1);
		rectangleFixtureDef.shape = &rectangleShape;
		rectangleFixtureDef.density = 1;
		myBodyDef.position.Set(0, 1);
		baseBody = m_world->CreateBody(&myBodyDef);
		baseBody->CreateFixture(&rectangleFixtureDef);

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
		upperJointDef.localAnchorA.Set(0, -1.95);
		upperJointDef.localAnchorB.Set(0, 2.95);
		upperJointDef.lowerAngle = -45 * DEGTORAD;
		upperJointDef.upperAngle = 45 * DEGTORAD;
		upperJointDef.enableLimit = true;
		upperJointDef.bodyA = upperBody;
		upperJointDef.bodyB = lowerBody;
		
		m_world->CreateJoint(&upperJointDef);
		
		b2RevoluteJointDef lowerJointDef;
		lowerJointDef.localAnchorA.Set(0, -2.95);
		lowerJointDef.localAnchorB.Set(0, 0.95);
		lowerJointDef.lowerAngle = -45 * DEGTORAD;
		lowerJointDef.upperAngle = 45 * DEGTORAD;
		lowerJointDef.enableLimit = true;		
		lowerJointDef.bodyA = lowerBody;
		lowerJointDef.bodyB = baseBody;
		m_world->CreateJoint(&lowerJointDef);

	}

	void Keyboard(unsigned char key){
		switch(key){
			case 's':
			break;
			case 'l':
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
		/*
		//show some information for the dynamic body
		b2Vec2 pos = bodies->GetPosition();
		float angle = bodies->GetAngle();
		b2Vec2 vel = bodies->GetLinearVelocity();
		float angularVel = bodies->GetAngularVelocity();
		m_debugDraw.DrawString(5, m_textLine, "position:%.3f,%.3f", pos.x, pos.y);
		m_textLine += 15;
		*/
	}

	//return an object of this class to the testbed
	static Test* Create()
	{
		return new Invert;
	}


};

#endif
