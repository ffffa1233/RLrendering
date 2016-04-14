#ifndef PRACTICE_H
#define PRACTICE_H

#define DEGTORAD 0.0174532925199432957f
#define RADTODEG 57.295779513082320876f

#include <stdio.h>
#include <stdlib.h>

double *Vvalue_function;
FILE *Ffp;
int NnumActions = 11;
int NnumStates = 51;

b2Body* Bbodies;

class practice : public Test
{
	public:
	practice() 
	{	
		//body definition
		b2BodyDef myBodyDef;
		myBodyDef.type = b2_dynamicBody;
		
		myBodyDef.position.Set(45,0);

		//shape definition
		b2PolygonShape polygonShape;
		polygonShape.SetAsBox(1, 1); //a 2x2 rectangle

		//fixture definition
		b2FixtureDef myFixtureDef;
		myFixtureDef.shape = &polygonShape;
		myFixtureDef.density = 1;

		//create identical bodies in different positions
		Bbodies = m_world->CreateBody(&myBodyDef);
		Bbodies->CreateFixture(&myFixtureDef);

		//a static floor to drop things on
		myBodyDef.type = b2_staticBody;
		myBodyDef.position.Set(0,0);
		polygonShape.SetAsEdge( b2Vec2(-1000000,0), b2Vec2(1000000,0) );
		m_world->CreateBody(&myBodyDef)->CreateFixture(&myFixtureDef);

		Vvalue_function = (double*)calloc(NnumActions*NnumStates, sizeof(double));
		
		Ffp = fopen("results.dat", "rb");
		while(Ffp == NULL) printf("???\n");
		fread(Vvalue_function, sizeof(double), NnumActions*NnumStates, Ffp);
		fclose(Ffp);

		int i;
		for(i=0;i<NnumActions*NnumStates;i++)
			printf("%lf\n",Vvalue_function[i]);
	}
	
	void launch()
	{	printf("launch :        ");
		Bbodies->SetTransform(b2Vec2(45.0f, 0.0f), 0.0f);
	}

	void Step(Settings* settings)
	{
		//run the default physics and rendering
		Test::Step(settings);
		
		//show some information for the dynamic body
		b2Vec2 pos = Bbodies->GetPosition();
		float angle = Bbodies->GetAngle();
		b2Vec2 vel = Bbodies->GetLinearVelocity();
		float angularVel = Bbodies->GetAngularVelocity();
		m_debugDraw.DrawString(5, m_textLine, "position:%.3f,%.3f",pos.x, pos.y);
		m_textLine += 15;

		int posx = (int)pos.x;
		int i,j,force;
		double max = -100;
		for(i=0;i<NnumActions;i++){
			if(max < Vvalue_function[posx*NnumActions+i]){
				max = Vvalue_function[posx*NnumActions + i];
				force = i;
			}
		}
		printf("posx %lf force : %d\n",pos.x, force);
		Bbodies->ApplyLinearImpulse(b2Vec2(force-5, 0), Bbodies->GetWorldCenter() );

		if (m_stepCount % 60 == 0)
		{
			launch();
		}
		
	}

	//return an object of this class to the testbed
	static Test* Create()
	{
		return new practice;
	}
};

#endif
