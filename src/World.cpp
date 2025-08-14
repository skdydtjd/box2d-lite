/*
* Copyright (c) 2006-2009 Erin Catto http://www.gphysics.com
*
* Permission to use, copy, modify, distribute and sell this software
* and its documentation for any purpose is hereby granted without fee,
* provided that the above copyright notice appear in all copies.
* Erin Catto makes no representations about the suitability
* of this software for any purpose.
* It is provided "as is" without express or implied warranty.
*/

#include "box2d-lite/World.h"
#include "box2d-lite/Body.h"
#include "box2d-lite/Joint.h"

using std::vector;
using std::map;
using std::pair;

typedef map<ArbiterKey, Arbiter>::iterator ArbIter;
typedef pair<ArbiterKey, Arbiter> ArbPair;

bool World::accumulateImpulses = true;
bool World::warmStarting = true;
bool World::positionCorrection = true;
bool World::Moter = true; // 변수 가져옴, 초기화

void World::Add(Body* body)
{
	bodies.push_back(body);
}

void World::Add(Joint* joint)
{
	joints.push_back(joint);
}

void World::Clear()
{
	bodies.clear();
	joints.clear();
	arbiters.clear();
}

void World::BroadPhase()
{
	// O(n^2) broad-phase
	for (int i = 0; i < (int)bodies.size(); ++i)
	{
		Body* bi = bodies[i];

		for (int j = i + 1; j < (int)bodies.size(); ++j)
		{
			Body* bj = bodies[j];

			if (bi->invMass == 0.0f && bj->invMass == 0.0f)
				continue;

			Arbiter newArb(bi, bj);
			ArbiterKey key(bi, bj);

			if (newArb.numContacts > 0)
			{
				ArbIter iter = arbiters.find(key);
				if (iter == arbiters.end())
				{
					arbiters.insert(ArbPair(key, newArb));
				}
				else
				{
					iter->second.Update(newArb.contacts, newArb.numContacts);
				}
			}
			else
			{
				arbiters.erase(key);
			}
		}
	}
}

void World::Step(float dt)
{
	//printf("debug - step \n");
	float inv_dt = dt > 0.0f ? 1.0f / dt : 0.0f;

	// Determine overlapping bodies and update contact points.
	BroadPhase();

	// Integrate forces.
	for (int i = 0; i < (int)bodies.size(); ++i)
	{
		//printf("debug - step-force \n");
		Body* b = bodies[i];

		if (b->invMass == 0.0f)
			continue;

		//Vec2 TestFor_oldVelocity = b->velocity; // DEBUG

		b->velocity += dt * (gravity + b->invMass * b->force);
		b->angularVelocity += dt * b->invI * b->torque;

		if (b->isItExist == false) {
			b->velocity.Set(0, 0);
			b->angularVelocity = 0.0f;
			//b->position.Set(20, 20);
		}
		//if (b->velocity == TestFor_oldVelocity) { printf("ERROR - No Change of b->velocity \n"); }
	}

	// Perform pre-steps.
	for (ArbIter arb = arbiters.begin(); arb != arbiters.end(); ++arb)
	{
		//printf("debug - ReadyPreStep \n");
		arb->second.PreStep(inv_dt);
	}

	for (int i = 0; i < (int)joints.size(); ++i)
	{
		joints[i]->PreStep(inv_dt);
	}

	// Perform iterations
	for (int i = 0; i < iterations; ++i)
	{
		for (ArbIter arb = arbiters.begin(); arb != arbiters.end(); ++arb)
		{
			//Body* dummy[2]; // 초기화되지 않았습니다.
			//arb->second.ApplyImpulse();
			//Body** deadBodyStorage = {NULL,};
			arb->second.ApplyImpulse(deadBodyStorage, 200);

			// 요기서 메모리를 쌓을까요
			//arb->second.ApplyImpulse(&deadBodyStorage[0],200);
		}

		for (int j = 0; j < (int)joints.size(); ++j)
		{
			joints[j]->ApplyImpulse();
		}
	}



	// Integrate Velocities
	for (int i = 0; i < (int)bodies.size(); ++i)
	{
		Body* b = bodies[i];

		Vec2 TestFor_OldPosition = b->position;

		b->position += dt * b->velocity;
		b->rotation += dt * b->angularVelocity;


		//if((b->position == TestFor_OldPosition) && !(b->velocity.x == 0 && b->velocity.y == 0)) { printf("ERROR - B->position change did not worked. \n"); }

		b->force.Set(0.0f, 0.0f);
		b->torque = 0.0f;
	}

	//Break Block
	for (int i = 0; i < 200; i++) {
		deadBodyStorage[i] = NULL;
	}
}
