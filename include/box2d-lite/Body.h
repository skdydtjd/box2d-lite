/*
* Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
*
* Permission to use, copy, modify, distribute and sell this software
* and its documentation for any purpose is hereby granted without fee,
* provided that the above copyright notice appear in all copies.
* Erin Catto makes no representations about the suitability
* of this software for any purpose.
* It is provided "as is" without express or implied warranty.
*/

#ifndef BODY_H
#define BODY_H

#include "MathUtils.h"

struct Body
{
	Body();
	void Set(const Vec2& w, float m);

	void AddForce(const Vec2& f)
	{
		force += f;
	}
	void setPosition2(Vec2& v);

	Vec2 position;
	float rotation;

	Vec2 velocity;
	float angularVelocity;

	Vec2 force;
	float torque;

	Vec2 width;

	float friction;
	float mass, invMass;
	float I, invI;

	// 추가한 변수들
	float impulseLimit; // 최대한 견딜 수 있는 충격량
	bool isBreakAble; // 충격을 받고 파괴될 수 있는지 여부
	bool isItExist = true; // 강체가 충격에 의해 파괴되었는지 생사 여부
};

#endif
