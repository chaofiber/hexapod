#pragma once

#include <utils/mathUtils.h>


class Segment{
	public:
	P3D a = P3D(0, 0, 0);
	P3D b = P3D(0, 0, 0);

	Segment(){}
	
	Segment(const P3D& a_, const P3D& b_){
		this->origin = a_;
		this->dir = b_;
	}
};

