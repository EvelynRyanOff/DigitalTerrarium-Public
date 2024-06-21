#ifndef MY_PARTSIM_H
#define MY_PARTSIM_H

#include <GLModel/GLModel.h>
#include <shared/defs.h>
#include <util/util.h>
#include "animTcl.h"
#include "BaseSimulator.h"
#include "BaseSystem.h"
#include <iostream>
#include <string>
#include <sstream>
#include <string>
#include "ParticleSystem.h"
#include <string.h>
#include "anim.h"
#include <time.h> 
#include <cstdlib>

using namespace std;
// a sample simulator

extern double timeStep;


class particleSimulator : public BaseSimulator 
{
public:

	particleSimulator( const std::string& name, PartRef target );
	~particleSimulator();

	
	double Force(double* pos, double* vel, int index);


	//double velocity(double vel, double force);
	virtual void Euler(double* pos, double* vel, double time, int i);
	virtual void Symplectic(double* pos, double* vel, double time, int i);
	virtual void predator_Symplectic(double* pos, double* vel, double time, int i);
	virtual void Verlet(double* pos, double* vel, double time, int i);
	double fn(double* pos, double* vel, int index);
	

	virtual void prey_behaviourTree(double* pos, double* vel, int i, double* target);
	virtual void prey_flockingEngine(double* pos, double* vel, int i, double* target);
	virtual void prey_flocking(double* pos, double* vel, int i, double* target);
	virtual void prey_danceEngine(double* pos, double* vel, int i, double* target);
	virtual void prey_dance(double* pos, double* vel, int i, double* target);
	virtual void prey_searchEngine(double* pos, double* vel, int i, double* target);
	virtual void prey_search(double* pos, double* vel, int i, double* target);
	virtual void prey_variationEngine(double* pos, double* vel, int i, double* target);
	virtual void prey_variation(double* pos, double* vel, int i, double* target);
	virtual void prey_override(double* pos, double* vel, int i, double* target);

	virtual void predator_behaviourTree(double* pos, double* vel, int i, double* target);
	virtual void predator_flockingEngine(double* pos, double* vel, int i, double* target);
	virtual void predator_flocking(double* pos, double* vel, int i, double* target);
	virtual void predator_danceEngine(double* pos, double* vel, int i, double* target);
	virtual void predator_dance(double* pos, double* vel, int i, double* target);
	virtual void predator_huntEngine(double* pos, double* vel, int i, double* target);
	virtual void predator_hunt(double* pos, double* vel, int i, double* target);
	virtual void predator_variationEngine(double* pos, double* vel, int i, double* target);
	virtual void predator_variation(double* pos, double* vel, int i, double* target);
	virtual void predator_override(double* pos, double* vel, int i, double* target);

	int step(double time);
	int init(double time) 
	{ 
		/*
		timeStep = 0.01;
		prevTime = 0;
		gravity[0] = 0;
		gravity[1] = -9.8;
		gravity[2] = 0;
		kdrag = 5;
		fix = -1;
		*/
	
		

		return 0;
	};

	int command(int argc, myCONST_SPEC char** argv);

protected:





	PartRef m_object;


	int spring_ind;
	long double prevTime;
	double gravity[3];
	double mass;
	double kdrag;
	bool euler;
	bool symplectic;
	bool verlet;
	int fix;
	double kd;
	double ks;
	//int fix;
	double previousPositions[1000][3];

	int part_num;
};


#endif