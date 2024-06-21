#ifndef MY_PARTICLE_H
#define MY_PARTICLE_H

/*

	This is a sample system. It accepts the command "read" followed by the 
	path to an OBJ model.

*/


#include "BaseSystem.h"
#include <shared/defs.h>
#include <util/util.h>
#include "animTcl.h"
#include <GLmodel/GLmodel.h>
#include <iostream>
#include <string>
#include <sstream>
#include "shared/opengl.h"

// a sample system
//turn this into a type ref
typedef std::shared_ptr<class ParticleSystem> PartRef;

class ParticleSystem : public BaseSystem
{ 

public:
	ParticleSystem( const std::string& name );
	virtual void getState( double *p );
	virtual void setState( double  *p );
	void reset( double time );

	void displayPart(Vector p, float r);
	
	void display( GLenum mode = GL_RENDER );

	
	int command(int argc, myCONST_SPEC char **argv) ;


	double particles[500][8]; //mass, x , y, z, vx, vy, vz, state
	double predator_particles[500][8]; //mass, x , y, z, vx, vy, vz, state
	int part_num;

protected:

	float m_sx;
	float m_sy;
	float m_sz;

	Vector m_pos ;
	Vector m_vel;
	
	
	


	//GLMmodel m_model ;

} ;
#endif
