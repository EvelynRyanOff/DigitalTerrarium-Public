#include "ParticleSystem.h"
#include <string.h>

using namespace std;

ParticleSystem::ParticleSystem( const std::string& name ):
	BaseSystem( name ),
	m_sx( .5f ),
	m_sy( .5f ),
	m_sz( .5f )
{ 

	setVector( m_pos, 0, 0, 0 );
	

	for (int i = 0; i < 500; i++) {
		particles[i][0] = -1;
		particles[i][1] = 0;
		particles[i][2] = 0;
		particles[i][3] = 0;
		particles[i][4] = 0;
		particles[i][5] = 0;
		particles[i][6] = 0;
		particles[i][7] = 0;

		predator_particles[i][0] = -1;
		predator_particles[i][1] = 0;
		predator_particles[i][2] = 0;
		predator_particles[i][3] = 0;
		predator_particles[i][4] = 0;
		predator_particles[i][5] = 0;
		predator_particles[i][6] = 0;
		predator_particles[i][7] = 0;

		



	}
	
	part_num = 0;

}	// ParticleSystem

void ParticleSystem::getState( double* p )
{ 

	VecCopy( p, m_pos ); 

}	// ParticleSystem::getState

void ParticleSystem::setState( double  *p )
{ 

	VecCopy(m_pos,p); 

}	// ParticleSystem::setState

void ParticleSystem::reset( double time ) 
{ 
	
	
	
	
}	// ParticleSystem::Reset


int ParticleSystem::command(int argc, myCONST_SPEC char **argv) 
{
	if( argc < 1 )
	{
		animTcl::OutputMessage("system %s: wrong number of params.", m_name.c_str()) ;
		return TCL_ERROR ;
	}
	else if( strcmp(argv[0], "read") == 0 )
	{
		if( argc == 2 )
		{
			//m_model.ReadOBJ(argv[1]) ;
			//glmFacetNormals(&m_model) ;
			//glmVertexNormals(&m_model, 90) ;
			return TCL_OK ;
		}
		else 
		{
			animTcl::OutputMessage("Usage: read <file_name>") ;
			return TCL_ERROR ;
		}
	}
	else if( strcmp(argv[0], "scale") == 0 )
	{
		if( argc == 4 )
		{
			m_sx = (float)atof(argv[1]) ;
			m_sy = (float)atof(argv[2]) ;
			m_sz = (float)atof(argv[3]) ;
		}
		else
		{
			animTcl::OutputMessage("Usage: scale <sx> <sy> <sz> ") ;
			return TCL_ERROR ;

		}
	}
	else if( strcmp(argv[0], "pos") == 0 )
	{
		if( argc == 4 )
		{
			m_pos[0] = atof(argv[1]) ;
			m_pos[1] = atof(argv[2]) ;
			m_pos[2] = atof(argv[3]) ;
		}
		else
		{
			animTcl::OutputMessage("Usage: pos <x> <y> <z> ") ;
			return TCL_ERROR ;

		}
	}
	else if( strcmp(argv[0], "flipNormals") == 0 )
	{
		//flipNormals() ;
		return TCL_OK ;
		
	}
	else if( strcmp(argv[0], "reset") == 0)
	{
		double p[3] = {0,0,0} ;
		setState(p) ;
	}
	else if (strcmp(argv[0], "dim")==0) {
		if (argc == 2) {
			if (atoi(argv[1]) > 500) {
				animTcl::OutputMessage("too many particles!");
			}
			part_num = atoi(argv[1]);
			animTcl::OutputMessage(argv[1]);
		}
	}
	else if (strcmp(argv[0], "particle") == 0) {
		if (argc == 9) {
			int index = atoi(argv[1]);
			particles[index][0] = std::stod(argv[2]); //mass
			particles[index][1] = std::stod(argv[3]); //x
			particles[index][2] = std::stod(argv[4]); //y
			particles[index][3] = std::stod(argv[5]); //z
			particles[index][4] = std::stod(argv[6]); //vx
			particles[index][5] = std::stod(argv[7]); //vy
			particles[index][6] = std::stod(argv[8]); //vz
		
			
			animTcl::OutputMessage(argv[5]);
		}
	}
	else if (strcmp(argv[0], "particle") == 0) {
		if (argc == 9) {
			int index = atoi(argv[1]);
			particles[index][0] = std::stod(argv[2]); //mass
			particles[index][1] = std::stod(argv[3]); //x
			particles[index][2] = std::stod(argv[4]); //y
			particles[index][3] = std::stod(argv[5]); //z
			particles[index][4] = std::stod(argv[6]); //vx
			particles[index][5] = std::stod(argv[7]); //vy
			particles[index][6] = std::stod(argv[8]); //vz
			particles[index][7] = 0; //state

			

			animTcl::OutputMessage(argv[5]);

			glutPostRedisplay();
			return TCL_OK;
		}
	}
	else if (strcmp(argv[0], "predpart") == 0) {
		if (argc == 9) {
			int index = atoi(argv[1]);
			predator_particles[index][0] = std::stod(argv[2]); //mass
			predator_particles[index][1] = std::stod(argv[3]); //x
			predator_particles[index][2] = std::stod(argv[4]); //y
			predator_particles[index][3] = std::stod(argv[5]); //z
			predator_particles[index][4] = std::stod(argv[6]); //vx
			predator_particles[index][5] = std::stod(argv[7]); //vy
			predator_particles[index][6] = std::stod(argv[8]); //vz
			predator_particles[index][7] = 0; //state

			stringstream strs;
			strs << predator_particles[index][4];
			strs << predator_particles[index][5];
			strs << predator_particles[index][6];
			string temp_str = strs.str();
			char* char_type = (char*)temp_str.c_str();
			animTcl::OutputMessage(char_type);
		}
	}
	else if (strcmp(argv[0], "getPos") == 0) {
		if (argc == 2) {
			int index = atoi( argv[1] );
			//animTcl::OutputMessage(argv[1]);
			m_pos[0] = particles[index][1];
			m_pos[1] = particles[index][2];
			m_pos[2] = particles[index][3];
		}
	}
	else if (strcmp(argv[0], "setPos") ==0) {

		if (argc == 2) {
			int index = atoi(argv[1]);
			//animTcl::OutputMessage(argv[1]);
			particles[index][1] = m_pos[0];
			particles[index][2] = m_pos[1];
			particles[index][3] = m_pos[2];
			/*
			stringstream strs;
			strs << particles[index][1];
			string temp_str = strs.str();
			char* char_type = (char*)temp_str.c_str();
			animTcl::OutputMessage(char_type);
			*/
		}
	}
	else if (strcmp(argv[0], "getVel") == 0) {
		if (argc == 2) {
			int index = atoi(argv[1]);
			//animTcl::OutputMessage(argv[1]);
			m_pos[0] = particles[index][4];
			m_pos[1] = particles[index][5];
			m_pos[2] = particles[index][6];
		}
	}
	else if (strcmp(argv[0], "setVel") == 0) {
		if (argc == 2) {
			int index = atoi(argv[1]);
			//animTcl::OutputMessage(argv[1]);
			particles[index][4] = m_pos[0];
			particles[index][5] = m_pos[1];
			particles[index][6] = m_pos[2];

		}
	}
	else if (strcmp(argv[0], "getMass") == 0) {
		if (argc == 2) {
			int index = atoi(argv[1]);
			//animTcl::OutputMessage(argv[1]);
			m_pos[0] = particles[index][0];
			/*
			animTcl::OutputMessage("in getMass");
			stringstream strs;
			strs << m_pos[0];
			string temp_str = strs.str();
			char* char_type = (char*)temp_str.c_str();
			animTcl::OutputMessage(char_type);*/
		}
		}
	else if (strcmp(argv[0], "setMass") == 0) {
		if (argc == 2) {
			int index = atoi(argv[1]);
			//animTcl::OutputMessage(argv[1]);
			particles[index][0] = m_pos[0];
		}
	}//
	else if (strcmp(argv[0], "getPartNum") == 0) {
		if (argc == 1) {
			
			//animTcl::OutputMessage(argv[1]);
			m_pos[0] = part_num;

			/*
			animTcl::OutputMessage("in getPartNum");
			stringstream strs;
			strs << part_num;
			string temp_str = strs.str();
			char* char_type = (char*)temp_str.c_str();
			animTcl::OutputMessage(char_type);
			*/
		}
		}
	else if (strcmp(argv[0], "all_velocities") == 0) {
		if (argc == 4) {

			for (int i = 0; i < part_num; i++) {
				particles[i][4] = stod(argv[1]);
				particles[i][5] = stod(argv[2]);
				particles[i][6] = stod(argv[3]);
			}

		}
	}
    
    glutPostRedisplay() ;
	return TCL_OK ;

}	// ParticleSystem::command

void ParticleSystem::displayPart(Vector p, float r) {
	/*
	stringstream strs;
	strs << p[0];
	string temp_str = strs.str();
	char* char_type = (char*)temp_str.c_str();
	animTcl::OutputMessage(char_type );
	*/
	glPointSize(r);
	//glBegin(GL_POINTS);
	glVertex3d(p[0], p[1],p[2]);
	//glEnd();
}


void ParticleSystem::display( GLenum mode ) 
{
	glEnable(GL_LIGHTING) ;
	glMatrixMode(GL_MODELVIEW) ;
	glEnable(GL_COLOR_MATERIAL);
	glPushMatrix() ;
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glScalef(m_sx, m_sy, m_sz);
	
	
	
	
	
	//glPointSize(10);
	//glBegin(GL_POINTS);
	for (int i = 0; i < part_num; i++) {
		if (particles[i][0] != -1) {

			glPushMatrix();
			glPushAttrib(GL_ALL_ATTRIB_BITS);
			glColor3f(1, 0, 1);

			glTranslated(particles[i][1], particles[i][2], particles[i][3]);
			glutSolidSphere(0.5, 20, 20);
			/*Vector p;
			p[0] = particles[i][1];
			p[1] = particles[i][2];
			p[2] = particles[i][3];
			displayPart(p, 10);*/
			glPopMatrix();
			glPopAttrib();

		}
		
		if (predator_particles[i][0] != -1) {
			glPushMatrix();
			glPushAttrib(GL_ALL_ATTRIB_BITS);
			glColor3f(1, 0.65, 0);

			glTranslated(predator_particles[i][1], predator_particles[i][2], predator_particles[i][3]);
			glutSolidSphere(0.5, 20, 20);
			/*Vector p;
			p[0] = particles[i][1];
			p[1] = particles[i][2];
			p[2] = particles[i][3];
			displayPart(p, 10);*/
			glPopMatrix();
			glPopAttrib();


		}
	}
	//glEnd();
	
	

	glPopMatrix();
	glPopAttrib();

}	// ParticleSystem::display
