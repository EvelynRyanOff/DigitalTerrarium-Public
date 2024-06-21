#include "particleSimulator.h"


particleSimulator::particleSimulator( const std::string& name, PartRef target ):
	BaseSimulator( name ),
	m_object( target )
{
	euler = false;
	symplectic = true;
	verlet = false;
	timeStep = 0.01;
	prevTime = 0;
	gravity[0] = 0;
	gravity[1] = -9.8;
	gravity[2] = 0;
	
	kdrag = 0.05;
	fix = -1;
	kd = 300;
	ks = 50;
	mass = 0;
	spring_ind = -1;

	srand(time(0));

}	// particleSimulator

particleSimulator::~particleSimulator()
{
}	// particleSimulator::~particleSimulator
int particleSimulator::command(int argc, myCONST_SPEC char** argv) {
	if (argc == 0) {
		return TCL_OK;
	}
	
	else if (strcmp(argv[0], "integration") == 0) {
		if (argc == 3) {
			if (strcmp(argv[1], "euler")== 0 ) {
				euler = true;
				symplectic = false;
				verlet = false;
			}
			else if (strcmp(argv[1], "symplectic")==0) {
				euler = false;
				symplectic = true;
				verlet = false;
			}
			else if (strcmp(argv[1], "verlet") == 0) {
				euler = false;
				symplectic = false;
				verlet = true;
			}

			timeStep = stod(argv[2]);
		}
		
	}
	else if (strcmp(argv[0], "gravity") == 0) {
		if (argc == 2) {
			gravity[1] = stod(argv[1]);
		}
	}else if (strcmp(argv[0], "fix")==0) {
		if (argc == 2) {
			
			fix = stoi(argv[1]);

		}
	}
	
	return TCL_OK;
}

//predator behaviour
void particleSimulator::predator_behaviourTree(double* pos, double* vel, int i, double* target) {
	if (m_object->particles[i][7] == 0) {
		predator_flockingEngine(pos, vel, i, target);
	}
	else if(m_object->particles[i][7] == 1){
		predator_huntEngine(pos, vel, i, target);
	}
	predator_override(pos, vel, i, vel);
}

void particleSimulator::predator_flockingEngine(double* pos, double* vel, int i, double* target) {
	int random;
	random = rand() % 100;
	if (random <= 60) {
		predator_flocking(pos, vel, i, target);
	}
	else if (random >= 61) {
		predator_hunt(pos, vel, i, target);
	}
}
void particleSimulator::predator_flocking(double* pos, double* vel, int i, double* target) {
	m_object->predator_particles[i][7] = 0;
	Vector separation, cohesion, alignment;
	separation[0] = 0;
	separation[1] = 0;
	separation[2] = 0;

	cohesion[0] = 0;
	cohesion[1] = 0;
	cohesion[2] = 0;

	alignment[0] = 0;
	alignment[1] = 0;
	alignment[2] = 0;




	Vector pos2, vel2, dist;
	
	int flock_mate = 0;
	double mass2, distance, speed;
	double total_mass = 0.0;


	//normalize given velocity, save the boids speed
	speed = sqrt(pow(vel[0], 2) + pow(vel[1], 2) + pow(vel[2], 2));
	/*
	stringstream strs;
	strs << vel[0];
	strs << vel[1];
	strs << vel[2];
	strs << speed;
	string temp_str = strs.str();
	char* char_type = (char*)temp_str.c_str();
	animTcl::OutputMessage(char_type);
	animTcl::OutputMessage("in flock");*/

	if (speed != 0) {
		vel[0] = vel[0] / speed;
		vel[1] = vel[1] / speed;
		vel[2] = vel[2] / speed;
	}
	else {
		//animTcl::OutputMessage("in else, vel normalized 1");
	}

	for (int j = 0; j < part_num; j++) {



		/*
		stringstream strs;
		strs << j;
		string temp_str = strs.str();
		char* index = (char*)temp_str.c_str();
		//animTcl::OutputMessage(index);
		args[1] = index;

		args[0] = "getPos";
		args[1] = index;
		m_object->command(2, args);
		m_object->getState(pos2);*/

		pos2[0] = m_object->predator_particles[j][1];
		pos2[1] = m_object->predator_particles[j][2];
		pos2[2] = m_object->predator_particles[j][3];


		//vector from given point to new point
		dist[0] = pos2[0] - pos[0];
		dist[1] = pos2[1] - pos[1];
		dist[2] = pos2[2] - pos[2];

		//length of vector
		distance = sqrt(pow(dist[0], 2) + pow(dist[1], 2) + pow(dist[2], 2));

		//check if boid is withing neighborhood
		if (distance < 10 && i != j) {
			//animTcl::OutputMessage("neighborfound");
			flock_mate++;

			//normalize dist
			if (distance != 0) {
				dist[0] = dist[0] / distance;
				dist[1] = dist[1] / distance;
				dist[2] = dist[2] / distance;
			}
			else {
				//animTcl::OutputMessage("in else, distance normalized");
			}
			//add dist to seperation
			separation[0] += dist[0];
			separation[1] += dist[1];
			separation[2] += dist[2];

			
			mass2 = m_object->predator_particles[j][0];

			if (mass2 == -1) {
				continue;
			}

			total_mass += mass2;
			cohesion[0] += mass2 * pos2[0];
			cohesion[1] += mass2 * pos2[1];
			cohesion[2] += mass2 * pos2[2];
			
			/*stringstream strs;
			strs << mass2;
			strs << cohesion[0];
			strs << cohesion[1];
			strs << cohesion[2];
			string temp_str = strs.str();
			char* char_type = (char*)temp_str.c_str();
			animTcl::OutputMessage(char_type);
			animTcl::OutputMessage("mass");*/
			
			vel2[0] = m_object->predator_particles[j][4];
			vel2[1] = m_object->predator_particles[j][5];
			vel2[2] = m_object->predator_particles[j][6];

			//normalize vel2 to get other boid's bearing
			distance = sqrt(pow(vel2[0], 2) + pow(vel2[1], 2) + pow(vel2[2], 2));
			if (distance != 0) {
				vel2[0] = vel2[0] / distance;
				vel2[1] = vel2[1] / distance;
				vel2[2] = vel2[2] / distance;
			}
			else {
				//animTcl::OutputMessage("in else, vel2 normalized");
			}
			//add to alignment
			alignment[0] += vel2[0];
			alignment[1] += vel2[1];
			alignment[2] += vel2[2];



		}
	}
	//complete cohesion: find center of mass then find direction from point to center of mass
	if (total_mass != 0) {
		cohesion[0] = cohesion[0] / total_mass;
		cohesion[1] = cohesion[1] / total_mass;
		cohesion[2] = cohesion[2] / total_mass;
		Vector center_mass;
		VecCopy(center_mass, cohesion);
		cohesion[0] = center_mass[0] - pos[0];
		cohesion[1] = center_mass[1] - pos[1];
		cohesion[2] = center_mass[2] - pos[2];
	}
	else {
		//animTcl::OutputMessage("in else, center of mass");
		cohesion[0] = 0;
		cohesion[1] = 0;
		cohesion[2] = 0;
	}

	distance = sqrt(pow(cohesion[0], 2) + pow(cohesion[1], 2) + pow(cohesion[2], 2));

	if (distance != 0) {

		cohesion[0] = cohesion[0] / distance;
		cohesion[1] = cohesion[1] / distance;
		cohesion[2] = cohesion[2] / distance;
	}
	else {
		//animTcl::OutputMessage("in else, cohesion normalize");
	}
	//average the direction sum then normalize
	if (flock_mate != 0) {
		alignment[0] = alignment[0] / flock_mate;
		alignment[1] = alignment[1] / flock_mate;
		alignment[2] = alignment[2] / flock_mate;
	}
	else {
		//animTcl::OutputMessage("in else, alignment average");
	}

	distance = sqrt(pow(alignment[0], 2) + pow(alignment[1], 2) + pow(alignment[2], 2));

	if (distance != 0) {
		alignment[0] = alignment[0] / distance;
		alignment[1] = alignment[1] / distance;
		alignment[2] = alignment[2] / distance;
	}
	else {
		//animTcl::OutputMessage("in else, alignment normalize");
	}
	//normalize directions to other boids then multiply by -1
	distance = sqrt(pow(separation[0], 2) + pow(separation[1], 2) + pow(separation[2], 2));
	if (distance != 0) {
		separation[0] = separation[0] / distance;
		separation[1] = separation[1] / distance;
		separation[2] = separation[2] / distance;
	}
	else {
		//animTcl::OutputMessage("in else, seperation normalize");
	}

	separation[0] = separation[0] * -1;
	separation[1] = separation[1] * -1;
	separation[2] = separation[2] * -1;

	vel[0] = vel[0] + cohesion[0] + alignment[0] + 2*separation[0];
	vel[1] = vel[1] + cohesion[1] + alignment[1] + 2*separation[1];
	vel[2] = vel[2] + cohesion[2] + alignment[2] + 2*separation[2];
	if (vel[0] == 0 && vel[1] == 0 && vel[2] == 0) {
		vel[0] = cohesion[0] + alignment[0] + separation[0];
		vel[1] = cohesion[1] + alignment[1] + separation[1];
		vel[2] = cohesion[2] + alignment[2] + separation[2];
	}

	distance = sqrt(pow(vel[0], 2) + pow(vel[1], 2) + pow(vel[2], 2));
	if (distance != 0) {
		vel[0] = vel[0] / distance;
		vel[1] = vel[1] / distance;
		vel[2] = vel[2] / distance;
	}
	else {
		//animTcl::OutputMessage("in else, vel normalize");
	}


	if (speed >= sqrt(3) || speed < 0) {
		speed = sqrt(3);

	}
	vel[0] = vel[0] * speed;
	vel[1] = vel[1] * speed;
	vel[2] = vel[2] * speed;
	
	


	VecCopy(target, vel);



}
void particleSimulator::predator_danceEngine(double* pos, double* vel, int i, double* target) {

}
void particleSimulator::predator_dance(double* pos, double* vel, int i, double* target) {

}
void particleSimulator::predator_huntEngine(double* pos, double* vel, int i, double* target) {
	int random;
	random = rand() % 100;
	if (random <= 60) {
		predator_hunt(pos, vel, i, target);
	}
	else if (random >= 61) {

		predator_flocking(pos, vel, i, target);
		
	}
}
void particleSimulator::predator_hunt(double* pos, double* vel, int i, double* target) {
	m_object->predator_particles[i][7] = 0;
	Vector angelvector, prey_pos;
	double distance, smallest_distance;
	int smallest_j;

	smallest_distance = 600;
	smallest_j = -1;
	for (int j = 0; j < part_num; j++) {
		prey_pos[0] = m_object->particles[j][1];
		prey_pos[1] = m_object->particles[j][2];
		prey_pos[2] = m_object->particles[j][3];

		angelvector[0] = prey_pos[0] - pos[0];
		angelvector[1] = prey_pos[1] - pos[1];
		angelvector[2] = prey_pos[2] - pos[2];

		distance = sqrt(pow(angelvector[0],2)+pow(angelvector[1],2)+ pow(angelvector[2],2));

		if (distance < smallest_distance && m_object->particles[j][0]!= -1) {
			smallest_distance = distance;
			smallest_j = j;
		}
	}
	if (smallest_j != -1) {
		//animTcl::OutputMessage("in if, hunt");
		prey_pos[0] = m_object->particles[smallest_j][1];
		prey_pos[1] = m_object->particles[smallest_j][2];
		prey_pos[2] = m_object->particles[smallest_j][3];

		angelvector[0] = prey_pos[0] - pos[0];
		angelvector[1] = prey_pos[1] - pos[1];
		angelvector[2] = prey_pos[2] - pos[2];

		distance = sqrt(pow(angelvector[0], 2) + pow(angelvector[1], 2) + pow(angelvector[2], 2));

		angelvector[0] = angelvector[0] / distance;
		angelvector[1] = angelvector[1] / distance;
		angelvector[2] = angelvector[2] / distance;

		vel[0] = angelvector[0];
		vel[1] = angelvector[1];
		vel[2] = angelvector[2];
		
		/*
		distance = sqrt(pow(vel[0], 2) + pow(vel[1], 2) + pow(vel[2], 2));

		vel[0] = vel[0] / distance;
		vel[1] = vel[1] / distance;
		vel[2] = vel[2] / distance;*/

		vel[0] = vel[0] * sqrt(5);
		vel[1] = vel[1] * sqrt(5);
		vel[2] = vel[2] * sqrt(5);

		VecCopy(target, vel);
	}



}
void particleSimulator::predator_variationEngine(double* pos, double* vel, int i, double* target) {

}
void particleSimulator::predator_variation(double* pos, double* vel, int i, double* target) {

}
void particleSimulator::predator_override(double* pos, double* vel, int i, double* target) {
	//avoid sides

	double speed;
	speed = sqrt(pow(vel[0], 2) + pow(vel[1], 2) + pow(vel[2], 2));

	vel[0] = vel[0] / speed;
	vel[1] = vel[1] / speed;
	vel[2] = vel[2] / speed;


	if (pos[0] >= 9) {
		if (vel[0] - 1 == 0) {
			vel[0] = -1;
		}
		else {
			vel[0] = vel[0] - 1;
		}
	}if (pos[0] <= -9)
	{
		if (vel[0] + 1 == 0) {
			vel[0] = 1;
		}
		else {
			vel[0] = vel[0] + 1;
		}
	}if (pos[1] >= 9) {
		if (vel[1] - 1 == 0) {
			vel[1] = -1;
		}
		else {
			vel[1] = vel[1] - 1;
		}
	}if (pos[1] <= -9)
	{
		if (vel[1] + 1 == 0) {
			vel[1] = 1;
		}
		else {
			vel[1] = vel[1] + 1;
		}
	}if (pos[2] >= 9) {
		if (vel[2] - 1 == 0) {
			vel[2] = -1;
		}
		else {
			vel[2] = vel[2] - 1;
		}
	}if (pos[2] <= -9)
	{
		if (vel[2] + 1 == 0) {
			vel[2] = 1;
		}
		else {
			vel[2] = vel[2] + 1;
		}
	}

	double length;
	length = sqrt(pow(vel[0],2)+ pow(vel[1], 2)+ pow(vel[2], 2));


	vel[0] = vel[0] / length;
	vel[1] = vel[1] / length;
	vel[2] = vel[2] / length;

	vel[0] = vel[0] * speed;
	vel[1] = vel[1] * speed;
	vel[2] = vel[2] * speed;


	VecCopy(target, vel);
}

//prey behaviour
void particleSimulator::prey_behaviourTree(double* pos, double* vel, int i, double* target) {
	prey_flockingEngine(pos,vel,i,target);
	prey_override(pos,vel,i,vel);
}
void particleSimulator::prey_flockingEngine(double* pos, double* vel, int i, double* target) {
	prey_flocking(pos,vel,i,target);
}
void particleSimulator::prey_flocking(double* pos, double* vel, int i, double* target) {
	Vector separation, cohesion, alignment;
	separation[0] = 0;
	separation[1] = 0;
	separation[2] = 0;

	cohesion[0] = 0;
	cohesion[1] = 0;
	cohesion[2] = 0;

	alignment[0] = 0;
	alignment[1] = 0;
	alignment[2] = 0;

	Vector pos2, vel2, dist;
	char** args = new char* [2];
	int flock_mate = 0;
	double mass2, distance, speed;
	double total_mass = 0.0;


	//normalize given velocity, save the boids speed
	speed = sqrt(pow(vel[0],2)+ pow(vel[1], 2)+ pow(vel[2], 2));
	if (speed != 0) {
		vel[0] = vel[0] / speed;
		vel[1] = vel[1] / speed;
		vel[2] = vel[2] / speed;
	}
	else {
		//animTcl::OutputMessage("in else, vel normalized 1");
	}
	
	for (int j = 0; j < part_num; j++) {


		

		stringstream strs;
		strs << j;
		string temp_str = strs.str();
		char* index = (char*)temp_str.c_str();
		//animTcl::OutputMessage(index);
		args[1] = index;

		args[0] = "getPos";
		args[1] = index;
		m_object->command(2, args);
		m_object->getState(pos2);

		//vector from given point to new point
		dist[0] = pos2[0] - pos[0];
		dist[1] = pos2[1] - pos[1];
		dist[2] = pos2[2] - pos[2];

		//length of vector
		distance = sqrt(pow(dist[0],2)+ pow(dist[1], 2) + pow(dist[2], 2));

		//check if boid is withing neighborhood
		if (distance < 10 && i!=j) {
			//animTcl::OutputMessage("neighborfound");
			flock_mate++;

			//normalize dist
			if (distance != 0) {
				dist[0] = dist[0] / distance;
				dist[1] = dist[1] / distance;
				dist[2] = dist[2] / distance;
			}
			else {
				//animTcl::OutputMessage("in else, distance normalized");
			}
			//add dist to seperation
			separation[0] += dist[0];
			separation[1] += dist[1];
			separation[2] += dist[2];

			args[0] = "getMass";
			m_object->command(2, args);
			m_object->getState(vel2);
			mass2 = vel2[0];
			if (mass2 == -1) {
				continue;
			}

			total_mass += mass2;
			cohesion[0] += mass2 * pos2[0];
			cohesion[1] += mass2 * pos2[1];
			cohesion[2] += mass2 * pos2[2];
			//stringstream strs;
			//strs << mass2;
			//strs << cohesion[1];
			//strs << cohesion[2];
			//string temp_str = strs.str();
			//char* char_type = (char*)temp_str.c_str();
			//animTcl::OutputMessage(char_type);
			//animTcl::OutputMessage("mass");


			args[0] = "getVel";

			m_object->command(2, args);
			m_object->getState(vel2);

			//normalize vel2 to get other boid's bearing
			distance = sqrt(pow(vel2[0], 2) + pow(vel2[1], 2)+ pow(vel2[2], 2));
			if (distance != 0) {
				vel2[0] = vel2[0] / distance;
				vel2[1] = vel2[1] / distance;
				vel2[2] = vel2[2] / distance;
			}
			else {
				///animTcl::OutputMessage("in else, vel2 normalized");
			}
			//add to alignment
			alignment[0] += vel2[0];
			alignment[1] += vel2[1];
			alignment[2] += vel2[2];


			
		}
	}
	//complete cohesion: find center of mass then find direction from point to center of mass
	if (total_mass != 0) {
		cohesion[0] = cohesion[0] / total_mass;
		cohesion[1] = cohesion[1] / total_mass;
		cohesion[2] = cohesion[2] / total_mass;
		Vector center_mass;
		VecCopy(center_mass, cohesion);
		cohesion[0] = center_mass[0] - pos[0];
		cohesion[1] = center_mass[1] - pos[1];
		cohesion[2] = center_mass[2] - pos[2];
	}
	else {
		//animTcl::OutputMessage("in else, center of mass");
		cohesion[0] = 0;
		cohesion[1] = 0;
		cohesion[2] = 0;
	}
	
	distance = sqrt(pow(cohesion[0], 2)+ pow(cohesion[1], 2)+ pow(cohesion[2], 2));

	if (distance != 0) { 
		
		cohesion[0] = cohesion[0] / distance;
		cohesion[1] = cohesion[1] / distance;
		cohesion[2] = cohesion[2] / distance;
	}
	else {
		//animTcl::OutputMessage("in else, cohesion normalize");
	}
	//average the direction sum then normalize
	if (flock_mate != 0) {
		alignment[0] = alignment[0] / flock_mate;
		alignment[1] = alignment[1] / flock_mate;
		alignment[2] = alignment[2] / flock_mate;
	}
	else {
		//animTcl::OutputMessage("in else, alignment average");
	}

	distance = sqrt(pow(alignment[0], 2) + pow(alignment[1], 2) + pow(alignment[2], 2));
	
	if (distance != 0) {
		alignment[0] = alignment[0] / distance;
		alignment[1] = alignment[1] / distance;
		alignment[2] = alignment[2] / distance;
	}
	else {
		//animTcl::OutputMessage("in else, alignment normalize");
	}
	//normalize directions to other boids then multiply by -1
	distance = sqrt(pow(separation[0], 2) + pow(separation[1], 2) + pow(separation[2], 2));
	if (distance != 0) {
		separation[0] = separation[0] / distance;
		separation[1] = separation[1] / distance;
		separation[2] = separation[2] / distance;
	}
	else {
		//animTcl::OutputMessage("in else, seperation normalize");
	}

	separation[0] = separation[0] * -1;
	separation[1] = separation[1] * -1;
	separation[2] = separation[2] * -1;

	vel[0] = vel[0] + cohesion[0] + alignment[0] + separation[0];
	vel[1] = vel[1] + cohesion[1] + alignment[1] + separation[1];
	vel[2] = vel[2] + cohesion[2] + alignment[2] + separation[2];
	if (vel[0]==0&& vel[1] ==0 && vel[2]==0) {
		vel[0] = cohesion[0] + alignment[0] + separation[0];
		vel[1] = cohesion[1] + alignment[1] + separation[1];
		vel[2] = cohesion[2] + alignment[2] + separation[2];
	}

	distance = sqrt(pow(vel[0], 2) + pow(vel[1], 2) + pow(vel[2], 2));
	if (distance!= 0) {
		vel[0] = vel[0] / distance;
		vel[1] = vel[1] / distance;
		vel[2] = vel[2] / distance;
	}
	else {
		//animTcl::OutputMessage("in else, vel normalize");
	}

	
	if (speed > sqrt(3) || speed < 0) {
		speed = sqrt(3);
			
	}
	vel[0] = vel[0] * speed;
	vel[1] = vel[1] * speed;
	vel[2] = vel[2] * speed;
	/*
	stringstream strs;
	strs << vel[0];
	strs << vel[1];
	strs << vel[2];
	strs << speed;
	string temp_str = strs.str();
	char* char_type = (char*)temp_str.c_str();
	animTcl::OutputMessage(char_type);
	animTcl::OutputMessage("in flock");*/
	

	VecCopy(target, vel);



}

void particleSimulator::prey_danceEngine(double* pos, double* vel, int i, double* target) {

}
void particleSimulator::prey_dance(double* pos, double* vel, int i, double* target) {

}
void particleSimulator::prey_searchEngine(double* pos, double* vel, int i, double* target) {

}
void particleSimulator::prey_search(double* pos, double* vel, int i, double* target) {

}
void particleSimulator::prey_variationEngine(double* pos, double* vel, int i, double* target) {

}
void particleSimulator::prey_variation(double* pos, double* vel, int i, double* target) {

}
void particleSimulator::prey_override(double* pos, double* vel, int i, double* target) {
	//avoid sides

	double speed;
	speed = sqrt(pow(vel[0], 2) + pow(vel[1], 2) + pow(vel[2], 2));

	vel[0] = vel[0] / speed;
	vel[1] = vel[1] / speed;
	vel[2] = vel[2] / speed;


	if (pos[0] >= 9) {
		if (vel[0] - 1 == 0) {
			vel[0] = -1;
		}
		else {
			vel[0] = vel[0] - 1;
		}
	}if (pos[0] <= -9)
	{
		if (vel[0] + 1 == 0) {
			vel[0] = 1;
		}
		else {
			vel[0] = vel[0] + 1;
		}
	}if (pos[1] >= 9) {
		if (vel[1] - 1 == 0) {
			vel[1] = -1;
		}
		else {
			vel[1] = vel[1] - 1;
		}
	}if (pos[1] <= -9)
	{
		if (vel[1] + 1 == 0) {
			vel[1] = 1;
		}
		else {
			vel[1] = vel[1] + 1;
		}
	}if (pos[2] >= 9) {
		if (vel[2] - 1 == 0) {
			vel[2] = -1;
		}
		else {
			vel[2] = vel[2] - 1;
		}
	}if (pos[2] <= -9)
	{
		if (vel[2] + 1 == 0) {
			vel[2] = 1;
		}
		else {
			vel[2] = vel[2] + 1;
		}
	}

	//predator avoidance + collision check here
	Vector pred_pos, angelvector;
	double distance;
	int smallest_j = -1;
	double smallest_d = 600;
	for (int j = 0; j < part_num; j++) {
		if (m_object->predator_particles[j][0] != -1) {
			pred_pos[0] = m_object->predator_particles[j][1];
			pred_pos[1] = m_object->predator_particles[j][2];
			pred_pos[2] = m_object->predator_particles[j][3];

			angelvector[0] = pred_pos[0] - pos[0];
			angelvector[1] = pred_pos[1] - pos[1];
			angelvector[2] = pred_pos[2] - pos[2];

			distance = sqrt(pow(angelvector[0], 2)+pow(angelvector[1],2)+ pow(angelvector[2],2));

			if (distance < smallest_d) {
				smallest_d = distance;
				smallest_j = j;
			}
			if (distance <= 1) {
				m_object->particles[i][0] = -1;
			}
		}
		else {
			continue;
		}

	}
	if (smallest_j != -1 && m_object->particles[i][0] != -1 && smallest_d <= 5) {
		pred_pos[0] = m_object->predator_particles[smallest_j][1];
		pred_pos[1] = m_object->predator_particles[smallest_j][2];
		pred_pos[2] = m_object->predator_particles[smallest_j][3];

		angelvector[0] = pos[0] - pred_pos[0];
		angelvector[1] = pos[1] - pred_pos[1];
		angelvector[2] = pos[2] - pred_pos[2];

	

		distance = sqrt(pow(angelvector[0], 2) + pow(angelvector[1], 2) + pow(angelvector[2], 2));

		angelvector[0] = angelvector[0] / distance;
		angelvector[1] = angelvector[1] / distance;
		angelvector[2] = angelvector[2] / distance;


		
		vel[0] = vel[0] + 2*angelvector[0];
		vel[1] = vel[1] + 2*angelvector[1];
		vel[2] = vel[2] + 2*angelvector[2];
		


	}
	double length;
	length = sqrt(pow(vel[0],2)+ pow(vel[1], 2)+ pow(vel[2], 2));

	vel[0] = vel[0] / length;
	vel[1] = vel[1] / length;
	vel[2] = vel[2] / length;

	if (speed >= sqrt(3) || speed < 0) {
		speed = sqrt(3);

	}

	vel[0] = vel[0] * speed;
	vel[1] = vel[1] * speed;
	vel[2] = vel[2] * speed;


	VecCopy(target, vel);
}
double particleSimulator::Force(double* pos, double* vel, int index) {
	//to add spring and collision force soon!
	return (vel[index] * (-1 * kdrag) + mass * gravity[index]) / mass;
}




double particleSimulator::fn(double* pos, double* vel, int index) {
	double Fn;
	Fn = 0;
	double N[3];
	N[0] = 0;
	N[1] = 1;
	N[2] = 0;
	double P[3];
	P[0] = 0;
	P[1] = -11;
	P[2] = 0;
	double dot, dot2;


	//vel[0] = vel[0] / (sqrt(pow((vel[0]), 2) + pow(vel[1], 2)) + pow(vel[2], 2));
	//vel[1] = vel[1] / (sqrt(pow((vel[0]), 2) + pow(vel[1], 2)) + pow(vel[2], 2));
	//vel[2] = vel[2] / (sqrt(pow((vel[0]), 2) + pow(vel[1], 2)) + pow(vel[2], 2));		

	P[0] = pos[0] - P[0];
	P[1] = pos[1] - P[1];
	P[2] = pos[2] - P[2];

	P[0] = P[0] / (sqrt(pow((P[0]), 2) + pow(P[1], 2) + pow(P[2], 2)));
	P[1] = P[1] / (sqrt(pow((P[0]), 2) + pow(P[1], 2) + pow(P[2], 2)));
	P[2] = P[2] / (sqrt(pow((P[0]), 2) + pow(P[1], 2) + pow(P[2], 2)));

	dot = (N[0] * P[0]) + (N[1] * P[1]) + (N[2] * P[2]);
	dot2 = (vel[0] * N[0]) + (vel[1] * N[1]) + (vel[2] * N[2]);
	if (dot < 0 ) {

		Fn = -1 * (ks/1000) * (dot * N[index]) - (kd) * dot2 * N[index];

		/*
		if (Fn<gravity[1]*mass && index == 1) {
			Fn = gravity[1] * mass;
		}
		//Fn = 0;
		*/
		

		//return Fn;
	}
	return Fn/mass;

}

void particleSimulator::Euler(double* pos, double* vel, double time, int i) {
	


	



	vel[0] = vel[0] + time * Force(pos, vel, 0);
	vel[1] = vel[1] + time * Force(pos, vel, 1);
	vel[2] = vel[2] + time * Force(pos, vel, 2);


	double nu_pos[3];
	nu_pos[0] = pos[0] + time * vel[0];
	nu_pos[1] = pos[1] + vel[1] * time;
	nu_pos[2] = pos[2] + vel[2] * time;

	/*
	vel[0] += (fn(nu_pos, vel, 0)) * time;
	vel[1] += (fn(nu_pos, vel, 1)) * time;
	vel[2] += (fn(nu_pos, vel, 2)) * time;
	*/

	pos[0] = pos[0] + vel[0] * time;
	pos[1] = pos[1] + vel[1] * time;
	pos[2] = pos[2] + vel[2] * time;

	
	stringstream strs;
	strs << pos[1];
	string temp_str = strs.str();
	char* char_type = (char*)temp_str.c_str();
	animTcl::OutputMessage(char_type);
	animTcl::OutputMessage("euler");
	
	//   pos[1]= 2 * sin(2*3.14*time) ;
	
	if (pos[1] < -11) {

		
		
		
		vel[1] = -vel[1];
		
	}

}
void particleSimulator::Symplectic(double* pos, double* vel, double time, int i) {
	

	

	/*
	stringstream strs;
	strs << spring_force[0];
	string temp_str = strs.str();
	char* char_type = (char*)temp_str.c_str();
	animTcl::OutputMessage(char_type);
	animTcl::OutputMessage("spring_force");
	*/
	prey_behaviourTree(pos, vel, i, vel);


	double nu_pos[3];
	nu_pos[0] = pos[0] + time * vel[0];
	nu_pos[1] = pos[1] + vel[1] * time;
	nu_pos[2] = pos[2] + vel[2] * time;

	
	double force_n[3];
	force_n[0] = fn(nu_pos, vel, 0);
	force_n[1] = fn(nu_pos, vel, 1);
	force_n[2] = fn(nu_pos, vel, 2);
	/*
	force_n[0] = force_n[0] / (sqrt(pow((force_n[0]), 2) + pow(force_n[1], 2) + pow(force_n[2], 2)));
	force_n[1] = force_n[1] / (sqrt(pow((force_n[0]), 2) + pow(force_n[1], 2) + pow(force_n[2], 2)));
	force_n[2] = force_n[2] / (sqrt(pow((force_n[0]), 2) + pow(force_n[1], 2) + pow(force_n[2], 2)));
	*/



	pos[0] = pos[0] + vel[0] * time;
	pos[1] = pos[1] + vel[1] * time;
	pos[2] = pos[2] + vel[2] * time;

	

	stringstream strs;

	string temp_str;
	char* char_type;
	
	if (pos[1] < -11) {

		vel[1] = -vel[1];
		pos[1] = -11;
		
	}
	if (pos[1] > 11) {
		pos[1] = 11;
		vel[1] = -vel[1];

		
	}
	if (pos[0] < -11) {
		pos[0] = -11;
		vel[0] = -vel[0];
		
	}
	if (pos[0] > 11) {

		vel[0] = -vel[0];
		pos[0] = 11;
		
	}
	if (pos[2] < -11) {
		pos[2] = -11;
		vel[2] = -vel[2];
		
	}
	if (pos[2] > 11) {
		pos[2] = 11;
		vel[2] = -vel[2];
		
	}
	

}
void particleSimulator::predator_Symplectic(double* pos, double* vel, double time, int i) {

	predator_behaviourTree(pos, vel, i, vel);


	double nu_pos[3];
	nu_pos[0] = pos[0] + time * vel[0];
	nu_pos[1] = pos[1] + vel[1] * time;
	nu_pos[2] = pos[2] + vel[2] * time;


	double force_n[3];
	force_n[0] = fn(nu_pos, vel, 0);
	force_n[1] = fn(nu_pos, vel, 1);
	force_n[2] = fn(nu_pos, vel, 2);
	/*
	force_n[0] = force_n[0] / (sqrt(pow((force_n[0]), 2) + pow(force_n[1], 2) + pow(force_n[2], 2)));
	force_n[1] = force_n[1] / (sqrt(pow((force_n[0]), 2) + pow(force_n[1], 2) + pow(force_n[2], 2)));
	force_n[2] = force_n[2] / (sqrt(pow((force_n[0]), 2) + pow(force_n[1], 2) + pow(force_n[2], 2)));
	*/



	pos[0] = pos[0] + vel[0] * time;
	pos[1] = pos[1] + vel[1] * time;
	pos[2] = pos[2] + vel[2] * time;



	stringstream strs;

	string temp_str;
	char* char_type;

	if (pos[1] < -11) {

		vel[1] = -vel[1];
		pos[1] = -11;

	}
	if (pos[1] > 11) {
		pos[1] = 11;
		vel[1] = -vel[1];


	}
	if (pos[0] < -11) {
		pos[0] = -11;
		vel[0] = -vel[0];

	}
	if (pos[0] > 11) {

		vel[0] = -vel[0];
		pos[0] = 11;

	}
	if (pos[2] < -11) {
		pos[2] = -11;
		vel[2] = -vel[2];

	}
	if (pos[2] > 11) {
		pos[2] = 11;
		vel[2] = -vel[2];

	}

}
void particleSimulator::Verlet(double* pos, double* vel, double time, int i) {
	
	

	Vector prevPos;
	if (prevTime == 0) {
		prevPos[0] = pos[0] - vel[0];
		prevPos[1] = pos[1] - vel[1];
		prevPos[2] = pos[2] - vel[2];
	}
	else {
		prevPos[0] = previousPositions[i][0];
		prevPos[1] = previousPositions[i][1];
		prevPos[2] = previousPositions[i][2];
	}

	previousPositions[i][0] = pos[0];	
	previousPositions[i][1] = pos[1];
	previousPositions[i][2] = pos[1];
	pos[0] = 2*pos[0] -( prevPos[0]) + (time)*(Force(pos,vel, 0));
	pos[1] = 2 * pos[1] - (prevPos[1]) + (time)*(Force(pos, vel, 1));
	pos[2] = 2 * pos[2] - (prevPos[2]) + (time)*(Force(pos,vel, 2));
	
	

	
	vel[0] = (pos[0] - prevPos[0])/ (2*time);
	vel[1] = (pos[0] - prevPos[1]) / (2 * time);
	vel[2] = (pos[0] - prevPos[1]) / (2 * time);

	
	stringstream strs;
	strs << prevPos[1];
	string temp_str = strs.str();
	char* char_type = (char*)temp_str.c_str();
	animTcl::OutputMessage(char_type);
	animTcl::OutputMessage("verlet");


	if (pos[1] < -45) {


	
	}
}





int particleSimulator::step(double time)
{
	
		
	Vector pos, vel, pos2, vel2;
	char** args = new char* [2];
	
		

	Vector num;
		
	args[0] = "getPartNum";

	m_object->command(1, args);
	m_object->getState(num);


	part_num = num[0];


	for (int i = 0; i < part_num; i++) {


			
			
		stringstream strs;
		strs << i;
		string temp_str = strs.str();
		char* index = (char*)temp_str.c_str();
			//animTcl::OutputMessage(index);
		args[1] = index;

			
		args[0] = "getMass";
		m_object->command(2, args);
		m_object->getState(vel);
		mass = vel[0];
			
			/*
			
			*/

		args[0] = "getPos";
		args[1] = index;
		m_object->command(2, args);
		m_object->getState(pos);

			
			
			
		args[0] = "getVel";

		m_object->command(2, args);
		m_object->getState(vel);

		
		if (m_object->particles[i][0] != -1) {
			Symplectic(pos, vel, time, i);

		}
		if (m_object->predator_particles[i][0] != -1) {

			pos2[0] = m_object->predator_particles[i][1];
			pos2[1] = m_object->predator_particles[i][2];
			pos2[2] = m_object->predator_particles[i][3];
			vel2[0] = m_object->predator_particles[i][4];
			vel2[1] = m_object->predator_particles[i][5];
			vel2[2] = m_object->predator_particles[i][6];



			predator_Symplectic(pos2, vel2, time, i);

			m_object->predator_particles[i][1] = pos2[0];
			m_object->predator_particles[i][2] = pos2[1];
			m_object->predator_particles[i][3] = pos2[2];
			m_object->predator_particles[i][4] = vel2[0];
			m_object->predator_particles[i][5] = vel2[1];
			m_object->predator_particles[i][6] = vel2[2];
			


		}
	


		args[0] = "setPos";
		//args[1] = "0";

		m_object->setState(pos);
		m_object->command(2, args);

		
		args[0] = "setVel";
		m_object->setState(vel);
		m_object->command(2, args);
		

		prevTime = time;
	}

	
	return 0;

}	// particleSimulator::step
