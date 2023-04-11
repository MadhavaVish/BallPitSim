#pragma once

#include <vector>
#include <thread>
#include <glm/glm.hpp>

#include "Particle.hpp"

#define THREAD_COUNT 8

class SPHSolver
{
private:
	//particle data
	unsigned int numParticles;
	bool started;

	//initializes the particles that will be used
	void initParticles();

	// Creates hash table for particles in infinite domain
	void buildTable();
	// Threads and thread blocks
	std::thread threads[THREAD_COUNT];
	int blockBoundaries[THREAD_COUNT + 1];
	int tableBlockBoundaries[THREAD_COUNT + 1];

public:
	SPHSolver(unsigned int numParticles, float mass, float restDensity, float gasConst, float viscosity, float h, float g, float tension);
	~SPHSolver();

	//kernel/fluid constants
	float POLY6, SPIKY_GRAD, SPIKY_LAP, GAS_CONSTANT, MASS, H2, SELF_DENS;

	//fluid properties
	float restDensity;
	float viscosity, h, g, tension;

	std::vector<Particle*> particles;
	Particle** particleTable;
	glm::ivec3 getCell(Particle* p) const;

	void getParticlePositions(std::vector<glm::vec4>& positions);
	// std::mutex mtx;

	//updates the SPH system
	void update(float deltaTime);

	void reset();
	void startSimulation();
};