#include "SPHSolver.hpp"

#include <iostream>
#include <cstdlib>
#include <cstring>
#include <mutex>

#include <glm/gtx/string_cast.hpp>
#include <glm/gtx/norm.hpp>

#define PI 3.14159265f
#define TABLE_SIZE 1000000

// This will lock across all instances of SPHSystem's,
// however since we only really have one instance, this
// should be okay for now  
std::mutex mtx;

/**
 * Hashes the position of a cell, giving where
 * the cell goes in the particle table
 */
unsigned int getHash(const glm::ivec3& cell) {
	return (
		(unsigned int)(cell.x * 73856093)
		^ (unsigned int)(cell.y * 19349663)
		^ (unsigned int)(cell.z * 83492791)
		) % TABLE_SIZE;
}

SPHSolver::SPHSolver(unsigned int numParticles, float mass, float restDensity, float gasConst, float viscosity, float h, float g, float tension) {
	this->numParticles = numParticles;
	this->restDensity = restDensity;
	this->viscosity = viscosity;
	this->h = h;
	this->g = g;
	this->tension = tension;

	POLY6 = 315.0f / (64.0f * PI * pow(h, 9));
	SPIKY_GRAD = -45.0f / (PI * pow(h, 6));
	SPIKY_LAP = 45.0f / (PI * pow(h, 6));
	MASS = mass;
	GAS_CONSTANT = gasConst;
	H2 = h * h;
	SELF_DENS = MASS * POLY6 * pow(h, 6);

	//setup densities & volume
	int cbNumParticles = numParticles * numParticles * numParticles;
	particles.resize(cbNumParticles);

	//initialize particles
	initParticles();

	//start init
	started = false;

	// Allocate table memory
	particleTable = (Particle**)malloc(sizeof(Particle*) * TABLE_SIZE);

	// Init block boundaries (for funcs that loop through particles)
	blockBoundaries[0] = 0;
	int blockSize = particles.size() / THREAD_COUNT;
	for (int i = 1; i < THREAD_COUNT; i++) {
		blockBoundaries[i] = i * blockSize;
	}
	blockBoundaries[THREAD_COUNT] = particles.size();

	// Init table block boundaries (for table clearing func)
	tableBlockBoundaries[0] = 0;
	blockSize = TABLE_SIZE / THREAD_COUNT;
	for (int i = 1; i < THREAD_COUNT; i++) {
		tableBlockBoundaries[i] = i * blockSize;
	}
	tableBlockBoundaries[THREAD_COUNT] = TABLE_SIZE;
}

SPHSolver::~SPHSolver() {
	// free table
	free(particleTable);

	//delete particles
	particles.clear();
	particles.shrink_to_fit();

	//delete neighbouring particles
}

void SPHSolver::initParticles() {
	std::srand(1024);
	float particleSeperation = h + 0.01f;
	for (int i = 0; i < numParticles; i++) {
		for (int j = 0; j < numParticles; j++) {
			for (int k = 0; k < numParticles; k++) {
				// dam like particle positions
				float ranX = (float(rand()) / float((RAND_MAX)) * 0.5f - 1) * h / 10;
				float ranY = (float(rand()) / float((RAND_MAX)) * 0.5f - 1) * h / 10;
				float ranZ = (float(rand()) / float((RAND_MAX)) * 0.5f - 1) * h / 10;
				glm::vec3 nParticlePos = glm::vec3(i * particleSeperation + ranX - 1.5f, j * particleSeperation + ranY + h + 0.1f, k * particleSeperation + ranZ - 1.5f);

				//create new particle
				Particle* nParticle = new Particle(MASS, h, nParticlePos, glm::vec3(0));

				//append particle
				particles[i + (j + numParticles * k) * numParticles] = nParticle;
			}
		}
	}
}

/**
 * Parallel computation function for calculating density
 * and pressures of particles in the given SPH System.
 */
void parallelDensityAndPressures(const SPHSolver& sphSystem, int start, int end) {
	float massPoly6Product = sphSystem.MASS * sphSystem.POLY6;

	for (int i = start; i < end; i++) {
		float pDensity = 0;
		Particle* pi = sphSystem.particles[i];
		glm::ivec3 cell = sphSystem.getCell(pi);

		for (int x = -1; x <= 1; x++) {
			for (int y = -1; y <= 1; y++) {
				for (int z = -1; z <= 1; z++) {
					glm::ivec3 near_cell = cell + glm::ivec3(x, y, z);
					unsigned int index = getHash(near_cell);
					Particle* pj = sphSystem.particleTable[index];

					// Iterate through cell linked list
					while (pj != NULL) {
						float dist2 = glm::length2(pj->position - pi->position);
						if (dist2 < sphSystem.H2 && pi != pj) {
							pDensity += massPoly6Product * glm::pow(sphSystem.H2 - dist2, 3);
						}
						pj = pj->next;
					}
				}
			}
		}

		// Include self density (as itself isn't included in neighbour)
		pi->density = pDensity + sphSystem.SELF_DENS;

		// Calculate pressure
		float pPressure = sphSystem.GAS_CONSTANT * (pi->density - sphSystem.restDensity);
		pi->pressure = pPressure;
	}
}

/**
 * Parallel computation function for calculating forces
 * of particles in the given SPH System.
 */
void parallelForces(const SPHSolver& sphSystem, int start, int end) {
	for (int i = start; i < end; i++) {
		Particle* pi = sphSystem.particles[i];
		pi->force = glm::vec3(0);
		glm::ivec3 cell = sphSystem.getCell(pi);

		for (int x = -1; x <= 1; x++) {
			for (int y = -1; y <= 1; y++) {
				for (int z = -1; z <= 1; z++) {
					glm::ivec3 near_cell = cell + glm::ivec3(x, y, z);
					unsigned int index = getHash(near_cell);
					Particle* pj = sphSystem.particleTable[index];

					// Iterate through cell linked list
					while (pj != NULL) {
						float dist2 = glm::length2(pj->position - pi->position);
						if (dist2 < sphSystem.H2 && pi != pj) {
							//unit direction and length
							float dist = sqrt(dist2);
							glm::vec3 dir = glm::normalize(pj->position - pi->position);

							//apply pressure force
							glm::vec3 pressureForce = -dir * sphSystem.MASS * (pi->pressure + pj->pressure) / (2 * pj->density) * sphSystem.SPIKY_GRAD;
							pressureForce *= std::pow(sphSystem.h - dist, 2);
							pi->force += pressureForce;

							//apply viscosity force
							glm::vec3 velocityDif = pj->velocity - pi->velocity;
							glm::vec3 viscoForce = sphSystem.viscosity * sphSystem.MASS * (velocityDif / pj->density) * sphSystem.SPIKY_LAP * (sphSystem.h - dist);
							pi->force += viscoForce;
						}
						pj = pj->next;
					}
				}
			}
		}
	}
}

/**
 * Parallel computation function moving positions
 * of particles in the given SPH System.
 */
void parallelUpdateParticlePositions(const SPHSolver& sphSystem, float deltaTime, int start, int end) {
	for (int i = start; i < end; i++) {
		Particle* p = sphSystem.particles[i];

		//calculate acceleration and velocity
		glm::vec3 acceleration = p->force / p->density + glm::vec3(0, sphSystem.g, 0);
		p->velocity += acceleration * deltaTime;

		// Update position
		p->position += p->velocity * deltaTime;

		// Handle collisions with box
		float boxWidth = 1.5f;
		float elasticity = 0.5f;
		if (p->position.y < p->size) {
			p->position.y = -p->position.y + 2 * p->size + 0.0001f;
			p->velocity.y = -p->velocity.y * elasticity;
		}

		if (p->position.x < p->size - boxWidth) {
			p->position.x = -p->position.x + 2 * (p->size - boxWidth) + 0.0001f;
			p->velocity.x = -p->velocity.x * elasticity;
		}

		if (p->position.x > -p->size + boxWidth) {
			p->position.x = -p->position.x + 2 * -(p->size - boxWidth) - 0.0001f;
			p->velocity.x = -p->velocity.x * elasticity;
		}

		if (p->position.z < p->size - boxWidth) {
			p->position.z = -p->position.z + 2 * (p->size - boxWidth) + 0.0001f;
			p->velocity.z = -p->velocity.z * elasticity;
		}

		if (p->position.z > -p->size + boxWidth) {
			p->position.z = -p->position.z + 2 * -(p->size - boxWidth) - 0.0001f;
			p->velocity.z = -p->velocity.z * elasticity;
		}
	}
}

void SPHSolver::update(float deltaTime) {
	if (!started) return;

	// To increase system stability, a fixed deltaTime is set
	deltaTime = 0.003f;

	// Build particle hash table
	buildTable();

	// Calculate densities and pressures of particles
	for (int i = 0; i < THREAD_COUNT; i++) {
		threads[i] = std::thread(parallelDensityAndPressures, std::ref(*this), blockBoundaries[i], blockBoundaries[i + 1]);
	}
	for (std::thread& thread : threads) {
		thread.join();
	}

	// Calclulate forces of particles
	for (int i = 0; i < THREAD_COUNT; i++) {
		threads[i] = std::thread(parallelForces, std::ref(*this), blockBoundaries[i], blockBoundaries[i + 1]);
	}
	for (std::thread& thread : threads) {
		thread.join();
	}

	// Update positions of all particles
	for (int i = 0; i < THREAD_COUNT; i++) {
		threads[i] = std::thread(parallelUpdateParticlePositions, std::ref(*this), deltaTime, blockBoundaries[i], blockBoundaries[i + 1]);
	}
	for (std::thread& thread : threads) {
		thread.join();
	}
}

/**
 * Parallel helper for clearing table
 */
void tableClearHelper(SPHSolver& sphSystem, int start, int end) {
	for (int i = start; i < end; i++) {
		sphSystem.particleTable[i] = NULL;
	}
}

/**
 * Parallel helper for building table
 */
void buildTableHelper(SPHSolver& sphSystem, int start, int end) {
	for (int i = start; i < end; i++) {
		Particle* pi = sphSystem.particles[i];

		// Calculate hash index using hashing formula
		unsigned int index = getHash(sphSystem.getCell(pi));

		// Setup linked list if need be
		mtx.lock();
		if (sphSystem.particleTable[index] == NULL) {
			pi->next = NULL;
			sphSystem.particleTable[index] = pi;
		}
		else {
			pi->next = sphSystem.particleTable[index];
			sphSystem.particleTable[index] = pi;
		}
		mtx.unlock();
	}
}

void SPHSolver::buildTable() {
	// Parallel empty the table
	for (int i = 0; i < THREAD_COUNT; i++) {
		threads[i] = std::thread(tableClearHelper, std::ref(*this), tableBlockBoundaries[i], tableBlockBoundaries[i + 1]);
	}
	for (std::thread& thread : threads) {
		thread.join();
	}

	// Parallel build table
	for (int i = 0; i < THREAD_COUNT; i++) {
		threads[i] = std::thread(buildTableHelper, std::ref(*this), blockBoundaries[i], blockBoundaries[i + 1]);
	}
	for (std::thread& thread : threads) {
		thread.join();
	}
}

void SPHSolver::getParticlePositions(std::vector<glm::vec4>& positions)
{
	for (int i = 0; i < particles.size(); ++i)
	{
		glm::vec3 pos = particles[i]->position;
		pos.y = -pos.y;
		positions[i] = glm::vec4(pos, 0.f);
	}
}

glm::ivec3 SPHSolver::getCell(Particle* p) const {
	return glm::ivec3(p->position.x / h, p->position.y / h, p->position.z / h);
}

void SPHSolver::reset() {
	initParticles();
	started = false;
}

void SPHSolver::startSimulation() {
	started = true;
}