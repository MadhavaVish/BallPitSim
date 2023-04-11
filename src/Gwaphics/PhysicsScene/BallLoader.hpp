#pragma once
#include <vector>
#include <stdio.h>
#include <string>
#include <cstring>
#include <iostream>
#include <Eigen/Dense>
#include <glm/glm.hpp>

using namespace std;
using namespace Eigen;

class BallLoader
{
public:
	BallLoader(char const* filename);
	vector<float> getBall(uint32_t i);
	vector<RowVector3d> balls;
	vector<RowVector3d> normals;
	vector<RowVector2i> constraints;
	void write(char const* filename);
};