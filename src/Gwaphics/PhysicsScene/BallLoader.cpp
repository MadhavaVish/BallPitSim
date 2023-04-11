#include "BallLoader.hpp"

// https://github.com/opengl-tutorials/ogl/blob/master/common/objloader.cpp
BallLoader::BallLoader(char const* path)
{	
	printf("Loading ball file %s...\n", path);

	std::vector<unsigned int> vertexIndices, uvIndices, normalIndices;


	FILE* file = fopen(path, "r");
	if (file == NULL) {
		printf("Impossible to open the file !\n");
		getchar();
	}

	while (1) {
		char lineHeader[128];
		// read the first word of the line
		int res = fscanf(file, "%s", lineHeader);
		if (res == EOF)
			break; // EOF = End Of File. Quit the loop.

		// else : parse lineHeader
		if (strcmp(lineHeader, "b") == 0) {
			float x, y, z;
			fscanf(file, "%f %f %f\n", &x, &y, &z);
			RowVector3d ball(x, y, z);
			balls.push_back(ball);
		}
		else if (strcmp(lineHeader, "bn") == 0) {
			float x, y, z;
			fscanf(file, "%f %f %f\n", &x, &y, &z);
			RowVector3d ball(x, y, z);
			normals.push_back(ball);
		}
		else if (strcmp(lineHeader, "c") == 0) {
			int a, b;
			fscanf(file, "%d %d\n", &a, &b);
			RowVector2i constraint(a,b);
			constraints.push_back(constraint);
		}
		else {
			// Probably a comment, eat up the rest of the line
			char stupidBuffer[1000];
			fgets(stupidBuffer, 1000, file);
		}

	}
	printf("Ball file %s loaded.\n", path);
	fclose(file);
}

void BallLoader::write(char const* filename) {
	FILE* file = fopen(filename, "w");

	//open file in write mode
	if (file == NULL) {
		printf("Impossible to open the file !\n");
		getchar();
	}
	std::cout << "\nFile created successfully." << std::endl;

	//write into file
	for (int i = 0; i < balls.size(); i++) {
		fprintf(file, "b %f %f %f\n", balls[i][0], balls[i][1], balls[i][2]);
	}

	for (int i = 0; i < normals.size(); i++) {
		fprintf(file, "bn %f %f %f\n", normals[i][0], normals[i][1], normals[i][2]);
	}

	for (int i = 0; i < constraints.size(); i++) {
		fprintf(file, "c %d %d\n", constraints[i][0], constraints[i][1]);
	}

	fclose(file);
}