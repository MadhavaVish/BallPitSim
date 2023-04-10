#ifndef CONSTRAINTS_H
#define CONSTRAINTS_H

#include <vector>
#include <stdio.h>
#include <string>
#include <cstring>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Sparse>

using namespace Eigen;
using namespace std;

typedef enum ConstraintType { DISTANCE, COLLISION } ConstraintType;   //You can expand it for more constraints
typedef enum ConstraintEqualityType { EQUALITY, INEQUALITY } ConstraintEqualityType;

//there is such constraints per two variables that are equal. That is, for every attached vertex there are three such constraints for (x,y,z);
class Constraint {
public:
    Constraint(const ConstraintType _constraintType, const ConstraintEqualityType _constraintEqualityType, const int& _b1, const int& _b2, const double& _invMass1, const double& _invMass2, const RowVector3d& _refVector, const double& _refValue, const double& _CRCoeff) :constraintType(_constraintType), constraintEqualityType(_constraintEqualityType), b1(_b1), b2(_b2), invMass1(_invMass1), invMass2(_invMass2), refValue(_refValue), CRCoeff(_CRCoeff) {
        refVector = _refVector;
    }

    ~Constraint() {}

    int numFullB, numFullC;
    vector<Constraint> constraints;
    void handleCollision(Vector3d& b1, Vector3d& b2, const double& depth, const RowVector3d& contactNormal, const RowVector3d& penPosition, const double CRCoeff, const double tolerance);
    int b1, b2;                     // Two vertices from the respective meshes - auxiliary data for users (constraint class shouldn't use that)
    double invMass1, invMass2;       // Inverse masses of two bodies
    double refValue;                // Reference values to use in the constraint, when needed (like distance)
    RowVector3d refVector;             // Reference vector when needed (like vector)
    double CRCoeff;                 // Extra velocity bias
    ConstraintType constraintType;  // The type of the constraint, and will affect the value and the gradient. This SHOULD NOT change after initialization!
    ConstraintEqualityType constraintEqualityType;  //whether the constraint is an equality or an inequality

    bool resolvePositionConstraint(const MatrixXd& currCOMPositions, const MatrixXd& currConstPositions, MatrixXd& correctedCOMPositions, double tolerance, const float flexCoeff);
    bool resolveVelocityConstraint(const MatrixXd& currCOMPositions, const MatrixXd& currVertexPositions, const MatrixXd& currCOMVelocities, MatrixXd& correctedCOMVelocities, double tolerance, const float flexCoeff);

};
    
#endif /* constraints_h */