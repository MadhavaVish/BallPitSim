#include "Constraint.hpp"

//computes the impulse needed for all particles to resolve the velocity constraint, and corrects the velocities accordingly.
//The velocities are a vector (vCOM1, w1, vCOM2, w2) in both input and output.
//returns true if constraint was already valid with "currVelocities", and false otherwise (false means there was a correction done)
//currCOMPositions is a 2x3 matrix, where each row is per one of the sides of the constraints; the rest of the relevant variables are similar, and so should the outputs be resized.
bool Constraint::resolveVelocityConstraint(const MatrixXd& currBallPositions, const MatrixXd& currConstPositions, const MatrixXd& currBallVelocities, MatrixXd& correctedBallVelocities, double tolerance) {

    /**************
    1. If the velocity Constraint is satisfied up to tolerate ("abs(Jv)<=tolerance"), set corrected values to original ones and return true

    2. Otherwise, correct linear and angular velocities as learnt in class.

    Note to differentiate between different constraint types; for inequality constraints you don't do anything unless it's unsatisfied.
    ***************/


    Matrix<double, 6, 6> invMassMatrix = Matrix<double, 6, 6>::Zero();

    for (int i = 0; i < 3; i++) {
        invMassMatrix(i, i) = this->invMass1;
    }
    for (int i = 3; i < 6; i++) {
        invMassMatrix(i, i) = this->invMass2;
    }

    RowVector3d b1 = currBallPositions.row(0);
    RowVector3d b2 = currBallPositions.row(1);

    RowVector3d v1 = currBallVelocities.row(0);
    RowVector3d v2 = currBallVelocities.row(1);

    RowVector<double, 6> j = RowVector<double, 6>::Zero();
    
    j << refVector, -refVector;

    RowVectorXd v = RowVectorXd::Zero(6);
    v << v1, v2;
    if (constraintType == COLLISION) {
        if (abs(j.dot(v)) <= tolerance) {
            correctedBallVelocities = currBallVelocities;
            return true;
        }
    }
    else if (constraintType == DISTANCE) {
        double dist = (b1 - b2).stableNorm();
        if ((dist - refValue) <  refValue) {
            correctedBallVelocities = currBallVelocities;
            return true;
        }
    }

    double num = (j * v.transpose())(0, 0);
    double denom = (j * invMassMatrix * j.transpose())(0, 0);
    double lagrange = 0;
    if (constraintType == COLLISION) {
        lagrange = -(1 + CRCoeff) * num / denom;
    }
    else if (constraintType == DISTANCE) {
        lagrange = -1 * num / denom;
    }

    Vector<double, 6> deltaV = Vector<double, 6>::Zero(); deltaV = lagrange * invMassMatrix * j.transpose();

    RowVector3d deltaV1; deltaV1 << deltaV(0), deltaV(1), deltaV(2);
    RowVector3d deltaV2; deltaV2 << deltaV(3), deltaV(4), deltaV(5);

    correctedBallVelocities = MatrixXd::Zero(2, 3);
    correctedBallVelocities << (v1 + deltaV1), (v2 + deltaV2);

    //Debug purpose
    RowVector3d v1corr = correctedBallVelocities.row(0);
    RowVector3d v2corr = correctedBallVelocities.row(1);

    return false;
}

//projects the position unto the constraint
//returns true if constraint was already valid with "currPositions"
bool Constraint::resolvePositionConstraint(const MatrixXd& currBallPositions, const MatrixXd& currConstPositions, MatrixXd& correctedBallPositions, double tolerance) {

    /**************
        TODO: write position correction procedure:
        1. If the position Constraint is satisfied up to tolerate ("abs(C(p)<=tolerance"), set corrected values to original ones and return true

        2. Otherwise, correct COM position as learnt in class. Note that since this is a linear correction, correcting COM position == correcting all positions the same offset. the currConstPositions are used to measure the constraint, and the COM values are corrected accordingly to create the effect.

        Note to differentiate between different constraint types; for inequality constraints you don't do anything unless it's unsatisfied.
        ***************/

    Matrix<double, 6, 6> invMassMatrix = Matrix<double, 6, 6>::Zero();

    for (int i = 0; i < 3; i++) {
        invMassMatrix(i, i) = this->invMass1;
    }
    for (int i = 3; i < 6; i++) {
        invMassMatrix(i, i) = this->invMass2;
    }

    RowVector3d com1 = currBallPositions.row(0);
    RowVector3d com2 = currBallPositions.row(1);

    RowVector3d r1 = currConstPositions.row(0);
    RowVector3d r2 = currConstPositions.row(1);

    RowVector3d n = r1 - r2;

    RowVector<double, 6> j = RowVector<double, 6>::Zero();

    if (constraintType == COLLISION) {
        j << refVector, -refVector;
    }
    else if (constraintType == DISTANCE) {
        j << n.normalized(), -n.normalized();
    }

    double num;
    if (constraintType == COLLISION) {
        num = refValue;
        if (refValue <= tolerance) {
            correctedBallPositions = currBallPositions;
            return true;
        }
    }
    else if (constraintType == DISTANCE) {
        num = n.stableNorm() - refValue;
        if (abs(num) <= tolerance) {
            correctedBallPositions = currBallPositions;
            return true;
        }
    }

    double denom = (j * invMassMatrix * j.transpose())(0, 0);
    double lagrange = -1 * num / denom;

    MatrixXd deltaR = MatrixXd::Zero(1, 6); deltaR = lagrange * invMassMatrix * j.transpose();

    Eigen::RowVector3d deltaR1; deltaR1 << deltaR(0), deltaR(1), deltaR(2);
    RowVector3d deltaR2; deltaR2 << deltaR(3), deltaR(4), deltaR(5);

    correctedBallPositions = MatrixXd::Zero(2, 3);
    // (Heuristic) Push back in the middle of the tolerance space along this axis 
    correctedBallPositions << (com1 + deltaR1 * (1 - tolerance/2)), (com2 + deltaR2 * (1 - tolerance/2));
    return false;
}




