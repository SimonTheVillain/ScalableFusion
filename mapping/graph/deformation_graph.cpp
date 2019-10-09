#include "deformation_graph.h"

#include <iostream>

#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

void DeformationGraph::initializeTransforms() {
	G_rot.resize(nodes.size());
	G_t.resize(nodes.size());
	for(size_t i = 0; i < nodes.size(); i++) {
		G_rot[i] = Matrix3f::Identity();
		G_t[i].setZero();
	}
}

void DeformationGraph::generateNeighbourList() {
	neighbours.resize(nodes.size());
	for(size_t i = 0; i < indices.size() / 2; i++) {
		int index1 = indices[i * 2 + 0];
		int index2 = indices[i * 2 + 1];
		//connect the neighbours on this level
		neighbours[index1].push_back(index2);
		neighbours[index2].push_back(index1);
	}
}


void DeformationGraph::generateResiduals() {
	int edge_con_count = 0;
	for(int i = 0; i < neighbours.size(); i++) {
		edge_con_count += neighbours[i].size();
	}

	residuals.resize(pin_constraints.size() * 3 + // constraints
	                 nodes.size() * 6 + // rotation 0 + //
	                 edge_con_count * 3 , 1); //regularization 0,1);//e
	residuals.setZero();

	//pin constraints
	float w = sqrt(wcon);
	int ind = 0;
	for(int i = 0; i < pin_constraints.size(); i++) {
		PinConstraint &constraint = pin_constraints[i];
		int k = constraint.node;

		Vector3f R_con = nodes[k] + G_t[k] - constraint.pos;
		R_con = R_con * w;
		residuals(i * 3 + 0) = R_con(0);
		residuals(i * 3 + 1) = R_con(1);
		residuals(i * 3 + 2) = R_con(2);
		ind += 3;
	}

	//rotations
	w = sqrt(wrot);
	for(size_t i = 0; i < nodes.size(); i++) {

		Matrix3f R_rot =  G_rot[i].transpose() * G_rot[i] - Matrix3f::Identity();

		R_rot = w * R_rot;

		//TODO: update the jacobian to reflect this change!!!!!!
		residuals(ind + 0) = R_rot(0, 1); //ab+de+gh
		residuals(ind + 1) = R_rot(0, 2); //ac+df+gi
		residuals(ind + 2) = R_rot(1, 2); //bc+ef+ni
		residuals(ind + 3) = R_rot(0, 0); //aa+dd+gg-1
		residuals(ind + 4) = R_rot(1, 1); //bb+ee+hh-1
		residuals(ind + 5) = R_rot(2, 2); //cc+ff+ii-1

		ind += 6;
	}

	//regularization
	w = sqrt(wreg);
	for(size_t i = 0; i < neighbours.size(); i++) {
		vector<unsigned int> &indices = neighbours[i];
		for(size_t j = 0; j < indices.size(); j++) {
			int l = indices[j];
			Vector3f R_reg = G_rot[i] * (nodes[l] - nodes[i]) + 
			                 nodes[i] + G_t[i] - (nodes[l] + G_t[l]);
			R_reg = w * R_reg;
			residuals(ind + 0) = R_reg(0);
			residuals(ind + 1) = R_reg(1);
			residuals(ind + 2) = R_reg(2);
			ind += 3;
		}
	}
	cout << "this was the residuals" << endl;
}

void DeformationGraph::generateJacobian() {
	int edge_con_count = 0;
	for(int i=0;i<neighbours.size();i++){
		edge_con_count += neighbours[i].size();
	}
	int variable_count = nodes.size() * 12;
	int residual_count =
			pin_constraints.size() * 3 + // pin constraint residuals
			nodes.size() * 6 + //rotation residuals 0 + //
			edge_con_count * 3; //regularization residuals 0;//

	jacobian.resize(residual_count, variable_count);

	//triplets to setup:
	typedef Triplet<double> T;
	vector<T> triplet_list;
	triplet_list.reserve(variable_count * 2);//this should approximately be the size of our vector

	int ind = 0;
	//the derivatives are always:
	// 9x rot  3x translate

	float w = sqrt(wcon);
	for(size_t i = 0; i < pin_constraints.size(); i++) {
		PinConstraint constraint = pin_constraints[i];
		int k = constraint.node;
		triplet_list.push_back(T(ind + 0, k * 12 + 9, w));
		triplet_list.push_back(T(ind + 1, k * 12 + 10, w));
		triplet_list.push_back(T(ind + 2, k * 12 + 11, w));

		ind += 3;
	}

	w = sqrt(wrot);
	for(size_t i = 0; i < nodes.size(); i++){
		Matrix3f &Gr = G_rot[i];

		triplet_list.push_back(T(ind + 0, i * 12 + 0, w * Gr(0, 1)));
		triplet_list.push_back(T(ind + 0, i * 12 + 1, w * Gr(0, 0)));
		triplet_list.push_back(T(ind + 0, i * 12 + 3, w * Gr(1, 1)));
		triplet_list.push_back(T(ind + 0, i * 12 + 4, w * Gr(1, 0)));
		triplet_list.push_back(T(ind + 0, i * 12 + 6, w * Gr(2, 1)));
		triplet_list.push_back(T(ind + 0, i * 12 + 7, w * Gr(2, 0)));

		triplet_list.push_back(T(ind + 1, i * 12 + 0, w * Gr(0, 2)));
		triplet_list.push_back(T(ind + 1, i * 12 + 2, w * Gr(0, 0)));
		triplet_list.push_back(T(ind + 1, i * 12 + 3, w * Gr(1, 2)));
		triplet_list.push_back(T(ind + 1, i * 12 + 5, w * Gr(1, 0)));
		triplet_list.push_back(T(ind + 1, i * 12 + 6, w * Gr(2, 2)));
		triplet_list.push_back(T(ind + 1, i * 12 + 8, w * Gr(2, 0)));

		triplet_list.push_back(T(ind + 2, i * 12 + 1, w * Gr(0, 2)));
		triplet_list.push_back(T(ind + 2, i * 12 + 2, w * Gr(0, 1)));
		triplet_list.push_back(T(ind + 2, i * 12 + 4, w * Gr(1, 2)));
		triplet_list.push_back(T(ind + 2, i * 12 + 5, w * Gr(1, 1)));
		triplet_list.push_back(T(ind + 2, i * 12 + 7, w * Gr(2, 2)));
		triplet_list.push_back(T(ind + 2, i * 12 + 8, w * Gr(2, 1)));

		//now the diagonal blocks (the ones with the squared matrix elements) aa dd gg bb ee hh cc ff ii
		triplet_list.push_back(T(ind + 3, i * 12 + 0, w * 2.0f * Gr(0, 0)));
		triplet_list.push_back(T(ind + 3, i * 12 + 3, w * 2.0f * Gr(1, 0)));
		triplet_list.push_back(T(ind + 3, i * 12 + 6, w * 2.0f * Gr(2, 0)));

		triplet_list.push_back(T(ind + 4, i * 12 + 1, w * 2.0f * Gr(0, 1)));
		triplet_list.push_back(T(ind + 4, i * 12 + 4, w * 2.0f * Gr(1, 1)));
		triplet_list.push_back(T(ind + 4, i * 12 + 7, w * 2.0f * Gr(2, 1)));

		triplet_list.push_back(T(ind + 5, i * 12 + 2, w * 2.0f * Gr(0, 2)));
		triplet_list.push_back(T(ind + 5, i * 12 + 5, w * 2.0f * Gr(1, 2)));
		triplet_list.push_back(T(ind + 5, i * 12 + 8, w * 2.0f * Gr(2, 2)));

		ind += 6;
	}
   

	w = sqrt(wreg);
	for(size_t i = 0; i < neighbours.size(); i++) {
		vector<unsigned int> &indices = neighbours[i];

		for(size_t j = 0; j < indices.size(); j++) {
			//Ereg^l,n derived by Gt^l
			triplet_list.push_back(T(ind + 0, i * 12 + 9, w));
			triplet_list.push_back(T(ind + 1, i * 12 + 10, w));
			triplet_list.push_back(T(ind + 2, i * 12 + 11, w));

			int k = indices[j];
			//Ereg^l,n derived by Gt^n
			triplet_list.push_back(T(ind + 0, k * 12 + 9, -w));
			triplet_list.push_back(T(ind + 1, k * 12 + 10, -w));
			triplet_list.push_back(T(ind + 2, k * 12 + 11, -w));

			Vector3f delta = nodes[k]-nodes[i];
			//Ereg^l,n derived by Gr^l

			triplet_list.push_back(T(ind + 0, i * 12 + 0*3 + 0, w * delta(0)));
			triplet_list.push_back(T(ind + 0, i * 12 + 0*3 + 1, w * delta(1)));
			triplet_list.push_back(T(ind + 0, i * 12 + 0*3 + 2, w * delta(2)));

			triplet_list.push_back(T(ind + 1, i * 12 + 1*3 + 0, w * delta(0)));
			triplet_list.push_back(T(ind + 1, i * 12 + 1*3 + 1, w * delta(1)));
			triplet_list.push_back(T(ind + 1, i * 12 + 1*3 + 2, w * delta(2)));

			triplet_list.push_back(T(ind + 2, i * 12 + 2*3 + 0, w * delta(0)));
			triplet_list.push_back(T(ind + 2, i * 12 + 2*3 + 1, w * delta(1)));
			triplet_list.push_back(T(ind + 2, i * 12 + 2*3 + 2, w * delta(2)));

			ind += 3;
		}
	}

	cout << "jacobian rows" << jacobian.rows() << " cols " << jacobian.cols() << endl;
	jacobian.setFromTriplets(triplet_list.begin(), triplet_list.end());
}

void DeformationGraph::gaussNewtonStep() {

	//https://forum.kde.org/viewtopic.php?f=74&t=98911

	SparseMatrix<double> J = jacobian;
	SparseMatrix<double> Jt = J.transpose();

	MatrixXd r = residuals;
	cout << "rows" << r.rows() << " cols " << r.cols() << endl;
	auto Jtr = Jt * r;

	auto JtJ = Jt * J;
	cout << "rows" << JtJ.rows() << " cols " << JtJ.cols() << endl;

	SimplicialLLT<SparseMatrix<double>> decomposer(JtJ);

	VectorXd step = decomposer.solve(Jtr);

	//applying step
	for(int i = 0; i < nodes.size(); i++) {

		G_rot[i](0,0) -= step(i * 12 + 0);
		G_rot[i](0,1) -= step(i * 12 + 1);
		G_rot[i](0,2) -= step(i * 12 + 2);

		G_rot[i](1,0) -= step(i * 12 + 3);
		G_rot[i](1,1) -= step(i * 12 + 4);
		G_rot[i](1,2) -= step(i * 12 + 5);

		G_rot[i](2,0) -= step(i * 12 + 6);
		G_rot[i](2,1) -= step(i * 12 + 7);
		G_rot[i](2,2) -= step(i * 12 + 8);

		for(int j = 0; j < 9; j++) {
			G_rot[i](j) -= step(i * 12 + j);
		}
		for(int j = 0; j < 3; j++) {
			G_t[i](j) -= step(i*12 + 9 + j);
		}
	}
}