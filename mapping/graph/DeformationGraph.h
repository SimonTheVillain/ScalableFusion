/****************************************************************************
**
** Copyright (C) 2019 TU Wien, ACIN, Vision 4 Robotics (V4R) group
** Contact: schreiberhuber@acin.tuwien.ac.at
**
** This file is part of ScalableFusion
**
** V4R is distributed under dual licenses - GPLv3 or closed source.
**
** GNU General Public License Usage
** V4R is free software: you can redistribute it and/or modify
** it under the terms of the GNU General Public License as published
** by the Free Software Foundation, either version 3 of the License, or
** (at your option) any later version.
**
** V4R is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
** GNU General Public License for more details.
**
** Please review the following information to ensure the GNU General Public
** License requirements will be met: https://www.gnu.org/licenses/gpl-3.0.html.
**
**
** Commercial License Usage
** If GPL is not suitable for your project, you must purchase a commercial
** license to use V4R. Licensees holding valid commercial V4R licenses may
** use this file in accordance with the commercial license agreement
** provided with the Software or, alternatively, in accordance with the
** terms contained in a written agreement between you and TU Wien, ACIN, V4R.
** For licensing terms and conditions please contact schreiberhuber@acin.tuwien.ac.at.
**
**
** The copyright holder additionally grants the author(s) of the file the right
** to use, copy, modify, merge, publish, distribute, sublicense, and/or
** sell copies of their contributions without any restrictions.
**
****************************************************************************/
#ifndef FILE_DEFORMATION_GRAPH_H
#define FILE_DEFORMATION_GRAPH_H

#include <vector>

#include <Eigen/Eigen>

using namespace std;
using namespace Eigen;

class PinConstraint {
public:
	Vector3f pos;
	int node;
};

class DeformationGraph {
public:

	void initializeTransforms();

	void generateNeighbourList();

	void generateResiduals();

	void generateJacobian();

	void gaussNewtonStep();

	void applyUpdate();

	float wrot = 1.0f;
	float wreg = 10.0f;
	float wcon = 100.0f;

	vector<PinConstraint> pin_constraints;

	vector<Vector3f> nodes;
	vector<unsigned int> indices;

	vector<vector<unsigned int>> neighbours;

	MatrixXd residuals;

	//lets work with double..... maybe this is a overkill
	SparseMatrix<double> jacobian;

	vector<Matrix3f> G_rot;
	vector<Vector3f> G_t;

};

#endif
