
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

#include "deformation_node.h"

using namespace std;
using namespace Eigen;

template <typename T> 
inline int sgn(T val) {
	return (T(0) < val) - (val < T(0));
}

void DeformationNode::findNeighbours(
		const Vector3f &pos, const cv::Vec2f &pix_pos,
		const vector<DeformationNode::NodePixPos> &nodes) {

	int quadrants[4][2] = {{1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
	int max_edges = 2;
	for(auto node : nodes) {
		if(node.node.get() == this){
			continue;
		}
		cv::Vec2f delta = pix_pos - node.pix_pos;
		float dist = sqrt(delta.ddot(delta));
		for(size_t i = 0; i < 4; i++) {
			if(sgn(delta[0]) == quadrants[i][0] &&
			   sgn(delta[1]) == quadrants[i][1]) {
				map<float, DeformationNode::WeakNodeDist> &neighbour_map = neighbours[i];
				if(neighbour_map.size()>= max_edges){
					auto last = --neighbour_map.end();
					if(last->first > dist) {
						//replace the element
						neighbour_map.erase(last);//remove old one!!!
						//add new one
						DeformationNode::WeakNodeDist new_node;
						new_node.dist = dist;//TODO: check if this distance is needed
						new_node.node = node.node;
						neighbour_map[dist] = new_node;
					}

				} else {
					//if the neighbour map is empty we just add a new neighbour
					DeformationNode::WeakNodeDist new_node;
					new_node.dist = dist;//TODO: check if this distance is needed
					new_node.node = node.node;
					neighbour_map[dist] = new_node;
				}
			}
		}
	}
}