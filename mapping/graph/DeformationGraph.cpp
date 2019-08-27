//
// Created by simon on 3/29/19.
//

#include "DeformationGraph.h"
#include <Eigen/Dense>
#include <iostream>
#include<Eigen/SparseCholesky>

using namespace std;
using namespace Eigen;


void DeformationGraph::initializeTransforms() {
    G_rot.resize(nodes.size());
    G_t.resize(nodes.size());
    for(size_t i=0;i<nodes.size ();i++){
        G_rot[i] = Eigen::Matrix3f::Identity();
        G_t[i].setZero();
    }
}

void DeformationGraph::generateNeighbourList() {
    neighbours.resize(nodes.size());
    for(size_t i=0;i<indices.size()/2; i++){
        int index1 = indices[i*2 + 0];
        int index2 = indices[i*2 + 1];
        //connect the neighbours on this level
        neighbours[index1].push_back(index2);
        neighbours[index2].push_back(index1);
    }

}


void DeformationGraph::generateResiduals() {

    Vector3f bla;
    Vector3f pla;
    bla.dot(pla);
    int edgeConCount = 0;
    for(int i=0;i<neighbours.size();i++){
        edgeConCount += neighbours[i].size();
    }

    residuals.resize(
            pinConstraints.size()*3 + // constraints
            nodes.size() * 6 + // rotation 0 + //
            edgeConCount * 3 ,1); //regularization 0,1);//e
    residuals.setZero();
    //pin constraints
    float w = sqrt(wcon);
    int ind = 0;
    for(int i=0;i<pinConstraints.size();i++){
        PinConstraint &constraint = pinConstraints[i];
        int k =constraint.node;

        Vector3f R_con = nodes[k] + G_t[k] - constraint.pos;
        R_con = R_con * w;
        residuals(i*3 + 0) = R_con(0);
        residuals(i*3 + 1) = R_con(1);
        residuals(i*3 + 2) = R_con(2);
        ind += 3;
    }
    //int ind = pinConstraints.size()*3;


    //rotations
    w = sqrt(wrot);
    for(size_t i=0;i<nodes.size();i++){

        Matrix3f R_rot =  G_rot[i].transpose() * G_rot[i] - Matrix3f::Identity();

        R_rot = w * R_rot;
        //TODO: update the jacobian to reflect this change!!!!!!
        residuals(ind + 0) = R_rot(0,1); //ab+de+gh
        residuals(ind + 1) = R_rot(0,2); //ac+df+gi
        residuals(ind + 2) = R_rot(1,2); //bc+ef+ni
        residuals(ind + 3) = R_rot(0,0); //aa+dd+gg - 1
        residuals(ind + 4) = R_rot(1,1); //bb+ee+hh-1
        residuals(ind + 5) = R_rot(2,2); //cc+ff+ii-1

        ind += 6;

    }


    //regularization
    w = sqrt(wreg);
    for(size_t i=0;i<neighbours.size();i++){
        vector<unsigned int> &indices = neighbours[i];
        for(size_t j = 0;j<indices.size();j++){
            int l = indices[j];
            Vector3f R_reg = G_rot[i] * (nodes[l] - nodes[i]) + nodes[i] + G_t[i] - (nodes[l] + G_t[l]);
            //Vector3f R_reg = (nodes[l] - nodes[i]) + nodes[i] + G_t[i] - (nodes[l] + G_t[l]); //TODO: remove this, This is regularization without rotation
            //cout << R_reg << endl; probably much smaller with double precision
            R_reg = w * R_reg;
            residuals(ind + 0) = R_reg(0);
            residuals(ind + 1) = R_reg(1);
            residuals(ind + 2) = R_reg(2);
            ind += 3;
        }
    }




    //cout << "residuals" << endl << residuals << endl;

    cout << "this was the residuals" << endl;


}

void DeformationGraph::generateJacobian() {
    int edgeConCount = 0;
    for(int i=0;i<neighbours.size();i++){
        edgeConCount += neighbours[i].size();
    }
    int variableCount = nodes.size() * 12;
    int residualCount =
            pinConstraints.size() * 3 + // pin constraint residuals
            nodes.size() * 6 + //rotation residuals 0 + //
            edgeConCount * 3; //regularization residuals 0;//

    jacobian.resize(residualCount,variableCount);

    //triplets to setup:
    typedef Triplet<double> T;
    vector<T> tripletList;
    tripletList.reserve(variableCount*2);//this should approximately be the size of our vector

    int ind = 0;
    //the derivatives are always:
    // 9x rot  3x translate


    float w = sqrt(wcon);
    for(size_t i=0;i<pinConstraints.size();i++){
        PinConstraint constraint = pinConstraints[i];
        int k = constraint.node;
        tripletList.push_back(T(ind + 0, k * 12 + 9,w));
        tripletList.push_back(T(ind + 1, k * 12 + 10,w));
        tripletList.push_back(T(ind + 2, k * 12 + 11,w));

        ind += 3;
    }


    w = sqrt(wrot);
    for(size_t i=0;i<nodes.size();i++){
        Matrix3f &Gr = G_rot[i];
        //cout << Gr << endl;
        //cout << " debuggidibug" << endl;

        tripletList.push_back(T(ind + 0, i * 12 + 0,w * Gr(0,1)));
        tripletList.push_back(T(ind + 0, i * 12 + 1,w * Gr(0,0)));
        tripletList.push_back(T(ind + 0, i * 12 + 3,w * Gr(1,1)));
        tripletList.push_back(T(ind + 0, i * 12 + 4,w * Gr(1,0)));
        tripletList.push_back(T(ind + 0, i * 12 + 6,w * Gr(2,1)));
        tripletList.push_back(T(ind + 0, i * 12 + 7,w * Gr(2,0)));

        tripletList.push_back(T(ind + 1, i * 12 + 0,w * Gr(0,2)));
        tripletList.push_back(T(ind + 1, i * 12 + 2,w * Gr(0,0)));
        tripletList.push_back(T(ind + 1, i * 12 + 3,w * Gr(1,2)));
        tripletList.push_back(T(ind + 1, i * 12 + 5,w * Gr(1,0)));
        tripletList.push_back(T(ind + 1, i * 12 + 6,w * Gr(2,2)));
        tripletList.push_back(T(ind + 1, i * 12 + 8,w * Gr(2,0)));

        tripletList.push_back(T(ind + 2, i * 12 + 1,w * Gr(0,2)));
        tripletList.push_back(T(ind + 2, i * 12 + 2,w * Gr(0,1)));
        tripletList.push_back(T(ind + 2, i * 12 + 4,w * Gr(1,2)));
        tripletList.push_back(T(ind + 2, i * 12 + 5,w * Gr(1,1)));
        tripletList.push_back(T(ind + 2, i * 12 + 7,w * Gr(2,2)));
        tripletList.push_back(T(ind + 2, i * 12 + 8,w * Gr(2,1)));


        //now the diagonal blocks (the ones with the squared matrix elements) aa dd gg bb ee hh cc ff ii

        tripletList.push_back(T(ind + 3, i * 12 + 0,w * 2.0f * Gr(0,0)));
        tripletList.push_back(T(ind + 3, i * 12 + 3,w * 2.0f * Gr(1,0)));
        tripletList.push_back(T(ind + 3, i * 12 + 6,w * 2.0f * Gr(2,0)));

        tripletList.push_back(T(ind + 4, i * 12 + 1,w * 2.0f * Gr(0,1)));
        tripletList.push_back(T(ind + 4, i * 12 + 4,w * 2.0f * Gr(1,1)));
        tripletList.push_back(T(ind + 4, i * 12 + 7,w * 2.0f * Gr(2,1)));

        tripletList.push_back(T(ind + 5, i * 12 + 2,w * 2.0f * Gr(0,2)));
        tripletList.push_back(T(ind + 5, i * 12 + 5,w * 2.0f * Gr(1,2)));
        tripletList.push_back(T(ind + 5, i * 12 + 8,w * 2.0f * Gr(2,2)));


        /*
        for(size_t m=0;m<9;m++){//indices of residual
            int m1 = m%3;//the matrices are column major
            int m2 = m/3;
            for(size_t n=0;n<9;n++){//derived by
                int n1 = n%3; // the matrices are column major (switching these two indices would probably not change much)
                int n2 = n/3;
                float val = 0;
                if(m2 == n2){

                    val = w * G_rot[i](n1,m1);
                    tripletList.push_back(T(ind + m, i*12 + n, val));//TODO: indexing definitely is off
                }else if(m1 == n2){
                    val = w * G_rot[i](m2,n2);
                    tripletList.push_back(T(ind + m, i*12 + n, val));

                }else if(m1 == m2 && n1 == n2 && n1 == m1){
                    val = w * 2.0f * G_rot[i](m1,m2);
                    tripletList.push_back(T(ind+ m, i*12 + n, val));
                }
            }
        }
         */
        ind += 6;
    }
   

    w = sqrt(wreg);
    for(size_t i=0;i<neighbours.size();i++){
        vector<unsigned int> &indices = neighbours[i];

        for(size_t j=0;j<indices.size();j++){
            //Ereg^l,n derived by Gt^l
            tripletList.push_back(T(ind + 0, i * 12 + 9, w ));
            tripletList.push_back(T(ind + 1, i * 12 + 10, w ));
            tripletList.push_back(T(ind + 2, i * 12 + 11, w ));

            int k = indices[j];
            //Ereg^l,n derived by Gt^n
            tripletList.push_back(T(ind + 0, k * 12 + 9, -w));
            tripletList.push_back(T(ind + 1, k * 12 + 10, -w));
            tripletList.push_back(T(ind + 2, k * 12 + 11, -w));

            Vector3f delta = nodes[k]-nodes[i];
            //Ereg^l,n derived by Gr^l

            tripletList.push_back(T(ind + 0, i * 12 + 0*3 + 0, w * delta(0)));
            tripletList.push_back(T(ind + 0, i * 12 + 0*3 + 1, w * delta(1)));
            tripletList.push_back(T(ind + 0, i * 12 + 0*3 + 2, w * delta(2)));

            tripletList.push_back(T(ind + 1, i * 12 + 1*3 + 0, w * delta(0)));
            tripletList.push_back(T(ind + 1, i * 12 + 1*3 + 1, w * delta(1)));
            tripletList.push_back(T(ind + 1, i * 12 + 1*3 + 2, w * delta(2)));

            tripletList.push_back(T(ind + 2, i * 12 + 2*3 + 0, w * delta(0)));
            tripletList.push_back(T(ind + 2, i * 12 + 2*3 + 1, w * delta(1)));
            tripletList.push_back(T(ind + 2, i * 12 + 2*3 + 2, w * delta(2)));

            ind += 3;
        }

    }


    cout << "jacobian rows" << jacobian.rows() << " cols " << jacobian.cols() << endl;


    jacobian.setFromTriplets(tripletList.begin(), tripletList.end());
    //cout << "jacobian " << jacobian << endl;
    MatrixXd test = jacobian.transpose() * jacobian;

    //cout << "jtj " << endl << test << endl;

    /*
    cout << "[";
    for(int i=0;i<test.rows();i++){
        for(int j=0;j<test.cols();j++){
            cout << test(i,j) << " ";
        }

        cout << ";";
    }

    cout << "]" << endl;
    */
    //lets also calculate the condition number:
    cout << "calculating condition number: " << endl;
    /*
    JacobiSVD<MatrixXd> svd(test);
    double cond = svd.singularValues()(0)
                  / svd.singularValues()(svd.singularValues().size()-1);
    cout << "condition number " << cond << endl;
     */
    cout << "not" << endl;

}

void DeformationGraph::gaussNewtonStep() {

    //https://forum.kde.org/viewtopic.php?f=74&t=98911

    SparseMatrix<double> J = jacobian;
    SparseMatrix<double> Jt = J.transpose();

    MatrixXd r = residuals;
    cout << "rows" << r.rows() << " cols " << r.cols() << endl;
    auto Jtr = Jt*r;

    auto JtJ = Jt*J;
    cout << "rows" << JtJ.rows() << " cols " << JtJ.cols() << endl;


    //SimplicialLDLT<SparseMatrix<double>> decomposer(JtJ); //the one recommended?
    SimplicialLLT<SparseMatrix<double>> decomposer(JtJ);
    //SimplicialCholesky<SparseMatrix<double>> decomposer(JtJ);

    VectorXd step = decomposer.solve(Jtr);

    //cout << " update step " << endl <<  step << endl;

    //applying step
    for(int i=0;i<nodes.size();i++){

        G_rot[i](0,0) -= step(i*12 + 0);
        G_rot[i](0,1) -= step(i*12 + 1);
        G_rot[i](0,2) -= step(i*12 + 2);

        G_rot[i](1,0) -= step(i*12 + 3);
        G_rot[i](1,1) -= step(i*12 + 4);
        G_rot[i](1,2) -= step(i*12 + 5);

        G_rot[i](2,0) -= step(i*12 + 6);
        G_rot[i](2,1) -= step(i*12 + 7);
        G_rot[i](2,2) -= step(i*12 + 8);
        for(int j=0;j<9;j++){
            G_rot[i](j) -= step(i*12 + j);
        }
        //cout << "G_rot " << endl << G_rot[i] << endl;
        for(int j=0;j<3;j++){
            G_t[i](j) -= step(i*12 + 9 + j);
        }
        //cout << "G_t" << endl << G_t[i]  << endl;
    }


    //cout << Jtr << endl;

}