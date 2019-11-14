//
// Created by simon on 14.11.19.
//
#include <iostream>
#include <vector>
#include <Eigen/Eigen>
using namespace std;
using namespace Eigen;
struct Test{
	float bla;
	Vector4f blabla;
};

int main(int argc,char *varg[]){
	int size = 100;
	Test *t = new Test();
	cout << t << endl;
	cout << &t->blabla << endl;
	cout << "dist " << (unsigned long)&t->blabla - (unsigned long)t << endl;

	for(int i=0;i<10;i++){
		vector<Vector4f> *test = new vector<Vector4f>(size);
		cout << &test[0] << endl;
		for(int j=0;j<size;j++){
			(*test)[j] = Vector4f(1,1,1,1);
		}
	}
	return 0;
}