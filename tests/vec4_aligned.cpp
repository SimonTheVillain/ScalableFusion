//
// Created by simon on 14.11.19.
//

#include <vector>
#include <Eigen/Eigen>
using namespace std;
using namespace Eigen;


int main(int argc,char *varg[]){
	int size = 100;
	for(int i=0;i<10;i++){
		vector<Vector4f,aligned_allocator<Vector4f>> test(size);

		for(int j=0;j<size;j++){
			test[j] = Vector4f(1,1,1,1);
		}
	}
	return 0;
}