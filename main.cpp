#include "dwa.h"
#include "simulator.h"

int main()
{
	std::cout<<"------------ DWA TEST -------------"<<std::endl;
	
	dwa::Simulator<float> sim;
	sim.initObtacles();
	
	for( int i = 0; i < 100; i ++ ){
		sim.dwaProcess();
		cv::waitKey(0);	
	}

	return 0;
}
