#include "dwa.h"
#include "simulator.h"

int main()
{
	std::cout<<"------------ DWA TEST -------------"<<std::endl;
	
	dwa::Simulator<float> sim;
	sim.initObtacles();
	cv::waitKey(0);	

	return 0;
}
