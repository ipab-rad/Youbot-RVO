#include "PTrackingBridge.h"
#include <ros/node_handle.h>

using namespace std;
using namespace ros;
using namespace PTracking;

int main(int argc, char** argv)
{
	ros::init(argc,argv,"PTrackingBridge");
	
	PTrackingBridge pTrackingBridge;
	
	while (ros::ok())
	{
		pTrackingBridge.exec();
		
		usleep(30e3);
	}
	
	return 0;
}
