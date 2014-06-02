#include "PTrackingBridge.h"
#include "UdpSocket.h"
#include <std_msgs/String.h>
#include <signal.h>

using namespace std;
using std_msgs::String;

#define ERR(x)  cerr << "\033[22;31;1m" << x << "\033[0m";
#define WARN(x) cerr << "\033[22;33;1m" << x << "\033[0m";
#define INFO(x) cerr << "\033[22;37;1m" << x << "\033[0m";
#define DEBUG(x)  cerr << "\033[22;34;1m" << x << "\033[0m";

PTrackingBridge::PTrackingBridge() : nodeHandle("~"), agentPort(-1)
{
	nodeHandle.getParam("agentPort",agentPort);
	
	publisherTargetEstimations = nodeHandle.advertise<String>("targetEstimations",1);
	
	signal(SIGINT,PTrackingBridge::interruptCallback);
}

PTrackingBridge::~PTrackingBridge() {;}

void PTrackingBridge::exec() const
{
	UdpSocket receiverSocket;
	InetAddress sender;
	string dataReceived;
	int ret;
	bool binding;
	
	if (agentPort != -1)
	{
		binding = receiverSocket.bind(agentPort);
		
		if (!binding)
		{
			ERR("Error during the binding operation. Exiting..." << endl);
			
			exit(-1);
		}
	}
	else
	{
		ERR("Agent id not set. Please check the launch file..." << endl);
		
		exit(-1);
	}
	
	INFO("PTracking bridge bound on port: " << agentPort << endl);
	
	while (true)
	{
		ret = receiverSocket.recv(dataReceived,sender);
		
		if (ret == -1)
		{
			ERR("Error in receiving message from: '" << sender.toString() << "'." << endl);
			
			continue;
		}
		
		WARN("Recv: " << dataReceived << endl);
	}
}

void PTrackingBridge::interruptCallback(int)
{
	ERR(endl << "*********************************************************************" << endl);
	ERR("Caught Ctrl^C. Exiting..." << endl);
	ERR("*********************************************************************" << endl);
	
	exit(0);
}
