#include "PTrackingBridge.h"
#include "UdpSocket.h"
#include <PTrackingBridge/TargetEstimations.h>
#include <geometry_msgs/Point32.h>
#include <signal.h>

using namespace std;
using PTrackingBridge::TargetEstimations;
using geometry_msgs::Point32;

#define ERR(x)  cerr << "\033[22;31;1m" << x << "\033[0m";
#define WARN(x) cerr << "\033[22;33;1m" << x << "\033[0m";
#define INFO(x) cerr << "\033[22;37;1m" << x << "\033[0m";
#define DEBUG(x)  cerr << "\033[22;34;1m" << x << "\033[0m";

namespace PTracking
{
	PTrackingBridge::PTrackingBridge() : nodeHandle("~"), agentPort(-1)
	{
		nodeHandle.getParam("agentPort",agentPort);
		
		publisherTargetEstimations = nodeHandle.advertise<TargetEstimations>("targetEstimations",1);
		
		/// Catching Ctrl^C to get a clean exit.
		signal(SIGINT,PTrackingBridge::interruptCallback);
	}
	
	PTrackingBridge::~PTrackingBridge() {;}
	
	void PTrackingBridge::exec() const
	{
		vector<Point32> positions, standardDeviations, velocities;
		vector<int> heights, identities, widths;
		UdpSocket receiverSocket;
		InetAddress sender;
		stringstream s;
		string dataReceived, token;
		int iterations, ret; 
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
		
		iterations = 0;
		
		while (true)
		{
			ret = receiverSocket.recv(dataReceived,sender);
			
			if (ret == -1)
			{
				ERR("Error in receiving message from: '" << sender.toString() << "'." << endl);
				
				continue;
			}
			
			heights.clear();
			identities.clear();
			positions.clear();
			standardDeviations.clear();
			velocities.clear();
			widths.clear();
			
			s.clear();
			s.str("");
			
			s << dataReceived;
			
			/// Splitting message (each estimation is separated by the ';' character).
			while (getline(s,token,';'))
			{
				Point32 position, standardDeviation, velocity;
				stringstream tokenStream;
				int height, identity, width;
				
				tokenStream << token;
				
				tokenStream >> identity >> position.x >> position.y >> standardDeviation.x >> standardDeviation.y >> width >> height >> velocity.x >> velocity.y;
				
				identities.push_back(identity);
				positions.push_back(position);
				standardDeviations.push_back(standardDeviation);
				widths.push_back(width);
				heights.push_back(height);
				velocities.push_back(velocity);
			}
			
			TargetEstimations targetEstimations;
			
			targetEstimations.header.seq = iterations++;
			targetEstimations.header.stamp = ros::Time::now();
			
			targetEstimations.identities = identities;
			targetEstimations.positions = positions;
			targetEstimations.standardDeviations = standardDeviations;
			targetEstimations.widths = widths;
			targetEstimations.heights = heights;
			targetEstimations.velocities = velocities;
			
			publisherTargetEstimations.publish(targetEstimations);
		}
	}
	
	void PTrackingBridge::interruptCallback(int)
	{
		ERR(endl << "*********************************************************************" << endl);
		ERR("Caught Ctrl^C. Exiting..." << endl);
		ERR("*********************************************************************" << endl);
		
		exit(0);
	}
}
