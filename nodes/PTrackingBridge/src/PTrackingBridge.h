#pragma once

#include <ros/node_handle.h>

namespace PTracking
{
	class PTrackingBridge
	{
		private:
			/**
			 * @brief Handle to a ros node.
			 */
			ros::NodeHandle nodeHandle;
			
			/**
			 * @brief Publisher of the target estimations performed by the PTracking library.
			 */
			ros::Publisher publisherTargetEstimations;
			
			/**
			 * @brief Represents the agent port where the target estimations messages are received.
			 */
			int agentPort;
			
			/**
			 * @brief Function that allows a clean exit catching the SIGINT signal.
			 */
			static void interruptCallback(int);
			
		public:
			/**
			 * @brief Class constructor. Here, every ROS structure is inizialized.
			 */
			PTrackingBridge();
			
			/**
			 * @brief Class destructor. Here, every structure is destroyed.
			 */
			~PTrackingBridge();
			
			/**
			 * @brief Function that receives target estimations messages coming from the agent and it publishes them into a topic.
			 */
			void exec() const;
	};
}
