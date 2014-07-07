agent_dispatcher
================

To launch the node

```bash
	$> rosrun agents_dispatcher agents_dispatcher
```

*** Subscribed topics
	
	* `/agent_1/PtrackingBridge/targetEstimations` (PtrackingBridge/targetEstimations)
	
*** Published topics
 
    * `/agentsDispatcher/agent_X` (nav_msgs/Path). The path of agent X. The id X is 
	  dynamically assigned as a new agent is detected and tracked.

*** Parameters
	
	* `/agentsDispatcher/ptracking_topic` (`string`, default: /agent_1/PtrackingBridge/targetEstimations).
	  The topic published by ptracking with the information about the detected agents
	  
    * `/agentsDispatcher/frame_id` (`string`, default: /map). The paths of pedestrians stored in
	  the published topics are refereed to this reference frame.

If `HRVO` variable is defined in the file `AgentsDispatcher.hpp` the `hrvo_add_agent` 
service is called whenever a new agent is detect by the ptracker.
