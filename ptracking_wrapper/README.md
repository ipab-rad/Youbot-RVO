ptracking_wrapper
================

To launch the node

```bash
	$> rosrun ptracking_wrapper ptracking_wrapper
```

## Subscribed topics
	
* `/agent_1/PtrackingBridge/targetEstimations` (PtrackingBridge/targetEstimations)
	
## Published topics
 
* `/ptracking_bridge/agent_X` (nav_msgs/Path). The path of agent X. The id X is 
 dynamically assigned as a new agent is detected and tracked.

## Parameters
	
* `/ptracking_wrapper/ptracking_topic` (`string`, default: /agent_1/PtrackingBridge/targetEstimations).
  The topic published by ptracking with the information about the detected agents
	  
* `/ptracking_wrapper/frame_id` (`string`, default: /map). The paths of pedestrians stored in
  the published topics are refereed to this reference frame.

### Note

If `HRVO` variable is defined in the file `AgentsDispatcher.hpp` the `hrvo_add_agent` 
service is called whenever a new agent is detect by the ptracker.
