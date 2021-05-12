# Current TODOs for TDP

## Software

- Sliding Window for pose of N pose measurements 
	- Because IMU is very noisy -> decrease variance
	- Remove outliers, above threshold of n*std not included 
		- alternatively no absolute difference to previous pose larger than threshold
- Time Synchronization of Pi and Livox
	- To make sure pose and scan are matched by time
	- Use time published by livox to set the time @pi
	- Synchronization Idea:
		- @Processing machine: take two close measurments of lidar and pose and check
		  their timestamps. Request current time of lidar and imu again. Consider the shift
		  in each to compute the absolute time difference -> timestamp both values with new  
                  difference internally
- Not urgent: Dorit's algorithm (Find planes and match points onto 
       the planes)
 
## Hardware
- Build Sample environment (Tunnel 2m x 1m x 1m) using cheap wood
