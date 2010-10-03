define maxSonarEZ1Set ranger
(
#      # number of sonars
      scount 6

      # define the pose of each transducer [xpos ypos heading]
      spose[0] [ 0.482 0.254 0]	   # front left headlight
      spose[1] [ 0.482 -0.254 0]   # front right headlight
      spose[2] [ 0.482 0.254 45]   # front left corner
      spose[3] [ 0.482 -0.254 -45] # front right corner
      spose[4] [ 0.450 0.254 90]   # front left side
      spose[5] [ 0.450 -0.254 -90] # front right side
		
      # define the field of view of each transducer 
      #[range_min range_max view_angle]
      sview [0.1524 6.4516 25]

      # define the size of each transducer
     # [xsize ysize] in meters
      ssize [0.0199 0.0164]
)

define hoku laser
(
      range_min 0.020
      range_max 4.000
      fov 240
      samples 180
      pose [-0.305 0 0 180]
      size [0.050 0.050 0.070]
)

define ottoModel position
(
      size [1.140 0.661 0.902]
      origin [0.091 0 0 0]
	
      drive "diff"
      localization "odom"

      # odometry error model parameters, 
      # only used if localization is set to "odom"
      odom_error [0.0 0.0 0.0 0.0]

      maxSonarEZ1Set(name "otto_EZ1s")
      hoku(name "otto_Hokuyo")
      
      obstacle_return 1           	# Can hit things.
      laser_return 1                 # reflects laser beams
      ranger_return 1             	# reflects sonar beams
) 



