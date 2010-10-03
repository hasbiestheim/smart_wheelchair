define floorplan model
(
  # sombre, sensible, artistic
  color "gray30"

  # most maps will need a bounding box
  boundary 1

  gui_nose 0
  gui_grid 0
  gui_move 0
  gui_outline 0
  gripper_return 0
  fiducial_return 1
  laser_return 1
  ranger_return 1
)

#load the actual map
floorplan
( 
    name "2nd_floor_one_door"
    bitmap "../maps/2nd_floor_one_door_stage.pgm"
    size [79.45 90.45 1.0]
    pose [ 0.0 0.0 0.0 0.0]
)