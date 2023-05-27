define lidar ranger
(
  sensor
  (
    range [0 20]
    fov 360.0
    samples 721
  )
  # generic model properties
  color "black"
)

define rosbot position
(
  #localization "gps"
  localization "odom"
  odom_error [0.0 0.01 0.0 0.01 0.0 0.01]

  size [ 0.2 0.2 0.2 ]
  origin [ 0.0 0.0 0.0 0.0 ]
  
  gui_nose 1
  drive "diff"
  color "red"
  obstacle_return 1    # Can hit things.
  laser_return 1
  
  lidar(pose [ 0.0 0.0 0.1 0.0 ])
)
