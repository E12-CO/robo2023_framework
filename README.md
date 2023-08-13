# robo2023_framework
e12 roboinnovator 2023 

build with ```colcon build --packages-select robo2023 robo2023_interfaces```

## How to use

### Image Processing Node

1.source workspace
2.run `ros2 run robo2023 imgproc_node.py`
3.in another terminal , source then run `ros2 service call /get_img_res robo2023_interfaces/srv/Imgres "{img: "run"}"`

### Servo Node

1.source workspace
2.run `ros2 run robo2023 servo_node.py`
3.in another terminal , source then run `ros2 service call /get_servo_res robo2023_interfaces/srv/Servores "{servo: "run"}"`


## TODO
change robot urdf
