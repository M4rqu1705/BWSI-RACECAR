from talker.py import publish_message

publish_message('forward.py', '/vesc/ackermann_cmd_mux/input/navigation/ackerman_msgs/AckermannDriveStamped', '{header: auto, drive: {steering_angle:0.2, speed:1.2}}')
