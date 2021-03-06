# Imported Python Transfer Function
@nrp.MapRobotPublisher('r_shoulder_roll', Topic('/robot/r_shoulder_roll/pos', std_msgs.msg.Float64))
@nrp.MapRobotPublisher('r_shoulder_pitch', Topic('/robot/r_shoulder_pitch/pos', std_msgs.msg.Float64))
@nrp.MapRobotPublisher('r_shoulder_yaw', Topic('/robot/r_shoulder_yaw/pos', std_msgs.msg.Float64))
@nrp.MapRobotPublisher('r_elbow', Topic('/robot/r_elbow/pos', std_msgs.msg.Float64))
@nrp.MapRobotPublisher('eye_tilt', Topic('/robot/eye_tilt/pos', std_msgs.msg.Float64))
@nrp.Neuron2Robot()
def simple_move_robot(t, r_shoulder_roll, r_shoulder_pitch, r_shoulder_yaw, r_elbow, eye_tilt):
    r_elbow.send_message(std_msgs.msg.Float64(2.))
    eye_tilt.send_message(std_msgs.msg.Float64(-0.8))
