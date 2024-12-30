import pybullet as p
import pybullet_data
import time
import numpy as np

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp  
        self.ki = ki  
        self.kd = kd  
        
        self.previous_error = 0
        self.integral = 0
        
    def compute(self, setpoint, measured_value, dt):
       
        error = setpoint - measured_value
        p_term = self.kp * error
        self.integral += error * dt
        i_term = self.ki * self.integral
        d_term = self.kd * (error - self.previous_error) / dt if dt > 0 else 0
        output = p_term + i_term + d_term
        self.previous_error = error
        
        return output

def compute_torque(joint_angles, joint_velocities, gravity, link_lengths, masses):
    num_joints = len(joint_angles)
    torques = np.zeros(num_joints)

    for i in range(num_joints):
        gravitational_torque = 0
        for j in range(i, num_joints):
            gravitational_torque += masses[j] * gravity * link_lengths[j] * np.cos(joint_angles[j])
        
        inertial_torque = 0
        for j in range(i, num_joints):
            inertial_torque += masses[j] * (link_lengths[j] ** 2) * joint_velocities[j]
        
        coriolis_torque = 0
        for j in range(num_joints):
            for k in range(num_joints):
                coriolis_coefficient = 0.5 * masses[j] * link_lengths[j] * link_lengths[k]
                coriolis_torque += coriolis_coefficient * np.sin(joint_angles[j]) * joint_velocities[j] * joint_velocities[k]
        
        torques[i] = gravitational_torque + inertial_torque + coriolis_torque

    return torques

def main():
    physics_client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)

    plane_id = p.loadURDF("plane.urdf")
    robot_id = p.loadURDF("kuka_iiwa/model.urdf", useFixedBase=True)

    weight_mass = 2.0 
    weight_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.05, 0.05, 0.05], rgbaColor=[1, 0, 0, 1])
    weight_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.05, 0.05, 0.05])
    weight_id = p.createMultiBody(
        baseMass=weight_mass,
        baseCollisionShapeIndex=weight_collision,
        baseVisualShapeIndex=weight_visual,
        basePosition=[0, 0, 0]
    )

    num_joints = p.getNumJoints(robot_id)
    link_lengths = [0.4, 0.4, 0.2]  
    masses = [5.0, 3.0, 2.0]
    gravity = 9.8
    end_effector_link = num_joints - 1
    p.createConstraint(
        parentBodyUniqueId=robot_id,
        parentLinkIndex=end_effector_link,
        childBodyUniqueId=weight_id,
        childLinkIndex=-1,
        jointType=p.JOINT_FIXED,
        jointAxis=[0, 0, 0],
        parentFramePosition=[0, 0, 0],
        childFramePosition=[0, 0, 0]
    )

    time_step = 0.01
    p.setTimeStep(time_step)

    initial_joint_angles = [0, -np.pi / 4, np.pi / 2]
    for i in range(3):
        p.resetJointState(robot_id, i, initial_joint_angles[i])

    target_joint_angles = [np.pi/4, 0, -np.pi/4]

    pid_controllers = [
        PIDController(kp=10.0, ki=0.1, kd=1.0),  # Joint 0
        PIDController(kp=10.0, ki=0.1, kd=1.0),  # Joint 1
        PIDController(kp=10.0, ki=0.1, kd=1.0)   # Joint 2
    ]

    for step in range(1000):
        joint_angles = []
        joint_velocities = []
        for i in range(3):
            joint_state = p.getJointState(robot_id, i)
            joint_angles.append(joint_state[0])
            joint_velocities.append(joint_state[1])

        dynamic_torques = compute_torque(joint_angles, joint_velocities, gravity, link_lengths, masses)

        pid_torques = np.zeros(3)
        for i in range(3):
            pid_torques[i] = pid_controllers[i].compute(target_joint_angles[i], joint_angles[i], time_step)

        total_torques = dynamic_torques + pid_torques

        print(f"Step {step}: Joint Torques: {total_torques}")
        for i in range(3):
            p.setJointMotorControl2(
                bodyIndex=robot_id,
                jointIndex=i,
                controlMode=p.TORQUE_CONTROL,
                force=total_torques[i]
            )

        p.stepSimulation()
        time.sleep(time_step)

    p.disconnect()

if __name__ == "__main__":
    main()