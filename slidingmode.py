import pybullet as p
import pybullet_data
import time
import numpy as np

class SlidingModeController:
    def __init__(self, lambda_val, eta, boundary_thickness):
        self.lambda_val = lambda_val
        self.eta = eta
        self.boundary_thickness = boundary_thickness
        
    def compute(self, position, velocity, desired_position, desired_velocity):
        position_error = desired_position - position
        velocity_error = desired_velocity - velocity
        
        s = velocity_error + self.lambda_val * position_error
        sat_s = np.tanh(s / self.boundary_thickness)
        
        u = self.eta * (sat_s + self.lambda_val * position_error)
        
        return u, position_error

def compute_dynamics(joint_angles, joint_velocities, gravity, link_lengths, masses):
    num_joints = len(joint_angles)
    torques = np.zeros(num_joints)
    
    for i in range(num_joints):
        g_torque = 0
        for j in range(i, num_joints):
            g_torque += masses[j] * gravity * link_lengths[j] * np.cos(joint_angles[j])
            
        m_torque = 0
        for j in range(i, num_joints):
            m_torque += masses[j] * (link_lengths[j] ** 2) * joint_velocities[j]
            
        c_torque = 0
        for j in range(num_joints):
            for k in range(num_joints):
                c_coef = 0.5 * masses[j] * link_lengths[j] * link_lengths[k]
                c_torque += c_coef * np.sin(joint_angles[j]) * joint_velocities[j] * joint_velocities[k]
        
        torques[i] = g_torque + m_torque + c_torque
    
    return torques

def calculate_accuracy(error):
    # New accuracy calculation using exponential decay
    threshold = np.pi/4  # Maximum acceptable error
    normalized_error = np.abs(error)/threshold
    return 100 * np.exp(-normalized_error)

def main():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)
    
    plane = p.loadURDF("plane.urdf")
    robot = p.loadURDF("kuka_iiwa/model.urdf", useFixedBase=True)
    
    mass = 1.0  # Reduced mass
    visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.05]*3, rgbaColor=[1,0,0,1])
    collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.05]*3)
    weight = p.createMultiBody(mass, collision, visual, [0,0,0])
    
    link_lengths = [0.4, 0.4, 0.2]
    masses = [5.0, 3.0, 2.0]
    gravity = 9.8
    
    p.createConstraint(robot, p.getNumJoints(robot)-1, weight, -1, 
                      p.JOINT_FIXED, [0,0,0], [0,0,0], [0,0,0])
    
    dt = 0.01
    p.setTimeStep(dt)
    
    # Adjusted controller parameters
    controllers = [
        SlidingModeController(lambda_val=8.0, eta=20.0, boundary_thickness=0.02),
        SlidingModeController(lambda_val=8.0, eta=20.0, boundary_thickness=0.02),
        SlidingModeController(lambda_val=8.0, eta=20.0, boundary_thickness=0.02)
    ]
    
    initial_angles = [0, -np.pi/4, np.pi/2]
    target_angles = [np.pi/4, 0, -np.pi/4]
    target_velocities = [0, 0, 0]
    
    for i in range(3):
        p.resetJointState(robot, i, initial_angles[i])
    
    all_errors = []
    all_accuracies = []
    
    for step in range(1000):
        current_errors = []
        current_accuracies = []
        joint_angles = []
        joint_velocities = []
        
        for i in range(3):
            state = p.getJointState(robot, i)
            joint_angles.append(state[0])
            joint_velocities.append(state[1])
        
        dynamic_torques = compute_dynamics(joint_angles, joint_velocities, 
                                        gravity, link_lengths, masses)
        
        control_torques = np.zeros(3)
        for i in range(3):
            u, error = controllers[i].compute(
                joint_angles[i], 
                joint_velocities[i],
                target_angles[i], 
                target_velocities[i]
            )
            control_torques[i] = u
            current_errors.append(error)
            accuracy = calculate_accuracy(error)
            current_accuracies.append(accuracy)
        
        total_torques = dynamic_torques + control_torques
        max_torque = 100.0  # Increased max torque
        total_torques = np.clip(total_torques, -max_torque, max_torque)
        
        for i in range(3):
            p.setJointMotorControl2(robot, i, p.TORQUE_CONTROL, 
                                  force=total_torques[i])
        
        avg_error = np.mean(np.abs(current_errors))
        avg_accuracy = np.mean(current_accuracies)
        all_errors.append(avg_error)
        all_accuracies.append(avg_accuracy)
        
        if step % 10 == 0:
            print(f"Step {step}:")
            print(f"Joint Torques: {[f'{t:.4f}' for t in total_torques]}")
            print(f"Joint Errors: {[f'{e:.4f}' for e in current_errors]}")
            print(f"Joint Accuracies: {[f'{a:.2f}%' for a in current_accuracies]}")
            print(f"Average Error: {avg_error:.4f}")
            print(f"Average Accuracy: {avg_accuracy:.2f}%")
            print("-" * 40)
        
        p.stepSimulation()
        time.sleep(dt)
    
    print("\nFinal Performance:")
    print(f"Final Average Error: {np.mean(all_errors):.4f}")
    print(f"Final Average Accuracy: {np.mean(all_accuracies):.2f}%")
    print(f"Best Accuracy: {np.max(all_accuracies):.2f}%")
    print(f"Worst Accuracy: {np.min(all_accuracies):.2f}%")
    
    p.disconnect()

if __name__ == "__main__":
    main()