import time
import odrive
from odrive.enums import  *
import fibre
import math

# Configuration Parameters
SPRING_CONSTANT  = 10.0      # N*rad/m (Spring constant)
SPRING_CENTER   = 0.25       # Center position for the spring effect (radians)
FRICTION_CONSTANT = 5        # N*m/s (Viscous friction coefficient)
STATIC_FRICTION_THRESHOLD = 0.05  # Static friction threshold

# Lever arm parameters
LEVER_ARM_LENGTH = 35.0      # cm (Length of the lever arm)
GRIP_WEIGHT = 700.0          # grams (Weight of the grip)

# Convert grip weight to kilograms
GRIP_WEIGHT_KG = GRIP_WEIGHT / 1000.0

# Gravitational acceleration
GRAVITY = 9.81               # m/s^2

# Detent Parameters
DETENTS = [
    {"position": 0.04, "force": 5.0, "direction": -1},  # Applies force in positive direction
    {"position": -0.05, "force": 5.0, "direction": 0} # Applies force in both directions
]

# Connect to ODrive
print("Connecting to ODrive...")
odrv = odrive.find_any()

# Initialize Motor Axis
axis = odrv.axis0
axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
axis.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
axis.motor.config.current_control_bandwidth =  10


# Helper Functions
def damping_effect(velocity):
    # Viscous friction proportional to velocity
    return -FRICTION_CONSTANT * velocity

def gravitational_torque(angle):
    # Torque due to gravity
    return GRIP_WEIGHT_KG * GRAVITY * (LEVER_ARM_LENGTH / 100.0) * math.sin(angle) * -1

def detent_effect(position, velocity):
    # Calculate the detent force based on the proximity to detent positions
    force = 0
    # Damping coefficient
    DAMPING_COEFFICIENT = 10

    # Calculate damping force
    damping_force = -DAMPING_COEFFICIENT * velocity

    for detent in DETENTS:
        distance = abs(position - detent["position"])
        if distance < 0.02:
            if detent["direction"] == 0:
                # Apply force in both directions
                force += detent["force"] * (0.02 - distance) / 0.02 * (-1 if position < detent["position"] else 1)
            else:
                # Apply force in specified direction
                #if (vel != 0 ):
                    force += detent["force"] * (0.02 - distance) / 0.02 * detent["direction"] + damping_force
                    print (vel, detent["direction"])
                    if (vel < 0 and detent["direction"] == -1):
                        force = 0
                    if (vel > 0 and detent["direction"] == 1):
                        force = 0

                
                    
        
    return force


# Main Control Loop
try:
    print("Starting force feedback control...")
    hold_position = axis.encoder.pos_estimate  # Initial hold position

    while True:
        pos = axis.encoder.pos_estimate
        vel = axis.encoder.vel_estimate

        # Calculate damping torque (friction)
        damping_torque = damping_effect(vel)
        #damping_torque = 0


        # Calculate gravitational torque
        gravity_torque = gravitational_torque(pos)

        # Calculate detent torque
        detent_torque = detent_effect(pos, vel)

        # Apply total torque
        total_torque = damping_torque + gravity_torque + detent_torque


        axis.controller.input_torque = total_torque

        #print(f"Torque: {total_torque:.4f}, Gravity Torque: {gravity_torque:.4f}, Detent Torque: {detent_torque:.4f}, Position: {pos:.4f}, Velocity: {vel:.4f}")

        # Small delay to prevent CPU overload
        #time.sleep(0.001)

except KeyboardInterrupt:
    print("Stopping motor control...")
    axis.controller.input_torque = 0
    axis.requested_state = AXIS_STATE_IDLE
except fibre.libfibre.ObjectLostError:
    print("Lost connection to ODrive.")