"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.

Joint Monkey
------------
- Animates degree-of-freedom ranges for a given asset.
- Demonstrates usage of DOF properties and states.
- Demonstrates line drawing utilities to visualize DOF frames (origin and axis).
"""

# conda activate py38
# export LD_LIBRARY_PATH=${CONDA_PREFIX}/lib

# import os
# os.environ["LD_LIBRARY_PATH"] = "/home/grl/repo/micromamba/envs/py38/lib"

# sudo LD_LIBRARY_PATH=/home/grl/repo/micromamba/envs/py38/lib /home/grl/repo/micromamba/envs/py38/bin/python joint_monkey.py

import math
import numpy as np
from isaacgym import gymapi, gymutil
from scipy.spatial.transform import Rotation 


# ENABLE_MOTOR=True
ENABLE_MOTOR=False


if ENABLE_MOTOR:
    import signal
    from build import ethercat_motor_py

    interface_name = "enp3s0"
    control_mode_int8 = 8
    max_velocity = 3
    max_torque= 240
    motor=ethercat_motor_py.PyMotor("enp3s0",control_mode_int8, max_velocity, max_torque)
    motor.should_print = 0
    target_offset = [0.00051, 0.22990, 0.16049, 0.31407, 0.40668, 0.00145]


    def signal_handler(sig, frame):
        print("Keyboard Interrupt detected. Attempting to stop background thread...")
        motor.set_should_terminate(True)
        exit()
    signal.signal(signal.SIGINT, signal_handler)



ENABLE_IMU = False
# ENABLE_IMU = True
if ENABLE_IMU:
    import mscl
    
class IMUHelper:
    # this might change if the IMU is unplugged, to identify
        # unplug IMU, run ls /dev/tty* in terminal
        # plug in IMU, run ls /dev/tty* in terminal
        # look for the new port, this is the IMU
    def __init__(self, tty_port='/dev/ttyACM0'):
        self.tty_port = tty_port
        self.connection = None
        self.node = None

    def connect(self):
        try:
            self.connection = mscl.Connection.Serial(self.tty_port)
            self.node = mscl.InertialNode(self.connection)
            print("Ping:", self.node.ping())

            # Configuration (adjust as needed)
            if self.node.features().supportsCategory(mscl.MipTypes.CLASS_AHRS_IMU):
                channels = mscl.MipChannels()
                channels.append(mscl.MipChannel(mscl.MipTypes.CH_FIELD_SENSOR_ORIENTATION_QUATERNION, 
                                                mscl.SampleRate.Hertz(1000)))  # Set sample rate
                self.node.setActiveChannelFields(mscl.MipTypes.CLASS_AHRS_IMU, channels)
                self.node.enableDataStream(mscl.MipTypes.CLASS_AHRS_IMU)

        except mscl.Error as e:
            print("Error initializing IMU:", e)

    def get_ned_quaternion(self):
        if self.node is None:
            return None

        try:
            packets = self.node.getDataPackets(100)
            packet = packets[-1]

            for dataPoint in packet.data():
                if dataPoint.channelName() == "orientQuaternion":
                    quaternion = dataPoint.as_Matrix()
                    return quaternion.as_floatAt(0, 3), quaternion.as_floatAt(0, 0), quaternion.as_floatAt(0, 1), quaternion.as_floatAt(0, 2)

        except mscl.Error as e:
            print("Error:", e)
            return None
    
    def get_neu_quaternion(self):
        if self.node is None:
            return None
        ned = self.get_ned_quaternion()
        if(ned == None):
            return None
        ned_array = np.array([ned[0], ned[1], ned[2], ned[3]])  

        # Create rotation objects
        ned_rotation = Rotation.from_quat(ned_array)
        fix_handedness = Rotation.from_rotvec(np.pi * np.array([0, 0, 1]))  # Rotation of 180 degrees around Z

        # Combine transformations
        neu_rotation = fix_handedness * ned_rotation.inv()  # Conjugation done implicitly

        # Return as SciPy quaternion
        return neu_rotation.as_quat() 

def clamp(x, min_value, max_value):
    return max(min(x, max_value), min_value)

# simple asset descriptor for selecting from a list


class AssetDesc:
    def __init__(self, file_name, flip_visual_attachments=False):
        self.file_name = file_name
        self.flip_visual_attachments = flip_visual_attachments


asset_root = "../assets/" # change to your asset root folder
asset_descriptors = [
    AssetDesc("urdf/legURDFv2/legURDFv2.urdf", False),
    # AssetDesc("urdf/legURDF/legURDF.urdf", False),
    # AssetDesc("biped_schematics/biped_schematics.urdf", False),
    # AssetDesc("mjcf/nv_humanoid.xml", False),
    # AssetDesc("mjcf/nv_ant.xml", False),
    # AssetDesc("urdf/cartpole.urdf", False),
    # AssetDesc("urdf/sektion_cabinet_model/urdf/sektion_cabinet.urdf", False),
    # AssetDesc("urdf/franka_description/robots/franka_panda.urdf", True),
    # AssetDesc("urdf/kinova_description/urdf/kinova.urdf", False),
    # AssetDesc("urdf/anymal_b_simple_description/urdf/anymal.urdf", True),
    
    
]


# parse arguments
args = gymutil.parse_arguments(
    description="Joint monkey: Animate degree-of-freedom ranges",
    custom_parameters=[
        {"name": "--asset_id", "type": int, "default": 0, "help": "Asset id (0 - %d)" % (len(asset_descriptors) - 1)},
        {"name": "--speed_scale", "type": float, "default": 0.1, "help": "Animation speed scale"},
        {"name": "--show_axis", "action": "store_true", "help": "Visualize DOF axis"}])

if args.asset_id < 0 or args.asset_id >= len(asset_descriptors):
    print("*** Invalid asset_id specified.  Valid range is 0 to %d" % (len(asset_descriptors) - 1))
    quit()


# initialize gym
gym = gymapi.acquire_gym()

# configure sim
sim_params = gymapi.SimParams()
sim_params.dt = dt = 1.0 / 1000.0
if args.physics_engine == gymapi.SIM_FLEX:
    pass
elif args.physics_engine == gymapi.SIM_PHYSX:
    sim_params.physx.solver_type = 1
    sim_params.physx.num_position_iterations = 1
    sim_params.physx.num_velocity_iterations = 0
    sim_params.physx.num_threads = args.num_threads
    sim_params.physx.use_gpu = args.use_gpu

sim_params.use_gpu_pipeline = False
if args.use_gpu_pipeline:
    print("WARNING: Forcing CPU pipeline.")

sim = gym.create_sim(args.compute_device_id, args.graphics_device_id, args.physics_engine, sim_params)
if sim is None:
    print("*** Failed to create sim")
    quit()

# add ground plane
plane_params = gymapi.PlaneParams()
gym.add_ground(sim, plane_params)

# create viewer
viewer = gym.create_viewer(sim, gymapi.CameraProperties())
if viewer is None:
    print("*** Failed to create viewer")
    quit()

# load asset

asset_file = asset_descriptors[args.asset_id].file_name

asset_options = gymapi.AssetOptions()
asset_options.fix_base_link = True
asset_options.flip_visual_attachments = asset_descriptors[args.asset_id].flip_visual_attachments
asset_options.use_mesh_materials = True

print("Loading asset '%s' from '%s'" % (asset_file, asset_root))
asset = gym.load_asset(sim, asset_root, asset_file, asset_options)

# get array of DOF names
dof_names = gym.get_asset_dof_names(asset)

# get array of DOF properties
dof_props = gym.get_asset_dof_properties(asset)

# create an array of DOF states that will be used to update the actors
num_dofs = gym.get_asset_dof_count(asset)
dof_states = np.zeros(num_dofs, dtype=gymapi.DofState.dtype)
dof_states_0 = np.zeros(num_dofs, dtype=gymapi.DofState.dtype)

# get list of DOF types
dof_types = [gym.get_asset_dof_type(asset, i) for i in range(num_dofs)]

# get the position slice of the DOF state array
dof_positions = dof_states['pos']

# get the limit-related slices of the DOF properties array
stiffnesses = dof_props['stiffness']
dampings = dof_props['damping']
armatures = dof_props['armature']
has_limits = dof_props['hasLimits']
lower_limits = dof_props['lower']
upper_limits = dof_props['upper']

# HACK override
dof_props['stiffness'][:]=100
dof_props['damping'][:]=1000

# initialize default positions, limits, and speeds (make sure they are in reasonable ranges)
defaults = np.zeros(num_dofs)
speeds = np.zeros(num_dofs)
for i in range(num_dofs):
    if has_limits[i]:
        if dof_types[i] == gymapi.DOF_ROTATION:
            lower_limits[i] = clamp(lower_limits[i], -math.pi, math.pi)
            upper_limits[i] = clamp(upper_limits[i], -math.pi, math.pi)
        # make sure our default position is in range
        if lower_limits[i] > 0.0:
            defaults[i] = lower_limits[i]
        elif upper_limits[i] < 0.0:
            defaults[i] = upper_limits[i]
    else:
        # set reasonable animation limits for unlimited joints
        if dof_types[i] == gymapi.DOF_ROTATION:
            # unlimited revolute joint
            lower_limits[i] = -math.pi
            upper_limits[i] = math.pi
        elif dof_types[i] == gymapi.DOF_TRANSLATION:
            # unlimited prismatic joint
            lower_limits[i] = -1.0
            upper_limits[i] = 1.0
    # set DOF position to default
    dof_positions[i] = defaults[i]
    # set speed depending on DOF type and range of motion
    if dof_types[i] == gymapi.DOF_ROTATION:
        speeds[i] = args.speed_scale * clamp(2 * (upper_limits[i] - lower_limits[i]), 0.25 * math.pi, 3.0 * math.pi)
    else:
        speeds[i] = args.speed_scale * clamp(2 * (upper_limits[i] - lower_limits[i]), 0.1, 7.0)

# Print DOF properties
for i in range(num_dofs):
    print("DOF %d" % i)
    print("  Name:     '%s'" % dof_names[i])
    print("  Type:     %s" % gym.get_dof_type_string(dof_types[i]))
    print("  Stiffness:  %r" % stiffnesses[i])
    print("  Damping:  %r" % dampings[i])
    print("  Armature:  %r" % armatures[i])
    print("  Limited?  %r" % has_limits[i])
    if has_limits[i]:
        print("    Lower   %f" % lower_limits[i])
        print("    Upper   %f" % upper_limits[i])

# set up the env grid
num_envs = 1
num_per_row = 1
spacing = 2.5
env_lower = gymapi.Vec3(-spacing, 0.0, -spacing)
env_upper = gymapi.Vec3(spacing, spacing, spacing)

# position the camera
# cam_pos = gymapi.Vec3(17.2, 2.0, 16)
# cam_target = gymapi.Vec3(5, -2.5, 13)

cam_pos = gymapi.Vec3(0, 1, 3)
cam_target = gymapi.Vec3(0, 1, 0)
gym.viewer_camera_look_at(viewer, None, cam_pos, cam_target)

# cache useful handles
envs = []
actor_handles = []

print("Creating %d environments" % num_envs)
for i in range(num_envs):
    # create env
    env = gym.create_env(sim, env_lower, env_upper, num_per_row)
    envs.append(env)

    # add actor
    pose = gymapi.Transform()
    pose.p = gymapi.Vec3(0.0, 1.32, 0.0)
    pose.r = gymapi.Quat(-0.707107, 0.0, 0.0, 0.707107)

    actor_handle = gym.create_actor(env, asset, pose, "actor", i, 1)
    actor_handles.append(actor_handle)

    # set default DOF positions
    gym.set_actor_dof_states(env, actor_handle, dof_states, gymapi.STATE_ALL)
    
    gym.set_actor_dof_properties(env,actor_handle,dof_props)

# joint animation states
ANIM_SEEK_LOWER = 1
ANIM_SEEK_UPPER = 2
ANIM_SEEK_DEFAULT = 3
ANIM_FINISHED = 4

# initialize animation state
anim_state = ANIM_SEEK_LOWER
current_dof = 0
print("Animating DOF %d ('%s')" % (current_dof, dof_names[current_dof]))

def get_axis_angle(q):
    """Converts a quaternion to axis-angle representation (axis, angle)."""
    rotation = Rotation.from_quat(q)
    return rotation.as_rotvec(), rotation.angle_from_quaternion()

def transform_with_axis(q1, q2):
    """Transforms q2 by the rotation axis and angle from q1."""
    axis, angle = get_axis_angle(q1)
    rotation = Rotation.from_egm(axis, angle)
    return rotation.apply(q2)

# Start IMU data collection
if ENABLE_IMU:
    imu_helper = IMUHelper()
    imu_helper.connect()
    
if ENABLE_MOTOR:
    motor.run()

if ENABLE_IMU:
    # Get data from IMU
    quat=None
    while(quat is None):
        quat = imu_helper.get_ned_quaternion()
    x, y, z, w = quat
    adjusted_quaternion = gymapi.Quat(w, x, y, z)


time_step_k = -1
while not gym.query_viewer_has_closed(viewer):
    
    time_step_k+=1
    
    # step the physics
    gym.simulate(sim)
    gym.fetch_results(sim, True)

    speed = speeds[current_dof]
    

    # animate the dofs
    # animate = False
    animate = True
    if animate: 
        if anim_state == ANIM_SEEK_LOWER:
            dof_positions[current_dof] -= speed * dt
            if dof_positions[current_dof] <= lower_limits[current_dof]:
                dof_positions[current_dof] = lower_limits[current_dof]
                anim_state = ANIM_SEEK_UPPER
        elif anim_state == ANIM_SEEK_UPPER:
            dof_positions[current_dof] += speed * dt
            if dof_positions[current_dof] >= upper_limits[current_dof]:
                dof_positions[current_dof] = upper_limits[current_dof]
                anim_state = ANIM_SEEK_DEFAULT
        if anim_state == ANIM_SEEK_DEFAULT:
            dof_positions[current_dof] -= speed * dt
            if dof_positions[current_dof] <= defaults[current_dof]:
                dof_positions[current_dof] = defaults[current_dof]
                anim_state = ANIM_FINISHED
        elif anim_state == ANIM_FINISHED:
            dof_positions[current_dof] = defaults[current_dof]
            current_dof = (current_dof + 1) % num_dofs
            anim_state = ANIM_SEEK_LOWER
            print("Animating DOF %d ('%s')" % (current_dof, dof_names[current_dof]))
        
        for i in range(num_envs):
            gym.set_actor_dof_states(envs[i], actor_handles[i], dof_states, gymapi.STATE_POS)
            
    # for i in range(num_envs): #override
    #     current_dof_state = gym.get_actor_dof_states(env, actor_handle, gymapi.STATE_POS)
    #     gym.set_actor_dof_states(envs[i], actor_handles[i], dof_states_0, gymapi.STATE_POS)
            

    current_dof_state = gym.get_actor_dof_states(env, actor_handle, gymapi.STATE_POS)
    
    dof_pos = current_dof_state['pos']
    
    if ENABLE_MOTOR:
        motr_pos = np.copy(dof_pos)
        motr_pos[4]+=motr_pos[3]
        motor_input = np.zeros(6)
        motor_input[:5]=motr_pos
        motor_input[:] = motor_input+target_offset
        motor.set_target_input(list(motor_input))
        
    print(dof_pos)
    
    # print(gym.get_actor_dof_dict(env, actor_handle))
    # print(gym.get_actor_dof_position_targets(env, actor_handle))
    
    if args.show_axis:
        gym.clear_lines(viewer)
        
        # clone actor state in all of the environments
        
        '''   
        q_initial = np.array([0.336905, 0.005120, 0.023604, 0.941229]) # measure output at default IMU position
        q_desired = np.array([0.707107, 0.707107, 0.0, 0])
            
        def rotation_quaternion_from_quaternions(q_initial, q_desired):
            q_relative = Rotation.from_quat(q_desired) * Rotation.from_quat(q_initial).inv()
            return q_relative.as_quat()  # Return as a quaternion
        
        q_transform = rotation_quaternion_from_quaternions(q_initial, q_desired)
        q_transform_gym = gymapi.Quat(q_transform[0], q_transform[1], q_transform[2], q_transform[3])
        '''
        # print(q_transform_gym)
    
    
    
    for i in range(num_envs):
        
        
        if ENABLE_IMU:
            # Get data from IMU
            quat = imu_helper.get_ned_quaternion()
            if (quat is not None):
                x, y, z, w = quat
                adjusted_quaternion = gymapi.Quat(w, x, y, z)
            state = gym.get_actor_rigid_body_states(envs[i], actor_handles[i], gymapi.STATE_POS)
            state['pose']['r'][0].fill((adjusted_quaternion.w, adjusted_quaternion.z, adjusted_quaternion.x, adjusted_quaternion.y))  # Update orientation
            # print(state['pose']['r'])
            state['pose']['p'][0].fill((0, 1.32, 0))
            gym.set_actor_rigid_body_states(envs[i], actor_handles[i], state, gymapi.STATE_POS)
            

        if args.show_axis:
            # get the DOF frame (origin and axis)
            dof_handle = gym.get_actor_dof_handle(envs[i], actor_handles[i], current_dof)
            frame = gym.get_dof_frame(envs[i], dof_handle)

            # draw a line from DOF origin along the DOF axis
            p1 = frame.originget_ned_quaternion(self)
            p2 = frame.origin + frame.axis * 0.7
            color = gymapi.Vec3(1.0, 0.0, 0.0)
            gymutil.draw_line(p1, p2, color, gym, viewer, envs[i])
                
    # update the viewer
    gym.step_graphics(sim)
    gym.draw_viewer(viewer, sim, True)

    # Wait for dt to elapse in real time.
    # This synchronizes the physics simulation with the rendering rate.
    # gym.sync_frame_time(sim)

print("Done")

gym.destroy_viewer(viewer)
gym.destroy_sim(sim)
