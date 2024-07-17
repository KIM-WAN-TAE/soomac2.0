from isaacgym import gymapi
from isaacgym import gymutil
import random

gym = gymapi.acquire_gym()

sim_params = gymapi.SimParams()

sim_params.up_axis = gymapi.UP_AXIS_Z
sim_params.gravity = gymapi.Vec3(0.0, 0.0, -9.81)
sim_params.dt = 1 / 60
sim_params.substeps = 2
sim_params.use_gpu_pipeline = False

sim_params.physx.num_position_iterations = 6
sim_params.physx.num_velocity_iterations = 0
sim_params.physx.rest_offset = 0.001
sim_params.physx.contact_offset = 0.02
sim_params.physx.use_gpu = True

sim = gym.create_sim(0, 0, gymapi.SIM_PHYSX, sim_params)

# configure the ground plane
plane_params = gymapi.PlaneParams()
plane_params.normal = gymapi.Vec3(0, 0, 1) # z-up!
plane_params.distance = 0
plane_params.static_friction = 1
plane_params.dynamic_friction = 1
plane_params.restitution = 0

# create the ground plane
gym.add_ground(sim, plane_params)
# asset_root = '/home/choiyoonji/isaacgym/python/RL-study/ur5e/ur_description'
asset_root = "/home/choiyj/catkin_ws/src/soomac/asset/soomac_description/urdf/"
# asset_root = "/home/choiyj/catkin_ws/src/soomac/asset/dsr_description/urdf/"
# asset_file = "m0609.urdf"
asset_file = "soomac.urdf"
# asset_file = "lk_ur5e_with_allegro_right.urdf"
asset_options = gymapi.AssetOptions()
asset_options.fix_base_link = True
# asset_options.flip_visual_attachments = False
# asset_options.armature = 0.01
asset = gym.load_asset(sim, asset_root, asset_file, asset_options)
num_robot_bodies = gym.get_asset_rigid_body_count(asset)
num_robot_dofs = gym.get_asset_dof_count(asset)
body_info = gym.get_asset_rigid_body_dict(asset)

print('rigid body : ', num_robot_bodies)
print('DOFs : ', num_robot_dofs)
print('body dict : ', body_info)

num_envs = 1
envs_per_row = 12
env_spacing = 0.5
env_lower = gymapi.Vec3(-env_spacing, 0.0, -env_spacing)
env_upper = gymapi.Vec3(env_spacing, env_spacing, env_spacing)

# cache some common handles for later use
envs = []
actor_handles = []

# create and populate the environments
for i in range(num_envs):
    env = gym.create_env(sim, env_lower, env_upper, envs_per_row)
    envs.append(env)

    height = 2.0

    pose = gymapi.Transform()
    pose.p = gymapi.Vec3(10.0, height, 0.0)

    actor_handle = gym.create_actor(env, asset, pose, "MyActor", i, 1)
    print('ouhfowefhoiwejfweo')
    actor_handles.append(actor_handle)
    
cam_props = gymapi.CameraProperties()
viewer = gym.create_viewer(sim, cam_props)

    
while not gym.query_viewer_has_closed(viewer):

    # step the physics
    gym.simulate(sim)
    gym.fetch_results(sim, True)

    # update the viewer
    gym.step_graphics(sim)
    gym.draw_viewer(viewer, sim, True)

    # Wait for dt to elapse in real time.
    # This synchronizes the physics simulation with the rendering rate.
    gym.sync_frame_time(sim)
    
gym.destroy_viewer(viewer)
gym.destroy_sim(sim)