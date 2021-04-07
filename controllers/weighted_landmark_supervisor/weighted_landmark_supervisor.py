"""weighted_landmark_supervisor controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Supervisor
import struct
from functions import *

# Import Simulations Stuff
from simulation import *

# Set up Webots stuff
WALL_THR = 300
supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())
emitter = supervisor.getDevice('emitter')

# Set up Simulation stuff
simulation = Simulations(supervisor, timestep)
simulation.plt_only_weights_and_vectors(to_plot='W', fig_tag=-1)

for t in range(0,int(total_time/dt)):
    dumy = supervisor.step(timestep)

    timestep = int(supervisor.getBasicTimeStep())
    simulation.sim_pre_step(t, emitter, supervisor, timestep)

    max_tdelta = simulation.agents.get_max_webot_tdelta()

    for i in range(0,max_tdelta):
        dumy = supervisor.step(timestep)
        timestep = int(supervisor.getBasicTimeStep())

    simulation.sim_after_step(t, switch_time=0)

    simulation.plt_only_weights_and_vectors(to_plot='W', fig_tag=t)

print('zijn hier')





# robot_names = ['ELISA3-' + str(i) for i in range(0,11)]
#
# robot_dict = {}
#
# for count, robot_name in enumerate(robot_names):
#     robot_dict[robot_name] = RobotController(count, robot_name, supervisor,timestep)
#
# pol_moves = {1: np.array([0.3, 1.*np.pi]),
#              0: np.array([0.3, 0.*np.pi])}
#
# iterations = 0
# while supervisor.step(timestep) != -1:
#     timestep = int(supervisor.getBasicTimeStep())
#
#     for count, robot_name in enumerate(robot_names):
#         move = np.array([0.2, np.random.uniform() * 2*np.pi])
#         robot_dict[robot_name].prepare_step(emitter,pol_move = move)
#         robot_dict[robot_name].update_step(emitter)
#
#     iterations += 1
#
#     # STEP
#     rot_times = [robot_dict[robot_name].waiting_time for count, robot_name in enumerate(robot_names)]
#     for i in range(0,max(rot_times)):
#         dumy = supervisor.step(timestep)
#         timestep = int(supervisor.getBasicTimeStep())
