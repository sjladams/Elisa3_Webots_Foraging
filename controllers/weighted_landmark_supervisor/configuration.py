local = True
total_time = 200
# domain = [2.5,2.5]           # [40,20] / [20,10]
# \TODO if using non sym obstacle, change plotting (add mirroring)
domain = [[-1.25,1.25],[-1.25,1.25]]

# provide corner points of as: obstacle = [lower left, upper left, upper right, lower right]
# obstacle = [[2, 2], [2,3], [3,3], [3,2]]
obstacle = None

nest_location = [-1., 0]    # [5.,4.] / [5., 4.]
food_location = [1.,0]   # [30.,14.] / [15., 6.]
default_beacon_grid = [10,8]        # [20,16] / [10,8]

N_batch = 2
N_total = 44 # 500 / 100

# ampFactor = 30
kappa=1   #1
lam= 0.8#1
rew=1
rho = 0.01 #0.0001
rho_v = 0.001
default_epsilon = 0.01 #0.05 #5
exploration_rate = 0.01 #previously called default_epsilon
DEBUG=1
dt=0.25
target_range=0.4
# default_var = 10
clip_range = 0.4 #,, 2.
min_clip_range = 0.3

# elips_a = 0.008    # 0.002 / 0.008
# elips_c = 0.03     # 0.009 / 0.03
# elips_ampl = 1

# offset = 1 # 1e-6
# threshold = (1-default_rho)**90 * offset
threshold = 1e-6
# n = np.log(threshold/offset)/np.log(1-default_rho)
step_threshold = 1e-6 #1e-3   # 1e-7

move_type = 'add_switch' #'der'/ 'add' / 'add_switch'

numeric_step_margin = 0

use_weights_updating_v = False
use_rhov_2_init = True

adapt_range_option = 'weights' # weights, angle, no adaption

pick_option = 'sum'  # 'max 'max_element_wise' , 'average'
