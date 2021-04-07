from configuration import *
from domain import *
from agents import *

import matplotlib.pyplot as plt
from scipy.interpolate import griddata
from matplotlib import cm
from scipy.spatial import Voronoi, voronoi_plot_2d

if local:
    FOLDER_LOCATION = './figures/'
else:
    FOLDER_LOCATION = './figures_manuels_desk/'

class Simulations:
    def __init__(self, supervisor, timestep):
        self.grid = Grid(obstacle=obstacle)
        self.total_trips_abs = dict()
        self.total_trips_rel = dict()

        self.agents = Agents(self.grid)

        _ = self.agents.release_foragers(1,supervisor, timestep)

        self.update_agents()

        self.agents.switch_states()

        # In theory we do not have to update the agents again, since we updated all info in the switch_state function
        # self.update_agents()

    def sim_pre_step(self, time_step, emitter, supervisor, timestep):
        N_till_now = (time_step+1)*N_batch

        # ACTION
        if N_till_now < N_total:
            release_tags = self.agents.release_foragers(N_batch,supervisor, timestep)
            # \todo remove beacon update, it isn't even sure if this are the only foragers within its range
            self.agents.update_beacon_w_v(0, release_tags)

        # UPDATE
        self.update_agents()

        # ACTION
        self.agents.pre_steps(emitter)

    def sim_after_step(self, time_step, switch_time=250):
        # ACTION
        self.agents.after_steps()

        # UPDATE
        self.update_agents()

        # ACTION
        if time_step >= switch_time:
            self.agents.switch_states()

        # UPDATE
        ## Done within switch_step

        # ACTION
        self.agents.evaporate_weights()
        self.agents.update_weights()

        # UPDATE
        self.update_agents()

        # STORE
        self.store_nr_trips(time_step)

    def update_agents(self):
        self.agents.check_states()
        self.agents.adapt_ranges() # /todo do we want to update the ranges all the time?
        self.agents.find_neighs()

    def store_nr_trips(self,t):
        # self.total_trips[t] = sum([self.ants.ants[ant_tag].trips for ant_tag in self.ants.ants])

        trips = [self.agents.agents[tag].trips for tag in self.agents.agents]
        self.total_trips_abs[t] = sum(trips)
        self.total_trips_rel[t] = sum(trips) / len(self.agents.agents)

    def plt_only_weights_and_vectors(self, to_plot='W', fig_tag=None):
        if len(self.agents.beac_tags) >3:
            # vor = Voronoi([self.agents.agents[beac_tag].pt[1] for beac_tag in self.agents.beac_tags])
            vor = Voronoi([np.array([self.agents.agents[beac_tag].pt[1][0],
                                     -self.agents.agents[beac_tag].pt[1][1]]) for beac_tag in self.agents.beac_tags])
            voronoi_plot_2d(vor, show_vertices=False)

        # /TODO now plotting only vectors larger than threshold, what we want?
        if to_plot == 'W1':
            checked_ws = self.agents.check_weights(to_check='W1', thres=step_threshold)

            for beac_tag in checked_ws:
                size = np.log(checked_ws[beac_tag] / max(checked_ws.values()) + 1) * 10
                # plt.plot([self.agents.agents[beac_tag].pt[1][0]], [self.agents.agents[beac_tag].pt[1][1]],
                #          'o', color='black', markersize=size)
                plt.plot([self.agents.agents[beac_tag].pt[1][0]], [-self.agents.agents[beac_tag].pt[1][1]],
                         'o', color='black', markersize=size)

                if np.linalg.norm(self.agents.agents[beac_tag].v[0]) > step_threshold:
                    arrow = self.normalize(self.agents.agents[beac_tag].v[0]) * dt
                    # plt.plot([self.agents.agents[beac_tag].pt[1][0],self.agents.agents[beac_tag].pt[1][0]+arrow[0]],
                    #          [self.agents.agents[beac_tag].pt[1][1],self.agents.agents[beac_tag].pt[1][1]+arrow[1]], color='black')
                    plt.plot([self.agents.agents[beac_tag].pt[1][0],self.agents.agents[beac_tag].pt[1][0]+arrow[0]],
                             [-self.agents.agents[beac_tag].pt[1][1],-(self.agents.agents[beac_tag].pt[1][1]+arrow[1])], color='black')


        elif to_plot == 'W2':
            checked_ws = self.agents.check_weights(to_check='W2', thres=step_threshold)

            for beac_tag in checked_ws:
                size = np.log(checked_ws[beac_tag]/max(checked_ws.values()) + 1) * 10
                # plt.plot([self.agents.agents[beac_tag].pt[1][0]], [self.agents.agents[beac_tag].pt[1][1]],
                #          'o', color='black', markersize=size)
                plt.plot([self.agents.agents[beac_tag].pt[1][0]], [-self.agents.agents[beac_tag].pt[1][1]],
                         'o', color='black', markersize=size)

                if np.linalg.norm(self.agents.agents[beac_tag].v[1]) > step_threshold:
                    arrow = self.normalize(self.agents.agents[beac_tag].v[1]) * dt
                    # plt.plot([self.agents.agents[beac_tag].pt[1][0],self.agents.agents[beac_tag].pt[1][0]+arrow[0]],
                    #          [self.agents.agents[beac_tag].pt[1][1],self.agents.agents[beac_tag].pt[1][1]+arrow[1]], color='black')
                    plt.plot([self.agents.agents[beac_tag].pt[1][0], self.agents.agents[beac_tag].pt[1][0] + arrow[0]],
                             [-self.agents.agents[beac_tag].pt[1][1], -(self.agents.agents[beac_tag].pt[1][1] + arrow[1])],
                                color='black')

        elif to_plot == 'W':
            checked_w0s = self.agents.check_weights(to_check='W1', thres=step_threshold)
            checked_w1s = self.agents.check_weights(to_check='W2', thres=step_threshold)

            checked_ws = self.agents.check_weights(to_check='W', thres=step_threshold)

            for beac_tag in checked_ws:
                size = np.log(checked_ws[beac_tag] / max(checked_ws.values()) + 1) * 10
                if np.isnan(size):
                    print('chekc hier')
                # plt.plot([self.agents.agents[beac_tag].pt[1][0]], [self.agents.agents[beac_tag].pt[1][1]],
                #          'o', color='black', markersize=size)
                plt.plot([self.agents.agents[beac_tag].pt[1][0]], [-self.agents.agents[beac_tag].pt[1][1]],
                         'o', color='black', markersize=size)

                if beac_tag in checked_w0s and np.linalg.norm(self.agents.agents[beac_tag].v[0]) > step_threshold:
                    arrow0 = self.normalize(self.agents.agents[beac_tag].v[0])*dt
                    # plt.plot([self.agents.agents[beac_tag].pt[1][0], self.agents.agents[beac_tag].pt[1][0] + arrow0[0]],
                    #          [self.agents.agents[beac_tag].pt[1][1], self.agents.agents[beac_tag].pt[1][1] + arrow0[1]], color='black')
                    plt.plot([self.agents.agents[beac_tag].pt[1][0], self.agents.agents[beac_tag].pt[1][0] + arrow0[0]],
                             [-self.agents.agents[beac_tag].pt[1][1], -(self.agents.agents[beac_tag].pt[1][1] + arrow0[1])], color='black')
                if beac_tag in checked_w1s and np.linalg.norm(self.agents.agents[beac_tag].v[1]) > step_threshold:
                    arrow1 = self.normalize(self.agents.agents[beac_tag].v[1])*dt
                    # plt.plot([self.agents.agents[beac_tag].pt[1][0], self.agents.agents[beac_tag].pt[1][0] + arrow1[0]],
                    #          [self.agents.agents[beac_tag].pt[1][1], self.agents.agents[beac_tag].pt[1][1] + arrow1[1]], color='blue')
                    plt.plot([self.agents.agents[beac_tag].pt[1][0], self.agents.agents[beac_tag].pt[1][0] + arrow1[0]],
                             [-self.agents.agents[beac_tag].pt[1][1], -(self.agents.agents[beac_tag].pt[1][1] + arrow1[1])], color='blue')

        # plt.plot([nest_location[0], food_location[0]],
        #          [nest_location[1], food_location[1]], 'r*')
        plt.plot([nest_location[0], food_location[0]],
                 [-nest_location[1], -food_location[1]], 'r*')

        # plt.plot([self.agents.agents[forager_tag].pt[1][0] for forager_tag in self.agents.forager_tags if
        #           self.agents.agents[forager_tag].mode[1] == 0],
        #          [self.agents.agents[forager_tag].pt[1][1] for forager_tag in self.agents.forager_tags if
        #           self.agents.agents[forager_tag].mode[1] == 0], 'g*', markersize=2)
        plt.plot([self.agents.agents[forager_tag].pt[1][0] for forager_tag in self.agents.forager_tags if
                  self.agents.agents[forager_tag].mode[1] == 0],
                 [-self.agents.agents[forager_tag].pt[1][1] for forager_tag in self.agents.forager_tags if
                  self.agents.agents[forager_tag].mode[1] == 0], 'g*', markersize=2)
        # plt.plot([self.agents.agents[forager_tag].pt[1][0] for forager_tag in self.agents.forager_tags if
        #           self.agents.agents[forager_tag].mode[1] == 1],
        #          [self.agents.agents[forager_tag].pt[1][1] for forager_tag in self.agents.forager_tags if
        #           self.agents.agents[forager_tag].mode[1] == 1], 'y*', markersize=2)
        plt.plot([self.agents.agents[forager_tag].pt[1][0] for forager_tag in self.agents.forager_tags if
                  self.agents.agents[forager_tag].mode[1] == 1],
                 [-self.agents.agents[forager_tag].pt[1][1] for forager_tag in self.agents.forager_tags if
                  self.agents.agents[forager_tag].mode[1] == 1], 'y*', markersize=2)

        plt.xlim(domain[0][0]-1, domain[0][1]+1)
        plt.ylim(domain[1][0]-1,domain[1][1]+1)

        for line in self.grid.ref_lines:
            plt.plot([item[0] for item in line], [item[1] for item in line], 'r')

        # plt.colorbar()
        if to_plot == 'W1' and fig_tag:
            plt.savefig(FOLDER_LOCATION + 'W1_WEIGHTS/' + str(to_plot) + '_' + str(fig_tag) + '.png')
            plt.close()
        elif to_plot == 'W2' and fig_tag:
            plt.savefig(FOLDER_LOCATION +'W2_WEIGHTS/' + str(to_plot) + '_' + str(fig_tag) + '.png')
            plt.close()
        elif to_plot == 'W' and fig_tag:
            plt.savefig(FOLDER_LOCATION +'W_WEIGHTS/' + str(to_plot) + '_' + str(fig_tag) + '.png')
            plt.close()
        else:
            plt.show()
            plt.close()


    def plot_trips(self,total_time,fig_tag=None):
        trips_sequence = np.array([self.total_trips_abs[time] for time in range(0,total_time)]) / N_total

        plt.plot(np.array(range(0,total_time))*dt, trips_sequence, 'r')
        plt.xlabel("Time")
        plt.ylabel("#Trips / #Agents")

        if fig_tag:
            plt.savefig(FOLDER_LOCATION + 'total_trips_' + str(fig_tag) + '.png')
            plt.close()
        else:
            plt.show()

    @staticmethod
    def normalize(item):
        return_value = item / np.linalg.norm(item)
        if np.isnan(return_value).any():
            print('return value in to normalize value')
            return None
        else:
            return return_value