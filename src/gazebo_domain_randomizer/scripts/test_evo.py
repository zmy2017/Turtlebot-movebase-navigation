#! /usr/bin/python
import rospy
import math
import os
import traceback
import sys
import gazebo_env
import gazebo_domain_randomization
from evo.core import trajectory, sync, metrics
from evo.tools import file_interface
from evo.tools import plot
import matplotlib.pyplot as plt
import numpy as np
import evo.main_ape as main_ape
import evo.common_ape_rpe as common
from evo.tools.plot import PlotMode

if __name__ == "__main__":
    rospy.init_node("test_evo")

    #Modify the timestamps of the real_traj.txt alighing with sim_traj.txt 
    sim_path =os.path.join(os.path.dirname(__file__),"sim_traj.txt")
    real_path =os.path.join(os.path.dirname(__file__),"real_traj.txt")
    real_path_new =os.path.join(os.path.dirname(__file__),"real_traj_new.txt")
    sim_data =np.loadtxt(sim_path)
    real_data = np.loadtxt(real_path)
    sim_time_start=sim_data[0,0]
    real_time_start = real_data[0,0]
    time_offset = real_time_start-sim_time_start
    real_data[:, 0] -= time_offset
    np.savetxt(real_path_new,real_data,fmt='%.14f')
        
    traj_est = file_interface.read_tum_trajectory_file(sim_path)
    traj_ref = file_interface.read_tum_trajectory_file(real_path_new)
    # print(traj_ref)
    # print(traj_est)

    print("registering and aligning trajectories")
    traj_ref, traj_est = sync.associate_trajectories(traj_ref, traj_est)
    # print(traj_ref)
    # print(traj_est)
    
    #translation_part:Specify that APE calculations only consider translation errors between trajectories
    # In result , rmse can be regarded as an evaluation target
    pose_relation = metrics.PoseRelation.translation_part
    result = main_ape.ape(traj_ref,traj_est,pose_relation,est_name='estimate')
    print(result)

    
    plot_mode = plot.PlotMode.xy
    fig = plt.figure()
    ax = plot.prepare_axis(fig, plot_mode)
    plot.traj(ax, plot_mode, traj_ref, style="--", alpha=0.5)
    plot.traj_colormap(
            ax, result.trajectories['estimate'], result.np_arrays["error_array"], plot_mode,
            min_map=result.stats["min"], max_map=result.stats["max"])
    plt.show()