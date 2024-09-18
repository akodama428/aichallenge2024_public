
import rclpy  # ROS2のPythonモジュール
from rclpy.node import Node
from nav_msgs.msg import Odometry
from autoware_auto_planning_msgs.msg import Trajectory, TrajectoryPoint
from autoware_auto_vehicle_msgs.msg import VelocityReport
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Point, Quaternion
import sys
import os
# -- Limit number of OPENBLAS library threads --
# On linux based operation systems, we observed a occupation of all cores by the underlying openblas library. Often,
# this slowed down other processes, as well as the planner itself. Therefore, it is recommended to set the number of
# threads to one. Note: this import must happen before the import of any openblas based package (e.g. numpy)
os.environ['OPENBLAS_NUM_THREADS'] = str(1)

import numpy as np
import datetime
import json
import time
import configparser
sys.path.append(os.path.dirname(__file__))
import graph_ltpl

class LocalPlanner(Node):
    def __init__(self):
        super().__init__('LocalPlanner')
        self.get_logger().info("Start LocalPlanner")
        self.is_ready = False
        self.velocity_report_ready = False
        self.ego_pose = None
        self.obj_list_ready = False
        self.obj_list = None
        self.is_offline_calculation_finish = False
        self.OfflineCalculation()

        self.create_subscription(Odometry, "in_odom", self.onOdometry, 1)
        self.create_subscription(VelocityReport, "/vehicle/status/velocity_status", self.onVelocityReport, 10)
        self.create_subscription(Float64MultiArray, "/aichallenge/objects", self.onPerception, 10)
        self.pub_traj_ = self.create_publisher(Trajectory, "output", 1)
        self.create_timer(1.0, self.onOnlineLoop)

    ## Check if input data is initialized. ##
    def isReady(self):
        if self.ego_pose is None:
            self.get_logger().info("The kinematic_state has not ready yet.")
            return False
        if self.is_offline_calculation_finish == False:
            self.get_logger().info("The offline calculation has not ready yet.")
            return False
        if self.obj_list_ready == False:
            self.get_logger().info("The dynamic objects data has not ready yet.")
            return False
        if self.velocity_report_ready == False:
            self.get_logger().info("The velocity report data has not ready yet.")
            return False           

        if self.is_ready == False:
            self.get_logger().info("online loop is ready now!")        
            self.is_ready = True
        return True

    def OfflineCalculation(self):
        # top level path (module directory)
        toppath = os.path.dirname(os.path.realpath(__file__)) + "/GraphBasedLocalTrajectoryPlanner"
        self.get_logger().info(f"toppath:{toppath}")
        sys.path.append(toppath)

        track_param = configparser.ConfigParser()
        if not track_param.read(toppath + "/params/driving_task.ini"):
            raise ValueError('Specified online parameter config file does not exist or is empty!')

        track_specifier = json.loads(track_param.get('DRIVING_TASK', 'track'))

        # define all relevant paths
        path_dict = {'globtraj_input_path': toppath + "/inputs/traj_ltpl_cl/traj_ltpl_cl_" + track_specifier + ".csv",
                     'graph_store_path': toppath + "/inputs/stored_graph.pckl",
                     'ltpl_offline_param_path': toppath + "/params/ltpl_config_offline.ini",
                     'ltpl_online_param_path': toppath + "/params/ltpl_config_online.ini",
                     'log_path': toppath + "/logs/graph_ltpl/",
                     'graph_log_id': datetime.datetime.now().strftime("%Y_%m_%d__%H_%M_%S")
                     }

        # intialize graph_ltpl-class
        self.ltpl_obj = graph_ltpl.Graph_LTPL.Graph_LTPL(path_dict=path_dict,
                                                         visual_mode=True,
                                                         log_to_file=True)
        # calculate offline graph
        self.ltpl_obj.graph_init()
        # set start pose based on first point in provided reference-line
        refline = graph_ltpl.imp_global_traj.src.import_globtraj_csv.\
                    import_globtraj_csv(import_path=path_dict['globtraj_input_path'])[0]
        pos_est = refline[0, :]
        heading_est = np.arctan2(np.diff(refline[0:2, 1]), np.diff(refline[0:2, 0])) - np.pi / 2

        # set start pos
        self.ltpl_obj.set_startpos(pos_est=pos_est,
                                   heading_est=heading_est)

        self.traj_set = {'straight': None}
        self.is_offline_calculation_finish = True

    ## Callback function for odometry subscriber ##
    def onVelocityReport(self, msg: VelocityReport):
        self.velocity_report_x = msg.longitudinal_velocity   # [m/s]
        self.velocity_report_y = msg.lateral_velocity    # [m/s]
        self.velocity_report_yawrate = msg.heading_rate  # [rad/s]
        self.velocity_report_ready = True 

    def onOdometry(self, msg: Odometry):
        self.ego_pose  = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        self.odom_header = msg.header
        
    ## Callback function for timer ##
    def onOnlineLoop(self):
        if self.isReady():
            for sel_action in ["right", "left", "straight", "follow"]:  # try to force 'right', else try next in list
                if sel_action in self.traj_set.keys():
                    break
            # -- CALCULATE PATHS FOR NEXT TIMESTAMP ----------------------------------------------------------------------------
            self.ltpl_obj.calc_paths(prev_action_id=sel_action,
                                     object_list=self.obj_list,
                                     blocked_zones=None)

            # -- CALCULATE VELOCITY PROFILE AND RETRIEVE TRAJECTORIES ----------------------------------------------------------
            self.traj_set = self.ltpl_obj.calc_vel_profile(pos_est=self.ego_pose,
                                                           vel_est=self.velocity_report_x)[0]
            self.get_logger().info(f"traj_set.kyes():{self.traj_set.keys()}, sel_action:{sel_action}")

            # -- SEND TRAJECTORIES TO CONTROLLER -- #
            for sel_action in ["right", "left", "straight", "follow"]:  # try to force 'right', else try next in list
                if sel_action in self.traj_set.keys():
                    break
            if self.traj_set[sel_action] is not None:
                output_traj= Trajectory()
                output_traj.header = self.odom_header
                for p in self.traj_set[f"{sel_action}"][0]:
                    traj_point = TrajectoryPoint()
                    traj_point.pose.position.x = p[1]
                    traj_point.pose.position.y = p[2]
                    traj_point.pose.position.z = 181.16197506872854
                    yaw = p[3]+np.pi/2  # Z軸周りの回転角度（ラジアン単位）
                    traj_point.pose.orientation.x = 0.0
                    traj_point.pose.orientation.y = 0.0
                    traj_point.pose.orientation.z = np.sin(yaw / 2)
                    traj_point.pose.orientation.w = np.cos(yaw / 2)
                    traj_point.longitudinal_velocity_mps = p[5]
                    traj_point.acceleration_mps2 = p[6]

                    stability_factor = 0.0018  # 仮の値
                    wheel_base = 2.0
                    ref_vel = traj_point.longitudinal_velocity_mps
                    ref_curvature = p[4]
                    traj_point.front_wheel_angle_rad = (1 + stability_factor * ref_vel * ref_vel) * ref_curvature * wheel_base
                    output_traj.points.append(traj_point)            
                self.pub_traj_.publish(output_traj)

    ## Callback function for predicted objects ##
    def onPerception(self, msg: Float64MultiArray):
        if msg.data is None:
            self.obj_list = None
        else:
            # self.get_logger().info(f"object apear! num of obj: {len(msg.data)//4}")
            obj_list = []
            for i in range(len(msg.data)//4):
                objA = {'X': msg.data[i * 4],
                        'Y': msg.data[i * 4 + 1],
                        'theta': 0.0,
                        'type': 'physical',
                        'id': i,
                        'length': msg.data[i * 4 + 3] / 10.0,
                        'width':  msg.data[i * 4 + 3] / 10.0,
                        'v': 0.0}
                obj_list.append(objA)
            self.obj_list = obj_list

        self.obj_list_ready = True            

    def getYawFromQuaternion(self, orientation: Quaternion):
        """
        Calculate yaw from quaternion
        """
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        return np.arctan2(siny_cosp, cosy_cosp)

def main(args=None):
    print('Hi LocalPlanner')
    rclpy.init(args=args)
    
    local_planner_class = LocalPlanner()
    rclpy.spin(local_planner_class)

    local_planner_class.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()