
import time
from src import UR
import numpy as np
import logging
logging.basicConfig(
    format="%(asctime)s-%(levelname)s-%(message)s",
    level=logging.INFO
    )

#%%
class GES_POS(
        object
        ):
    def __init__(self):
        # self.ROBOT_IP = '192.168.0.101'  # UR3 local IP
        self.ROBOT_IP = '192.168.88.128' # UR3 local IP Simulation
        self.acceletion = 0.9  # Robot acceleration value
        self.velocity = 1.0    # Robot speed value
        #  ARM UR3 POSITION (Base, Shouldel, Elbow, Wrist 1, Wrist 2, Wrist 3)
        self.start_pos = [-218, #   Base
                          -63,  #   Shoulder
                          -93,  #   Elbow
                          -20,  #   Wrist 1
                          88,   #   Wrist 2
                          0]    #   Wrist 3

        logging.info("Initializing Arm Robot !")
        self.robotModel = UR.robotModel.RobotModel()
        self.robot = UR.urScriptExt.UrScriptExt(
            host=self.ROBOT_IP,
            robotModel=self.robotModel
            )
        self.robot.reset_error()
        logging.info("Initialized !")
        time.sleep(2)
        
        self.robot.movej(
            q= np.radians(self.start_pos),
            a= self.acceletion,
            v= self.velocity
            )
        # starts the realtime control loop on the Universal-Robot Controller
        self.robot.init_realtime_control()  
        time.sleep(2) # just a short wait to make sure everything is initialised
        
    def read_ur_data(
            self,
            fps = 20,
            read_data = 'TCP Pos'
            ):
        """
        Parameters
        ----------
        fps : (int) Speed read data. The default is 20 fps.
        read_data : The current actual TCP vector : ([X, Y, Z, Rx, Ry, Rz]).
        X, Y, Z in meter, Rx, Ry, Rz in rad. The default is 'TCP Pos'. 
        
        If 'joint Pos':    
        The current actual joint angular position vector in rad : 
        [Base, Shoulder, Elbow, Wrist1, Wrist2, Wrist3]

        Returns
        -------
        TYPE
            DESCRIPTION.

        """
        if read_data == 'TCP Pos':    
            self.data = self.robot.get_actual_tcp_pose()
        elif read_data == 'joint Pos':
            self.data = self.robot.get_actual_joint_positions()
        # time.sleep((1/fps))
        
        return self.data
    
    def set_realtime_TCP_pos(
            self,
            X,
            Y,
            Z,
            Rx,
            Ry,
            Rz,
            dis = 30,   # mm 
            ):
        dis = dis
        if X < 0 or Y <0 or Z < 0:
            dis = -30
        self.robot.set_realtime_pose([
            X,
            Y,
            Z,
            Rx,
            Ry,
            Rz
            ])
        
    def close(
            self
            ):
        """
        Remember to always close the robot connection,
        otherwise it is not possible to reconnect
        Returns
        -------
        None.
        Closing robot connection

        """
        self.robot.close()









