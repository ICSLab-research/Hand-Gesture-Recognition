from src import UR
#import URplus #import if any UPplus modules is needed

class RobotConnector(object):
    '''
    Class to hold all connection to the Universal Robot and plus devises

    Input parameters:

    '''


    def __init__(self, robotModel, host, hasForceTorque=False, conf_filename=None):
        '''
        Constructor see class description for more info.
        '''
        if(False):
            assert isinstance(robotModel, UR.robotModel.RobotModel)  ### This line is to get code completion for RobotModel
        self.RobotModel = robotModel
        self.RobotModel.ipAddress = host
        self.RobotModel.hasForceTorqueSensor = hasForceTorque
        self.RealTimeClient = UR.realTimeClient.RealTimeClient(robotModel)
        self.DataLog = UR.dataLog.DataLog(robotModel)
        self.RTDE = UR.rtde.RTDE(robotModel, conf_filename=conf_filename)
        self.DashboardClient = UR.dashboard.DashBoard(robotModel)
        self.ForceTourqe = None
        # if hasForceTorque:
        #     self.ForceTourqe = URplus.forceTorqueSensor.ForceTorqueSensor(robotModel)

        logger = UR.dataLogging.DataLogging()
        name = logger.AddEventLogging(__name__)
        self.__logger = logger.__dict__[name]
        self.__logger.info('Init done')


    def close(self):
        self.DataLog.close()
        self.RTDE.close()
        self.RealTimeClient.Disconnect()
        self.DashboardClient.close()
        if self.ForceTourqe is not None:
            self.ForceTourqe.close()
