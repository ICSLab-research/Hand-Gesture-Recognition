import time
import logging
import sys, os
import threading
from threading import Thread
from queue import Queue
from tensorflow.keras.models import load_model
import numpy as np
from PyQt5 import QtWidgets
from PyQt5.QtCore import (QObject, pyqtSignal,
                          QRunnable, pyqtSlot,
                          QThreadPool)
from src.DSP import DSP as dsp
from src.radar_configs import DCA1000EVM_backend
from src.UI import _UI_
from src.UR import UR3_GESTURE as ur3

os.environ["CUDA_VISIBLE_DEVICES"] = "-1"
logging.basicConfig(
    format="%(asctime)s-%(levelname)s-%(message)s",
    level=logging.INFO
    )

#%%
class WorkerSignals(
        QObject
        ):
    '''
    Defines the signals available from a running worker thread.
    '''
    bin_data_signal = pyqtSignal(object)  ## Radar woker signal
    robot_signal    = pyqtSignal(object)  ## UR3 woker signal
    
#%%
class Thread_Prediction(
        Thread
        ):
    '''
    Prediction thread to run the TF model
    '''
    def __init__(
            self,
            group=None,
            target=None,
            name=None,
            args=(),
            kwargs={},
            Verbose=None
            ):
        Thread.__init__(
            self,
            group,
            target,
            name,
            args,
            kwargs
            )
        self._return = None

    def run(
            self
            ):
        if self._target is not None:
            self._return = self._target(
                *self._args,
                **self._kwargs
                )
    def join(
            self,
            *args
            ):
        Thread.join(
            self,
            *args
            )
        
        return self._return

#%%
class Radar(
        QRunnable
        ):
    '''
    Worker Radar thread to read raw data
    '''
    def __init__(
            self,
            *args,
            **kwargs
            ):
        super(
            Radar,
            self
            ).__init__(*args, **kwargs)
        self.read_raw_data = DCA1000EVM_backend.DCA1000()
        self.signals = WorkerSignals()

    @pyqtSlot(object)
    def run(
            self
            ):
        '''
        Initialise the runner function.
        '''
        while True:
            self.num, self.adcData = \
                self.read_raw_data._read_data_packet()
            self.signals.bin_data_signal.emit(
                self.adcData
                ) 
            
#%%            
class UR3(
        QRunnable
        ):
    '''
    Worker Robot thread to read data from Universal Robots
    '''
    def __init__(
            self,
            *args,
            **kwargs
            ):
        super(
            UR3,
            self
            ).__init__(*args, **kwargs)
        self.control_robot = ur3.GES_POS()
        self.signals = WorkerSignals()

    @pyqtSlot(object)
    def run(
            self
            ):
        '''
        Initialise the runner function.
        '''
        while True:
            self.tcp_pos_data = self.control_robot.read_ur_data(
                fps = 20,
                read_data = 'TCP Pos'
                )
            self.signals.robot_signal.emit(
                self.tcp_pos_data
                ) 
#%%       
class MainWindow(
        _UI_.Ui_MainWindow
        ):
    def __init__(
            self,
            *args,
            **kwargs
            ):
        super(
            MainWindow,
            self
            ).__init__(*args, **kwargs)
        self.mainWindow = QtWidgets.QMainWindow()
        self.setupUi(self.mainWindow)
        
        self.btn_start.clicked.connect(self.thread_read_radar)
        self.btn_UR.clicked.connect(self.thread_read_ur3)
        
        self.threadpool = QThreadPool.globalInstance()
        print("Multithreading with maximum %d threads" %
              self.threadpool.maxThreadCount())
        self.signals = WorkerSignals()
        self.radar = Radar()       
        self.ur3 = UR3()
        self.Line = np.array([], dtype = np.int16)
        self.temp = np.ones((1, 10),
                             dtype = np.int8).squeeze()*9
        self.temp1 = np.ones((1, 2),
                              dtype = np.int8).squeeze()*9
        self.micro = np.ones((40, 4, 2, 40))
        self.adcData = np.array([], dtype = np.int16)
        self.Ns = 64
        self.Nc = 128
        self.dsp = dsp._FFT_(
            Ns = self.Ns,
            Nc = self.Nc
            )
        self.Nr = 4
        self.IQ = 2
        self.num_Ns = self.Ns * self.Nr * self.IQ * self.Nc
        
        self.model_reg = load_model("model//model.h5")
        self.labels = [' ', 'counter clock-wise',
                       'clock-wise', 'push-down', 'push-up',
                       'zoom-in', 'zoom-out',
                       'to left', 'to right', '  ']
        self.idx_ges = 9
        self.ctrl = [9, 9]
        self.dis = 50e-3 # meter

    def connect_rs_cfg(self): ### DCA1000EVM Init ######
        self.radar.read_raw_data._bind_()
        self.radar.read_raw_data._cfg_fpga_()  
#%%
    def read_radar(
            self,
            UDP_packet
            ):   
        self.Line = np.concatenate(
            (self.Line, UDP_packet)
            )
        if len(self.Line)>=self.num_Ns:
            frame = self.Line[:self.num_Ns]
            self.Line = self.Line[self.num_Ns:] 
            data = self.dsp.pre_processing(
                np.array([frame])
                )
            fft_cube = np.expand_dims(
                data[:, :, 33, 44:84],
                axis = 0
                )
            self.micro = np.concatenate(
                (self.micro, fft_cube),
                axis = 0
                )
            self.micro = self.micro[-40:, :, :, :]
            self.graph.setImage(
                (
                    20*np.log10(
                        np.abs(
                            self.micro[:, 0, 0, :] + \
                       1j * self.micro[:, 0, 1, :]
                        )
                        )
                        ),
                    levels = [65, 80]
                    )
            win_ges = self.micro[20:, :, :, :].transpose(
                0, 3, 1, 2
                )
            if self.pred_check.isChecked() == True:
                th_pred = Thread_Prediction(
                    target=self.prediction,
                    args=(win_ges.reshape(1, 20, 40, 8),)
                    )
                th_pred.start()
                self.ctrl = th_pred.join()
                if self.ctrl_check.isChecked() == True:
                    self.thread_read_ur3()
                    th_ctrl = threading.Thread(
                        target=self.thread_shared_data,
                        args = (th_pred.join(),)
                        )
                    th_ctrl.start()
                    th_ctrl.join()
#%%
    def thread_read_radar(
            self
            ):
        self.connect_rs_cfg()
        self.radar.read_raw_data.start_record()
        self.btn_start.setEnabled(False)
        self.radar.signals.bin_data_signal.connect(self.read_radar)
        self.threadpool.start(self.radar)            

    def thread_read_ur3(
            self
            ):
        self.btn_UR.setEnabled(False)
        self.ur3.signals.robot_signal.connect(self.thread_control)
        self.threadpool.start(self.ur3)
        
#%%     
    def prediction(
            self,
            window
            ):
        predict = np.argmax(
            self.model_reg.predict_on_batch(window),
            axis=1
            )
        self.temp = np.concatenate(
            (self.temp, predict)
            )
        self.temp = self.temp[-10:]
        self.max_cnt = np.array(
            [np.argmax(np.bincount(self.temp))]
            )
        self.temp1 = np.concatenate(
            (self.temp1, self.max_cnt)
            )
        self.temp1 = self.temp1[-2:]
        
        return self.temp1
    
    def sig_ctrl(
            self,
            predict
            ):
        if predict[1] != predict[0]:
            self.idx_ges = predict[1]
            # logging.info(
            #     self.labels[int(self.idx_ges)]
            #     )
            text = self.labels[int(self.idx_ges)]
            self.label_7.setText(text)
            
            return self.idx_ges

    def thread_control(
            self,
            TCP_position
            ):
        signal_control = self.sig_ctrl(self.ctrl)
        # print(signal_control)
        
        try:
            if signal_control == 3:
                self.ur3.control_robot.set_realtime_TCP_pos(
                    TCP_position[0],
                    TCP_position[1],
                    TCP_position[2]-self.dis,
                    TCP_position[3],
                    TCP_position[4],
                    TCP_position[5],
                    self.dis
                    )
                
                # time.sleep(3)
            elif signal_control == 4:
                self.ur3.control_robot.set_realtime_TCP_pos(
                    TCP_position[0],
                    TCP_position[1],
                    TCP_position[2]+self.dis,
                    TCP_position[3],
                    TCP_position[4],
                    TCP_position[5],
                    self.dis
                    )
                # time.sleep(3)
                
            elif signal_control == 7:
                self.ur3.control_robot.set_realtime_TCP_pos(
                    TCP_position[0]+self.dis,
                    TCP_position[1],
                    TCP_position[2],
                    TCP_position[3],
                    TCP_position[4],
                    TCP_position[5],
                    self.dis
                    )
                # time.sleep(3)
            elif signal_control == 8:
                self.ur3.control_robot.set_realtime_TCP_pos(
                    TCP_position[0]-self.dis,
                    TCP_position[1],
                    TCP_position[2],
                    TCP_position[3],
                    TCP_position[4],
                    TCP_position[5],
                    self.dis
                    )
        except KeyboardInterrupt:
            print("closing robot connection")
            # Remember to always close the robot connection,
            # otherwise it is not possible to reconnect
            self.ur3.control_robot.close()
        except:
            self.ur3.control_robot.close()            
        
#%%   
app = QtWidgets.QApplication(sys.argv)
window = MainWindow()
window.mainWindow.show()
sys.exit(app.exec())