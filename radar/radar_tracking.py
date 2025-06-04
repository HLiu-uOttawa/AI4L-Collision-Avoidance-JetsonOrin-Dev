import os
import sys
from typing import List
import pandas as pd
import numpy as np
from datetime import datetime
import csv
import multiprocessing as mp
import time
from datetime import  timedelta
from constants import RADAR_DETECTION_TYPE
from tracking.DetectionsAtTime import DetectionDetails, DetectionsAtTime
from radar.cfar import get_range_bin_for_indexs
from radar.configuration.RunType import RunType

from radar.configuration.RadarConfiguration import RadarConfiguration

from radar.radarprocessing.FDDataMatrix import FDSignalType
from radar.radarprocessing.RadarDataWindow import RadarDataWindow
from radar.dataparsing.td_textdata_parser import read_columns

from radar.radarprocessing.get_td_sensor_data import get_td_data_voltage
from radar.radarprocessing.radar_processing_functions import movmean2d, compensatePathloss_dB, Clutter_rejection

class RadarTracking():
    def __init__(self, 
                 radar_configuration: RadarConfiguration,
                 start_time: pd.Timestamp = pd.Timestamp.now(),
                 radar_data_queue: mp.Queue = None):
        
        self.config = radar_configuration
        self.radar_data_queue = radar_data_queue
        self.start_time = start_time
        # output directory will be the given folder, with a timestamp and 'radar' appended to it
        self.output_dir = os.path.join(self.config.output_path, self.start_time.strftime('%Y-%m-%d_%H-%M-%S'), 'radar')
        
        if self.config.run_type == RunType.LIVE:
            self.radar_module = self.config.connect_get_radar_module()
            if (not self.radar_module.connected):
                print("Radar module is NOT connected. Exiting.")
                sys.exit("Could not connect to radar module. Please check the connection, or disable the radar with the '--skip-radar' flag.")
            self.bin_size_meters = self.radar_module.sysParams.tic / 1000000
        
        self.radar_window = RadarDataWindow(cfar_params=self.config.cfar_params, 
                                            start_time=self.start_time,
                                            bin_size=self.config.bin_size_meters,
                                            f_c=self.config.f_c,
                                            capacity=self.config.processing_window,
                                            run_velocity_measurements=False)
        self.count_between_processing = 5

    def object_tracking(self, stop_event):
        # If this is a rerun, read the data from the folder until it's completed
        if self.config.run_type == RunType.RERUN:
            self.process_data_from_folder()

        # If this is a live run, keep reading the data from the radar until the stop event is set
        elif self.config.run_type == RunType.LIVE:
            self.process_live_data(stop_event)

    def process_radar_data(self, td_data, max_rangefft1, max_rangefft2):
        """
        Obtain range estimate from radar data
        """
        fc_start        = 24e9       
        fc_stop         = 24.75e9
        delta_f         = fc_stop - fc_start
        c               = 3e8         # Speed of light (m/s)
        ramp_time       = 1e-3        # Ramp time (s)
        numSamples      = 1024        # Number of range bins
        MBW             = 750e6       # Clutter-rejection bandwidth
        wavelength      = c/fc_stop  # Wavelength (m)
        d               = 0.00625     # Element spacing (m)
        Radar_trade_off_flag = 0     #For long range set it to 1, for high velocity set it to 0`
        Threshold_dB    = 10
        
        range_bin = c/(2*delta_f)
        range_axis   = np.linspace(0 ,range_bin*(numSamples/2-1) , int(numSamples/2))
        range_axis_reversed  = range_axis[::-1]
        x = pd.DataFrame(td_data.td_data, columns=["I1", "Q1", "I2", "Q2"])
        
        # Simulate the timestamp as the current time to we can use the real-time windowing.
        # Add a 0.08 second delay between processing since we expect it to take roughly that long to get the radar data

        timestamp = td_data.timestamp
        #new_td_data.timestamp = pd.Timestamp.now() #Change

        sig1 = (x.I1  + 1j*x.Q1 ) / max(abs(x.I1  + 1j*x.Q1 )) 
        sig2 = (x.I2  + 1j*x.Q2 ) / max(abs(x.I2  + 1j*x.Q2 )) 

        fft1 = np.fft.fft(sig1 , n = 1024) /1024 
        fft2 = np.fft.fft(sig2 , n = 1024) /1024 
        
        fft1 = np.transpose(fft1)
        fft2 = np.transpose(fft2)
        
        rangefft1 = fft1[int(numSamples/2):  ]
        rangefft2 = fft2[int(numSamples/2):  ]

        cell_max_rangefft1 = max(abs(rangefft1)) 
        cell_max_rangefft2 = max(abs(rangefft2))

        if cell_max_rangefft1 > max_rangefft1:
            max_rangefft1 = cell_max_rangefft1
        if cell_max_rangefft2 > max_rangefft2:
            max_rangefft2 = cell_max_rangefft2
        
        rangefft1 = rangefft1/ max_rangefft1  #max(abs(rangefft1)) 
        rangefft2 = rangefft2/  max_rangefft2  #max(abs(rangefft2)) 
        
        rangefft1 = compensatePathloss_dB(rangefft1, range_axis_reversed, 1.15)
        rangefft2 = compensatePathloss_dB(rangefft2, range_axis_reversed, 1.15)

        rangefft1, a, time1 = Clutter_rejection(rangefft1, range_axis, 1, 1, MBW)
        rangefft2, a, time2 = Clutter_rejection(rangefft2, range_axis, 1, 2, MBW)

        avg_fft_linear =abs((abs(rangefft1) - abs(rangefft2)))
        return avg_fft_linear, timestamp, max_rangefft1, max_rangefft2



    def process_live_data(self, stop_event):
        """
        Process the radar data from the radar module until the stop event is set.
        """
        if self.config.record_data:
            print(f"Running radar tracking on live data. Recording raw results to folder: {self.output_dir}")
            os.makedirs(self.output_dir, exist_ok=True)
            self.export_radar_config_to_file(self.output_dir)
        else:
            print("Running radar tracking on live data. Not recording results.")


        #Set parameters
        fc_start        = 24e9       
        fc_stop         = 24.75e9
        delta_f         = fc_stop - fc_start
        c               = 3e8         # Speed of light (m/s)
        numSamples = 1024
        Radar_trade_off_flag = 0     #For long range set it to 1, for high velocity set it to 0`
        Threshold_dB    = 10
        range_bin = c/(2*delta_f)
        range_axis   = np.linspace(0 ,range_bin*(numSamples/2-1) , int(numSamples/2))
        range_axis_reversed      = range_axis[::-1]
        timestamps = []
        avglinear = np.array([])
        #range_bin = c/(2*delta_f)
        i=0
        max_rangefft1, max_rangefft2 = 0,0


        start_time = pd.Timestamp.now()
        batching_time = 1
        time_window = timedelta(seconds=batching_time)
        window_start= pd.Timestamp.now()
        window_end = window_start + time_window
        while not stop_event.is_set():
            voltage_data = get_td_data_voltage(self.radar_module)
            if voltage_data is None:
                # There was likely an error - reset error code, try again
                self.radar_module.error = False
                continue
            
            avg_fft_linear,timestamp, max_rangefft1, max_rangefft2 = self.process_radar_data(voltage_data, max_rangefft1, max_rangefft2)         
            timestamps.append(timestamp)
            # print("Max range fft1: ", max_rangefft1)
            # print("Max range fft2: ", max_rangefft2)
            if avglinear.size == 0:
                avglinear = avg_fft_linear
            else:
                avglinear = np.column_stack((  avglinear, avg_fft_linear))
                
            window_size = int(20) #Adjust window size
   
            #Gather multiple frames for window
            window_start= pd.Timestamp.now()
        
            if(i%window_size== window_size-1  ): #or window_start> window_end 
                #print(i)
                window_end = window_start + time_window
                start_time = pd.Timestamp.now()
                if avglinear.size > numSamples:
                    
                    if Radar_trade_off_flag:
                        avglinear = movmean2d(avglinear, 3, 139, '') 
                    else:
                        avglinear = movmean2d(avglinear, 3, 5, '') 

                    avg_rangeFFT_db = 20*np.log10(abs(avglinear))
                    
                    #Reset window
                    avglinear = np.array([])
                
                    nFrames  = len(avg_rangeFFT_db[0,:])
                    
                    rangeindex=np.linspace(0, nFrames-1, nFrames).astype(int)

                    start_index = i-nFrames+1
                    for k in rangeindex:
                        peakBin= np.argmax(avg_rangeFFT_db[:,k]) 
                        peakVal = avg_rangeFFT_db[peakBin,k]

                        if peakVal > np.mean( avg_rangeFFT_db[:,k] ) + Threshold_dB:
                            range_est= range_axis_reversed[peakBin]
                            self.radar_data_queue.put(DetectionsAtTime(timestamps[k+start_index],RADAR_DETECTION_TYPE, [range_est] )  )
                        else:
                            range_est= 0
            i = i+1

            if self.config.record_data:
                voltage_data.print_data_to_file(self.output_dir)
            
            #self.process_time_domain_data(voltage_data)
            
    def process_data_from_folder(self):
        """
        Process the radar data from the folder specified in the configuration.
        """
        directory_to_process = self.config.source_path
        print(f"Processing prerecorded radar data from folder {directory_to_process}.")
        
        # List all files in the directory
        files = os.listdir(directory_to_process)
        
        # Filter the files based on the naming convention, and sort them
        txt_files = [f for f in files if f.endswith('.txt')]
        txt_files.sort()

        #State parameters for radar processing
        fc_start        = 24e9       
        fc_stop         = 24.75e9
        delta_f         = fc_stop - fc_start
        c               = 3e8         # Speed of light (m/s)
        ramp_time       = 1e-3        # Ramp time (s)
        numSamples      = 1024        # Number of range bins
        MBW             = 750e6       # Clutter-rejection bandwidth
        wavelength      = c/fc_stop  # Wavelength (m)
        d               = 0.00625     # Element spacing (m)
        Radar_trade_off_flag = 0     #For long range set it to 1, for high velocity set it to 0`
        Threshold_dB    = 10
        time_vector = np.linspace(1*ramp_time, len(txt_files)*ramp_time, len(txt_files))
        range_bin = c/(2*delta_f)
        range_axis   = np.linspace(0 ,range_bin*(numSamples/2-1) , int(numSamples/2))
        range_axis_reversed      = range_axis[::-1]
        range_est = np.zeros(len(txt_files))   

        avglinear = np.array([])
        i = 0  #i Represnets which frame is currently being assessed 
        timestamps = []

        max_rangefft1, max_rangefft2 = 0,0

        deltaT = pd.Timestamp.now() -pd.Timestamp.now() 
        # Process each file one by one
        batching_time = 1
        time_window = timedelta(seconds=batching_time)
        window_start= pd.Timestamp.now()
        window_end = window_start + time_window
        for file_name in txt_files:
            
            t1 =  pd.Timestamp.now() 
            file_path = os.path.join(directory_to_process, file_name)
            new_td_data = read_columns(file_path)
            
            #self.process_time_domain_data(new_td_data) #Change 
            avg_fft_linear,timestamp, max_rangefft1, max_rangefft2 = self.process_radar_data(new_td_data, max_rangefft1, max_rangefft2)
            timestamps.append(timestamp)
       
            if avglinear.size == 0:
                avglinear = avg_fft_linear
            else:
                avglinear = np.column_stack((  avglinear, avg_fft_linear))
                
            #window_size = int(len(txt_files)/300) #Adjust window size
            window_start = pd.Timestamp.now()
            #Gather multiple frames for window
            if(window_start> window_end): #or i == len(txt_files)-1 
                window_end = window_start + time_window
                if avglinear.size > numSamples:
                    
                    if Radar_trade_off_flag:
                        avglinear = movmean2d(avglinear, 3, 139, '') 
                    else:
                        avglinear = movmean2d(avglinear, 3, 5, '') 

                    avg_rangeFFT_db = 20*np.log10(abs(avglinear))
                    
                    #Reset window
                    avglinear = np.array([])
                
                    nFrames  = len(avg_rangeFFT_db[0,:])
                    
                    rangeindex=np.linspace(0, nFrames-1, nFrames).astype(int)

                    start_index = i-nFrames+1
                    for k in rangeindex:
                        peakBin= np.argmax(avg_rangeFFT_db[:,k]) 
                        peakVal = avg_rangeFFT_db[peakBin,k]

                        if peakVal > np.mean( avg_rangeFFT_db[:,k] ) + Threshold_dB:
                            range_est[k+start_index] = range_axis_reversed[peakBin]
                            
                            self.radar_data_queue.put(DetectionsAtTime(timestamps[k+start_index],RADAR_DETECTION_TYPE, [range_est[k+start_index]] )  )
                        else:
                            range_est[k+start_index] = 0
                            #self.radar_data_queue.put([timestamps[k+start_index], range_est[k+start_index],k+start_index ])
            i = i+1
            t2 =  pd.Timestamp.now() 
            
            delta = t2 - t1
            #print("timestamps")
            #print(delta)
            deltaT = delta+ deltaT

            time.sleep(0.001) # Change back 0.08
        print("All data from folder is done")
        
    def process_time_domain_data(self, td_data):
        # Add the raw TD record to the radar window
        self.radar_window.add_raw_record(td_data)
        # Call method to process the latest data
        self.radar_window.process_data()
        
        # detections = self.radar_window.get_most_recent_detections_split_xy()
        detections = self.radar_window.get_detections_combined_xy()
        self.send_object_tracks_to_queue(detectionsAtTime=detections) # Send the object tracks to the queue
    
    def export_radar_config_to_file(self, output_dir, output_file="RadarConfigurationReport.txt"):
        """
        Exports the radar configuration settings to a text file in a formatted structure.

        Args:
            output_file (str): The name of the file to write the radar configuration report.
        """
        current_date = self.start_time.strftime("%Y-%m-%d")
        start_time = self.start_time.now().strftime("%H:%M:%S.%f")[:-3]
        sysParmas = self.radar_module.sysParams
         # Format the bin size to three decimal points
        formatted_bin_size_mm = f"{(self.bin_size_meters*1000):.3f}"

        report_content = (
            f"Date:\t{current_date}\n"
            f"Start Time:\t{start_time}\n"
            f"Interface:\tEthernet\n"
            f"Start-Frequency [MHz]:\t{self.config.minimum_frequency_mhz}\n"
            f"Stop-Frequency [MHz]:\t{self.config.maximum_frequency_mhz}\n"
            f"Ramp Time [ms]:\t{self.config.ramp_time_fmcw_chirp}\n"
            f"Attenuation [dB]:\t{sysParmas.atten}\n"
            f"Bin Size [mm]:\t{formatted_bin_size_mm}\n"
            f"Number of Samples:\t1024\n"
            f"Bin Size [Hz]:\t{sysParmas.freq_bin}\n"
            f"Zero Pad Factor:\t{sysParmas.zero_pad}\n"
            f"Normalization:\t{sysParmas.norm}\n"
            f"Active Channels:\tI1, Q1, I2, Q2\n"
        )
        
        # Write the report to the specified output file
        file_to_write = os.path.join(output_dir, output_file)
        with open(file_to_write, 'w') as file:
            file.write(report_content)
            
    def send_object_tracks_to_queue(self, detectionsAtTime: DetectionsAtTime):
        """
        Push the detections to the Queue
        """
        if self.radar_data_queue is not None:
            self.radar_data_queue.put(detectionsAtTime)
    