import numpy as np
import math
import scipy


def movmean2d(A, winRow, winCol, varargin):
    omitnan =  varargin=='omitnan' 
    if omitnan:
        validMask = ~np.isnan(A)
        A_zeroed  = A
        A_zeroed[~validMask] = 0 
    else:
        validMask = np.ones((len(A[:,0]),len(A[0,:])))
        A_zeroed  = A

    window = np.ones((winRow,winCol) )

    sumVals = scipy.signal.convolve2d(A_zeroed, window, mode='same')
    countVals = scipy.signal.convolve2d(validMask, window, 'same')
    #print(countVals)
    B = sumVals / countVals
    B[countVals==0] = float('nan');  # in case window falls completely outside
    return B
def  Clutter_rejection(rangeFFT, range_axis, time_vector, chnidx, MBW):
    # Compute number of bins and pulses
    MBW = MBW

    [nBins, nPulses] = [len(rangeFFT ), len(rangeFFT )]

    # Low‐range elimination width (bins)
    elim_bins_1 = math.ceil(1/ (3e8/(2*MBW)))
    elim_bins_2 = math.ceil(4/ (3e8/(2*MBW)))


    range_bins_1 = len(range_axis) - elim_bins_1 + np.linspace(1, len(range_axis), len(range_axis))[0:5]
    range_bins_2 = np.linspace(1, elim_bins_2, elim_bins_2)  

    

    
    elim1 = (1e-4/math.sqrt(2)) * (np.random.randn(len(range_bins_1) ) + 1j*np.random.randn(len(range_bins_1) ) )
    elim2 = (1e-4/math.sqrt(2)) * (np.random.randn(len(range_bins_2) ) + 1j*np.random.randn(len(range_bins_2) ) )
    rangeFFT[int(range_bins_1[0]-1):int(range_bins_1[-1] )] = elim1
    rangeFFT[int(range_bins_2[0]-1):int(range_bins_2[-1] )] = elim2

     
    
    return rangeFFT, range_axis, time_vector


def compensatePathloss_dB(P, ranges, n):
    scale_range_bins  = 2 

    if len(ranges) != len(P):
        print('Length of ranges vector must match number of rows in P.');
    
    # build compensation factor
    # this is an M×1 vector, raised to the nth power
 

    compensation_factor = np.power(ranges[0:scale_range_bins],-n)
    compensation_factor = np.concatenate( (compensation_factor,np.power(ranges[scale_range_bins:],n)) ) 
    compensation_factor = np.array( compensation_factor   ).transpose() 
    P_comp = P * compensation_factor
    #rangefft1/ np.max(abs(rangefft1) )
     
    return P_comp 