# Script to evaluate the " 18 m/s Hurdles" for the LAC Summer Games 2025.
# Task:
# Get the best possible wind preview (2s) by improving the baseline lidar
# data processing (LDP_v3)!
# Results 4BeamPulsed:
# v3: Cost for Summer Games 2025 ("18 m/s hurdles"):  0.515998 m/s
# Results CircularCW:
# v3: Cost for Summer Games 2025 ("18 m/s hurdles"):  0.463842 m/s

# Setup
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os
from LDP_v3 import LDP_v3

# Select LDP function
MyLDPfunction = LDP_v3          # [LDP_v3/other LDP functions...]

# Select LidarType
LidarType = '4BeamPulsed'       # [4BeamPulsed/CircularCW]

# Seeds (can be adjusted, but will provide different results)
nSeed = 6                                       # [-]	    number of stochastic turbulence field samples
Seed_vec = np.arange(1, nSeed+1) + 18 * 100     # [-]  	    vector of seeds

# Parameters postprocessing (can be adjusted, but will provide different results)
t_start = 60                                    # [s] 	    ignore data before for STD and spectra
TMax = 660                                      # [s]       total run time
DT = 0.01                                       # [s]       time step
time = np.arange(0, TMax + DT, DT)              # [s]       time vector

# Parameter for Cost (Summer Games 2025)
tau = 2                                         # [s]       time to overcome pitch actuator, from Example 1: tau = T_Taylor - T_buffer, since there T_filter = T_scan = 0

if LidarType == '4BeamPulsed':
    # configuration from LDP_v1_4BeamPulsed.IN and FFP_v1_4BeamPulsed.IN
    LDP = {
        'NumberOfBeams': 4,                     # [-]       Number of beams measuring at different directions
        'AngleToCenterline': 19.176,            # [deg]     Angle around centerline
        'IndexGate': 6,                         # [-]       IndexGate
        'FlagLPF': 1,                           # [0/1]     Enable low-pass filter (flag)
        'omega_cutoff': 0.13,                   # [rad/s]   Corner frequency (-3dB) of the low-pass filter
        'T_buffer': 0.2                         # [s]       Buffer time for filtered REWS signal
    }
elif LidarType == 'CircularCW':
    # configuration from LDP_v1_CircularCW.IN and FFP_v1_CircularCW.IN
    LDP = {
        'NumberOfBeams': 50,                    # [-]       Number of beams measuring at different directions
        'AngleToCenterline': 15,                # [deg]     Angle around centerline
        'IndexGate': 1,                         # [-]       IndexGate
        'FlagLPF': 1,                           # [0/1]     Enable low-pass filter (flag)
        'omega_cutoff': 0.20,                   # [rad/s]   Corner frequency (-3dB) of the low-pass filter
        'T_buffer': 4.2                         # [s]       Buffer time for filtered REWS signal
    }
else:
    raise ValueError('Unknown LidarType!')

# Files (should not be be changed)
SimulationFolderLAC = 'solis_lidar_data'

# Allocation
MAE = np.full(nSeed, np.nan)

for iSeed, Seed in enumerate(Seed_vec):

    # Filenames
    WindFileName = f"URef_18_Seed_{Seed:02d}"
    SolisResultFile = os.path.join(SimulationFolderLAC,
                                   f"{WindFileName}_lidar_data_{LidarType}.csv")

    # Load data
    SolisData = pd.read_csv(SolisResultFile)

    beamID_series = pd.Series(SolisData['beamID'].values, index=SolisData['time'])
    beamID = beamID_series.reindex(time, method='pad').bfill().to_numpy()

    isValid_col = f"isValid{LDP['IndexGate']}"
    isValid_series = pd.Series(SolisData[isValid_col].values, index=SolisData['time'])
    isValid = isValid_series.reindex(time, method='pad').fillna(0).to_numpy()

    lineOfSight_col = f"lineOfSightWindSpeed{LDP['IndexGate']}"
    los_series = pd.Series(SolisData[lineOfSight_col].values, index=SolisData['time'])
    lineOfSightWindSpeed = los_series.reindex(time, method='pad').bfill().to_numpy()

    # Get REWS from the wind field
    RewsFile = f"TurbulentWind/{WindFileName}.csv"
    RewsData = pd.read_csv(RewsFile)

    ext_time = np.concatenate([RewsData['time'], RewsData['time'] + 600])
    ext_rews = np.concatenate([RewsData['REWS'], RewsData['REWS']])

    REWS_WindField = np.interp(time, ext_time, ext_rews)                    # REWS is circular
    REWS_WindField_shifted = np.interp(time + tau, ext_time, ext_rews)

    # Calculate REWS from the lidar data
    REWS, REWS_f, REWS_b = MyLDPfunction(time, isValid, beamID, lineOfSightWindSpeed, DT, LDP)

    # Calculate mean absolute error
    Error = REWS_WindField_shifted - REWS_b                                 # error is  REWS from wind field shifted minus REWS from lidar filtered and buffered.

    idx_valid = time >= t_start
    Error_detrended = Error[idx_valid] - np.mean(Error[idx_valid])          # only consider error after t_start
    MAE[iSeed] = np.mean(np.abs(Error_detrended))

    # Plot REWS for absolute error
    fig = plt.figure(figsize=(10, 8))
    plt.subplot(311)
    plt.plot(time, REWS_WindField, label='wind field')
    plt.plot(time, REWS, label='lidar estimate')
    plt.grid()
    plt.legend()
    plt.ylabel('REWS [m/s]')

    plt.subplot(312)
    plt.plot(time, REWS_WindField_shifted, label='wind field shifted')
    plt.plot(time, REWS_b, label='lidar estimate filtered and buffered')
    plt.grid()
    plt.legend()
    plt.ylabel('REWS [m/s]')

    plt.subplot(313)
    plt.plot(time, Error, label='error')
    plt.grid()
    plt.legend()
    plt.ylabel('error [m/s]')
    plt.xlabel('time [s]')

    plt.suptitle(f'REWS seed {Seed}')
    plt.tight_layout()

#plt.show()

# Compute cost for Summer Games 2025
Cost = np.mean(MAE)
print(f'Cost for Summer Games 2025 (\"18 m/s hurdles\"): {Cost:.6f}')