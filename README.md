# Gust_Guardians_Summer_Games_2025
# IEA Wind Task 52: The 18 m/s Hurdles

This repository contains the robust wind field reconstruction framework developed by the Gust Guardians for the IEA Wind Task 52 Summer Games 2025. The primary objective is to predict turbulent rotor effective wind speed (REWS) exactly 2.0 seconds in advance using data from a Lidar system positioned 200 meters upstream. This advanced prediction allows turbines to optimize pitch settings before a gust arrives, reducing structural fatigue and maximizing power output.

## Core Methodology

To overcome physical measurement limitations such as blade blockage and induction zones, this project moves beyond simple data filtering by implementing several advanced reconstruction techniques:
*   A Robust Sine-Wave Fit is utilized to reconstruct the azimuthal line-of-sight pattern via $V_{los}(\theta)\approx A~cos(\theta)+B~sin(\theta)+C$.
*   The algorithm successfully estimates the full wind field even when rotating turbine blades occlude up to 20% of the Lidar scan.
*   A calibrated -15% induction adjustment corrects for the upstream free-stream velocity being systematically higher than the actual rotor-plane velocity.
*   A height bias correction compensates for the vertical offset of the Lidar, which is mounted 7.4 meters above the turbine hub.
*   A first-order IIR filter with a cutoff frequency of 0.28 rad/s and a 4.95-second time-delay buffer syncs the processing delay with physical wind convection.

## Performance Results

By transitioning from simple beam-averaging to a comprehensive mathematical reconstruction, the final model achieved an RMSE cost of 0.447608 m/s. This performance successfully eliminated earlier systematic biases and aligns closely with the physical limits of prediction for the benchmark dataset.


## Project Evaluation and Related Context

For broader context on the challenge and its outcomes, you can refer to the presentation titled "Evaluation of the Lidar-Assisted Control Summer Games 2025 and Update on LAC Open Source Tools" published on Zenodo https://zenodo.org/records/20625931?preview_file=Schlipf2026b+-+Evaluation+of+the+Lidar-Assisted+Control+Summer+Games+2025+and+Update+on+LAC+Open+Source+Tools.pdf.

This presentation, published in June 2026 by authors from Flensburg University of Applied Sciences, Shanghai Jiao Tong University, and sowento, summarizes the results and lessons learned from the 2025 edition of the games, building on insights from the 2024 edition and presenting updates on LAC Open Source Tools.
