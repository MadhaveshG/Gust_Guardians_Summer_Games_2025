function [REWS,REWS_f,REWS_b]  = LDP_v3(time,isValid,beamID,lineOfSightWindSpeed,DT,LDP)
% Function to postprocess lidar data to get the rotor-effective wind speed
% (REWS) equal to the LDP_v1/FFP_v1 without the need of compiling a DLL. 
% Code is intented to be as close as possble to the Fortran Code.
% v3: similar to v1, but includes ignoring of signals with invalid data.

% internal variables
PreviousBeamID  = -1;       % force WFR on first call
REWS_i          = 0;        % Dummy
n_t             = length(time);

% allocation
REWS            = NaN(n_t,1);
REWS_f          = NaN(n_t,1);
REWS_b          = NaN(n_t,1);

% loop over time
for i_t = 1:n_t

    % if there is a new measurement perform wind field reconstruction    
    if beamID(i_t) ~= PreviousBeamID  
        v_los_i         = lineOfSightWindSpeed(i_t);
        isValid_i       = isValid(i_t);
        REWS_i          = WindFieldReconstruction(v_los_i,isValid_i,LDP.NumberOfBeams,LDP.AngleToCenterline);
        PreviousBeamID  = beamID(i_t); % update beamID
    end

    % Low pass filter the REWS
	if LDP.FlagLPF
		REWS_f_i     	= LPFilter(REWS_i,DT,LDP.omega_cutoff);
    else
		REWS_f_i      	= REWS_i;
    end

    % Get buffered and filtered REWS from buffer
    REWS_b_i            = Buffer(REWS_f_i,DT,LDP.T_buffer);

    % Store in structure
    REWS(i_t)           = REWS_i;
    REWS_f(i_t)         = REWS_f_i;
    REWS_b(i_t)         = REWS_b_i;
end

end


function REWS = WindFieldReconstruction(v_los,isValid,NumberOfBeams,AngleToCenterline)
% matlab version of the subroutine WindFieldReconstruction in LDP_v1_Subs.f90
% extended to deal with invalid data AND induction zone correction.

% init u_est_Buffer
persistent u_est_Buffer;
if isempty(u_est_Buffer)      
    u_est_Buffer = NaN(NumberOfBeams,1);  
end 

% --- CALIBRATION FACTOR ---

% Negative value indicates "Speed Up" (Lidar measures faster than free stream).
InductionFactor = -0.08; 

% Estimate u component assuming perfect alignment
if isValid
    % 1. De-project Line-of-Sight to Horizontal
    v_horiz_raw = v_los/cosd(AngleToCenterline);
    
    % 2. Apply Induction/Shear Calibration
    u_est       = v_horiz_raw / (1 - InductionFactor);
else
    % Blade Blockage detected
    u_est       = NaN;
end

% Update Buffer for estimated u component
u_est_Buffer    = [u_est;u_est_Buffer(1:NumberOfBeams-1)];

% Calculate REWS from mean over all estimated u components
% 'omitnan' ensures we calculate average only on valid beams
REWS  	        = mean(u_est_Buffer,'omitnan');

end

function OutputSignal = LPFilter(InputSignal,DT,CornerFreq)
% matlab version of the function LPFilter in FFP_v1_Subs.f90

% Initialization
persistent OutputSignalLast InputSignalLast;
if isempty(OutputSignalLast)      
    OutputSignalLast    = InputSignal;  
    InputSignalLast     = InputSignal;
end 

% Define coefficients 
a1          = 2 + CornerFreq*DT;
a0          = CornerFreq*DT - 2;
b1          = CornerFreq*DT;
b0          = CornerFreq*DT;

% Filter
OutputSignal = 1.0/a1 * (-a0*OutputSignalLast + b1*InputSignal + b0*InputSignalLast);

% Save signals for next time step
InputSignalLast     = InputSignal;
OutputSignalLast    = OutputSignal;
end

function REWS_b = Buffer(REWS,DT,T_buffer)

% init REWS_f_Buffer
nBuffer = 2000; % Size of REWS_f_buffer, 20 seconds at 100 Hz  [-] 
persistent REWS_f_Buffer;
if isempty(REWS_f_Buffer)      
    REWS_f_Buffer = ones(nBuffer,1)*REWS;   
end 

% Update Buffer for estimated u component
REWS_f_Buffer    = [REWS;REWS_f_Buffer(1:nBuffer-1)];

% Index for entry at T_buffer, minimum 1, maximum nBuffer
Idx     = min( max(floor(T_buffer/DT),1) , nBuffer);
		
% Get buffered and filtered REWS from buffer
REWS_b  = REWS_f_Buffer(Idx);

end