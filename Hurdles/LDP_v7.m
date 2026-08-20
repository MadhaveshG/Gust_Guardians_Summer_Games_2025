function [REWS, REWS_f, REWS_b] = LDP_v7(time, isValid, beamID, lineOfSightWindSpeed, DT, LDP)
% LDP_v7: The "Aligned" Hybrid
% Fixes: 
% 1. Azimuth Direction (Matches YAML Clockwise order)
% 2. Height Bias (Corrects for Lidar being 7.4m above Hub)

    % --- 1. SETUP & STATE ---
    n_t = length(time);
    n_beams = LDP.NumberOfBeams;
    
    % Outputs
    REWS   = NaN(n_t,1);
    REWS_f = NaN(n_t,1);
    REWS_b = NaN(n_t,1);
    
    % Memory (State)
    beam_state = NaN(n_beams, 1); 
    filter_state = NaN; 
    
    % Buffer (Optimized from v6 results)
    % If v6 optimal was 4.60s, we stick to that.
    BufferSize = round(LDP.T_buffer / DT);
    buffer_memory = repmat(18.0, BufferSize, 1); 
    
    % --- 2. PRE-CALCULATION (FIXED) ---
    
    % FIX 1: MATCH YAML AZIMUTHS (CLOCKWISE)
    % YAML: [360.0, 352.8, ... 7.2]
    % This generates the sequence 360 down to 360/50
    azimuths_deg = linspace(360, 360 - 360*((n_beams-1)/n_beams), n_beams)';
    
    azimuths_rad = deg2rad(azimuths_deg);
    cone_rad     = deg2rad(LDP.AngleToCenterline);
    
    Cos_Azi = cos(azimuths_rad);
    Sin_Azi = sin(azimuths_rad);
    
    % FIX 2: SHEAR CORRECTION BIAS
    % Lidar is 7.4m above Hub. Assuming alpha=0.12 shear exponent.
    % Bias ~ (157.4/150)^0.12 ~ 1.005
    % We must scale DOWN by 0.995 to match Hub height.
    HeightBiasCorrection = 0.995; 
    
    % Physics (Locked)
    InductionFactor = -0.15; 
    alpha = exp(-DT * 0.28); 

    % --- 3. MAIN LOOP ---
    for i_t = 1:n_t
        
        % A. INPUT PROCESSING
        c_beam  = beamID(i_t);
        c_val   = lineOfSightWindSpeed(i_t);
        c_valid = isValid(i_t);
        
        if c_valid && c_val ~= 0 && c_beam >= 1 && c_beam <= n_beams
            beam_state(c_beam) = c_val;
        end
        
        % B. RECONSTRUCTION (SINE FIT)
        valid_indices = find(~isnan(beam_state));
        
        if length(valid_indices) < 5
            REWS_i = 18.0; 
        else
            y = beam_state(valid_indices);
            H = [Cos_Azi(valid_indices), Sin_Azi(valid_indices), ones(length(valid_indices),1)];
            x = H \ y; 
            
            % 1. Geometric De-projection
            Raw_Horizontal = x(3) / cos(cone_rad);
            
            % 2. Apply Height Bias Correction (New!)
            Raw_HubHeight = Raw_Horizontal * HeightBiasCorrection;
            
            % 3. Apply Induction Correction
            REWS_i = Raw_HubHeight / (1 - InductionFactor);
        end
        
        % C. FILTERING (IIR)
        if isnan(filter_state), filter_state = REWS_i; end
        REWS_f_i = (1 - alpha)*REWS_i + alpha*filter_state;
        filter_state = REWS_f_i; 
        
        % D. BUFFERING
        buffer_memory = [buffer_memory(2:end); REWS_f_i];
        REWS_b_i = buffer_memory(1);
        
        % Store
        REWS(i_t)   = REWS_i;
        REWS_f(i_t) = REWS_f_i;
        REWS_b(i_t) = REWS_b_i;
    end
end