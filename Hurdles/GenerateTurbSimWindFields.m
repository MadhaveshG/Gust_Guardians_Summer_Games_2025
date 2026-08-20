% Generates wind fields for DLC 1.2 running TurbSim.
% Only for documentation on how RewsFiles were generated. 

%% Setup
clearvars;
close all;
clc;
addpath(genpath('..\WetiMatlabFunctions'))

% Parameters (can be adjusted, but will provide different results)
URef_vector         = 18;       % [m/s]         range of wind speeds (operation points) for 18 m/s Hurdles
n_Seed              = 6;        % [-]           number of stochastic turbulence field seeds

% Seed Matrix Definition
Seed_vector         = [1:n_Seed];
n_URef              = length(URef_vector);
Seed_matrix         = repmat(URef_vector',1,n_Seed)*100+repmat(Seed_vector,n_URef,1);

% Files (should not be changed)
TurbSimExeFile      = 'TurbSim_x64.exe';
TurbSimTemplateFile = 'TurbSimInputFileTemplateIEA15MW.inp';


%% Preprocessing: generate turbulent wind field

% Copy the adequate TurbSim version to the example folder 
copyfile(['..\TurbSim\',TurbSimExeFile],['TurbulentWind\',TurbSimExeFile])
    
% Generate all wind fields for different URef and RandSeed1
for i_URef    = 1:n_URef
    URef      = URef_vector(i_URef);
    parfor i_Seed = 1:n_Seed          
        Seed                = Seed_matrix(i_URef,i_Seed);
        WindFileName        = ['URef_',num2str(URef,'%02d'),'_Seed_',num2str(Seed,'%02d')];
        TurbSimInputFile  	= ['TurbulentWind\',WindFileName,'.ipt'];
        TurbSimResultFile  	= ['TurbulentWind\',WindFileName,'.wnd'];
        if ~exist(TurbSimResultFile,'file')
            copyfile([TurbSimTemplateFile],TurbSimInputFile)
            % Adjust the TurbSim input file
            ManipulateFastInputFile(TurbSimInputFile,'URef ',     num2str(URef,'%4.1d'));   % adjust URef
            ManipulateFastInputFile(TurbSimInputFile,'RandSeed1 ',num2str(Seed));        	% adjust seed
            % Generate wind field
            dos(['TurbulentWind\',TurbSimExeFile,' ',TurbSimInputFile]);
        end
    end
end

% Clean up
delete(['TurbulentWind\',TurbSimExeFile])

%% Calculate REWS
R                   = 120;                      % [m]  	    rotor radius to calculate REWS

figure
for i_URef    = 1:n_URef
    subplot(n_URef,1,i_URef);
    hold on;box on;grid on
    URef      = URef_vector(i_URef);
    for i_Seed = 1:n_Seed        
        Seed                        = Seed_matrix(i_URef,i_Seed);
        WindFileName                = ['URef_',num2str(URef,'%02d'),'_Seed_',num2str(Seed,'%02d')];
        TurbSimResultFile  	        = ['TurbulentWind\',WindFileName,'.wnd'];

        [REWS_WindField,Time_WindField]  	= CalculateREWSfromWindField(TurbSimResultFile,R,1);  

        % plot
        plot(Time_WindField,REWS_WindField)  

        RewsFile  	                = ['TurbulentWind\',WindFileName,'.csv'];

        % write file
        fid = fopen(RewsFile,'w+');
        fprintf(fid,'time,REWS\r\n');
        fprintf(fid,['%f,%f\r\n'],[Time_WindField REWS_WindField]');
        fclose(fid);

    end
end
