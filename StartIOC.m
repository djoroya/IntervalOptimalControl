
%% SETTIGS PATH
main_file = 'StartIOC.m';
path_file = which(main_file);
path_file = replace(path_file,main_file,'');

path_depen = fullfile(path_file,'src','dependences');


%% clean 

try rmdir(path_depen,'s')
catch  err
end

%% CREATE FOLDERS 

mkdir(path_depen)

%%
%% CasADi


try
    casadi.SX.sym('s');
catch
    disp('CasADi will be download ...')
    casADi_folder = fullfile(path_file,'src','dependences','CasADi');
    if ~exist(casADi_folder,'dir')
        mkdir(casADi_folder)
    end
    if ismac
        untar('https://github.com/casadi/casadi/releases/download/3.5.1/casadi-osx-matlabR2015a-v3.5.1.tar.gz',casADi_folder)
    elseif ispc
        unzip('https://github.com/casadi/casadi/releases/download/3.5.1/casadi-windows-matlabR2016a-v3.5.1.zip',casADi_folder)
    elseif isunix
        untar('https://github.com/casadi/casadi/releases/download/3.5.1/casadi-linux-matlabR2014b-v3.5.1.tar.gz',casADi_folder)
    end
    addpath(genpath(path_file))

end

%% ADD FOLDERS
addpath(genpath(path_file))