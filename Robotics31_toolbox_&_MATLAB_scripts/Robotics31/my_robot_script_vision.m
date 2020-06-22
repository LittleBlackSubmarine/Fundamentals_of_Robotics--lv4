% The following parameters must be defined before running function rvsim:
%
%     method     - image processing method: 'corners' or 'edges'. Default is
%                  'edges'.
%     xmin, xmax - working range, i.e. minimum and maximum x-coordinate 
%                  which can be reached by the tool 
%     zT0        - height of the working plane relative to the base
%                  c. s. of the robot

clear;
close all;
clc;


% Setting parameters explained at beginning of script
zT0 = -14.5; 
xmin = 50; 
xmax =155.1; 
method ='edges';

my_rvsim;