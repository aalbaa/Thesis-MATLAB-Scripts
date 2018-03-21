% Amro AL Baali
% March 17, 2018
%
% This is to test the DSS LMI (there exists a Q s.t the LMI holds)
%
% PART I: The first part consists of testing the LMI on one stable system

% PART II: The second part consists of testing the LMI on a system with its
% controller using the CL equations developed by hand

% PART III: Solve LMI to FIND a controller

clear all;
close all;
home;


% part = input('part ... ?\n');
part = 3;
LMI = 2; % 1 or 2?
solver = 2; % 1: mosek. 2: sdpt3. 3: sedumi. anything else: no specific solve
SS = 1;

% 
if part == 1
    PART1;
elseif part == 2
    PART2;
elseif part == 3
    PART3;
elseif part ==4
    PART4;
end