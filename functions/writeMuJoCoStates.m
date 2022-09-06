function writeMuJoCoStates(MR_B,Mo_B,BV_MB)
%% Writing initial states to csv file
%Function to write the release states (initial states of the simulation) to
%csv files to be used as input for the Algoryx simulation
%
% INPUTS:    MR_B    : 3-by-3-by-N matrix containing N rotation matrices
%            Mo_B    : 3-by-1-by-N matrix containing N position vectors
%            BV_MB   : 3-by-1-by-N matrix, linear + angular velocity of the box at release
%
% OUTPUTS:   -       : Writes the box states and conveyor states to a csv
%                      file
%% Script
Nstates = length(MR_B(1,1,:));

for is = 1:Nstates
    %Header of the file
    states_csv(1,:) = num2cell(["qpos_x", "qpos_y" "qpos_z", "Rx1", "Rx2","Rx3","Ry1","Ry2","Ry3","linvel1","linvel2","linvel3","angvel1","angvel2","angvel3"]);
    %Write the initial state
    t = [Mo_B(1,is), Mo_B(2,is), Mo_B(3,is), MR_B(1,1,is), MR_B(2,1,is), MR_B(3,1,is), MR_B(1,2,is), MR_B(2,2,is), MR_B(3,2,is), ...
         BV_MB(1,is), BV_MB(2,is), BV_MB(3,is), BV_MB(4,is), BV_MB(5,is), BV_MB(6,is)];
    states_csv(is+1,:) = num2cell(t);
end
%Write them to the csv files
if ~isfolder("paramID/MuJoCo_init_states")
    mkdir("paramID/MuJoCo_init_states");
end
writecell(states_csv,'paramID/MuJoCo_init_states/test_states.csv');
end