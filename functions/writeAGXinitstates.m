function writeAGXinitstates(MR_B,Mo_B,Bv_MB,Bw_MB,MR_C,Mo_C,fnname)
%% Writing initial states to csv file
%Function to write the release states (initial states of the simulation) to
%csv files to be used as input for the Algoryx simulation
%
% INPUTS:    MR_B    : 3-by-3-by-N matrix containing N rotation matrices
%            Mo_B    : 3-by-1-by-N matrix containing N position vectors
%            Bv_MB   : 3-by-1-by-N matrix, linear velocity of the box at release
%            Bw_MB   : 3-by-1-by-N matrix, angular velocity of the box at release
%            MR_C    : 3-by-3-by-N matrix, orientation of the contact frame
%            Mo_C    : 3-by-1-by-N matrix, position of the contact frame
%
% OUTPUTS:   -       : Writes the box states and conveyor states to a csv
%                      file
%% Script
Nstates = length(MR_B(1,1,:));

for is = 1:Nstates
    %Select the object pose and twist, and the conveyor pose
    H  = [MR_B(:,:,is),Mo_B(:,is);zeros(1,3),1]; 
    BV_MB = [Bv_MB(:,is);Bw_MB(:,is)];       
    MH_C  = [MR_C(:,:,is),Mo_C(:,is);zeros(1,3),1];

    %For the selected impact, write the initial pose and velocity of the box to a csv file.
    %Header of the file
    states_csv(1,:) = num2cell(["t11","t12","t13","t14","t21","t22","t23","t24","t31","t32","t33","t34","t41","t42","t43","t44","vx","vy","vz","avx","avy","avz"]);
    %Write the initial state
    t = [H(1,1) H(2,1) H(3,1) H(4,1) H(1,2) H(2,2) H(3,2) H(4,2) H(1,3) H(2,3) H(3,3) H(4,3) H(1,4) H(2,4) H(3,4) H(4,4)];
    v = [H(1:3,1:3) zeros(3,3); zeros(3,3) H(1:3,1:3)]*BV_MB;
    t = [t v'];
    states_csv(is+1,:) = num2cell(t);

    %Now we do the same thing for the conveyor
    states_conv_csv(1,:) = num2cell(["t11","t12","t13","t14","t21","t22","t23","t24","t31","t32","t33","t34","t41","t42","t43","t44"]);
    %Write the initial state
    H = MH_C;
    H(3,4) = H(3,4)-0.05; %Compensate for AGX conv frame position
    t = [H(1,1) H(2,1) H(3,1) H(4,1) H(1,2) H(2,2) H(3,2) H(4,2) H(1,3) H(2,3) H(3,3) H(4,3) H(1,4) H(2,4) H(3,4) H(4,4)];
    states_conv_csv(is+1,:) = num2cell(t);
end
%Write them to the csv files
if ~isfolder(fnname)
    mkdir(fnname);
end
writecell(states_csv,append(fnname,'/test_states.csv'));
writecell(states_conv_csv,append(fnname,'/test_conveyor_states.csv'));