function writeBULLETinitstates(MR_B,Mo_B,Bv_MB,Bw_MB,MR_C,Mo_C,fnname)
%% Writing initial states to csv file
%Function to write the release states (initial states of the simulation) to
%csv files to be used as input for the PyBullet simulation
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
    MH_B  = [MR_B(:,:,is),Mo_B(:,is);zeros(1,3),1];                      %Pose of the box w.r.t world frame
    BV_MB = [Bv_MB(:,is);Bw_MB(:,is)];                                   %Left-trivialized velocity
    BMV_MB = [MH_B(1:3,1:3) zeros(3,3); zeros(3,3) MH_B(1:3,1:3)]*BV_MB; %Hybrid velocity
    MH_C  = [MR_C(:,:,is),Mo_C(:,is);zeros(1,3),1];                      %Pose of the conveyor w.r.t world frame

    quat = rotm2quat(MH_B(1:3,1:3));
    quatP = rotm2quat(MH_C(1:3,1:3));

    startOrientation = [quat(2:4) quat(1)];
    startPos = MH_B(1:3,4)';

    linearVelocity = BMV_MB(1:3)';
    angularVelocity = BMV_MB(4:6)';

    planePos = MH_C(1:3,4)';
    planeOrientation = [quatP(2:4) quatP(1)];

    %Write states to csv file
    states_csv(is,:) = num2cell([startPos startOrientation linearVelocity angularVelocity planePos planeOrientation]);
end
%Write them to the csv files
if ~isfolder(fnname)
    mkdir(fnname);
end
writecell(states_csv,append(fnname,'/test_states.csv'));