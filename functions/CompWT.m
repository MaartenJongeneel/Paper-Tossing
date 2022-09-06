function WT = CompWT(AR_B,Ax_C,Ay_C,Bp_1,Bp_2,Bp_3,Bp_4,Bp_5,Bp_6,Bp_7,Bp_8)
% Compute the matrix containing the normal force directions.
%
% INPUTS:    AR_B    : Rotation matrix expressing frame B w.r.t. frame A
%            Ax_C    : Unit vector in x-direction of frame C expressed in A
%            Ay_C    : Unit vector in y-direction of frame C expressed in A
%            Bp_i    : Position vector expressing vertex i w.r.t. frame B
%
% OUTPUTS:   WN      : matrix containing the normal force directions
%% Script
%Compute the matrices containing the tangential force directions 

wTC = {[Ax_C'*[AR_B -AR_B*hat(Bp_1)];
        Ay_C'*[AR_B -AR_B*hat(Bp_1)]]'};
wTD = {[Ax_C'*[AR_B -AR_B*hat(Bp_2)];
        Ay_C'*[AR_B -AR_B*hat(Bp_2)]]'};
wTE = {[Ax_C'*[AR_B -AR_B*hat(Bp_3)];
        Ay_C'*[AR_B -AR_B*hat(Bp_3)]]'};
wTF = {[Ax_C'*[AR_B -AR_B*hat(Bp_4)];
        Ay_C'*[AR_B -AR_B*hat(Bp_4)]]'};
wTG = {[Ax_C'*[AR_B -AR_B*hat(Bp_5)];
        Ay_C'*[AR_B -AR_B*hat(Bp_5)]]'};
wTH = {[Ax_C'*[AR_B -AR_B*hat(Bp_6)];
        Ay_C'*[AR_B -AR_B*hat(Bp_6)]]'};
wTI = {[Ax_C'*[AR_B -AR_B*hat(Bp_7)];
        Ay_C'*[AR_B -AR_B*hat(Bp_7)]]'};
wTJ = {[Ax_C'*[AR_B -AR_B*hat(Bp_8)];
        Ay_C'*[AR_B -AR_B*hat(Bp_8)]]'};

WT = [wTC, wTD, wTE, wTF, wTG, wTH, wTI, wTJ];