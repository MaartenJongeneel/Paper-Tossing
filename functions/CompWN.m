function WN = CompWN(AR_B,Az_C,Bp_1,Bp_2,Bp_3,Bp_4,Bp_5,Bp_6,Bp_7,Bp_8)
% Compute the matrix containing the normal force directions.
%
% INPUTS:    AR_B    : Rotation matrix expressing frame B w.r.t. frame A
%            Az_C    : Unit vector in z-direction of frame C expressed in A
%            Bp_i    : Position vector expressing vertex i w.r.t. frame B
%
% OUTPUTS:   WN      : matrix containing the normal force directions
%% Script
%Compute the matrices containing the normal force directions 
wNC = [Az_C'*AR_B -Az_C'*AR_B*hat(Bp_1)]';
wND = [Az_C'*AR_B -Az_C'*AR_B*hat(Bp_2)]';
wNE = [Az_C'*AR_B -Az_C'*AR_B*hat(Bp_3)]';
wNF = [Az_C'*AR_B -Az_C'*AR_B*hat(Bp_4)]';
wNG = [Az_C'*AR_B -Az_C'*AR_B*hat(Bp_5)]';
wNH = [Az_C'*AR_B -Az_C'*AR_B*hat(Bp_6)]';
wNI = [Az_C'*AR_B -Az_C'*AR_B*hat(Bp_7)]';
wNJ = [Az_C'*AR_B -Az_C'*AR_B*hat(Bp_8)]';

WN = [wNC, wND, wNE, wNF, wNG, wNH, wNI, wNJ];