function y=proxCN(x)
% Proximal point formulation for CN. See thesis for reference.
%
% prox_CN(x) = 0 for x=< 0
%            = x for x > 0
%
% INPUTS:    x  : Vector in R^n
%
% OUTPUTS:   y  : Vector in R^n 
%% Script
y=max(x,0);