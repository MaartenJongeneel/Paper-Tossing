function vertices = BoxVertices(l,w,h)
%This function computes the positions of the vertices of the box, expressed
%in terms of the box frame B at the COM. 
% INPUTS:    l         : 1x1 double, length of the box
%            w         : 1x1 double, width of the box
%            h         : 1x1 double, height of the box
%
% OUTPUTS:   vertices  : struct containing vertices
%% vertices of the box expressed in body fixed frame B (frame B at COM)
vertices.Bp_1 = [-l/2; -w/2; -h/2];      vertices.Bp_5 = [-l/2; -w/2;  h/2];
vertices.Bp_2 = [-l/2;  w/2; -h/2];      vertices.Bp_6 = [-l/2;  w/2;  h/2];
vertices.Bp_3 = [ l/2;  w/2; -h/2];      vertices.Bp_7 = [ l/2;  w/2;  h/2];
vertices.Bp_4 = [ l/2; -w/2; -h/2];      vertices.Bp_8 = [ l/2; -w/2;  h/2];