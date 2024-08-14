function [AH_B, BV_AB, PN, PT] = BoxSimulatorLCPNEW(x,c,box,surface)
%% Box-simulator-FixedPoint:
%This script uses an LCP formulation
%for solving the nonlinear algebraic equations for contact impact and
%friction. The box has a body fixed frame B at its COM. Furthermore, the
%contact surface has its position and orientation defined by frame C, and
%we express all positions and velocities in terms of frame A, located at
%the camera coordinate frame of a camera, also plotted in the figure.
%
% INPUTS:    x.releasePosition   : 3x1 double, position of the box at release           [m]
%            x.releaseOrientation: 3x3 double, orientation of the box at release        [deg]
%            x.releaseLinVel     : 3x1 double, linear velocity of the box at release    [m/s]
%            x.releaseAngVel     : 3x1 double, angular velocity of the box at release   [rad/s]
%            c.eN                : 1x1 double, normal coefficient of restitution        [-]
%            c.eT                : 1x1 double, tangential coefficient of restitution    [-]
%            c.mu                : 1x1 double, coefficient of friction                  [-]
%            c.dt                : 1x1 double, Timestep at which the simulator runs     [1/s]
%            c.endtime           : 1x1 double, Simulation time you want to run          [s]
%            box                 : struct, with fields of box properties as
%                                  box.B_M_B   : 6x6 double intertia tensor of the box  []  
%                                  box.mass    : 1x1 double mass of the box             [kg]
%                                  box.vertices: 3xN double position of the contact     [m]
%                                                points w.r.t the body-fixed frame
%            surface             : Nx1 cell array, with each element a struct containing
%                                  dim      : 1x2 double, dimensions of the surface
%                                  speed    : 3x1 double, speed of the surface
%                                  transform: 4x4 transformation matrix, epxression the 
%                                             position of the surface w.r.t. the world frame
%
% OUTPUTS:   AH_B                : 4x4xN double, transformation matrices expressing the 
%                                  pose of the box w.r.t. world frame over time
%            BV_AB               : 6x1xN double, left trivialized velocity of the box over time
%            FN                  : Normal force acting on the box over time
%            FT                  : Tangential force acting on the box over time

%% Constants and settings
g     = 9.81;                                 %Gravitational acceleration              [m/s^2]
PNfull = zeros(length(box.vertices),1);       %Initial guess for momenta PN            [N*s]
PTfull(1:length(box.vertices),1) = {[0;0]};   %initial guess for momenta PT            [N*s]
N = c.endtime/c.dt;                           %Run to this frame                       [-]

%% Retrieve info of box struct
M = box.B_M_B;
                           
%% Wrench acting on body B: expressed in B with orientation of A:
BA_fo  = [0; 0; -box.mass*g;];
BA_Tau = [0; 0; 0];
BA_f   = [BA_fo; BA_Tau];

%% Initial pose and velocity
%Initial pose of the box: Transformation matrix B w.r.t. frame A
AR_B    = x.releaseOrientation;%Initial orientation
Ao_B    = x.releasePosition;   %Initial position
AH_B(:,:,1) = [AR_B, Ao_B; zeros(1,3), 1]; %Homogeneous transformation matrix

%Initial left trivialized velocity of frame B w.r.t. frame A
Bv_AB = x.releaseLinVel;                      %Linear velocity at t0 of B wrt frame A  [m/s]
Bomg_AB = x.releaseAngVel;                    %Angular velocity at t0 of B wrt frame A [m/s]
BV_AB(:,1) = [Bv_AB; Bomg_AB];

%Discretizing the friction cone in dimd parts
dimd = c.dimd;
D = [];
angles = linspace(0,360,dimd+1);
for ss = 1:length(angles)-1
    theta = angles(ss);
    D = [D; [cos(deg2rad(theta)) sin(deg2rad(theta)) 0]];
end

%Construct the matrix E (diagonal with columns of ones, each of length dimd)
E = zeros(dimd*length(box.vertices),length(box.vertices));
ci = 1; %column index
ri = 1; %row index
for cp = 1:length(box.vertices)
    E(ri:ri+dimd-1,ci) = 1;
    ci = ci +1;
    ri = ri+dimd;
end

%% Dynamics
for t = 1:N %For each time step
    %Rewrite the body velocity as a 4x4 matrix in se(3)
    BV_ABs = hat(BV_AB(:,t));
    
    %Kinematics: Compute the configuration at the mid-time (tM)
    AH_Bm = AH_B(:,:,t)*expm(0.5*c.dt*BV_ABs);
    AR_Bm = AH_Bm(1:3,1:3);
    Ao_Bm = AH_Bm(1:3,4);
    AR_B  = AH_B(1:3,1:3,t);
    
    %Compute the wrench at tM
    B_fM = ([AR_Bm zeros(3); zeros(3) AR_Bm])'*BA_f;
    
    %And compute the gap-functions at tM
%     gN = (Az_C'*(Ao_Bm + AR_Bm*box.vertices-Ao_C));
    for jj = 1:length(surface)
        gN(:,jj) = (surface{jj}.transform(1:3,3)'*(Ao_Bm + AR_Bm*box.vertices-surface{jj}.transform(1:3,4)));
    end 
       
    %Obtain the linear and angular velocity at tA
    vA = BV_AB(:,t);
    
    %If a gap function has been closed, contact has been madeAo_B(:,)ii)
    IN = find(gN<=0);
    if  IN > 0
        nn = length(IN);

        %Construct the diagonal friction matrix
        Mu = diag(repmat(c.mu,1,nn));

        %Compute the matrix containing the normal and tangential force directions at tM
        [WNA,~] = CompW(AR_B,surface{jj}.transform(1:3,1:3),box.vertices(:,IN));
        [WNM,~] = CompW(AR_Bm,surface{jj}.transform(1:3,1:3),box.vertices(:,IN));
        
        WTA = []; WTM = [];
        for ic =1:length(IN)
            WTM = [WTM [D*[AR_Bm -AR_Bm*hat(box.vertices(:,IN(ic)))]]'];
            WTA = [WTA [D*[AR_B -AR_B*hat(box.vertices(:,IN(ic)))]]'];
        end

        term1 = [hat(vA(4:6)), zeros(3); hat(vA(1:3)), hat(vA(4:6))]*M*vA*c.dt - B_fM*c.dt; 

        % --------- OPTION C -----------%
        %See Claude's thesis, page 128.
        C =[0*diag(ones(1,nn*dimd)), zeros(nn*dimd,nn),  E(1:nn*dimd,1:nn);
            zeros(nn,nn*dimd),      c.dt*diag(ones(1,nn)),       zeros(nn,nn);
            -E(1:nn*dimd,1:nn)',     Mu,                0*diag(ones(1,nn))];

        G =  [WTM'; WNM'; zeros(nn,6)];

        %In the vector below, the restitution parameters appear (not sure this is correct)
        v = [-c.eT*WTA'*vA; -c.eN*WNA'*vA; zeros(nn,1)];
        u = M*vA-term1;

        A = C + G/M*G';
        b = G/M*u-v;
       
        %Solve the LCP formulation
%         [y,x] = LCP(A,b);
        [x,y] = lemke_ferris(A,b);

        PT = x(1:dimd*nn);
        PN = x(dimd*nn+1:dimd*nn+nn);
%         beta = x(end-nn+1:end);

        vE = vA + M\(B_fM*c.dt - [hat(vA(4:6)), zeros(3); hat(vA(1:3)), hat(vA(4:6))]*M*vA*c.dt + WNM*PN + WTM*PT);        
        BV_AB(:,t+1) = vE;        
    else
        %Update the velocity to the next time step
        vE = M\(B_fM*c.dt - [hat(vA(4:6)), zeros(3); hat(vA(1:3)), hat(vA(4:6))]*M*vA*c.dt) + vA;
        BV_AB(:,t+1) = vE;
    end
    
    %Complete the time step
    AH_B(:,:,t+1)  = AH_Bm*expm(0.5*c.dt*hat(BV_AB(:,t+1)));
end

%% Functions 
function R = Rx(th)
%Rotate around x with th degrees;
R = [1 0 0; 0 cos(deg2rad(th)) -sin(deg2rad(th)); 0 sin(deg2rad(th)) cos(deg2rad(th))];
end

function R = Ry(th)
%Rotate around y with th degrees;
R = [cos(deg2rad(th)) 0 sin(deg2rad(th)); 0 1 0;  -sin(deg2rad(th)) 0 cos(deg2rad(th))];
end

function R = Rz(th)
%Rotate around z with th degrees;
R = [cos(deg2rad(th)) -sin(deg2rad(th)) 0; sin(deg2rad(th)) cos(deg2rad(th)) 0; 0 0 1];
end

function res = hat(vec)
% Take the 3- or 6-vector representing an isomorphism of so(3) or se(3) and
% writes this as element of so(3) or se(3). 
%
% INPUTS:    vec     : 3- or 6-vector. Isomorphism of so(3) or se(3)
%
% OUTPUTS:   res     : element of so(3) or se(3)
%
%% Hat operator
if length(vec) == 3
    res = [0, -vec(3), vec(2); vec(3), 0, -vec(1); -vec(2), vec(1), 0];
elseif length(vec) == 6
    skew = [0, -vec(6), vec(5); vec(6), 0, -vec(4); -vec(5), vec(4), 0];
    v = [vec(1);vec(2);vec(3)];
    res = [skew, v; zeros(1,4)];
end
end

function res = vee(mat)
% Takes an element of so(3) or se(3) and returns its isomorphism in R^n.
%
% INPUTS:    mat     : element of so(3) or se(3)
%
% OUTPUTS:   res     : 3- or 6-vector. Isomorphism of so(3) or se(3)
%
%% Vee operator

xi1 = (mat(3,2)-mat(2,3))/2;
xi2 = (mat(1,3)-mat(3,1))/2;
xi3 = (mat(2,1)-mat(1,2))/2;

if length(mat) == 3
   res = [xi1; xi2; xi3];
elseif length(mat) == 4
   res = [mat(1:3,4);xi1;xi2;xi3]; 
end
end

% Matrix with force directions
function [WN,WT] = CompW(AR_B,AR_C,vertices)
% Compute the matrix containing the tangential force directions.
tel = 1;
for ii = 1:length(vertices(1,:))
    w = (AR_C'*[AR_B -AR_B*hat(vertices(:,ii))])';
    WN(:,ii) = w(:,3);
    WT(:,tel:tel+1) = w(:,1:2);
    tel = tel+2;
end
end
end
% %% Figures
% %Set plots to use LaTeX interface
% set(groot,'defaulttextinterpreter','latex');
% set(groot,'defaultAxesTickLabelInterpreter','latex');
% set(groot,'defaultLegendInterpreter','latex');
% 
% %Plot the trajectory of the box
% if plottrajectory ==1
%     tel = 1;
%     figure("pos",[500 100 1500 800]);
%     for ii=1:15:length(time)
%         AR_B = AH_B(1:3,1:3,ii);
% 
%         lcp = plotBox(AH_B(:,:,ii),box,[1 0 0]);
%         fp = plotBox(AH_B_fp(:,:,ii),box,[0 1 0]);
%         
%         
%         %Plot the inclined table C
%         table3 = fill3(spoints(1,1:4),spoints(2,1:4),spoints(3,1:4),1);hold on;
%         set(table3,'FaceColor',0.8*[1 1 1],'FaceAlpha',1);
%         
%         %Plot the origin of the contact surface with its unit vectors
%         tip = [Ao_C+0.3*AR_C(:,1) Ao_C+0.3*AR_C(:,2) Ao_C+0.3*AR_C(:,3)];
%         plot3([Ao_C(1) tip(1,1)],[Ao_C(2) tip(2,1)],[Ao_C(3) tip(3,1)],'r'); hold on
%         plot3([Ao_C(1) tip(1,2)],[Ao_C(2) tip(2,2)],[Ao_C(3) tip(3,2)],'g');
%         plot3([Ao_C(1) tip(1,3)],[Ao_C(2) tip(2,3)],[Ao_C(3) tip(3,3)],'b');
%         
%         %Plot the origin of the world coordinate frame
%         tip = [0.3*[1;0;0] 0.3*[0;1;0] 0.3*[0;0;1]];
%         plot3([0 tip(1,1)],[0 tip(2,1)],[0 tip(3,1)],'r'); hold on
%         plot3([0 tip(1,2)],[0 tip(2,2)],[0 tip(3,2)],'g');
%         plot3([0 tip(1,3)],[0 tip(2,3)],[0 tip(3,3)],'b');
%                 
%         grid on;axis equal;axis off;
%         axis([-1.8 0.8 -1.2 0.8 0.0 0.8]);
%         ax = gca;
%         ax.DataAspectRatioMode = 'manual';
%         ax.Clipping = 'off';
% %         legend([lcp, fp],'LCP','Fixed-point');
% %         zoom(ax,1.5)
% %         axis([-100 100 -100 100 -100 100]);
%         xlabel('x [m]');
%         ylabel('y [m]');
%         zlabel('z [m]');
% %         set(gca, 'ZDir','reverse')
%         view(-36,15);
%         drawnow
% %         pause()
%         hold off
%         tel = tel+1;
%     end
% end
% if dosave ==1
%     save('AH_B.mat','AH_B');
% end
% 
% finalPose=AH_B(:,:,end)
% 
% %% Functions

% 

% 
% function Bplot = plotBox(AH_B,box,color) 
% %% Box-simulator-FixedPoint:
% %This script is used to plot the box given a certain state
% %
% % INPUTS:    AH_B  : 4x4 double, pose of the box
% %            box   : struct, with fields of box properties as
% %                    box.B_M_B  : 6x6 double intertia tensor of the box
% %                    box.mass    : 1x1 double mass of the box
% %                    box.vertices: 3x8 double position of the vertices of 
% %                                  the box w.r.t body-fixed frame
% %            color : 3x1 double, rgb color of the box
% %
% % OUTPUTS:   Bplot : Plot of the box
% %% Plot the box
%         AR_B = AH_B(1:3,1:3);
%         %Output the position of the current time step for plotting purposes
%         q(:,1)  = AH_B(1:3,4);
%         Ao_B1{1} = AH_B(1:3,4);
%         R1(:,1) = AH_B(1:3,1);
%         R2(:,1) = AH_B(1:3,2);
%         R3(:,1) = AH_B(1:3,3);
%         
%         %Plot the origin of the box with its unit vectors
%         tip = [q(:,1)+ 0.3*R1(:,1) q(:,1)+ 0.3*R2(:,1) q(:,1)+ 0.3*R3(:,1)];
%         plot3([q(1,1) tip(1,1)],[q(2,1) tip(2,1)],[q(3,1) tip(3,1)],'r'); hold on
%         plot3([q(1,1) tip(1,2)],[q(2,1) tip(2,2)],[q(3,1) tip(3,2)],'g');
%         plot3([q(1,1) tip(1,3)],[q(2,1) tip(2,3)],[q(3,1) tip(3,3)],'b');
%         
%         %Create the box
% %         pbool = (abs(box.vertices(1,:))==max(abs(box.vertices(1,:))))&(abs(box.vertices(2,:))==max(abs(box.vertices(2,:))))&(abs(box.vertices(3,:))==max(abs(box.vertices(3,:))));
% %         Ap = AR_B*box.vertices(:,pbool)+AH_B(1:3,4);
%         Ap = AR_B*box.vertices+AH_B(1:3,4);
%         Ap_1 = Ap(:,1);
%         Ap_2 = Ap(:,2);
%         Ap_3 = Ap(:,6);
%         Ap_4 = Ap(:,5);
%         Ap_5 = Ap(:,3);
%         Ap_6 = Ap(:,4);
%         Ap_7 = Ap(:,8);
%         Ap_8 = Ap(:,7);
%         
%         plot3([Ap_1(1) Ap_2(1)],[Ap_1(2) Ap_2(2)],[Ap_1(3) Ap_2(3)],'k');%
%         plot3([Ap_2(1) Ap_3(1)],[Ap_2(2) Ap_3(2)],[Ap_2(3) Ap_3(3)],'k');%
%         plot3([Ap_3(1) Ap_4(1)],[Ap_3(2) Ap_4(2)],[Ap_3(3) Ap_4(3)],'k');
%         plot3([Ap_4(1) Ap_1(1)],[Ap_4(2) Ap_1(2)],[Ap_4(3) Ap_1(3)],'k');
%         plot3([Ap_5(1) Ap_6(1)],[Ap_5(2) Ap_6(2)],[Ap_5(3) Ap_6(3)],'k');%
%         plot3([Ap_6(1) Ap_7(1)],[Ap_6(2) Ap_7(2)],[Ap_6(3) Ap_7(3)],'k');%
%         plot3([Ap_7(1) Ap_8(1)],[Ap_7(2) Ap_8(2)],[Ap_7(3) Ap_8(3)],'k');
%         plot3([Ap_8(1) Ap_5(1)],[Ap_8(2) Ap_5(2)],[Ap_8(3) Ap_5(3)],'k');
%         plot3([Ap_1(1) Ap_5(1)],[Ap_1(2) Ap_5(2)],[Ap_1(3) Ap_5(3)],'k');
%         plot3([Ap_2(1) Ap_6(1)],[Ap_2(2) Ap_6(2)],[Ap_2(3) Ap_6(3)],'k');
%         plot3([Ap_3(1) Ap_7(1)],[Ap_3(2) Ap_7(2)],[Ap_3(3) Ap_7(3)],'k');
%         plot3([Ap_4(1) Ap_8(1)],[Ap_4(2) Ap_8(2)],[Ap_4(3) Ap_8(3)],'k');
%         
%         %Color the surfaces of the box
%         Bplot = fill3([Ap_1(1) Ap_2(1) Ap_6(1) Ap_5(1)],[Ap_1(2) Ap_2(2) Ap_6(2) Ap_5(2)],[Ap_1(3) Ap_2(3) Ap_6(3) Ap_5(3)],1,'FaceColor',color,'FaceAlpha',1);%Face F
%         fill3([Ap_1(1) Ap_2(1) Ap_3(1) Ap_4(1)],[Ap_1(2) Ap_2(2) Ap_3(2) Ap_4(2)],[Ap_1(3) Ap_2(3) Ap_3(3) Ap_4(3)],1,'FaceColor',color,'FaceAlpha',1);%Face A
%         fill3([Ap_8(1) Ap_7(1) Ap_6(1) Ap_5(1)],[Ap_8(2) Ap_7(2) Ap_6(2) Ap_5(2)],[Ap_8(3) Ap_7(3) Ap_6(3) Ap_5(3)],1,'FaceColor',color,'FaceAlpha',1);%Face C
%         fill3([Ap_8(1) Ap_7(1) Ap_3(1) Ap_4(1)],[Ap_8(2) Ap_7(2) Ap_3(2) Ap_4(2)],[Ap_8(3) Ap_7(3) Ap_3(3) Ap_4(3)],1,'FaceColor',color,'FaceAlpha',1);%Face E
%         fill3([Ap_1(1) Ap_4(1) Ap_8(1) Ap_5(1)],[Ap_1(2) Ap_4(2) Ap_8(2) Ap_5(2)],[Ap_1(3) Ap_4(3) Ap_8(3) Ap_5(3)],1,'FaceColor',color,'FaceAlpha',1);%Face D
%         fill3([Ap_2(1) Ap_3(1) Ap_7(1) Ap_6(1)],[Ap_2(2) Ap_3(2) Ap_7(2) Ap_6(2)],[Ap_2(3) Ap_3(3) Ap_7(3) Ap_6(3)],1,'FaceColor',color,'FaceAlpha',1);%Face B
% 
%         %Plot all contact points
%         vertices = AR_B*box.vertices+AH_B(1:3,4);
%         plot3(vertices(1,:),vertices(2,:),vertices(3,:),'.',MarkerSize=1)
%         
% end