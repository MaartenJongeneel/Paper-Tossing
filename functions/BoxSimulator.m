function [AH_B, BV_AB, FN, FT] = BoxSimulator(releasePosition,releaseOrientation,releaseLinVel,releaseAngVel,eN,eT,mu,box,AR_C,Ao_C,dt,Ntimeidx)
%% Box-simulator-FixedPoint:
%This script uses an augmented Lagrangian approach (fixed-point iteration)
%for solving the nonlinear algebraic equations for contact impact and
%friction. The box has a body fixed frame B at its COM. Furthermore, the
%contact surface has its position and orientation defined by frame C, and
%we express all positions and velocities in terms of frame A, located at
%the camera coordinate frame of a camera, also plotted in the figure.
%
% INPUTS:    releasePosition     : 3x1 double, position of the box at release
%            releaseOrientation  : 3x3 double, orientation of the box at release
%            releaseLinVel       : 3x1 double, linear velocity of the box at release
%            releaseAngVel       : 3x1 double, angular velocity of the box at release
%            eN                  : 1x1 double, normal coefficient of restitution
%            eT                  : 1x1 double, tangential coefficient of restitution
%            mu                  : 1x1 double, coefficient of friction
%            box                : struct, with fields of box properties as
%                                  box.B_M_B  : 6x6 double intertia tensor of the box
%                                  box.mass    : 1x1 double mass of the box
%                                  box.vertices: 3x8 double position of the vertices of 
%                                                the box w.r.t body-fixed frame
%            AR_C                : 3x3 double, Orientation of the contact surface
%            Ao_C                : 3x1 double, Position of the contact surface
%            dt                  : 1x1 double, Timestep at which the simulator runs
%            Ntimeidx          : 1x1 double, Number of discrete time indices you want to run the simulation
%
% OUTPUTS:   AH_B                : Pose of the box over time
%            BV_AB               : Left trivialized velocity of the box over time
%            FN                  : Normal force acting on the box over time
%            FT                  : Tangential force acting on the box over time
%% Constants and settings
g     = 9.81;              %Gravitational acceleration              [m/s^2]
a     = 0.0001;              %Prox point auxilary parameter           [-]
tol   = 1e-6;              %Error tol for fixed-point               [-]
Cz_C  = [0;0;1];           %z-component of the C frame              [-]
Cy_C  = [0;1;0];           %y-component of the C frame              [-]
Cx_C  = [1;0;0];           %x-component of the C frame              [-]
Bv_AB = releaseLinVel;     %Linear velocity at t0 of B wrt frame A  [m/s]
Bomg_AB = releaseAngVel;   %Angular velocity at t0 of B wrt frame A [m/s]
PNfull = zeros(8,1);       %Initial guess for momenta PN            [N*s]
PTfull(1:8,1) = {[0;0]};   %initial guess for momenta PT            [N*s]
run    = true;             %Boolean to determine if box is moving   [-]
% Ntimeidx = endtime/dt;     %Run to this frame                       [-]

%% Preallocate memory to speed up the process
FT = NaN(8,Ntimeidx);
FN = NaN(4,Ntimeidx); 
AH_B = cell(1,Ntimeidx);
E  = NaN(1,Ntimeidx);

%% Retrieve info of box struct
Bp_1 = box.vertices.ds(1,:)'; Bp_5 = box.vertices.ds(5,:)';
Bp_2 = box.vertices.ds(2,:)'; Bp_6 = box.vertices.ds(6,:)';
Bp_3 = box.vertices.ds(3,:)'; Bp_7 = box.vertices.ds(7,:)';
Bp_4 = box.vertices.ds(4,:)'; Bp_8 = box.vertices.ds(8,:)';

B_M_B = box.inertia.ds;
                           
%% Wrench acting on body B: expressed in B with orientation of A:
BA_fo  = [0; 0; -box.mass.ds*g;];
BA_Tau = [0; 0; 0];
BA_f   = [BA_fo; BA_Tau];

%% Initial pose and velocity
%Initial pose of the box: Transformation matrix B w.r.t. frame A
AR_B    = releaseOrientation;%Initial orientation
Ao_B    = releasePosition;   %Initial position
AH_B{1} = [AR_B, Ao_B; zeros(1,3), 1]; %Homogeneous transformation matrix

%Initial left trivialized velocity of frame B w.r.t. frame A
BV_AB(:,1) = [Bv_AB; Bomg_AB];

%% Defining the contact plane
%Direction of the normal and in-plane vectors of the plane w.r.t. frame A
Az_C = AR_C*Cz_C;
Ay_C = AR_C*Cy_C;
Ax_C = AR_C*Cx_C;

%% Dynamics
ii = 1;
while run 
    %Rewrite the body velocity as a 4x4 matrix in se(3)
    BV_ABs = hat(BV_AB(:,ii));
    
    %Kinematics: Compute the configuration at the mid-time (tM)
    AH_Bm = AH_B{ii}*expm(0.5*dt*BV_ABs);
    AR_Bm = AH_Bm(1:3,1:3);
    Ao_Bm = AH_Bm(1:3,4);
    AR_B  = AH_B{ii}(1:3,1:3);
    
    %Compute the wrench at tM
    B_fM = ([AR_Bm zeros(3); zeros(3) AR_Bm])'*BA_f;
    
    %And compute the gap-functions at tM
    gNm1 = Az_C'*((Ao_Bm + AR_Bm*Bp_1)-Ao_C);
    gNm2 = Az_C'*((Ao_Bm + AR_Bm*Bp_2)-Ao_C);
    gNm3 = Az_C'*((Ao_Bm + AR_Bm*Bp_3)-Ao_C);
    gNm4 = Az_C'*((Ao_Bm + AR_Bm*Bp_4)-Ao_C);
    gNm5 = Az_C'*((Ao_Bm + AR_Bm*Bp_5)-Ao_C);
    gNm6 = Az_C'*((Ao_Bm + AR_Bm*Bp_6)-Ao_C);
    gNm7 = Az_C'*((Ao_Bm + AR_Bm*Bp_7)-Ao_C);
    gNm8 = Az_C'*((Ao_Bm + AR_Bm*Bp_8)-Ao_C);
    
    %column of normal contact distances
    gN = [gNm1;gNm2;gNm3;gNm4;gNm5;gNm6;gNm7;gNm8];
    
    %Obtain the linear and angular velocity at tA
    vA = BV_AB(:,ii);
    
    %If a gap function has been closed, contact has been made
    IN = find(gN<0);
    if  IN > 0
        %Compute the matrix containing the normal force directions at tA and tM
        WNta = CompWN(AR_B,Az_C,Bp_1,Bp_2,Bp_3,Bp_4,Bp_5,Bp_6,Bp_7,Bp_8);
        WNtm = CompWN(AR_Bm,Az_C,Bp_1,Bp_2,Bp_3,Bp_4,Bp_5,Bp_6,Bp_7,Bp_8);
        %Compute the matrix containing the tangential force directions at tA and tM
        WTta = CompWT(AR_B,Ax_C,Ay_C,Bp_1,Bp_2,Bp_3,Bp_4,Bp_5,Bp_6,Bp_7,Bp_8);
        WTtm = CompWT(AR_Bm,Ax_C,Ay_C,Bp_1,Bp_2,Bp_3,Bp_4,Bp_5,Bp_6,Bp_7,Bp_8);
        
        %Matrix with force directions at tA and tM
        WNA = WNta(:,IN);
        WNM = WNtm(:,IN);
        WTA = cell2mat(WTta(:,IN));
        WTM = cell2mat(WTtm(:,IN));
        
        %Give an initial guess for the normal and tangential momenta
        PN=PNfull(IN);
        PT=cell2mat(PTfull(IN));
        converged = 0;
        while converged==0
            %Discrete time dynamics: Equations of motion
            vE = vA + B_M_B\(B_fM*dt - [hat(vA(4:6)), zeros(3); hat(vA(1:3)), hat(vA(4:6))]*B_M_B*vA*dt + WNM*PN + WTM*PT);
            
            %Define the normal velocities at tA and tM
            gammaNA = WNA'*vA;
            gammaNE = WNM'*vE;
            
            %Define the tangential velocities at tA and tM
            gammaTA = WTA'*vA;
            gammaTE = WTM'*vE;
            
            %Newtons restitution law
            xiN = gammaNE+eN*gammaNA;
            xiT = gammaTE+eT*gammaTA;
            
            %Using prox functions: project PN and PT
            PNold = PN;
            PTold = PT;
            PN = proxCN(PN-a*xiN);
            PT = proxCT(PT-a*xiT,mu*PN);
            
            %Compute the error
            error = norm(PN-PNold)+norm(PT-PTold);
            
            %Check for convergence
            converged = error<tol;
        end
        BV_AB(:,ii+1) = vE;
    else
        %Update the velocity to the next time step using configuration at tM
        vE = B_M_B\(B_fM*dt - [hat(vA(4:6)), zeros(3); hat(vA(1:3)), hat(vA(4:6))]*B_M_B*vA*dt) + vA;
        BV_AB(:,ii+1) = vE;
        PN = 0;
        PT = [0;0];
    end
    %Give estimate for PN and PT for next timestep (speeds up convergence
    %in case of persistant contact)
    if IN ~= 0
        PNfull(IN)=PN;
        cnt=1;
        for jj = length(IN)
            PTfull(IN(jj)) = {PT(cnt:cnt+1)};
            cnt=cnt+2;
        end
    end
    
    %Complete the time step
    AH_B{ii+1}  = AH_Bm*expm(0.5*dt*hat(BV_AB(:,ii+1)));
    
    %Kinetic energy
    E(ii) = 0.5*BV_AB(:,ii)'*B_M_B*BV_AB(:,ii);  %Kinetic energy
    
    FN(1:length(PN),ii) = PN;
    FT(1:length(PT),ii) = PT;
    
    %Run untill the last frame
    if  ii == Ntimeidx-1
       run = false;
    end
    ii = ii+1;
end

end
