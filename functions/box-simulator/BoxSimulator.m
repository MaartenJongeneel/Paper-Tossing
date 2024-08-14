function [AH_B, BV_AB, FN, FT] = BoxSimulator(x,c,box,surface)
%% Box-simulator-FixedPoint:
%This script uses an augmented Lagrangian approach (fixed-point iteration)
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
%            c.a                 : 1x1 double, Prox point auxilary parameter            [-]
%            c.tol               : 1x1 double, Error tol for fixed-point                [-]
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
Bv_AB = x.releaseLinVel;                      %Linear velocity at t0 of B wrt frame A  [m/s]
Bomg_AB = x.releaseAngVel;                    %Angular velocity at t0 of B wrt frame A [m/s]
PNfull = zeros(length(box.vertices),1);       %Initial guess for momenta PN            [N*s]
PTfull(1:length(box.vertices),1) = {[0;0]};   %initial guess for momenta PT            [N*s]
N = single(c.endtime/c.dt);                           %Run to this frame                       [-]

%% Preallocate memory to speed up the process
FT = NaN(length(box.vertices),N);
FN = NaN(length(box.vertices),N); 
AH_B = NaN(4,4,N);
E  = NaN(1,N);

%% Retrieve info of box struct
B_M_B = box.B_M_B;
                           
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
BV_AB(:,1) = [Bv_AB; Bomg_AB];

%% Dynamics

for ii=1:N
    %Rewrite the body velocity as a 4x4 matrix in se(3)
    BV_ABs = hat(BV_AB(:,ii));
    
    %Kinematics: Compute the configuration at the mid-time (tM)
    AH_Bm = AH_B(:,:,ii)*expm(0.5*c.dt*BV_ABs);
    AR_Bm = AH_Bm(1:3,1:3);
    Ao_Bm = AH_Bm(1:3,4);
    AR_B  = AH_B(1:3,1:3,ii);
    
    %Compute the wrench at tM
    B_fM = ([AR_Bm zeros(3); zeros(3) AR_Bm])'*BA_f;

    %And compute the gap-functions at tM column of normal contact distances
    gN=[];
    for jj=1:length(surface)
        %Tangential distance w.r.t surface frame
        gT = (surface{jj}.transform(1:3,1:2)'*(Ao_Bm + AR_Bm*box.vertices-surface{jj}.transform(1:3,4)));
        %Vertices outside the contact plane
        vert_o = find(sum((abs(gT)<(0.5*surface{jj}.dim'))==0));
        %Normal distance of the vertices w.r.t. the contact plane
        gN(:,jj) = (surface{jj}.transform(1:3,3)'*(Ao_Bm + AR_Bm*box.vertices-surface{jj}.transform(1:3,4)));

        %If the object is outside the plane area or underneath it, we do
        %not take it into account in solving the contact problem
        if (~isempty(vert_o)) 
            gN(vert_o,jj)=0;
        elseif (all(gN(:,jj)<0))
            gN(:,jj)=0;
        end
    end
    
    %Obtain the linear and angular velocity at tA
    vA = BV_AB(:,ii);
    
    %If a gap function has been closed, contact has been made
    [IN,surf] = find(gN<0);
    if  IN > 0
        if length(surf)>1
            surf = unique(surf);
        end
        indx = gN<0;
        %Compute the matrix containing force directions
        WNAt =[]; WTAt=[]; WNMt=[]; WTMt=[];
        for jj=1:length(surf)
            [WNA{jj}, WTA{jj}] = CompW(AR_B,surface{surf(jj)}.transform(1:3,1:3),box.vertices(:,indx(:,surf(jj))));
            [WNM{jj}, WTM{jj}] = CompW(AR_Bm,surface{surf(jj)}.transform(1:3,1:3),box.vertices(:,indx(:,surf(jj))));
            WNAt = [WNAt WNA{jj}];
            WTAt = [WTAt WTA{jj}];
            WNMt = [WNMt WNM{jj}];
            WTMt = [WTMt WTM{jj}];
        end
        
        %Give an initial guess for the normal and tangential momenta
        PN=PNfull(IN);
        PT=cell2mat(PTfull(IN));
        term1 = B_fM*c.dt - [hat(vA(4:6)), zeros(3); hat(vA(1:3)), hat(vA(4:6))]*B_M_B*vA*c.dt;
        converged = 0;
        tel = 1;
        while converged==0
            %Discrete time dynamics: Equations of motion
            vE = vA + B_M_B\(term1 + WNMt*PN + WTMt*PT);
            
            %Define the normal and tangential velocities at tA and tM. We
            %substranct from the relative velocity the velocity of the
            %contact surface.
            gammaNA=[]; gammaNE=[]; gammaTA=[]; gammaTE=[];
            for jj=1:length(surf)
                speed = surface{surf(jj)}.speed';
                gammaNA = [gammaNA; (WNA{jj}'*vA-repmat(surface{surf(jj)}.speed(3,1),sum(indx(:,surf(jj))),1))];
                gammaNE = [gammaNE; (WNM{jj}'*vE-repmat(surface{surf(jj)}.speed(3,1),sum(indx(:,surf(jj))),1))];

                %Define the tangential velocities at tA and tM
                gammaTA = [gammaTA; (WTA{jj}'*vA-repmat(surface{surf(jj)}.speed(1:2,1),sum(indx(:,surf(jj))),1))];
                gammaTE = [gammaTE; (WTM{jj}'*vE-repmat(surface{surf(jj)}.speed(1:2,1),sum(indx(:,surf(jj))),1))];
            end
            
            %Newtons restitution law
            xiN = gammaNE+c.eN*gammaNA;
            xiT = gammaTE+c.eT*gammaTA;
            
            %Using prox functions: project PN and PT
            PNold = PN;
            PTold = PT;
            PN = proxCN(PN-c.a*xiN);
            PT = proxCT(PT-c.a*xiT,c.mu*PN);

            %Compute the error
            error(tel) = norm(PN-PNold)+norm(PT-PTold);
            
            %Check for convergence
            converged = error(tel)<c.tol;
            tel = tel+1;
        end
        BV_AB(:,ii+1) = vE;

    else
        %Update the velocity to the next time step using configuration at tM
        vE = B_M_B\(B_fM*c.dt - [hat(vA(4:6)), zeros(3); hat(vA(1:3)), hat(vA(4:6))]*B_M_B*vA*c.dt) + vA;
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
    AH_B(:,:,ii+1)  = AH_Bm*expm(0.5*c.dt*hat(BV_AB(:,ii+1)));
    
    %Kinetic energy
    E(ii) = 0.5*BV_AB(:,ii)'*B_M_B*BV_AB(:,ii);  %Kinetic energy
    
    FN(1:length(PN),ii) = PN;
    FT(1:length(PT),ii) = PT;
end

end

%% Matrix with force directions
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

%% Proximal point Normal
function y=proxCN(x)
% Proximal point formulation for CN. See thesis for reference.
% prox_CN(x) = 0 for x=< 0
%            = x for x > 0
y=max(x,0);
end

%% Proximal point Tangential
function y=proxCT(x,a)
% Proximal point formulation for CT. See thesis for reference.
%
% prox_CT(x) = x           for ||x|| =< a
%            = a*(x/||x||) for ||x||  > a
% for CT = {x in R^n| ||x|| =< a}
cnt = 1;
for ii = 1:length(a) %For each point in contact
    if norm(x(cnt:cnt+1)) <= a(ii)
        y(cnt:cnt+1,1) = x(cnt:cnt+1); %Stick
    else
        y(cnt:cnt+1,1) = a(ii)*x(cnt:cnt+1)/norm(x(cnt:cnt+1)); %Slip
    end
    cnt = cnt+2;
end
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