function [indx_b, indx_c, BV_CBaf, BV_CBef, BCV_CB, BCV_CBaf,BCV_CBef] = IdentifyPPinds(dCo_p,w_ext,g,freq,gravity,CH_Bimp,BV_CBimp,indx)
% This function automatically identifies the indices of the pre- and post
% impact time based on the velocity of the contact point. The velocity
% profile looks like a logistic function with gravity acceleration and
% initial speed. Pre- and post-impact time are the first discrete time
% steps ti_b, when the contact point velocity starts to deviate from the
% expected velocity of a free-falling object. This deviation is reflected
% by the value of Vmin, assigned below.
%
% INPUTS:    dCo_p       : [cell] Array of linear velocity vectors
%                          expressing the linear velocity of contact point
%                          p w.r.t. C
%            w_ext       : [double] Scalar indicating window around impact
%            plotbool    : [boolian] (0/1 or false/true) to indicate to
%                           plot or not
%
% OUTPUTS:   indx_b      : [double] Scalar indicating begin point of impact
%            indx_c      : [double] Scalar indicating end point of impact
%
%% Identify the pre- and post-impact velocity indices.
%% Settings
%If no indx is given, we are going to compute it ourselves
  
idx   = 3;    % Index of the velocity vector (component in which the impact is happening)    
Vper  = 0.04; % Minimum velocity difference from steady state in percentage    
Rmin  = 0.99; % Coeffcient of determination    
dImax = 0.4;  % Maximum difference by index rounding before a check is required    
a_max = 8;    % The steepness of the logistic function may not exceed alpha_max

% Create indices vector
k = (-w_ext:w_ext)';        % Crude indices, comply with OptiTrack freq.
kf = (-w_ext:0.01:w_ext)';  % Refined indices for fitting purposes

% Standard function: logistic function with gravity acceleration and initial speed
if gravity
    f = @(a,b,c,d,x)a./(exp(b*(c - x)) + 1) - g/freq*x - d;
else
    f = @(a,b,c,d,x)a./(exp(b*(c - x)) + 1) - d;
end

% Fit function to velocity profile from OptiTrack
Cfit = fit(k,dCo_p(idx,:)',f,'StartPoint',[1.5,2,0,1]);

% Fitted vertical contact point velocity
F = feval(Cfit,kf);

% Find pre-impact index and project -w_ext:w_ext onto 1:2*w_ext+1
if gravity
    Vmin = Vper*max(F + g/freq*kf + Cfit.d);
    indx_bf = find(F + g/freq*kf + Cfit.d>Vmin, 1, 'first');
    indx_cf = find(F + g/freq*kf + Cfit.d-Cfit.a>-Vmin, 1, 'first');
else
    Vmin = Vper*max(F + Cfit.d);
    indx_bf = find(F + Cfit.d>Vmin, 1, 'first');
    indx_cf = find(F + Cfit.d-Cfit.a>-Vmin, 1, 'first');
end

if ~exist('indx','var')
    indx_b  = round(kf(indx_bf)) +1+w_ext;
    indx_c  = round(kf(indx_cf)) +1+w_ext;

    % Check validity of estimation, determine fit quality by R2 test
    SSRes = sum((dCo_p(idx,:)'-feval(Cfit,k)).^2);
    SST = sum((dCo_p(idx,:)-mean(dCo_p(idx,:))).^2);
    R2 = 1-SSRes/SST;

    % Some extra checks to see if indices b and c are valid.
    if isempty(indx_b) || isempty(indx_c)
        indx_b = 1; indx_c = 1; indx_bf = 1; indx_cf = 1;
    end
    if indx_b == indx_c
        warning('Indices overlap!')
    end
    if Cfit.b>a_max
        warning('Velocity fit is so steep, indices cannot be estimated accurately')
    end
    if R2<Rmin
        warning('Fit of points is not very accurate according to R2 test');
    end
    if abs(kf(indx_bf) - indx_b +1+w_ext)>dImax
        warning('First index is close to the middle between two indices, rounding is therefore indecisive');
    end
    if  abs(kf(indx_cf) - indx_c +1+w_ext)>dImax
        warning('Last index is close to the middle between two indices, rounding is therefore indecisive');
    end
else
    indx_b = indx(1);
    indx_c = indx(2);
end

%% Plotting of velocity  and identified indices
% Fit velocity of body (eq. 4.16 of Luuk's report)
for it = 1:11
    % Express velocity in body frame with orientation of conveyor (inertia) frame
    BCV_CB(:,it) = [CH_Bimp{it}(1:3,1:3)*BV_CBimp{it}(1:3);CH_Bimp{it}(1:3,1:3)*BV_CBimp{it}(4:6)];
end
for it = 1:11
    % Take mean of velocity after substracting gravitational influence
    if gravity
        BCV_CBa_avg(:,it)      = mean(BCV_CB(:,1:it)         +([0,0,g/freq,0,0,0]'*(0:it-1)),2);
        BCv_CBe_avg(:,12-it)   = mean(BCV_CB(:,end-it+1:end) +([0,0,g/freq,0,0,0]'*(-it+1:0)),2);
    else
        BCV_CBa_avg(:,it)      = mean(BCV_CB(:,1:it)         ,2);
        BCv_CBe_avg(:,12-it)   = mean(BCV_CB(:,end-it+1:end) ,2);
    end
end

% Determine fit of velocity
if gravity
    BCV_CBaf = BCV_CBa_avg(:,indx_b) - ([0,0,g/freq,0,0,0]'*(0:10));
    BCV_CBef = BCv_CBe_avg(:,indx_c) - ([0,0,g/freq,0,0,0]'*(-10:0));
else
    BCV_CBaf = BCV_CBa_avg(:,indx_b) - zeros(6,1)*(0:10);
    BCV_CBef = BCv_CBe_avg(:,indx_c) - zeros(6,1)*(-10:0);
end
% Fitted box twist back to body orientation
for it = 1:11
    BV_CBaf{it} = [CH_Bimp{it}(1:3,1:3)'*BCV_CBaf(1:3,it);CH_Bimp{it}(1:3,1:3)'*BCV_CBaf(4:6,it)];
    BV_CBef{it} = [CH_Bimp{it}(1:3,1:3)'*BCV_CBef(1:3,it);CH_Bimp{it}(1:3,1:3)'*BCV_CBef(4:6,it)];
end

%% Plot the figures
%%

fig = figure;
set(gcf,'Position', [50  280.2000  607.2000  407.2000])

plot(k,dCo_p(1,:),'c',k,dCo_p(2,:),'m',k,dCo_p(3,:),'g');
grid; hold('On')
title('Contact point velocity')
plot(kf,F,'--k');
xline(kf(indx_bf),'-.','color','k','linewidth',1);
xline(indx_b-1-w_ext,'-.','color','r','linewidth',1);
xline(kf(indx_cf),'-.','color','k','linewidth',1);
xline(indx_c-1-w_ext,'-.','color','r','linewidth',1);
xline(-5,'-.','color','r','linewidth',1);
xline(5,'-.','color','r','linewidth',1);
xlabel('Normalized time around the impact time $(t-t_j)/\Delta t$ [-]');
ylabel('C.P. velocity ${^C\dot{\mathbf{p}}_i}$ [m/s]');
scatter(kf([indx_bf,indx_cf]),F([indx_bf,indx_cf]),'xk','linewidth',1) %Plot fitted points in continuous function
scatter([indx_b,indx_c]-1-w_ext,dCo_p(idx,[indx_b,indx_c]),'r','linewidth',1) %Plot fitted points in discrete function
text(-5,1.6,'-a','FontSize',12);
text(indx_b-1-w_ext-0.1,1.6,'-b','FontSize',12);
text(indx_c-1-w_ext-0.1,1.6,'c','FontSize',12);
text(5-0.1,1.6,'d','FontSize',12);
legend('$(^C\dot{\mathbf{p}}_i)_x$','$(^C\dot{\mathbf{p}}_i)_y$',...
    '$(^C\dot{\mathbf{p}}_i)_z$','$(^C\dot{\mathbf{p}}_i)_z$-fit',...
    'Cont. indices', 'Disc. indices $a$, $b$, $c$, $d$','Location','SouthEast')
xlim([-5.05 5.05]);
% end

%Plot the hybrid velocity and fitted data to see if it
%can be used
figure('rend','painters','pos',[660  280.2000 2*380 250]);
ha = tight_subplot(1,2,[.08 .07],[.18 .15],[0.08 0.03]);  %[gap_h gap_w] [lower upper] [left right]
axes(ha(1));
plot(-5:5,BCV_CB(1:3,:)); grid on; hold on; %Measurement
plot(-5:(indx_b-6),BCV_CBaf(1:3,1:indx_b)','-.','color','k');
plot((indx_c-6):5,BCV_CBef(1:3,indx_c:11)','-.','color','k');
xlabel('Normalized time around the impact time $(t-t_j)/\Delta t$ [-]');
ylabel('Linear velocity $^{B[C]}$\boldmath${v}_{C,B}$ [m/s]');
ha(1).XTick = [-5:5];

axes(ha(2));
plot(-5:5,BCV_CB(4:6,:)); grid on; hold on; %Measurement
plot(-5:(indx_b-6),BCV_CBaf(4:6,1:indx_b)','-.','color','k');
plot((indx_c-6):5,BCV_CBef(4:6,indx_c:11)','-.','color','k');
xlabel('Normalized time around the impact time $(t-t_j)/\Delta t$ [-]');
ylabel('Angular velocity $^{B[C]}$\boldmath${\omega}_{C,B}$ [m/s]');
ha(2).XTick = [-5:5];
L1 = legend('$x$-meas','$y$-meas','$z$-meas','Fitted','NumColumns',7,'location','northeast');
L1.Position(2) = 0.91;
L1.Position(1) = 0.52-(L1.Position(3)/2);

pause(0.1)
indx_input = input('\n Is estimation OK? Press ENTER to accept values, or give [-b, c]. If not good, give [0,0]:\n');
if ~isempty(indx_input) %If not empty, it means you overwrite the computed values, so we need to recompute the other values with the new given values
    indx_b = indx_input(1)+w_ext+1; indx_c = indx_input(2)+w_ext+1;
    if indx_b == 6 || indx_c == 6 %in case we say it is not good
        return;
    end    

    %If we changed it, we need to recompute, so 
    IdentifyPPinds(dCo_p,w_ext,g,freq,gravity,CH_Bimp,BV_CBimp,[indx_b indx_c]);
end
