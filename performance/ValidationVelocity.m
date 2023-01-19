close all; set(groot,'defaulttextinterpreter','latex'); set(groot,'defaultAxesTickLabelInterpreter','latex'); set(groot,'defaultLegendInterpreter','latex');
%% Compare AGX with MATLAB and MEASUREMENTS
%This functions uses the impact_data.mat file created by "getImpactData.m"
%and writes the initial states, as used for simulations for parameter 
%identification, to a csv file, which can then be used for the simulations
%in AGX.
%
%Note that impacts.mat is used in 'writeAGXstates.m', where a subset of
%impacts is used to create the inital states. This therefore defines the
%total number of simulation results. Hence, the selected impacts there
%should match the selected impacts here.


%% ------------------------------ Settings ------------------------------%%
%These will be input parsers to a function
Object = 'Box006';
Environment = "Conveyor002";  %Select what conveyor you want to consider
impact_data = append('paramID/',Object,'_Vel/',Object,'_impacts.mat');
% AGXResult_h5file = append('paramID/',Object,'_Vel/',Object,'_ParamID_Vel_BoxTossBatch_result.hdf5');
evalAlgoryx = false;
evalMatlab = true;
evalMuJoCo = false;

dt        = 1/360;  %OptiTrack time step
g         = 9.81;   %Gravitational constant                    [m/s^2]
freq      = 1/dt;   %OptiTrack sample frequency
w_ext     = 5;      %extension window
endframe  = 11;     %Determine to what frame to run MATLAB
weight    = [1 1 1 0.1 0.1 0.1]; %Weight of the cost function

MATLAB.Box004.Vel   = [0.40 0.00 0.60]; %eN eT mu
MATLAB.Box004.Traj  = [0.00 0.00 0.45]; %eN eT mu [0.05 0.00 0.45]; %eN eT mu
MATLAB.Box005.Vel   = [0.35 0.00 0.45]; %eN eT mu
MATLAB.Box005.Traj  = [0.00 0.00 0.45]; %eN eT mu [0.10 0.00 0.45]; %eN eT mu
MATLAB.Box006.Vel   = [0.40 0.00 0.25]; %eN eT mu 
MATLAB.Box006.Traj  = [0.00 0.40 0.40]; %eN eT mu [0.25 0.00 0.40]; %eN eT mu
MATLAB.Box007.Vel   = [];
MATLAB.Box007.Traj  = [];

ObjStr = "Box006"; %The object for which you want to do paramID
ImpPln = "ConveyorPart002"; %"GroundPlane001";
Param = "Vel";  %Vel or Traj based parameters are tested 

doPlot    = false;
doSave    = false;

color.Green = [0.4660 0.6740 0.1880];
color.Blue = [0 0.4470 0.7410];
color.Red = [0.8500 0.3250 0.0980]; 
color.GreenLight = [0.4660 0.6740 0.1880 0.5];
color.BlueLight = [0 0.4470 0.7410 0.5];
color.RedLight = [0.8500 0.3250 0.0980 0.5];
color.Matlab = [237 176 33]/255;
color.Algoryx = [77 191 237]/255;
color.Meas = [128 128 128]/255;

%% ----------- Load the data and evalute the function inputs ----------- %%
%Load the impacts.mat file as used for Algoryx/Matlab/MuJoCo simulations
load(impact_data,"impacts");

%Select what impacts you want to evaluate
ind_B = contains(impacts.box.name,Object);     %Select the object
ind_S = contains(impacts.surface,Environment); %Select the conveyor
% ind_M = vecnorm(cell2mat(impacts.CV_CD(:)'),2)' == 0; %Only idle conveyor
ind_Tot = ind_B & ind_S; %& ind_M;               %total selection
imp_sel = find(ind_Tot);                       %Selected impact events

maxImpactEvel = length(imp_sel);  %Determine to what impact you want to evalute. Max value should be smaller than length(imp_sel) 

%If Algoryx is used, load the simulation results
if evalAlgoryx    
    AGXData = readH5(AGXResult_h5file);
    Results = orderfields(AGXData.box);
end


%% Write the release states to CSV file for Algoryx simulation
for is = 1:maxImpactEvel
    MH_B_rel(:,:,imp_sel(is)) = impacts.MH_B{imp_sel(is),1};
    BV_MB_rel(:,imp_sel(is)) = impacts.BV_CBaf{imp_sel(is),1};
    MH_C_rel(:,:,imp_sel(is)) = impacts.MH_C{imp_sel(is),6}; %Take MH_C at the moment of impact (=index 6)
end

writeAGXinitstates(MH_B_rel(1:3,1:3,:),MH_B_rel(1:3,4,:),BV_MB_rel(1:3,:),BV_MB_rel(4:6,:),MH_C_rel(1:3,1:3,:),MH_C_rel(1:3,4,:),append('paramID/',Object,'_Vel/'));

%% Write the release states to CSV file for MuJoCO simulations
writeMuJoCoStates(MH_B_rel(1:3,1:3,:),MH_B_rel(1:3,4,:),BV_MB_rel)
%% -------------------- Evaluate the impacts Algoryx --------------------%%
if evalAlgoryx
    % Because we obtain the values for mu, eN, and eT from Algoryx
    % simulations, we delete the values here
    clear mu eN eT
    
    for is = 1%:maxImpactEvel
        BCV_CBef   = cell2mat(impacts.BCV_CBef(imp_sel(is),:));
        MH_C       = impacts.MH_C(imp_sel(is));
        Mo_C       = MH_C{1}(1:3,4);
        MR_C       = MH_C{1}(1:3,1:3);

        indx_b = impacts.indx(imp_sel(is),3) - impacts.indx(imp_sel(is),2)+6;
        indx_c = impacts.indx(imp_sel(is),4) - impacts.indx(imp_sel(is),2)+6;
        
        %Vertices
        Bp = impacts.box.V{imp_sel(is)}; %Vertices
        
        %Initialize the AGX data import
        mu_i = 1; eN_i = 1; eT_i = 1;
        
        %Extract information from AGX simulation result on what COF, CONR, and COTR were used
        mu(mu_i) = str2double(strtrim(extractAfter(extractBefore(extractAfter(string(AGXData.model.BoxTossBatch.ds),"friction"),"step"),"default:")));
        eN(eN_i) = str2double(strtrim(extractAfter(extractBefore(extractAfter(string(AGXData.model.BoxTossBatch.ds),"normal_restitution"),"step"),"default:")));
        eT(eT_i) = str2double(strtrim(extractAfter(extractBefore(extractAfter(string(AGXData.model.BoxTossBatch.ds),"tangential_restitution"),"step"),"default:")));
        fn = fieldnames(Results);
        num_par = (length(fn)/(length(imp_sel)));
        cnt = 0; %Counter to count what parameter combination we are considering (0 <= cnt <= num_par)
        %For the current impact event, we are now looping through the parameters
        for ip = ((num_par*(is-1))+1):((num_par*(is))) %For all the parameters for this impact event
            cnt = cnt+1;
            %Select the data
            t = AGXData.box.(fn{ip}).ds;
            conf_nm = AGXData.box.(fn{ip}).attr.configuration_data_names;
            mu2 = str2double(extractBefore(extractAfter(conf_nm,"friction="),","));
            eN2 = str2double(extractBefore(extractAfter(conf_nm,"normal_restitution="),","));
            eT2 = str2double(extractAfter(conf_nm,"tangential_restitution="));
            
            if sum(mu2 == mu) == 0;  mu_i = mu_i +1; mu(mu_i) = mu2; else mu_i = find(mu2 == mu); end
            if sum(eN2 == eN) == 0;  eN_i = eN_i +1; eN(eN_i) = eN2; else eN_i = find(eN2 == eN); end
            if sum(eT2 == eT) == 0;  eT_i = eT_i +1; eT(eT_i) = eT2; else eT_i = find(eT2 == eT); end
            
            %Obtain AGX results
            for ii = 1:length(t(:,1))
                MH_B_AGX(:,:,ii,cnt) = [t(ii,1) t(ii,5) t(ii,9) t(ii,13); t(ii,2) t(ii,6) t(ii,10) t(ii,14); t(ii,3) t(ii,7) t(ii,11) t(ii,15); t(ii,4) t(ii,8) t(ii,12) t(ii,16)];
                BMV_MB_AGX(:,ii) = t(ii,17:22)';
                MX_B = [MH_B_AGX(1:3,1:3,ii,cnt), zeros(3,3); zeros(3,3), MH_B_AGX(1:3,1:3,ii,cnt)];
                BV_MB_AGX(:,ii) = MX_B\BMV_MB_AGX(:,ii);
                BCV_MB_AGX(:,ii,cnt) = [MR_C\MH_B_AGX(1:3,1:3,ii,cnt)*BV_MB_AGX(1:3,ii);MR_C\MH_B_AGX(1:3,1:3,ii,cnt)*BV_MB_AGX(4:6,ii)];
                for iv = 1:length(Bp)
                    dCp_AGX(:,iv,ii,cnt) = [(MR_C)\MH_B_AGX(1:3,1:3,ii,cnt) -MR_C\MH_B_AGX(1:3,1:3,ii,cnt)*hat(Bp(:,iv))]*BV_MB_AGX(:,ii);
                    Cp_AGX(:,iv,ii,cnt) = (MR_C)\(MH_B_AGX(1:3,4,ii,cnt) - Mo_C)+(MR_C)\MH_B_AGX(1:3,1:3,ii,cnt)*(Bp(:,iv));
                end
            end

            %Compute the cost
            E_AGX(mu_i,eN_i,eT_i,is)= norm(diag(weight)*(BCV_CBef(:,indx_c)-BCV_MB_AGX(:,indx_c,cnt)));
            
        end
        CurrentE_AGX = E_AGX(:,:,:,is);
        [~,optAGX_idx]= min(CurrentE_AGX(:));
                
        [a1,b1,c1]=ind2sub(size(E_AGX),optAGX_idx);
        if length(a1)==1 && length(b1)==1 && length(c1)==1
            AGXmu_opt(is) = mu(a1);
            AGXeN_opt(is) = eN(b1);
            AGXeT_opt(is) = eT(c1);
        end
        AGXwi(is) = E_AGX(a1,b1,c1,is);
    end
end


%% -------------------- Evaluate the impacts Matlab --------------------%%
if evalMatlab
    for is = 1:maxImpactEvel
        %Load impact data from struct
        MH_B_meas  = impacts.MH_B(imp_sel(is),:);
        CH_B_meas  = impacts.CH_B(imp_sel(is),:);
        BV_CBaf    = impacts.BV_CBaf(imp_sel(is),:);
        BCV_CBef   = cell2mat(impacts.BCV_CBef(imp_sel(is),:));

        MH_C       = impacts.MH_C(imp_sel(is),:);
        MR_C       = MH_C{6}(1:3,1:3); %Take rotation at moment of impact
        Mo_C       = MH_C{6}(1:3,4);   %Take position at moment of impact

        indx_c = impacts.indx(imp_sel(is),4) - impacts.indx(imp_sel(is),2)+6;

        %Load box parameters
        box.inertia.ds = impacts.box.M{imp_sel(is)};    %Inertia tensor
        box.vertices.ds = impacts.box.V{imp_sel(is)}'; %Vertices
        box.mass.ds = impacts.box.M{imp_sel(is)}(1);  %Mass

        %Vertices
        Bp = box.vertices.ds';

        %Run the Matlab simulation
        [MH_B_MATLAB,BV_MB_MATLAB] = BoxSimulator(MH_B_meas{1}(1:3,4),MH_B_meas{1}(1:3,1:3),BV_CBaf{1}(1:3,1),BV_CBaf{1}(4:6,1),MATLAB.(ObjStr).(Param)(1),MATLAB.(ObjStr).(Param)(2),MATLAB.(ObjStr).(Param)(3),box,MR_C,Mo_C,dt,endframe);
        MH_B_M(:,:,:,is) = cat(3,MH_B_MATLAB{:});
        BV_MB_M(:,:,is) = BV_MB_MATLAB;

        for ii = 1:endframe
            BCV_MB_M(:,ii,is) = [MR_C\MH_B_M(1:3,1:3,ii,is)*BV_MB_M(1:3,ii,is); MR_C\MH_B_M(1:3,1:3,ii,is)*BV_MB_M(4:6,ii,is)];
            for iv = 1:length(Bp)
                Cp_m(:,iv,ii)   = CH_B_meas{ii}(1:3,4)+CH_B_meas{ii}(1:3,1:3)*(Bp(:,iv));
                Cp_M(:,iv,ii,is) = MR_C\(MH_B_MATLAB{ii}(1:3,4) - Mo_C)+(MR_C)\MH_B_M(1:3,1:3,ii,is)*(Bp(:,iv));
                dCp_M(:,iv,ii,is) = [MR_C\MH_B_M(1:3,1:3,ii,is) -MR_C\MH_B_M(1:3,1:3,ii,is)*hat(Bp(:,iv))]*BV_MB_M(:,ii,is);
            end
        end

        %Compute the cost
        E_MATLAB(is)= norm(diag(weight)*(BCV_CBef(:,indx_c)-BCV_MB_M(:,indx_c,is)));
    end
end


%% -------------------------- Plot the figures --------------------------%%
close all;

%Create a plot grid
sizex = 380;
sizey = 250;
px = (0:7)*(sizex+10)+10;
py = (0:4)*(sizey+90)+45;
for  ii = 1:length(px)
    for jj = 1:length(py)
        pp{jj,ii} = [px(ii) py(jj)];
    end 
end 

%%
close all;

%Select the indices that indicate the time of pre- and post-impact velocity
is = 120; %is = 136; %Box006

Ia = impacts.indx(imp_sel(is),2) -5;
Ib = impacts.indx(imp_sel(is),3);
Ic = impacts.indx(imp_sel(is),4);
Id = impacts.indx(imp_sel(is),2) +5;

indx_b = Ib-Ia+1;
indx_c = Ic-Ia+1;

dCp_meas   = cell2mat(impacts.dCo_p(imp_sel(is),:));
BV_CB      = impacts.BV_CB(imp_sel(is),:);
BV_CBef    = impacts.BV_CBef(imp_sel(is),:);
BCV_CB     = cell2mat(impacts.BCV_CB(imp_sel(is),:));
BCV_CBaf   = cell2mat(impacts.BCV_CBaf(imp_sel(is),:));
BCV_CBef   = cell2mat(impacts.BCV_CBef(imp_sel(is),:));
V_impact   = impacts.indx(imp_sel(is),1);

%Some parameters
k = (-w_ext:w_ext)';        % Crude indices, comply with OptiTrack freq. 
kf = (-w_ext:0.01:w_ext)';  % Refined indices for fitting purposes
%Minimum velocity difference from steady state in percentage
Vper = 0.03; 

% Fit function and get indices
% Standard function: logistic function with gravity acceleration and initial speed
f = @(a,b,c,d,x)a./(exp(b*(c - x)) + 1) - g/freq*x - d;

% Fit function to velocity profile from OptiTrack
Cfit = fit(k,dCp_meas(3,:)',f,'StartPoint',[1.5,2,0,1]);

% Fitted vertical contact point velocity
F = feval(Cfit,kf);

% Minimum velocity difference from steady state to indicate impact
Vmin = Vper*max(F + g/freq*kf + Cfit.d);

% Find pre-impact index and project -w_ext:w_ext onto 1:2*w_ext+1
indx_bf = find(F + g/freq*kf + Cfit.d>Vmin, 1, 'first');
% indx_b  = round(kf(indx_bf)) +1+w_ext;

% Find post-impact index and project -w_ext:w_ext onto 1:2*w_ext+1
indx_cf = find(F + g/freq*kf + Cfit.d-Cfit.a>-Vmin, 1, 'first');
% indx_c  = round(kf(indx_cf)) +1+w_ext;

%plot contact point velocities
figure('rend','painters','pos',[pp{1,1} sizex sizey]);
    ha = tight_subplot(1,1,[.08 .07],[.18 .1],[0.12 0.03]);  %[gap_h gap_w] [lower upper] [left right]
    axes(ha(1));
    p1 = plot(k+impacts.indx(imp_sel(is),2),dCp_meas(3,:),'color',color.Meas,'LineWidth',1.5);
    grid; hold('On')
    if evalMatlab;  p2 = plot(k+impacts.indx(imp_sel(is),2),squeeze(dCp_M(3,V_impact,:,is)),'color',color.Matlab,'LineWidth',1.5); end
    p4 = plot(kf+impacts.indx(imp_sel(is),2),F,'--k','LineWidth',1);
    p5 = scatter(kf([indx_bf,indx_cf])+impacts.indx(imp_sel(is),2),F([indx_bf,indx_cf]),'xk','linewidth',1); %Plot fitted points in continuous function
    p6 = scatter([Ib Ic],dCp_meas(3,[indx_b,indx_c]),'r','linewidth',1); %Plot fitted points in discrete function
    if evalAlgoryx 
        p3 =    plot(k+impacts.indx(imp_sel(is),2),squeeze(dCp_AGX(3,V_impact,(1:11),optAGX_idx)),'color',color.Algoryx,'linewidth',1.5);
        legend([p1 p2 p3 p4 p5 p6],'$(^C\dot{\mathbf{p}}_1)_z$-Measured','$(^C\dot{\mathbf{p}}_1)_z$-Matlab','$(^C\dot{\mathbf{p}}_1)_z$-Algoryx','$(^C\dot{\mathbf{p}}_1)_z$-fit','Cont. indices','Disc. indices $b$, $c$','Location','NorthWest'); 
    else
        legend([p1 p2 p4 p5 p6],'$(^C\dot{\mathbf{p}}_1)_z$-Measured','$(^C\dot{\mathbf{p}}_1)_z$-Matlab','$(^C\dot{\mathbf{p}}_1)_z$-fit','Cont. indices','Disc. indices $b$, $c$','Location','NorthWest'); 
    end
    xlim([impacts.indx(imp_sel(is),2)-5 impacts.indx(imp_sel(is),2)+5]);
    xlabel('Time index $t_i$ [-]');
    ylabel('C.P. velocity ${^C\dot{\mathbf{p}}_1}$ [m/s]');
    ha(1).XTick = k+impacts.indx(imp_sel(is),2);
%     print(gcf,'Contact-point-vel.png','-dpng','-r500');
    if doSave
        fig = gcf;
        fig.PaperPositionMode = 'auto';
        fig_pos = fig.PaperPosition;
        fig.PaperSize = [fig_pos(3) fig_pos(4)];
        print(fig,'LaTeX/figures/CPVelocity_CentralEuler.pdf','-dpdf','-vector')
    end
% Paper figures

%plot measured contact point velocities
figure('rend','painters','pos',[pp{1,2} sizex 1.2*sizey]);
    ha = tight_subplot(1,1,[.12 .07],[.18 .23],[0.12 0.03]);  %[gap_h gap_w] [lower upper] [left right]
    axes(ha(1));
    p1 = plot(k,dCp_meas(1,:),'color',color.Red,'LineWidth',1.5);
    hold('On'); 
    p2 = plot(k,dCp_meas(2,:),'color',color.Green,'LineWidth',1.5);
    p3 = plot(k,dCp_meas(3,:),'color',color.Blue,'LineWidth',1.5);    
    p4 = plot(kf,F,'--k','LineWidth',1);
    p5 = scatter(kf([indx_bf,indx_cf]),F([indx_bf,indx_cf]),'r','linewidth',1); %Plot fitted points in continuous function
    p6 = scatter([indx_b,indx_c]-1-w_ext,dCp_meas(3,[indx_b,indx_c]),'xk');%,'linewidth',1); %Plot fitted points in discrete function
    xline(indx_b-1-w_ext,'-.','color','k');%,'linewidth',1.3)
    xline(indx_c-1-w_ext,'-.','color','k');%,'linewidth',1.3)
    xline(-5,'-.','color','k');%,'linewidth',1.3)
    xline(5,'-.','color','k');%,'linewidth',1.3)
    xline(kf(indx_bf),'-.','color','r');%,'linewidth',1.2)
    xline(kf(indx_cf),'-.','color','r');%,'linewidth',1.2)
    L1 = legend([p1 p2 p3 p4 p5 p6],'$(^C\dot{\mathbf{p}}_1)_x$','$(^C\dot{\mathbf{p}}_1)_y$','$(^C\dot{\mathbf{p}}_1)_z$','$(^C\dot{\mathbf{p}}_1)_{z,\textrm{fit}}$',...
        'Cont. indices','Disc. indices $-b$, $c$','NumColumns',3,'Location','NorthWest'); 
    L1.Position(2) = 0.88;
    L1.Position(1) = 0.545-(L1.Position(3)/2);
    xlim([-5 5]);
    xlabel('Normalized time around the impact time $t-t_{imp}$ [-]');
    ylabel('Contact point velocity ${^C\dot{\mathbf{p}}_1}$ [m/s]');
    ha(1).XTick = k;
    text(-5.4,0.62,'$-a$','FontSize',12);
    text(-2.4,0.62,'$-b$','FontSize',12);
    text(1.9,0.62,'$c$','FontSize',12);
    text(4.9,0.62,'$d$','FontSize',12);
%     print(gcf,'Contact-point-vel.png','-dpng','-r500');
    grid; 
    if doSave
        fig = gcf;
        fig.PaperPositionMode = 'auto';
        fig_pos = fig.PaperPosition;
        fig.PaperSize = [fig_pos(3) fig_pos(4)];
        print(fig,'figures/CPVelocity_CentralEuler.pdf','-dpdf','-vector')
    end

% Combined plot
    figure('rend','painters','pos',[pp{2,1} sizex 1.7*sizey]);
    ha = tight_subplot(2,1,[.08 .07],[.1 .01],[0.12 0.03]);  %[gap_h gap_w] [lower upper] [left right]
    axes(ha(1));
    hm = plot(Ia:Id,BCV_CB(1:3,:),'linewidth',1.2); grid on; hold on;                           %Measurement
    set(hm,{'color'},{color.Red; color.Green; color.Blue})
    p = plot(Ia:Ib,BCV_CBaf(1:3,1:indx_b)','--','color','k','LineWidth',1.2);
    plot(Ic:Id,BCV_CBef(1:3,indx_c:11)','--','color','k','LineWidth',1.2);
    xlabel('Time index $k$ [-]');
    ylabel('Linear velocity $^{B[C]}$\boldmath${v}_{C,B}$ [m/s]');
    ylim([min(min(BCV_CB(1:3,:)))-0.15 max(max(BCV_CB(1:3,:)))+0.15])
    ha(1).XTick = k+Ia+5;
    xlim([impacts.indx(imp_sel(is),2)-5 impacts.indx(imp_sel(is),2)+5]);
    xline(Ia,'-.')
    xline(Ib,'-.')
    xline(Ic,'-.')
    xline(Id,'-.')
    
    axes(ha(2));
    hm = plot(Ia:Id,BCV_CB(4:6,:),'LineWidth',1.2); grid on; hold on;                          %Measurement
    set(hm,{'color'},{color.Red; color.Green; color.Blue})
    plot(Ia:Ib,BCV_CBaf(4:6,1:indx_b)','--','color','k','LineWidth',1.2);
    plot(Ic:Id,BCV_CBef(4:6,indx_c:11)','--','color','k','LineWidth',1.2);
    xlabel('Time index $k$ [-]');
    ylabel('Angular velocity $^{B[C]}$\boldmath${\omega}_{C,B}$ [m/s]');
    ha(2).XTick = k+Ia+5;
    xlim([impacts.indx(imp_sel(is),2)-5 impacts.indx(imp_sel(is),2)+5]);
    ylim([min(min(BCV_CB(4:6,:)))-0.3 max(max(BCV_CB(4:6,:)))+0.3])
    xline(Ia,'-.')
    xline(Ib,'-.')
    xline(Ic,'-.')
    xline(Id,'-.')
  
    if doSave
        fig = gcf;
        fig.PaperPositionMode = 'auto';
        fig_pos = fig.PaperPosition;
        fig.PaperSize = [fig_pos(3) fig_pos(4)];
        print(fig,'figures/MeasuredVelocities.pdf','-dpdf','-vector')
    end


% Combined plot Matlab results
    figure('rend','painters','pos',[pp{2,2} sizex 1.7*sizey]);
    ha = tight_subplot(2,1,[.08 .07],[.1 .01],[0.12 0.03]);  %[gap_h gap_w] [lower upper] [left right]
    axes(ha(1));
    hm = plot(Ia:Id,BCV_CB(1:3,:),'linewidth',1); grid on; hold on;                           %Measurement
    hM = plot(Ia:Id,BCV_MB_M(1:3,1:11,is),'linewidth',1,'LineStyle','--');            %MATLAB data
    set(hm,{'color'},{color.RedLight; color.GreenLight; color.BlueLight})
    set(hM,{'color'},{color.Red; color.Green; color.Blue})
    p = plot(Ia:Ib,BCV_CBaf(1:3,1:indx_b)','--','color','k','LineWidth',1.2);
    plot(Ic:Id,BCV_CBef(1:3,indx_c:11)','--','color','k','LineWidth',1.2);
    ylim([min(min(BCV_CB(1:3,:)))-0.15 max(max(BCV_CB(1:3,:)))+0.15]);
    ha(1).XTick = Ia:Id;
    xlim([impacts.indx(imp_sel(is),2)-5 impacts.indx(imp_sel(is),2)+5]);
    xline(Ia,'-.')
    xline(Ib,'-.')
    xline(Ic,'-.')
    xline(Id,'-.')    
    xlabel('Time index $k$ [-]');
    ylabel('Linear velocity $^{B[C]}$\boldmath${v}_{C,B}$ [m/s]');
    
    
    axes(ha(2));
    hm = plot(Ia:Id,BCV_CB(4:6,:),'linewidth',1); grid on; hold on;                          %Measurement
    hM = plot(Ia:Id,BCV_MB_M(4:6,1:11,is),'linewidth',1,'LineStyle','--');           %MATLAB data
    set(hm,{'color'},{color.RedLight; color.GreenLight; color.BlueLight})
    set(hM,{'color'},{color.Red; color.Green; color.Blue})
    plot(Ia:Ib,BCV_CBaf(4:6,1:indx_b)','--','color','k','LineWidth',1.2);
    plot(Ic:Id,BCV_CBef(4:6,indx_c:11)','--','color','k','LineWidth',1.2);
    ylim([min(min(BCV_CB(4:6,:)))-0.3 max(max(BCV_CB(4:6,:)))+0.3])
    xlim([impacts.indx(imp_sel(is),2)-5 impacts.indx(imp_sel(is),2)+5]);
    xline(Ia,'-.')
    xline(Ib,'-.')
    xline(Ic,'-.')
    xline(Id,'-.')
    xlabel('Time index $k$ [-]');
    ylabel('Angular velocity $^{B[C]}$\boldmath${\omega}_{C,B}$ [m/s]');
    ha(2).XTick = Ia:Id;

    if doSave
        fig = gcf;
        fig.PaperPositionMode = 'auto';
        fig_pos = fig.PaperPosition;
        fig.PaperSize = [fig_pos(3) fig_pos(4)];
        print(fig,'figures/VelResMATLAB.pdf','-dpdf','-vector')
    end

% % Combined plot AGX Results
%     figure('rend','painters','pos',[pp{2,3} sizex 1.7*sizey]);
%     ha = tight_subplot(2,1,[.08 .07],[.1 .01],[0.12 0.03]);  %[gap_h gap_w] [lower upper] [left right]
%     axes(ha(1));
%     hm = plot(Ia:Id,BCV_CB(1:3,:),'linewidth',1); grid on; hold on;                           %Measurement
%     hA = plot(Ia:Id,BCV_MB_AGX(1:3,(1:11),is),'linewidth',1,'LineStyle','--');            %MATLAB data
%     set(hm,{'color'},{color.RedLight; color.GreenLight; color.BlueLight})
%     set(hA,{'color'},{color.Red; color.Green; color.Blue})
%     p = plot(Ia:Ib,BCV_CBaf(1:3,1:indx_b)','--','color','k','LineWidth',1.2);
%     plot(Ic:Id,BCV_CBef(1:3,indx_c:11)','--','color','k','LineWidth',1.2);
%     xlabel('Time index $k$ [-]');
%     ylabel('Linear velocity $^{B[C]}$\boldmath${v}_{C,B}$ [m/s]');
%     ylim([min(min(BCV_CB(1:3,:)))-0.15 max(max(BCV_CB(1:3,:)))+0.15]);
%     ha(1).XTick = Ia:Id;
%     xline(Ia,'-.')
%     xline(Ib,'-.')
%     xline(Ic,'-.')
%     xline(Id,'-.')
%     
%     axes(ha(2));
%     hm = plot(Ia:Id,BCV_CB(4:6,:),'linewidth',1); grid on; hold on;                          %Measurement
%     hA = plot(Ia:Id,BCV_MB_AGX(4:6,(1:11),is),'linewidth',1,'LineStyle','--');           %MATLAB data
%     set(hm,{'color'},{color.RedLight; color.GreenLight; color.BlueLight})
%     set(hA,{'color'},{color.Red; color.Green; color.Blue})
%     plot(Ia:Ib,BCV_CBaf(4:6,1:indx_b)','--','color','k','LineWidth',1.2);
%     plot(Ic:Id,BCV_CBef(4:6,indx_c:11)','--','color','k','LineWidth',1.2);
%     xlabel('Time index $k$ [-]');
%     ylabel('Angular velocity $^{B[C]}$\boldmath${\omega}_{C,B}$ [m/s]');
%     ylim([min(min(BCV_CB(4:6,:)))-0.3 max(max(BCV_CB(4:6,:)))+0.3])
%     ha(2).XTick = Ia:Id;
%     xline(Ia,'-.')
%     xline(Ib,'-.')
%     xline(Ic,'-.')
%     xline(Id,'-.')
%    
%     if doSave
%         fig = gcf;
%         fig.PaperPositionMode = 'auto';
%         fig_pos = fig.PaperPosition;
%         fig.PaperSize = [fig_pos(3) fig_pos(4)];
%         print(fig,'figures/VelResAGX.pdf','-dpdf','-vector')
%     end
%%
%Plot contact point trajectories over time
figure('rend','painters','pos',[pp{1,2} sizex sizey]);
    ha = tight_subplot(1,1,[.08 .07],[.18 .1],[0.15 0.03]);  %[gap_h gap_w] [lower upper] [left right]
    axes(ha(1));
    p1 = plot(k+impacts.indx(imp_sel(is),2),(squeeze(Cp_m(3,V_impact(1),:))),'color',color.Meas,'LineWidth',1.5);hold on;grid on;
    p2 = plot(k+impacts.indx(imp_sel(is),2),(squeeze(Cp_M(3,V_impact(1),:,optMATLAB_idx))),'color',color.Matlab,'LineWidth',1.5);
    if evalAlgoryx
       p3 = plot(k+impacts.indx(imp_sel(is),2),(squeeze(Cp_AGX(3,V_impact(1),(1:11),optAGX_idx))),'color',color.Algoryx,'LineWidth',1.5); 
       legend([p1 p2 p3], '$(^C{\mathbf{p}}_1)_z$-Measured','$(^C{\mathbf{p}}_1)_z$-Matlab','$(^C{\mathbf{p}}_1)_z$-Algoryx','Location','NorthEast'); 
    else
       legend([p1 p2], '$(^C{\mathbf{p}}_1)_z$-Measured','$(^C{\mathbf{p}}_1)_z$-Matlab','Location','NorthEast');
    end
    xlim([impacts.indx(imp_sel(is),2)-5 impacts.indx(imp_sel(is),2)+5]);
    xlabel('Time index $t_i$ [-]');
    ylabel('z-coordinate of C.P. position $({^C{\mathbf{p}}_1})_z$ [m]');
    ha(1).XTick = k+impacts.indx(imp_sel(is),2);
%     print(gcf,'Contact-point-pos.png','-dpng','-r500');
    if doSave
        fig = gcf;
        fig.PaperPositionMode = 'auto';
        fig_pos = fig.PaperPosition;
        fig.PaperSize = [fig_pos(3) fig_pos(4)];
        print(fig,'LaTeX/figures/CPposition_z.pdf','-dpdf','-vector')
    end


    

%% %Plot contact point trajectories in space
figure('rend','painters','pos',[pp{1,3} sizex sizey]);
    ha = tight_subplot(1,1,[.08 .07],[.18 .1],[0.15 0.03]);  %[gap_h gap_w] [lower upper] [left right]
    axes(ha(1));
    plot3((squeeze(Cp_m(1,V_impact(1),:))),(squeeze(Cp_m(2,V_impact(1),:))),(squeeze(Cp_m(3,V_impact(1),:))),'color',color.Meas); hold on;
    plot3((squeeze(Cp_M(1,V_impact(1),:,optMATLAB_idx))),(squeeze(Cp_M(2,V_impact(1),:,optMATLAB_idx))),(squeeze(Cp_M(3,V_impact(1),:,optMATLAB_idx))),'color',color.Matlab);
    plot3((squeeze(Cp_AGX(1,V_impact(1),1:11,optAGX_idx))),(squeeze(Cp_AGX(2,V_impact(1),1:11,optAGX_idx))),(squeeze(Cp_AGX(3,V_impact(1),1:11,optAGX_idx))),'color',color.Algoryx);
    axis equal; grid on;

% %Fitting of contact point velocity
% figure('rend','painters','pos',[pp{2,1} sizex sizey]);
%     ha = tight_subplot(1,1,[.08 .07],[.18 .1],[0.15 0.03]);  %[gap_h gap_w] [lower upper] [left right]
%     axes(ha(1));
%     vel1 = cat(2,BV_CBaf{:});
%     vel2 = cat(2,BV_CBef{:});
%     for ii = 1:11
%         Cp1(:,ii) = CH_B_meas{ii}(1:3,1:3)*vel1(1:3,ii) + CH_B_meas{ii}(1:3,1:3)*hat(vel1(4:6,ii))*(Bp(:,V_impact(1)));
%         Cp2(:,ii) = CH_B_meas{ii}(1:3,1:3)*vel2(1:3,ii) + CH_B_meas{ii}(1:3,1:3)*hat(vel2(4:6,ii))*(Bp(:,V_impact(1)));
%         %                         Cp1(:,ii) = BCV_CBaf(1:3,ii) - CH_Bimp{ii}(1:3,1:3)*hat(Bp(:,V_impact))*BCV_CBaf(4:6,ii);
%         %                         Cp2(:,ii) = BCV_CBef(1:3,ii) - CH_Bimp{ii}(1:3,1:3)*hat(Bp(:,V_impact))*BCV_CBef(4:6,ii);
%     end
%     plot(-5:5,dCp_meas); grid on; hold on;
%     plot(-5:Ib,Cp1(:,1:indx_b),'-.','color','k');
%     plot(Ic:5,Cp2(:,indx_c:11),'-.','color','k');
%     xlabel('Normalized time around the impact time $t-t_{imp}$ [-]');
%     ylabel('C.P. velocity ${^C\dot{\mathbf{p}}_i}$ [m/s]');
%     legend('$x$','$y$','$z$','Fitted','Location','NorthWest');
%     ha(1).XTick = k;
%     if doSave
%         fig = gcf;
%         fig.PaperPositionMode = 'auto';
%         fig_pos = fig.PaperPosition;
%         fig.PaperSize = [fig_pos(3) fig_pos(4)];
%         print(fig,'D:\OneDrive - TU Eindhoven\I.AM\12.TOSS\AGXvsMATLAB\LaTeX\figures/FitCp.pdf','-dpdf','-vector')
%     end

% %Contact point velocity
% figure('rend','painters','pos',[pp{2,2} 1.3*sizex sizey]);
%     ha = tight_subplot(1,1,[.08 .07],[.18 .1],[0.15 0.03]);  %[gap_h gap_w] [lower upper] [left right]
%     axes(ha(1));
%     plot(k,dCp_meas(1,:),'c',k,dCp_meas(2,:),'m',k,dCp_meas(3,:),'g');
%     grid; hold('On')
%     %     title('Contact point velocity')
%     plot(kf,F,'--k');
%     xline(kf(indx_bf),'-.','color','k','linewidth',1);
%     xline(indx_b-1-w_ext,'-.','color','r','linewidth',1);
%     xline(kf(indx_cf),'-.','color','k','linewidth',1);
%     xline(indx_c-1-w_ext,'-.','color','r','linewidth',1);
%     xline(-5,'-.','color','r','linewidth',1);
%     xline(5,'-.','color','r','linewidth',1);
%     xlabel('Normalized time around the impact time $t-t_{imp}$ [-]');
%     ylabel('C.P. velocity ${^C\dot{\mathbf{p}}_i}$ [m/s]');
%     scatter(kf([indx_bf,indx_cf]),F([indx_bf,indx_cf]),'xk','linewidth',1) %Plot fitted points in continuous function
%     scatter([indx_b,indx_c]-1-w_ext,dCp_meas(3,[indx_b,indx_c]),'r','linewidth',1) %Plot fitted points in discrete function
%     text(-5,1.3,'-a','FontSize',12);
%     text(indx_b-1-w_ext-0.1,1.3,'-b','FontSize',12);
%     text(indx_c-1-w_ext-0.1,1.3,'c','FontSize',12);
%     text(5-0.1,1.3,'d','FontSize',12);
%     legend('$(^C\dot{\mathbf{p}}_i)_x$','$(^C\dot{\mathbf{p}}_i)_y$',...
%         '$(^C\dot{\mathbf{p}}_i)_z$','$(^C\dot{\mathbf{p}}_i)_z$-fit',...
%         'Cont. indices', 'Disc. indices','Location','eastoutside')
%     xlim([-5.05 5.05]);
%     ha(1).XTick = k;
%     if doSave
%         fig = gcf;
%         fig.PaperPositionMode = 'auto';
%         fig_pos = fig.PaperPosition;
%         fig.PaperSize = [fig_pos(3) fig_pos(4)];
%         print(fig,'D:\OneDrive - TU Eindhoven\I.AM\12.TOSS\AGXvsMATLAB\LaTeX\figures/CPVelocity.pdf','-dpdf','-vector')
%     end
   
%%   
   
% Plotting options 
%For plotting the contact surface
ws    = 1;                 %With of the contact surface             [m]
ls    = 0.8;               %Length of the contact surface           [m]
surfacepoints = [0.5*ws -0.5*ws -0.5*ws 0.5*ws 0.5*ws; -0.5*ls -0.5*ls 0.5*ls 0.5*ls -0.5*ls; 0 0 0 0 0;];
spoints = MR_C*surfacepoints +Mo_C; %Transform the vertices according to position/orientation of the surface

%Plot the trajectory of the box
if doPlot
%Plot contact point trajectories over time
% figure('rend','painters','pos',[pp{2,3} sizex sizey]);
%     ha = tight_subplot(1,1,[.08 .07],[.18 .1],[0.15 0.03]);  %[gap_h gap_w] [lower upper] [left right]
%     axes(ha(1));
figure('pos',[pp{2,3} 319 150]);
    for ii=1:endframe
        
        %plot Measured box
        plotBox(MH_B_meas{ii},box,color.Meas,0);hold on;
        
        %Plot MATLAB box
        plotBox(MH_B_M(:,:,ii,optMATLAB_idx),box,color.Matlab,0); hold on;     

        %Plot new AGX box results
        plotBox(MH_B_AGX(:,:,ii,optAGX_idx),box,color.Algoryx,0);hold on;
                
        %Plot the conveyor C
        table3 = fill3(spoints(1,1:4),spoints(2,1:4),spoints(3,1:4),1);hold on;
        set(table3,'FaceColor',0.8*[1 1 1],'FaceAlpha',1);
        
        %Plot the origin of the contact surface with its unit vectors
        tip = [Mo_C+0.3*MR_C(:,1) Mo_C+0.3*MR_C(:,2) Mo_C+0.3*MR_C(:,3)];
        plot3([Mo_C(1) tip(1,1)],[Mo_C(2) tip(2,1)],[Mo_C(3) tip(3,1)],'r'); hold on
        plot3([Mo_C(1) tip(1,2)],[Mo_C(2) tip(2,2)],[Mo_C(3) tip(3,2)],'g');
        plot3([Mo_C(1) tip(1,3)],[Mo_C(2) tip(2,3)],[Mo_C(3) tip(3,3)],'b');
        
        %Plot the origin of the world coordinate frame
        tip = [0.3*[1;0;0] 0.3*[0;1;0] 0.3*[0;0;1]];
        plot3([0 tip(1,1)],[0 tip(2,1)],[0 tip(3,1)],'r'); hold on
        plot3([0 tip(1,2)],[0 tip(2,2)],[0 tip(3,2)],'g');
        plot3([0 tip(1,3)],[0 tip(2,3)],[0 tip(3,3)],'b');

        grid on;axis equal;
%         axis([-0.1 1.3 -0.5 0.3 -0.2 0.3]);
        axis([-0.2 0.4 -0.5 0.5 -0.2 0.3]);
        xlabel('x [m]');
        ylabel('y [m]');
        zlabel('z [m]');
%         legend('MATLAB','AGX','FD');
        view(0,0); %to see impact correctly
%         view(90,19);
text(1,0.2,0.2,append('Frame:',sprintf('%d',ii)));
        drawnow
        hold off
        pause()
        
%         f = gcf;
%         exportgraphics(f,sprintf('%d.jpg',ii),'Resolution',600)
        
%         doSave1 = false;
%         if doSave1
%             fig = gcf;
%             fig.PaperPositionMode = 'auto';
%             fig_pos = fig.PaperPosition;
%             fig.PaperSize = [fig_pos(3) fig_pos(4)];
%             print(fig,'D:\OneDrive - TU Eindhoven\I.AM\12.TOSS\AGXvsMATLAB\LaTeX\figures/LastPose','-dpdf','-painters')
%         end
    end
end
%%
close all
figure;
plot(meN,mmu,'+');
grid on;
xlabel('Coefficient of normal restitution $e_N$');
ylabel('Coefficient of friction $\mu$');
axis square
axis([0 1 0 1])



mu1 = 0:0.05:1;
eN1 = 0:0.05:1;
x1 = zeros(length(eN1),length(mu1)); %Create a grid of zeros
for ii=1:length(mmu)
        x1(single(mu1)==single(mmu(ii)),single(eN1)==single(meN(ii))) = x1(single(mu1)==single(mmu(ii)),single(eN1)==single(meN(ii))) + 1;
end

tel=0;
for ii=1:length(eN1)
    for jj = 1:length(mu1)        
        tel =tel+1;
        x(tel) = eN1(ii);
        y(tel) = mu1(jj);
        z(tel) = sum(mwi(single(mu1(jj))==single(mmu)&single(eN1(ii))==single(meN)));
    end
end

xvec = 0:0.001:1;
yvec = 0:0.001:1;
[xq,yq] = meshgrid(xvec, yvec);
vq = griddata(x,y,z,xq,yq,'v4');
vq(vq<0)=0;

figure
surf(xq,yq,vq,'EdgeColor','none');hold on;
plot3(x,y,z,'o');
view(0,90);
axis square

    

%%
ws    = 4;                 %With of the contact surface             [m]
ls    = 4;               %Length of the contact surface           [m]
surfacepoints = [0.5*ws -0.5*ws -0.5*ws 0.5*ws 0.5*ws; -0.5*ls -0.5*ls 0.5*ls 0.5*ls -0.5*ls; 0 0 0 0 0;];
spoints = MR_C*surfacepoints +Mo_C; %Transform the vertices according to position/orientation of the surface

figure('pos',[492,466,440,226]);
        ii=6   ;     
        %plot Measured box
        p1 = plotBox(MH_B_meas{ii},box,color.Meas);hold on;
        
        %Plot MATLAB box
        p2 = plotBox(MH_B_M(:,:,ii,optMATLAB_idx),box,color.Matlab); hold on;     

        %Plot new AGX box results
        p3 = plotBox(MH_B_AGX(:,:,ii,optAGX_idx),box,color.Algoryx);hold on;
                
        %Plot the conveyor C
        table3 = fill3(spoints(1,1:4),spoints(2,1:4),spoints(3,1:4),1);hold on;
        set(table3,'FaceColor',0.8*[1 1 1],'FaceAlpha',1);
        
        %Plot the origin of the world coordinate frame
        tip = [0.3*[1;0;0] 0.3*[0;1;0] 0.3*[0;0;1]];
        plot3([0 tip(1,1)],[0 tip(2,1)],[0 tip(3,1)],'r'); hold on
        plot3([0 tip(1,2)],[0 tip(2,2)],[0 tip(3,2)],'g');
        plot3([0 tip(1,3)],[0 tip(2,3)],[0 tip(3,3)],'b');

        grid on;axis equal;
        axis([-0.3 0.4 -0.1 0.6 -0.01 0.3]);
        xlabel('x [m]');
        ylabel('y [m]');
        zlabel('z [m]');
        view(-44,8); %to see impact correctly
        L1 = legend([p1 p2 p3],'Measured','Matlab','Algoryx','NumColumns',3,'location','east');
        L1.Position(2) = 0.90;
        L1.Position(1) = 0.52-(L1.Position(3)/2);
        text(0.662,0.391,0.268,append('Frame: ',sprintf('%d',ii)));
        drawnow      
       
        if doSave
            fig = gcf;
            fig.PaperPositionMode = 'auto';
            fig_pos = fig.PaperPosition;
            fig.PaperSize = [fig_pos(3) fig_pos(4)];
            print(fig,'D:\OneDrive - TU Eindhoven\I.AM\6.GIT\i-am-toss/PosPerspective','-dpdf','-vector')
        end      
