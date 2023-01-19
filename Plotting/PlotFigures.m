close all; set(groot,'defaulttextinterpreter','latex'); set(groot,'defaultAxesTickLabelInterpreter','latex'); set(groot,'defaultLegendInterpreter','latex');
%% -------------------------- Plot the figures --------------------------%%
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

doSave = false;

lowlim = 0.05;
uplim = 0.8;

if evalAlgoryx && evalMatlab
    good = (MATLABmu_opt< uplim) & (MATLABmu_opt > lowlim) & (MATLABeN_opt < uplim) & (MATLABeN_opt > lowlim)...
    & (AGXmu_opt< uplim) & (AGXmu_opt > lowlim) & (AGXeN_opt < uplim) & (AGXeN_opt > lowlim);
elseif evalAlgoryx && ~evalMatlab
    good = (AGXmu_opt< uplim) & (AGXmu_opt > lowlim) & (AGXeN_opt < uplim) & (AGXeN_opt > lowlim);
elseif ~evalAlgoryx && evalMatlab
    good = (MATLABmu_opt< uplim) & (MATLABmu_opt > lowlim) & (MATLABeN_opt < uplim) & (MATLABeN_opt > lowlim);
end
if evalAlgoryx
    amu = AGXmu_opt(good);
    aeN = AGXeN_opt(good);
    awi = AGXwi(good);   %Take the good measurements
    awi = 1./awi;        %Flip the weights
    awi = awi./sum(awi); %Normalize the weights
    SUMAGX = (1/length(E_AGX(1,1,1,good)))*sum(E_AGX(:,:,:,good),4);
end
if evalMatlab
    mmu = MATLABmu_opt(good);
    meN = MATLABeN_opt(good);
    mwi = MATLABwi(good); %Take the good measurements
    mwi = 1./mwi;         %Flip the weights
    mwi = mwi./sum(mwi);  %Normalize the weights
    SUMMATLAB = (1/length(E_MATLAB(1,1,1,good)))*sum(E_MATLAB(:,:,:,good),4);
end

clear meanM_mu stdM_mu meanM_eN stdM_eN meanA_mu stdA_mu meanA_eN stdA_eN

%Randomly sample from data, without replacement:
[~,idx] = datasample(mmu,length(mmu),'Replace',false);

%Select the indices that indicate the time of pre- and post-impact velocity
is = 1; %is = 136; %Box006

Ia = impacts.indx(imp_sel(is),2) -5;
Ib = impacts.indx(imp_sel(is),3);
Ic = impacts.indx(imp_sel(is),4);
Id = impacts.indx(imp_sel(is),2) +5;

dCp_meas   = cell2mat(impacts.dCo_p(imp_sel(is),:));
BV_CB      = impacts.BV_CB(imp_sel(is),:);
BV_CBef    = impacts.BV_CBef(imp_sel(is),:);
BCV_CB     = cell2mat(impacts.BCV_CB(imp_sel(is),:));
BCV_CBaf   = cell2mat(impacts.BCV_CBaf(imp_sel(is),:));
BCV_CBef   = cell2mat(impacts.BCV_CBef(imp_sel(is),:));
V_impact   = impacts.indx(imp_sel(is),1);

%%
if evalMatlab  
    %Plot the combined cost of Matlab
    figure('rend','painters','pos',[pp{3,4} 0.45*sizex 0.6*sizey]);
    ha = tight_subplot(1,1,[.08 .07],[.18 .1],[0.12 -0.05]);  %[gap_h gap_w] [lower upper] [left right]
    axes(ha(1));
    surf(eN,mu,SUMMATLAB); axis square;
    xlabel('$e_N$');ylabel('$\mu$');zlabel('$\frac{1}{M}\sum_{i=1}^M L_{vel}(i;\mu,e_N)$');
    zlim([0 max([max(SUMMATLAB(:)) max(SUMAGX(:))])]);
    xlim([0 1]);
    ylim([0 1]);
    view(-40,15);
    
    if doSave;fig = gcf;fig.PaperPositionMode = 'auto';fig_pos = fig.PaperPosition; fig.PaperSize = [fig_pos(3) fig_pos(4)];
        print(fig,append('figures/CostMatlab.pdf'),'-dpdf','-vector'); end
end
if evalAlgoryx    
    % Plot the combined cost of AGX
    figure('rend','painters','pos',[pp{2,4} 0.45*sizex 0.6*sizey]);
    ha = tight_subplot(1,1,[.08 .07],[.18 .1],[0.12 -0.05]);  %[gap_h gap_w] [lower upper] [left right]
    axes(ha(1));
    surf(eN,mu,SUMAGX); axis square;
    xlabel('$e_N$');ylabel('$\mu$');zlabel('$\frac{1}{M}\sum_{i=1}^M L_{vel}(i;\mu,e_N)$');
    zlim([0 max([max(SUMMATLAB(:)) max(SUMAGX(:))])]);
    xlim([0 1]);
    ylim([0 1]);
    view(-40,15);
    if doSave; fig = gcf; fig.PaperPositionMode = 'auto'; fig_pos = fig.PaperPosition; fig.PaperSize = [fig_pos(3) fig_pos(4)];
        print(fig,append('figures/CostAlgoryx.pdf'),'-dpdf','-vector'); end
    
end
    

    %% Combined plot
    close all;
    figure('rend','painters','pos',[pp{2,1} sizex 1.7*sizey]);
    ha = tight_subplot(2,1,[.09 .05],[.08 .07],[0.12 0.06]);  %[gap_h gap_w] [lower upper] [left right]
    axes(ha(1));
    hm = plot(Ia:Id,BCV_CB(1:3,:),'linewidth',1.2); grid on; hold on;                           %Measurement
    set(hm,{'color'},{color.Red; color.Green; color.Blue})
    p = plot(Ia:Ib,BCV_CBaf(1:3,1:indx_b)','--','color','k','LineWidth',1.2);
    plot(Ic:Id,BCV_CBef(1:3,indx_c:11)','--','color','k','LineWidth',1.2);
    xlabel('Time index $k$ [-]');
    ylabel('Linear velocity $^{B[C]}$\boldmath${v}_{C,B}$ [m/s]');
    ylim([min(min(BCV_CB(1:3,:)))-0.15 max(max(BCV_CB(1:3,:)))+0.15])
    ha(1).XTick = k+Ia+5;
    set(gca,'Clipping','Off')
    plot([Ia Ia], [max(max(BCV_CB(1:3,:)))+0.19 max(max(BCV_CB(1:3,:)))+0.32],'-.','color',[0 0 0 0.4],'LineWidth',0.5)
    plot([Ib Ib], [max(max(BCV_CB(1:3,:)))+0.19 max(max(BCV_CB(1:3,:)))+0.32],'-.','color',[0 0 0 0.4],'LineWidth',0.5)
    plot([Ic Ic], [max(max(BCV_CB(1:3,:)))+0.19 max(max(BCV_CB(1:3,:)))+0.32],'-.','color',[0 0 0 0.4],'LineWidth',0.5)
    plot([Id Id], [max(max(BCV_CB(1:3,:)))+0.19 max(max(BCV_CB(1:3,:)))+0.32],'-.','color',[0 0 0 0.4],'LineWidth',0.5)
    plot([Ia+5 Ia+5], [max(max(BCV_CB(1:3,:)))+0.19 max(max(BCV_CB(1:3,:)))+0.32],'-.','color',[0 0 0 0.4],'LineWidth',0.5)
    
    xline(Ia,'-.');
    xline(Ib,'-.')
    xline(Ia+5,'-.')
    xline(Ic,'-.')
    xline(Id,'-.')
    text(Ia-0.7,max(max(BCV_CB(1:3,:)))+0.4,'$k_{im}-5$','FontSize',9);
    text(Ib-0.7,max(max(BCV_CB(1:3,:)))+0.4,'$k_{im}-b$','FontSize',9);
    text(Ia+5-0.25,max(max(BCV_CB(1:3,:)))+0.4,'$k_{im}$','FontSize',9);
    text(Ic-0.7,max(max(BCV_CB(1:3,:)))+0.4,'$k_{im}+c$','FontSize',9);
    text(Id-0.7,max(max(BCV_CB(1:3,:)))+0.4,'$k_{im}+5$','FontSize',9);
    
    axes(ha(2));
    hm = plot(Ia:Id,BCV_CB(4:6,:),'LineWidth',1.2); grid on; hold on;                          %Measurement
    set(hm,{'color'},{color.Red; color.Green; color.Blue})
    plot(Ia:Ib,BCV_CBaf(4:6,1:indx_b)','--','color','k','LineWidth',1.2);
    plot(Ic:Id,BCV_CBef(4:6,indx_c:11)','--','color','k','LineWidth',1.2);
    xlabel('Time index $k$ [-]');
    ylabel('Angular velocity $^{B[C]}$\boldmath${\omega}_{C,B}$ [m/s]');
    ha(2).XTick = k+Ia+5;
    ylim([min(min(BCV_CB(4:6,:)))-0.3 max(max(BCV_CB(4:6,:)))+0.3])
    xline(Ia,'-.')
    xline(Ib,'-.')
    xline(Ia+5,'-.')
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
    ha = tight_subplot(2,1,[.09 .05],[.08 .07],[0.12 0.06]);  %[gap_h gap_w] [lower upper] [left right]
    axes(ha(1));
    hm = plot(Ia:Id,BCV_CB(1:3,:),'linewidth',1); grid on; hold on;                           %Measurement
    hM = plot(Ia:Id,BCV_MB_M(1:3,1:11,optMATLAB_idx),'linewidth',1,'LineStyle','--');            %MATLAB data
    set(hm,{'color'},{color.RedLight; color.GreenLight; color.BlueLight})
    set(hM,{'color'},{color.Red; color.Green; color.Blue})
    p = plot(Ia:Ib,BCV_CBaf(1:3,1:indx_b)','--','color','k','LineWidth',1.2);
    plot(Ic:Id,BCV_CBef(1:3,indx_c:11)','--','color','k','LineWidth',1.2);
    ylim([min(min(BCV_CB(1:3,:)))-0.15 max(max(BCV_CB(1:3,:)))+0.15]);
    ha(1).XTick = Ia:Id;
    xlabel('Time index $k$ [-]');
    ylabel('Linear velocity $^{B[C]}$\boldmath${v}_{C,B}$ [m/s]');
    set(gca,'Clipping','Off')
    plot([Ia Ia], [max(max(BCV_CB(1:3,:)))+0.19 max(max(BCV_CB(1:3,:)))+0.32],'-.','color',[0 0 0 0.4],'LineWidth',0.5)
    plot([Ib Ib], [max(max(BCV_CB(1:3,:)))+0.19 max(max(BCV_CB(1:3,:)))+0.32],'-.','color',[0 0 0 0.4],'LineWidth',0.5)
    plot([Ic Ic], [max(max(BCV_CB(1:3,:)))+0.19 max(max(BCV_CB(1:3,:)))+0.32],'-.','color',[0 0 0 0.4],'LineWidth',0.5)
    plot([Id Id], [max(max(BCV_CB(1:3,:)))+0.19 max(max(BCV_CB(1:3,:)))+0.32],'-.','color',[0 0 0 0.4],'LineWidth',0.5)
    plot([Ia+5 Ia+5], [max(max(BCV_CB(1:3,:)))+0.19 max(max(BCV_CB(1:3,:)))+0.32],'-.','color',[0 0 0 0.4],'LineWidth',0.5)
    
    xline(Ia,'-.');
    xline(Ib,'-.')
    xline(Ia+5,'-.')
    xline(Ic,'-.')
    xline(Id,'-.')
    text(Ia-0.7,max(max(BCV_CB(1:3,:)))+0.4,'$k_{im}-5$','FontSize',9);
    text(Ib-0.7,max(max(BCV_CB(1:3,:)))+0.4,'$k_{im}-b$','FontSize',9);
    text(Ia+5-0.25,max(max(BCV_CB(1:3,:)))+0.4,'$k_{im}$','FontSize',9);
    text(Ic-0.7,max(max(BCV_CB(1:3,:)))+0.4,'$k_{im}+c$','FontSize',9);
    text(Id-0.7,max(max(BCV_CB(1:3,:)))+0.4,'$k_{im}+5$','FontSize',9);
    
    
    axes(ha(2));
    hm = plot(Ia:Id,BCV_CB(4:6,:),'linewidth',1); grid on; hold on;                          %Measurement
    hM = plot(Ia:Id,BCV_MB_M(4:6,1:11,optMATLAB_idx),'linewidth',1,'LineStyle','--');           %MATLAB data
    set(hm,{'color'},{color.RedLight; color.GreenLight; color.BlueLight})
    set(hM,{'color'},{color.Red; color.Green; color.Blue})
    plot(Ia:Ib,BCV_CBaf(4:6,1:indx_b)','--','color','k','LineWidth',1.2);
    plot(Ic:Id,BCV_CBef(4:6,indx_c:11)','--','color','k','LineWidth',1.2);
    ylim([min(min(BCV_CB(4:6,:)))-0.3 max(max(BCV_CB(4:6,:)))+0.3])
    xline(Ia,'-.')
    xline(Ib,'-.')
    xline(Ia+5,'-.')
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

% Combined plot AGX Results
    figure('rend','painters','pos',[pp{2,3} sizex 1.7*sizey]);
    ha = tight_subplot(2,1,[.09 .05],[.08 .07],[0.12 0.06]);  %[gap_h gap_w] [lower upper] [left right]
    axes(ha(1));
    hm = plot(Ia:Id,BCV_CB(1:3,:),'linewidth',1); grid on; hold on;                           %Measurement
    hA = plot(Ia:Id,BCV_MB_AGX(1:3,(1:11),optAGX_idx),'linewidth',1,'LineStyle','--');            %MATLAB data
    set(hm,{'color'},{color.RedLight; color.GreenLight; color.BlueLight})
    set(hA,{'color'},{color.Red; color.Green; color.Blue})
    p = plot(Ia:Ib,BCV_CBaf(1:3,1:indx_b)','--','color','k','LineWidth',1.2);
    plot(Ic:Id,BCV_CBef(1:3,indx_c:11)','--','color','k','LineWidth',1.2);
    xlabel('Time index $k$ [-]');
    ylabel('Linear velocity $^{B[C]}$\boldmath${v}_{C,B}$ [m/s]');
    ylim([min(min(BCV_CB(1:3,:)))-0.15 max(max(BCV_CB(1:3,:)))+0.15]);
    ha(1).XTick = Ia:Id;
    set(gca,'Clipping','Off')
    plot([Ia Ia], [max(max(BCV_CB(1:3,:)))+0.19 max(max(BCV_CB(1:3,:)))+0.32],'-.','color',[0 0 0 0.4],'LineWidth',0.5)
    plot([Ib Ib], [max(max(BCV_CB(1:3,:)))+0.19 max(max(BCV_CB(1:3,:)))+0.32],'-.','color',[0 0 0 0.4],'LineWidth',0.5)
    plot([Ic Ic], [max(max(BCV_CB(1:3,:)))+0.19 max(max(BCV_CB(1:3,:)))+0.32],'-.','color',[0 0 0 0.4],'LineWidth',0.5)
    plot([Id Id], [max(max(BCV_CB(1:3,:)))+0.19 max(max(BCV_CB(1:3,:)))+0.32],'-.','color',[0 0 0 0.4],'LineWidth',0.5)
    plot([Ia+5 Ia+5], [max(max(BCV_CB(1:3,:)))+0.19 max(max(BCV_CB(1:3,:)))+0.32],'-.','color',[0 0 0 0.4],'LineWidth',0.5)
    
    xline(Ia,'-.');
    xline(Ib,'-.')
    xline(Ia+5,'-.')
    xline(Ic,'-.')
    xline(Id,'-.')
    text(Ia-0.7,max(max(BCV_CB(1:3,:)))+0.4,'$k_{im}-5$','FontSize',9);
    text(Ib-0.7,max(max(BCV_CB(1:3,:)))+0.4,'$k_{im}-b$','FontSize',9);
    text(Ia+5-0.25,max(max(BCV_CB(1:3,:)))+0.4,'$k_{im}$','FontSize',9);
    text(Ic-0.7,max(max(BCV_CB(1:3,:)))+0.4,'$k_{im}+c$','FontSize',9);
    text(Id-0.7,max(max(BCV_CB(1:3,:)))+0.4,'$k_{im}+5$','FontSize',9);
    
    axes(ha(2));
    hm = plot(Ia:Id,BCV_CB(4:6,:),'linewidth',1); grid on; hold on;                          %Measurement
    hA = plot(Ia:Id,BCV_MB_AGX(4:6,(1:11),optAGX_idx),'linewidth',1,'LineStyle','--');           %MATLAB data
    set(hm,{'color'},{color.RedLight; color.GreenLight; color.BlueLight})
    set(hA,{'color'},{color.Red; color.Green; color.Blue})
    plot(Ia:Ib,BCV_CBaf(4:6,1:indx_b)','--','color','k','LineWidth',1.2);
    plot(Ic:Id,BCV_CBef(4:6,indx_c:11)','--','color','k','LineWidth',1.2);
    xlabel('Time index $k$ [-]');
    ylabel('Angular velocity $^{B[C]}$\boldmath${\omega}_{C,B}$ [m/s]');
    ylim([min(min(BCV_CB(4:6,:)))-0.3 max(max(BCV_CB(4:6,:)))+0.3])
    ha(2).XTick = Ia:Id;
    xline(Ia,'-.')
    xline(Ib,'-.')
    xline(Ia+5,'-.')
    xline(Ic,'-.')
    xline(Id,'-.')
   
    if doSave
        fig = gcf;
        fig.PaperPositionMode = 'auto';
        fig_pos = fig.PaperPosition;
        fig.PaperSize = [fig_pos(3) fig_pos(4)];
        print(fig,'figures/VelResAGX.pdf','-dpdf','-vector')
    end

%% Impact detection
figure('rend','painters','pos',[pp{1,1} 380 200]);
    ha = tight_subplot(1,1,[.08 .07],[.17 .03],[0.11 0.03]);  %[gap_h gap_w] [lower upper] [left right]
    axes(ha(1));
    vertex=find(plocs==ii_imp);
    vrest = 1:8~=vertex;
    p1 = plot(Cp_z(vrest,:)','color',[0 0 0 0.2]); hold on; grid on;
    p2 = plot(Cp_z(vertex,:)','color',[0 0.4470 0.7410]);
    p3 = plot(plocs(vertex),Cp_z(vertex,plocs(vertex)),'o','color',[0 0.4470 0.7410],'LineWidth',1.4,'MarkerSize',7);
    axis([240 450 -0.02 0.3]);
    xlabel('Time index $k$ [-]');
    ylabel('$(^C\mathbf{p}_i(k))_z$ [m]')
%             L1 = legend([p2 p3 p1(1)],'Trajectory of $(^C\mathbf{p}_1)_z$','Found impact time $t_{imp}$ = 275','Trajectory of $(^C\mathbf{p}_{i\neq1})_z$','location','northeast');
    if doSave
        fig = gcf;
        fig.PaperPositionMode = 'auto';
        fig_pos = fig.PaperPosition;
        fig.PaperSize = [fig_pos(3) fig_pos(4)];
        print(fig,'figures/impact_selection.pdf','-dpdf','-vector')
    end

%% Trajectory based costs
%Plot the cost of MATLAB simulation
figure('rend','painters','pos',[pp{3,5} 0.45*sizex 0.6*sizey]);
    ha = tight_subplot(1,1,[.08 .07],[.18 .1],[0.12 0.03]);  %[gap_h gap_w] [lower upper] [left right]
    axes(ha(1));
    surf(eN,mu,SUMMATLABtraj); 
    axis square; 
    view(-40,15); 
    xlim([0 1]);
    ylim([0 1]);
    zlim([0 3]);
    clim([0.3 2.8]);
    xlabel('$e_N$');
    ylabel('$\mu$');
    zlabel('$\frac{1}{M}\sum_{i=1}^ML_{traj}(i;\mu,e_N)$');
    

    if doSave; fig = gcf; fig.PaperPositionMode = 'auto'; fig_pos = fig.PaperPosition; fig.PaperSize = [fig_pos(3) fig_pos(4)];
    print(fig,append('figures/Traj_Based_Cost_Matlab'),'-dpdf','-vector'); end

%Plot the cost of Algoryx simulation
figure('rend','painters','pos',[pp{2,5} 0.45*sizex 0.6*sizey]);
    ha = tight_subplot(1,1,[.08 .07],[.18 .1],[0.12 0.03]);  %[gap_h gap_w] [lower upper] [left right]
    axes(ha(1));
    surf(eN,mu,SUMAGXtraj); 
    axis square; 
    view(-40,15); 
    xlim([0 1]);
    ylim([0 1]);
    zlim([0 3]);
    clim([0.3 2.8]);
    xlabel('$e_N$');
    ylabel('$\mu$');
    zlabel('$\frac{1}{M}\sum_{i=1}^ML_{traj}(i;\mu,e_N)$');

    if doSave; fig = gcf; fig.PaperPositionMode = 'auto'; fig_pos = fig.PaperPosition; fig.PaperSize = [fig_pos(3) fig_pos(4)];
    print(fig,append('figures/Traj_Based_Cost_Algoryx'),'-dpdf','-vector'); end

%% Plot measured trajectory for paper figure
% Plotting options For plotting the contact surface
ws    = 3.5;  %Width of the contact surface             [m]
ls    = 3.5;  %Length of the contact surface           [m]
surfacepoints = [0.5*ws -0.5*ws -0.5*ws 0.5*ws 0.5*ws; -0.5*ls -0.5*ls 0.5*ls 0.5*ls -0.5*ls; 0 0 0 0 0;];
FR_C = eye(3); 
Fo_C = zeros(3,1);
spoints = FR_C*surfacepoints +Fo_C; %Transform the vertices according to position/orientation of the surface

plotnr = 1;
%Plot the trajectory of the box
figure('pos',[pp{1,2} 400 200]);
    ha = tight_subplot(1,1,[.08 .07],[.01 -.3],[0.03 0.03]);  %[gap_h gap_w] [lower upper] [left right]
    axes(ha(1));
    %Plot the conveyor C
    table3 = fill3(spoints(1,1:4),spoints(2,1:4),spoints(3,1:4),1);hold on;
    set(table3,'FaceColor',[220 220 220]/255,'FaceAlpha',1);
    
    %plot Measured box
    for ii=[id(plotnr,1) 409 427 445 463 481 499 535 id(plotnr,2)]  %id(plotnr,1):18:id(plotnr,1)+(id(plotnr,2)-id(plotnr,1))-1        
        if ii == id(plotnr,1)
            g1 = plotBox(MH_Bm(:,:,ii,plotnr),Box,[217 83 25]/255,0);hold on;
        elseif ii == id(plotnr,2)
            g1 = plotBox(MH_Bm(:,:,ii,plotnr),Box,[237 177 32]/255,0);hold on;
        else
            g1 = plotBox(MH_Bm(:,:,ii,plotnr),Box,[194 135 43]/255,0);hold on;            
        end
        drawnow
    end   

    %Other plot options
    axis equal;
    axis([-0.4 0.6 -0.7 0.4 -0.05 0.3]);
%     view(-118,27);
    view(90,2)
    camproj('perspective')
    axis off;

    if doSave
        f = gcf;
        exportgraphics(f,'figures/Measured_box_trajectory_2.png','Resolution',1500);
    end
%% Plot figure to demonstrate the release and rest determination
plotnr = 1;
figure('rend','painters','pos',[pp{1,3} 380 200]);
ha = tight_subplot(1,1,[.08 .07],[.17 .01],[0.13 0.03]);  %[gap_h gap_w] [lower upper] [left right]
axes(ha(1));
    plot(((id(plotnr,1)-20):id(plotnr,2)+20),Mo_B(3,(id(plotnr,1)-20):id(plotnr,2)+20,plotnr)); hold on; 
    plot(id(plotnr,1),Mo_B(3,id(plotnr,1),plotnr),'o','markersize',7,'linewidth',1.4);
    plot(id(plotnr,2),Mo_B(3,id(plotnr,2),plotnr),'o','markersize',7,'linewidth',1.4);
    grid on;
    xlim([id(plotnr,1)-20,id(plotnr,2)+20]);
    xlabel('Time index $k$ [-]');
    ylabel('$(^M\mathbf{o}_B)_z$ [m]');
    ylim([0.04 0.16]);
%     X = [0.74 0.89];
%     Y = [0.47 0.27];
%     annotation('arrow',X,Y);
%     text(1.5,0.095,'Moment of rest','Fontsize',12);
%     X = [0.36 0.19];
%     Y = [0.81 0.90];
%     annotation('arrow',X,Y);
%     text(1.26,0.138,'Moment of release','Fontsize',12);
    f = gcf;
%     print(gcf,'Rest-Release.png','-dpng','-r500'); %Uncomment if you want to save this image
    if doSave
        fig = gcf;
        fig.PaperPositionMode = 'auto';
        fig_pos = fig.PaperPosition;
        fig.PaperSize = [fig_pos(3) fig_pos(4)];
        print(fig,'figures/moment_of_release.pdf','-dpdf','-vector')
    end