close all; set(groot,'defaulttextinterpreter','latex'); set(groot,'defaultAxesTickLabelInterpreter','latex'); set(groot,'defaultLegendInterpreter','latex');
%% -------------------------- Plot the figures --------------------------%%
%Create a plot grid
sizex = 380;
sizey = 250;
px = (0:11)*(190+10)+10;
py = (0:6)*(195+90)+45;
for  ii = 1:length(px)
    for jj = 1:length(py)
        pp{jj,ii} = [px(ii) py(jj)];
    end 
end 

J = customcolormap(linspace(0,1,22), fliplr({'#DCD9D0','#D2D1CA', '#C7C9C4','#BDC1BE','#B3B9B8', '#A9B1B2', '#9EAAAC', '#94A2A6', '#8A9AA0', '#7F929A', '#758A94', '#6B828F', '#617A89', '#567283', '#4C6A7D', '#426277', '#375B71', '#2D536B', '#234B65', '#19435F', '#0E3B59', '#043353'}));
J = customcolormap(linspace(0,1,11), fliplr({'#DCD9D0','#C7C9C4','#B3B9B8',  '#9EAAAC', '#8A9AA0',  '#758A94',  '#617A89',  '#4C6A7D',  '#375B71',  '#234B65',  '#0E3B59'}));
J = customcolormap(linspace(0,1,6), fliplr({'#DCD9D0','#B3B9B8', '#8A9AA0',   '#617A89',   '#375B71',   '#0E3B59'}));
J = customcolormap(linspace(0,1,3), fliplr({'#9E2A2B','#E7E0D0', '#043353'}));
%%
doSave = false;

color.Green = [0.4660 0.6740 0.1880];
color.Blue = [0 0.4470 0.7410];
color.Red = [0.8500 0.3250 0.0980]; 
color.GreenLight = [0.4660 0.6740 0.1880 0.5];
color.BlueLight = [0 0.4470 0.7410 0.5];
color.RedLight = [0.8500 0.3250 0.0980 0.5];
color.Meas = [4 51 83]/255;%[33 67 77]/255;%[85 122 149]/255;%[237 176 33]/255;
color.Algoryx = [220 217 208]/255;%[77 191 237]/255;
color.Bullet = [182, 174, 163]/255;%[77 191 237]/255; %[255 0 0]/255;%
color.Matlab = [0.4 0.4 0.4]; %[128 128 128]/255;
 
%% Plot the Vel based combined cost of Matlab (Box006)
set(groot,'defaultAxesFontSize',22)
% figure('rend','painters','units','centimeters','pos',[0.238125,31.3267,4.524375,3.96875]);
% ha = tight_subplot(1,1,[.08 .07],[.18 .1],[0.12 -0.05]);
figure('rend','painters','units','centimeters','pos',[0.238125,28,8.7,7.6315]);
    ha = tight_subplot(1,1,[.08 .07],[.18 .12],[0.1 -0.1]);  %[gap_h gap_w] [lower upper] [left right]
    axes(ha(1));
    s = surf(eN,mu,SUMMATLAB); axis square;
    xlabel('$e_N$','Position',[0.5 -0.1 -0.1]);
    ylabel('$\mu$','Position',[-0.1 0.5 -0.1]);
    zlabel('$\frac{1}{M}\sum_{i=1}^M L_{vel}(i;\mu,e_N)$','Position',[-0.84 0.5 0.47]);
    zlim([0 max([max(SUMMATLAB(:)) max(SUMAGX(:)) max(SUMBULLET(:))])]);
    xlim([0 1]);
    ylim([0 1]);
    view(-40,15);
    colormap(flipud(J));
%     if doSave; exportgraphics(gcf,'Plotting/paper_figures/CostMatlab.png','Resolution',1500); end
    if doSave;fig = gcf;fig.PaperPositionMode = 'auto';fig_pos = fig.PaperPosition; fig.PaperSize = [fig_pos(3) fig_pos(4)];
        print(fig,append('Plotting/paper_figures/CostMatlab.pdf'),'-dpdf','-vector'); end
    
%% Plot the Vel based combined cost of AGX (Box006)
% close Figure 2
% figure('rend','painters','pos',[pp{5,2} 0.45*sizex 0.6*sizey]);
%     ha = tight_subplot(1,1,[.08 .07],[.18 .1],[0.12 -0.05]);  %[gap_h gap_w] [lower upper] [left right]
figure('rend','painters','units','centimeters','pos',[9,28,7.5,7.6315]);
    ha = tight_subplot(1,1,[.08 .07],[.18 .1],[0.16 0.09]);  %[gap_h gap_w] [lower upper] [left right]
    axes(ha(1));
    surf(eN,mu,SUMAGX); axis square;
    xlabel('$e_N$','Position',[0.5 -0.1 -0.1]);
    ylabel('$\mu$','Position',[-0.1 0.5 -0.1]);
%     zlabel('$\frac{1}{M}\sum_{i=1}^M L_{vel}(i;\mu,e_N)$');
    zlim([0 max([max(SUMMATLAB(:)) max(SUMAGX(:)) max(SUMBULLET(:))])]);
    xlim([0 1]);
    ylim([0 1]);
    view(-40,15);
    colormap(flipud(J));;
    if doSave; fig = gcf; fig.PaperPositionMode = 'auto'; fig_pos = fig.PaperPosition; fig.PaperSize = [fig_pos(3) fig_pos(4)];
        print(fig,append('Plotting/paper_figures/CostAlgoryx.pdf'),'-dpdf','-vector'); end
    
%% Plot the Vel based combined cost of BULLET (Box006)
% figure('rend','painters','pos',[pp{5,3} 0.45*sizex 0.6*sizey]);
%     ha = tight_subplot(1,1,[.08 .07],[.18 .1],[0.12 -0.05]);  %[gap_h gap_w] [lower upper] [left right]
figure('rend','painters','units','centimeters','pos',[16.6,28,7.5,7.6315]);
    ha = tight_subplot(1,1,[.08 .07],[.18 .1],[0.16 0.09]);  %[gap_h gap_w] [lower upper] [left right]
    axes(ha(1));
    surf(eN,mu,SUMBULLET); axis square;
    xlabel('$e_N$','Position',[0.5 -0.1 -0.1]);
    ylabel('$\mu$','Position',[-0.1 0.5 -0.1]);
%     zlabel('$\frac{1}{M}\sum_{i=1}^M L_{vel}(i;\mu,e_N)$');
    zlim([0 max([max(SUMMATLAB(:)) max(SUMAGX(:)) max(SUMBULLET(:))])]);
    xlim([0 1]);
    ylim([0 1]);
    view(-40,15);
    colormap(flipud(J));;
    if doSave; fig = gcf; fig.PaperPositionMode = 'auto'; fig_pos = fig.PaperPosition; fig.PaperSize = [fig_pos(3) fig_pos(4)];
        print(fig,append('Plotting/paper_figures/CostBullet.pdf'),'-dpdf','-vector'); end
    
%% Plot the hybrid velocities from measurements around the impact time
figure('rend','painters','pos',[pp{4,7} sizex 1.7*sizey]);
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
    
    if doSave; fig = gcf; fig.PaperPositionMode = 'auto'; fig_pos = fig.PaperPosition; fig.PaperSize = [fig_pos(3) fig_pos(4)];
        print(fig,'Plotting/paper_figures/MeasuredVelocities.pdf','-dpdf','-vector'); end


%% Plot the hybrid velocities from MATLAB around the impact time
    figure('rend','painters','pos',[pp{4,9} sizex 1.7*sizey]);
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
    hm = plot(Ia:Id,BCV_CB(4:6,:),'linewidth',1); grid on; hold on;                       %Measurement
    hM = plot(Ia:Id,BCV_MB_M(4:6,1:11,optMATLAB_idx),'linewidth',1,'LineStyle','--');     %MATLAB data
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
    
    if doSave; fig = gcf; fig.PaperPositionMode = 'auto'; fig_pos = fig.PaperPosition; fig.PaperSize = [fig_pos(3) fig_pos(4)];
        print(fig,'Plotting/paper_figures/VelResMATLAB.pdf','-dpdf','-vector'); end

%% Plot the hybrid velocities from Algoryx around the impact time
figure('rend','painters','pos',[pp{4,11} sizex 1.7*sizey]);
    ha = tight_subplot(2,1,[.09 .05],[.08 .07],[0.12 0.06]);  %[gap_h gap_w] [lower upper] [left right]
    axes(ha(1));
    hm = plot(Ia:Id,BCV_CB(1:3,:),'linewidth',1); grid on; hold on;                      %Measurement
    hA = plot(Ia:Id,BCV_MB_AGX(1:3,(1:11),optAGX_idx),'linewidth',1,'LineStyle','--');   %Algoryx data
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
    hm = plot(Ia:Id,BCV_CB(4:6,:),'linewidth',1); grid on; hold on;                      %Measurement
    hA = plot(Ia:Id,BCV_MB_AGX(4:6,(1:11),optAGX_idx),'linewidth',1,'LineStyle','--');   %Algoryx data
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
   
    if doSave; fig = gcf; fig.PaperPositionMode = 'auto'; fig_pos = fig.PaperPosition; fig.PaperSize = [fig_pos(3) fig_pos(4)];
        print(fig,'Plotting/paper_figures/VelResAGX.pdf','-dpdf','-vector'); end


%% Impact selection
figure('rend','painters','pos',[pp{3,11} sizex 200]);
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
    if doSave; fig = gcf; fig.PaperPositionMode = 'auto'; fig_pos = fig.PaperPosition; fig.PaperSize = [fig_pos(3) fig_pos(4)];
        print(fig,'Plotting/paper_figures/impact_selection.pdf','-dpdf','-vector'); end


%% Plot the Traj based cost of MATLAB simulation (Box006)
% figure('rend','painters','pos',[pp{5,4} 0.45*sizex 0.6*sizey]);
%     ha = tight_subplot(1,1,[.08 .07],[.18 .1],[0.12 0.03]);  %[gap_h gap_w] [lower upper] [left right]
figure('rend','painters','units','centimeters','pos',[0.238125,28,9.3,7.6315]);
    ha = tight_subplot(1,1,[.08 .07],[.18 .12],[0.1 -0.1]);  %[gap_h gap_w] [lower upper] [left right]
    axes(ha(1));
    surf(eN,mu,SUMMATLABtraj); 
    axis square; 
    view(-40,15); 
    xlim([0 1]);
    ylim([0 1]);
    zlim([0 3]);
    clim([0.3 2.8]);
    xlabel('$e_N$','Position',[0.5 -0.1 -0.5]);
    ylabel('$\mu$','Position',[-0.1 0.5 -0.5]);
    zlabel('$\frac{1}{M}\sum_{i=1}^M L_{vel}(i;\mu,e_N)$','Position',[-0.9 0.5 2.1]);
    colormap(flipud(J));;
    if doSave; fig = gcf; fig.PaperPositionMode = 'auto'; fig_pos = fig.PaperPosition; fig.PaperSize = [fig_pos(3) fig_pos(4)];
    print(fig,append('Plotting/paper_figures/Traj_Based_Cost_Matlab'),'-dpdf','-vector'); end

%% Plot the Traj based cost of Algoryx simulation (Box006)
% figure('rend','painters','pos',[pp{5,5} 0.45*sizex 0.6*sizey]);
%     ha = tight_subplot(1,1,[.08 .07],[.18 .1],[0.12 0.03]);  %[gap_h gap_w] [lower upper] [left right]
figure('rend','painters','units','centimeters','pos',[9.6,28,7.0,7.6315]);
    ha = tight_subplot(1,1,[.08 .07],[.18 .1],[0.1 0.09]);  %[gap_h gap_w] [lower upper] [left right]
    axes(ha(1));
    surf(eN,mu,SUMAGXtraj); 
    axis square; 
    view(-40,15); 
    xlim([0 1]);
    ylim([0 1]);
    zlim([0 3]);
    clim([0.3 2.8]);
    xlabel('$e_N$','Position',[0.5 -0.1 -0.5]);
    ylabel('$\mu$','Position',[-0.1 0.5 -0.5]);
%     zlabel('$\frac{1}{M}\sum_{i=1}^M L_{vel}(i;\mu,e_N)$');
    colormap(flipud(J));;
    if doSave; fig = gcf; fig.PaperPositionMode = 'auto'; fig_pos = fig.PaperPosition; fig.PaperSize = [fig_pos(3) fig_pos(4)];
    print(fig,append('Plotting/paper_figures/Traj_Based_Cost_Algoryx'),'-dpdf','-vector'); end

%% Plot the Traj based cost of PyBullet simulation (Box006)
% figure('rend','painters','pos',[pp{5,6} 0.45*sizex 0.6*sizey]);
%     ha = tight_subplot(1,1,[.08 .07],[.18 .1],[0.12 0.03]);  %[gap_h gap_w] [lower upper] [left right]
figure('rend','painters','units','centimeters','pos',[16.7,28,7.0,7.6315]);
    ha = tight_subplot(1,1,[.08 .07],[.18 .1],[0.1 0.09]);  %[gap_h gap_w] [lower upper] [left right]
    axes(ha(1));
    surf(eN,mu,SUMBULLETtraj); 
    axis square; 
    view(-40,15); 
    xlim([0 1]);
    ylim([0 1]);
    zlim([0 3]);
    clim([0.3 2.8]);
    xlabel('$e_N$','Position',[0.5 -0.1 -0.5]);
    ylabel('$\mu$','Position',[-0.1 0.5 -0.5]);
%     zlabel('$\frac{1}{M}\sum_{i=1}^ML_{traj}(i;\mu,e_N)$');
    colormap(flipud(J));
    if doSave; fig = gcf; fig.PaperPositionMode = 'auto'; fig_pos = fig.PaperPosition; fig.PaperSize = [fig_pos(3) fig_pos(4)];
    print(fig,append('Plotting/paper_figures/Traj_Based_Cost_Bullet'),'-dpdf','-vector'); end

%% Plot measured trajectory in 3D for paper figure
% Plotting options For plotting the contact surface
ws    = 3.5;  %Width of the contact surface             [m]
ls    = 3.5;  %Length of the contact surface           [m]
surfacepoints = [0.5*ws -0.5*ws -0.5*ws 0.5*ws 0.5*ws; -0.5*ls -0.5*ls 0.5*ls 0.5*ls -0.5*ls; 0 0 0 0 0;];
FR_C = eye(3); 
Fo_C = zeros(3,1);
spoints = FR_C*surfacepoints +Fo_C; %Transform the vertices according to position/orientation of the surface
% 
% plotnr = 1;

%Plot the trajectory of the box
figure('pos',[pp{3,9} sizex 200]);
    ha = tight_subplot(1,1,[.08 .07],[.01 -.3],[0.03 0.03]);  %[gap_h gap_w] [lower upper] [left right]
    axes(ha(1));
    %Plot the conveyor C
    table3 = fill3(spoints(1,1:4),spoints(2,1:4),spoints(3,1:4),1);hold on;
    set(table3,'FaceColor',[220 220 220]/255,'FaceAlpha',1);
    
    %plot Measured box
    for ii=[id(plotnr,1) 409 427 445 463 481 499 535 id(plotnr,2)]  %id(plotnr,1):18:id(plotnr,1)+(id(plotnr,2)-id(plotnr,1))-1        
        if ii == id(plotnr,1)
            g1 = plotBox(MH_Bm(:,:,ii,plotnr),Boxes.Box006,[217 83 25]/255,0);hold on;
        elseif ii == id(plotnr,2)
            g1 = plotBox(MH_Bm(:,:,ii,plotnr),Boxes.Box006,[237 177 32]/255,0);hold on;
        else
            g1 = plotBox(MH_Bm(:,:,ii,plotnr),Boxes.Box006,[194 135 43]/255,0);hold on;            
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
    if doSave; f = gcf; exportgraphics(f,'Plotting/paper_figures/Measured_box_trajectory_2.png','Resolution',1500); end

%% Plot moment of release and moment of rest
% plotnr = 1;
figure('rend','painters','pos',[pp{3,7} 380 200]);
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
        print(fig,'Plotting/paper_figures/moment_of_release.pdf','-dpdf','-vector')
    end

%% Plot the results of the single Matlab + AGX simulation in smaller figure
close all;
for jj = 1:12
    if jj > 9
        ObjStr = 'Box006';
        Method = 'Traj';
    elseif jj > 6
        ObjStr = 'Box005';
        Method = 'Traj';
    elseif jj > 3
        ObjStr = 'Box006';
        Method = 'Vel';
    else
        ObjStr = 'Box005';
        Method = 'Vel';
    end

    plotidx = [1 12 25 1 6 91 1 12 25 1 6 91];

    if jj == 1 || jj == 2
        figure('rend','painters','pos',[pp{Val.idx(jj,1),Val.idx(jj,2)} 150 190]);
        ha = tight_subplot(1,1,[.08 .07],[.21 .005],[0.342 0.042]);  %[gap_h gap_w] [lower upper] [left right]
        axes(ha(1));
    elseif jj == 3
        figure('rend','painters','pos',[pp{Val.idx(jj,1),Val.idx(jj,2)} 180 190]);
        ha = tight_subplot(1,1,[.08 .07],[.25 .005],[0.342 0.042]);  %[gap_h gap_w] [lower upper] [left right]
        axes(ha(1));
    elseif jj == 4 || jj == 5 || jj == 7 || jj == 8 || jj == 10 || jj == 11 
        figure('rend','painters','pos',[pp{Val.idx(jj,1),Val.idx(jj,2)} 150 190]);
        ha = tight_subplot(1,1,[.08 .07],[.21 .005],[0.342 0.042]);  %[gap_h gap_w] [lower upper] [left right]
        axes(ha(1));
    elseif jj == 6 || jj == 9 || jj == 12
        figure('rend','painters','pos',[pp{Val.idx(jj,1),Val.idx(jj,2)} 150 190]);
        ha = tight_subplot(1,1,[.08 .07],[.21 .005],[0.342 0.042]);  %[gap_h gap_w] [lower upper] [left right]
        axes(ha(1));
    end

    Ptrans = Val.(append(ObjStr,Method)).MH_B_rest(:,:,plotidx(jj))*[Boxes.(ObjStr).vertices.ds';ones(1,8)];
    PtransM = Val.(append(ObjStr,Method)).MH_B_restM(:,:,plotidx(jj))*[Boxes.(ObjStr).vertices.ds';ones(1,8)];
    PtransA = Val.(append(ObjStr,Method)).MH_B_restAGX(:,:,plotidx(jj))*[Boxes.(ObjStr).vertices.ds';ones(1,8)];
    PtransB = Val.(append(ObjStr,Method)).MH_B_restB(:,:,plotidx(jj))*[Boxes.(ObjStr).vertices.ds';ones(1,8)];
    x1 = [Ptrans(1,1:4) Ptrans(1,1)];
    y1 = [Ptrans(2,1:4) Ptrans(2,1)];
    x2 = [PtransM(1,1:4) PtransM(1,1)];
    y2 = [PtransM(2,1:4) PtransM(2,1)];
    x3 = [PtransA(1,1:4) PtransA(1,1)];
    y3 = [PtransA(2,1:4) PtransA(2,1)];
    x4 = [PtransB(1,1:4) PtransB(1,1)];
    y4 = [PtransB(2,1:4) PtransB(2,1)];

    fill(x1,y1,color.Meas); %Measured box
    grid on; hold on;
    fill(x2,y2,color.Matlab);  %Matlab box
    fill(x3,y3,color.Algoryx); %AGX box
    fill(x4,y4,color.Bullet); %Bullet box
    axis equal
    axis([-0.1 0.6 -0.1 0.9]);

        
        
    if jj == 1 || jj == 2
        ylabel('$(^M\mathbf{o}_B)_y$');
    elseif jj == 3
        xlabel('$(^M\mathbf{o}_B)_x$');
        ylabel('$(^M\mathbf{o}_B)_y$');
    elseif jj == 4 || jj == 5 || jj == 7 || jj == 8 || jj == 10 || jj == 11 
        set(gca,'XTickLabel',[], 'YTickLabel', [])
    elseif jj == 6 || jj == 9 || jj == 12
        xlabel('$(^M\mathbf{o}_B)_x$');
    end

    fontsize(gca, 12, "points")

    if doSave; fig = gcf; fig.PaperPositionMode = 'auto'; fig_pos = fig.PaperPosition; fig.PaperSize = [fig_pos(3) fig_pos(4)];
        print(fig,append('figures/Thesis/',ObjStr,Method,'_Rest-Pose_',sprintf('%.2d.pdf',plotidx(jj))),'-dpdf','-vector'); end
    hold off;
end
%% Plot results of the multiple Matlab and AGX simulations (varying parameters)
for jj = 1:24
    if jj> 22
        sim = "Bullet";
    elseif jj > 20
        sim = "Algoryx";
    elseif jj > 18
        ObjStr = 'Box006';
        Param = 'eN';
        sim = 'Matlab';
    elseif jj > 16
        sim = "Bullet";
    elseif jj > 14
        sim = "Algoryx";
    elseif jj > 12
        ObjStr = 'Box006';
        Param = 'mu';
        sim = 'Matlab';
    elseif jj > 10
        sim = "Bullet";
    elseif jj > 8
        sim = "Algoryx";
    elseif jj > 6
        ObjStr = 'Box005';
        Param = 'eN';
        sim = 'Matlab';
    elseif jj > 4
        sim = "Bullet";
    elseif jj > 2
        sim = "Algoryx";
    else
        ObjStr = 'Box005';
        Param = 'mu';
        sim = 'Matlab';
    end

    plotidx = [1 12 1 12 1 12 1 12 1 12 1 12 1 6 1 6 1 6 1 6 1 6 1 6];
    
    Sens.idx(:,1) = [4;4;4;4;4;4;3;3;3;3;3;3;2;2;2;2;2;2;1;1;1;1;1;1];
    Sens.idx(:,2) = [1;4;2;5;3;6;1;4;2;5;3;6;1;4;2;5;3;6;1;4;2;5;3;6];

    figure('rend','painters','pos',[pp{Sens.idx(jj,1),Sens.idx(jj,2)} 150 190]);
    ha = tight_subplot(1,1,[.08 .07],[.16 .005],[0.21 0.05]);  %[gap_h gap_w] [lower upper] [left right]
    axes(ha(1));
    Ptrans = Sens.(append(ObjStr,Param)).MH_B_rest(:,:,plotidx(jj))*[Boxes.(ObjStr).vertices.ds';ones(1,8)];
    if sim == "Matlab"
        PtransS = Sens.(append(ObjStr,Param)).MH_B_restM_P(:,:,6,plotidx(jj))*[Boxes.(ObjStr).vertices.ds';ones(1,8)]; %6th is the mean
    elseif sim == "Algoryx"
        PtransS = Sens.(append(ObjStr,Param)).MH_B_restAGX_P(:,:,6,plotidx(jj))*[Boxes.(ObjStr).vertices.ds';ones(1,8)]; %6th is the mean
    elseif sim == "Bullet"
        PtransS = Sens.(append(ObjStr,Param)).MH_B_restB_P(:,:,6,plotidx(jj))*[Boxes.(ObjStr).vertices.ds';ones(1,8)]; %6th is the mean
    end
    x1 = [Ptrans(1,1:4) Ptrans(1,1)];
    y1 = [Ptrans(2,1:4) Ptrans(2,1)];
    x2 = [PtransS(1,1:4) PtransS(1,1)];
    y2 = [PtransS(2,1:4) PtransS(2,1)];

    %Plot result from the experiments
    fill(x1,y1,color.Meas); %Measured box
    axis equal; axis([-0.3 1 -0.7 0.3]); grid on; hold on
    if sim == "Matlab"
        fill(x2,y2,color.Matlab);      %Matlab box
    elseif sim == "Algoryx"
        fill(x2,y2,color.Algoryx);      %Algoryx box
    elseif sim == "Bullet"
        fill(x2,y2,color.Bullet);      %Algoryx box
    end
    

    %Plot the result from the sampled simulations
    for ib = 1:11
        if sim == "Matlab"
            PtransS(:,:,ib) = Sens.(append(ObjStr,Param)).MH_B_restM_P(:,:,ib,plotidx(jj))*[Boxes.(ObjStr).vertices.ds';ones(1,8)];
        elseif sim == "Algoryx"
            PtransS(:,:,ib) = Sens.(append(ObjStr,Param)).MH_B_restAGX_P(:,:,ib,plotidx(jj))*[Boxes.(ObjStr).vertices.ds';ones(1,8)];
        elseif sim == "Bullet"
            PtransS(:,:,ib) = Sens.(append(ObjStr,Param)).MH_B_restB_P(:,:,ib,plotidx(jj))*[Boxes.(ObjStr).vertices.ds';ones(1,8)];
        end
        
        x2 = [PtransS(1,1:4,ib) PtransS(1,1,ib)];
        y2 = [PtransS(2,1:4,ib) PtransS(2,1,ib)];

        
        if sim == "Matlab"
            h = fill(x2,y2,color.Matlab); %Matlab box
        elseif sim == "Algoryx"
            h = fill(x2,y2,color.Algoryx); %Matlab box
        elseif sim == "Bullet"
            h = fill(x2,y2,color.Bullet); %Matlab box
        end
        h.FaceAlpha = 0.1;
        h.EdgeColor = [0 0 0];
        h.EdgeAlpha = 0.4;
    end

    xlabel('$(^Mo_B)_x$');
    ylabel('$(^Mo_B)_y$');
    axis([-0.1 0.6 -0.1 0.9]);
    if doSave; fig = gcf; fig.PaperPositionMode = 'auto'; fig_pos = fig.PaperPosition; fig.PaperSize = [fig_pos(3) fig_pos(4)];
        print(fig,append('figures/RestPose/sensitivity/',ObjStr,'_',Param,'/',sim,'/Rest-Pose_',sprintf('%.2d.pdf',plotidx(jj))),'-dpdf','-vector'); end

    hold off;
    %     tel = tel+1;
end