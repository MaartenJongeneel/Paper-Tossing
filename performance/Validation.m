clearvars; close all; set(groot,'defaulttextinterpreter','latex'); set(groot,'defaultAxesTickLabelInterpreter','latex'); set(groot,'defaultLegendInterpreter','latex');
addpath(genpath('readH5')); addpath('data');
%% Evaluate the manual tosses
%Goal is to see if for the manual tosses, the MATLAB and AGX simulators are
%able to predict the rest pose accurately, as we saw that this was not
%really the case for the automatic tosses by the robot arm, which might
%have been from too much height. In the performed experiments, box5 is
%tossed manually on an idle conveyor.
%% Load the data
% data = readH5('220920_Box006_Validation.h5'); %Validation of Box006
data = readH5('220920_Box005_Validation.h5'); %Validation of Box005
%% Constants
th_Rmean = 1e-5; %Threshold rotation mean
color.Matlab = [237 176 33]/255;
color.Algoryx = [77 191 237]/255;
color.Meas = [128 128 128]/255;
N_pos    = 20; %Number of consecutive points where the error is low
doSave   = false;
MATLAB.Box005.vel   = [0.35 0.00 0.45]; %eN eT mu
MATLAB.Box005.traj  = [0.10 0.00 0.45]; %eN eT mu
MATLAB.Box006.vel   = [0.45 0.00 0.30]; %eN eT mu 
MATLAB.Box006.traj  = [0.25 0.00 0.40]; %eN eT mu
Algoryx.Box005.vel  = [0.30 0.00 0.40]; %eN eT mu
Algoryx.Box005.traj = []; %eN eT mu
Algoryx.Box006.vel  = [0.40 0.00 0.25]; %eN eT mu 
Algoryx.Box006.traj = []; %eN eT mu


MATLAB_eN_sigma = 0.125; %Covariance of eN parameter set (not covariance of mean!)
MATLAB_mu_sigma = 0.124; %Covariance of mu parameter set (not covariance of mean!)
Algoryx_eN_sigma = 0.127; %Covariance of eN parameter set (not covariance of mean!)
Algoryx_mu_sigma = 0.143; %Covariance of mu parameter set (not covariance of mean!)


ObjStr = "Box005"; %The object for which you want to do paramID
ImpPln = "GroundPlane001";
Param = "vel";  %Trajectory based parameters are tested 
%% Loop through the data
tel = 0;
fn = fieldnames(data);
for ii = 1:length(fn)
    if startsWith(fn{ii},'Rec')
        tel = tel+1;

        %Get the data from the file
        Mocap = data.(fn{ii}).SENSOR_MEASUREMENT.Mocap;
        dt   = 1/double(Mocap.datalog.attr.sample_frequency);   %Timestep of the recording 
        
        %Get the object info
        Box = data.(fn{ii}).OBJECT.(ObjStr);

        %Get the tranforms of the object
        MH_B = Mocap.POSTPROCESSING.(ObjStr).transforms.ds; 
        FH_C = Mocap.POSTPROCESSING.(ImpPln).transforms.ds;

        %Rewrite the data into mat structures
        Nsamples = length(MH_B);
        for jj = 1:Nsamples
            FH_Cm(:,:,jj,tel) = FH_C{jj};
            MH_Bm(:,:,jj,tel) = MH_B{jj};
        end
        
        %Few definitions from data:
        Fo_C(:,1:length(FH_Cm(1:3,4,:,tel)),tel) = squeeze(FH_Cm(1:3,4,:,tel));
        Mo_B(:,1:length(MH_Bm(1:3,4,:,tel)),tel) = squeeze(MH_Bm(1:3,4,:,tel));

        
        %-------------------- Compute the velocities --------------------%
        %Compute angular and linear velocity of B w.r.t. M
        for ij = 2:Nsamples-1
            %Compute dMo_B using central difference approximation
            dMo_B(:,ij) = 1/(2*dt)*(MH_Bm(1:3,4,ij+1,tel)-MH_Bm(1:3,4,ij-1,tel));
            
            %Left trivialized velocity w.r.t F
            Bomg_MB(:,ij) = vee(1/(2*dt)*((logm(MH_Bm(1:3,1:3,ij,tel)\(MH_Bm(1:3,1:3,ij+1,tel))))-(logm(MH_Bm(1:3,1:3,ij,tel)\(MH_Bm(1:3,1:3,ij-1,tel))))));
            Bv_MB(:,ij)   = MH_Bm(1:3,1:3,ij,tel)' * dMo_B(:,ij);
            
            %Hybrid velocity of frame B w.r.t. frame F
            BMV_MB(:,ij,tel)  = [MH_Bm(1:3,1:3,ij,tel) zeros(3,3); zeros(3,3) MH_Bm(1:3,1:3,ij,tel)] * [Bv_MB(:,ij); Bomg_MB(:,ij)];
        end
        BV_MB(:,1:length(Bv_MB),tel) = [Bv_MB;Bomg_MB];

            
        %--------------- Determine the moment of release ----------------%
        %Find the peaks of the position data (height) to find when the box is released from the hand
            t = find(vecnorm(dMo_B(:,150:end))<0.02); %Find the indices where difference in rel. pos. is small
            x = diff(t)==1;
            f = find([false,x]~=[x,false]);
            g = find(f(2:2:end)-f(1:2:end-1)>=N_pos,1,'first');
            id_rest = t(f(2*g-1))+149; % First t followed by >=N_pos consecutive numbers
        if ObjStr == "Box005"
            [pks,id_rel] = findpeaks(Mo_B(3,:,tel),'MinPeakHeight',0.12);%,'MinPeakProminence',0.05,'MinPeakWidth',10);
            id_rel = id_rel(end);
        elseif ObjStr == "Box006"
            if isempty(id_rest); id_rest = 700; end %If due to noise we cannot get the rest index 
            [pks,id_rel] = findpeaks(Mo_B(3,100:end,tel),'MinPeakHeight',0.08,'MinPeakWidth',10);
            id_rel = id_rel+99;            
            id_rel = id_rel(1);
        end

%             figure; plot(Mo_B(3,:,tel)); hold on;
%                 plot(id_rel,Mo_B(3,id_rel,tel),'o','markersize',10,'linewidth',2);
%                 plot(id_rest,Mo_B(3,id_rest,tel),'o','markersize',10,'linewidth',2);
%                 grid on;
%                 pause
%                 close all
        
        %------------- Determine the relative release-pose --------------%
        Mo_B_rel(:,tel) = Mo_B(:,id_rel,tel);
        MR_B_rel(:,:,tel) = MH_Bm(1:3,1:3,id_rel,tel);
        MH_B_rel(:,:,tel) = [MR_B_rel(:,:,tel),Mo_B_rel(:,tel);zeros(1,3),1];
        
        %----------- Determine the release (hybrid) velocity ------------%
        BMV_MB_rel(:,tel) = BMV_MB(:,id_rel,tel);
        BV_MB_rel(:,tel)  = BV_MB(:,id_rel,tel);
        
                
        %----------- Determine the average relative rest-pose -----------%
        Mo_B_rest(:,tel) = mean(Mo_B(:,id_rest:id_rest+100,tel),2);
        MR_Bmat_rest = MH_Bm(1:3,1:3,id_rest:id_rest+100,tel);
        Rmean = MR_Bmat_rest(:,:,1);
        for zz = 1:length(MR_Bmat_rest)
            rpoints(:,zz) = vee(logm(Rmean\MR_Bmat_rest(:,:,zz)));
        end
        rmean = mean(rpoints,2);
        
        while norm(rmean) > th_Rmean
            Rmean = Rmean*expm(hat(rmean));
            for zz = 1:length(MR_Bmat_rest)
                rpoints(:,zz) = vee(logm(Rmean\MR_Bmat_rest(:,:,zz)));
            end
            rmean = mean(rpoints,2);
        end
        MR_B_rest(:,:,tel) = Rmean*expm(hat(rmean));
        MH_B_rest(:,:,tel) = [MR_B_rest(:,:,tel), Mo_B_rest(:,tel); zeros(1,3),1];    
        
        id(tel,:) = [id_rel,id_rest];
    end
end

%% Write the release states to CSV file for Algoryx simulation
writeAGXinitstates(MH_B_rel(1:3,1:3,:),MH_B_rel(1:3,4,:),BV_MB_rel(1:3,:),BV_MB_rel(4:6,:),repmat(eye(3),1,1,length(BV_MB_rel(1,:))),zeros(3,1,length(BV_MB_rel(1,:))),'performance/AGX_init_states')

%% Write the release states to CSV file for MuJoCo simulation
writeMuJoCoStates(MH_B_rel(1:3,1:3,:),MH_B_rel(1:3,4,:),BV_MB_rel)

%% Write the varying parameters to a CSV file for Algoryx simulations
eN_vec_A = Algoryx_eN + randn(1,Nib)*Algoryx_eN_sigma/2;
mu_vec_A = Algoryx_mu + randn(1,Nib)*Algoryx_mu_sigma/2;

params_csv(1,:) = num2cell(["friction_distribution","restitution_distribution"]);    
params_csv(2:length(eN_vec_A)+1,:) = num2cell([mu_vec_A', eN_vec_A']);

if ~isfolder("performance/AGX_init_states")
    mkdir("performance/AGX_init_states");
end

writecell(params_csv,'performance/AGX_init_states/friction_restitution_distribution_test.csv');

%% Plot figure to demonstrate the release and rest determination
close all
plotnr = 1;
figure('rend','painters','pos',[500 500 500 230]);
ha = tight_subplot(1,1,[.08 .07],[.16 .05],[0.1 0.03]);  %[gap_h gap_w] [lower upper] [left right]
axes(ha(1));
    plot(((id(plotnr,1)-20):id(plotnr,2)+20)*dt,Mo_B(3,(id(plotnr,1)-20):id(plotnr,2)+20,plotnr)); hold on; 
    plot(id(plotnr,1)*dt,Mo_B(3,id(plotnr,1),plotnr),'o','markersize',10,'linewidth',2);
    plot(id(plotnr,2)*dt,Mo_B(3,id(plotnr,2),plotnr),'o','markersize',10,'linewidth',2);
    grid on;
    xlim([id(plotnr,1)-20,id(plotnr,2)+20]*dt);
    xlabel('Time [s]');
    ylabel('$(^M\mathbf{o}_B)_z$ [m]');
    X = [0.74 0.89];
    Y = [0.47 0.27];
    annotation('arrow',X,Y);
    text(1.5,0.095,'Moment of rest','Fontsize',12);
    X = [0.36 0.19];
    Y = [0.81 0.90];
    annotation('arrow',X,Y);
    text(1.26,0.138,'Moment of release','Fontsize',12);
    f = gcf;
%     print(gcf,'Rest-Release.png','-dpng','-r500'); %Uncomment if you want to save this image
    if doSave
        fig = gcf;
        fig.PaperPositionMode = 'auto';
        fig_pos = fig.PaperPosition;
        fig.PaperSize = [fig_pos(3) fig_pos(4)];
        print(fig,'figures/moment_of_release.pdf','-dpdf','-vector')
    end

%% Do the Matlab simulations of propagating the mean
for is = 1:tel
    %Obtain MATLAB results
    Ntimeidx = id(is,2)-id(is,1)+1; %Number of discrete time indices we want to run the simulation
    [MH_B_MATLAB,BV_MB_MATLAB] = BoxSimulator(MH_B_rel(1:3,4,is),MH_B_rel(1:3,1:3,is),BV_MB_rel(1:3,is),BV_MB_rel(4:6,is),MATLAB.(ObjStr).(Param)(1),MATLAB.(ObjStr).(Param)(2),MATLAB.(ObjStr).(Param)(3),Box,eye(3),zeros(3,1),dt,Ntimeidx);
    for ii = 1:length(MH_B_MATLAB)
    MH_B_Matlab(:,:,ii,is) = MH_B_MATLAB{ii};
    end
    MH_B_restM(:,:,is) = MH_B_MATLAB{end};
end

%% Load the single AGX simulations
AGXResult_h5file = "performance/data/220408_ManualTossBox5_AGX_CombCost.hdf5";
AGXData = readH5(AGXResult_h5file);
Results = orderfields(AGXData.box);
fnAGX = fieldnames(Results);
%Run through the simulations
for ia = 1:length(fnAGX)
    ta = Results.(fnAGX{ia}).ds;    
%     clear MH_B_AGX BMV_MB_AGX MX_B BV_MB_AGX
    %Obtain AGX results (run for each simulation through results)
    for ii = 1:length(ta(:,1))
        MH_B_AGX(:,:,ii,ia) = [ta(ii,1) ta(ii,5) ta(ii,9) ta(ii,13); ta(ii,2) ta(ii,6) ta(ii,10) ta(ii,14); ta(ii,3) ta(ii,7) ta(ii,11) ta(ii,15); ta(ii,4) ta(ii,8) ta(ii,12) ta(ii,16)];
        BMV_MB_AGX(:,ii) = ta(ii,17:22)';
        MX_B = [MH_B_AGX(1:3,1:3,ii,ia), zeros(3,3); zeros(3,3), MH_B_AGX(1:3,1:3,ii,ia)];
        BV_MB_AGX(:,ii) = MX_B\BMV_MB_AGX(:,ii);
    end
    MH_B_restAGX(:,:,ia) = MH_B_AGX(:,:,end,ia);
end


%% Now see if we take uncertainty of the parameters into account, what do we get
Nib = 25;
for is = 1:tel
    eN_vec = MATLAB_eN + randn(1,Nib)*MATLAB_eN_sigma/2;
    eT_vec = zeros(1,Nib);
    mu_vec = MATLAB_mu + randn(1,Nib)*MATLAB_mu_sigma/2;
    endframe = 3*fps;
    for ib = 1:Nib
        [MH_B_MATLAB,BV_MB_MATLAB] = BoxSimulator(MH_B_rel(1:3,4,is),MH_B_rel(1:3,1:3,is),BV_MB_rel(1:3,is),BV_MB_rel(4:6,is),eN_vec(ib),eT_vec(ib),mu_vec(ib),box5,eye(3),zeros(3,1),1/120,endframe);
        MH_B_restM_P(:,:,ib,is) = MH_B_MATLAB{end};
    end
end    

%% Load the multiple (varying params) AGX simulations
AGXResult_h5file = "performance/data/TossTest_BoxTossRandomFriction_result.hdf5";
AGXData = readH5(AGXResult_h5file);
Results = orderfields(AGXData.box);
fnAGX = fieldnames(Results);
%Run through the simulations
for toss_nr = 1:length(fnAGX)/Nib %The file contains length(fnAGX) simulation, where Nib simulations are per toss_nr
    for ib = ((Nib*(toss_nr-1))+1):((Nib*(toss_nr))) %For all the varying paramters in this toss
        ta = Results.(fnAGX{ib}).ds;   
        clear MH_B_AGX_P BMV_MB_AGX_P MX_B_P BV_MB_AGX_P 

        %Obtain AGX results (run for each simulation through results)
        for ii = 1:length(ta(:,1))
            MH_B_AGX_P(:,:,ii) = [ta(ii,1) ta(ii,5) ta(ii,9) ta(ii,13); ta(ii,2) ta(ii,6) ta(ii,10) ta(ii,14); ta(ii,3) ta(ii,7) ta(ii,11) ta(ii,15); ta(ii,4) ta(ii,8) ta(ii,12) ta(ii,16)];
            BMV_MB_AGX_P(:,ii) = ta(ii,17:22)';
            MX_B_P = [MH_B_AGX_P(1:3,1:3,ii), zeros(3,3); zeros(3,3), MH_B_AGX_P(1:3,1:3,ii)];
            BV_MB_AGX_P(:,ii) = MX_B_P\BMV_MB_AGX_P(:,ii);
        end
        MH_B_restAGX_P(:,:,(ib-((Nib*(toss_nr-1)))),toss_nr) = MH_B_AGX_P(:,:,end);
    end
end

%% Plot the results of the single Matlab + AGX simulation
figure('rend','painters','pos',[500 500 300 260]);
    ha = tight_subplot(1,1,[.08 .07],[.1 .08],[0.17 0.03]);  %[gap_h gap_w] [lower upper] [left right]
    axes(ha(1));
    for ii =1:tel
    Ptrans = MH_B_rest(:,:,ii)*[Box.vertices.ds';ones(1,8)];
    PtransM = MH_B_restM(:,:,ii)*[Box.vertices.ds';ones(1,8)];
%     PtransA = MH_B_restAGX(:,:,ii)*[Box.vertices.ds';ones(1,8)];
    x1 = [Ptrans(1,1:4) Ptrans(1,1)];
    y1 = [Ptrans(2,1:4) Ptrans(2,1)];
    x2 = [PtransM(1,1:4) PtransM(1,1)];
    y2 = [PtransM(2,1:4) PtransM(2,1)];
%     x3 = [PtransA(1,1:4) PtransA(1,1)];
%     y3 = [PtransA(2,1:4) PtransA(2,1)];
    
    fill(x1,y1,color.Meas); %Measured box 
    grid on; hold on;
    fill(x2,y2,color.Matlab);      %Matlab box
%     fill(x3,y3,color.Algoryx); %AGX box
    xlabel('$(^M\mathbf{o}_B)_x$');
    ylabel('$(^M\mathbf{o}_B)_y$');
    L1 = legend('Measured','Matlab','Algoryx','NumColumns',3,'location','northeast');
    L1.Position(2) = 0.90;
    L1.Position(1) = 0.52-(L1.Position(3)/2);
    axis equal
    axis([-0.3 1 -0.7 0.3]);
%     print(gcf,append('Rest-Pose_',sprintf('%.2d.png',ii)),'-dpng','-r500'); %Uncomment if you want to save this image
    if doSave
        fig = gcf;
        fig.PaperPositionMode = 'auto';
        fig_pos = fig.PaperPosition;
        fig.PaperSize = [fig_pos(3) fig_pos(4)];
        print(fig,append('figures/RestPose/Rest-Pose_',sprintf('%.2d.pdf',ii)),'-dpdf','-vector')
    end
    pause();
    hold off;
    end
%% Plot the results of the single Matlab + AGX simulation in smaller figure
figure('rend','painters','pos',[500 500 150 195]);
    ha = tight_subplot(1,1,[.08 .07],[.16 .02],[0.21 0.05]);  %[gap_h gap_w] [lower upper] [left right]
    axes(ha(1));
    for ii =1:tel
    Ptrans = MH_B_rest(:,:,ii)*[Box.vertices.ds';ones(1,8)];
    PtransM = MH_B_restM(:,:,ii)*[Box.vertices.ds';ones(1,8)];
%     PtransA = MH_B_restAGX(:,:,ii)*[Box.vertices.ds';ones(1,8)];
    x1 = [Ptrans(1,1:4) Ptrans(1,1)];
    y1 = [Ptrans(2,1:4) Ptrans(2,1)];
    x2 = [PtransM(1,1:4) PtransM(1,1)];
    y2 = [PtransM(2,1:4) PtransM(2,1)];
%     x3 = [PtransA(1,1:4) PtransA(1,1)];
%     y3 = [PtransA(2,1:4) PtransA(2,1)];
    
    fill(x1,y1,color.Meas); %Measured box 
    grid on; hold on;
    fill(x2,y2,color.Matlab);      %Matlab box
%     fill(x3,y3,color.Algoryx); %AGX box
    xlabel('$(^M\mathbf{o}_B)_x$');
    ylabel('$(^M\mathbf{o}_B)_y$');
    axis equal
    axis([-0.1 0.6 -0.1 0.9]); %Box006
%     axis([0 0.7 -0.7 0.3]); %Box005
    if doSave
        fig = gcf;
        fig.PaperPositionMode = 'auto';
        fig_pos = fig.PaperPosition;
        fig.PaperSize = [fig_pos(3) fig_pos(4)];
        print(fig,append('figures/RestPose/',ObjStr,'_',Param,'/Rest-Pose_',sprintf('%.2d.pdf',ii)),'-dpdf','-vector')
    end
    pause();
    hold off;
    end
%% Plot results of the multiple Matlab simulations (varying parameters)
figure('rend','painters','pos',[500 500 300 250]);
    ha = tight_subplot(1,1,[.08 .07],[.08 .01],[0.15 0.03]);  %[gap_h gap_w] [lower upper] [left right]
    axes(ha(1));
    for toss_nr =13 %:tel
    Ptrans = MH_B_rest(:,:,toss_nr)*[box5.vertices;ones(1,8)];
    PtransM = MH_B_restM(:,:,toss_nr)*[box5.vertices;ones(1,8)];
    x1 = [Ptrans(1,1:4) Ptrans(1,1)];
    y1 = [Ptrans(2,1:4) Ptrans(2,1)];
    x2 = [PtransM(1,1:4) PtransM(1,1)];
    y2 = [PtransM(2,1:4) PtransM(2,1)];

    %Plot result from the experiments
    fill(x1,y1,color.Meas); %Measured box  
    axis equal; axis([-0.3 1 -0.7 0.3]); grid on; hold on
    fill(x2,y2,color.Matlab);      %Matlab box

    %Plot the result from the sampled simulations
    for ib = 1:Nib
    PtransM(:,:,ib) = MH_B_restM_P(:,:,ib,toss_nr)*[box5.vertices;ones(1,8)];
    x2 = [PtransM(1,1:4,ib) PtransM(1,1,ib)];
    y2 = [PtransM(2,1:4,ib) PtransM(2,1,ib)];

    h = fill(x2,y2,color.Matlab); %Matlab box
    h.FaceAlpha = 0.05;
    h.EdgeColor = [0 0 0];
    h.EdgeAlpha = 0.2;    
    end

    xlabel('$(^Mo_B)_x$');
    ylabel('$(^Mo_B)_y$');
    legend('Measured','Matlab');
    
%     print(gcf,append('Rest_Pose_multiple_M',sprintf('%.2d.png',toss_nr)),'-dpng','-r500'); %Uncomment if you want to save this image
    pause();

    hold off;
    end

%% Plot results of the multiple AGX simulations (varying parameters)
figure('rend','painters','pos',[500 500 300 250]);
    ha = tight_subplot(1,1,[.08 .07],[.08 .01],[0.15 0.03]);  %[gap_h gap_w] [lower upper] [left right]
    axes(ha(1));
    for toss_nr = 13; %1:length(fnAGX)/Nib
    Ptrans = MH_B_rest(:,:,toss_nr)*[box5.vertices;ones(1,8)];
    PtransA = MH_B_restAGX(:,:,toss_nr)*[box5.vertices;ones(1,8)];
    x1 = [Ptrans(1,1:4) Ptrans(1,1)];
    y1 = [Ptrans(2,1:4) Ptrans(2,1)];
    x3 = [PtransA(1,1:4) PtransA(1,1)];
    y3 = [PtransA(2,1:4) PtransA(2,1)];

    %Plot result from the experiments

    fill(x1,y1,color.Meas); %Measured box
    axis equal; axis([-0.3 1 -0.7 0.3]); grid on; hold on
    fill(x3,y3,color.Algoryx); %AGX box

    %Plot the result from the sampled simulations
    for ib = 1:Nib
        PtransA(:,:,ib) = MH_B_restAGX_P(:,:,ib,toss_nr)*[box5.vertices;ones(1,8)];
        x3 = [PtransA(1,1:4,ib) PtransA(1,1,ib)];
        y3 = [PtransA(2,1:4,ib) PtransA(2,1,ib)];

        j = fill(x3,y3,color.Algoryx); %AGX box
        j.FaceAlpha = 0.05;
        j.EdgeColor = [0 0 0];
        j.EdgeAlpha = 0.2;
    end

    xlabel('$(^Mo_B)_x$');
    ylabel('$(^Mo_B)_y$');
    legend('Measured','Algoryx');

%         print(gcf,append('Rest_Pose_multiple_A',sprintf('%.2d.png',toss_nr)),'-dpng','-r500'); %Uncomment if you want to save this image
%         pause();

    hold off;
    end
%% Compute the errors of the rest-orientation and rest-position
for ii =1:tel
    E_rot_M(ii,:) = rad2deg(rotm2eul(MH_B_rest(1:3,1:3,ii)\MH_B_restM(1:3,1:3,ii)));
%     E_rot_A(ii,:) = rad2deg(rotm2eul(MH_B_rest(1:3,1:3,ii)\MH_B_restAGX(1:3,1:3,ii)));
    E_pos_M(ii,:) = (MH_B_rest(1:3,4,ii)-MH_B_restM(1:3,4,ii))';
%     E_pos_A(ii,:) = (MH_B_rest(1:3,4,ii)-MH_B_restAGX(1:3,4,ii))';
%     E_pos_M_P(ii,:) = (MH_B_rest(1:3,4,ii)-mean(squeeze((MH_B_restM_P(1:3,4,:,ii))),2))';
%     E_pos_A_P(ii,:) = (MH_B_rest(1:3,4,ii)-mean(squeeze((MH_B_restAGX_P(1:3,4,:,ii))),2))';
end

e_pos_M = norm(mean(abs(E_pos_M(:,1:2))));
e_rot_M = mean(abs(E_rot_M(:,1)));
% e_pos_A = norm(mean(abs(E_pos_A(:,1:2))));
% e_rot_A = mean(abs(E_rot_A(:,1)));
% e_pos_M_P = norm(mean(abs(E_pos_M_P(:,1:2))));
% e_pos_A_P = norm(mean(abs(E_pos_A_P(:,1:2))));

%% Plot single trajectory in space to demonstrate simulation
% Plotting options For plotting the contact surface
ws    = 1.5;  %Width of the contact surface             [m]
ls    = 1.5;  %Length of the contact surface           [m]
surfacepoints = [0.5*ws -0.5*ws -0.5*ws 0.5*ws 0.5*ws; -0.5*ls -0.5*ls 0.5*ls 0.5*ls -0.5*ls; 0 0 0 0 0;];
FR_C = eye(3); 
Fo_C = zeros(3,1);
spoints = FR_C*surfacepoints +Fo_C; %Transform the vertices according to position/orientation of the surface

plotnr = 1;
%Plot the trajectory of the box
figure('pos',[500 500 500 300]);
    for ii=id(plotnr,1):5:id(plotnr,1)+(id(plotnr,2)-id(plotnr,1))-1
        
        %plot Measured box
        g1 = plotBox(MH_Bm(:,:,ii,plotnr),Box,color.Meas,0);hold on;
        
        %Plot MATLAB box
        g2 = plotBox(MH_B_Matlab(:,:,ii-(id(plotnr,1)-1),plotnr),Box,color.Matlab,0); hold on;     

        %Plot new AGX box results
%         g3 = plotBox(MH_B_AGX(:,:,ii-(id(plotnr,1)-1),plotnr),box5,color.Algoryx,0);hold on;

        %Plot the conveyor C
        table3 = fill3(spoints(1,1:4),spoints(2,1:4),spoints(3,1:4),1);hold on;
        set(table3,'FaceColor',0.8*[1 1 1],'FaceAlpha',1);
        
        %Plot the origin of the contact surface with its unit vectors
        tip = [Fo_C+0.3*FR_C(:,1) Fo_C+0.3*FR_C(:,2) Fo_C+0.3*FR_C(:,3)];
        plot3([Fo_C(1) tip(1,1)],[Fo_C(2) tip(2,1)],[Fo_C(3) tip(3,1)],'r'); hold on
        plot3([Fo_C(1) tip(1,2)],[Fo_C(2) tip(2,2)],[Fo_C(3) tip(3,2)],'g');
        plot3([Fo_C(1) tip(1,3)],[Fo_C(2) tip(2,3)],[Fo_C(3) tip(3,3)],'b');
        
        %Plot the origin of the world coordinate frame
        tip = [0.3*[1;0;0] 0.3*[0;1;0] 0.3*[0;0;1]];
        plot3([0 tip(1,1)],[0 tip(2,1)],[0 tip(3,1)],'r'); hold on
        plot3([0 tip(1,2)],[0 tip(2,2)],[0 tip(3,2)],'g');
        plot3([0 tip(1,3)],[0 tip(2,3)],[0 tip(3,3)],'b');

        grid on;axis equal;
        axis([-0.4 0.6 -0.6 0.8 -0.05 0.3]);
        xlabel('x [m]');
        ylabel('y [m]');
        zlabel('z [m]');
%         view(-118,16);
%         view(-118,27);
        view(-315,31);
%         view(-90,0);
        text(1,0.6,0.4,append('Frame:',sprintf('%d',ii-(id(plotnr,1)-1))));
        L1 = legend([g1 g2 ],'Measured','Matlab','Algoryx','NumColumns',3,'location','northeast');
        L1.Position(2) = 0.90;
        L1.Position(1) = 0.52-(L1.Position(3)/2);
        drawnow
        hold off
%         pause()        
%         f = gcf;
%         exportgraphics(f,append('Frame_',sprintf('%.2d.jpg',ii-(id(plotnr,1)-1))),'Resolution',500)
    end

%% Plot measured trajectory for paper figure
% Plotting options For plotting the contact surface
ws    = 3.5;  %Width of the contact surface             [m]
ls    = 3.5;  %Length of the contact surface           [m]
surfacepoints = [0.5*ws -0.5*ws -0.5*ws 0.5*ws 0.5*ws; -0.5*ls -0.5*ls 0.5*ls 0.5*ls -0.5*ls; 0 0 0 0 0;];
FR_C = eye(3); 
Fo_C = zeros(3,1);
spoints = FR_C*surfacepoints +Fo_C; %Transform the vertices according to position/orientation of the surface

plotnr = 11;
%Plot the trajectory of the box
figure('pos',[50 50 400 200]);
    ha = tight_subplot(1,1,[.08 .07],[.01 -.3],[0.03 0.03]);  %[gap_h gap_w] [lower upper] [left right]
    axes(ha(1));
    %Plot the conveyor C
    table3 = fill3(spoints(1,1:4),spoints(2,1:4),spoints(3,1:4),1);hold on;
    set(table3,'FaceColor',[56 53 48]/255,'FaceAlpha',1);
    
    %plot Measured box
    for ii=id(plotnr,1):18:id(plotnr,1)+(id(plotnr,2)-id(plotnr,1))-1        
        g1 = plotBox(MH_Bm(:,:,ii,plotnr),Box,[194 135 43]/255,0);hold on;   
        drawnow
    end

    %Other plot options
    axis equal;
    axis([-0.4 0.6 -1 0.4 -0.05 0.3]);
    view(-118,27);
    camproj('perspective')
    axis off;

    if doSave
        f = gcf;
        exportgraphics(f,'figures/Measured_box_trajectory_2.png','Resolution',1500);
    end

%% Plot impact sequence over time (Figure 1 of paper) based on Sander data
close all;
%For plotting the contact surface
ws    = 1.5;  %Width of the contact surface             [m]
ls    = 1.5;  %Length of the contact surface           [m]
surfacepoints = [0.5*ws -0.5*ws -0.5*ws 0.5*ws 0.5*ws; -0.5*ls -0.5*ls 0.5*ls 0.5*ls -0.5*ls; 0 0 0 0 0;];
FR_C = eye(3); 
Fo_C = zeros(3,1);
spoints = FR_C*surfacepoints +Fo_C; %Transform the vertices according to position/orientation of the surface

dataSander = load('data.mat');
MH_B_sander = dataSander.data.POSTPROCESSING.Box006.transforms.ds;
for jj = 1:length(MH_B_sander); MH_Bm_sander(:,:,jj) = MH_B_sander{jj}; end
T = [Rx(90) zeros(3,1);zeros(1,3) 1];

plotnr = 10;
fig = figure('pos',[200 200 2*380 2*150]);
    ha = tight_subplot(1,1,[.08 .07],[-0.1 -.5],[0.01 0.03]);  %[gap_h gap_w] [lower upper] [left right]
    axes(ha(1));

    %Plot the conveyor C
    table3 = fill3(spoints(1,1:4),spoints(2,1:4),spoints(3,1:4),1);hold on;
    set(table3,'FaceColor',[56 53 48]/255,'FaceAlpha',1);

    for ii = [2050 2314 2355 2381 2397 2600]
        plotBox(T*cat(3,MH_Bm_sander(:,:,ii)),Box,[194 135 43]/255,0);
        drawnow;
    end    
    camproj('perspective')
    grid on; axis equal; axis off;
    view(-124,15);
    axis([-0.2 0.6 -0.7 0.7 0 0.5]);

    if doSave
        f = gcf;
        exportgraphics(f,'figures/ImpactSequence.png','Resolution',1500);
    end
      
%% FOR THE POSTER
close all
plotnr = 1;
figure('rend','painters','pos',[704,405,558,357]);
ha1 = tight_subplot(1,2,[.08 .09],[.12 .06],[0.1 0.0]);  %[gap_h gap_w] [lower upper] [left right]
ha = tight_subplot(2,2,[.1 0],[.1 .06],[0.0 0.0]);  %[gap_h gap_w] [lower upper] [left right]
ha1(2).Visible = "off";
ha(1).Visible = "off"; ha(3).Visible = "off";
axes(ha1(1));
    plot(((id(plotnr,1)-20):id(plotnr,2)+20)/120,Mo_B(3,(id(plotnr,1)-20):id(plotnr,2)+20,plotnr)); hold on; 
    plot(id(plotnr,1)*dt,Mo_B(3,id(plotnr,1),plotnr),'o','markersize',10,'linewidth',2);
    plot(id(plotnr,2)*dt,Mo_B(3,id(plotnr,2),plotnr),'o','markersize',10,'linewidth',2);
    grid on;
    xlim([id(plotnr,1)-5,id(plotnr,2)+20]*dt);
    xlabel('Time [s]');
    ylabel('$(^M\mathbf{o}_B)_z$ [m]');
    X = [0.37 0.42];
    Y = [0.44 0.26];
    annotation('arrow',X,Y);
    text(1.68,0.1,'Moment of rest','Fontsize',12);
    X = [0.22 0.14];
    Y = [0.82 0.91];
    annotation('arrow',X,Y);
    text(1.59,0.158,'Moment of release','Fontsize',12);

 axes(ha(2));
    toss_nr =13;
    Ptrans = MH_B_rest(:,:,toss_nr)*[box5.vertices;ones(1,8)];
    PtransM = MH_B_restM(:,:,toss_nr)*[box5.vertices;ones(1,8)];
    x1 = [Ptrans(1,1:4) Ptrans(1,1)];
    y1 = [Ptrans(2,1:4) Ptrans(2,1)];
    x2 = [PtransM(1,1:4) PtransM(1,1)];
    y2 = [PtransM(2,1:4) PtransM(2,1)];

    %Plot result from the experiments
    p1 = fill(x1,y1,color.Meas); %Measured box  
    axis equal; axis([0 0.8 -0.5 0.1]); grid on; hold on
    p2 = fill(x2,y2,color.Matlab);      %Matlab box

    %Plot the result from the sampled simulations
    for ib = 1:Nib
    PtransM(:,:,ib) = MH_B_restM_P(:,:,ib,toss_nr)*[box5.vertices;ones(1,8)];
    x2 = [PtransM(1,1:4,ib) PtransM(1,1,ib)];
    y2 = [PtransM(2,1:4,ib) PtransM(2,1,ib)];
    h = fill(x2,y2,color.Matlab); %Matlab box
    h.FaceAlpha = 0.05;
    h.EdgeColor = [0 0 0];
    h.EdgeAlpha = 0.2;    
    end
    xlabel('$(^Mo_B)_x$');
    ylabel('$(^Mo_B)_y$');
    


 axes(ha(4));
    Ptrans = MH_B_rest(:,:,toss_nr)*[box5.vertices;ones(1,8)];
    PtransA = MH_B_restAGX(:,:,toss_nr)*[box5.vertices;ones(1,8)];
    x1 = [Ptrans(1,1:4) Ptrans(1,1)];
    y1 = [Ptrans(2,1:4) Ptrans(2,1)];
    x3 = [PtransA(1,1:4) PtransA(1,1)];
    y3 = [PtransA(2,1:4) PtransA(2,1)];

    %Plot result from the experiments
    fill(x1,y1,color.Meas); %Measured box
    axis equal; axis([0 0.8 -0.5 0.1]); grid on; hold on
    p3 = fill(x3,y3,color.Algoryx); %AGX box

    %Plot the result from the sampled simulations
    for ib = 1:Nib
        PtransA(:,:,ib) = MH_B_restAGX_P(:,:,ib,toss_nr)*[box5.vertices;ones(1,8)];
        x3 = [PtransA(1,1:4,ib) PtransA(1,1,ib)];
        y3 = [PtransA(2,1:4,ib) PtransA(2,1,ib)];

        j = fill(x3,y3,color.Algoryx); %AGX box
        j.FaceAlpha = 0.05;
        j.EdgeColor = [0 0 0];
        j.EdgeAlpha = 0.2;
    end

    xlabel('$(^Mo_B)_x$');
    ylabel('$(^Mo_B)_y$');
    L1 = legend([p1 p2 p3],'Measured','Matlab','Algoryx','NumColumns',3,'location','east');
    L1.Position = [0.3 0.95 0.5 L1.Position(4)]

    if doSave
        fig = gcf;
        fig.PaperPositionMode = 'auto';
        fig_pos = fig.PaperPosition;
        fig.PaperSize = [fig_pos(3) fig_pos(4)];
        print(fig,'PosterResultsToss.pdf','-dpdf','-vector')
    end

    


