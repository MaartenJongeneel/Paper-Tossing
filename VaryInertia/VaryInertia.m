clearvars; close all; set(groot,'defaulttextinterpreter','latex'); set(groot,'defaultAxesTickLabelInterpreter','latex'); set(groot,'defaultLegendInterpreter','latex');
addpath(genpath('readH5')); addpath('data');
%% Evaluate the manual tosses
%Goal is to see how the varying mass affects the predictability of the
%restpose
%% Load the data
% data = readH5('221021_Archive_011_Box005Box006_Validation.h5'); %Using dataset for validation also for sensitivity
data = readH5('230104_Archive_020_Box007_Validation.h5'); %Using dataset for validation also for sensitivity
%% Constants
th_Rmean = 1e-5; %Threshold rotation mean
color.Matlab = [237 176 33]/255;
color.Algoryx = [77 191 237]/255;
color.Meas = [128 128 128]/255;
N_pos    = 20; %Number of consecutive points where the error is low
doSave   = false;
MATLAB.Box005.Traj  = [0.10 0.00 0.45]; %eN eT mu
MATLAB.Box006.Traj  = [0.25 0.00 0.40]; %eN eT mu
MATLAB.Box007.Traj  = [0.45 0.00 0.35]; 
Algoryx.Box005.Traj = [0.00 0.00 0.40]; %eN eT mu
Algoryx.Box006.Traj = [0.30 0.00 0.40]; %eN eT mu

ObjStr = "Box007"; %The object for which you want to do paramID
% ImpPln = "GroundPlane001";
ImpPln = "ConveyorPart002"; %For Box007
Param = "mass";  %Sensitivity of mu or eN 

%% Loop through the data
tel = 0;
fn = fieldnames(data);
for ii = 1:length(fn)
    if startsWith(fn{ii},'Rec')
        %Get the object info
        try
            Box = data.(fn{ii}).OBJECT.(ObjStr);
        catch
            continue;
        end
        if contains(data.(fn{ii}).attr.note,"moving") || contains(data.(fn{ii}).attr.note,"running")
            continue;
        end

        %Now we know this recording is used, we can update the teller
        tel = tel+1;

        %Get the data from the file
        Mocap = data.(fn{ii}).SENSOR_MEASUREMENT.Mocap;
        dt   = 1/double(Mocap.datalog.attr.sample_frequency);   %Timestep of the recording 
        


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
        elseif ObjStr == "Box007"
            if ii==31
                [pks,id_rel] = findpeaks(Mo_B(3,:,tel),'MinPeakHeight',0.12,'MinPeakWidth',10);
            else
            [pks,id_rel] = findpeaks(Mo_B(3,:,tel),'MinPeakHeight',0.115,'MinPeakWidth',10);%,'MinPeakProminence',0.05,'MinPeakWidth',10);
            id_rel = id_rel(end);
            end
        end

            figure; plot(Mo_B(3,:,tel)); hold on;
                plot(id_rel,Mo_B(3,id_rel,tel),'o','markersize',10,'linewidth',2);
                plot(id_rest,Mo_B(3,id_rest,tel),'o','markersize',10,'linewidth',2);
                grid on;
                pause
                close all
        
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

%% Write the varying parameters to a CSV file for Algoryx simulations
% if Param == "mu" %If we consider sensitivity of mu, vary mu, keep eN constant
%     eN_vec_A = Algoryx.(ObjStr).Traj(1) + linspace(0,0,11);
%     mu_vec_A = Algoryx.(ObjStr).Traj(3) + linspace(-0.05,0.05,11);
% elseif Param == "eN" %If we consider sensitivity of eN, vary eN, keep mu constant
%     eN_vec_A = Algoryx.(ObjStr).Traj(1) + linspace(-0.05,0.05,11);
%     mu_vec_A = Algoryx.(ObjStr).Traj(3) + linspace(0,0,11);
% end
% 
% params_csv(1,:) = num2cell(["friction_distribution","restitution_distribution"]);    
% params_csv(2:length(eN_vec_A)+1,:) = num2cell([mu_vec_A', eN_vec_A']);
% 
% 
% writecell(params_csv,append('sensitivity/',ObjStr,'_',Param,'/friction_restitution_distribution_test.csv'));
% 
% %Write initial states file of box and conveyor 
% writeAGXinitstates(MH_B_rel(1:3,1:3,:),MH_B_rel(1:3,4,:),BV_MB_rel(1:3,:),BV_MB_rel(4:6,:),repmat(eye(3),1,1,length(BV_MB_rel(1,:))),zeros(3,1,length(BV_MB_rel(1,:))),append('sensitivity/',ObjStr,'_',Param,'/'));

%% Matlab simulations for varying params
Nib = 11;
if Param == "mass"
    mass_vec = Box.mass.ds + linspace(-0.3,0.3,Nib);
end
for is = 1:tel
    endframe = 2*1/dt;
    for ib = 1:Nib
        Box.inertia.ds(1:3,1:3)=diag(repmat(mass_vec(ib),1,3));
        Box.mass.ds=mass_vec(ib);
        [MH_B_MATLAB,BV_MB_MATLAB] = BoxSimulator(MH_B_rel(1:3,4,is),MH_B_rel(1:3,1:3,is),BV_MB_rel(1:3,is),BV_MB_rel(4:6,is),MATLAB.(ObjStr).Traj(1),MATLAB.(ObjStr).Traj(2),MATLAB.(ObjStr).Traj(3),Box,eye(3),zeros(3,1),dt,endframe);
        MH_B_restM_P(:,:,ib,is) = MH_B_MATLAB{end};
    end
end    

% %% Load the multiple (varying params) AGX simulations
% AGXResult_h5file = append('sensitivity/',ObjStr,'_',Param,'/',ObjStr,'_sens_',Param,'_BoxTossRandomFriction_result.hdf5');
% AGXData = readH5(AGXResult_h5file);
% Results = orderfields(AGXData.box);
% fnAGX = fieldnames(Results);
% %Run through the simulations
% for toss_nr = 1:length(fnAGX)/Nib %The file contains length(fnAGX) simulation, where Nib simulations are per toss_nr
%     for ib = ((Nib*(toss_nr-1))+1):((Nib*(toss_nr))) %For all the varying paramters in this toss
%         ta = Results.(fnAGX{ib}).ds;   
%         clear MH_B_AGX_P BMV_MB_AGX_P MX_B_P BV_MB_AGX_P 
% 
%         %Obtain AGX results (run for each simulation through results)
%         for ii = 1:length(ta(:,1))
%             MH_B_AGX_P(:,:,ii) = [ta(ii,1) ta(ii,5) ta(ii,9) ta(ii,13); ta(ii,2) ta(ii,6) ta(ii,10) ta(ii,14); ta(ii,3) ta(ii,7) ta(ii,11) ta(ii,15); ta(ii,4) ta(ii,8) ta(ii,12) ta(ii,16)];
%             BMV_MB_AGX_P(:,ii) = ta(ii,17:22)';
%             MX_B_P = [MH_B_AGX_P(1:3,1:3,ii), zeros(3,3); zeros(3,3), MH_B_AGX_P(1:3,1:3,ii)];
%             BV_MB_AGX_P(:,ii) = MX_B_P\BMV_MB_AGX_P(:,ii);
%         end
%         MH_B_restAGX_P(:,:,(ib-((Nib*(toss_nr-1)))),toss_nr) = MH_B_AGX_P(:,:,end);
%     end
% end

%% Plot results of the multiple Matlab simulations (varying parameters)
figure('rend','painters','pos',[500 500 150 195]);
    ha = tight_subplot(1,1,[.08 .07],[.16 .02],[0.21 0.05]);  %[gap_h gap_w] [lower upper] [left right]
    axes(ha(1));
    for toss_nr =1:tel
    Ptrans = MH_B_rest(:,:,toss_nr)*[Box.vertices.ds';ones(1,8)];
    PtransM = MH_B_restM_P(:,:,6,toss_nr)*[Box.vertices.ds';ones(1,8)]; %6th is the mean
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
    PtransM(:,:,ib) = MH_B_restM_P(:,:,ib,toss_nr)*[Box.vertices.ds';ones(1,8)];
    x2 = [PtransM(1,1:4,ib) PtransM(1,1,ib)];
    y2 = [PtransM(2,1:4,ib) PtransM(2,1,ib)];

    h = fill(x2,y2,color.Matlab); %Matlab box
    h.FaceAlpha = 0.1;
    h.EdgeColor = [0 0 0];
    h.EdgeAlpha = 0.4;    
    end

    xlabel('$(^Mo_B)_x$');
    ylabel('$(^Mo_B)_y$');
%     legend('Measured','Matlab');
    axis([-0.1 0.6 -0.1 0.9]); 
    if doSave; fig = gcf; fig.PaperPositionMode = 'auto'; fig_pos = fig.PaperPosition; fig.PaperSize = [fig_pos(3) fig_pos(4)];
        print(fig,append('VaryInertia/figures/',ObjStr,'RestPose/Rest-Pose_',sprintf('%.2d.pdf',toss_nr)),'-dpdf','-vector'); end

%     pause();

    hold off;
    end

%% Plot results of the multiple AGX simulations (varying parameters)
% figure('rend','painters','pos',[500 500 150 195]);
%     ha = tight_subplot(1,1,[.08 .07],[.16 .02],[0.21 0.05]);  %[gap_h gap_w] [lower upper] [left right]
%     axes(ha(1));
%     for toss_nr = 1:length(fnAGX)/Nib
%     Ptrans = MH_B_rest(:,:,toss_nr)*[Box.vertices.ds';ones(1,8)];
%     PtransA = MH_B_restAGX_P(:,:,6,toss_nr)*[Box.vertices.ds';ones(1,8)]; %6th is the mean
%     x1 = [Ptrans(1,1:4) Ptrans(1,1)];
%     y1 = [Ptrans(2,1:4) Ptrans(2,1)];
%     x3 = [PtransA(1,1:4) PtransA(1,1)];
%     y3 = [PtransA(2,1:4) PtransA(2,1)];
% 
%     %Plot result from the experiments
% 
%     fill(x1,y1,color.Meas); %Measured box
%     axis equal; axis([-0.3 1 -0.7 0.3]); grid on; hold on
%     fill(x3,y3,color.Algoryx); %AGX box
% 
%     %Plot the result from the sampled simulations
%     for ib = 1:Nib
%         PtransA(:,:,ib) = MH_B_restAGX_P(:,:,ib,toss_nr)*[Box.vertices.ds';ones(1,8)];
%         x3 = [PtransA(1,1:4,ib) PtransA(1,1,ib)];
%         y3 = [PtransA(2,1:4,ib) PtransA(2,1,ib)];
% 
%         j = fill(x3,y3,color.Algoryx); %AGX box
%         j.FaceAlpha = 0.1;
%         j.EdgeColor = [0 0 0];
%         j.EdgeAlpha = 0.4;
%     end
% 
%     xlabel('$(^Mo_B)_x$');
%     ylabel('$(^Mo_B)_y$');
%     axis([-0.1 0.6 -0.1 0.9]); 
%     if doSave; fig = gcf; fig.PaperPositionMode = 'auto'; fig_pos = fig.PaperPosition; fig.PaperSize = [fig_pos(3) fig_pos(4)];
%         print(fig,append('figures/RestPose/sensitivity/',ObjStr,'_',Param,'/AGX/Rest-Pose_',sprintf('%.2d.pdf',toss_nr)),'-dpdf','-vector'); end
% 
%     pause();
% 
%     hold off;
%     end
%% Compute the errors of the rest-orientation and rest-position
for ii =1:tel
    E_rot_M(ii,:) = rad2deg(rotm2eul(MH_B_rest(1:3,1:3,ii)\MH_B_restM_P(1:3,1:3,6,ii)));
%     E_rot_A(ii,:) = rad2deg(rotm2eul(MH_B_rest(1:3,1:3,ii)\MH_B_restAGX_P(1:3,1:3,6,ii)));
    E_pos_M(ii,:) = (MH_B_rest(1:3,4,ii)-MH_B_restM_P(1:3,4,6,ii))';
    E_pos_M_deviation(ii,:)= mean(vecnorm(squeeze(MH_B_restM_P(1:3,4,6,ii)-MH_B_restM_P(1:3,4,:,ii)))); % mean pos deviation of varied mass to perfect mass
    E_rot_M_deviation(ii,:) = mean(abs(rad2deg(rotm2eul(MH_B_restM_P(1:3,1:3,6,ii))-rotm2eul(MH_B_restM_P(1:3,1:3,:,ii)))),1); % mean rot deviation of varied mass to perfect mass
    %     E_pos_A(ii,:) = (MH_B_rest(1:3,4,ii)-MH_B_restAGX_P(1:3,4,6,ii))';
    E_pos_M_P(ii,:) = (MH_B_rest(1:3,4,ii)-mean(squeeze((MH_B_restM_P(1:3,4,:,ii))),2))';
%     E_pos_A_P(ii,:) = (MH_B_rest(1:3,4,ii)-mean(squeeze((MH_B_restAGX_P(1:3,4,:,ii))),2))';
end

e_pos_M = norm(mean(abs(E_pos_M(:,1:2))));
e_rot_M = mean(abs(E_rot_M(:,1)));
% e_pos_A = norm(mean(abs(E_pos_A(:,1:2))));
% e_rot_A = mean(abs(E_rot_A(:,1)));
e_pos_M_P = norm(mean(abs(E_pos_M_P(:,1:2))));
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
        g3 = plotBox(MH_B_AGX(:,:,ii-(id(plotnr,1)-1),plotnr),Box,color.Algoryx,0);hold on;

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
    


