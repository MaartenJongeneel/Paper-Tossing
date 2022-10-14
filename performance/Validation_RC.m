clearvars; close all; set(groot,'defaulttextinterpreter','latex'); set(groot,'defaultAxesTickLabelInterpreter','latex'); set(groot,'defaultLegendInterpreter','latex');
addpath(genpath('readH5')); addpath('data');
%% Evaluate the manual tosses
%Goal is to see if for the manual tosses, the MATLAB and AGX simulators are
%able to predict the rest pose accurately, as we saw that this was not
%really the case for the automatic tosses by the robot arm, which might
%have been from too much height. In the performed experiments, box5 is
%tossed manually on an idle conveyor.
%% Load the data
data = readH5('221005_Box005_RC_Validation.h5'); %Validation of Box005
% data = readH5('221005_Box006_RC_Validation.h5'); %Validation of Box006
%% Constants
th_Rmean = 1e-5; %Threshold rotation mean
color.Matlab = [237 176 33]/255;
color.Algoryx = [77 191 237]/255;
color.Meas = [128 128 128]/255;
N_pos    = 20; %Number of consecutive points where the error is low
doSave   = false;
MATLAB.Box005.Vel   = [0.35 0.00 0.45]; %eN eT mu
MATLAB.Box005.Traj  = [0.10 0.00 0.45]; %eN eT mu
MATLAB.Box006.Vel   = [0.40 0.00 0.25]; %eN eT mu 
MATLAB.Box006.Traj  = [0.25 0.00 0.40]; %eN eT mu

ObjStr = "Box005"; %The object for which you want to do paramID
ImpPln = "ConveyorPart002";
Param = "Traj";
%% Loop through the data
tel = 0;
fn = fieldnames(data);
for ii = 1:length(fn)
    if startsWith(fn{ii},'Rec')
        

        %Get the data from the file
        Mocap = data.(fn{ii}).SENSOR_MEASUREMENT.Mocap;
        dt   = 1/double(Mocap.datalog.attr.sample_frequency);   %Timestep of the recording 
        
        %Get the object info
        Box = data.(fn{ii}).OBJECT.(ObjStr);

        %Get the tranforms of the object
        if ~isfield(Mocap.POSTPROCESSING,ObjStr)
            continue;
        end
        tel = tel+1;

        MH_B = Mocap.POSTPROCESSING.(ObjStr).transforms.ds; 
        MH_C = Mocap.POSTPROCESSING.(ImpPln).transforms.ds;

        %Rewrite the data into mat structures
        Nsamples = length(MH_B);
        for jj = 1:Nsamples
            MH_Cm(:,:,jj,tel) = MH_C{jj};
            MH_Bm(:,:,jj,tel) = MH_B{jj};
        end
        
        %Few definitions from data:
        Mo_C(:,1:Nsamples,tel) = squeeze(MH_Cm(1:3,4,1:Nsamples,tel));
        Mo_B(:,1:Nsamples,tel) = squeeze(MH_Bm(1:3,4,1:Nsamples,tel));
        Co_B(:,1:Nsamples,tel) = Mo_B(:,1:Nsamples,tel)-Mo_C(:,1:Nsamples,tel);

        
        %-------------------- Compute the velocities --------------------%
        %Compute angular and linear velocity of B w.r.t. M
        for ij = 2:Nsamples-1
            %Compute velocities using central difference approximation
            dMo_B(:,ij) = 1/(2*dt)*(MH_Bm(1:3,4,ij+1,tel)-MH_Bm(1:3,4,ij-1,tel));
            dCo_B(:,ij) = 1/(2*dt)*(Co_B(:,ij+1,tel)-Co_B(:,ij-1,tel));
            dMo_C(:,ij) = 1/(2*dt)*(Mo_C(:,ij+1,tel)-Mo_C(:,ij-1,tel));
            
            %Left trivialized velocity w.r.t M
            Bomg_MB(:,ij) = vee(1/(2*dt)*((logm(MH_Bm(1:3,1:3,ij,tel)\(MH_Bm(1:3,1:3,ij+1,tel))))-(logm(MH_Bm(1:3,1:3,ij,tel)\(MH_Bm(1:3,1:3,ij-1,tel))))));
            Bv_MB(:,ij)   = MH_Bm(1:3,1:3,ij,tel)' * dMo_B(:,ij);
            
            %Hybrid velocity of frame B w.r.t. frame M
            BMV_MB(:,ij,tel)  = [MH_Bm(1:3,1:3,ij,tel) zeros(3,3); zeros(3,3) MH_Bm(1:3,1:3,ij,tel)] * [Bv_MB(:,ij); Bomg_MB(:,ij)];
        end
        BV_MB(:,1:length(Bv_MB),tel) = [Bv_MB;Bomg_MB];

        %Conveyor velocity direction and magnitude:
        conv_vel_dir(:,tel) = (MH_Cm(1:3,4,1090,tel)-MH_Cm(1:3,4,400,tel))/(norm(MH_Cm(1:3,4,1090,tel)-MH_Cm(1:3,4,400,tel)));
        conv_vel_mag(1,tel) = mean(vecnorm(dMo_C(:,400:1090)));

            
        %--------------- Determine the moment of release ----------------%
        %Find the peaks of the position data (height) to find when the box is released from the hand
            id_rest = 1090;
        if ObjStr == "Box005"
            iets = dMo_B(:,550:end); 
            id_imp(tel,:) = find(iets(3,:)==min(iets(3,(iets(3,1:end)>-2.5))))+550; %Index of the impact
            id_rel = id_imp(tel,:)-25;
        elseif ObjStr == "Box006"
            iets = dMo_B(:,550:end); 
            id_imp(tel,:) = find(iets(3,:)==min(iets(3,(iets(3,1:end)>-2.5))))+550; %Index of the impact
            id_rel = id_imp(tel,:)-25;
        end
%             figure('pos',[500 500 300 300]); plot(dMo_C(1,:)); hold on; 
%                 plot([400 1090],[mean(dMo_C(1,400:1090)) mean(dMo_C(1,400:1090))],'linewidth',3')
% 
%             figure('pos',[850 500 300 300]); plot(Mo_B(3,:,tel)); hold on;
%                 plot(id_rel,Mo_B(3,id_rel,tel),'o','markersize',10,'linewidth',2);
%                 plot(id_rest,Mo_B(3,id_rest,tel),'o','markersize',10,'linewidth',2);
%                 grid on;
%                 pause
%                 close all

        %----------------- Determine the impact plane -------------------%
        Imp_pln(:,:,tel) = MH_Cm(:,:,id_imp(tel),tel);
        
        %------------- Determine the relative release-pose --------------%
        Mo_B_rel(:,tel) = Mo_B(:,id_rel,tel);
        MR_B_rel(:,:,tel) = MH_Bm(1:3,1:3,id_rel,tel);
        MH_B_rel(:,:,tel) = [MR_B_rel(:,:,tel),Mo_B_rel(:,tel);zeros(1,3),1];
        
        %----------- Determine the release (hybrid) velocity ------------%
        BMV_MB_rel(:,tel) = BMV_MB(:,id_rel,tel);
        BV_MB_rel(:,tel)  = BV_MB(:,id_rel,tel);

        %Compensate for conveyor velocity
        BMX_B(:,:,1) = [MR_B_rel(:,:,tel) zeros(3); zeros(3) MR_B_rel(:,:,tel)];
        BMV_MB(:,tel) = BMX_B*BV_MB_rel(:,tel);
        CMV_MC = [conv_vel_dir(:,tel)*conv_vel_mag(:,tel); zeros(3,1)];

        %Remove conveyor velocity from box speed
        BMV_MB(:,tel) = BMV_MB(:,tel) - CMV_MC;
        BV_MB_rel(:,tel) = BMX_B\BMV_MB(:,tel);
        
                
        %----------- Determine the average relative rest-pose -----------%
        Mo_B_rest(:,tel) = Mo_B(:,id_rest,tel);
%         MR_Bmat_rest = MH_Bm(1:3,1:3,id_rest:id_rest+100,tel);
%         Rmean = MR_Bmat_rest(:,:,1);
%         for zz = 1:length(MR_Bmat_rest)
%             rpoints(:,zz) = vee(logm(Rmean\MR_Bmat_rest(:,:,zz)));
%         end
%         rmean = mean(rpoints,2);
%         
%         while norm(rmean) > th_Rmean
%             Rmean = Rmean*expm(hat(rmean));
%             for zz = 1:length(MR_Bmat_rest)
%                 rpoints(:,zz) = vee(logm(Rmean\MR_Bmat_rest(:,:,zz)));
%             end
%             rmean = mean(rpoints,2);
%         end
        MR_B_rest(:,:,tel) = MH_Bm(1:3,1:3,id_rest,tel); %Rmean*expm(hat(rmean));
        MH_B_rest(:,:,tel) = [MR_B_rest(:,:,tel), Mo_B_rest(:,tel); zeros(1,3),1];    
        
        id(tel,:) = [id_rel,id_rest];
    end
end

%% Write the release states to CSV file for Algoryx simulation
writeAGXinitstates(MH_B_rel(1:3,1:3,:),MH_B_rel(1:3,4,:),BV_MB_rel(1:3,:),BV_MB_rel(4:6,:),Imp_pln(1:3,1:3,:),Imp_pln(1:3,4,:),append('performance/',ObjStr,'_',Param,'/RunningConveyor/'));

%% Write the release states to CSV file for MuJoCo simulation
writeMuJoCoStates(MH_B_rel(1:3,1:3,:),MH_B_rel(1:3,4,:),BV_MB_rel)

%% Do the Matlab simulations of propagating the mean
for is = 1:tel
    %Obtain MATLAB results
    Ntimeidx = id(is,2)-id(is,1)+1; %Number of discrete time indices we want to run the simulation
    [MH_B_MATLAB,BV_MB_MATLAB] = BoxSimulator(MH_B_rel(1:3,4,is),MH_B_rel(1:3,1:3,is),BV_MB_rel(1:3,is),BV_MB_rel(4:6,is),MATLAB.(ObjStr).(Param)(1),MATLAB.(ObjStr).(Param)(2),MATLAB.(ObjStr).(Param)(3),Box,Imp_pln(1:3,1:3,is),Imp_pln(1:3,4,is),dt,Ntimeidx);
    for ii = 1:length(MH_B_MATLAB)
        MH_B_Matlab(:,:,ii,is) = MH_B_MATLAB{ii} + [zeros(3) (conv_vel_dir(:,tel)*conv_vel_mag(:,tel))*dt*ii; zeros(1,4)]; %Compensate back the conveyor speed
    end
    MH_B_restM(:,:,is) = MH_B_Matlab(:,:,ii,is);
end

%% Load the single AGX simulations
AGXResult_h5file = append('performance/',ObjStr,'_',Param,'/RunningConveyor/RC_',ObjStr,'_Validation_',Param,'_BoxTossBatch_result.hdf5');
AGXData = readH5(AGXResult_h5file);
Results = orderfields(AGXData.box);
fnAGX = fieldnames(Results);
%Run through the simulations
for ia = 1:length(fnAGX)
    ta = Results.(fnAGX{ia}).ds;    
    %Obtain AGX results (run for each simulation through results)
    for ii = 1:length(ta(:,1))
        MH_B_AGX(:,:,ii,ia) = [ta(ii,1) ta(ii,5) ta(ii,9) ta(ii,13); ta(ii,2) ta(ii,6) ta(ii,10) ta(ii,14); ta(ii,3) ta(ii,7) ta(ii,11) ta(ii,15); ta(ii,4) ta(ii,8) ta(ii,12) ta(ii,16)];
        BMV_MB_AGX(:,ii) = ta(ii,17:22)';
%         MX_B = [MH_B_AGX(1:3,1:3,ii,ia), zeros(3,3); zeros(3,3), MH_B_AGX(1:3,1:3,ii,ia)];
%         BV_MB_AGX(:,ii) = MX_B\BMV_MB_AGX(:,ii);
        MH_B_AGX(:,:,ii,ia) = MH_B_AGX(:,:,ii,ia) + [zeros(3) (conv_vel_dir(:,ia)*conv_vel_mag(:,ia))*dt*(ii-1); zeros(1,4)]; %Compensate back the conveyor speed
    end
    MH_B_restAGX(:,:,ia) = MH_B_AGX(:,:,(id(ia,2)-id(ia,1)),ia);
end

%% Plot the results of the single Matlab + AGX simulation in smaller figure
figure('rend','painters','pos',[500 500 150 195]);
    ha = tight_subplot(1,1,[.08 .07],[.16 .02],[0.23 0.05]);  %[gap_h gap_w] [lower upper] [left right]
    axes(ha(1));
    for ii =1:tel
    Ptrans = MH_B_rest(:,:,ii)*[Box.vertices.ds';ones(1,8)];
    PtransM = MH_B_restM(:,:,ii)*[Box.vertices.ds';ones(1,8)];
    PtransA = MH_B_restAGX(:,:,ii)*[Box.vertices.ds';ones(1,8)];
    x1 = [Ptrans(1,1:4) Ptrans(1,1)];
    y1 = [Ptrans(2,1:4) Ptrans(2,1)];
    x2 = [PtransM(1,1:4) PtransM(1,1)];
    y2 = [PtransM(2,1:4) PtransM(2,1)];
    x3 = [PtransA(1,1:4) PtransA(1,1)];
    y3 = [PtransA(2,1:4) PtransA(2,1)];
    
    fill(x1,y1,color.Meas); %Measured box 
    grid on; hold on;
    fill(x2,y2,color.Matlab);      %Matlab box
    fill(x3,y3,color.Algoryx); %AGX box
    xlabel('$(^M\mathbf{o}_B)_x$');
    ylabel('$(^M\mathbf{o}_B)_y$');
    axis equal
    if ObjStr == "Box005"
        axis([-0.1 0.6 -0.4 0.6]); 
    elseif ObjStr == "Box006"
        axis([0 0.7 -0.4 0.6]); 
    end
    if doSave; fig = gcf; fig.PaperPositionMode = 'auto'; fig_pos = fig.PaperPosition; fig.PaperSize = [fig_pos(3) fig_pos(4)];
        print(fig,append('figures/RestPose/',ObjStr,'_',Param,'/RunningConveyor/Rest-Pose_',sprintf('%.2d.pdf',ii)),'-dpdf','-vector'); end
%     pause();
    hold off;
    end

%% Compute the errors of the rest-orientation and rest-position
for ii =1:tel
    E_rot_M(ii,:) = rad2deg(rotm2eul(MH_B_rest(1:3,1:3,ii)\MH_B_restM(1:3,1:3,ii)));
    E_rot_A(ii,:) = rad2deg(rotm2eul(MH_B_rest(1:3,1:3,ii)\MH_B_restAGX(1:3,1:3,ii)));
    E_pos_M(ii,:) = (MH_B_rest(1:3,4,ii)-MH_B_restM(1:3,4,ii))';
    E_pos_A(ii,:) = (MH_B_rest(1:3,4,ii)-MH_B_restAGX(1:3,4,ii))';
end

e_pos_M = mean(vecnorm(E_pos_M(:,1:2)'));
e_rot_M = mean(abs(E_rot_M(:,1)));
e_pos_A = mean(vecnorm(E_pos_A(:,1:2)'));
e_rot_A = mean(abs(E_rot_A(:,1)));

std_pos_M = std(vecnorm(E_pos_M(:,1:2)'));
std_rot_M = std(abs(E_rot_M(:,1)));

std_pos_A = std(vecnorm(E_pos_A(:,1:2)'));
std_rot_A = std(abs(E_rot_A(:,1)));

%% Plot single trajectory in space to demonstrate simulation
% Plotting options For plotting the contact surface
ws    = 2.5;  %Width of the contact surface             [m]
ls    = 1;  %Length of the contact surface           [m]
surfacepoints = [0.5*ws -0.5*ws -0.5*ws 0.5*ws 0.5*ws; -0.5*ls -0.5*ls 0.5*ls 0.5*ls -0.5*ls; 0 0 0 0 0;];


plotnr = 11;
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
        spoints = Imp_pln(1:3,1:3,plotnr)*surfacepoints+Imp_pln(1:3,4,plotnr); %Transform the vertices according to position/orientation of the surface
        table3 = fill3(spoints(1,1:4),spoints(2,1:4),spoints(3,1:4),1);hold on;
        set(table3,'FaceColor',0.8*[1 1 1],'FaceAlpha',1);
        
        %Plot the origin of the contact surface with its unit vectors
        tip = [MH_Cm(1:3,4,ii)+0.3*MH_Cm(1:3,1,ii) MH_Cm(1:3,4,ii)+0.3*MH_Cm(1:3,2,ii) MH_Cm(1:3,4,ii)+0.3*MH_Cm(1:3,3,ii)];
        plot3([Mo_C(1,ii) tip(1,1)],[Mo_C(2,ii) tip(2,1)],[Mo_C(3,ii) tip(3,1)],'r'); hold on
        plot3([Mo_C(1,ii) tip(1,2)],[Mo_C(2,ii) tip(2,2)],[Mo_C(3,ii) tip(3,2)],'g');
        plot3([Mo_C(1,ii) tip(1,3)],[Mo_C(2,ii) tip(2,3)],[Mo_C(3,ii) tip(3,3)],'b');
        
        %Plot the origin of the world coordinate frame
        tip = [0.3*[1;0;0] 0.3*[0;1;0] 0.3*[0;0;1]];
        plot3([0 tip(1,1)],[0 tip(2,1)],[0 tip(3,1)],'r'); hold on
        plot3([0 tip(1,2)],[0 tip(2,2)],[0 tip(3,2)],'g');
        plot3([0 tip(1,3)],[0 tip(2,3)],[0 tip(3,3)],'b');

        grid on;axis equal;
        axis([-1 1 -0.9 0.51 -0.05 0.5]);
        xlabel('x [m]');
        ylabel('y [m]');
        zlabel('z [m]');
%         view(-118,16);
%         view(-118,27);
%         view(-315,31);
        view(-37,27);
%         text(1,0.6,0.4,append('Frame:',sprintf('%d',ii-(id(plotnr,1)-1))));
%         L1 = legend([g1],'Measured','Matlab','Algoryx','NumColumns',3,'location','northeast');
%         L1.Position(2) = 0.90;
%         L1.Position(1) = 0.52-(L1.Position(3)/2);
        drawnow
        hold off
%         pause()        
%         f = gcf;
%         exportgraphics(f,append('Frame_',sprintf('%.2d.jpg',ii-(id(plotnr,1)-1))),'Resolution',500)
    end

%% Plot single trajectory in space to demonstrate simulation (paper figure)
% Plotting options For plotting the contact surface
ws    = 1;  %Width of the contact surface             [m]
ls    = 1;  %Length of the contact surface           [m]
surfacepoints = [0.5*ws -0.5*ws -0.5*ws 0.5*ws 0.5*ws; -0.5*ls -0.5*ls 0.5*ls 0.5*ls -0.5*ls; 0 0 0 0 0;];


plotnr = 20;
%Plot the trajectory of the box
figure('pos',[500 500 500 300]);
    ha = tight_subplot(1,1,[0.0 0.0],[0.0 -0.2],[0.0 0.0]);  %[gap_h gap_w] [lower upper] [left right]
    axes(ha(1));
    for ii=[806 821 836 851 866 881 896 911  941  971  1031]
        
        %plot Measured box
        g1 = plotBox(MH_Bm(:,:,ii,plotnr),Box,color.Meas,0);hold on;
        
        %Plot MATLAB box
        g2 = plotBox(MH_B_Matlab(:,:,ii-(id(plotnr,1)-1),plotnr),Box,color.Matlab,0); hold on;  

        %Plot new AGX box results
        g3 = plotBox(MH_B_AGX(:,:,ii-(id(plotnr,1)-1),plotnr),Box,color.Algoryx,0);hold on;
    end

        %Plot the conveyor C
        spoints = Imp_pln(1:3,1:3,plotnr)*surfacepoints+Imp_pln(1:3,4,plotnr)-[0.2;0;0]; %Transform the vertices according to position/orientation of the surface
        table3 = fill3(spoints(1,1:4),spoints(2,1:4),spoints(3,1:4),1);hold on;
        set(table3,'FaceColor',[56 53 48]/255,'FaceAlpha',1);
        grid on;axis equal; axis off; camproj('perspective')
        view(-142,30);
        drawnow
        hold off
        if doSave; f = gcf; exportgraphics(f,'figures/impact_sequence_RC.png','Resolution',1500);  end

