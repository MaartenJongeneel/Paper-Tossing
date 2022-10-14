clearvars; close all; set(groot,'defaulttextinterpreter','latex'); set(groot,'defaultAxesTickLabelInterpreter','latex'); set(groot,'defaultLegendInterpreter','latex');
addpath(genpath('readH5')); addpath('data');
%% Trajectory based Parameter Identification
% Trajectory based parameter identification to find the coefficient of
% friction and coefficient of normal restitution. We consider here a set of
% long-horizon tosses and find the parameters that minimize the error in
% position and orientation over the full trajectory
%% Load the data
data = readH5('220920_Box005_ParamID_Traj.h5');
% data = readH5('220920_Box006_ParamID_Traj.h5');

%% Constants
mu    = 0:0.05:1;  %Define the parameter range of mu for which you want to run simulations
eN    = 0:0.05:1;  %Define the parameter range of eN for which you want to run simulations
eT = 0;            %Define the parameter range of eT for which you want to run simulations
N_pos = 20;        %Number of consecutive points where the error is low
th_Rmean = 1e-5;   %Threshold rotation mean
evalAlgoryx = true;
evalMatlab = true;
doSave = false;

color.Matlab = [237 176 33]/255;
color.Algoryx = [77 191 237]/255;
color.Meas = [128 128 128]/255;

ObjStr = "Box005"; %The object for which you want to do paramID
ImpPln = "GroundPlane001"; %"ConveyorPart001 GroundPlane001";

%% If Algoryx is used, load the simulation results
AGXResult_h5file = append('paramID/',ObjStr,'_Traj/',ObjStr,'_ParamID_Traj_BoxTossBatch_result.hdf5');

if evalAlgoryx    
    AGXData = readH5(AGXResult_h5file);
    Results = orderfields(AGXData.box);
end
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
        Nsamples = length(MH_B);
        if isfield(Mocap.POSTPROCESSING,ImpPln)
            MH_C = Mocap.POSTPROCESSING.(ImpPln).transforms.ds;
        else
            for kk=1:Nsamples; MH_C{kk} = [eye(3) [0; 0; 0.01]; zeros(1,3) 1]; end
        end
                      
        %Rewrite the data into mat structures        
        for jj = 1:Nsamples
            MH_Bm(:,:,jj,tel) = MH_B{jj};
            MH_Cm(:,:,jj,tel) = MH_C{jj};
        end
        
        %Few definitions from data:
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

        
        if ObjStr == "Box005"
            t = find(vecnorm(dMo_B(:,4000:end))<0.02); %Find the indices where difference in rel. pos. is small
            x = diff(t)==1;
            f = find([false,x]~=[x,false]);
            g = find(f(2:2:end)-f(1:2:end-1)>=N_pos,1,'first');
            id_rest = t(f(2*g-1))+3999; % First t followed by >=N_pos consecutive numbers

            [pks,id_rel] = findpeaks(Mo_B(3,:,tel),'MinPeakHeight',0.12);%,'MinPeakProminence',0.05,'MinPeakWidth',10);
            id_rel = id_rel(end);

%             figure; plot(((id_rel-20):id_rest+20)*dt,Mo_B(3,(id_rel-20):id_rest+20,tel)); hold on;
%             plot(id_rel*dt,Mo_B(3,id_rel,tel),'o','markersize',10,'linewidth',2);
%             plot(id_rest*dt,Mo_B(3,id_rest,tel),'o','markersize',10,'linewidth',2);
%             grid on;
%             xlim([id_rel-20,id_rest+20]*dt);
%             xlabel('Time [s]');
%             ylabel('$(^M\mathbf{o}_B)_z$ [m]');
%             pause
%             close all
        end

        if ObjStr == "Box006"
            t = find(vecnorm(dMo_B(:,170:end))<0.02); %Find the indices where difference in rel. pos. is small
            x = diff(t)==1;
            f = find([false,x]~=[x,false]);
            g = find(f(2:2:end)-f(1:2:end-1)>=N_pos,1,'first');
            id_rest = t(f(2*g-1))+169; % First t followed by >=N_pos consecutive numbers
            if isempty(id_rest) || id_rest> 900; id_rest = 900; end

            [pks,id_rel] = findpeaks(Mo_B(3,200:end,tel),'MinPeakHeight',0.105,'MinPeakWidth',10);
            id_rel = id_rel+199;
            if isempty(id_rel)
                id_rel = find(dMo_B(3,:)==min(dMo_B(3,:)))-50;
            end
            id_rel = id_rel(1);

                        figure; plot(Mo_B(3,:,tel)); hold on;
                            plot(id_rel,Mo_B(3,id_rel,tel),'o','markersize',10,'linewidth',2);
                            plot(id_rest,Mo_B(3,id_rest,tel),'o','markersize',10,'linewidth',2);
                            grid on;
                            pause
                            close all
        end
        
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
        MH_Ca(:,:,tel) = MH_Cm(:,:,1,tel);
        
        id(tel,:) = [id_rel,id_rest];
    end
end
%% Write the release states to CSV file for Algoryx simulation
writeAGXinitstates(MH_B_rel(1:3,1:3,:),MH_B_rel(1:3,4,:),BV_MB_rel(1:3,:),BV_MB_rel(4:6,:),MH_Ca(1:3,1:3,:),MH_Ca(1:3,4,:),append('paramID/',ObjStr,'_Traj/'));

%% -------------------- Evaluate the impacts Algoryx --------------------%%
if evalAlgoryx
    % Because we obtain the values for mu, eN, and eT from Algoryx simulations, we delete the values here
    clear mu eN eT
    
    for is = 1:tel
        %Initialize the AGX data import
        mu_i = 1; eN_i = 1; eT_i = 1;
        
        %Extract information from AGX simulation result on what COF, CONR, and COTR were used
        mu(mu_i) = str2double(strtrim(extractAfter(extractBefore(extractAfter(string(AGXData.model.BoxTossBatch.ds),"friction"),"step"),"default:")));
        eN(eN_i) = str2double(strtrim(extractAfter(extractBefore(extractAfter(string(AGXData.model.BoxTossBatch.ds),"normal_restitution"),"step"),"default:")));
        eT(eT_i) = str2double(strtrim(extractAfter(extractBefore(extractAfter(string(AGXData.model.BoxTossBatch.ds),"tangential_restitution"),"step"),"default:")));
        
        fn = fieldnames(Results);
        num_par = (length(fn)/(tel));
        cnt = 0; %Counter to count what parameter combination we are considering (0 <= cnt <= num_par)
        %For the current impact event, we are now looping through the parameters
        for ip = ((num_par*(is-1))+1):((num_par*(is))) %For all parameters
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
            NtimeidxA = length(t(:,1));
            for ii = 1:NtimeidxA
                MH_B_AGX(:,:,ii,cnt) = [t(ii,1) t(ii,5) t(ii,9) t(ii,13); t(ii,2) t(ii,6) t(ii,10) t(ii,14); t(ii,3) t(ii,7) t(ii,11) t(ii,15); t(ii,4) t(ii,8) t(ii,12) t(ii,16)];
            end

            Mo_B_meas = Mo_B(:,id(is,1):id(is,1)+NtimeidxA,is);                %Measured position data
            MR_B_meas = cat(3,MH_Bm(1:3,1:3,id(is,1):id(is,1)+NtimeidxA,is));  %Measured Rotation data
            Mo_B_A = squeeze(MH_B_AGX(1:3,4,:,cnt));                       %Simulated position data
            MR_B_A = MH_B_AGX(1:3,1:3,:,cnt);                              %Simulated Rotation data

            %Compute the cost
            for it = 1:NtimeidxA
                e_pos_A(it) = norm(Mo_B_meas(:,it)-Mo_B_A(:,it));
                e_rot_A(it) = norm(logm(MR_B_meas(:,:,it)\MR_B_A(:,:,it))); 
            end

            %Cost function
            E_AGX(mu_i,eN_i,eT_i,is) = 1/Ntimeidx * (1/Box.dimensions.ds(1)*sum(e_pos_A) + sum(e_rot_A));             
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
%% Do the Matlab simulations from the release states computed above
if evalMatlab    
    %Obtain the vectors of mu, eN, and eT for which we run the
    %parameter identification. If evalAlgoryx is true, the vectors for mu1,
    %eN1, and eT1 are defined there. Otherwise, it will grab the defaults.
    muvec = repmat(mu,1,length(eN)*length(eT));
    eNvec = repmat(repelem(eN,length(mu)),1,length(eT));
    eTvec = repelem(eT,length(mu)*length(eN));

    for is = 1:tel
        for ip = 1:(length(mu)*length(eN)*length(eT)) %For all parameters 
            %Obtain MATLAB results
            Ntimeidx = id(is,2)-id(is,1)+1; %Number of discrete time indices we want to run the simulation
            [MH_B_MATLAB,BV_MB_MATLAB] = BoxSimulator(MH_B_rel(1:3,4,is),MH_B_rel(1:3,1:3,is),BV_MB_rel(1:3,is),BV_MB_rel(4:6,is),eNvec(ip),eTvec(ip),muvec(ip),Box,MH_Ca(1:3,1:3,tel),MH_Ca(1:3,4,tel),dt,Ntimeidx);
            MH_B_M = cat(3,MH_B_MATLAB{:});

            Mo_B_meas = Mo_B(:,id(is,1):id(is,2),is);                %Measured position data
            MR_B_meas = cat(3,MH_Bm(1:3,1:3,id(is,1):id(is,2),is));  %Measured Rotation data
            Mo_B_M = squeeze(MH_B_M(1:3,4,:));                       %Simulated position data
            MR_B_M = MH_B_M(1:3,1:3,:);                              %Simulated Rotation data

            %Current used index of the parameters
            mu_i = find(muvec(ip) == mu);  eN_i = find(eNvec(ip) == eN); eT_i = find(eTvec(ip) == eT);
            
            %Compute the cost
            for it = 1:Ntimeidx
                e_pos(it) = norm(Mo_B_meas(:,it)-Mo_B_M(:,it));
                e_rot(it) = norm(logm(MR_B_meas(:,:,it)\MR_B_M(:,:,it))); 
            end

            %Cost function
            E_MATLAB(mu_i,eN_i,eT_i,is) = 1/Ntimeidx * (1/Box.dimensions.ds(1)*sum(e_pos) + sum(e_rot));            
        end

        CurrentE_MATLAB = E_MATLAB(:,:,:,is);
        [~,optMATLAB_idx] = min(CurrentE_MATLAB(:));
        
        [a1,b1,c1]=ind2sub(size(E_MATLAB),optMATLAB_idx);
        if length(a1)==1 && length(b1)==1 && length(c1)==1
            MATLABmu_opt(is) = mu(a1);
            MATLABeN_opt(is) = eN(b1);
            MATLABeT_opt(is) = eT(c1);
        end
    end
end


%% Evaluate the parameters
SUMMATLAB = (1/length(E_MATLAB(1,1,1,:)))*sum(E_MATLAB(:,:,:,:),4);
SUMAGX = (1/length(E_AGX(1,1,1,:)))*sum(E_AGX(:,:,:,:),4);















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

%Plot the cost of MATLAB simulation
figure('rend','painters','pos',[pp{3,5} 0.7*sizex sizey]);
    ha = tight_subplot(1,1,[.08 .07],[.18 .1],[0.12 0.03]);  %[gap_h gap_w] [lower upper] [left right]
    axes(ha(1));
    surf(eN,mu,SUMMATLAB); 
    axis square; 
    view(-40,15); 
    xlim([0 1]);
    ylim([0 1]);
    zlim([0 12.5]);
    xlabel('$e_N$');
    ylabel('$\mu$');
    zlabel('$\frac{1}{N}\sum_{i=1}^NL_{traj}(\mu,e_N)_i$');

    if doSave; fig = gcf; fig.PaperPositionMode = 'auto'; fig_pos = fig.PaperPosition; fig.PaperSize = [fig_pos(3) fig_pos(4)];
    print(fig,append('figures/Traj_Based_Cost_Matlab'),'-dpdf','-vector'); end

%Plot the cost of Algoryx simulation
figure('rend','painters','pos',[pp{3,5} 0.7*sizex sizey]);
    ha = tight_subplot(1,1,[.08 .07],[.18 .1],[0.12 0.03]);  %[gap_h gap_w] [lower upper] [left right]
    axes(ha(1));
    surf(eN,mu,SUMAGX); 
    axis square; 
    view(-40,15); 
    xlim([0 1]);
    ylim([0 1]);
    zlim([0 12.5]);
    xlabel('$e_N$');
    ylabel('$\mu$');
    zlabel('$\frac{1}{N}\sum_{i=1}^NL_{traj}(\mu,e_N)_i$');

    if doSave; fig = gcf; fig.PaperPositionMode = 'auto'; fig_pos = fig.PaperPosition; fig.PaperSize = [fig_pos(3) fig_pos(4)];
    print(fig,append('figures/Traj_Based_Cost_Algoryx'),'-dpdf','-vector'); end

%% Plot single trajectory in space to demonstrate simulation
% Plotting options For plotting the contact surface
plotnr = 100;

ws    = 1.5;  %Width of the contact surface             [m]
ls    = 1.5;  %Length of the contact surface           [m]
surfacepoints = [0.5*ws -0.5*ws -0.5*ws 0.5*ws 0.5*ws; -0.5*ls -0.5*ls 0.5*ls 0.5*ls -0.5*ls; 0 0 0 0 0;];
FR_C = MH_Cm(1:3,1:3,1,plotnr); 
Fo_C = MH_Cm(1:3,4,1,plotnr); 
spoints = FR_C*surfacepoints +Fo_C; %Transform the vertices according to position/orientation of the surface


%Plot the trajectory of the box
figure('pos',[500 500 500 300]);
    for ii=id(plotnr,1):5:id(plotnr,1)+(id(plotnr,2)-id(plotnr,1))-1
        
        %plot Measured box
        g1 = plotBox(MH_Bm(:,:,ii,plotnr),Box,color.Meas,0);hold on;
        
        %Plot MATLAB box
        g2 = plotBox(MH_B_M(:,:,ii-(id(plotnr,1)-1),1),Box,color.Matlab,0); hold on;     

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