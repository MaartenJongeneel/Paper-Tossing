clearvars; close all; set(groot,'defaulttextinterpreter','latex'); set(groot,'defaultAxesTickLabelInterpreter','latex'); set(groot,'defaultLegendInterpreter','latex');
addpath(genpath('readH5')); addpath('data');
%% Trajectory based Parameter Identification
% Trajectory based parameter identification to find the coefficient of
% friction and coefficient of normal restitution. We consider here a set of
% long-horizon tosses and find the parameters that minimize the error in
% position and orientation over the full trajectory
%% Load the data
Data = readH5('211224_ManualTossesBox5.h5');
%% Constants
fps   = 120;       %Frequency of the recording (I was stupid to not put it on 360..)
dt    = 1/fps;     %Timestep of the recording 
mu    = 0:0.05:1;  %Define the parameter range of mu for which you want to run simulations
eN    = 0:0.05:1;  %Define the parameter range of eN for which you want to run simulations
N_pos = 20;        %Number of consecutive points where the error is low
th_Rmean = 1e-5;   %Threshold rotation mean

mu = 0:0.05:1;      %Define the parameter range of mu for which you want to run simulations
eN = 0:0.05:1;      %Define the parameter range of eN for which you want to run simulations
eT = 0;             %Define the parameter range of eT for which you want to run simulations

evalMatlab = true;
%% Loop through the data
tel = 0;
fn = fieldnames(Data);
for ii = 1:length(fn)
    if startsWith(fn{ii},'Rec')
        tel = tel+1;
        %Get the data from the file
        Mocap = Data.(fn{ii}).SENSOR_MEASUREMENT.Mocap;
        FH_B = Mocap.POSTPROCESSING.Box5.transforms.ds;
        FH_C = Mocap.POSTPROCESSING.CS_200.transforms.ds;
        
        %Get the transform of the Base w.r.t. Motive origin
        i_bd_Box5_R   = contains(string(Mocap.datalog.ds.Properties.VariableNames),"Box5_R");
        i_bd_Box5_P   = contains(string(Mocap.datalog.ds.Properties.VariableNames),"Box5_P");
        
        %Rewrite the data into mat structures
        Nsamples = length(FH_B);
        for jj = 1:Nsamples
            FH_Bm(:,:,jj) = FH_B{jj};
            FH_Cm(:,:,jj,tel) = FH_C{jj};
            CH_Bm(:,:,jj,tel) = FH_C{jj}\FH_B{jj};
            MH_Bm(:,:,jj,tel) = makehgtform('translate', table2array(Mocap.datalog.ds(jj,i_bd_Box5_P)))* ... %translation
                quat2tform(table2array(Mocap.datalog.ds(jj,i_bd_Box5_R))); %Rotation
            MH_Bm(:,:,jj,tel) = [Rx(90) zeros(3,1); zeros(1,3),1]*MH_Bm(:,:,jj,tel);
        end
        
        %Few definitions from data:
        Fo_B(:,1:length(FH_Bm(1:3,4,:)),tel) = squeeze(FH_Bm(1:3,4,:));
        Fo_C(:,1:length(FH_Cm(1:3,4,:,tel)),tel) = squeeze(FH_Cm(1:3,4,:,tel));
        Co_B(:,1:length(CH_Bm(1:3,4,:,tel)),tel) = squeeze(CH_Bm(1:3,4,:,tel));  
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
        %Find the peaks of the position data (height) to find when the box
        %is released from the hand
        
        t = find(vecnorm(dMo_B(:,100:end))<0.02); %Find the indices where difference in rel. pos. is small
        x = diff(t)==1;
        f = find([false,x]~=[x,false]);
        g = find(f(2:2:end)-f(1:2:end-1)>=N_pos,1,'first');
        id_rest = t(f(2*g-1))+99; % First t followed by >=N_pos consecutive numbers
        
        if id_rest > 131
            [pks,id_rel] = findpeaks(Mo_B(3,id_rest-130:id_rest,tel),'MinPeakHeight',0.12,'MinPeakProminence',0.05,'MinPeakWidth',20);
            id_rel = id_rel+(id_rest-131);
        else
            [pks,id_rel] = findpeaks(Mo_B(3,:,tel),'MinPeakHeight',0.12,'MinPeakProminence',0.05,'MinPeakWidth',20);
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
        
        id(tel,:) = [id_rel,id_rest];
    end
end
%% Do the Matlab simulations from the release states computed above
load('box5.mat')
if evalMatlab    
    %Obtain the vectors of mu, eN, and eT for which we run the
    %parameter identification. If evalAlgoryx is true, the vectors for mu1,
    %eN1, and eT1 are defined there. Otherwise, it will grab the defaults.
    muvec = repmat(mu,1,length(eN)*length(eT));
    eNvec = repmat(repelem(eN,length(mu)),1,length(eT));
    eTvec = repelem(eT,length(mu)*length(eN));

    for is = 2:4%tel
        for ip = 1:(length(mu)*length(eN)*length(eT)) %For all parameters 
            %Obtain MATLAB results
            Ntimeidx = id(is,2)-id(is,1)+1; %Number of discrete time indices we want to run the simulation
            [MH_B_MATLAB,BV_MB_MATLAB] = BoxSimulator(MH_B_rel(1:3,4,is),MH_B_rel(1:3,1:3,is),BV_MB_rel(1:3,is),BV_MB_rel(4:6,is),eNvec(ip),eTvec(ip),muvec(ip),box5,eye(3),zeros(3,1),1/120,Ntimeidx);
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
            E_MATLAB(mu_i,eN_i,eT_i,is) = 1/Ntimeidx * (sum(e_pos) + sum(e_rot));            
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
%% Evaluate the 