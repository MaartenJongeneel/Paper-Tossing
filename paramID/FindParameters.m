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
impact_data = "paramID/impact_data/220428_Box006_impacts.mat";
AGXResult_h5file = "paramID/AGX_results/Box006_paramID/BoxImpactBox006_BoxTossBatch_result.hdf5";
evalAlgoryx = true;
evalMatlab = true;
evalMuJoCo = false;


dt        = 1/360;  %OptiTrack time step
g         = 9.81;   %Gravitational constant                    [m/s^2]
freq      = 1/dt;   %OptiTrack sample frequency
w_ext     = 5;      %extension window
endframe  = 11;     %Determine to what frame to run MATLAB
weight    = [1 1 1 0.1 0.1 0.1]; %Weight of the cost function

mu = 0:0.05:1;      %Define the parameter range of mu for which you want to run simulations
eN = 0:0.05:1;      %Define the parameter range of eN for which you want to run simulations
eT = 0;             %Define the parameter range of eT for which you want to run simulations
doPlot    = false;
doSave    = false;

color.Green = [0.4660 0.6740 0.1880];
color.Blue = [0 0.4470 0.7410];
color.Red = [0.8500 0.3250 0.0980]; 
color.GreenLight = [0.4660 0.6740 0.1880 0.5];
color.BlueLight = [0 0.4470 0.7410 0.5];
color.RedLight = [0.8500 0.3250 0.0980 0.5];
color.Matlab = [31 62 77]/255;
color.Algoryx = [187 123 59]/255;
color.Meas = [135 134 132]/255;

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

%Some parameters
k = (-w_ext:w_ext)';        % Crude indices, comply with OptiTrack freq. 
kf = (-w_ext:0.01:w_ext)';  % Refined indices for fitting purposes

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

writeAGXinitstates(MH_B_rel(1:3,1:3,:),MH_B_rel(1:3,4,:),BV_MB_rel(1:3,:),BV_MB_rel(4:6,:),MH_C_rel(1:3,1:3,:),MH_C_rel(1:3,4,:),'paramID/AGX_init_states');

%% Write the release states to CSV file for MuJoCO simulations
writeMuJoCoStates(MH_B_rel(1:3,1:3,:),MH_B_rel(1:3,4,:),BV_MB_rel)
%% -------------------- Evaluate the impacts Algoryx --------------------%%
if evalAlgoryx
    % Because we obtain the values for mu, eN, and eT from Algoryx
    % simulations, we delete the values here
    clear mu eN eT
    
    for is = 1:maxImpactEvel
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
    %Obtain the vectors of mu, eN, and eT for which we run the
    %parameter identification. If evalAlgoryx is true, the vectors for mu1,
    %eN1, and eT1 are defined there. Otherwise, it will grab the defaults.
    muvec = repmat(mu,1,length(eN)*length(eT));
    eNvec = repmat(repelem(eN,length(mu)),1,length(eT));
    eTvec = repelem(eT,length(mu)*length(eN));
            
    for is = 1:maxImpactEvel       
        %Load impact data from struct
        MH_B_meas  = impacts.MH_B(imp_sel(is),:);
        CH_B_meas  = impacts.CH_B(imp_sel(is),:);
        BV_CBaf    = impacts.BV_CBaf(imp_sel(is),:);
        BCV_CBef   = cell2mat(impacts.BCV_CBef(imp_sel(is),:));

        MH_C       = impacts.MH_C(imp_sel(is),:);
        MR_C       = MH_C{6}(1:3,1:3); %Take rotation at moment of impact
        Mo_C       = MH_C{6}(1:3,4);   %Take position at moment of impact
        
        %Load box parameters
        box.B_M_B = impacts.box.M{imp_sel(is)};    %Inertia tensor
        box.vertices = impacts.box.V{imp_sel(is)}; %Vertices
        box.mass = impacts.box.M{imp_sel(is)}(1);  %Mass
        
        %Vertices
        Bp = box.vertices;
                        
        for ip = 1:(length(mu)*length(eN)*length(eT)) %For all parameters   
            %Run the Matlab simulation
            [MH_B_MATLAB,BV_MB_MATLAB] = BoxSimulator(MH_B_meas{1}(1:3,4),MH_B_meas{1}(1:3,1:3),BV_CBaf{1}(1:3,1),BV_CBaf{1}(4:6,1),eNvec(ip),eTvec(ip),muvec(ip),box,MR_C,Mo_C,dt,endframe);
            MH_B_M(:,:,:,ip) = cat(3,MH_B_MATLAB{:});
            BV_MB_M(:,:,ip) = BV_MB_MATLAB;

            for ii = 1:endframe
                BCV_MB_M(:,ii,ip) = [MR_C\MH_B_M(1:3,1:3,ii,ip)*BV_MB_M(1:3,ii,ip); MR_C\MH_B_M(1:3,1:3,ii,ip)*BV_MB_M(4:6,ii,ip)];
                for iv = 1:length(Bp)                    
                    Cp_m(:,iv,ii)   = CH_B_meas{ii}(1:3,4)+CH_B_meas{ii}(1:3,1:3)*(Bp(:,iv));
                    Cp_M(:,iv,ii,ip) = MR_C\(MH_B_MATLAB{ii}(1:3,4) - Mo_C)+(MR_C)\MH_B_M(1:3,1:3,ii,ip)*(Bp(:,iv));
                    dCp_M(:,iv,ii,ip) = [MR_C\MH_B_M(1:3,1:3,ii,ip) -MR_C\MH_B_M(1:3,1:3,ii,ip)*hat(Bp(:,iv))]*BV_MB_M(:,ii,ip);
                end
            end

            mu_i = find(muvec(ip) == mu);  eN_i = find(eNvec(ip) == eN); eT_i = find(eTvec(ip) == eT);
            
            %Compute the cost
            E_MATLAB(mu_i,eN_i,eT_i,is)= norm(diag(weight)*(BCV_CBef(:,indx_c)-BCV_MB_M(:,indx_c,ip))); 
        end
        
        CurrentE_MATLAB = E_MATLAB(:,:,:,is);
        [~,optMATLAB_idx] = min(CurrentE_MATLAB(:));
        
        [a1,b1,c1]=ind2sub(size(E_MATLAB),optMATLAB_idx);
        if length(a1)==1 && length(b1)==1 && length(c1)==1
            MATLABmu_opt(is) = mu(a1);
            MATLABeN_opt(is) = eN(b1);
            MATLABeT_opt(is) = eT(c1);
        end
        MATLABwi(is) = E_MATLAB(a1,b1,c1,is);
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

%Box 5 indices
% idx = [24,93,35,19,86,107,11,67,3,103,94,76,91,83,42,78,114,72,77,23,32,49,69,58,100,57,113,21,2,81,31,55,116,108,104,51,38,18,14,102,75,28,63,99,17,7,112,54,34,98,22,73,90,40,33,43,74,110,46,89,44,79,26,92,62,50,15,27,84,4,5,82,10,65,29,66,13,80,47,36,59,111,56,53,64,16,1,106,88,109,95,60,97,30,9,85,48,20,39,96,12,25,68,8,37,101,6,52,61,70,105,71,45,87,41,115];

if evalMatlab
    y1 = mmu(idx);
    y2 = meN(idx); %datasample(meN,length(meN),'Replace',false);
    ywm = mwi(idx);
    
    for ii = 1:length(mmu)
        meanM_mu(ii) = 1/(sum(ywm(1:ii)))*ywm(1:ii)*y1(1:ii)'; 
        stdM_mu(ii) = 1/sqrt(ii)*sqrt(1/sum(ywm(1:ii))*ywm(1:ii)*((y1(1:ii) - meanM_mu(ii)).^2)');
        
        meanM_eN(ii) = 1/(sum(ywm(1:ii)))*ywm(1:ii)*y2(1:ii)';
        stdM_eN(ii) = 1/sqrt(ii)*sqrt(1/sum(ywm(1:ii))*ywm(1:ii)*((y2(1:ii) - meanM_eN(ii)).^2)'); 
    end
end

if evalAlgoryx
    y3 = amu(idx); %datasample(amu,length(amu),'Replace',false);
    y4 = aeN(idx); %datasample(aeN,length(aeN),'Replace',false);
    ywa = awi(idx); 
    
    for ii = 1:length(amu)
        meanA_mu(ii) = 1/(sum(ywa(1:ii)))*ywa(1:ii)*y3(1:ii)';
        stdA_mu(ii) = 1/sqrt(ii)*sqrt(1/sum(ywa(1:ii))*ywa(1:ii)*((y3(1:ii) - meanA_mu(ii)).^2)');
        
        meanA_eN(ii) = 1/(sum(ywa(1:ii)))*ywa(1:ii)*y4(1:ii)';
        stdA_eN(ii) = 1/sqrt(ii)*sqrt(1/sum(ywa(1:ii))*ywa(1:ii)*((y4(1:ii) - meanA_eN(ii)).^2)');
    end
end

%Select the indices that indicate the time of pre- and post-impact velocity
is = 136;
Ib = impacts.indx(imp_sel(is),3) - impacts.indx(imp_sel(is),2);
Ic = impacts.indx(imp_sel(is),4) - impacts.indx(imp_sel(is),2);

indx_b = impacts.indx(imp_sel(is),3) - impacts.indx(imp_sel(is),2)+6;
indx_c = impacts.indx(imp_sel(is),4) - impacts.indx(imp_sel(is),2)+6;

dCp_meas   = cell2mat(impacts.dCo_p(imp_sel(is),:));
BV_CB      = impacts.BV_CB(imp_sel(is),:);
BV_CBef    = impacts.BV_CBef(imp_sel(is),:);
BCV_CB     = cell2mat(impacts.BCV_CB(imp_sel(is),:));
BCV_CBaf   = cell2mat(impacts.BCV_CBaf(imp_sel(is),:));
BCV_CBef   = cell2mat(impacts.BCV_CBef(imp_sel(is),:));
V_impact   = impacts.indx(imp_sel(is),1);

if evalMatlab
    %Plot the evolution of mu MATLAB
    figure('rend','painters','pos',[pp{3,1} sizex sizey]);
    ha = tight_subplot(1,1,[.08 .07],[.18 .1],[0.12 0.03]);  %[gap_h gap_w] [lower upper] [left right]
    axes(ha(1));
    plot(meanM_mu,'color',[0 0.4470 0.7410],'linewidth',1.2); hold on; grid on;
    plot(2:length(meanM_mu),meanM_mu(2:end)+2*stdM_mu(2:end),'--','color','k','linewidth',0.1);
    plot(2:length(meanM_mu),meanM_mu(2:end)-2*stdM_mu(2:end),'--','color','k','linewidth',0.1);
    legend('Mean value','Two standard deviations');
    ylim([0 1]);
    xlim([1 length(meanM_mu)]);
    xticks([1 20 40 60 80 100]);
    xticklabels({'1','20','40','60','80','100'});
    xlabel('Number of impact events');
    ylabel('Coefficient of friction $\mu$')
%     print(gcf,'M_mu_evel.png','-dpng','-r500');
    if doSave
        fig = gcf;
        fig.PaperPositionMode = 'auto';
        fig_pos = fig.PaperPosition;
        fig.PaperSize = [fig_pos(3) fig_pos(4)];
        print(fig,append('figures/',Object,'/NewSolver/MATLABmu.pdf'),'-dpdf','-vector')
    end
    
    %Plot the evolution of eN MATLAB
    figure('rend','painters','pos',[pp{3,2} sizex sizey]);
    ha = tight_subplot(1,1,[.08 .07],[.18 .1],[0.12 0.03]);  %[gap_h gap_w] [lower upper] [left right]
    axes(ha(1));
    plot(meanM_eN,'color',[0.8500 0.3250 0.0980],'linewidth',1.2); hold on; grid on;
    plot(2:length(meanM_eN),meanM_eN(2:end)+2*stdM_eN(2:end),'--','color','k','linewidth',0.1);
    plot(2:length(meanM_eN),meanM_eN(2:end)-2*stdM_eN(2:end),'--','color','k','linewidth',0.1);
    legend('Mean value','Two standard deviations');
    ylim([0 1]);
    xlim([1 length(meanM_eN)]);
    xticks([1 20 40 60 80 100]);
    xticklabels({'1','20','40','60','80','100'});
    xlabel('Number of impact events');
    ylabel('Coefficient of normal restitution $e_N$')
%     print(gcf,'M_eN_evel.png','-dpng','-r500');
    if doSave
        fig = gcf;
        fig.PaperPositionMode = 'auto';
        fig_pos = fig.PaperPosition;
        fig.PaperSize = [fig_pos(3) fig_pos(4)];
        print(fig,append('figures/',Object,'/NewSolver/MATLABeN.pdf'),'-dpdf','-vector')
    end
   
    %Combined plot
    figure('rend','painters','pos',[pp{3,3} 2*sizex sizey]);
    ha = tight_subplot(1,2,[.08 .07],[.18 .15],[0.08 0.03]);  %[gap_h gap_w] [lower upper] [left right]
    axes(ha(1));
    hm = plot(-5:5,BCV_CB(1:3,:),'linewidth',1); grid on; hold on;                           %Measurement
    hM = plot(k,BCV_MB_M(1:3,1:11,optMATLAB_idx),'linewidth',1,'LineStyle','--');            %MATLAB data
    set(hm,{'color'},{color.Red; color.Green; color.Blue})
    set(hM,{'color'},{color.Red; color.Green; color.Blue})
    p = plot(-5:Ib,BCV_CBaf(1:3,1:indx_b)','-.','color','k','LineWidth',1);
    plot(Ic:5,BCV_CBef(1:3,indx_c:11)','-.','color','k','LineWidth',1);
    xlabel('Normalized time around the impact time $(t-t_j)/\Delta t$ [-]');
    ylabel('Linear velocity $^{B[C]}$\boldmath${v}_{C,B}$ [m/s]');
    ha(1).XTick = k;
    
    axes(ha(2));
    hm = plot(-5:5,BCV_CB(4:6,:),'linewidth',1); grid on; hold on;                          %Measurement
    hM = plot(k,BCV_MB_M(4:6,1:11,optMATLAB_idx),'linewidth',1,'LineStyle','--');           %MATLAB data
    set(hm,{'color'},{color.Red; color.Green; color.Blue})
    set(hM,{'color'},{color.Red; color.Green; color.Blue})
    plot(-5:Ib,BCV_CBaf(4:6,1:indx_b)','-.','color','k');
    plot(Ic:5,BCV_CBef(4:6,indx_c:11)','-.','color','k');
    xlabel('Normalized time around the impact time $(t-t_j)/\Delta t$ [-]');
    ylabel('Angular velocity $^{B[C]}$\boldmath${\omega}_{C,B}$ [m/s]');
    ha(2).XTick = k;
%     L1 = legend([hm(1) hm(2) hm(3) p(1)],'$x$-meas','$y$-meas','$z$-meas','Fitted','NumColumns',4,'location','northeast');
    L1 = legend([hm(1) hm(2) hm(3) hM(1) hM(2) hM(3) p(1)],'$x$-meas','$y$-meas','$z$-meas','$x$-MATLAB','$y$-MATLAB','$z$-MATLAB','Fitted','NumColumns',7,'location','northeast');
    L1.Position(2) = 0.91;
    L1.Position(1) = 0.52-(L1.Position(3)/2);
%     print(gcf,'VelMATLAB.png','-dpng','-r500');
    
    if doSave
        fig = gcf;
        fig.PaperPositionMode = 'auto';
        fig_pos = fig.PaperPosition;
        fig.PaperSize = [fig_pos(3) fig_pos(4)];
        print(fig,'VelMATLAB.pdf','-dpdf','-vector')
    end

    %Plot the combined cost of Matlab
    figure('rend','painters','pos',[pp{3,5} 0.7*sizex sizey]);
    ha = tight_subplot(1,1,[.08 .07],[.18 .1],[0.12 0.03]);  %[gap_h gap_w] [lower upper] [left right]
    axes(ha(1));
    surf(eN,mu,SUMMATLAB); axis square;
    xlabel('$e_N$');ylabel('$\mu$');zlabel('$\frac{1}{N}\sum_{k=1}^N(\|$diag$(\mathbf{w})(\Delta \mathbf{v})\|)_k$');
    zlim([0 max([max(SUMMATLAB(:)) max(SUMAGX(:))])]);
    xlim([0 1]);
    ylim([0 1]);
    view(-40,15);
    if doSave
        fig = gcf;
        fig.PaperPositionMode = 'auto';
        fig_pos = fig.PaperPosition;
        fig.PaperSize = [fig_pos(3) fig_pos(4)];
        print(fig,append('figures/',Object,'/NewSolver/Cost_SUMMATLAB.pdf'),'-dpdf','-vector')
    end
    
    %
    %Plot the optimum parameters in the parameter space
    [a1,b1,c1] = find(SUMMATLAB == min(SUMMATLAB(:)));
    figure('rend','painters','pos',[pp{1,1} 0.7*sizex sizey]);
    ha = tight_subplot(1,1,[.08 .07],[.15 .04],[0.12 0.03]);  %[gap_h gap_w] [lower upper] [left right]
    axes(ha(1)); 
        surf(eN,mu,SUMMATLAB); 
        hold on;
        g1=plot3(eN(b1),mu(a1),c1,'.','color',[0.3010 0.7450 0.9330],'MarkerSize',22); 
        g2=plot3(mwi*meN',mwi*mmu',c1,'.','color',[0.9290 0.6940 0.1250],'MarkerSize',22);
        g3=plot3(mean(meN),mean(mmu),c1,'.','color',[0.4660 0.6740 0.1880],'MarkerSize',22);
        axis square;    
        xlabel('$e_N$');ylabel('$\mu$');zlabel('$\frac{1}{N}\sum_{k=1}^N(\|$diag$(\mathbf{w})(\Delta \mathbf{v})\|)_k$');
        zlim([0 1.25]);
        xlim([0.25 0.45]);
        ylim([0.35 0.55]);
        view(0,90);
        L1 = legend([g1,g2,g3],'Combined CF optimum','Mean of CF''s','Weighted mean of CF''s');
        L1.Position(2) = 0.78;
        L1.Position(1) = 0.545-(L1.Position(3)/2);
%         print(gcf,'M_OptParam.png','-dpng','-r500');

    figure('rend','painters','pos',[pp{1,2} sizex sizey]);
        ha = tight_subplot(1,1,[.08 .07],[.15 .06],[0.15 0.03]);  %[gap_h gap_w] [lower upper] [left right]
        axes(ha(1)); 
        plot(mwi,'o');
        xlabel('Impact event');
        ylabel('Normalized weight');
%         print(gcf,'WeightsMatlab.png','-dpng','-r500');
end

%%
if evalAlgoryx    
    %Plot the evolution of mu AGX
    figure('rend','painters','pos',[pp{2,1} sizex sizey]);
    ha = tight_subplot(1,1,[.08 .07],[.18 .1],[0.12 0.03]);  %[gap_h gap_w] [lower upper] [left right]
    axes(ha(1));
    plot(meanA_mu,'color',[0 0.4470 0.7410],'linewidth',1.2); hold on; grid on;
    plot(2:length(meanA_mu),meanA_mu(2:end)+2*stdA_mu(2:end),'--','color','k','linewidth',0.1);
    plot(2:length(meanA_mu),meanA_mu(2:end)-2*stdA_mu(2:end),'--','color','k','linewidth',0.1);
    legend('Mean value','Two standard deviations');
    ylim([0 1]);
    xlim([1 length(meanA_mu)]);
    xticks([1 20 40 60 80 100]);
    xticklabels({'1','20','40','60','80','100'});
    xlabel('Number of impact events');
    ylabel('Coefficient of friction $\mu$')
%     print(gcf,'A_mu_evel.png','-dpng','-r500');
    if doSave
        fig = gcf;
        fig.PaperPositionMode = 'auto';
        fig_pos = fig.PaperPosition;
        fig.PaperSize = [fig_pos(3) fig_pos(4)];
        print(fig,append('figures/',Object,'/NewSolver/AGXmu.pdf'),'-dpdf','-vector')
    end
    
    %Plot the evolution of eN AGX
    figure('rend','painters','pos',[pp{2,2} sizex sizey]);
    ha = tight_subplot(1,1,[.08 .07],[.18 .1],[0.12 0.03]);  %[gap_h gap_w] [lower upper] [left right]
    axes(ha(1));
    plot(meanA_eN,'color',[0.8500 0.3250 0.0980],'linewidth',1.2); hold on; grid on;
    plot(2:length(meanM_eN),meanA_eN(2:end)+2*stdA_eN(2:end),'--','color','k','linewidth',0.1);
    plot(2:length(meanM_eN),meanA_eN(2:end)-2*stdA_eN(2:end),'--','color','k','linewidth',0.1);
    legend('Mean value','Two standard deviations');
    ylim([0 1]);
    xlim([1 length(meanA_eN)]);
    xticks([1 20 40 60 80 100]);
    xticklabels({'1','20','40','60','80','100'});
    xlabel('Number of impact events');
    ylabel('Coefficient of normal restitution $e_N$')
%     print(gcf,'A_eN_evel.png','-dpng','-r500');
    if doSave
        fig = gcf;
        fig.PaperPositionMode = 'auto';
        fig_pos = fig.PaperPosition;
        fig.PaperSize = [fig_pos(3) fig_pos(4)];
        print(fig,append('figures/',Object,'/NewSolver/AGXeN.pdf'),'-dpdf','-vector')
    end
    
    % PLOT TWIST RESULTS OF ALGORYX
    %Combined plot
    figure('rend','painters','pos',[pp{2,3} 2*sizex sizey]);
    ha = tight_subplot(1,2,[.08 .07],[.18 .15],[0.08 0.03]);  %[gap_h gap_w] [lower upper] [left right]
    axes(ha(1));
    hm = plot(-5:5,BCV_CB(1:3,:),'linewidth',1); grid on; hold on;                           %Measurement
    hA = plot(k,BCV_MB_AGX(1:3,(1:11),optAGX_idx),'linewidth',1,'LineStyle','--');            %MATLAB data
    set(hm,{'color'},{[0.9290 0.6940 0.1250]; [0.4940 0.1840 0.5560]; [0.3010 0.7450 0.9330]})
    set(hA,{'color'},{[0.9290 0.6940 0.1250]; [0.4940 0.1840 0.5560]; [0.3010 0.7450 0.9330]})
    p = plot(-5:Ib,BCV_CBaf(1:3,1:indx_b)','-.','color','k','LineWidth',1);
    plot(Ic:5,BCV_CBef(1:3,indx_c:11)','-.','color','k','LineWidth',1);
    xlabel('Normalized time around the impact time $(t-t_j)/\Delta t$ [-]');
    ylabel('Linear velocity $^{B[C]}$\boldmath${v}_{C,B}$ [m/s]');
    ha(1).XTick = k;
    
    axes(ha(2));
    hm = plot(-5:5,BCV_CB(4:6,:),'linewidth',1); grid on; hold on;                          %Measurement
    hA = plot(k,BCV_MB_AGX(4:6,(1:11),optAGX_idx),'linewidth',1,'LineStyle','--');           %MATLAB data
    set(hm,{'color'},{[0.9290 0.6940 0.1250]; [0.4940 0.1840 0.5560]; [0.3010 0.7450 0.9330]})
    set(hA,{'color'},{[0.9290 0.6940 0.1250]; [0.4940 0.1840 0.5560]; [0.3010 0.7450 0.9330]})
    plot(-5:Ib,BCV_CBaf(4:6,1:indx_b)','-.','color','k');
    plot(Ic:5,BCV_CBef(4:6,indx_c:11)','-.','color','k');
    xlabel('Normalized time around the impact time $(t-t_j)/\Delta t$ [-]');
    ylabel('Angular velocity $^{B[C]}$\boldmath${\omega}_{C,B}$ [m/s]');
    ha(2).XTick = k;
    L1 = legend([hm(1) hm(2) hm(3) hM(1) hM(2) hM(3) p(1)],'$x$-meas','$y$-meas','$z$-meas','$x$-Algoryx','$y$-Algoryx','$z$-Algoryx','Fitted','NumColumns',7,'location','northeast');
    L1.Position(2) = 0.91;
    L1.Position(1) = 0.52-(L1.Position(3)/2);
    print(gcf,'VelAGX.png','-dpng','-r500');
    
    if doSave
        fig = gcf;
        fig.PaperPositionMode = 'auto';
        fig_pos = fig.PaperPosition;
        fig.PaperSize = [fig_pos(3) fig_pos(4)];
        print(fig,'figures/VelMATLAB.pdf','-dpdf','-vector')
    end

    %Plot the combined cost of AGX
    figure('rend','painters','pos',[pp{2,5} 0.7*sizex sizey]);
    ha = tight_subplot(1,1,[.08 .07],[.18 .1],[0.12 0.03]);  %[gap_h gap_w] [lower upper] [left right]
    axes(ha(1));
    surf(eN,mu,SUMAGX); axis square;
    xlabel('$e_N$');ylabel('$\mu$');zlabel('$\frac{1}{N}\sum_{k=1}^N(\|$diag$(\mathbf{w})(\Delta \mathbf{v})\|)_k$');
    zlim([0 max([max(SUMMATLAB(:)) max(SUMAGX(:))])]);
    xlim([0 1]);
    ylim([0 1]);
    view(-40,15);
    if doSave
        fig = gcf;
        fig.PaperPositionMode = 'auto';
        fig_pos = fig.PaperPosition;
        fig.PaperSize = [fig_pos(3) fig_pos(4)];
        print(fig,append('figures/',Object,'/NewSolver/Cost_SUMAGX.pdf'),'-dpdf','-vector')
    end
    
    %Plot the optimum parameters in the parameter space
    [a1,b1,c1] = find(SUMAGX == min(SUMAGX(:)));
    figure('rend','painters','pos',[pp{1,3} 0.7*sizex sizey]);
    ha = tight_subplot(1,1,[.08 .07],[.15 .04],[0.12 0.03]);  %[gap_h gap_w] [lower upper] [left right]
    axes(ha(1)); 
        surf(eN,mu,SUMAGX); 
        hold on;
        g1=plot3(eN(b1),mu(a1),c1,'.','color',[0.3010 0.7450 0.9330],'MarkerSize',22); 
        g2=plot3(awi*aeN',awi*amu',c1,'.','color',[0.9290 0.6940 0.1250],'MarkerSize',22);
        g3=plot3(mean(aeN),mean(amu),c1,'.','color',[0.4660 0.6740 0.1880],'MarkerSize',22);
        axis square;    
        xlabel('$e_N$');ylabel('$\mu$');zlabel('$\frac{1}{N}\sum_{k=1}^N(\|$diag$(\mathbf{w})(\Delta \mathbf{v})\|)_k$');
        zlim([0 1.25]);
        xlim([0.25 0.45]);
        ylim([0.35 0.55]);
        view(0,90);
        L1 = legend([g1,g2,g3],'Combined CF optimum','Mean of CF''s','Weighted mean of CF''s');
        L1.Position(2) = 0.78;
        L1.Position(1) = 0.545-(L1.Position(3)/2);
%         print(gcf,'A_OptParam.png','-dpng','-r500');

    figure('rend','painters','pos',[pp{1,4} sizex sizey]);
    ha = tight_subplot(1,1,[.08 .07],[.15 .06],[0.15 0.03]);  %[gap_h gap_w] [lower upper] [left right]
    axes(ha(1)); 
    plot(awi,'o');
    xlabel('Impact event');
    ylabel('Normalized weight');
%     print(gcf,'WeightsAlgoryx.png','-dpng','-r500');

end
    
%%
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
    if evalMatlab;  p2 = plot(k+impacts.indx(imp_sel(is),2),squeeze(dCp_M(3,V_impact,:,optMATLAB_idx)),'color',color.Matlab,'LineWidth',1.5); end
    p4 = plot(kf+impacts.indx(imp_sel(is),2),F,'--k','LineWidth',1);
    p5 = scatter(kf([indx_bf,indx_cf])+impacts.indx(imp_sel(is),2),F([indx_bf,indx_cf]),'xk','linewidth',1); %Plot fitted points in continuous function
    p6 = scatter([indx_b,indx_c]-1-w_ext+impacts.indx(imp_sel(is),2),dCp_meas(3,[indx_b,indx_c]),'r','linewidth',1); %Plot fitted points in discrete function
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
%% Paper figures

%plot measured contact point velocities
figure('rend','painters','pos',[pp{1,1} sizex 1.2*sizey]);
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
    L1 = legend([p1 p2 p3 p4 p5 p6],'$(^C\dot{\mathbf{p}}_1)_x$','$(^C\dot{\mathbf{p}}_1)_y$','$(^C\dot{\mathbf{p}}_1)_z$','$(^C\dot{\mathbf{p}}_1)_z$-fit',...
        'Cont. indices','Disc. indices $-b$, $c$','NumColumns',3,'Location','NorthWest'); 
    L1.Position(2) = 0.88;
    L1.Position(1) = 0.545-(L1.Position(3)/2);
    xlim([-5 5]);
    xlabel('Normalized time around the impact time $(t-t_j)/\Delta t$ [-]');
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
        print(fig,'CPVelocity_CentralEuler.pdf','-dpdf','-vector')
    end

%Combined plot
    figure('rend','painters','pos',[pp{3,3} sizex 1.8*sizey]);
    ha = tight_subplot(2,1,[.08 .07],[.1 .1],[0.12 0.03]);  %[gap_h gap_w] [lower upper] [left right]
    axes(ha(1));
    hm = plot(-5:5,BCV_CB(1:3,:),'linewidth',1.2); grid on; hold on;                           %Measurement
    set(hm,{'color'},{color.Red; color.Green; color.Blue})
    p = plot(-5:Ib,BCV_CBaf(1:3,1:indx_b)','--','color','k','LineWidth',1.2);
    plot(Ic:5,BCV_CBef(1:3,indx_c:11)','--','color','k','LineWidth',1.2);
    xlabel('Normalized time around the impact time $(t-t_j)/\Delta t$ [-]');
    ylabel('Linear velocity $^{B[C]}$\boldmath${v}_{C,B}$ [m/s]');
    ha(1).XTick = k;
    text(-5.4,0.75,'$-a$','FontSize',12);
    text(-2.4,0.75,'$-b$','FontSize',12);
    text(1.9,0.75,'$c$','FontSize',12);
    text(4.9,0.75,'$d$','FontSize',12);
    xline(-5,'-.')
    xline(indx_b-6,'-.')
    xline(indx_c-6,'-.')
    xline(5,'-.')
    
    axes(ha(2));
    hm = plot(-5:5,BCV_CB(4:6,:),'LineWidth',1.2); grid on; hold on;                          %Measurement
    set(hm,{'color'},{color.Red; color.Green; color.Blue})
    plot(-5:Ib,BCV_CBaf(4:6,1:indx_b)','--','color','k','LineWidth',1.2);
    plot(Ic:5,BCV_CBef(4:6,indx_c:11)','--','color','k','LineWidth',1.2);
    xlabel('Normalized time around the impact time $(t-t_j)/\Delta t$ [-]');
    ylabel('Angular velocity $^{B[C]}$\boldmath${\omega}_{C,B}$ [m/s]');
    ha(2).XTick = k;
    xline(-5,'-.')
    xline(indx_b-6,'-.')
    xline(indx_c-6,'-.')
    xline(5,'-.')
    L1 = legend([hm(1) hm(2) hm(3) p(1)],'$x$-meas','$y$-meas','$z$-meas','Fitted','NumColumns',4,'location','northeast');
    L1.Position(2) = 0.96;
    L1.Position(1) = 0.52-(L1.Position(3)/2);
%     print(gcf,'MeasuredVelocities.png','-dpng','-r500');    
    if doSave
        fig = gcf;
        fig.PaperPositionMode = 'auto';
        fig_pos = fig.PaperPosition;
        fig.PaperSize = [fig_pos(3) fig_pos(4)];
        print(fig,'MeasuredVelocities.pdf','-dpdf','-vector')
    end


%Combined plot Matlab results
    figure('rend','painters','pos',[pp{3,3} sizex 1.8*sizey]);
    ha = tight_subplot(2,1,[.08 .07],[.1 .12],[0.12 0.03]);  %[gap_h gap_w] [lower upper] [left right]
    axes(ha(1));
    hm = plot(-5:5,BCV_CB(1:3,:),'linewidth',1); grid on; hold on;                           %Measurement
    hM = plot(k,BCV_MB_M(1:3,1:11,optMATLAB_idx),'linewidth',1,'LineStyle','--');            %MATLAB data
    set(hm,{'color'},{color.RedLight; color.GreenLight; color.BlueLight})
    set(hM,{'color'},{color.Red; color.Green; color.Blue})
    p = plot(-5:Ib,BCV_CBaf(1:3,1:indx_b)','--','color','k','LineWidth',1.2);
    plot(Ic:5,BCV_CBef(1:3,indx_c:11)','--','color','k','LineWidth',1.2);
    xline(-5,'-.')
    xline(indx_b-6,'-.')
    xline(indx_c-6,'-.')
    xline(5,'-.')
    xlabel('Normalized time around the impact time $(t-t_j)/\Delta t$ [-]');
    ylabel('Linear velocity $^{B[C]}$\boldmath${v}_{C,B}$ [m/s]');
    ha(1).XTick = k;
    
    axes(ha(2));
    hm = plot(-5:5,BCV_CB(4:6,:),'linewidth',1); grid on; hold on;                          %Measurement
    hM = plot(k,BCV_MB_M(4:6,1:11,optMATLAB_idx),'linewidth',1,'LineStyle','--');           %MATLAB data
    set(hm,{'color'},{color.RedLight; color.GreenLight; color.BlueLight})
    set(hM,{'color'},{color.Red; color.Green; color.Blue})
    plot(-5:Ib,BCV_CBaf(4:6,1:indx_b)','--','color','k','LineWidth',1.2);
    plot(Ic:5,BCV_CBef(4:6,indx_c:11)','--','color','k','LineWidth',1.2);
    xline(-5,'-.')
    xline(indx_b-6,'-.')
    xline(indx_c-6,'-.')
    xline(5,'-.')
    xlabel('Normalized time around the impact time $(t-t_j)/\Delta t$ [-]');
    ylabel('Angular velocity $^{B[C]}$\boldmath${\omega}_{C,B}$ [m/s]');
    ha(2).XTick = k;
%     L1 = legend([hm(1) hm(2) hm(3) p(1)],'$x$-meas','$y$-meas','$z$-meas','Fitted','NumColumns',4,'location','northeast');
    L1 = legend([hm(1) hm(2) hm(3) hM(1) hM(2) hM(3) p(1)],'$x$-meas','$y$-meas','$z$-meas','$x$-MATLAB','$y$-MATLAB','$z$-MATLAB','Fitted','NumColumns',3,'location','northeast');
    L1.Position(2) = 0.9035;
    L1.Position(1) = 0.52-(L1.Position(3)/2);
%     print(gcf,'VelMATLAB.png','-dpng','-r500');
    
    if doSave
        fig = gcf;
        fig.PaperPositionMode = 'auto';
        fig_pos = fig.PaperPosition;
        fig.PaperSize = [fig_pos(3) fig_pos(4)];
        print(fig,'VelResMATLAB.pdf','-dpdf','-vector')
    end


    %Combined plot AGX Results
    figure('rend','painters','pos',[pp{3,3} sizex 1.8*sizey]);
    ha = tight_subplot(2,1,[.08 .07],[.1 .12],[0.12 0.03]);  %[gap_h gap_w] [lower upper] [left right]
    axes(ha(1));
    hm = plot(-5:5,BCV_CB(1:3,:),'linewidth',1); grid on; hold on;                           %Measurement
    hA = plot(k,BCV_MB_AGX(1:3,(1:11),optAGX_idx),'linewidth',1,'LineStyle','--');            %MATLAB data
    set(hm,{'color'},{color.RedLight; color.GreenLight; color.BlueLight})
    set(hA,{'color'},{color.Red; color.Green; color.Blue})
    p = plot(-5:Ib,BCV_CBaf(1:3,1:indx_b)','--','color','k','LineWidth',1.2);
    plot(Ic:5,BCV_CBef(1:3,indx_c:11)','--','color','k','LineWidth',1.2);
    xlabel('Normalized time around the impact time $(t-t_j)/\Delta t$ [-]');
    ylabel('Linear velocity $^{B[C]}$\boldmath${v}_{C,B}$ [m/s]');
    ha(1).XTick = k;
    xline(-5,'-.')
    xline(indx_b-6,'-.')
    xline(indx_c-6,'-.')
    xline(5,'-.')
    
    axes(ha(2));
    hm = plot(-5:5,BCV_CB(4:6,:),'linewidth',1); grid on; hold on;                          %Measurement
    hA = plot(k,BCV_MB_AGX(4:6,(1:11),optAGX_idx),'linewidth',1,'LineStyle','--');           %MATLAB data
    set(hm,{'color'},{color.RedLight; color.GreenLight; color.BlueLight})
    set(hA,{'color'},{color.Red; color.Green; color.Blue})
    plot(-5:Ib,BCV_CBaf(4:6,1:indx_b)','--','color','k','LineWidth',1.2);
    plot(Ic:5,BCV_CBef(4:6,indx_c:11)','--','color','k','LineWidth',1.2);
    xlabel('Normalized time around the impact time $(t-t_j)/\Delta t$ [-]');
    ylabel('Angular velocity $^{B[C]}$\boldmath${\omega}_{C,B}$ [m/s]');
    ha(2).XTick = k;
    xline(-5,'-.')
    xline(indx_b-6,'-.')
    xline(indx_c-6,'-.')
    xline(5,'-.')
    L1 = legend([hm(1) hm(2) hm(3) hA(1) hA(2) hA(3) p(1)],'$x$-meas','$y$-meas','$z$-meas','$x$-Algoryx','$y$-Algoryx','$z$-Algoryx','Fitted','NumColumns',3,'location','northeast');
    L1.Position(2) = 0.90;
    L1.Position(1) = 0.52-(L1.Position(3)/2);
%     print(gcf,'VelAGX.png','-dpng','-r500');
    
    if doSave
        fig = gcf;
        fig.PaperPositionMode = 'auto';
        fig_pos = fig.PaperPosition;
        fig.PaperSize = [fig_pos(3) fig_pos(4)];
        print(fig,'figures/VelResAGX.pdf','-dpdf','-vector')
    end

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
%     xlabel('Normalized time around the impact time $(t-t_j)/\Delta t$ [-]');
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
%     xlabel('Normalized time around the impact time $(t-t_j)/\Delta t$ [-]');
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
        plotBox(MH_B_meas{ii},box,color.Meas);hold on;
        
        %Plot MATLAB box
        plotBox(MH_B_M(:,:,ii,optMATLAB_idx),box,color.Matlab); hold on;     

        %Plot new AGX box results
        plotBox(MH_B_AGX(:,:,ii,optAGX_idx),box,color.Algoryx);hold on;
                
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
vq = griddata(eN,mu,SUMMATLAB,xq,yq,'v4');
vq(vq<0)=0;

scalefactor = @(x) (x).^-0.2;
Z2 = scalefactor(vq);

n = 10;
a = 100;
lower = min(Z2(:));
upper = max(Z2(:));
temp = -1./(linspace(0.1,0.13,n))+(1/0.1);
% re-scale to be between 0 and 1
temp_01 = temp/max(temp) - min(temp)/max(temp);
% re-scale to be between your limits (i.e. 1 and 1.05)
out = temp_01*(upper-lower) + lower;
%
figure;
[cf, cf1] = contourf(xvec,yvec,Z2,[out 1.29 1.2925],'LineColor','none');
% [cf cf1] = contourf(xvec,yvec,Z2,linspace(min(Z2(:)),max(Z2(:)),20));
% colormap(map)
map = colormap(flipud(gray));
map(1:20,:) = [];
map(end-40:end,:)=[];
colormap(map);
axis square


%% FOR THE POSTER
close all
%Plot the combined cost of AGX
    figure('rend','painters','pos',[pp{2,5} 0.7*sizex sizey]);
    ha = tight_subplot(1,1,[.08 .07],[.18 .1],[0.12 0.03]);  %[gap_h gap_w] [lower upper] [left right]
    axes(ha(1));
    surf(eN,mu,SUMAGX); axis square;
    xlabel('$e_N$');ylabel('$\mu$');zlabel('$\frac{1}{N}\sum_{k=1}^N(\|$diag$(\mathbf{w})(\Delta \mathbf{v})\|)_k$');
    zlim([0 max([max(SUMMATLAB(:)) max(SUMAGX(:))])]);
    xlim([0 1]);
    ylim([0 1]);
    view(-40,15);
    ax = gca;
    exportgraphics(ax,'CostAGX.eps')


    %Plot the combined cost of Matlab
    figure('rend','painters','pos',[pp{3,5} 0.7*sizex sizey]);
    ha = tight_subplot(1,1,[.08 .07],[.18 .1],[0.12 0.03]);  %[gap_h gap_w] [lower upper] [left right]
    axes(ha(1));
    surf(eN,mu,SUMMATLAB); axis square;
    xlabel('$e_N$');ylabel('$\mu$');zlabel('$\frac{1}{N}\sum_{k=1}^N(\|$diag$(\mathbf{w})(\Delta \mathbf{v})\|)_k$');
    zlim([0 max([max(SUMMATLAB(:)) max(SUMAGX(:))])]);
    xlim([0 1]);
    ylim([0 1]);
    view(-40,15);
    ax = gca;
    exportgraphics(ax,'CostMatlab.eps')

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

%%
ws    = 1;                 %With of the contact surface             [m]
ls    = 1;               %Length of the contact surface           [m]
surfacepoints = [0.5*ws -0.5*ws -0.5*ws 0.5*ws 0.5*ws; -0.5*ls -0.5*ls 0.5*ls 0.5*ls -0.5*ls; 0 0 0 0 0;];
spoints = MR_C*surfacepoints; %Transform the vertices according to position/orientation of the surface
figure('pos',[497,221,465,231]);
        ii=11   ;     
        %plot Measured box
        p1 = plotBox(MH_B_meas{ii},box,color.Meas);hold on;
        
        %Plot MATLAB box
        p2 = plotBox(MH_B_M(:,:,ii,optMATLAB_idx),box,color.Matlab); hold on;     

        %Plot new AGX box results
        p3 = plotBox(MH_B_AGX(:,:,ii,optAGX_idx),box,color.Algoryx);hold on;
                
        %Plot the conveyor C
        table3 = fill3(spoints(1,1:4),spoints(2,1:4),spoints(3,1:4),1);hold on;
        set(table3,'FaceColor',0.8*[1 1 1],'FaceAlpha',1);
       
        grid on;axis equal;
        axis([-0.3 0.4 -0.1 0.6 -0.02 0.3]);
        xlabel('x [m]');
        ylabel('y [m]');
        zlabel('z [m]');
        view(0,0); %to see impact correctly
        text(0.185290858725762,0,0.28287811634349,append('Frame: ',sprintf('%d',ii)),'FontSize',22);
        grid off; axis off;
        drawnow  

        if doSave
            fig = gcf;
            fig.PaperPositionMode = 'auto';
            fig_pos = fig.PaperPosition;
            fig.PaperSize = [fig_pos(3) fig_pos(4)];
            print(fig,'D:\OneDrive - TU Eindhoven\I.AM\6.GIT\i-am-toss/Pos11','-dpdf','-vector')
        end
        
