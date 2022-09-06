% function impacts = getImpactData(h5file,measname)
% h5file = "D:\OneDrive - TU Eindhoven\I.AM\9.Database\Archives_local\220426_ParamID_Box006.h5";
h5file = "D:\OneDrive - TU Eindhoven\I.AM\9.Database\Archives_local\220430_BoxDrops.h5";
ObjStr = "Box005"; %The object for which you want to do paramID
EnvStr = "Conveyor002"; %The Environment for which you want to do paramID
% ImpPln = true; %Impact plane: false = no Motive impact plane, taking origin as impact plane. Otherwise, give the string name in ImpStr
ImpPln = "GroundPlane001";

% This function loads a HDF5 file, and loads the specified measurement
% data. It then selects the time instances of impact and evaluates if the
% impact is suitable for parameter identification. If so, the following
% data concerning this impact is stored:
%   - pre-impact velocity of the object
%   - post-impact velocity of the object
%   - measurement time
%   - contact surface name
%   - object name
%
% INPUTS:    h5file       : .h5 file containing dataset of measurements
%            measName     : name of the measurement inside .h5 file. (char,
%                           e.g.: 'Rec_20200519T091937Z')
%
% OUTPUTS:   impacts      : Saves a struct "impacts.mat" containing above
%                           mentioned impact data.
%
%% Obtain the impact data
set(groot,'defaulttextinterpreter','latex');
set(groot,'defaultAxesTickLabelInterpreter','latex');
set(groot,'defaultLegendInterpreter','latex');
%% User settings and constants
w_ext = 5;          % Extension size of window on both sides
gravity = true;    % Determine if you have gravity in -z direction of inertial frame
g  = 9.81;         % Gravitational constant [m/s^2]
%% Initializing
H5data = readH5(h5file);
%Collect the recording names
fname = fieldnames(H5data);

%% Load the impact data from given measurements

for iim = 1:length(fname)
    if contains(string(fname(iim)),'Rec')
        data = H5data.(string(fname(iim)));
    
        %If the current recording does not have the object we want, we
        %continue to the next recording
        if ~isfield(data.SENSOR_MEASUREMENT.Mocap.POSTPROCESSING, ObjStr)
            continue;
        end

        %Timestamp of the recording
        orgtime = data.attr.recording_timestamp;

        % Get the box object
        OBJECT = string(fieldnames(data.OBJECT)); % Object fieldnames
        ibox = contains(OBJECT,ObjStr);           % Index of box
        box = data.OBJECT.(OBJECT(ibox));         % Box object
        Bp = box.vertices.ds';                    % Vertices of the box

        % Set initial parameters        
        t  = table2array(data.SENSOR_MEASUREMENT.Mocap.datalog.ds(:,'Time(s)'));  % Time vector
        freq = str2double(data.SENSOR_MEASUREMENT.Mocap.datalog.attr.sample_frequency); % OptiTrack sample frequency
        dt = 1/freq;                     % Time step [s]

        % Transforamtion matrix expressing object in terms of robot base
        MH_B = data.SENSOR_MEASUREMENT.Mocap.POSTPROCESSING.(OBJECT(ibox)).transforms.ds';

        %Preallocate memmory based on the number of samples of the data
        Nsamples = length(MH_B);
        CH_Bc   = repmat({NaN(4,4)},1,Nsamples);
        Cp_z    = NaN(8,Nsamples);
        dCo_B   = NaN(3,Nsamples);
        dMo_B   = NaN(3,Nsamples);
        Bv_MB   = NaN(3,Nsamples);
        Cv_CB   = NaN(3,Nsamples);
        Bv_CB   = NaN(3,Nsamples);
        Bomg_MB = NaN(3,Nsamples);
        Comg_CB = NaN(3,Nsamples);
        Bomg_CB = NaN(3,Nsamples);
        dCp     = NaN(3,length(Bp),Nsamples);

        % Get the contact plane posture
        if isfield(data.SENSOR_MEASUREMENT.Mocap.POSTPROCESSING, ImpPln)
            try
                MH_C = data.SENSOR_MEASUREMENT.Mocap.POSTPROCESSING.(ImpPln).transforms.ds';                
            catch
                error(append("Can't get the data from object",ImpPln));
            end
            % Velocity of conveyor belt w.r.t. base frame
            Mv_MC = [0;0;0];
        else
            %Take the motive frame as the impact plane frame with zero velocity
            MH_C = repmat({eye(4)},1,Nsamples);
            Mv_MC = [0;0;0];
        end        

        for ii = 1:Nsamples
            %Transformation of box expressed in conveyor frame
            CH_Bc{1,ii} = MH_C{ii}\MH_B{ii};
            % Vertices per timestep
            Cpn = CH_Bc{1,ii}*[Bp;ones(1,size(Bp,2))]; %Vertices expressed in C
            Cp_z(:,ii) = Cpn(3,:)';                     %Z-coordinate of vertices
        end

        % Determine all substancial impacts
        plocs = [];     ii_imp = [];
        for iv = 1:4 %Loop through first 4 vertices
            [~,ploc] = findpeaks(-Cp_z(iv,:),'MinPeakHeight',-0.005,'MinPeakDistance',5,'MinPeakProminence',0.02); %Find the peaks (moments of impact)
            plocs(iv,1:length(ploc)) = ploc; %Store the peak locations in matrix
        end
        for iv = 1:4 %Loop through first 4 vertices (only these can make contact)
            for ip = 1:length(nonzeros(plocs(iv,:))) %For each peak of the current vertex
                vrest = (1:8~=iv);
                %Create a window around the impact location of w_ext timesteps
                wind = plocs(iv,ip)-w_ext:plocs(iv,ip)+w_ext;
                %Check if window is not at beginning or end of data. Note that 1st and last timestep of dCp are NaN and cannot be used
                if min(wind)<2 || max(wind)>(Nsamples-1), continue; end
                %Check if any other vertex is in contact in window (So vertex position w.r.t. conv. is smaller than 0??)
                if any(any(Cp_z(vrest,wind)<0)), continue; end
                %Check for frame drops (then two positions within the window are exactly the same (why zero order hold??)
%                 if length(unique(squeeze(CH_Bm(1:3,4,wind))','rows'))~=2*w_ext+1, continue; end
                %Check for mimimal coincident velocity (in 5 timesteps before the impact, relative vel. should be \leq 0.3 m/s)
%                 if min(dCp(3,iv,plocs(iv,ip)-[1:w_ext]))>-0.3,  continue; end
                %If all checks are passed, indices of substantial impacts are saved
                ii_imp = [ii_imp, plocs(iv,ip)];
            end
        end
        ii_imp = sort(ii_imp);
        if isempty(ii_imp)
            continue;
        end

         for ii = 1:Nsamples
            %Transformation of box expressed in conveyor frame
            MH_C{ii} = MH_C{ii}+[zeros(3,3),[0;0;min(Cp_z(:,ii_imp(1)))];zeros(1,4)]; %Align impact plane with box vertex
            CH_Bc{1,ii} = MH_C{ii}\MH_B{ii};
         end        

        % Put Transformation matrices in 4D matrix
        CH_Bm = cat(3,CH_Bc{:});
        MH_Bm = cat(3,MH_B{:});

        %Compute angular and linear velocity of B w.r.t. C and M w/ central Euler differencing
        for ii = 2:Nsamples-1
            %Compute dCo_B and dMo_B using central difference approximation
            dCo_B(:,ii) = 1/(2*dt)*(CH_Bm(1:3,4,ii+1)-CH_Bm(1:3,4,ii-1));
            dMo_B(:,ii) = 1/(2*dt)*(MH_Bm(1:3,4,ii+1)-MH_Bm(1:3,4,ii-1));
            %Right trivialized velocity w.r.t C
            Comg_CB(:,ii) = vee(1/(2*dt)*((logm(CH_Bm(1:3,1:3,ii+1)/(CH_Bm(1:3,1:3,ii))))-(logm(CH_Bm(1:3,1:3,ii-1)/(CH_Bm(1:3,1:3,ii)))))); %See Eq. 4.66 of my thesis (but then right trivialized version)
            Cv_CB(:,ii)   = dCo_B(:,ii) - hat(Comg_CB(:,ii)) * CH_Bm(1:3,4,ii); %Eq. 23 of notation document
            %Left trivialized velocity w.r.t C
            Bomg_CB(:,ii) = CH_Bm(1:3,1:3,ii)' * Comg_CB(:,ii); %Eq. 27/28 of  Using BV_CB = BX_C*CV_CB
            Bv_CB(:,ii)   = CH_Bm(1:3,1:3,ii)' * dCo_B(:,ii) ;  %Eq. 18 of notation document
            %Left trivialized velocity w.r.t M
            Bomg_MB(:,ii) = vee(1/(2*dt)*((logm(MH_Bm(1:3,1:3,ii)\(MH_Bm(1:3,1:3,ii+1))))-(logm(MH_Bm(1:3,1:3,ii)\(MH_Bm(1:3,1:3,ii-1))))));
            Bv_MB(:,ii)   = MH_Bm(1:3,1:3,ii)' * dMo_B(:,ii);
        end
        CV_CB = [Cv_CB;Comg_CB];
        BV_CB = [Bv_CB; Bomg_CB];
        BV_MB = [Bv_MB;Bomg_MB];

        % Velocity of vertices in inertia frame
        for ii = 2:Nsamples-1 %For all timesteps
            for iv = 1:length(Bp) %For all vertices
                dCp(:,iv,ii) = dCo_B(:,ii) + hat(Comg_CB(:,ii))*CH_Bc{ii}(1:3,1:3)*Bp(:,iv); %The term vee(Comg_CB(:,ii))*CH_Bc{ii}(1:3,1:3) comes from Eq.24 of notation document
            end
        end

        % Evaluation of impact state
        if ~isempty(ii_imp)             % If substantial impacts are found in measurement
            for ii = 1:length(ii_imp)   % Treat all impacts
                % Find the surrounding poses and twists
                CH_Bimp         = CH_Bc(ii_imp(ii)+(-5:5));
                MH_Bimp         = MH_B(ii_imp(ii)+(-5:5));
                MH_Cimp         = MH_C(ii_imp(ii)+(-5:5));
                BV_CBimp        = num2cell(BV_CB(:,ii_imp(ii)+(-5:5)),1);
                BV_MBimp        = num2cell(BV_MB(:,ii_imp(ii)+(-5:5)),1);
                dCo_Bimp        = num2cell(dCo_B(:,ii_imp(ii)+(-5:5)),1);
                [~, V_impact]   = min(Cp_z(:,ii_imp(ii))); %Find the vertex that impacts
                dCo_Pimp        = num2cell(squeeze(dCp(:,V_impact,ii_imp(ii)+(-5:5))),1); %Get the velocity of the impacting point
                indx(1,1:2) 	= [V_impact,ii_imp(ii)]; %Add vertex index and timestep of impact time

                % Try to determine pre- and post impact indices automatically
                [indx_b, indx_c, BV_CBaf, BV_CBef, BCV_CB, BCV_CBaf,BCV_CBef] = IdentifyPPinds(cell2mat(dCo_Pimp),w_ext,g,freq,gravity,CH_Bimp,BV_CBimp);
                if indx_b==6 && indx_c==6 %if input was 0, skip this step
                   continue;
                end
                
                % Add indices to impacts
                indx(1,3) = indx(1,2)+indx_b-(w_ext+1); % global index of pre-impact
                indx(1,4) = indx(1,2)+indx_c-(w_ext+1); % global index of post-impact

                close all;
                
                % Save Impacts to struct
                % Extend impacts
                if exist('impacts','var')
                    impacts.meas        = [impacts.meas; repmat(orgtime,size(CH_Bimp,1),1)];
                    impacts.indx        = [impacts.indx; indx];
                    impacts.CH_B        = [impacts.CH_B; CH_Bimp];
                    impacts.MH_B        = [impacts.MH_B; MH_Bimp];
                    impacts.MH_C        = [impacts.MH_C; MH_Cimp];
                    impacts.BV_MB       = [impacts.BV_MB; BV_MBimp];
                    impacts.BV_CB       = [impacts.BV_CB; BV_CBimp];
                    impacts.BV_CBaf     = [impacts.BV_CBaf; BV_CBaf];
                    impacts.BV_CBef     = [impacts.BV_CBef; BV_CBef];
                    impacts.BCV_CB      = [impacts.BCV_CB; mat2cell(BCV_CB,6,ones(1,11))];
                    impacts.BCV_CBaf    = [impacts.BCV_CBaf; mat2cell(BCV_CBaf,6,ones(1,11))];
                    impacts.BCV_CBef    = [impacts.BCV_CBef; mat2cell(BCV_CBef,6,ones(1,11))];
                    %                         impacts.CV_CD       = [impacts.CV_CD; num2cell(CV_CD,1)];
                    impacts.dCo_B       = [impacts.dCo_B; dCo_Bimp];
                    impacts.dCo_p       = [impacts.dCo_p; dCo_Pimp];
                    impacts.box.name    = [impacts.box.name; OBJECT(ibox)];
                    impacts.box.M       = [impacts.box.M; {box.inertia.ds}];
                    impacts.box.V       = [impacts.box.V; {Bp}];
                    impacts.surface     = [impacts.surface; EnvStr];
                else
                    impacts.meas        = repmat(orgtime,size(CH_Bimp,1),1);
                    impacts.indx        = indx;
                    impacts.CH_B        = CH_Bimp;
                    impacts.MH_B        = MH_Bimp;
                    impacts.MH_C        = MH_Cimp;
                    impacts.BV_MB       = BV_MBimp;
                    impacts.BV_CB       = BV_CBimp;
                    impacts.BV_CBaf     = BV_CBaf;
                    impacts.BV_CBef     = BV_CBef;
                    impacts.BCV_CB      = mat2cell(BCV_CB,6,ones(1,11));
                    impacts.BCV_CBaf    = mat2cell(BCV_CBaf,6,ones(1,11));
                    impacts.BCV_CBef    = mat2cell(BCV_CBef,6,ones(1,11));
                    %                     impacts.CV_CD       = num2cell(CV_CD,1);
                    impacts.dCo_B       = dCo_Bimp;
                    impacts.dCo_p       = dCo_Pimp;
                    impacts.box.name    = OBJECT(ibox);
                    impacts.box.M       = {box.inertia.ds};
                    impacts.box.V       = {Bp};
                    impacts.surface     = EnvStr;
                end
            end
            % Save if so indicated
            if ~isfolder('paramID/impact_data')
                mkdir('paramID/impact_data');
            end
            if w_ext == 5
                save('paramID/impact_data/impacts.mat','impacts');
                disp(['   >> Added ' num2str(length(ii_imp)) ' suitable impact(s) to impacts.mat']);
            else
                save(strcat('paramID/impact_data/impacts', num2str(w_ext), '.mat'),'impacts');
                disp(['   >> Added ' num2str(length(ii_imp)) ' suitable impact(s) to impacts' num2str(w_ext) '.mat']);
            end
        end
    end
    textwaitbar(iim,length(fname),['Collecting impact data']);
end
% end

