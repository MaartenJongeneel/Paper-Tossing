function data = geth5dset(h5file,dsets,grpname)
% This function writes the HDF5 datasets corresponding to a group as
% to a Matlab struct
%
% INPUTS:    h5file       : [string] .h5 file containing dataset of
%                           measurements.
%            dsets        : [struct] data struct containing the datasets
%            grpname      : [string] name of the current group
%
% OUTPUTS:   data         : [struct] data struct containing as strings the
%                           attributes corresponding to the group
%
%% Write the datasets to the struct
%Loop through the datasets
for id = 1:length(dsets)
    dset = dsets(id);
    
    %% Pre-process dataset
    %Read dataset and convert to double if previously converted to single or int
    val = h5read(h5file,strcat(grpname,"/",dset.Name));               % Read dataset from file
    if ~(class(val) == "char")
        val = permute(val, fliplr(1:ndims(val)));                         % Flip dimensions
    end
    if any(class(val) == ["single";"int16"]), val = double(val); end  % Convert to double
    
    %Convert compound datasets (imported as structs) to tables
    if isstruct(val)
        % Characters, not allowed for struct fieldnames, are converted to hexadecimals
        % Find these hexadecimals in the names, and replace them by the symbols
        fns = string(fieldnames(val));
        for ii = 1:length(fns)
            hx = regexp(fns(ii),'(\dx\d.)','Match');                   % Find hexadecimals
            symbs = string(char(hex2dec(extractAfter(hx,'x')))')';     % Convert to symbols
            varnames(ii) = replace(fns(ii),hx,symbs);                  % Replace hex by symb
        end
        val = struct2table(val);                         % Convert struct to table
        val.Properties.VariableNames = varnames;         % Set the fixed variablenames
    end
    
    % Convert cells of char arrays to the char arrays themselves
    if class(val) == "cell"
        val = val{1,1};
    end
    
    % Convert the transformation matrices to cell arrays per timestep
    if ndims(val)==3 && all(size(val,1,2)==[4,4])
        val = squeeze(num2cell(val,[1,2]));
    end
    
    
    %% Write data to struct
    % If the dataset contains attrs, transfer them to struct
    if ~isempty(dset.Attributes)
%         if length(dset.Name) == 1
%             StrName = append('Result_0000',dset.Name);
%         elseif length(dset.Name) == 2
%             StrName = append('Result_000',dset.Name);
%         elseif length(dset.Name) == 3
%             StrName = append('Result_00',dset.Name);
%         elseif length(dset.Name) == 4
%             StrName = append('Result_0',dset.Name);
%         elseif length(dset.Name) == 5
%             StrName = append('Result_',dset.Name);
%         else 
            StrName = dset.Name;
%         end
        data.(StrName).ds = val;                          % Transfer dataset
        data.(StrName).attr = geth5attr(dset.Attributes); % Transfer attributes
    else
        data.(StrName) = val;                             % Transfer dataset
    end
end
end