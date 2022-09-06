function data = readH5(h5file,measname)
% This function imports a HDF5 file into a Matlab struct
% 
% INPUTS:    h5file       : [string] .h5 file containing dataset of 
%                           measurements.
%            measname     : [string] (optional) a specific measurement 
%                           (H5 group)
%
% OUTPUTS:   data         : [struct] data struct containing the data from 
%                           the HDF5 file
%
%% Import the HDF5 file
% Initialization 
if nargin < 2
    %In case no specific measurement is chosen
    grpname = "/"; 
else
    % Set groupname according to measurement name, starting with "/"
    if startsWith(measname,"/"), grpname = measname;
    else,                        grpname = strcat("/",measname); end
    
    %Check if the file contains the measurement (it is a top-level group)
    if ~any(endsWith(string({h5info(h5file).Groups.Name})',grpname))
        error("Given measurement name not found in HDF5 file!")
    end
end

%Check if filename refers to a hdf5 file
% if ~isfile(h5file) || ~endsWith(h5file,".h5") 
%     error("Not a valid HDF5 filename given!")
% end

%Import the h5file into a struct
data = hdf52struct(h5file,grpname);

end
