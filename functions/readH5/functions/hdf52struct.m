function data = hdf52struct(h5file,grpname)
% This function turns a hdf5 file into a struct. It recursively loops over
% all groups, locates the attributes and datasets and writes them
% accordingly to a Matlab struct.
% 
% INPUTS:    h5file       : [string] .h5 file containing dataset of 
%                           measurements.
%            grpname      : [string] a specific HDF5 group
%
% OUTPUTS:   data         : [struct] data struct containing the data from 
%                           the HDF5 group
%
%% Write hdf5 to struct
%Define groups, datasets and attributes in the current group
grps  = h5info(h5file,grpname).Groups;     %Groups underneath current group
dsets = h5info(h5file,grpname).Datasets;   %Datasets of current group
attrs = h5info(h5file,grpname).Attributes; %Attributes of current group

%Write the datasets to struct
if ~isempty(dsets)
    data = geth5dset(h5file,dsets,grpname);
end

%Write attributes to struct
if ~isempty(attrs)
    data.attr = geth5attr(attrs);
end

%Step one level lower into groups to also write their attributes and datasets
for ig = 1:length(grps)
    grp = grps(ig);
    %Write the struct fieldname as the HDF5 groupname 
    fn = extractAfter(grp.Name,find(grp.Name == '/',1,'last'));
    %Step one layer deeper
    data.(fn) = hdf52struct(h5file,grp.Name);
end
     
end