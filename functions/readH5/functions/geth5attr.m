function data = geth5attr(attrs)
% This function writes the HDF5 attributes corresponding to a group as
% strings in the Matlab struct
%
% INPUTS:    attr         : [struct] HDF5 attributes corresponding to the
%                           current group, obtained with h5info
%
% OUTPUTS:   data         : [struct] data struct containing as strings the
%                           attributes corresponding to the group
%
%% Write the attributes to the struct
%Loop through the attributes
for ia = 1:length(attrs)
    %Select the attribute, take its value, and convert it to string
    attr = attrs(ia);
    str  = string(attr.Value);
    
    %If attribute is empty, replace empty 0x0 string by empty 1x1 string
    if isempty(str), str = ""; end
    
    % If the attribute name contains a space, it cannot be used as field
    % name in the struct. Hence replace it by an underscore
    attrName = strrep(attr.Name," ","_");
    
    %Write the attribute value to the attribute name field in the struct
    data.(attrName) = str;
end
end