The content of this folder can be used to read an HDF5 file, writing the content of the file to a Matlab struct.
The matlab function "readH5.m" can be used to do this. It asks as input (string or char) a path to the HDF5 file that needs to be imported.
Additionally, one can use the input "measname" to import only a specific measurement of the HDF5 file, where "measname" is a subgroup of the HDF5 file. 
If no input for "measname" is given, "readH5.m" imports all measurements contained in the HDF5 file. 
The folder "functions" additionally contains all the necessary functions to execute this process.

EXAMPLE: 
Required input options:
   - h5file  	  = 'C:\Users\person\I-AM_Archive_Version_1.h5';
Optional input options:
   - measname     = 'Rec_20200701T100851Z';

then run:

readH5(h5file) or readH5(h5file,measname)

which creates a matlab struct containing the data from the HDF5 file. 