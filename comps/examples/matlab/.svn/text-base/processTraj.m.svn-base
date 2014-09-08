%% post-process trajectories generated with CBiRRT
%   USAGE: [data]=processTraj(filename,timestep,[newname])
%   filename is the path to the raw trajectory file (with extension)
%   timestep is the new timestep to re-interpolate the trajectory with
%   
%   This function uses MATLAB's interp1 function to interpolate over the input
%   trajectory at the specified timesteps. This is written to the file given in
%   "newname"
%   Author: Rob Ellenberg
%   
function [fname,data]=processTraj(name,tstep,fname)
    
    fid=fopen(name);

    if ~exist('fname')
        fname=['new-' name];
    end

    line1=fgetl(fid);
    line2=fgetl(fid);
    initdata=sscanf(line2,'%f')';

    A=[];
    while ~feof(fid)
        tline=fgets(fid);
        A=[A;sscanf(tline,'%f')'];
    end
    fclose(fid);
    tmax=max(A(:,1));
    tused=[0:tstep:tmax,tmax];
    
    data=interp1(A(:,1),A(:,2:end),tused);
    fout=fopen(fname,'w');
    fprintf(fout,line1);fprintf(fout,'\n');
    initdata(1)=length(tused);
    fprintf(fout,'%d %d %d\n',initdata);
    for k=1:length(tused)
        fprintf(fout,'%f ',[tused(k),data(k,:)]);
        fprintf(fout,'\n');
    end
    fclose(fout);
end

