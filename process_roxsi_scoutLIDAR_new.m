function [CHECK] = process_roxsi_scoutLIDAR_new(basedir, filename, TRAJ)
 
    %basedir = '/project/LiDAR/LiDAR_on_Waves/ROXSI/20220706-173422'
    %filename = '20220706_flight2_C06_hover.las';

s = LASread(fullfile(basedir,filename));

CC = strsplit(filename,'.');
save_file = strcat(CC{1},'.mat')


%%
xyz = [s.record.x s.record.y s.record.z];
[nn,mm]=size(xyz);

%% first sort by distance
distance = s.record.distance;

[Xr,Yr] = UTMtoxy_chinarock(xyz(:,2),xyz(:,1)); % must be northing and easting
%mean_xyz = mean(xyz(:,1:2));
xyz(:,1:2) = [Xr Yr];
[tUTC,tLST] = GPStoUTC(s.record.gps_time,filename);
tGPS = s.record.gps_time;
[T, sortedInd] = sort(tGPS);
%[T, sortedInd] = sort(tUTC);


tUTC = tUTC-7/24;  % convert to PDT
% x = x( sortedInd);
% y = y( sortedInd);
xyz = xyz( sortedInd,:);
int = s.record.intensity( sortedInd);
beamID = s.record.laser_id(sortedInd);
distance = s.record.distance(sortedInd);
amplitude = s.record.amplitude(sortedInd);
red = s.record.red(sortedInd);
green = s.record.green(sortedInd);
blue = s.record.blue(sortedInd);
laser_id = s.record.laser_id(sortedInd);

clear s

tGPS = tGPS(sortedInd);
tUTC = tUTC(sortedInd);
tUTCdt = datetime(tUTC,'convertfrom','datenum');
ddT = diff(tUTC);
tInd =  find(ddT>1e-7); % this is in datenum, dt=0.1 s is at 1.16e-6 s; TODO: What is the scan rate of the instrument?? Is that in the las?
%%

dt = 0.1;
dt_dnum = dt/86400;  % in day
duration = tGPS(end)-tGPS(1);  % in seconds
Nframes = floor(duration/dt);
frame_time_sec =  (0.5+[0:Nframes-1])*dt;
frame_time_gps = tGPS(1) + frame_time_sec;
frame_time_dnum = tUTC(1) + frame_time_sec/86400;

%NumberofFrames = length(tInd);
%TimeStamps = tUTC(tInd);
n = 1;
nkk = nan*ones(Nframes,1);
num_good = nan*ones(Nframes,1);

Frame = cell(Nframes,1);
zmin0 = -4;  % min z value of lidar point
zmax0 = 6;   % max z value of lidar point

xmax = -1e+12;  xmin = 1e+12;
ymax = -1e+12;  ymin = 1e+12;
zmax = -100;    zmin = 100;
imin = 1e+6;  imax = -1;
for i=1:Nframes,
    tstart = frame_time_gps(i) -0.5*dt;
    tend = frame_time_gps(i) +0.5*dt;
    itt = find( (tGPS>tstart) & (tGPS<=tend));
    d = distance(itt);
    zz = xyz(itt,3);   % this is the z value
    ii = find( (d>10)&(d<80) & (zz>zmin0) & (zz<zmax0) );
    igood = itt(ii);
    num_good(i) = length(igood);
    if (~isempty(igood)),
        XYZ = xyz(igood,:);
        tmin = min(XYZ(:,1));  tmax = max(XYZ(:,1));
        if (tmin<xmin),
            xmin=tmin;
        end;
        if (tmax>xmax),
            xmax=tmax;
        end;
        tmin = min(XYZ(:,2));  tmax = max(XYZ(:,2));
        if (tmin<ymin),
            ymin=tmin;
        end;
        if (tmax>ymax),
            ymax=tmax;
        end;
        tmin = min(XYZ(:,3));  tmax = max(XYZ(:,3));
        if (tmin<zmin),
            zmin=tmin;
        end;
        if (tmax>zmax),
            zmax=tmax;
        end;
      Beamnum = beamID(igood);
      Intensity = int(igood);
      min_int = min(Intensity);        max_int = max(Intensity);
      if (min_int<imin),  imin = min_int; end;
      if (max_int>imax),  imax = max_int; end;      
      Distance = distance(igood);
%       Red = red(igood);
%       Blue = red(igood);
%       Green = red(igood);
      FRAME{i} = struct('time_dnum',frame_time_dnum(i),'time_sec',frame_time_sec(i),'XYZ',XYZ,'Beamnum',Beamnum,'Intensity',Intensity,'Distance',Distance); %,'Red',Red,'Blue',Blue,'Green',Green);
    else
        FRAME{i}=[];
    end;
      n = tInd(i)+1;
    disp(sprintf('Processed Frame: i=%05d of  %d, max/min(dist) = (%4.1f, %4.1f) (m)',i,Nframes,max(Distance), min(Distance)));
    %    disp(sprintf('max/min(amplitude) = (%3d, %3d) (m)',i,NumberofFrames,max(amplitude(kk)), min(amplitude(kk))));    
    
end



INFO = struct('xmin',xmin,'xmax',xmax,'ymin',ymin,'ymax',ymax,'zmin',zmin,'zmax',zmax,'imin',imin,'imax',imax);

%% now deal with trajectory
itr = find( (TRAJ.time_dnum >= frame_time_dnum(1))& (TRAJ.time_dnum <= frame_time_dnum(end)));
[Xdrone, Ydrone] = UTMtoxy_chinarock( TRAJ.UTM_N(itr), TRAJ.UTM_E(itr));
drone_time_dnum = TRAJ.time_dnum(itr);

DRONE = struct('drone_time_dnum',drone_time_dnum,'Xdrone',Xdrone,'Ydrone',Ydrone);

            
save_basedir = '/project/LiDAR/LiDAR_on_Waves/ROXSI/mat_processed';
sfile_full = sprintf('%s/%s',save_basedir,save_file);
disp(sprintf('Saving to: %s',sfile_full));
save(sfile_full,'FRAME','frame_time_dnum','frame_time_sec','Nframes','num_good','INFO','DRONE','-v7.3');


return

