%Base Station and UE positions configuration
fc = 3e9;                             % carrier frequency (Hz)
bsPosition = [45.4786959, 9.2322961]; % lat, lon
bsAntSize = [8 8];                    % number of rows and columns in rectangular array (base station)
bsArrayOrientation = [-100 0].';      % azimuth (0 deg is East, 90 deg is North) and elevation (positive points upwards) in deg

%Conversion from x,y coordinates to lat, long for UE
uePositionCoordx = [-168 ; -152.5 ; -131.7 ; -111.7 ; -83.5 ; -55.7 ; -23.3 ; 1.9 ; 23.4 ; 50.1 ; 77.6 ; 104.4 ; 129.7 ; 150.3 ; 166.1]; 
uePositionCoordy = [10.1 ; 10.1 ; 9.9 ; 10.1 ; 10.3 ; 10.5 ; 11 ; 11.3 ; 11.5 ; 11.8 ; 12.2 ; 12.5 ; 12.9 ; 13.2 ; 13.4];

%Get time instants
x = [-168 ; -152.5 ; -131.7 ; -111.7 ; -83.5 ; -55.7 ; -23.3 ; 1.9 ; 23.4 ; 50.1 ; 77.6 ; 104.4 ; 129.7 ; 150.3 ; 166.1]; 
speed = 5;  
for i=length(x)
    x = x + 168;
    time = x / speed;
    %disp("Time: " + time + " s")
end

for i=1:length(uePositionCoordx)
    origin = [45.47841 9.232165 0];
    zUp = 1;
    [lat,lon,alt] = local2latlon(uePositionCoordx,uePositionCoordy, zUp ,origin);
end
uePosition = [lat,lon,alt];

%Need x,y coordinates for BS
origin = [45.47841 9.232165 0];
zUp = 4;
[xbsPosition,ybsPosition,zbsPosition] = latlon2local(bsPosition(1), bsPosition(2), zUp ,origin);
bsPositionCoord = [xbsPosition,ybsPosition,zbsPosition];

ueAntSize = [2 2];                    % number of rows and columns in rectangular array (UE).
ueArrayOrientation = [180 45].';      % azimuth (0 deg is East, 90 deg is North) and elevation (positive points upwards)  in deg
reflectionsOrder = 3;                 % number of reflections for ray tracing analysis (0 for LOS)
 
%Bandwidth configuration, required to set the channel sampling rate and for perfect channel estimation
SCS = 15; % subcarrier spacing
NRB = 52; % number of resource blocks, 10 MHz bandwidth

%Import and Visualize 3-D Environment with Buildings for Ray Tracing
if exist('viewer','var') && isvalid(viewer) % viewer handle exists and viewer window is open
    viewer.clearMap();
else
    viewer = siteviewer("Basemap","openstreetmap","Buildings","map.osm");    
end

%Create Base Station 
bsSite = txsite("Name","Base station", ...
    "Latitude",bsPosition(1),"Longitude",bsPosition(2),...
    "AntennaAngle",bsArrayOrientation(1:2),...
    "AntennaHeight",4,...  % in m
    "TransmitterFrequency",fc);
bsSite.show();

receivedpowers = zeros(15,1);
receivedpowers_beam = zeros(15,1);
gain_receivedpowers = zeros(15,1);
distance_ray = zeros(15,1);
delay_ray = zeros(15,1);
AoD_az_ray = zeros(15,1);
AoD_el_ray = zeros(15,1);
AoA_az_ray = zeros(15,1);
AoA_el_ray = zeros(15,1);
pathloss_ray = zeros(15,1);
reflessions_ray = zeros(15,1);
phase_shifts = zeros(15,1);

%Create UE (changing its position)
for i=1:length(uePosition)
    ueSite = rxsite("Name","UE", ...
    "Latitude",uePosition(i,1),"Longitude",uePosition(i,2), ...
    "AntennaHeight",1,... % in m
    "AntennaAngle",ueArrayOrientation(1:2),'ReceiverSensitivity', -90);
    ueSite.show();

    %Ray Tracing Analysis
    pm = propagationModel("raytracing","Method","sbr","MaxNumReflections",reflectionsOrder);
    rays = raytrace(bsSite,ueSite,pm,"Type","pathloss");
    plot(rays{1}(1,1))   %(1,1) is allowing me so show one ray per position
    pathToAs = [rays{1}.PropagationDelay]-min([rays{1}.PropagationDelay]);  % Time of arrival of each ray (normalized to 0 sec)
    avgPathGains  = -[rays{1}.PathLoss];                                    % Average path gains of each ray
    pathAoDs = [rays{1}.AngleOfDeparture];                                  % AoD of each ray
    pathAoAs = [rays{1}.AngleOfArrival];                                    % AoA of each ray
    isLOS = any([rays{1}.LineOfSight]);                                     % Line of sight flag

    pm.BuildingsMaterial = 'perfect-reflector';
    pm.TerrainMaterial = 'perfect-reflector';
    ssPerfect = sigstrength(ueSite,bsSite,pm);
    receivedpowers(i) = ssPerfect; 
    %disp("Received power using perfect reflection: " + ssPerfect + " dBm")

    delay_ray(i) = [rays{1}(1,1).PropagationDelay];
    distance_ray(i) = [rays{1}(1,1).PropagationDistance];
    AoD_az_ray(i) = [rays{1}(1,1).AngleOfDeparture(1)];
    AoD_el_ray(i) = [rays{1}(1,1).AngleOfDeparture(2)];
    AoA_az_ray(i) = [rays{1}(1,1).AngleOfArrival(1)];
    AoA_el_ray(i) = [rays{1}(1,1).AngleOfArrival(2)];
    pathloss_ray(i) = [rays{1}(1,1).PathLoss];
    reflessions_ray(i) = [rays{1}(1,1).NumInteractions];
    phase_shifts(i) = [rays{1}(1,1).PhaseShift];
    
    %Set Up CDL Channel Model
    channel = nrCDLChannel;
    channel.DelayProfile = 'Custom';
    channel.PathDelays = pathToAs;
    channel.AveragePathGains = avgPathGains;
    channel.AnglesAoD = pathAoDs(1,:);       % azimuth of departure
    channel.AnglesZoD = 90-pathAoDs(2,:);    % channel uses zenith angle, rays use elevation
    channel.AnglesAoA = pathAoAs(1,:);       % azimuth of arrival
    channel.AnglesZoA = 90-pathAoAs(2,:);    % channel uses zenith angle, rays use elevation
    channel.HasLOSCluster = isLOS;
    channel.CarrierFrequency = fc;
    channel.NormalizeChannelOutputs = false; % do not normalize by the number of receive antennas, this would change the receive power
    channel.NormalizePathGains = false;      % set to false to retain the path gains
    c = physconst('LightSpeed');
    lambda = c/fc;

    %Geometric propagation delay calculations
    x_calc = (xbsPosition - uePositionCoordx).^2;
    y_calc = (ybsPosition - uePositionCoordy).^2;
    z_calc = (zbsPosition - 1).^2;
    geometric_distance = sqrt(x_calc+y_calc+z_calc);
    geometric_delay = geometric_distance/c;

    % UE array (single panel)
    ueArray = phased.NRRectangularPanelArray('Size',[ueAntSize(1:2) 1 1],'Spacing', [0.5*lambda*[1 1] 1 1]);
    ueArray.ElementSet = {phased.IsotropicAntennaElement};   % isotropic antenna element
    channel.ReceiveAntennaArray = ueArray;
    channel.ReceiveArrayOrientation = [ueArrayOrientation(1); (-1)*ueArrayOrientation(2); 0];  % the (-1) converts elevation to downtilt

    % Base station array (single panel)
    bsArray = phased.NRRectangularPanelArray('Size',[bsAntSize(1:2) 1 1],'Spacing', [0.5*lambda*[1 1] 1 1]);
    bsArray.ElementSet = {phased.NRAntennaElement('PolarizationAngle',-45) phased.NRAntennaElement('PolarizationAngle',45)}; % cross polarized elements
    channel.TransmitAntennaArray = bsArray;
    channel.TransmitArrayOrientation = [bsArrayOrientation(1); (-1)*bsArrayOrientation(2); 0];   % the (-1) converts elevation to downtilt

    %Set Channel Sampling Rate
    ofdmInfo = nrOFDMInfo(NRB,SCS);
    channel.SampleRate = ofdmInfo.SampleRate;

end 

%Use Beam Steering to Enhance Received Power --> I can only achieve this if I do a separate 
%                                                for loop for calculating new received powers
%                                                (otherwise I seem to compromise original values of RP)
for i=1:length(uePosition)
    ueSite = rxsite("Name","UE", ...
    "Latitude",uePosition(i,1),"Longitude",uePosition(i,2), ...
    "AntennaHeight",1,... % in m
    "AntennaAngle",ueArrayOrientation(1:2),'ReceiverSensitivity', -90);
    pm = propagationModel("raytracing","Method","sbr","MaxNumReflections",reflectionsOrder);
    rays = raytrace(bsSite,ueSite,pm,"Type","pathloss");
    plot(rays{1}(1,1))  
    pathToAs = [rays{1}.PropagationDelay]-min([rays{1}.PropagationDelay]);  
    avgPathGains  = -[rays{1}.PathLoss];                                    
    pathAoDs = [rays{1}.AngleOfDeparture];                                  
    pathAoAs = [rays{1}.AngleOfArrival];                                    
    isLOS = any([rays{1}.LineOfSight]);                                     
    pm.BuildingsMaterial = 'perfect-reflector';
    pm.TerrainMaterial = 'perfect-reflector';
    bsSite.Antenna = helperM2412PhasedArray(bsSite.TransmitterFrequency);
    bsSite.AntennaAngle = -100;

    steeringaz = wrapTo180(AoD_az_ray(i)-bsSite.AntennaAngle);
    steeringVector = phased.SteeringVector("SensorArray",bsSite.Antenna);
    sv = steeringVector(bsSite.TransmitterFrequency,[steeringaz;AoD_el_ray(i)]);
    bsSite.Antenna.Taper = conj(sv);
    pattern(bsSite,fc,"Size",10);
    ss = sigstrength(ueSite,bsSite,pm);
    receivedpowers_beam(i) = ss;
    %disp("Received power with beam steering: " + ss + " dBm")

    gain_receivedpowers(i) = receivedpowers_beam(i) - receivedpowers(i);
    %disp("The new received power increases by about: " + gain_receivedpowers(i) + " dBm")
end
avg_gain = mean(gain_receivedpowers);
disp("The average received power gain with beam steering is: " + avg_gain + " dBm")


%Calculations of nominal path loss for each UE position
viewer2 = siteviewer("Basemap","openstreetmap");    
bsSite = txsite("Name","Base station", ...
    "Latitude",bsPosition(1),"Longitude",bsPosition(2),...
    "AntennaAngle",bsArrayOrientation(1:2),...
    "AntennaHeight",20,...  % in m
    "TransmitterFrequency",fc);
bsSite.show();

nominal_pathloss_ray = zeros(15,1);
for i=1:length(uePosition)
    ueSite = rxsite("Name","UE", ...
    "Latitude",uePosition(i,1),"Longitude",uePosition(i,2), ...
    "AntennaHeight",1,... % in m
    "AntennaAngle",ueArrayOrientation(1:2),'ReceiverSensitivity', -90);
    ueSite.show();
    pm = propagationModel("raytracing","Method","sbr","MaxNumReflections",reflectionsOrder);
    rays = raytrace(bsSite,ueSite,pm,"Type","pathloss");
    plot(rays{1}(1,1))   %(1,1) is allowing me so show one ray per position
    nominal_pathloss_ray(i) = [rays{1}(1,1).PathLoss];
    
end 



%Graphs below


figure(1);
cmap = AoA_az_ray.';
grid on; hold on;
c = colorbar; hold on;
colormap hot; hold on;
c.Label.String = 'AoA azimuth [deg]';
c.Label.FontSize = 12;
scatter(time, receivedpowers_beam, 60, cmap, 'filled');hold on;
hp1 = plot(time, receivedpowers_beam, 'k' ,'LineStyle','-'); hold off;
xlabel('Time [s]');
ylabel('Received Power [dBm]');
title('Evolution of RP with Beam Steering upon AoA(az) changes');

figure(2);
cmap = AoA_el_ray.';
grid on; hold on;
c = colorbar; hold on;
colormap spring; hold on;
c.Label.String = 'AoA elevation [deg]';
c.Label.FontSize = 12;
scatter(time, receivedpowers_beam, 60, cmap, 'filled');hold on;
hp2 = plot(time, receivedpowers_beam, 'k' ,'LineStyle','-'); hold off;
xlabel('Time [s]');
ylabel('Received Power [dBm]');
title('Evolution of RP with Beam Steering upon AoA(el) changes');

figure(3);
cmap = AoD_az_ray.';
grid on; hold on;
c = colorbar; hold on;
colormap hot; hold on;
c.Label.String = 'AoD azimuth [deg]';
c.Label.FontSize = 12;
scatter(time, receivedpowers_beam, 60, cmap, 'filled');hold on;
hp3 = plot(time, receivedpowers_beam, 'k' ,'LineStyle','-'); hold off;
xlabel('Time [s]');
ylabel('Received Power [dBm]');
title('Evolution of RP with Beam Steering upon AoD(az) changes');

figure(4);
cmap = AoD_el_ray.';
grid on; hold on;
c = colorbar; hold on;
colormap spring; hold on;
c.Label.String = 'AoD elevation [deg]';
c.Label.FontSize = 12;
scatter(time, receivedpowers_beam, 60, cmap, 'filled');hold on;
hp4 = plot(time, receivedpowers_beam, 'k' ,'LineStyle','-'); hold off;
xlabel('Time [s]');
ylabel('Received Power [dBm]');
title('Evolution of RP with Beam Steering upon AoD(el) changes');

figure(5);
cmap = phase_shifts.';
grid on; hold on;
c = colorbar; hold on;
c.Label.String = 'Phase Shift [rad]';
c.Label.FontSize = 12;
scatter(time, receivedpowers_beam, 60, cmap, 'filled');hold on;
hp5 = plot(time, receivedpowers_beam, 'k' ,'LineStyle','-'); hold off;
xlabel('Time [s]');
ylabel('Received Power [dBm]');
title('Evolution of RP upon PS changes');

figure(6);
cmap = reflessions_ray.';
grid on; hold on;
c = colorbar; hold on;
c.Ticks = [0 1 2 3];
c.Label.String = 'Number of Reflessions';
c.Label.FontSize = 12;
scatter(time, receivedpowers, 60, cmap, 'filled'); hold on;
scatter(time, receivedpowers_beam, 60, cmap, 'filled');hold on;
hp6 = plot(time, receivedpowers, 'k'); hold on;
hp7 = plot(time, receivedpowers_beam, 'k' ,'LineStyle','--'); hold off;
xlabel('Time [s]');
ylabel('Received Power [dBm]');
title('Evolution of RP as UE moves');
legend([hp6 hp7], "without Beam Steering", "with Beam Steering")
 
figure(7);
cmap = reflessions_ray.';
grid on; hold on;
c = colorbar; hold on;
c.Ticks = [0 1 2 3];
c.Label.String = 'Number of Reflessions';
c.Label.FontSize = 12;
scatter(time, delay_ray, 60, cmap, 'filled'); hold on;
hp8 = plot(time, delay_ray, 'k'); hold on;
hp9 = plot(time, geometric_delay, 'r' ,'LineStyle','--'); hold off;
xlabel('Time [s]');
ylabel('Propagation Delay [s]');
title('Evolution of PD of the rays as UE moves');
legend([hp8 hp9], "Propagation Delay", "Geometric Propagation Delay")

figure(8);
cmap = reflessions_ray.';
grid on; hold on;
c = colorbar; hold on;
c.Ticks = [0 1 2 3];
c.Label.String = 'Number of Reflessions';
c.Label.FontSize = 12;
scatter(time, pathloss_ray, 60, cmap, 'filled'); hold on;
hp10 = plot(time, pathloss_ray, 'k'); hold on;
hp11 = plot(time, nominal_pathloss_ray, 'r', 'LineStyle','--'); hold off;
xlabel('Time [s]');
ylabel('Path Loss [dBm]');
title('Evolution of PL of the rays as UE moves');
legend([hp10 hp11], "Path Loss", "Nominal Path Loss")

figure(9);
tiledlayout(2,1);

nexttile
cmap = reflessions_ray.';
scatter(time, AoA_az_ray, 60, cmap, 'filled')
grid on;
c = colorbar;
c.Ticks = [0 1 2 3];
c.Label.String = 'Number of Reflessions';
c.Label.FontSize = 12;
hold on;
plot(time, AoA_az_ray, 'k');
grid on;
xlabel('Time [s]');
ylabel('Angle of Arrival Azimuth [deg]');
title('Evolution of AoA(az) of the rays as UE moves');

nexttile
cmap = reflessions_ray.';
scatter(time, AoA_el_ray, 60, cmap, 'filled')
grid on;
c = colorbar;
c.Ticks = [0 1 2 3];
c.Label.String = 'Number of Reflessions';
c.Label.FontSize = 12;
hold on;
plot(time, AoA_el_ray, 'k');
grid on;
xlabel('Time [s]');
ylabel('Angle of Arrival Elevation [deg]');
title('Evolution of AoA(el) of the rays as UE moves');

figure(10);
cmap = reflessions_ray.';
scatter(time, phase_shifts, 60, cmap, 'filled')
grid on;
c = colorbar;
c.Ticks = [0 1 2 3];
c.Label.String = 'Number of Reflessions';
c.Label.FontSize = 12;
hold on;
plot(time, phase_shifts, 'k');
grid on;
xlabel('Time [s]');
ylabel('Phase Shift [rad]');
title('Evolution of PS of the rays as UE moves');

figure(11);
tiledlayout(2,1);

nexttile
custom_colormap = [
    1  0 0 ; ... %// red
    1 .5 0 ; ... %// orange
    1  1 0 ; ... %// yellow
    0  1 0 ; ... %// green
    0  0 1 ; ... %// blue
    ] ;

xx=[x x];                                       % create a 2D matrix 
cc =[distance_ray distance_ray];                % matrix for "CData"
zz=zeros(size(xx));                             % everything in the Z=0 plane

yy=[receivedpowers receivedpowers];             
hs1=surf(xx,yy,zz,cc,'EdgeColor','interp','FaceColor','none', 'LineWidth', 4, 'Marker','o','MarkerSize', 6, 'MarkerFaceColor','k', 'MarkerEdgeColor','k') ;
colormap(custom_colormap) ;       % assign the colormap
shading interp                    % so each line segment has a plain color
view(2)                           % set view in X-Y plane
xlabel('Distance [m]');
ylabel('Received Power [dBm]');
title('Evolution of RP of the rays as UE moves');
c = colorbar;
c.Label.String = 'Propagation Distance of the Ray [m]';
c.Label.FontSize = 12;

nexttile
yy2=[receivedpowers_beam receivedpowers_beam]; 
hs2=surf(xx,yy2,zz,cc,'EdgeColor','interp','FaceColor','none', 'LineWidth', 4, 'Marker','o','MarkerSize', 6, 'MarkerFaceColor','k', 'MarkerEdgeColor','k') ;
colormap(custom_colormap) ;       
shading interp                    
view(2)                          
xlabel('Distance [m]');
ylabel('Received Power with Beam Steering [dBm]');
title('Evolution of RP of the rays with BS as UE moves');
c = colorbar;
c.Label.String = 'Propagation Distance of the Ray [m]';
c.Label.FontSize = 12;

figure(12);
yy3=[delay_ray delay_ray]; 
hs3=surf(xx,yy3,zz,cc,'EdgeColor','interp','FaceColor','none', 'LineWidth', 4, 'Marker','o','MarkerSize', 6, 'MarkerFaceColor','k', 'MarkerEdgeColor','k') ;
colormap(custom_colormap) ;       
shading interp                    
view(2)                           
xlabel('Distance [m]');
ylabel('Propagation Delay [s]');
title('Evolution of PD of the rays as UE moves');
c = colorbar;
c.Label.String = 'Propagation Distance of the Ray [m]';
c.Label.FontSize = 12;

figure(13);
yy4=[pathloss_ray pathloss_ray]; 
hs4=surf(xx,yy4,zz,cc,'EdgeColor','interp','FaceColor','none', 'LineWidth', 4, 'Marker','o','MarkerSize', 6, 'MarkerFaceColor','k', 'MarkerEdgeColor','k') ;
colormap(custom_colormap) ;       
shading interp                      
view(2)                           
xlabel('Distance [m]');
ylabel('Path Loss [dBm]');
title('Evolution of PL of the rays as UE moves');
c = colorbar;
c.Label.String = 'Propagation Distance of the Ray [m]';
c.Label.FontSize = 12;


figure(14);
tiledlayout(2,1);

nexttile
yy5=[AoA_az_ray AoA_az_ray]; 
hs5=surf(xx,yy5,zz,cc,'EdgeColor','interp','FaceColor','none', 'LineWidth', 4, 'Marker','o','MarkerSize', 6, 'MarkerFaceColor','k', 'MarkerEdgeColor','k') ;
colormap(custom_colormap) ;       
shading interp                    
view(2)                           
xlabel('Distance [m]');
ylabel('Angle of Arrival Azimuth [deg]');
title('Evolution of AoA(az) of the rays as UE moves');
c = colorbar;
c.Label.String = 'Propagation Distance of the Ray [m]';
c.Label.FontSize = 12;

nexttile
yy6=[AoA_el_ray AoA_el_ray]; 
hs6=surf(xx,yy6,zz,cc,'EdgeColor','interp','FaceColor','none', 'LineWidth', 4, 'Marker','o','MarkerSize', 6, 'MarkerFaceColor','k', 'MarkerEdgeColor','k') ;
colormap(custom_colormap) ;       
shading interp                    
view(2)                           
xlabel('Distance [m]');
ylabel('Angle of Arrival Elevation Azimuth [deg]');
title('Evolution of AoA(el) of the rays as UE moves');
c = colorbar;
c.Label.String = 'Propagation Distance of the Ray [m]';
c.Label.FontSize = 12;

figure(15);
tiledlayout(2,1);

nexttile
yy7=[AoD_az_ray AoD_az_ray]; 
hs7=surf(xx,yy7,zz,cc,'EdgeColor','interp','FaceColor','none', 'LineWidth', 4, 'Marker','o','MarkerSize', 6, 'MarkerFaceColor','k', 'MarkerEdgeColor','k') ;
colormap(custom_colormap) ;       
shading interp                    
view(2)                           
xlabel('Distance [m]');
ylabel('Angle of Departure Azimuth [deg]');
title('Evolution of AoD(az) of the rays as UE moves');
c = colorbar;
c.Label.String = 'Propagation Distance of the Ray [m]';
c.Label.FontSize = 12;

nexttile
yy8=[AoA_el_ray AoA_el_ray]; 
hs8=surf(xx,yy8,zz,cc,'EdgeColor','interp','FaceColor','none', 'LineWidth', 4, 'Marker','o','MarkerSize', 6, 'MarkerFaceColor','k', 'MarkerEdgeColor','k') ;
colormap(custom_colormap) ;       
shading interp                    
view(2)                           
xlabel('Distance [m]');
ylabel('Angle of Departure Elevation Azimuth [deg]');
title('Evolution of AoD(el) of the rays as UE moves');
c = colorbar;
c.Label.String = 'Propagation Distance of the Ray [m]';
c.Label.FontSize = 12;

figure(16);
yy9=[phase_shifts phase_shifts]; 
hs9=surf(xx,yy9,zz,cc,'EdgeColor','interp','FaceColor','none', 'LineWidth', 4, 'Marker','o','MarkerSize', 6, 'MarkerFaceColor','k', 'MarkerEdgeColor','k') ;
colormap(custom_colormap) ;       
shading interp                    
view(2)                           
xlabel('Distance [m]');
ylabel('Phase Shift [rad]');
title('Evolution of PS of the rays as UE moves');
c = colorbar;
c.Label.String = 'Propagation Distance of the Ray [m]';
c.Label.FontSize = 12;



%Geoplots below

%This is GeoBubble: it's also showing data besides path of the vehicle, but not the BS position 
figure(17);
colordata = categorical(reflessions_ray);
gb = geobubble(lat,lon,pathloss_ray, colordata,'Title','Path Loss of the Channel');
gb.Basemap = 'satellite';
gb.SizeLegendTitle = 'Path Loss [dBm]';
gb.ColorLegendTitle = '# of Reflessions';


figure(18);
colordata = categorical(reflessions_ray);
gb = geobubble(lat,lon,delay_ray, colordata,'Title','Propagation Delay of the Channel');
gb.Basemap = 'satellite';
gb.SizeLegendTitle = 'Propagation Delay [s]';
gb.ColorLegendTitle = '# of Reflessions';

figure(19);
colordata = categorical(reflessions_ray);
gb = geobubble(lat,lon,phase_shifts, colordata,'Title','Phase Shifts of the Channel');
gb.Basemap = 'satellite';
gb.SizeLegendTitle = 'Phase Shifts [rad]';
gb.ColorLegendTitle = '# of Reflessions';

figure(20);
colordata = categorical(reflessions_ray);
gb = geobubble(lat,lon,receivedpowers_beam, colordata,'Title','Received Power with Beam Steering of the Channel');
gb.Basemap = 'satellite';
gb.SizeLegendTitle = 'Received Power [dBm]';
gb.ColorLegendTitle = '# of Reflessions';


%This is geoplot: it's showing the path and the position of the BS
figure(21);
g2 = geoplot(lat,lon,'r','LineWidth',3); hold on;
g3 = geoplot(bsPosition(1),bsPosition(2),'Marker','x','MarkerFaceColor','r','MarkerSize',15,'LineWidth', 10); hold on;
g4 = geoplot(lat(1),lon(1),'Marker','.','MarkerSize',30,'MarkerFaceColor','y','MarkerEdgeColor','y','LineWidth', 20); hold on;
g5 = geoplot(lat(15),lon(15),'Marker','.','MarkerSize',30,'MarkerFaceColor','y','MarkerEdgeColor','y','LineWidth', 20); hold on;
text(lat(1),lon(1),'Start', 'Color', 'white','FontSize',15,'FontWeight','bold','VerticalAlignment','bottom');
text(lat(15),lon(15),'Finish', 'Color', 'white','FontSize',15,'FontWeight','bold', 'HorizontalAlignment','right','VerticalAlignment','bottom');
text(bsPosition(1),bsPosition(2),'Base Station', 'Color', 'white','FontSize',15,'FontWeight','bold','HorizontalAlignment','right','VerticalAlignment','bottom');
title('Route of the vehicle and Base Station Position');
geobasemap satellite

%This is geoplot: it's showing the path, the position of the BS, and data
figure(22);
g6 = geoplot(lat,lon, 'r','LineStyle','-.','LineWidth',3); hold on;
g7 = geoplot(bsPosition(1),bsPosition(2),'Marker','x','MarkerFaceColor','w','MarkerSize',15,'LineWidth', 10); hold on;
g8 = geoplot(lat(1),lon(1),'Marker','.','MarkerSize',30,'MarkerFaceColor','y','MarkerEdgeColor','y','LineWidth', 20); hold on;
g9 = geoplot(lat(15),lon(15),'Marker','.','MarkerSize',30,'MarkerFaceColor','y','MarkerEdgeColor','y','LineWidth', 20); hold on;
cmap = pathloss_ray.';
grid on; hold on;
c = colorbar; hold on;
c.Label.String = 'Path Loss [dBm]';
c.Label.FontSize = 12;
g10 = geoscatter(lat, lon, 150, cmap, 'filled'); hold on;
text(lat(1),lon(1),'Start', 'Color', 'white','FontSize',15,'FontWeight','bold','VerticalAlignment','bottom');
text(lat(15),lon(15),'Finish', 'Color', 'white','FontSize',15,'FontWeight','bold', 'HorizontalAlignment','right','VerticalAlignment','bottom');
text(bsPosition(1),bsPosition(2),'Base Station', 'Color', 'white','FontSize',15,'FontWeight','bold','HorizontalAlignment','right','VerticalAlignment','bottom');
%text(45.4787,9.2323,'Base Station', 'Color', 'r','FontSize',15,'FontWeight','bold','HorizontalAlignment','center','VerticalAlignment','bottom');
title('Evolution of PL as the UE moves and Base Station Position');
geobasemap satellite
