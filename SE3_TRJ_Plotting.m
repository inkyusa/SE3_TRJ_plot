%============================================
%	SE3 Trajectory plot
%	Author: Inkyu Sa, enddl22@gmail.com
%	http://www.enddl22.net
%	
%	This program is free software: you can redistribute it and/or modify
%	it under the terms of the GNU General Public License as published by
%	the Free Software Foundation, either version 3 of the License, or
%	(at your option) any later version.
%
%	This program is distributed in the hope that it will be useful,
%	but WITHOUT ANY WARRANTY; without even the implied warranty of
%	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%	GNU General Public License for more details.
%
%	You should have received a copy of the GNU General Public License
%	along with this program.  If not, see <http://www.gnu.org/licenses/>.
%
%	
%============================================


clear all;
close all;
addpath('./func');

imu=importdata('./data/sfly/imu_1loopDown.txt',' ',3);
vicon = importdata('./data/sfly/vicon_1loopDown.txt',' ',3);

imu_acc=struct('x',imu.data(:,2),'y',imu.data(:,3),'z',imu.data(:,4));
vicon_pose =struct('x',vicon.data(:,2),'y',vicon.data(:,3),'z',vicon.data(:,4));
vicon_angle = struct('roll',vicon.data(:,5),'pitch',vicon.data(:,6),'yaw',vicon.data(:,7));
time=struct('imu',imu.data(:,1),'vicon',vicon.data(:,1));

time.vicon=time.vicon-time.vicon(1);
time.imu = time.imu-time.imu(1);

length = 0.1;

aviobj = VideoWriter('test.avi');
open(aviobj);
scrsz = get(0,'ScreenSize');
%ScreenSize is a four-element vector: [left, bottom, width, height]:

fig=figure('Position',[1 scrsz(4)/2 scrsz(3)/2 scrsz(4)/2]);
title_handle = title('Quadrotor 6DOF coordinates plot');


itv=20;
rotation_spd=0.5;
delay=0.02;

az=15;
el=64;
view(az,el);
grid on;
xlabel('x', 'fontsize',16);
ylabel('y', 'fontsize',16);
zlabel('z', 'fontsize',16);
h_legend=legend('X','Y','Z');


count=1;
for i=1:itv:numel(vicon_angle.roll)
    el=64;
    
    % From peter's RVC toolbox
    %http://petercorke.com/wordpress/toolboxes/robotics-toolbox#Downloading_the_Toolbox
    R = rpy2r(vicon_angle.roll(i),vicon_angle.pitch(i),vicon_angle.yaw(i));

    % generate axis vectors
    tx = [length,0.0,0.0];
    ty = [0.0,length,0.0];
    tz = [0.0,0.0,length];
    % Rotate it by R
    t_x_new = R*tx';
    t_y_new = R*ty';
    t_z_new = R*tz';
    
    
    
    % translate vectors to camera position. Make the vectors for plotting
    origin=[vicon_pose.x(i),vicon_pose.y(i),vicon_pose.z(i)];
    tx_vec(1,1:3) = origin;
    tx_vec(2,:) = t_x_new + origin';
    ty_vec(1,1:3) = origin;
    ty_vec(2,:) = t_y_new + origin';
    tz_vec(1,1:3) = origin;
    tz_vec(2,:) = t_z_new + origin';
    hold on;
    
    
    
    % Plot the direction vectors at the point
    p1=plot3(tx_vec(:,1), tx_vec(:,2), tx_vec(:,3));
    set(p1,'Color','Green','LineWidth',1);
    p1=plot3(ty_vec(:,1), ty_vec(:,2), ty_vec(:,3));
    set(p1,'Color','Blue','LineWidth',1);
    p1=plot3(tz_vec(:,1), tz_vec(:,2), tz_vec(:,3));
    set(p1,'Color','Red','LineWidth',1);
    
    perc = count*itv/numel(vicon_angle.roll)*100;
    %fprintf('Process = %f\n',perc);
    %text(1,-3,0,['Process = ',num2str(perc),'%']);
    set(title_handle,'String',['Process = ',num2str(perc),'%'],'fontsize',16);
    count=count+1;   
    
    az=az+rotation_spd;
    view(az,el);
    drawnow;
    pause(delay);  % in second
    f = getframe(fig);
    %aviobj=addframe(aviobj,f);
    writeVideo(aviobj,f);
end;


close(aviobj);

fprintf('Done\n');






