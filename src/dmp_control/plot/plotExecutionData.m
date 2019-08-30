function plotExecutionData()

addpath('utils/');

binary = true;
filename = '../data/output_data.bin';
fid = fopen(filename);
if (fid < 0)
    error('Could not load %s\n', filename);
end
    
Time = read_mat(fid, binary);
Yr_data = read_mat(fid, binary);
Y_data = read_mat(fid, binary);
dY_data = read_mat(fid, binary);
ddY_data = read_mat(fid, binary);
Y_robot_data = read_mat(fid, binary);
s_data = read_mat(fid, binary);
F_dist_data = read_mat(fid, binary);
% eo_data = read_mat(fid, binary);
% u_data = read_mat(fid, binary);

figure;
plot(Time, s_data);

% figure;
% plot(u_data');
% legend({'u_1','u_2','u_3','u_4','u_5','u_6','u_7'});

% figure;
% plot(eo_data'*180/pi);

% return

D = size(Yr_data,1);

k = 1;
fontsize = 14;
figure('NumberTitle', 'off', 'Name', 'Training Data');
for i=1:D
    subplot(D,3,k);
    plot(Time,Yr_data(i,:));
    ylabel(['dim-$' num2str(i) '$'],'interpreter','latex','fontsize',fontsize);
    if (i==1), title('pos [$m$]','interpreter','latex','fontsize',fontsize); end
    if (i==D), xlabel('time [$s$]','interpreter','latex','fontsize',fontsize); end
    subplot(D,3,k+1);
    plot(Time,dY_data(i,:));
    if (i==1), title('vel [$m/s$]','interpreter','latex','fontsize',fontsize); end
    if (i==D), xlabel('time [$s$]','interpreter','latex','fontsize',fontsize); end
    subplot(D,3,k+2);
    plot(Time,ddY_data(i,:));
    if (i==1), title('accel [$m/s^2$]','interpreter','latex','fontsize',fontsize); end
    if (i==D), xlabel('time [$s$]','interpreter','latex','fontsize',fontsize); end
    k = k+3;
end


PlotErrorNorm(Time,Y_robot_data,Yr_data)
PlotFdist(Time,F_dist_data)
plotPath3d(Y_robot_data,Yr_data)


end