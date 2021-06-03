function [P_ate, rot_rmse] = check_viral_cal_err_parall(test_id, test_fullname)

% clearvars('-except', vars_to_keep{:});
close all;

% addpath('/home/tmn/MATLAB_WS');

t_shift = 0;

mygreen   = [0.0,    0.75,   0.0];
mymagenta = [1.0,    0.0,    1.0];
myorange  = [1.0,    0.3333, 0.0];
mycyan    = [0.0314, 0.75, 0.75];

fighd = [];



%% Get the exp number
exp_name =  test_fullname.name;
exp_path = [test_fullname.folder '/' test_fullname.name '/'];


gndtr_pos_fn       = [exp_path 'leica_pose.csv'];
gndtr_dji_imu_fn   = [exp_path 'dji_sdk_imu.csv'];
gndtr_vn100_imu_fn = [exp_path 'vn100_imu.csv'];
rtopt_est_fn       = [exp_path 'opt_odom.csv'];
baopt_est_fn       = [exp_path 'vins_ba_pose_graph.csv'];
trans_B2prism_fn   = [exp_path '../trans_B2prism.csv'];
rot_B2vn100_fn     = [exp_path '../rot_B2vn100.csv'];
rot_B2djiimu_fn    = [exp_path '../rot_B2djiimu.csv'];


% Translation from body frame to the prism
trans_B2prism = csvread(trans_B2prism_fn, 0, 0);



%% Read the data from log and start processing

% Read the gndtr data from logs
for dummy = 1

% Position groundtr
gndtr_pos_data = csvread(gndtr_pos_fn,  1, 0);

% Orientation groundtr
% Check if file size is 0
dji_file   = dir(gndtr_dji_imu_fn);
vn100_file = dir(gndtr_vn100_imu_fn);
imu_topic = '';
% Orientation is in z upward frame, convert it to body frame
rot_B_Beimu = eye(3);

dji_present   = (dji_file.bytes ~= 0);
vn100_present = (vn100_file.bytes ~= 0);

if vn100_present
    gndtr_vn100_data = csvread([exp_path 'vn100_imu.csv'], 1, 0);
end

if dji_present
    gndtr_dji_data = csvread(gndtr_dji_imu_fn, 1, 0);
    rot_B_Beimu    = csvread(rot_B2djiimu_fn, 0, 0);
    imu_topic = '/dji_sdk/imu';
elseif ~dji_present && vn100_present
    gndtr_dji_data = csvread([exp_path 'vn100_imu.csv'], 1, 0);
    rot_B_Beimu    = csvread(rot_B2vn100_fn,  0, 0);
    imu_topic = '/imu/imu';
end

t0_ns = gndtr_pos_data(1, 1);

% pos groundtruthdata
t_pos = (gndtr_pos_data(:, 1) - t0_ns)/1e9 + t_shift;
P     = gndtr_pos_data(:, 4:6);

% ori groundtruthdata
t_ori = (gndtr_dji_data(:, 1)- t0_ns)/1e9;
Q     =  quatnormalize(gndtr_dji_data(:, [7, 4:6]));
Q0    =  Q(1, :);

% Delete the duplicate in position groundtruth data
[~, Px_unq_idx] = unique(P(:, 1));
[~, Py_unq_idx] = unique(P(:, 2));
[~, Pz_unq_idx] = unique(P(:, 3));

P_unq_idx = union(union(Px_unq_idx, Py_unq_idx), Pz_unq_idx);
P = P(P_unq_idx, :);
t_pos = t_pos(P_unq_idx, :);

% Delete the duplicate in orientation groundtruth data
[~, Qx_unq_idx] = unique(Q(:, 1));
[~, Qy_unq_idx] = unique(Q(:, 2));
[~, Qz_unq_idx] = unique(Q(:, 3));
[~, Qw_unq_idx] = unique(Q(:, 4));

Q_unq_idx = union(union(union(Qx_unq_idx, Qy_unq_idx), Qz_unq_idx),...
                        Qw_unq_idx);
Q     = Q(Q_unq_idx, :);
t_ori = t_ori(Q_unq_idx, :);

Q_B_Beimu = rotm2quat(rot_B_Beimu);
Q = quatmultiply(Q, quatinv(Q_B_Beimu));

end

% Read the viral fusion estimate data from logs
for dummy = 1

% SLAM estimate
viral_data = csvread(rtopt_est_fn, 1, 0);
t_h = (viral_data(:, 1) - t0_ns)/1e9;
P_h =  viral_data(:, 4:6);
Q_h =  quatnormalize(viral_data(:, [10, 7:9]));
V_h =  viral_data(:, 11:13);

% Compensate the position estimate with the prism displacement
P_h = P_h + quatconv(Q_h, trans_B2prism);

end

% Read the ba fusion estimate data from logs
for dummy = 1
    
% BA estimate
ba_data = csvread(baopt_est_fn, 0, 0);
t_ba = ba_data(:, 2) - t0_ns/1e9;
P_ba = ba_data(:, 10:12);
Q_ba = ba_data(:, 13:16);

% Compensate the position estimate with the prism displacement
P_ba = P_ba + quatconv(Q_ba, trans_B2prism);
  
end


%% Resample and align the groundtruth with each estimate

%Resample gndtruth by viral time
for dummy = 1
    
% Find the interpolated time stamps
[rsh_pos_itp_idx(:, 1), rsh_pos_itp_idx(:, 2)] = combteeth(t_h, t_pos);
[rsh_ori_itp_idx(:, 1), rsh_ori_itp_idx(:, 2)] = combteeth(t_h, t_ori);

% Remove the un-associatable samples
rsh_nan_idx = find(isnan(rsh_pos_itp_idx(:, 1))...
                   | isnan(rsh_pos_itp_idx(:, 2))...
                   | isnan(rsh_ori_itp_idx(:, 1))...
                   | isnan(rsh_ori_itp_idx(:, 2)));

t_h_org = t_h;
P_h_org = P_h;
Q_h_org = Q_h;
V_h_org = V_h;

rsh_pos_itp_idx(rsh_nan_idx, :) = [];
rsh_ori_itp_idx(rsh_nan_idx, :) = [];
t_h(rsh_nan_idx, :)     = [];
P_h(rsh_nan_idx, :)     = [];
Q_h(rsh_nan_idx, :)     = [];
V_h(rsh_nan_idx, :)     = [];

% interpolate the pos gndtr state
P_rsh = vecitp(P,  t_pos, t_h, rsh_pos_itp_idx);

% Align the viral estimate with ground truth
[rot_align_h, trans_align_h ] = traj_align(P_rsh, P_h);

% Align the position estimate
P_h      = (rot_align_h*P_h'     + trans_align_h)';
P_h_full = (rot_align_h*P_h_org' + trans_align_h)';

% Align the velocity estimate
V_h      = (rot_align_h*V_h')';
V_h_full = (rot_align_h*V_h_org')';

% interpolate the ori gndtr state
Q_rsh = quatitp(Q, t_ori, t_h, rsh_ori_itp_idx);


% Find the optimized rotation between the groundtruth and the estimate
rot_rsh = quat2rotm(Q_rsh);
rot_h   = quat2rotm(Q_h);

rot_rsh2h_opt = rot_opt(rot_rsh, rot_h);

% Align the ori estimate
Q_h = quatmultiply(rotm2quat(rot_rsh2h_opt), Q_h);

end


%Resample gndtruth by ba time
for dummy = 1
    
% Find the interpolated time stamps
[rsba_pos_itp_idx(:, 1), rsba_pos_itp_idx(:, 2)] = combteeth(t_ba, t_pos);
[rsba_ori_itp_idx(:, 1), rsba_ori_itp_idx(:, 2)] = combteeth(t_ba, t_ori);

% Remove the un-associatable samples
rsba_nan_idx = find(isnan(rsba_pos_itp_idx(:, 1))...
                   | isnan(rsba_pos_itp_idx(:, 2))...
                   | isnan(rsba_ori_itp_idx(:, 1))...
                   | isnan(rsba_ori_itp_idx(:, 2)));

t_ba_org = t_ba;
P_ba_org = P_ba;
Q_ba_org = Q_ba;

rsba_pos_itp_idx(rsba_nan_idx, :) = [];
rsba_ori_itp_idx(rsba_nan_idx, :) = [];
t_ba(rsba_nan_idx, :) = [];
P_ba(rsba_nan_idx, :) = [];
Q_ba(rsba_nan_idx, :) = [];

% interpolate the pos gndtr state
P_rsba = vecitp(P,  t_pos, t_ba, rsba_pos_itp_idx);

% Align the viral estimate with ground truth
[rot_align_ba, trans_align_ba ] = traj_align(P_rsba, P_ba);

% Align the position estimate
P_ba      = (rot_align_ba*P_ba'     + trans_align_ba)';
P_ba_full = (rot_align_ba*P_ba_org' + trans_align_ba)';

% interpolate the ori gndtr state
Q_rsba = quatitp(Q, t_ori, t_ba, rsba_ori_itp_idx);

% Find the optimized rotation between the groundtruth and the estimate
rot_rsba = quat2rotm(Q_rsba);
rot_ba   = quat2rotm(Q_ba);

rot_rsba2ba_opt = rot_opt(rot_rsba, rot_ba);

% Align the ori estimate
Q_ba = quatmultiply(rotm2quat(rot_rsba2ba_opt), Q_ba);

end



% %% Log down the transforms in yaml files for the vizualization
% 
% % Export the leica transform to a yaml file   
% for dummy = 1
% 
% fileID = fopen([exp_name '/leica_tf.yaml'], 'w');
% fprintf(fileID, ['%%YAML:1.0\n'...
%                  'T_W_Wleica: !!opencv-matrix\n'...
%                  '  rows: 4\n'...
%                  '  cols: 4\n'...
%                  '  dt: d\n']);
% R_W2L   =  rot_align_h';
% t_W2L   = -rot_align_h'*trans_align_h;
% T_W2L   = [R_W2L, t_W2L; 0 0 0 1];
% T_W2L_str = sprintf(['  data: [ %13.9f, %13.9f, %13.9f, %13.9f,\n'...
%                      '          %13.9f, %13.9f, %13.9f, %13.9f,\n'...
%                      '          %13.9f, %13.9f, %13.9f, %13.9f,\n'...
%                      '          %13.9f, %13.9f, %13.9f, %13.9f ]\n\n'],...
%                      T_W2L(1, 1), T_W2L(1, 2), T_W2L(1, 3), T_W2L(1, 4),...
%                      T_W2L(2, 1), T_W2L(2, 2), T_W2L(2, 3), T_W2L(2, 4),...
%                      T_W2L(3, 1), T_W2L(3, 2), T_W2L(3, 3), T_W2L(3, 4),...
%                      T_W2L(4, 1), T_W2L(4, 2), T_W2L(4, 3), T_W2L(4, 4));
% fprintf(fileID, T_W2L_str);
% 
% fprintf(fileID, ['T_B_Bleica: !!opencv-matrix\n'...
%                  '  rows: 4\n'...
%                  '  cols: 4\n'...
%                  '  dt: d\n']);
%              
% T_B2Bleica_str = sprintf(['  data: [ %13.9f, %13.9f, %13.9f, %13.9f,\n'...
%                           '          %13.9f, %13.9f, %13.9f, %13.9f,\n'...
%                           '          %13.9f, %13.9f, %13.9f, %13.9f,\n'...
%                           '          %13.9f, %13.9f, %13.9f, %13.9f ]\n\n'],...
%                           1, 0, 0, trans_B2prism(1),...
%                           0, 1, 0, trans_B2prism(2),...
%                           0, 0, 1, trans_B2prism(3),...
%                           0, 0, 0, 1.0);
%                       
% fprintf(fileID, T_B2Bleica_str);
% fclose(fileID);
% 
% end
% 
% % Export the external IMU rotations to a yaml file
% for dummy = 1
%     
% % Inertial frame rotation
% R_Wh_Weimu = rot_rsh2h_opt';
% 
% fileID = fopen([exp_name '/ext_imu_rot.yaml'], 'w');
% fprintf(fileID, ['%%YAML:1.0\n'...
%                  'imu_topic: ' imu_topic '\n'...
%                  'R_W_Weimu: !!opencv-matrix\n'...
%                  '  rows: 3\n'...
%                  '  cols: 3\n'...
%                  '  dt: d\n']);
% R_Wh_Weimu_str...
%     = sprintf(['  data: [ %13.9f, %13.9f, %13.9f,\n'...
%                '          %13.9f, %13.9f, %13.9f,\n'...
%                '          %13.9f, %13.9f, %13.9f ]'],...
%                R_Wh_Weimu(1, 1), R_Wh_Weimu(1, 2), R_Wh_Weimu(1, 3),...
%                R_Wh_Weimu(2, 1), R_Wh_Weimu(2, 2), R_Wh_Weimu(2, 3),...
%                R_Wh_Weimu(3, 1), R_Wh_Weimu(3, 2), R_Wh_Weimu(3, 3));
% fprintf(fileID, R_Wh_Weimu_str);
% 
% % Body rotation
% fprintf(fileID, ['\n'...
%                  'R_B_Beimu: !!opencv-matrix\n'...
%                  '  rows: 3\n'...
%                  '  cols: 3\n'...
%                  '  dt: d\n']);
% rot_B_Beimu_str...
%     = sprintf(['  data: [ %13.9f, %13.9f, %13.9f,\n'...
%                '          %13.9f, %13.9f, %13.9f,\n'...
%                '          %13.9f, %13.9f, %13.9f ]\n'],...
%                rot_B_Beimu(1, 1), rot_B_Beimu(1, 2), rot_B_Beimu(1, 3),...
%                rot_B_Beimu(2, 1), rot_B_Beimu(2, 2), rot_B_Beimu(2, 3),...
%                rot_B_Beimu(3, 1), rot_B_Beimu(3, 2), rot_B_Beimu(3, 3));
%            
% fprintf(fileID, rot_B_Beimu_str);
% fclose(fileID);
% 
% end



%% Calculate the position and rotation errors

% VIRAL estimate
for dummy = 1
    
P_h_err     = P_rsh - P_h;
P_h_rmse    = rms(P_h_err);
P_h_ate     = norm(P_h_rmse);
P_h_err_nrm = sqrt(dot(P_h_err, P_h_err, 2));

Q_h_err       = quatmultiply(quatinv(Q_h), Q_rsh);
YPR_h_err	  = wrapToPi(quat2eul(Q_h_err));
rot_h_ang_err = quat2axang(Q_h_err);
% Wrap this error to -pi to pi;
rot_h_ang_err(:, end) = wrapToPi(rot_h_ang_err(:, end));
% Find the outliers
olrh_idx = find(isoutlier(rot_h_ang_err(:, end), 'mean'));
% Extract the inlier errors
rot_h_ang_err_nolr = rot_h_ang_err(:, end);
t_h_nolr           = t_h;
rot_h_ang_err_nolr(olrh_idx, :) = [];
t_h_nolr(olrh_idx)              = [];
% Calculate the error
rot_h_rmse         = rms(rot_h_ang_err_nolr(:, end))/pi*180;
% rot_h_ang_err_norm = abs(rot_h_ang_err(:, end));

end

% VIRAL BA estimate
for dummy = 1
    
P_ba_err     = P_rsba - P_ba;
P_ba_rmse    = rms(P_ba_err);
P_ba_ate     = norm(P_ba_rmse);
P_ba_err_nrm = sqrt(dot(P_ba_err, P_ba_err, 2));

Q_ba_err       = quatmultiply(quatinv(Q_ba), Q_rsba);
YPR_ba_err	   = wrapToPi(quat2eul(Q_ba_err));
rot_ba_ang_err = quat2axang(Q_ba_err);
% Wrap this error to -pi to pi;
rot_ba_ang_err(:, end) = wrapToPi(rot_ba_ang_err(:, end));
% Find the outliers
olrba_idx = find(isoutlier(rot_ba_ang_err(:, end), 'mean'));
% Extract the inlier errors
rot_ba_ang_err_nolr = rot_ba_ang_err(:, end);
t_ba_nolr           = t_ba;
rot_ba_ang_err_nolr(olrba_idx, :) = [];
t_ba_nolr(olrba_idx)              = [];
% Calculate the error
rot_ba_rmse         = rms(rot_ba_ang_err_nolr(:, end))/pi*180;

end



%% Save the important variables for later analyses
save([exp_path exp_name '_poses.mat'],...
     't_h',      'P_rsh',   'Q_rsh',...
     'P_h',      'Q_h',     'rot_align_h', 'trans_align_h',...
     't_pos',    'P',...
	 't_h_org',  'P_h_org',...
     't_ba_org', 'P_ba_org');
 
% Save the errors calculated 
save([exp_path exp_name '_rmse.mat'],...
     'P_h_ate',  'rot_h_rmse',...
     'P_ba_ate', 'rot_ba_rmse');
 

%% Print the result
% loops = size(P_lp_curr, 1);
fprintf(['test: %2d. %s.\n'...
         'Error:\tPos [m]\t Rot [deg]\n'...
         'VINS:\t%6.4f\t %7.4f.\n'...
         'VINSBA:\t%6.4f\t %7.4f. Loops: %d\n\n'],...
          test_id, exp_name(8:end),...
          P_h_ate, rot_h_rmse,... 
          P_ba_ate, rot_ba_rmse, 0);
      
P_ate    = [P_h_ate, P_ba_ate];
rot_rmse = [rot_h_rmse, rot_ba_rmse];



%% Calculate the maximum time
t_max = max([t_pos; t_h]);



%% Plot the 3D trajectory
figpos = [1920 0 0 0] + [0, 480, 630, 400];
figure('position', figpos, 'color', 'w', 'paperpositionmode', 'auto');
fighd = [fighd gcf];
hold on;

plot3(P(:, 1), P(:, 2), P(:, 3),...
      '.r', 'markersize', 5);
plot3(P_h_full(:, 1),  P_h_full(:, 2),  P_h_full(:, 3),...
      'b', 'linewidth', 2);
plot3(P_ba_full(:, 1), P_ba_full(:, 2), P_ba_full(:, 3), 'o',...
      'markerfacecolor', mygreen, 'markeredgecolor', 'k',...
      'markersize', 5);

% if loop_present
%     if size(P_lp_curr, 1) ~= 0
%         for n = 1:loops
%            line([P_lp_curr(n, 1), P_lp_prev(n, 1)],...
%                 [P_lp_curr(n, 2), P_lp_prev(n, 2)],...
%                 [P_lp_curr(n, 3), P_lp_prev(n, 3)],...
%                 'color', mycyan, 'linewidth', 2); 
%         end
%     else
%         fprintf('Loop index not recorded properly!\n');
%     end
% end

xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
grid on;
daspect([1 1 1]);
view([-21 15]);
set(gca, 'fontsize', 13);
tightfig;
lg_hd = legend('Groundtruth', 'VIRAL', 'VIRAL-BA');
set(lg_hd, 'numcolumns', 3, 'position', [0.13, 0.9175, 0.58, 0.06]);

tightfig;

saveas(gcf, [exp_path exp_name '_traj.fig']);
% saveas(gcf, [exp_path exp_name '_traj.pdf']);
img = getframe(gcf);
imwrite(img.cdata, [exp_path exp_name '_traj.png']);



%% Plot the time evolution of position

figpos = [1920 0 0 0] + [0, 0, 630, 400];
figure('position', figpos, 'color', 'w');
fighd = [fighd gcf];

subplot(3, 1, 1);
hold on;
plot(t_pos, P(:, 1),   'color', 'r', 'linewidth', 3);
plot(t_h,   P_h(:, 1), 'color', 'b', 'linewidth', 2);

ylabel('X [m]');
grid on;
set(gca, 'fontsize', 13);
xlim([0 t_max]);

subplot(3, 1, 2);
hold on;
plot(t_pos, P(:, 2),   'color', 'r', 'linewidth', 3);
plot(t_h,   P_h(:, 2), 'color', 'b', 'linewidth', 2);


ylabel('Y [m]');
grid on;
set(gca, 'fontsize', 13);
xlim([0 t_max]);

subplot(3, 1, 3);
hold on;
plot(t_pos, P(:, 3),    'color', 'r', 'linewidth', 3);
plot(t_h,   P_h(:, 3),  'color', 'b', 'linewidth', 2);

xlabel('Time [s]');
ylabel('X [m]');
grid on;
set(gca, 'fontsize', 13);
xlim([0 t_max]);


lg_hd = legend('Groundtruth', 'VIRAL');
set(lg_hd, 'numcolumns', 2, 'position', [0.6 0.69 0.38 0.08]);

tightfig(gcf);

saveas(gcf, [exp_path exp_name '_xyzt.fig']);
% saveas(gcf, [exp_path exp_name '_xyzt.pdf']);
img = getframe(gcf);
imwrite(img.cdata, [exp_path exp_name '_xyzt.png']);



%% Plot the time evolution of orientation

% Calculate the yaw pitch rol relative to the initial position
YPR       = wrapToPi(quat2eul(quatmultiply(quatinv(Q(1, :)), Q)));
YPR_h     = wrapToPi(quat2eul(quatmultiply(quatinv(Q(1, :)), Q_h)));


figpos = [1920 0 0 0] + [630, 480, 630, 400];
figure('position', figpos, 'color', 'w');
fighd = [fighd gcf];

subplot(3, 1, 1);
hold on;
plot(t_ori,   YPR(:, 1)*180/pi,   'r', 'linewidth', 3);
plot(t_h,     YPR_h(:, 1)*180/pi, 'b', 'linewidth', 2);
ylabel('Yaw [deg]');
grid on;
set(gca, 'fontsize', 13);
xlim([0 t_max]);

subplot(3, 1, 2);
hold on;
plot(t_ori,   YPR(:, 2)*180/pi,   'r', 'linewidth', 3);
plot(t_h,     YPR_h(:, 2)*180/pi, 'b', 'linewidth', 2);
ylabel('Pitch [deg]');
grid on;
set(gca, 'fontsize', 13);
xlim([0 t_max]);

subplot(3, 1, 3);
hold on;
plot(t_ori,   YPR(:, 3)*180/pi,   'r', 'linewidth', 3);
plot(t_h,     YPR_h(:, 3)*180/pi, 'b', 'linewidth', 2);
xlabel('Time [s]');
ylabel('Roll [deg]');
grid on;
set(gca, 'fontsize', 13);
xlim([0 t_max]);

lg_hd = legend('Groundtruth', 'VIRAL');
set(lg_hd, 'numcolumns', 2, 'position', [0.15 0.31 0.39 0.06]);

tightfig(gcf);

saveas(gcf, [exp_path exp_name '_yprt.fig']);
img = getframe(gcf);
imwrite(img.cdata, [exp_path exp_name '_yprt.png']);



%% Plot the time evolution of velocity estimate
figpos = [1920 0 0 0] + [630, 0, 630, 400];
figure('position', figpos, 'color', 'w');
fighd = [fighd gcf];

hold on;
plot(t_h, V_h(:, 1), 'r', 'linewidth', 2);
plot(t_h, V_h(:, 2), 'g', 'linewidth', 2);
plot(t_h, V_h(:, 3), 'b', 'linewidth', 2);
plot(t_h, sqrt(dot(V_h, V_h, 2)), 'g', 'linewidth', 2);
% xlabel('Time [s]');
ylabel('Vel. Est. [m/s]');
grid on;
set(gca, 'fontsize', 13);
xlim([0 t_max]);

lg_hd = legend('Vx', 'Vy', 'Vz', 'norm (speed)');

set(lg_hd, 'numcolumns', 4,...
           'position', [0.2317 0.8767 0.6000 0.1000]);

tightfig(gcf);

saveas(gcf, [exp_path exp_name '_vxvyvz_t.fig']);
% saveas(gcf, [exp_path exp_name '_vxvyvz_t.pdf']);
img = getframe(gcf);
imwrite(img.cdata, [exp_path exp_name '_vxvyvz_t.png']);



%% Plot the time evolution of position error
figpos = [1920 0 0 0] + [630*2, 480, 630, 400];
figure('position', figpos, 'color', 'w');
fighd = [fighd gcf];

subplot(3, 1, 1);
hold on;
plot(t_h,     P_h_err(:, 1),     'color', 'b',       'linewidth', 2);
ylabel('X Err. [m]');
grid on;
set(gca, 'fontsize', 13);
xlim([0 t_max]);

subplot(3, 1, 2);
hold on;
plot(t_h,     P_h_err(:, 2),     'color', 'b',       'linewidth', 2);
ylabel('Y Err [m]');
grid on;
set(gca, 'fontsize', 13);
xlim([0 t_max]);

subplot(3, 1, 3);
hold on;
plot(t_h,     P_h_err(:, 3),     'color', 'b',       'linewidth', 2);
ylabel('Z Err [m]');
xlabel('Time [s]');
grid on;
set(gca, 'fontsize', 13);
xlim([0 t_max]);


lg_hd = legend('VIRAL');
set(lg_hd, 'numcolumns', 4, 'position', [0.73 0.6875 0.20 0.06]);


tightfig(gcf);
saveas(gcf, [exp_path exp_name '_xyz_err_t.fig']);
% saveas(gcf, [exp_path exp_name '_xyz_err_t.pdf']);
img = getframe(gcf);
imwrite(img.cdata, [exp_path exp_name '_xyz_err_t.png']);



%% Plot the time evolution of orientation err
figpos = [1920 0 0 0] + [630*2, 0, 630, 400];
figure('position', figpos, 'color', 'w');
fighd = [fighd gcf];

subplot(3, 1, 1);
hold on;
plot(t_h,  YPR_h_err(:, 1)*180/pi,  'b', 'linewidth', 2);
ylabel('$\tilde{\psi}$ [deg]', 'interpreter', 'latex');
grid on;
set(gca, 'fontsize', 13);
xlim([0 t_max]);

subplot(3, 1, 2);
hold on;
plot(t_h, YPR_h_err(:, 2)*180/pi,    'b', 'linewidth', 2);
ylabel('$\tilde{\theta}$ [deg]', 'interpreter', 'latex');
grid on;
set(gca, 'fontsize', 13);
xlim([0 t_max]);

subplot(3, 1, 3);
hold on;
plot(t_h,  YPR_h_err(:, 3)*180/pi,    'b', 'linewidth', 2);
xlabel('Time [s]');
ylabel('$\tilde{\phi}$ [deg]', 'interpreter', 'latex');
grid on;
set(gca, 'fontsize', 13);
xlim([0 t_max]);

lg_hd = legend('VIRAL');
set(lg_hd, 'position', [0.12 0.38 0.21 0.06]);

tightfig(gcf);
saveas(gcf, [exp_path exp_name '_ypr_err_t.fig']);
% saveas(gcf, [exp_path exp_name '_ypr_err_t.pdf']);
img = getframe(gcf);
imwrite(img.cdata, [exp_path exp_name '_ypr_err_t.png']);



%% Plot the combined time evolution of position estimation error
figpos = [1920 0 0 0] + [930, 750, 630, 200];
figure('position', figpos, 'color', 'w');
fighd = [fighd gcf];

hold on;
plot(t_h, P_h_err(:, 1), 'r', 'linewidth', 2);
plot(t_h, P_h_err(:, 2), 'g', 'linewidth', 2);
plot(t_h, P_h_err(:, 3), 'b', 'linewidth', 2);
xlabel('Time [s]');
ylabel('Error [m]');
grid on;
set(gca, 'fontsize', 13);
xlim([0 t_max]);

lg_hd = legend('Px error', 'Py error', 'Pz error');

set(lg_hd, 'orientation', 'horizontal',...
    'position', [0.3 0.85 0.4 0.1000]);

tightfig(gcf);
saveas(gcf, [exp_path exp_name '_xyz_h_err_t.fig']);
img = getframe(gcf);
imwrite(img.cdata, [exp_path exp_name '_xyz_h_err_t.png']);



%% Plot the combined time evolution of orientation estimation error
figpos = [1920 0 0 0] + [300, 750, 630, 200];
figure('position', figpos, 'color', 'w');
fighd = [fighd gcf];

hold on;
plot(t_h, YPR_h_err(:, 1)/pi*180, 'r', 'linewidth', 2);
plot(t_h, YPR_h_err(:, 2)/pi*180, 'g', 'linewidth', 2);
plot(t_h, YPR_h_err(:, 3)/pi*180, 'b', 'linewidth', 2);
xlabel('Time [s]');
ylabel('Error [deg]');
ylim([-8 8]);
grid on;
set(gca, 'fontsize', 13);
xlim([0 t_max]);

lg_hd = legend('Yaw error', 'Pitch error', 'Roll error');

set(lg_hd, 'orientation', 'horizontal',...
           'position', [0.3 0.85 0.4 0.1000]);

tightfig(gcf);
saveas(gcf, [exp_path exp_name '_ypr_h_err_t.fig']);
% saveas(gcf, [exp_path exp_name '_ypr_h_err_t.pdf']);
img = getframe(gcf);
imwrite(img.cdata, [exp_path exp_name '_ypr_h_err_t.png']);


end