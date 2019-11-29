%% Extract desired messages from log to CSV files
messages = ["vehicle_gps_position", "vehicle_global_position_groundtruth", "vehicle_visual_odometry", "vehicle_attitude", "vehicle_local_position", "vehicle_attitude_groundtruth"];
output_dir = "px4_logs_csv";
delimeter = ",";
ulog_filename = "px4_log";
ulog_file = strcat(ulog_filename, ".ulg");

if latest == 1
   px4_latest_log;
else
   cmd_copy = strcat("cp ", log_name, " ", output_dir, "/", ulog_file);
   system(cmd_copy);
end

messages_str = messages(1);
for i = 2:6
   messages_str = strcat(messages_str, ",", messages(i)); 
end
csv_command = strcat("ulog2csv -m ", messages_str, " -o ", output_dir, " -d ", delimeter, " ", output_dir, "/", ulog_file);
system(csv_command);

%% Read
gps_pos_msg_name = messages(1);
gps_pos_csv = csvread(strcat(output_dir, "/", ulog_filename, "_", gps_pos_msg_name, "_0.csv"), 1, 0);

local_pos_grndtruth_msg_name = messages(2);
local_pos_grndtruth_csv = csvread(strcat(output_dir, "/", ulog_filename, "_", local_pos_grndtruth_msg_name, "_0.csv"), 1, 0);

vis_odom_msg_name = messages(3);
vis_odom_csv = csvread(strcat(output_dir, "/", ulog_filename, "_", vis_odom_msg_name, "_0.csv"), 1, 0);

att_msg_name = messages(4);
att_csv = csvread(strcat(output_dir, "/", ulog_filename, "_", att_msg_name, "_0.csv"), 1, 0);

pos_msg_name = messages(5);
pos_csv = csvread(strcat(output_dir, "/", ulog_filename, "_", pos_msg_name, "_0.csv"), 1, 0);

att_grndtruth_msg_name = messages(6);
att_grndtruth_csv = csvread(strcat(output_dir, "/", ulog_filename, "_", att_grndtruth_msg_name, "_0.csv"), 1, 0);

%% Get fields of interest
% GPS
gps_pos_time    = gps_pos_csv(:,1) ./ 1e6;
gps_pos_lat     = gps_pos_csv(:,3)*pi*1e-7/180;
gps_pos_lon     = gps_pos_csv(:,4)*pi*1e-7/180;
gps_pos_alt     = gps_pos_csv(:,5);
home_lat = gps_pos_lat(1);
home_lon = gps_pos_lon(1);
home_alt = gps_pos_alt(1);

for i = 1:1:size(gps_pos_time)
    cosD = cos(gps_pos_lon(i) - home_lon);
    arg  = sin(home_lat)*sin(gps_pos_lat(i)) + cos(home_lat)*cos(gps_pos_lat(i))*cosD;
    c = acos(arg);    
    if(abs(c)>0)
        k = c/sin(c);
    else
        k =1;
    end
    
    x = k* (cos(home_lat)*sin(gps_pos_lat(i)) - sin(home_lat)*cos(gps_pos_lat(i))*cosD ) *6371000;
    y = k*cos(gps_pos_lat(i))*sin(gps_pos_lon(i)-home_lon)*6371000;
    gps_pos_lat(i) = x;
    gps_pos_lon(i) = y+0.6059;
    gps_pos_alt(i) = -(gps_pos_alt(i) - home_alt)/1000 -2.4;
end
    
local_pos_grndtruth_time    = local_pos_grndtruth_csv(:,1) ./ 1e6;
local_pos_grndtruth_x       = local_pos_grndtruth_csv(:,2)*pi/180;
local_pos_grndtruth_y       = local_pos_grndtruth_csv(:,3)*pi/180;
local_pos_grndtruth_z       = local_pos_grndtruth_csv(:,4);
home_lat = local_pos_grndtruth_x(1);
home_lon = local_pos_grndtruth_y(1);
home_alt = local_pos_grndtruth_z(1);

for i = 1:1:size(local_pos_grndtruth_time)
    cosD = cos(local_pos_grndtruth_y(i) - home_lon);
    arg  = sin(home_lat)*sin(local_pos_grndtruth_x(i)) + cos(home_lat)*cos(local_pos_grndtruth_x(i))*cosD;
    c = acos(arg);
    if(abs(c)>0)
        k = c/sin(c);
    end
    x = k* (cos(home_lat)*sin(local_pos_grndtruth_x(i)) - sin(home_lat)*cos(local_pos_grndtruth_x(i))*cosD ) *6371000;
    y = k*cos(local_pos_grndtruth_x(i))*sin(local_pos_grndtruth_y(i)-home_lon)*6371000;
    local_pos_grndtruth_x(i) = x;
    local_pos_grndtruth_y(i) = y;
    
    local_pos_grndtruth_z(i) = -local_pos_grndtruth_z(i) + home_alt;
end


%% TODO!!!
vis_odom_time       = vis_odom_csv(:,1) ./ 1e6;
vis_odom_x          = vis_odom_csv(:,2);
vis_odom_y          = vis_odom_csv(:,3);
vis_odom_z          = vis_odom_csv(:,4);

% this is in ENU axis system
vis_odom_q_w        = vis_odom_csv(:,5);
vis_odom_q_x        = vis_odom_csv(:,6);
vis_odom_q_y        = vis_odom_csv(:,7);%-0.045;
vis_odom_q_z        = vis_odom_csv(:,8);

vis_odom_temp = find(vis_odom_q_w < 0);
vis_odom_q_w(vis_odom_temp) = -vis_odom_q_w(vis_odom_temp);
vis_odom_q_x(vis_odom_temp) = -vis_odom_q_x(vis_odom_temp);
vis_odom_q_y(vis_odom_temp) = -vis_odom_q_y(vis_odom_temp);
vis_odom_q_z(vis_odom_temp) = -vis_odom_q_z(vis_odom_temp);

vis_eul = quat2eul([vis_odom_q_w,vis_odom_q_x,vis_odom_q_y,vis_odom_q_z], 'XYZ') - vis_biases*pi/180;


% 
% vis_odom_x(find(abs(vis_odom_x)>max)) =  max;
% vis_odom_y(find(abs(vis_odom_y)>max)) =  max;
% vis_odom_z(find(abs(vis_odom_z)>max)) =  -max;


att_time            = att_csv(:,1) ./ 1e6;
att_q_w             = att_csv(:,5);
att_q_x             = att_csv(:,6);%+0.012;
att_q_y             = att_csv(:,7);
att_q_z             = att_csv(:,8);

if (latest == 0) && (log_choice == 21)
    ekf_odom_temp = find(att_q_w < 0);
    att_q_w(ekf_odom_temp) = -att_q_w(ekf_odom_temp);
end

att_eul = quat2eul([att_q_w,att_q_x,att_q_y,att_q_z], 'XYZ');

pos_time = pos_csv(:,1) ./ 1e6;
pos_x = pos_csv(:,5);
pos_y = pos_csv(:,6);
pos_z = pos_csv(:,7);

vis_odom_time(find(vis_odom_time>[1,0]*pos_time(size(pos_time)) )) =  [1,0]*pos_time(size(pos_time));

grnd_att_time = att_grndtruth_csv(:,1)./1e6;
grnd_att_q_w = att_grndtruth_csv(:,5);
grnd_att_q_x = att_grndtruth_csv(:,6);
grnd_att_q_y = att_grndtruth_csv(:,7);
grnd_att_q_z = att_grndtruth_csv(:,8)-0.01;
grnd_eul = quat2eul([grnd_att_q_w, grnd_att_q_x, grnd_att_q_y, grnd_att_q_z], 'XYZ');
