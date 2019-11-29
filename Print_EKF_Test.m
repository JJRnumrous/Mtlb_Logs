%% Extract desired messages from log to CSV files
messages = ["vehicle_gps_position", "vehicle_global_position_groundtruth", "vehicle_visual_odometry", "vehicle_attitude", "vehicle_local_position", "vehicle_attitude_groundtruth"];
output_d = ["gps"; "vision"];
delimeter = ",";
add = [0.00 ; 7.63];
time_cuttoff = 45;
% #  [0; 1];
% #2 [1; 0];
% #3 [0; 0];
% #4 [0; 0];
% #5 [8 ; 0.18];

log_name = ["Logs/EKF/ekf_gps2.ulg"; "Logs/EKF/ekf_vis2.ulg"];

for K = 1:1:size(output_d)  
    
    output_dir = output_d(K);
    ulog_file = strcat(output_dir, ".ulg");
    cmd_copy = strcat("cp ", log_name(K), " ", output_dir, "/", ulog_file);
    system(cmd_copy);

    messages_str = messages(1);
    for i = 2:6
       messages_str = strcat(messages_str, ",", messages(i)); 
    end

    csv_command = strcat("ulog2csv -m ", messages_str, " -o ", output_dir, " -d ", delimeter, " ", output_dir, "/", ulog_file);
    system(csv_command);

    %% Read
    gps_pos_msg_name = messages(1);
    gps_pos_csv = csvread(strcat(output_dir+"/"+output_dir, "_", gps_pos_msg_name, "_0.csv"), 1, 0);

    local_pos_grndtruth_msg_name = messages(2);
    local_pos_grndtruth_csv = csvread(strcat(output_dir+"/"+output_dir, "_", local_pos_grndtruth_msg_name, "_0.csv"), 1, 0);

    vis_odom_msg_name = messages(3);
    vis_odom_csv = csvread(strcat(output_dir+"/"+output_dir, "_", vis_odom_msg_name, "_0.csv"), 1, 0);

    att_msg_name = messages(4);
    att_csv = csvread(strcat(output_dir+"/"+output_dir, "_", att_msg_name, "_0.csv"), 1, 0);

    pos_msg_name = messages(5);
    pos_csv = csvread(strcat(output_dir+"/"+output_dir, "_", pos_msg_name, "_0.csv"), 1, 0);

    att_grndtruth_msg_name = messages(6);
    att_grndtruth_csv = csvread(strcat(output_dir+"/"+output_dir, "_", att_grndtruth_msg_name, "_0.csv"), 1, 0);

    %% Get fields of interest
    % GPS
    gps_pos_time    = gps_pos_csv(:,1) ./ 1e6;
    gps_pos_lat     = gps_pos_csv(:,4)*pi*1e-7/180;
    gps_pos_lon     = gps_pos_csv(:,3)*pi*1e-7/180;
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
        end
        x = k* (cos(home_lat)*sin(gps_pos_lat(i)) - sin(home_lat)*cos(gps_pos_lat(i))*cosD ) *6371000;
        y = k*cos(gps_pos_lat(i))*sin(gps_pos_lon(i)-home_lon)*6371000;
        gps_pos_lat(i) = x;
        gps_pos_lon(i) = y;
        gps_pos_alt(i) = -(gps_pos_alt(i) - home_alt)/1000;
    end

    local_pos_grndtruth_time    = local_pos_grndtruth_csv(:,1) ./ 1e6;
    local_pos_grndtruth_x       = local_pos_grndtruth_csv(:,3)*pi/180;
    local_pos_grndtruth_y       = local_pos_grndtruth_csv(:,2)*pi/180;
    local_pos_grndtruth_z       = local_pos_grndtruth_csv(:,4);
    local_pos_grndtruth_vx      = local_pos_grndtruth_csv(:,7);
    local_pos_grndtruth_vy      = local_pos_grndtruth_csv(:,8);
    local_pos_grndtruth_vz      = local_pos_grndtruth_csv(:,9);
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
    vis_odom_x          = vis_odom_csv(:,3);
    vis_odom_y          = vis_odom_csv(:,2);
    vis_odom_z          = vis_odom_csv(:,4);
    vis_odom_q_w        = vis_odom_csv(:,5);
    vis_odom_q_x        = vis_odom_csv(:,6);
    vis_odom_q_y        = vis_odom_csv(:,7);
    vis_odom_q_z        = vis_odom_csv(:,8);

    vis_eul = quat2eul([vis_odom_q_w,vis_odom_q_x,vis_odom_q_y,vis_odom_q_z], 'XYZ');

    att_time            = att_csv(:,1) ./ 1e6;
    att_q_w             = att_csv(:,5);
    att_q_x             = -att_csv(:,6);
    att_q_y             = -att_csv(:,7);
    att_q_z             = att_csv(:,8);
    att_rollspeed       = att_csv(:,2)*180/pi;
    att_pitchspeed      = att_csv(:,3)*180/pi;
    att_yawspeed        = att_csv(:,4)*180/pi;
    att_eul = quat2eul([att_q_w,att_q_x,att_q_y,att_q_z], 'XYZ');

    pos_time = pos_csv(:,1) ./ 1e6;
    pos_x   = pos_csv(:,6);
    pos_y   = pos_csv(:,5);
    pos_z   = pos_csv(:,7);
    pos_vx  = pos_csv(:,11);
    pos_vy  = pos_csv(:,12);
    pos_vz  = pos_csv(:,13);

    

    grnd_att_time           = att_grndtruth_csv(:,1)./1e6;
    grnd_att_q_w            = att_grndtruth_csv(:,5);
    grnd_att_q_x            = -att_grndtruth_csv(:,6);
    grnd_att_q_y            = -att_grndtruth_csv(:,7);
    grnd_att_q_z            = att_grndtruth_csv(:,8);
    grnd_att_rollspeed      = att_grndtruth_csv(:,2)*180/pi;
    grnd_att_pitchspeed     = att_grndtruth_csv(:,3)*180/pi;
    grnd_att_yawspeed       = att_grndtruth_csv(:,4)*180/pi;
    grnd_eul                = quat2eul([grnd_att_q_w, grnd_att_q_x, grnd_att_q_y, grnd_att_q_z], 'XYZ');
   

    local_pos_grndtruth_time(find(local_pos_grndtruth_time>time_cuttoff-add(k))) = NaN;
    att_time(find(att_time>time_cuttoff-add(k))) = NaN;
    pos_time(find(pos_time>time_cuttoff-add(k))) = NaN;
    grnd_att_time(find(grnd_att_time>time_cuttoff-add(k))) = NaN;
    
    choice_ekf = [pos_x, pos_y, pos_z];
    choice_gps = [gps_pos_lat, gps_pos_lon, gps_pos_alt];
    choice_vio = [vis_odom_x, vis_odom_y, vis_odom_z];
    choice_gnd = [local_pos_grndtruth_x, local_pos_grndtruth_y, local_pos_grndtruth_z];

    choice_vis_att = [vis_odom_q_w, vis_odom_q_x, vis_odom_q_y, vis_odom_q_z];
    choice_att = [att_q_w, att_q_x, att_q_y, att_q_z];
    choice_gnd_att = [grnd_att_q_w, grnd_att_q_x, grnd_att_q_y, grnd_att_q_z];    

    
    figure(1);
    hold on;
    subplot(3,1,1)
    plot(pos_time+add(K), choice_ekf(:,1),local_pos_grndtruth_time+add(K), choice_gnd(:,1));
    title('N-position');
    xlabel('Time [ms]');
    ylabel('Position [m]');
    legend("GPS-EKF", "GPS-GND","VIS-EKF", "VIS-GND"); 
    
    hold on;    
    subplot(3,1,2)
    plot(pos_time+add(K), choice_ekf(:,2), local_pos_grndtruth_time+add(K), choice_gnd(:,2));
    title('E-position');
    xlabel('Time [ms]');
    ylabel('Position [m]');
    legend("GPS-EKF", "GPS-GND","VIS-EKF", "VIS-GND"); 

    hold on;    
    subplot(3,1,3)
    plot(pos_time+add(K), choice_ekf(:,3),local_pos_grndtruth_time+add(K), choice_gnd(:,3));
    title('D-position');
    xlabel('Time [ms]');
    ylabel('Position [m]');
    legend("GPS-EKF", "GPS-GND","VIS-EKF", "VIS-GND"); 

    figure(2);
    hold on;
    subplot(3,1,1)
    plot(pos_time+add(K), pos_vx,local_pos_grndtruth_time+add(K), local_pos_grndtruth_vx);
    title('N-Speed');
    xlabel('Time [ms]');
    ylabel('Speed [m/s]');
    legend("GPS-EKF", "GPS-GND","VIS-EKF", "VIS-GND"); 
    
    hold on;    
    subplot(3,1,2)
    plot(pos_time+add(K), pos_vy,local_pos_grndtruth_time+add(K), local_pos_grndtruth_vy);
    title('E-Speed');
    xlabel('Time [ms]');
    ylabel('Speed [m/s]');
    legend("GPS-EKF", "GPS-GND","VIS-EKF", "VIS-GND"); 

    hold on;    
    subplot(3,1,3)
    plot(pos_time+add(K), pos_vz,local_pos_grndtruth_time+add(K), local_pos_grndtruth_vz);
    title('D-Speed');
    xlabel('Time [ms]');
    ylabel('Speed [m/s]');
    legend("GPS-EKF", "GPS-GND","VIS-EKF", "VIS-GND"); 
    
    
    figure(3);
    hold on;
    subplot(3,1,1)
    plot(att_time, att_rollspeed, grnd_att_time, grnd_att_rollspeed);
    title('Rollspeed');
    xlabel('Time [ms]');
    ylabel('[deg/s]');
    legend("GPS-EKF", "GPS-GND","VIS-EKF", "VIS-GND");

    
    hold on;
    subplot(3,1,2)
    plot(att_time, att_pitchspeed, grnd_att_time, grnd_att_pitchspeed);
    title('Pitch Speed');
    xlabel('Time [ms]');
    ylabel('deg/s');
    legend("GPS-EKF", "GPS-GND","VIS-EKF", "VIS-GND");

    hold on;
    subplot(3,1,3)
    plot(att_time, att_yawspeed, grnd_att_time, grnd_att_yawspeed);
    title('Yaw Speed');
    xlabel('Time [ms]');
    ylabel('deg/s');
    legend("GPS-EKF", "GPS-GND","VIS-EKF", "VIS-GND");
    
    
    figure(4);
    hold on;
    subplot(3,1,1)
    plot(att_time, att_eul(:,2)*180/pi, grnd_att_time, grnd_eul(:,2)*180/pi);
    title('Roll');
    xlabel('Time [ms]');
    ylabel('Angle [deg]');
    legend("GPS-EKF", "GPS-GND", "Vis-EKF", "Vis-GND"); 

    hold on;
    subplot(3,1,2)
    plot(att_time, att_eul(:,1)*180/pi, grnd_att_time, grnd_eul(:,1)*180/pi);
    title('Pitch');
    xlabel('Time [ms]');
    ylabel('Angle [deg]');
    legend("GPS-EKF", "GPS-GND", "Vis-EKF", "Vis-GND"); 

    hold on;
    subplot(3,1,3)
    plot(att_time, att_eul(:,3)*180/pi -90, grnd_att_time, grnd_eul(:,3)*180/pi -90);
    title('Yaw');
    xlabel('Time [ms]');
    ylabel('Angle [deg]');
    legend("GPS-EKF", "GPS-GND", "Vis-EKF", "Vis-GND"); 
    
end
hold off;