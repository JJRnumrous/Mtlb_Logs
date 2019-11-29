clear;
GPS         = 0;
GND         = 0;
VIS         = 1;
EKF         = 1;
latest      = 0;         % ( 0/ 1)      logs / latest logs 
down_scaler = -1;    % (+1/-1)      for when using down and vz for inverting graph
graphs = 1;

% 19 - positions;
% 21 - orientations; 
% 26 - multi-marker positions
% 29 - multi-marker orientations
% 30 - multi-marker orientations better
% 300 - dry run/ no GPS
% 400 - first flight test (5-11-2019)
% 500 - gps own cntrl flight (11-11-2019
% 501 - gps px4 cntrl flight (11-11-2019
% 502 - vio own cntrl flight (11-11-2019
% 504 - vio px4 cntrl flight (11-11-2019
log_choice = 400; % 19 21 26 29 30

% __________________ BIAS __________________________
cam_offset = -[ 0.12, 0.038, +0.01];
att_bias = [ 0, 0, 0];
bias = [ 0.0, 0.0, 0.0];
% gps_bias = [0.0, 0, 0.0];
% vis_biases = [0, 0, 0];

% 502
gps_bias = [0, 0, 0];
vis_biases = [0, 2.1, 0];
timer = 150;
max_time = 200;

% 504
% gps_bias = [-0.5, -0.4, 0.8];
% vis_biases = [0, 5, 0];
% timer = 200;
% max_time = 320;
% PosYLtds = [-1.6, -0; -0.6, 1.5;-inf, inf];
% EulLtds  = [-15, 15; -15, 15;-5, 20];

% 502
% gps_bias = [-0.6, -0.5, 6];
% vis_biases = [-0.5, 2.1, 0];
% timer = 530;
% max_time = 655;

% 501
% gps_bias = [0, -0.5, 3.5];
% vis_biases = [0, 1, 0];
% timer = 220;
% max_time = 380;

% 500
% gps_bias = [0.5, -4.5, -2.5];
% vis_biases = [0, 1, 0];
% timer = 320;
% max_time = 450;

% 400
% gps_bias = [0.1, 0, 0.6];
% vis_biases = [0, 1, 0];
% timer = 150;
% max_time = 15000;

% __________________ PLot Detail ___________________
% timer = 0; %150 530
% max_time = 1650; %15000 650
lineWidth = 1.5;
colour_GPS         = [0,1,0];
colour_GND         = [0.843,0.6392,0.1059];%[0.5,0.5,0.5];%
colour_VIS         = [1,0,0];
colour_EKF         = [0,0,1];

%POs
% PosYLtds = [-0.22, 0.23; -0.22, 0.22; -0.17, 0.17];
% EulLtds = [-1, 1; -2, 4; 84, 95];

% Eul
% PosYLtds = [-0.01, 0.15; -0.1, 0.1; -0.1, 0.1];
% EulLtds = [-11, 11; -12, 12; 73, 110];

%other
PosYLtds = [-inf, inf; -inf, inf;-inf, inf];
EulLtds  = [-inf, inf; -inf, inf;-inf, inf];

file = "Drone";
log_name = "Logs/"+file+"/"+log_choice + "_step.ulg";
max = 1;



read_px4_log;    
i = 1:1:3; 
j = 3;

choice_gnd = [local_pos_grndtruth_x, local_pos_grndtruth_y, local_pos_grndtruth_z];
choice_vis_att = [vis_odom_q_w, vis_odom_q_x, vis_odom_q_y, vis_odom_q_z];
choice_att = [att_q_w, att_q_x, att_q_y, att_q_z];
choice_gnd_att = [grnd_att_q_w, grnd_att_q_x, grnd_att_q_y, grnd_att_q_z];

if(latest ==0)
    if(log_choice == 19)
        bias = [ 0.012, 0.028, -0.015]; 
        choice_gnd = [   0, 0.0,   0;
                         0, 0.0,   0;
                      -0.2, 0.2,   0;
                      -0.2, 0.2,   0;
                      -0.1, 0.2,   0;
                      -0.1, 0.2,   0;
                         0, 0.2,   0;
                         0, 0.2,   0;
                       0.1, 0.2,   0;
                       0.1, 0.2,   0;
                       0.2, 0.2,   0;                      
                       0.2, 0.2,   0;                      
                       0.2, 0.0,   0;
                       0.2, 0.0,   0;
                       0.1, 0.0,   0;
                       0.1, 0.0,   0;
                         0, 0.0,   0;
                         0, 0.0,   0;
                      -0.1, 0.0,   0;
                      -0.1, 0.0,   0;
                      -0.2, 0.0,   0;
                      -0.2, 0.0,   0;                    
                      -0.2,-0.2,   0;
                      -0.2,-0.2,   0;
                      -0.1,-0.2,   0;
                      -0.1,-0.2,   0;
                         0,-0.2,   0;
                         0,-0.2,   0;
                       0.1,-0.2,   0;
                       0.1,-0.2,   0;
                       0.2,-0.2,   0;
                       0.2,-0.2,   0;
                         0,   0,   0;
                         0,   0,   0;
                         0,   0,-0.15;
                         0,   0,-0.15;
                         0,   0, 0.08;
                         0,   0, 0.08   ];
        
        grnd_eul = zeros(size(choice_gnd)) + [0,0,pi/2];
        choice_gnd_att     = eul2quat(grnd_eul, 'XYZ');
        
        
        choice_gnd = [ choice_gnd(:,2), choice_gnd(:,1), -choice_gnd(:,3)];           
        local_pos_grndtruth_time = vis_odom_time(1,1) + [0; 30; 45; 75; 90; 120; 135; 165; 180; 210; 225; 255; 270; 300; 315; 345; 360; 390; 405; 435; 450; 480; 495; 525; 540; 570; 585; 615; 630; 660; 675; 705; 720; 750; 765; 795; 810; 840];
        local_pos_grndtruth_time(1) = timer;
        grnd_att_time = local_pos_grndtruth_time;
    elseif (log_choice == 21)
     grnd_eul = [         0,  0,   90;
                         0,  0,   90;
                         0,  0,   90+5;
                         0,  0,   90+5;
                         0,  0,  90+10;
                         0,  0,  90+10;
                         0,  0,  90+15;
                         0,  0,  90+15;
                         0,  0,   90;
                         0,  0,   90;
                         0,  0,  90-5;
                         0,  0,  90-5;
                         0,  0, 90-10;
                         0,  0, 90-10;
                         0,  0, 90-15;
                         0,  0, 90-15;
                         0,  0,   90;
                         0,  0,   90;
                         0, 10,   90;
                         0, 10,   90;
                         0,  5,   90;
                         0,  5,   90; 
                         0,  0,   90;
                         0,  0,   90;
                         0,-10,   90;
                         0,-10,   90;
                         0, -5,   90;
                         0, -5,   90;                         
                         0,  0,   90;
                         0,  0,   90;
                         5,  0,   90;
                         5,  0,   90;
                        10,  0,   90;
                        10,  0,   90;
                         0,  0,   90;
                         0,  0,   90;
                        -5,  0,   90;
                        -5,  0,   90;
                       -10,  0,   90;
                       -10,  0,   90;
                         0,  0,   90;
                         0,  0,   90];   
        grnd_eul = grnd_eul*pi/180;           
        choice_gnd_att     = eul2quat(grnd_eul, 'XYZ');
        choice_gnd = zeros(size(grnd_eul)) + [0.076,0,0];
          
        grnd_att_time = vis_odom_time(1,1) + [0; 30; 45; 75; 90; 120; 135; 165; 180; 210; 225; 255; 270; 300; 315; 345; 360; 390; 405; 435; 450; 480; 495; 525; 540; 570; 585; 615; 630; 660; 675; 705; 720; 750; 765; 795; 810; 840; 855; 885; 900; 930];
        grnd_att_time = grnd_att_time      - [0; 0;  0;  0;  0;  0;  0;   0;     0;   0;   0;   0;  0;   0;   0;    0;   0;   0;   0;   15; 15; 15; 15; 15; 15; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];
        grnd_att_time(1) = timer;
        local_pos_grndtruth_time = grnd_att_time;
        
    elseif (log_choice == 26)
        bias = [ 0.005, 0.016, -0.014];
        choice_gnd = [   0, 0.0,   0;
                         0, 0.0,   0;
                      -0.2, 0.2,   0;
                      -0.2, 0.2,   0;
                      -0.1, 0.2,   0;
                      -0.1, 0.2,   0;
                         0, 0.2,   0;
                         0, 0.2,   0;
                       0.1, 0.2,   0;
                       0.1, 0.2,   0;
                       0.2, 0.2,   0;                      
                       0.2, 0.2,   0;                      
                       0.2, 0.0,   0;
                       0.2, 0.0,   0;
                       0.1, 0.0,   0;
                       0.1, 0.0,   0;
                         0, 0.0,   0;
                         0, 0.0,   0;
                      -0.1, 0.0,   0;
                      -0.1, 0.0,   0;
                      -0.2, 0.0,   0;
                      -0.2, 0.0,   0;                    
                      -0.2,-0.2,   0;
                      -0.2,-0.2,   0;
                      -0.1,-0.2,   0;
                      -0.1,-0.2,   0;
                         0,-0.2,   0;
                         0,-0.2,   0;
                       0.1,-0.2,   0;
                       0.1,-0.2,   0;
                       0.2,-0.2,   0;
                       0.2,-0.2,   0;
                         0,   0,   0;
                         0,   0,   0;
                         0,   0,-0.15;
                         0,   0,-0.15;
                         0,   0, 0.1;
                         0,   0, 0.1   ];
        grnd_eul = zeros(size(choice_gnd)) + [0,0,pi/2];
        choice_gnd_att     = eul2quat(grnd_eul, 'XYZ');
                     
        choice_gnd = [ choice_gnd(:,2), choice_gnd(:,1), -choice_gnd(:,3)];           
        local_pos_grndtruth_time = vis_odom_time(1,1) + [0; 30; 45; 75; 90; 120; 135; 165; 180; 210; 225; 255; 270; 300; 315; 345; 360; 390; 405; 435; 450; 480; 495; 525; 540; 570; 585; 615; 630; 660; 675; 705; 720; 750; 765; 795; 810; 840];
        local_pos_grndtruth_time(1) = timer;
        grnd_att_time = local_pos_grndtruth_time;
    elseif (log_choice == 29)
        att_bias = [ 0.5, -1, 0.5];
        grnd_eul = [     0,  0,   90;
                         0,  0,   90;
                         0,  0,   90+5;
                         0,  0,   90+5;
                         0,  0,  90+10;
                         0,  0,  90+10;
                         0,  0,  90+15;
                         0,  0,  90+15;
                         0,  0,   90;
                         0,  0,   90;
                         0,  0,  90-5;
                         0,  0,  90-5;
                         0,  0, 90-10;
                         0,  0, 90-10;
                         0,  0,   90;
                         0,  0,   90;
                         0,  0, 90-15;
                         0,  0, 90-15;
                         0, 10,   90;
                         0, 10,   90;
                         0,  5,   90;
                         0,  5,   90;                        
                         0,-10,   90;
                         0,-10,   90;
                         0, -5,   90;
                         0, -5,   90;
                         0,  0,   90;
                         0,  0,   90;
                         0,  0,   90;
                         0,  0,   90;
                         5,  0,   90;
                         5,  0,   90;
                        10,  0,   90;
                        10,  0,   90;
                         0,  0,   90;
                         0,  0,   90;
                        -5,  0,   90;
                        -5,  0,   90;
                       -10,  0,   90;
                       -10,  0,   90;
                         0,  0,   90;
                         0,  0,   90];   
        grnd_eul = grnd_eul*pi/180;           
        choice_gnd_att     = eul2quat(grnd_eul, 'XYZ');
        choice_gnd = zeros(size(grnd_eul));
        grnd_att_time = vis_odom_time(1,1) + [0; 30; 45; 75; 90; 120; 135; 165; 180; 210; 225; 255; 270; 300; 315; 345; 360; 390; 405; 435; 450; 480; 495; 525; 540; 570; 585; 615; 630; 660; 675; 705; 720; 750; 765; 795; 810; 840; 855; 885; 900; 930];
        grnd_att_time = grnd_att_time      - [0; 0;  0;  0;  0;  0;  0;   0;     0;   0;   0;   0;  0;   0;   0;    0;   0;   0;   0;   15; 15; 15; 15; 15; 15; 15; 15; 15; 15; 15; 15; 15; 15; 15; 15; 15; 15; 15; 15; 15; 15; 15];
        grnd_att_time(1) = timer;
        local_pos_grndtruth_time = grnd_att_time;
    elseif (log_choice == 30)
        att_bias = [ 0.45, -0.75, 0.65];
        grnd_eul = [     0,  0,   90;
                         0,  0,   90;
                         0,  0,   90+5;
                         0,  0,   90+5;
                         0,  0,  90+10;
                         0,  0,  90+10;
                         0,  0,  90+15;
                         0,  0,  90+15;
                         0,  0,   90;
                         0,  0,   90;
                         0,  0,  90-5;
                         0,  0,  90-5;
                         0,  0, 90-10;
                         0,  0, 90-10;
                         0,  0, 90-15;
                         0,  0, 90-15;
                         0,  0,   90;
                         0,  0,   90;
                         0, 9,   90;
                         0, 9,   90;
                         0, 4.5,   90;
                         0, 4.5,   90;                        
                         0,  0,   90;
                         0,  0,   90;
                         0,-9,   90;
                         0,-9,   90;
                         0, -5,   90;
                         0, -5,   90;
                         0,  0,   90;
                         0,  0,   90;
                         4.5,  0,   90;
                         4.5,  0,   90;
                         9,  0,   90;
                         9,  0,   90;
                         0,  0,   90;
                         0,  0,   90;
                        -4.5,  0,   90;
                        -4.5,  0,   90;
                        -9.5,  0,   90;
                        -9.5,  0,   90;
                         0,  0,   90;
                         0,  0,   90];   
        grnd_eul = grnd_eul*pi/180;           
        choice_gnd_att     = eul2quat(grnd_eul, 'XYZ');
        choice_gnd = zeros(size(grnd_eul));
        grnd_att_time = vis_odom_time(1,1) + [0; 30; 45; 75; 90; 120; 135; 165; 180; 210; 225; 255; 270; 300; 315; 345; 360; 390; 405; 435; 450; 480; 495; 525; 540; 570; 585; 615; 630; 660; 675; 705; 720; 750; 765; 795; 810; 840; 855; 885; 900; 930];
        grnd_att_time = grnd_att_time      - [0; 0;  0;  0;  0;  0;  0;   0;     0;   0;   0;   0;  0;   0;   0;    0;   0;   0;   0;   15; 15; 15; 15; 15; 15; 15; 15; 15; 15; 15; 15; 15; 15; 15; 15; 15; 15; 15; 15; 15; 15; 15];
        grnd_att_time(1) = timer;
        local_pos_grndtruth_time = grnd_att_time;
    end  

end

choice_ekf = [pos_x+bias(1), pos_y+bias(2), pos_z+bias(3)];
choice_gps = [gps_pos_lat+gps_bias(1), gps_pos_lon+gps_bias(2), gps_pos_alt+gps_bias(3)];
choice_vio = [vis_odom_x + cam_offset(1) + bias(1), vis_odom_y+ cam_offset(2) + bias(2), vis_odom_z + cam_offset(3) + bias(3)];


vis_odom_time(find(vis_odom_time < timer)) = nan;
att_time(find(att_time < timer)) = nan;
pos_time(find(pos_time < timer)) = nan;
local_pos_grndtruth_time(find(local_pos_grndtruth_time < timer)) = nan;
gps_pos_time(find(gps_pos_time < timer)) = nan;
grnd_att_time(find(grnd_att_time < timer)) = nan;

vis_odom_time(find(vis_odom_time > max_time)) = nan;
att_time(find(att_time > max_time)) = nan;
pos_time(find(pos_time > max_time)) = nan;
local_pos_grndtruth_time(find(local_pos_grndtruth_time > max_time)) = nan;
gps_pos_time(find(gps_pos_time > max_time)) = nan;
grnd_att_time(find(grnd_att_time > max_time)) = nan;


figure(1);
% suptitle('Position');
    subplot(3,1,1)
        if(EKF)
            plot(pos_time, choice_ekf(:,1), 'Color', colour_EKF, 'DisplayName', 'EKF', 'Linewidth',lineWidth); hold on;
        end
        if(VIS)
            plot(vis_odom_time, choice_vio(:,1), 'Color', colour_VIS, 'DisplayName', 'EV', 'Linewidth',lineWidth); hold on;
        end
        if(GND)
            plot(local_pos_grndtruth_time, choice_gnd(:,1), 'Color', colour_GND, 'DisplayName', 'Gnd', 'Linewidth',lineWidth); hold on;
        end    
        if(GPS)
            plot(gps_pos_time, choice_gps(:,1), 'Color', colour_GPS, 'DisplayName', 'GPS', 'Linewidth',lineWidth); hold on;
        end    
       set(gca,'FontSize',14,'FontName','Times');        
        set(get(gca,'Title'),'interpreter','latex','FontSize',24,'FontName','Times');                
        set(get(gca,'XLabel'),'interpreter','latex','FontSize',14,'FontName','Times');
        set(get(gca,'YLabel'),'interpreter','latex','FontSize',14,'FontName','Times');
        title('North');
        xlabel('Time [s]');
        ylabel('Position [m]');
        ylim(PosYLtds(1,:));
        legend('show'); 
        grid on;
        hold off;
    
    subplot(3,1,2)
        if(EKF)
            plot(pos_time, choice_ekf(:,2), 'Color', colour_EKF, 'DisplayName', 'EKF', 'Linewidth',lineWidth); hold on;
        end
        if(VIS)
            plot(vis_odom_time, choice_vio(:,2), 'Color', colour_VIS, 'DisplayName', 'EV', 'Linewidth',lineWidth); hold on;
        end
        if(GND)
            plot(local_pos_grndtruth_time, choice_gnd(:,2), 'Color', colour_GND, 'DisplayName', 'Gnd', 'Linewidth',lineWidth); hold on;
        end    
        if(GPS)
            plot(gps_pos_time, choice_gps(:,2), 'Color', colour_GPS, 'DisplayName', 'GPS', 'Linewidth',lineWidth); hold on;
        end
        set(gca,'FontSize',14,'FontName','Times');        
        set(get(gca,'Title'),'interpreter','latex','FontSize',24,'FontName','Times');                
        set(get(gca,'XLabel'),'interpreter','latex','FontSize',14,'FontName','Times');
        set(get(gca,'YLabel'),'interpreter','latex','FontSize',14,'FontName','Times');
        
        title('East');        
        xlabel('Time [s]');
        ylabel('Position [m]');
        ylim(PosYLtds(2,:));
        legend('show'); 
        grid on;
        hold off;
   
    subplot(3,1,3)
        if(EKF)
            plot(pos_time, down_scaler*choice_ekf(:,3), 'Color', colour_EKF, 'DisplayName', 'EKF', 'Linewidth',lineWidth); hold on;
        end
        if(VIS)
            plot(vis_odom_time, down_scaler*choice_vio(:,3), 'Color', colour_VIS, 'DisplayName', 'EV', 'Linewidth',lineWidth); hold on;
        end
        if(GND)
            plot(local_pos_grndtruth_time, down_scaler*choice_gnd(:,3), 'Color', colour_GND, 'DisplayName', 'Gnd', 'Linewidth',lineWidth); hold on;
        end    
        if(GPS)
            plot(gps_pos_time, down_scaler*choice_gps(:,3), 'Color', colour_GPS, 'DisplayName', 'GPS', 'Linewidth',lineWidth); hold on;
        end
        set(gca,'FontSize',14,'FontName','Times');        
        set(get(gca,'Title'),'interpreter','latex','FontSize',24,'FontName','Times');                
        set(get(gca,'XLabel'),'interpreter','latex','FontSize',14,'FontName','Times');
        set(get(gca,'YLabel'),'interpreter','latex','FontSize',14,'FontName','Times');
        title('Down');
        xlabel('Time [s]');
        ylabel('Position [m]');
        ylim(PosYLtds(3,:));
        legend('show'); 
        grid on;
    	hold off;
    
    figures = figure(1);
    figures.PaperPositionMode='auto';
    fig_pos = figures.PaperPosition;
    figures.PaperSize = [fig_pos(3) fig_pos(4)];
    print(['/home/jjr/Pictures/FlightLog/' num2str(log_choice) 'Pos'],'-dpdf','-fillpage');

   
figure(2);
% suptitle('Attitude (Quaternions)');
    subplot(4,1,1)
        if(EKF)
            plot(att_time, choice_att(:,1), 'Color', colour_EKF, 'DisplayName', 'EKF', 'Linewidth',lineWidth); hold on;
        end
        if(VIS)
            plot(vis_odom_time, choice_vis_att(:,1), 'Color', colour_VIS, 'DisplayName', 'EV', 'Linewidth',lineWidth); hold on;
        end
        if(GND)
            plot(grnd_att_time, choice_gnd_att(:,1), 'Color', colour_GND, 'DisplayName', 'Gnd', 'Linewidth',lineWidth); hold on;
        end       
       set(gca,'FontSize',14,'FontName','Times');        
        set(get(gca,'Title'),'interpreter','latex','FontSize',24,'FontName','Times');                
        set(get(gca,'XLabel'),'interpreter','latex','FontSize',14,'FontName','Times');
        set(get(gca,'YLabel'),'interpreter','latex','FontSize',14,'FontName','Times');
        title('W');
        xlabel('Time [s]');
        ylabel('Norm');
        legend('show'); 
        hold off;

    subplot(4,1,2)
        if(EKF)
            plot(att_time, choice_att(:,2), 'Color', colour_EKF, 'DisplayName', 'EKF', 'Linewidth',lineWidth); hold on;
        end
        if(VIS)
            plot(vis_odom_time, choice_vis_att(:,2), 'Color', colour_VIS, 'DisplayName', 'EV', 'Linewidth',lineWidth); hold on;
        end
        if(GND)
            plot(grnd_att_time, choice_gnd_att(:,2), 'Color', colour_GND, 'DisplayName', 'Gnd', 'Linewidth',lineWidth); hold on;
        end  
       set(gca,'FontSize',14,'FontName','Times');        
        set(get(gca,'Title'),'interpreter','latex','FontSize',24,'FontName','Times');                
        set(get(gca,'XLabel'),'interpreter','latex','FontSize',14,'FontName','Times');
        set(get(gca,'YLabel'),'interpreter','latex','FontSize',14,'FontName','Times');
        title('X');
        xlabel('Time [s]');
        ylabel('Norm');
        legend('show'); 
        hold off;

    subplot(4,1,3)
       if(EKF)
            plot(att_time, choice_att(:,3), 'Color', colour_EKF, 'DisplayName', 'EKF', 'Linewidth',lineWidth); hold on;
        end
        if(VIS)
            plot(vis_odom_time, choice_vis_att(:,3), 'Color', colour_VIS, 'DisplayName', 'EV', 'Linewidth',lineWidth); hold on;
        end
        if(GND)
            plot(grnd_att_time, choice_gnd_att(:,3), 'Color', colour_GND, 'DisplayName', 'Gnd', 'Linewidth',lineWidth); hold on;
        end  
       set(gca,'FontSize',14,'FontName','Times');        
        set(get(gca,'Title'),'interpreter','latex','FontSize',24,'FontName','Times');                
        set(get(gca,'XLabel'),'interpreter','latex','FontSize',14,'FontName','Times');
        set(get(gca,'YLabel'),'interpreter','latex','FontSize',14,'FontName','Times');   
        title('Y');
        xlabel('Time [s]');
        ylabel('Norm');
        legend('show'); 
        hold off;

    subplot(4,1,4)
       if(EKF)
            plot(att_time, choice_att(:,4), 'Color', colour_EKF, 'DisplayName', 'EKF', 'Linewidth',lineWidth); hold on;
        end
        if(VIS)
            plot(vis_odom_time, choice_vis_att(:,4), 'Color', colour_VIS, 'DisplayName', 'EV', 'Linewidth',lineWidth); hold on;
        end
        if(GND)
            plot(grnd_att_time, choice_gnd_att(:,4), 'Color', colour_GND, 'DisplayName', 'Gnd', 'Linewidth',lineWidth); hold on;
        end  
        set(gca,'FontSize',14,'FontName','Times');        
        set(get(gca,'Title'),'interpreter','latex','FontSize',24,'FontName','Times');                
        set(get(gca,'XLabel'),'interpreter','latex','FontSize',14,'FontName','Times');
        set(get(gca,'YLabel'),'interpreter','latex','FontSize',14,'FontName','Times'); 
        title('Z');
        xlabel('Time [s]');
        ylabel('Norm');
        legend('show'); 
        hold off;

   
    figures = figure(2);
    figures.PaperPositionMode='auto';
    fig_pos = figures.PaperPosition;
    figures.PaperSize = [fig_pos(3) fig_pos(4)];
    print(['/home/jjr/Pictures/FlightLog/' num2str(log_choice) 'Quat'],'-dpdf','-fillpage');

    
figure(3);
    subplot(3,1,1)
    
       if(EKF)
            plot(att_time, att_eul(:,1)*180/pi + att_bias(1), 'Color', colour_EKF, 'DisplayName', 'EKF', 'Linewidth',lineWidth); hold on;            
        end
        if(VIS)
            plot(vis_odom_time, vis_eul(:,1)*180/pi + att_bias(1), 'Color', colour_VIS, 'DisplayName', 'EV', 'Linewidth',lineWidth); hold on;            
        end
        if(GND)
            plot(grnd_att_time, grnd_eul(:,1)*180/pi, 'Color', colour_GND, 'DisplayName', 'Gnd', 'Linewidth',lineWidth); hold on;            
        end  
        set(gca,'FontSize',14,'FontName','Times');        
        set(get(gca,'Title'),'interpreter','latex','FontSize',24,'FontName','Times');                
        set(get(gca,'XLabel'),'interpreter','latex','FontSize',14,'FontName','Times');
        set(get(gca,'YLabel'),'interpreter','latex','FontSize',14,'FontName','Times');
        title('Roll');
        xlabel('Time [s]');
        ylabel('Angle [deg]');
        ylim(EulLtds(1,:));
        legend('show');
        grid on;
        hold off;


    subplot(3,1,2)
    if(EKF)
            plot(att_time, att_eul(:,2)*180/pi + att_bias(2), 'Color', colour_EKF, 'DisplayName', 'EKF', 'Linewidth',lineWidth); hold on;            
        end
        if(VIS)
            plot(vis_odom_time, vis_eul(:,2)*180/pi + att_bias(2), 'Color', colour_VIS, 'DisplayName', 'EV', 'Linewidth',lineWidth); hold on;            
        end
        if(GND)
            plot(grnd_att_time, grnd_eul(:,2)*180/pi, 'Color', colour_GND, 'DisplayName', 'Gnd', 'Linewidth',lineWidth); hold on;            
        end   
       set(gca,'FontSize',14,'FontName','Times');        
        set(get(gca,'Title'),'interpreter','latex','FontSize',24,'FontName','Times');                
        set(get(gca,'XLabel'),'interpreter','latex','FontSize',14,'FontName','Times');
        set(get(gca,'YLabel'),'interpreter','latex','FontSize',14,'FontName','Times');
        title('Pitch');
        xlabel('Time [s]');
        ylabel('Angle [deg]');
        ylim(EulLtds(2,:));
        legend('show'); 
        grid on;
        hold off;


    subplot(3,1,3)
        
         if(EKF)
            plot(att_time, att_eul(:,3)*180/pi + att_bias(3), 'Color', colour_EKF, 'DisplayName', 'EKF', 'Linewidth',lineWidth); hold on;            
        end
        if(VIS)
            plot(vis_odom_time, vis_eul(:,3)*180/pi + att_bias(3), 'Color', colour_VIS, 'DisplayName', 'EV', 'Linewidth',lineWidth); hold on;            
        end
        if(GND)
            plot(grnd_att_time, grnd_eul(:,3)*180/pi + att_bias(3), 'Color', colour_GND, 'DisplayName', 'Gnd', 'Linewidth',lineWidth); hold on;            
        end  
        set(gca,'FontSize',14,'FontName','Times');        
        set(get(gca,'Title'),'interpreter','latex','FontSize',24,'FontName','Times');                
        set(get(gca,'XLabel'),'interpreter','latex','FontSize',14,'FontName','Times');
        set(get(gca,'YLabel'),'interpreter','latex','FontSize',14,'FontName','Times');
        
        title('Yaw');
        xlabel('Time [s]');
        ylabel('Angle [deg]');
        ylim(EulLtds(3,:));
        legend('show');
        grid on;
        hold off;
    
        
    figures = figure(3);
    figures.PaperPositionMode='auto';
    fig_pos = figures.PaperPosition;
    figures.PaperSize = [fig_pos(3) fig_pos(4)];
    print(['/home/jjr/Pictures/FlightLog/' num2str(log_choice) 'Eul'],'-dpdf','-fillpage');


    