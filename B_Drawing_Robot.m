% ______________________________________________________
% ___________________written by_________________________
% _______S A K S O R N____ R U A N G T A N U S A K______
% ______________________________________________________
% RUN WORKSPACE Generate TO GET Plot2d_data.csv first! 

clc
clear 
close all
%% Set Parameters
syms theta1 theta2 theta3 theta4 theta5 theta6
DH_param = [[200;890;150;0;0;0] [-3.14/2; 0; -3.14/2; 3.14/2; -3.14/2; 0] [680;0;0;880;0;140] [theta1;theta2-pi()/2;theta3;theta4;theta5;theta6]]
ROM_param =  [[165 -165];[95 -70]; [65 -60]; [200 -200]; [120 -120]; [400 -400]] .*(pi()/180.0)
interval = [33*3,14*2,12*2,1,1,1];
limit_cnc = [400,1200] % 400:100:1200

% interval = [15,6,6,1,1,1];
limit_var = [2000,2000,2601];

final_time_steps = 1;
for i = 2:length(interval)
    final_time_steps = final_time_steps*interval(i);
end
%% Find other prop from DH parameters
global d
global a
d = [DH_param(1,3) DH_param(2,3) DH_param(3,1)];
a = [DH_param(1,1) DH_param(2,1) sum(DH_param(4:6,3))];

%% Read 2D Array from CSV (In case of didn't need calculation)
plot2d_raw = readmatrix('plot2d_data.csv');
plot2d_cutz = []

plot2d_cutz = (plot2d_raw(1,:) >= 0);
x_cut = plot2d_raw(1,plot2d_cutz);
z_cut = plot2d_raw(3,plot2d_cutz);

plot2d_arr = [x_cut; zeros(1,length(z_cut)); z_cut];
fprintf("Finished Read 2D Array\n")
%% Make 3D Array
plot3d_arr3 = [];
for angle1 = linspace(ROM_param(1,2),ROM_param(1,1),interval(1))
    Transform_arr = [plot2d_arr(1,:).*cos(angle1); plot2d_arr(1,:).*sin(angle1); plot2d_arr(3,:)];
    plot3d_arr3 = [plot3d_arr3 Transform_arr];
end
fprintf("Finished Making 3D Array\n")    
%% Draw Canvas
fh1 = figure();
fh1.WindowState = 'maximized';

axh = axes;
hold(axh, 'on')
% axis([1000 2000 -1000 1000])
axis([-2000 2000 -2000 2000])
draw_pt = [900,1800]
A = plot2d_arr(3,:) 
min(A)

plot3d_arr2 = [];
for angle1 = linspace(ROM_param(1,2),ROM_param(1,1),interval(1))
    Transform_arr = [draw_pt.*cos(angle1); draw_pt.*sin(angle1)];
    plot3d_arr2 = [plot3d_arr2 Transform_arr];
end

x = plot3d_arr2(1,:)
y = plot3d_arr2(2,:)

% plot(x,y)
hold on
k = boundary(x',y',0.9);

plot(x(k),y(k),'Color','b')
axis equal
xlabel("X-axis"); ylabel("Y-axis")

xy = [0,0];
xy_mat = [];
i=1;
while ~isempty(xy)
    xy = ginput(1);

    if ~isempty(xy)
        xy_mat(i,1) = xy(1)
        xy_mat(i,2) = xy(2)
        i=i+1;
        plot(axh, xy(1), xy(2));

        plot(axh, xy(1), xy(2), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
%         plot(axh, xy(1), xy(2),'MarkerFaceColor', 'b');
    end
end
hold off
close(fh1)
%% Plot 3D Surface Workspace
s_pt = 2000; l_b = 400; % low bound
fh = figure();
fh.WindowState = 'maximized';

plot3(linspace(0,limit_var(1),10),linspace(0,0,10),linspace(l_b,l_b,10), '--k','LineWidth',2) % x axis
hold on
plot3(linspace(0,0,10),linspace(0,limit_var(2),10),linspace(l_b,l_b,10), '--k','LineWidth',2) % y axis
plot3(linspace(0,0,10),linspace(0,0,10),linspace(-418,limit_var(3),10), '--k','LineWidth',2) % z axis
j = boundary(plot3d_arr3',0.9);
trisurf(j,plot3d_arr3(1,:),plot3d_arr3(2,:),plot3d_arr3(3,:),'FaceAlpha',0.2,'LineWidth',0.01,'LineStyle','none')

grid on
hold on
axis equal
axis([-2500 2500 -2500 2500 400 2600])
xlabel("X-axis")
ylabel("Y-axis")
zlabel("Z-axis")
title('HARRY CNC BOT')
%% Plot Animated
angle_mat_arr = [];
curve = animatedline('LineWidth',1,'LineStyle','--');
k=1;
for i = flip(linspace(limit_cnc(1),limit_cnc(2),20)) 
    for j =1:length(xy_mat)
        xc = xy_mat(j,1);         yc = xy_mat(j,2);        zc = i;
        addpoints(curve,xc,yc,zc);
        [angle_mat_out,err] = inv_kine([xc,yc,zc]);
        if err == 0
            color1 = 'b';
        else
            color1 = 'r';
        end
        head = scatter3(xc,yc,zc,'filled','MarkerFaceColor',color1);
        
        [x5,y5,z5] = r_viz(angle_mat_out,xc,yc,zc);
        ang_text = text(x5(2:4),y5(2:4),z5(2:4),["    " + string(angle_mat_out)],'Color',color1,'FontSize',14,'FontWeight','bold'); 
        visual = plot3(x5,y5,z5,'.-','Color',color1,'LineWidth',4);
        drawnow
        pause(0.2);
        delete(head); delete(ang_text); delete(visual);
        angle_mat_arr = [angle_mat_arr angle_mat_out'];
    end
end
visual = plot3(x5,y5,z5,'.-','Color',color1,'LineWidth',4);
head = scatter3(xc,yc,zc,'filled','MarkerFaceColor',color1);
ang_text = text(x5(3:5),y5(3:5),z5(3:5),["   " + string(angle_mat_out)],'Color',color1,'FontSize',14,'FontWeight','bold'); 
%%  Save Angle.csv
vars = {'Theta1','Theta2','Theta3'};
t = array2table(angle_mat_arr','VariableNames',{'Theta1','Theta2','Theta3'});
t = t(:,vars);
writetable(t,'angle.csv')
%% Def Inverse Kinematics Function
function [angle_mat_final,err] = inv_kine(loc)
    d = [680 0 150];
    a = [200 890 1020];
    
    xc=loc(1); yc=loc(2); zc=loc(3);
    r = [sqrt(xc^2 + yc^2)-a(1), zc-d(1)];
    r = [r,sqrt(r(1)^2 + r(2)^2), sqrt(a(3)^2 + d(3)^2)];
    theta1 = atan2d(yc,xc);
    theta2 = acosd((a(3)^2+d(3)^2-a(2)^2-r(3)^2)/(-2*(a(2)*r(3)))) - atan2d(r(1),r(2));
    theta3 = acosd((r(3)^2-r(4)^2-a(2)^2)/(-2*(r(4)*a(2)))) - 180 + atan2d(a(3),d(3));
    ROM_param =  [[165 -165];[95 -80]; [65 -60]];

    theta_mat = [theta1 theta2 theta3];

    angle_mat = round(theta_mat,2);
    error_mat = [0 0 0];

    for i = 1:length(angle_mat)
        if (angle_mat(i) > ROM_param(i,1)) | (angle_mat(i) < ROM_param(i,2))
    %         fprintf("error %d\n", i)
            error_mat(i) = 1;
        end
    end
    if max(error_mat) == 1
        fprintf('[angle_mat (deg) ] = [')
        for i = 1:length(angle_mat)
            if error_mat(i) == 1
                fprintf('error! ')
                err = 1;
%                 angle_mat = [1000 1000 1000];
            else
                fprintf('%.2f ',angle_mat(i))
            end
        end
        fprintf(']\n')
    else
        fprintf('[angle_mat (deg) ] = [%.2f %.2f %.2f]\n',angle_mat)
        err = 0;
    end
    angle_mat_final = angle_mat;
end
%% Robot Arm Visualized
function [x,y,z] = r_viz(th,xc,yc,zc)
    global d a
    row12 = [0 0 0; 0 0 d(1)];
    row3  = [ a(1)*cosd(th(1)) a(2)*sind(th(1)) d(1)];
    row4  = double(row3 + [-a(2)*sind(th(2))*cosd(th(1)) -a(2)*sind(th(2))*sind(th(1)) a(2)*cosd(th(2))]);
    row5  = [xc yc zc];
    coor_plt = double([row12;row3;row4;row5]);
    x = coor_plt(:,1)'; y = coor_plt(:,2)'; z = coor_plt(:,3)';
%     visual = plot3(x,y,z,'.-b','LineWidth',4);
end