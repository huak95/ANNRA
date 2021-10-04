% ______________________________________________________
% ___________________written by_________________________
% _______S A K S O R N____ R U A N G T A N U S A K______
% ______________________________________________________

clc
clear
close all
%% Set Parameters

syms theta1 theta2 theta3 theta4 theta5 theta6
DH_param = [[200;890;150;0;0;0] [-3.14/2; 0; -3.14/2; 3.14/2; -3.14/2; 0] [680;0;0;880;0;140] [theta1;theta2-pi()/2;theta3;theta4;theta5;theta6]]
ROM_param =  -[[165 -165];[95 -70]; [65 -60]; [200 -200]; [120 -120]; [400 -400]] .*(pi()/180.0)
interval = [33*3,14,12,1,1,1];
limit_var = [1237,1237,2601]

% Continue? (Please type true / false)
cont = false;

final_time_steps = 1
for i = 2:length(interval)
    final_time_steps = final_time_steps*interval(i)
end
%% DH-Parameters Calculation
if cont == 0
    syms theta alpha d a
    A(a,alpha,d,theta) = [[cos(theta) -sin(theta)*cos(alpha)  sin(theta)*sin(alpha)   a*cos(theta)];
        [sin(theta)  cos(theta)*cos(alpha)    -cos(theta)*sin(alpha)  a*sin(theta)];
        [0           sin(alpha)              cos(alpha)              d];
        [0 0 0 1]];

    A1(theta1) = subs(A,[a alpha d theta],DH_param(1,:));
    A2(theta2) = subs(A,[a alpha d theta],DH_param(2,:));
    A3(theta3) = subs(A,[a alpha d theta],DH_param(3,:));
    A4(theta4) = subs(A,[a alpha d theta],DH_param(4,:));
    A5(theta5) = subs(A,[a alpha d theta],DH_param(5,:));
    A6(theta6) = subs(A,[a alpha d theta],DH_param(6,:));

    T_mat = A1(theta1)*A2(theta2)*A3(theta3)*A4(0)*A5(0)*A6(0);

    x_co(theta1,theta2,theta3) = T_mat(1,4);
    y_co(theta1,theta2,theta3) = T_mat(2,4);
    z_co(theta1,theta2,theta3) = T_mat(3,4);

    T_mat_co(theta1,theta2,theta3) = [T_mat(1,4);T_mat(2,4);T_mat(3,4)]
    coor = []
    angle2_extend = []
    angle3_extend = []
    i = 0

    test_diff = linspace(ROM_param(3,1),ROM_param(3,2),interval(3))
    diff_int = test_diff(2)-test_diff(1) % diff interval

    for angle3 = linspace(ROM_param(3,1),ROM_param(3,2),interval(3)) %[-60,65]
        if angle3 > -40*pi()/180
           ROM_param(2,2) = ROM_param(2,2) - (12*pi()/180);
        elseif   angle3 >= 0
           ROM_param(2,2) = 0*pi()/180;
        else
            ROM_param(2,2) = 70*pi()/180;
        end
        for angle2 = linspace(ROM_param(2,1),ROM_param(2,2),interval(2)) %[-70,95]
    %        fprintf('t2: %d, t3: %d\n',round(-angle2*180/pi()),round(-angle3*180/pi())) 
           coor = horzcat(coor,subs(T_mat_co,[theta2,theta3],[-angle2,-angle3]));
           i=i+1;
           fprintf('TS now:%i TS final: %i\n', i,final_time_steps)
        end
    end
end
%% Read 2D Array from CSV (In case of didn't need calculation)
if cont == 1
    plot2d_arr = readmatrix('plot2d_data.csv');
    fprintf("Finished Read 2D Array\n")
end
%% Make 2D Array
if cont == 0
    plot2d_arr = double(subs(coor,[theta1],[0]));
    writematrix(plot2d_arr,'plot2d_data.csv');
    fprintf("Finished Making 2D Array\n")
end
%% Make 3D Array
plot3d_arr2 = [];
for angle1 = linspace(ROM_param(1,2),ROM_param(1,1),interval(1))
    Transform_arr = [plot2d_arr(1,:).*cos(angle1); plot2d_arr(1,:).*sin(angle1); plot2d_arr(3,:)];
    plot3d_arr2 = [plot3d_arr2 Transform_arr];
end
fprintf("Finished Making 3D Array\n")    
%% Plot 2D Workspace (Plot 1)
f1 = figure(1);
arm_x = [0 0 200 200 200 1220];
arm_z = [0 680 680 1570 1720 1720];

plot(plot2d_arr(1,:),plot2d_arr(3,:), '.','Color','#0072BD')
hold on
grid on
plot(arm_x,arm_z,'-or','MarkerSize',10,'LineWidth',2)
title('2D Workspace')

x = plot2d_arr(1,:);
z = plot2d_arr(3,:);
k = boundary(x',z',0.7);

plot(x(k),z(k),'Color','b')
axis equal
% axis([-2000 3000 -1000 3000])
xlabel("X-axis")
ylabel("Z-axis")
title('2D Workspace')
hold off
%% Make Boundary 3D Array 
plot3d_bound = [];
for angle1 = linspace(ROM_param(1,2),ROM_param(1,1),interval(1))
    if angle1 == ROM_param(1,2)
        Transform_arr = [plot2d_arr(1,:).*cos(angle1); plot2d_arr(1,:).*sin(angle1); plot2d_arr(3,:)];
    elseif angle1 == ROM_param(1,1)
        Transform_arr = [plot2d_arr(1,:).*cos(angle1); plot2d_arr(1,:).*sin(angle1); plot2d_arr(3,:)];
    else
        Transform_arr = [x(k).*cos(angle1); x(k).*sin(angle1); z(k)];
    end
    plot3d_bound = [plot3d_bound Transform_arr];
end
%% Plot 3D Point Cloud (Plot 2)
% 
f2 = figure(2);
plot3(linspace(0,limit_var(1),10),linspace(0,0,10),linspace(0,0,10), '-r','LineWidth',2) % x axis
hold on
plot3(linspace(0,0,10),linspace(0,limit_var(2),10),linspace(0,0,10), '-r','LineWidth',2) % y axis
plot3(linspace(0,0,10),linspace(0,0,10),linspace(-418,limit_var(3),10), '-r','LineWidth',2) % z axis

plot3(plot3d_arr2(1,:),plot3d_arr2(2,:),plot3d_arr2(3,:), '.','MarkerSize',3.5,'Color','k')

grid on
hold off
axis equal
xlabel("X-axis")
ylabel("Y-axis")
zlabel("Z-axis")
title('3D Point Workspace')

%% Plot 3D Surface Workspace (Plot 3)
f3 = figure(3);
plot3(linspace(0,limit_var(1),10),linspace(0,0,10),linspace(0,0,10), '-r','LineWidth',2) % x axis
hold on
plot3(linspace(0,0,10),linspace(0,limit_var(2),10),linspace(0,0,10), '-r','LineWidth',2) % y axis
plot3(linspace(0,0,10),linspace(0,0,10),linspace(-418,limit_var(3),10), '-r','LineWidth',2) % z axis

plot3(plot3d_bound(1,:),plot3d_bound(2,:),plot3d_bound(3,:),'--','Color',ones(1,3)*0.5)

plane_len = length(plot2d_arr)
plot3(plot3d_bound(1,1:plane_len),plot3d_bound(2,1:plane_len),plot3d_bound(3,1:plane_len),'-','Color','k')
plot3(plot3d_bound(1,end-plane_len:end),plot3d_bound(2,end-plane_len:end),plot3d_bound(3,end-plane_len:end),'-','Color','k')

j = boundary(plot3d_arr2',0.9);
trisurf(j,plot3d_arr2(1,:),plot3d_arr2(2,:),plot3d_arr2(3,:),'FaceAlpha',0.1,'EdgeColor','k','LineWidth',0.01,'LineStyle','none')

grid on
hold off
axis equal
xlabel("X-axis")
ylabel("Y-axis")
zlabel("Z-axis")
title('3D Surface Workspace')