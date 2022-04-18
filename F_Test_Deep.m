clear
clc
close all
hrobot = harryBotDef;
% hrobot = robotDef
%% Set Parameters
syms theta1 theta2 theta3 theta4 theta5 theta6
interval = [33*3,14*2,12*2,1,1,1];
% limit_cnc = [400,1200] % 400:100:1200
limit_cnc = [800,1200] % 400:100:1200

% interval = [15,6,6,1,1,1];

limit_var = [2000,2601];
DH_param = [[200;890;150;0;0;0] [-3.14/2; 0; -3.14/2; 3.14/2; -3.14/2; 0] [680;0;0;880;0;140] [theta1;theta2-pi()/2;theta3;theta4;theta5;theta6]]
ROM_param =  [[165 -165];[95 -70]; [65 -60]; [200 -200]; [120 -120]; [400 -400]] .*(pi()/180.0)

final_time_steps = 1;
for i = 2:length(interval)
    final_time_steps = final_time_steps*interval(i);
end
%% Find other prop from DH parameters
d = double([DH_param(1,3) DH_param(2,3) DH_param(3,1)])*1e-3;
a = double([DH_param(1,1) DH_param(2,1) sum(DH_param(4:6,3))])*1e-3;
%% Crate 2D Workspace Array
plot2d_arr = array_2d_gen

%% Draw Canvas
fh1 = figure();
fh1.WindowState = 'maximized';

axh = axes;
hold(axh, 'on')
% axis([1000 2000 -1000 1000])
axis(([-2000 2000 -2000 2000])*1e-3)
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

plot(x(k)*1e-3,y(k)*1e-3,'Color','b')
axis equal
xlabel("X-axis (m)"); ylabel("Y-axis (m)")

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
xy_mat2 = path_gen(xy_mat);
figure(9)
plot(xy_mat(:,1), xy_mat(:,2),"-o")
hold on
plot(xy_mat2(:,1), xy_mat2(:,2),"-x")
hold off
axis equal
xlabel("X-axis (m)"); ylabel("Y-axis (m)")
legend('original-path','gen-path')

%% Inversed Kinematic

xc = [];
yc = [];
zc = [];
count = 1
coor_xyz = []
% zc_lim = linspace(limit_cnc(1),limit_cnc(2),2
z_bound = flip(linspace(limit_cnc(1),limit_cnc(2),length(xy_mat2)))/1000
% for i =  z_bound
    for j =1:length(xy_mat2)
        xc = xy_mat2(j,1);         yc = xy_mat2(j,2);        zc = z_bound(j);
%         zc = i;
        [path_single,err] = inv_kine([xc,yc,zc]);
        coor_xyz(count,:) = [xc,yc,zc];
        path(count,:) = path_single;
        count = count + 1;
    end
% end
coor_xyz

%%
path_xyz = coor_xyz(1:size(coor_xyz,1),:);
path_pre = []
for i=1:length(path_xyz)
    path_pre(i,:) = deg2rad(inv_kine(path_xyz(i,:)))
end

%% Plot 1: Sgolayfilt Angle
f1 =figure(1)
path_fit = sgolayfilt(path_pre,3,11)
f1.Position = [10 10 500 600]
f1.Name = "path-filtering"

for i=1:3
    subplot(3,1,i)
    plot(path_pre(:,i),'-x','DisplayName',"Before")
    hold on
    plot(path_fit(:,i),'-x','DisplayName',"After")
    xlabel("Time"); ylabel(["q_"+int2str(i)+" (rad)"])
    l = legend; l.Location = "best"
    hold off
end
saveas(1,"1_path-filtering.svg","svg")
path = path_fit
%% Path Diff and Forward Kinematics
time = linspace(0,10,length(path));
pathd = [zeros(1,3); diff(path)]; 
pathdd = [zeros(1,3); diff(pathd)];
pathcoord=[];
for i = 1:length(path)
%     i
    path_T_mat = double(double(hrobot.fkine(path(i,:))));
    pathcoord(i,:) = path_T_mat(1:3,4);
end
%% RNE - Inversed Dynamics
tic
Q = hrobot.rne(path, pathd, pathdd);
toc
load("net_fdyn.mat")
load net_idyn

%%
tic
Q_fdyn = net_fdyn(path')'
toc

tic
Q_idyn = net_idyn(path')'
toc

%% Control - Forward Dynamics
KP=[1 200 100];
KD=[1 3 3];
qreal1=joint_control(hrobot,path, pathd, pathdd, Q, KP, KD); 
q_fdyn=joint_control(hrobot,path, pathd, pathdd, Q_fdyn, KP, KD); 
q_idyn=joint_control(hrobot,path, pathd, pathdd, Q_idyn, KP, KD); 
%% Forward Kinematics (Deep Learning)
%fdyn
coor_fdyn = [];
qreal = q_fdyn;
for i=1:length(qreal)
%     i
    T1 = double(double(hrobot.fkine(qreal(i,:))));
    coor_fdyn(:,i) = [T1(1,4) T1(2,4) T1(3,4)];
end
coor_fdyn = coor_fdyn';

% idyn
coor_idyn = [];
qreal = q_idyn;
for i=1:length(qreal)
%     i
    T1 = double(double(hrobot.fkine(qreal(i,:))));
    coor_idyn(:,i) = [T1(1,4) T1(2,4) T1(3,4)];
end
coor_idyn = coor_idyn';

%%
realcoord = [];
qreal = qreal1;
for i=1:length(qreal)
    T1 = double(double(hrobot.fkine(qreal(i,:))));
    realcoord(:,i) = [T1(1,4) T1(2,4) T1(3,4)];
end
realcoord = realcoord'
%% Plot 2: Path Comparison
f2=figure(2)
f2.Name = "Path-Comparison"
plot3path(pathcoord)
hold on
plot3path(realcoord)
plot3path(coor_fdyn)
plot3path(coor_idyn)
view(3) % view at top
% plot3path(path_xyz/1000)
legend(["Planning","Actual","ANN-fdyn","ANN-idyn"],"Box","off",'Location','best')
xlabel("X-axis (m)"); ylabel("Y-axis (m)"); zlabel("Z-axis (m)")
axis equal
hold off
grid on
saveas(2,"2_path-compare.svg",'svg')
%%
% plot3path(pathcoord)

%% Plot 3: Each joint response
f = figure(3)
f.Position = [10 10 500 600]
% l_deep = length(Q_fdyn)
l_deep = 10
time = 1:l_deep;
for i = 1:3
    subplot(size(path,2),1,i)
    plot(time,path(1:l_deep,i),'--','DisplayName',["Planning"+int2str(i)])
    hold on
    plot(time,qreal(1:l_deep,i),'.--k','DisplayName',["Actual q"+int2str(i)],'MarkerSize',10)
    plot(time,q_fdyn(1:l_deep,i),'DisplayName',["ANN-fdyn q"+int2str(i)])
    plot(time,q_idyn(1:l_deep,i),'DisplayName',["ANN-idyn q"+int2str(i)])
    l = legend; xlabel("Time"); ylabel(["q_"+int2str(i)+" (rad)"])
    l.Location = "best"
%     title(["KP = "+int2str(KP(i))+", KD = " + int2str(KD(i))])
%     title(sprintf('KP = %0.2f,  KD = %0.2f',KP(i), KD(i)))
    hold off
end
saveas(3,"3_plot_q.svg","svg")
%% Plot Animate
f5 = figure
f5.WindowState = 'maximized';

plot_a = '-'
plot3path2(realcoord,plot_a)
hold on
plot3path2(pathcoord,plot_a)
plot3path2(coor_fdyn,plot_a)
plot3path2(coor_idyn,plot_a)
% plot3path(path_xyz/1000)
legend(["realcoord","pathcoord"],"Box","off",'Location','best')
axis equal
axis([-3 3 -3 3 0 3])
% axis([-1 2 -1 2 0 3])
grid on
% hrobot.plot([q_fdyn])
for i=1:length(path_xyz)
%     path(i,:) = deg2rad(inv_kine(path_xyz(i,:)))
    hrobot.plot([q_fdyn(i,:)])
        pause(0.0001);
end
% hrobot.plot3d(qreal1)
%% Def Function

function  E=joint_control(hrobot,q,qd,qdd,Tau,KP,KD)

    T=linspace(1,length(q),length(q));
     
%      CF=1;
     R=0.003;
     J=0.01;
     B=0;
    
 for i=1:length(q)
%     i
    for j=1:3
%      f = @(t,x)[x(2);-CF*KP(j)/(R*J)*x(1) - (R*B +CF^2 +CF*KD(j))/ (R*J)*x(2) + CF*KP(j)*q(i,j)/(R*J) + CF*KP(j)/(R*J)*qd(i,j) - Tau(i,j)/J]; 
     f = @(t,x)[x(2);(-KP(j)/(R*J)*x(1) - (R*B +KD(j))/ (R*J)*x(2) + KP(j)*q(i,j)/(R*J) + KP(j)/(R*J)*qd(i,j)) - Tau(i,j)/J]; 
       if (i==1)
         [t,x] = ode15s(f,[T(i) T(i+1)],[ q(1,j);qd(1,j)]);
         last=length(t);
         E(i,j)=x(last,1);
       end
       if(i<length(q) && i~=1)
         [t,x]=ode15s(f,[T(i) T(i+1)],[ q(i-1,j);qd(i-1,j)]);
         last=length(t);
         E(i,j)=x(last,1);
       end
       if(i==length(q))
         [t,x] = ode15s(f,[T(i-1) T(i)],[ q(end,j);qd(end,j)]);
         last=length(t);
         E(i,j)=x(last,1);
       end
    end
 end
end

function plot3path(path_xyz)
    hold on
    plot3(path_xyz(:,1),path_xyz(:,2),path_xyz(:,3),'-x')
end

function plot3path2(path_xyz, point)
    hold on
    plot3(path_xyz(:,1),path_xyz(:,2),path_xyz(:,3),point)
end

function array_out = path_gen(array_in)
%     figure("Path Gen")
    new_x_arr = []; new_y_arr = []
    x = [array_in(:,1)];
    y = [array_in(:,2)];
    
    % Re to first point
%     x = [array_in(:,1); array_in(1,1)];
%     y = [array_in(:,2); array_in(1,2)];
%     fprintf("x: %d\n",length(x))
    
   for i = 1:(length(x)-1)
%         i
        X = x(i+1)-x(i);
        Y = y(i+1)-y(i);
        R = sqrt(X^2+Y^2)
        if R < 0.5
            INTERP_SIZE = round(2.0/R)
        elseif R < 1.0
            INTERP_SIZE = round(8.0/R)
        else 
            INTERP_SIZE = round(20.0/R)
        end
                new_x = [linspace(x(i),x(i+1),INTERP_SIZE)];
%         new_x = [linspace(x(i),x(i+1),10)];
        new_y = interp1(x(i:i+1),y(i:i+1),new_x);
        new_x_arr = [new_x_arr new_x(2:end-1)];
        new_y_arr = [new_y_arr new_y(2:end-1)];
    end
    new_x_arr = [x(1) new_x_arr x(end)];
    new_y_arr = [y(1) new_y_arr y(end)];
    array_out = [new_x_arr; new_y_arr]';
    
%     plot(x,y,"-o")
%     hold on
%     plot(new_x_arr, new_y_arr,"-x")
%     hold off
%     axis equal
%     legend('original-path','gen-path')
end

function plot2d_arr = array_2d_gen()
    % Read 2D Array from CSV (In case of didn't need calculation)
    plot2d_raw = readmatrix('plot2d_data.csv');
    plot2d_cutz = []

    plot2d_cutz = (plot2d_raw(1,:) >= 0);
    x_cut = plot2d_raw(1,plot2d_cutz);
    z_cut = plot2d_raw(3,plot2d_cutz);

    plot2d_arr = [x_cut; zeros(1,length(z_cut)); z_cut];
%     fprintf("Finished Read 2D Array\n")
end

