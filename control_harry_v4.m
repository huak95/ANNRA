clear
clc
close all
hrobot = harryBotDef;
% hrobot = robotDef
%%
% Tau = hrobot.gravload([0 0 0])

path_xyz1 = readmatrix("rectangle_coor_xyz.csv");
path_num = 50
path_xyz = [path_xyz1(1:path_num,1:2)*1.5 - 200, path_xyz1(1:path_num,3)];
% path_xyz = [path_xyz1(:,1:2)*1.5 - 200, path_xyz1(:,3)];
% axis equal
path_pre = []
% plot(path_xyz(:,1),path_xyz(:,2))
% hold on
for i=1:length(path_xyz)
    path_pre(i,:) = deg2rad(inv_kine(path_xyz(i,:)))
%     hrobot.plot([path(i,:)])
%     axis([-3 3 -3 3 0 3])
%     pause(0.1);
end
path_pre = [zeros(2,3); path_pre]
%% Plot 1: Sgolayfilt Angle
f1 =figure(1)
path_fit = sgolayfilt(path_pre,4,31)
% path_fit = [zeros(1,3); path_fit]
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
pathcoord = pathcoord
%% RNE - Inversed Dynamics
Q = hrobot.rne(path, pathd, pathdd);

%% New Controller
s = tf('s');
% KP = [183 50 1]; KD = [2 2 0.5]; KI = [0.01 1.9 0.1]
% KP = [183 150 100]; KD = [1 1 1]; KI = [0.1 0 0];
% KP = [700 300 100]; KD = [50 10 6]; KI = [0.001 0.1 0.1];
KP = [2.2 2.2 0]; KD = [0 2.0 0]; KI = [0 0 0];
% sys = (KI*s + KP + KD/s)/s;
% sys = (KI*s + KP + KD/s)/s;
sys = (KP+ KD/s)/s;
Tau_fw =[]
qreal1 = [0 0 0]
qreal2 = zeros(2,3)
qreal_mat = zeros(2,3)
for j=2:length(time)
    j
    if j == 2 q_error = [0.0224   -0.0583   -0.0428; 0.0224   -0.0583   -0.0428];
    end
        for i=1:length(KP)
            Tau_fw1= lsim(sys(i),q_error(1:j,i),time(1:j));
            Tau_fw(1:j,i) = Tau_fw1;
        end
            qreal1 = cumtrapz(cumtrapz(hrobot.accel(path(1:j,:),pathd(1:j,:),Q(1:j,:)+Tau_fw)));
            qreal2 = [qreal2; qreal1(end,:)];
            q_error = [q_error; path(j,:)];
%             hrobot.fdyn()
end
% for j=2:length(time)
%     j
%     if j == 2 q_error = [0.0224   -0.0583   -0.0428; 0.0224   -0.0583   -0.0428];
%     end
%         for i=1:length(KP)
%             Tau_fw1= lsim(sys(i),q_error(:,i),time(j-1:j));
%             Tau_fw(j,i) = Tau_fw1(end);
%         end
%             qreal1 = hrobot.accel(path(1:j,:),pathd(1:j,:),Q(1:j,:)+Tau_fw);
%             qreal2 = [qreal2; qreal1(end,:)];
%             q_error = [q_error(end,:); path(j,:)-qreal1(end,:)];
% end
%%
qreal2 = qreal2(2:end,:)
%% Control - Forward Dynamics
% KP=[1 200 100];
% KD=[1 3 3];
% % % KP = [1 1 1];
% % % KD = [1 1 1];
% qreal1=joint_control2(hrobot,path, pathd, pathdd, Q, KP, KD); 
%% Forward Kinematics
realcoord = []
% for i=1:20
for i=1:length(qreal2)
    T1 = double(double(hrobot.fkine(qreal2(i,:))));
%     realcoord(:,i)=[T(1,4) T(2,4) T(3,4)]';
    realcoord(:,i) = [T1(1,4) T1(2,4) T1(3,4)];
end
% T1 = hrobot.fkine(qreal2)
% realcoord = [T1(1,4) T1(2,4) T1(3,4)];
realcoord = realcoord'
%% Plot 2: Path Comparison
f2=figure(2)
f2.Name = "Path-Comparison"
plot3path(pathcoord)
hold on
plot3path(realcoord)
view(2) % view at top
% plot3path(path_xyz/1000)
legend(["Planning","Actual"],"Box","off",'Location','best')
axis equal
saveas(2,"2_path-compare.svg",'svg')
%% Plot 3: Each joint response
% f = figure(3)
% f.Position = [10 10 500 200]
% % for i = 1:1
% i =3
% %     subplot(size(path,2),1,i)
%     plot(time,path(:,i),'-x','DisplayName',["Planning q"+int2str(i)])
%     hold on
%     plot(time,qreal1(:,i),'-x','DisplayName',["Actual q"+int2str(i)])
%     l = legend; xlabel("Time"); ylabel(["q_"+int2str(i)+" (rad)"])
%     l.Location = "best"
% %     title(["KP = "+int2str(KP(i))+", KD = " + int2str(KD(i))])
%     title(sprintf('KP = %0.2f,  KD = %0.2f, KI = %0.2f',KP(i), KD(i),KI(i)))
%     hold off
% % end
% saveas(3,"3_plot_q.svg","svg")
%% Plot 3: Each joint response
f = figure(3)
f.Position = [10 10 500 600]
time = 1:length(path);
for i = 1:3
    subplot(size(path,2),1,i)
    plot(time,path(:,i),'-x','DisplayName',["Planning q"+int2str(i)])
    hold on
    plot(time,qreal2(:,i),'-x','DisplayName',["Actual q"+int2str(i)])
    l = legend; xlabel("Time"); ylabel(["q_"+int2str(i)+" (rad)"])
    l.Location = "best"
%     title(["KP = "+int2str(KP(i))+", KD = " + int2str(KD(i))])
    title(sprintf('KP = %0.2f,  KD = %0.2f',KP(i), KD(i)))
    hold off
end
saveas(3,"3_plot_q.svg","svg")

%% Plot 4: Plot Torque
f = figure(4)
f.Position = [10 10 500 600]
time = 1:length(Q);
for i = 1:3
    subplot(size(Q,2),1,i)
    plot(time,Q(:,i),'-x','DisplayName',["Q Tau"+int2str(i)])
    hold on
    plot(time,Tau_fw(:,i),'-x','DisplayName',["fw Tau"+int2str(i)])
    l = legend; xlabel("Time"); ylabel(["q_"+int2str(i)+" (rad)"])
    l.Location = "best"
%     title(["KP = "+int2str(KP(i))+", KD = " + int2str(KD(i))])
    title(sprintf('KP = %0.2f,  KD = %0.2f',KP(i), KD(i)))
    hold off
end
saveas(4,"4_plot_tau.svg","svg")
%%

figure
for i = 1:3
    subplot(size(q_error,2),1,i)
    plot(q_error(:,i))
end
%% Plot Animate
% figure(4)
% plot3path(realcoord)
% hold on
% plot3path(pathcoord)
% % plot3path(path_xyz/1000)
% legend(["realcoord","pathcoord"],"Box","off",'Location','best')
% axis equal
% axis([-3 3 -3 3 0 3])
% grid on
% for i=1:length(path_xyz)
% %     path(i,:) = deg2rad(inv_kine(path_xyz(i,:)))
%     hrobot.plot([qreal1(i,:)])
%         pause(0.1);
% end
%%
% qreal1=joint_control(hrobot,path, pathd, pathdd, Q, KP, KD); 

%% Def Function

% function  E=joint_control(hrobot,q,qd,qdd,Tau,KP,KD)
% 
%     T=linspace(1,length(q),length(q))
%     
% %     KD=[2 4 4];
% %     KD=[2 5 1];
% %     KD=[200 400 400];
% %     KP=[1 10 10];
%      
%      CF=1;
%      R=0.003;
%      J=0.01;
%      B=0;
%      
%      %y(1)=x(2)
%     %  y(2)=-CF*Kp(el)/(R*J)*x(1)+ (R*B +CF^2 +CF*KD(el))/ (R*J)*x(2)...
%     %      +CF*Kp(el)*Th/(R*J) + CF*KP(el)/(R*J)*Thd+ D/J;
%          
%     
%  for i=1:length(q)
% % for i=1:5
% %      disp(i);
%     i
%     for j=1:3
% %      f = @(t,x)[x(2);-CF*KP(j)/(R*J)*x(1) - (R*B +CF^2 +CF*KD(j))/ (R*J)*x(2) + CF*KP(j)*q(i,j)/(R*J) + CF*KP(j)/(R*J)*qd(i,j) - Tau(i,j)/J]; 
%      f = @(t,x)[x(2);-KP(j)/(R*J)*x(1) - (R*B+KD(j))/ (R*J)*x(2) + KP(j)*q(i,j)/(R*J) + KP(j)/(R*J)*qd(i,j) - Tau(i,j)/J]; 
%        if (i==1)
% %          test_x1 = [ q(length(q),j);qd(length(q),j)]
%          [t,x] = ode45(f,[T(i) T(i+1)],[ q(length(q),j);qd(length(q),j)]);
%          last=length(t);
%          E(i,j)=x(last,1);
%        end
%        if(i<length(q) && i~=1)
%          [t,x]=ode45(f,[T(i) T(i+1)],[ q(i-1,j);qd(i-1,j)]);
%          last=length(t);
%          E(i,j)=x(last,1);
%        end
%        if(i==length(q))
%          E(length(q),j)=E(1,j);
%        end
%     end
% 
%  end
% end

function plot3path(path_xyz)
    plot3(path_xyz(:,1),path_xyz(:,2),path_xyz(:,3),'-x')
end