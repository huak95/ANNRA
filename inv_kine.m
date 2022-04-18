%% Def Inverse Kinematics Function
function [angle_mat_final,err] = inv_kine(loc)
    loc = loc*1000;
    d = ([680 0 150]);
    a = ([200 890 1020]);
    
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
%                 fprintf('%.2f ',angle_mat(i))
            end
        end
        fprintf(']\n')
    else
%         fprintf('[angle_mat (deg) ] = [%.2f %.2f %.2f]\n',angle_mat)
        err = 0;
    end
    angle_mat_final = angle_mat;
end


