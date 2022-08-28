clc; clear;
number = 1;
flag = [false, false, false, false, false];
for i = 1:number
    flag(i) = true;
end
if flag(1)
    uav1 = readtable('plot_uav1.csv'); end
if flag(2)
    uav2 = readtable('plot_uav2.csv'); end
if flag(3)
    uav3 = readtable('plot_uav3.csv'); end
if flag(4)
    uav4 = readtable('plot_uav4.csv'); end
if flag(5)
    uav5 = readtable('plot_uav5.csv'); end

for i = 2:height(uav1)
    figure(1)
        if i ~= height(uav1)
            %%%% uav 1
            if flag(1)
                plot3(uav1.x(2:i), uav1.y(2:i), uav1.z(2:i), '-b', LineWidth=3.0);hold on;
                quiver3(uav1.x(i), uav1.y(i), uav1.z(i), ...
                    cos(uav1.yaw(i)), sin(uav1.yaw(i)), 0, 0, 'r', LineWidth=1.0);hold off;
            end
            %%%% uav 2
            if flag(2)
                plot3(uav2.x(2:i), uav2.y(2:i), uav2.z(2:i), '-g', LineWidth=1.0);hold on;
                quiver3(uav2.x(i), uav2.y(i), uav2.z(i), ...
                    cos(uav2.yaw(i)), sin(uav2.yaw(i)), 0, 0, 'g', LineWidth=1.0);hold off;
            end
            %%%% uav 3
            if flag(3)
                plot3(uav3.x(2:i), uav3.y(2:i), uav3.z(2:i), '-g', LineWidth=1.0);hold on;
                quiver3(uav3.x(i), uav3.y(i), uav3.z(i), ...
                    cos(uav3.yaw(i)), sin(uav3.yaw(i)), 0, 0, 'g', LineWidth=1.0);hold off;
            end
            %%%% uav 4
            if flag(4)
                plot3(uav4.x(2:i), uav4.y(2:i), uav4.z(2:i), '-g', LineWidth=1.0);hold on;
                quiver3(uav4.x(i), uav4.y(i), uav4.z(i), ...
                    cos(uav4.yaw(i)), sin(uav4.yaw(i)), 0, 0, 'g', LineWidth=1.0);hold off;
            end
            %%%% uav 5
            if flag(5)
                plot3(uav5.x(2:i), uav5.y(2:i), uav5.z(2:i), '-g', LineWidth=1.0);hold on;
                quiver3(uav5.x(i), uav5.y(i), uav5.z(i), ...
                    cos(uav5.yaw(i)), sin(uav5.yaw(i)), 0, 0, 'g', LineWidth=1.0);hold off;
            end
        else
            %%%% uav 1
            if flag(1)
                plot3(uav1.x, uav1.y, uav1.z, '-b');hold on;
                quiver3(uav1.x, uav1.y, uav1.z, ...
                    cos(uav1.yaw), sin(uav1.yaw), 0*uav1.yaw, 0.5, 'r', LineWidth=1.0);hold off;
            end
            %%%% uav 2
            if flag(2)
                plot3(uav2.x, uav2.y, uav2.z, '-b');hold on;
                quiver3(uav2.x, uav2.y, uav2.z, ...
                    cos(uav2.yaw), sin(uav2.yaw), 0*uav2.yaw, 0.5, 'r', LineWidth=1.0);hold off;
            end
            %%%% uav 3
            if flag(3)
                plot3(uav3.x, uav3.y, uav3.z, '-b');hold on;
                quiver3(uav3.x, uav3.y, uav3.z, ...
                    cos(uav3.yaw), sin(uav3.yaw), 0*uav3.yaw, 0.5, 'r', LineWidth=1.0);hold off;
            end
            %%%% uav 4
            if flag(4)
                plot3(uav4.x, uav4.y, uav4.z, '-b');hold on;
                quiver3(uav4.x, uav4.y, uav4.z, ...
                    cos(uav4.yaw), sin(uav4.yaw), 0*uav4.yaw, 0.5, 'r', LineWidth=1.0);hold off;
            end
            %%%% uav 5
            if flag(5)
                plot3(uav5.x, uav5.y, uav5.z, '-b');hold on;
                quiver3(uav5.x, uav5.y, uav5.z, ...
                    cos(uav5.yaw), sin(uav5.yaw), 0*uav5.yaw, 0.5, 'r', LineWidth=1.0);hold off;
            end
        end
        grid on;
        axis([min(uav1.x)-0.5 max(uav1.x)+0.5 min(uav1.y)-0.5, max(uav1.y)+0.5, 0.0 max(uav1.z)+0.5]);
%         pause(0.1)
        drawnow
end
            
