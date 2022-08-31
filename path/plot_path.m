clc; clear;
number = 5;
flag = [false, false, false, false, false];
for i = 1:number
    flag(i) = true;
end
file_path = 'outdoor/path3/';
if flag(1)
    uav1 = readtable(strcat(file_path,'plot_uav1.csv')); end
if flag(2)
    uav2 = readtable(strcat(file_path,'plot_uav2.csv')); end
if flag(3)
    uav3 = readtable(strcat(file_path,'plot_uav3.csv')); end
if flag(4)
    uav4 = readtable(strcat(file_path,'plot_uav4.csv')); end
if flag(5)
    uav5 = readtable(strcat(file_path,'plot_uav5.csv')); end

for i = 2:height(uav1)
    figure(1)
        if i ~= height(uav1)
            %%%% uav 1
            if flag(1)
                plot3(uav1.x(2:i), uav1.y(2:i), uav1.z(2:i), '-b', LineWidth=3.0);hold on;
                quiver3(uav1.x(i), uav1.y(i), uav1.z(i), ...
                    cos(uav1.yaw(i)), sin(uav1.yaw(i)), 0, 0, 'r', LineWidth=1.5);
                quiver3(uav1.x(i), uav1.y(i), uav1.z(i), ...
                    0, 0, 1, 0.3, 'g', LineWidth=3.0);hold off;
            end
            %%%% uav 2
            if flag(2)
                hold on;
                plot3(uav2.x(2:i), uav2.y(2:i), uav2.z(2:i), '-b', LineWidth=3.0);hold on;
                quiver3(uav2.x(i), uav2.y(i), uav2.z(i), ...
                    cos(uav2.yaw(i)), sin(uav2.yaw(i)), 0, 0, 'r', LineWidth=1.5);
                quiver3(uav2.x(i), uav2.y(i), uav2.z(i), ...
                    0, 0, 1, 0.3, 'g', LineWidth=3.0);hold off;
            end
            %%%% uav 3
            if flag(3)
                hold on;
                plot3(uav3.x(2:i), uav3.y(2:i), uav3.z(2:i), '-b', LineWidth=3.0);hold on;
                quiver3(uav3.x(i), uav3.y(i), uav3.z(i), ...
                    cos(uav3.yaw(i)), sin(uav3.yaw(i)), 0, 0, 'r', LineWidth=1.5);
                quiver3(uav3.x(i), uav3.y(i), uav3.z(i), ...
                    0, 0, 1, 0.3, 'g', LineWidth=3.0);hold off;
            end
            %%%% uav 4
            if flag(4)
                hold on;
                plot3(uav4.x(2:i), uav4.y(2:i), uav4.z(2:i), '-b', LineWidth=3.0);hold on;
                quiver3(uav4.x(i), uav4.y(i), uav4.z(i), ...
                    cos(uav4.yaw(i)), sin(uav4.yaw(i)), 0, 0, 'r', LineWidth=1.5);
                quiver3(uav4.x(i), uav4.y(i), uav4.z(i), ...
                    0, 0, 1, 0.3, 'g', LineWidth=3.0);hold off;
            end
            %%%% uav 5
            if flag(5)
                hold on;
                plot3(uav5.x(2:i), uav5.y(2:i), uav5.z(2:i), '-b', LineWidth=3.0);hold on;
                quiver3(uav5.x(i), uav5.y(i), uav5.z(i), ...
                    cos(uav5.yaw(i)), sin(uav5.yaw(i)), 0, 0, 'r', LineWidth=1.5);
                quiver3(uav5.x(i), uav5.y(i), uav5.z(i), ...
                    0, 0, 1, 0.3, 'g', LineWidth=3.0);hold off;
            end
        else
            %%%% uav 1
            if flag(1)
                plot3(uav1.x, uav1.y, uav1.z, '-b');hold on;
                quiver3(uav1.x, uav1.y, uav1.z, ...
                    cos(uav1.yaw), sin(uav1.yaw), 0*uav1.yaw, 1.0, 'r', LineWidth=1.0);hold off;
            end
            %%%% uav 2
            if flag(2)
                hold on;
                plot3(uav2.x, uav2.y, uav2.z, '-b');hold on;
                quiver3(uav2.x, uav2.y, uav2.z, ...
                    cos(uav2.yaw), sin(uav2.yaw), 0*uav2.yaw, 1.0, 'r', LineWidth=1.0);hold off;
            end
            %%%% uav 3
            if flag(3)
                hold on;
                plot3(uav3.x, uav3.y, uav3.z, '-b');hold on;
                quiver3(uav3.x, uav3.y, uav3.z, ...
                    cos(uav3.yaw), sin(uav3.yaw), 0*uav3.yaw, 1.0, 'r', LineWidth=1.0);hold off;
            end
            %%%% uav 4
            if flag(4)
                hold on;
                plot3(uav4.x, uav4.y, uav4.z, '-b');hold on;
                quiver3(uav4.x, uav4.y, uav4.z, ...
                    cos(uav4.yaw), sin(uav4.yaw), 0*uav4.yaw, 1.0, 'r', LineWidth=1.0);hold off;
            end
            %%%% uav 5
            if flag(5)
                hold on;
                plot3(uav5.x, uav5.y, uav5.z, '-b');hold on;
                quiver3(uav5.x, uav5.y, uav5.z, ...
                    cos(uav5.yaw), sin(uav5.yaw), 0*uav5.yaw, 1.0, 'r', LineWidth=1.0);hold off;
            end
        end
        grid on;
        if flag(5)
            x_m = [min([min(uav1.x), min(uav2.x), min(uav3.x), min(uav4.x), min(uav5.x)]), ...
                max([max(uav1.x), max(uav2.x), max(uav3.x), max(uav4.x), max(uav5.x)])];
            y_m = [min([min(uav1.y), min(uav2.y), min(uav3.y), min(uav4.y), min(uav5.y)]), ...
                max([max(uav1.y), max(uav2.y), max(uav3.y), max(uav4.y), max(uav5.y)])];
            z_m = [0.0, max([max(uav1.z), max(uav2.z), max(uav3.z), max(uav4.z), max(uav5.z)])];
        elseif flag(4)
            x_m = [min([min(uav1.x), min(uav2.x), min(uav3.x), min(uav4.x)]), ...
                max([max(uav1.x), max(uav2.x), max(uav3.x), max(uav4.x)])];
            y_m = [min([min(uav1.y), min(uav2.y), min(uav3.y), min(uav4.y)]), ...
                max([max(uav1.y), max(uav2.y), max(uav3.y), max(uav4.y)])];
            z_m = [0.0, max([max(uav1.z), max(uav2.z), max(uav3.z), max(uav4.z)])];
        elseif flag(3)
            x_m = [min([min(uav1.x), min(uav2.x), min(uav3.x)]), ...
                max([max(uav1.x), max(uav2.x), max(uav3.x)])];
            y_m = [min([min(uav1.y), min(uav2.y), min(uav3.y)]), ...
                max([max(uav1.y), max(uav2.y), max(uav3.y)])];
            z_m = [0.0, max([max(uav1.z), max(uav2.z), max(uav3.z)])];
        elseif flag(2)
            x_m = [min(min(uav1.x), min(uav2.x)), max(max(uav1.x), max(uav2.x))];
            y_m = [min(min(uav1.y), min(uav2.y)), max(max(uav1.y), max(uav2.y))];
            z_m = [0.0, max(max(uav1.z), max(uav2.z))];
        elseif flag(1)
            x_m = [min(uav1.x), max(uav1.x)];
            y_m = [min(uav1.y), max(uav1.y)];
            z_m = [0.0, max(uav1.z)];
        end
        axis([x_m(1)-1.0 x_m(2)+1.0 ...
                y_m(1)-1.0, y_m(2)+1.0, ...
                0.0 z_m(2)+1.0]);
        xlabel('X');ylabel('Y');zlabel('Z')
%         pause(0.1)
        drawnow
end
            
