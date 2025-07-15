%% Go2 3D路径生成器快速测试 - 验证功能
clc; clear; close all;

fprintf('===== Go2 3D路径生成器快速测试 =====\n');
fprintf('使用较小参数快速验证功能\n\n');

%% 创建3D路径生成器
path_generator = Go2PathCluster3D();

% 快速测试参数 (大幅减少计算量)
path_generator.max_range = 2.0;        % 减少最大距离
path_generator.horizontal_angle = 30;   % ±30° 水平角度 (减少范围)
path_generator.vertical_angle = 20;     % ±20° 垂直角度 (减少范围)
path_generator.delta_angle_h = 20;      % 20° 水平分辨率 (减少密度)
path_generator.delta_angle_v = 20;      % 20° 垂直分辨率 (减少密度)
path_generator.scale = 0.5;            % 更小的衰减因子

fprintf('快速测试参数:\n');
fprintf('- 最大距离: %.1f m\n', path_generator.max_range);
fprintf('- 水平角度: ±%d° (步长%d°)\n', path_generator.horizontal_angle, path_generator.delta_angle_h);
fprintf('- 垂直角度: ±%d° (步长%d°)\n', path_generator.vertical_angle, path_generator.delta_angle_v);

% 预估计算量
h_steps = length(-path_generator.horizontal_angle : path_generator.delta_angle_h : path_generator.horizontal_angle);
v_steps = length(-path_generator.vertical_angle : path_generator.delta_angle_v : path_generator.vertical_angle);
fprintf('- 预估外层循环: %d × %d = %d 次\n', h_steps, v_steps, h_steps * v_steps);
fprintf('- 预估总路径数: ~%d 条\n\n', h_steps * v_steps * 20);

%% 生成3D路径
fprintf('开始快速生成3D路径...\n');
tic;
path_generator.generatePaths();
generation_time = toc;

%% 显示生成统计
path_generator.showStatistics();
fprintf('路径生成时间: %.2f 秒\n', generation_time);

if path_generator.pathID > 0
    fprintf('生成速度: %.0f 路径/秒\n', path_generator.pathID / generation_time);
    
    %% 可视化验证
    fprintf('\n可视化验证...\n');
    figure('Name', '快速测试 - 3D路径验证', 'Position', [100, 100, 1000, 600]);
    
    subplot(1, 2, 1);
    hold on; grid on; box on;
    title('3D路径全景');
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    
    % 显示所有路径点 (数量不多，可以全部显示)
    colors = lines(path_generator.groupID + 1);
    for g = 0:path_generator.groupID
        group_indices = path_generator.pathAll(5, :) == g;
        if any(group_indices)
            scatter3(path_generator.pathAll(1, group_indices), ...
                    path_generator.pathAll(2, group_indices), ...
                    path_generator.pathAll(3, group_indices), ...
                    2, colors(g+1, :), '.');
        end
    end
    axis equal; view(3);
    
    subplot(1, 2, 2);
    % 路径端点分布验证
    endpoints = path_generator.pathList;
    scatter3(endpoints(1, :), endpoints(2, :), endpoints(3, :), ...
             30, endpoints(5, :), 'filled');
    colorbar; title('路径端点分布'); 
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    axis equal; view(3);
    
    %% 快速保存验证
    output_dir = 'go2_paths_3d_quick_test';
    if ~exist(output_dir, 'dir')
        mkdir(output_dir);
    end
    
    fprintf('保存快速测试数据...\n');
    tic;
    path_generator.savePaths(output_dir);
    save_time = toc;
    fprintf('保存时间: %.2f 秒\n', save_time);
    
    % 检查文件大小
    files = {'go2_startPaths_3d.ply', 'go2_paths_3d.ply', 'go2_pathList_3d.ply'};
    fprintf('\n生成的文件:\n');
    for i = 1:length(files)
        filepath = fullfile(output_dir, files{i});
        if exist(filepath, 'file')
            info = dir(filepath);
            fprintf('- %s: %.1f KB\n', files{i}, info.bytes/1024);
        end
    end
    
    fprintf('\n✅ 快速测试成功! 功能验证通过!\n');
    fprintf('\n如果需要完整版本，请运行以下参数:\n');
    fprintf('- horizontal_angle = 60, vertical_angle = 30\n');
    fprintf('- delta_angle_h = 10, delta_angle_v = 10\n');
    fprintf('- 预估生成时间: %.1f 分钟\n', generation_time * (60/30) * (30/20) * (10/20)^2 / 60);
    
else
    fprintf('❌ 路径生成失败，请检查代码\n');
end

fprintf('\n测试完成!\n'); 