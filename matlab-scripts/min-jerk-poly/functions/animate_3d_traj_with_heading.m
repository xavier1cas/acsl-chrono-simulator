%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2025 Giri M. Kumar, Andrea L'Afflitto. All rights reserved.                                                        
%                                                                             
% Redistribution and use in source and binary forms, with or without          
% modification, are permitted provided that the following conditions are met: 
%                                                                             
% 1. Redistributions of source code must retain the above copyright notice,   
%    this list of conditions and the following disclaimer.                    
%                                                                             
% 2. Redistributions in binary form must reproduce the above copyright        
%    notice, this list of conditions and the following disclaimer in the      
%    documentation and/or other materials provided with the distribution.     
%                                                                             
% 3. Neither the name of the copyright holder nor the names of its            
%    contributors may be used to endorse or promote products derived from     
%    this software without specific prior written permission.                 
%                                                                             
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
% AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE   
% IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE  
% ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE   
% LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR         
% CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF        
% SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS    
% INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN     
% CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)     
% ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE  
% POSSIBILITY OF SUCH DAMAGE.                                                 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% File:       animate_3d_traj_with_heading.m                                                   
% Author:     Giri Mugundan Kumar
% Date:       June 23, 2025
% For info:   Andrea L'Afflitto                                               
%             a.lafflitto@vt.edu                                              
%                                                                             
% Description: This function plots generated trajectory and waypoints with 
%              heading data.
%
% GitHub: https://github.com/girimugundankumar/qrbp-unwinding-constraints.git   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [] = animate_3d_traj_with_heading(t,x,y,z,yaw,yd,ydd,waypoints)
%ANIMATE_3D_TRAJ_WITH_HEADING Animates 3D trajectory with NED frames and heading plots
%   Creates an animation showing a static reference frame I and a moving frame J
%   that follows the trajectory. The right side shows yaw angle and its derivatives.
%
%   Inputs:
%       t           - time vector (1D array)
%       x,y,z       - position components (1D arrays)
%       yaw         - yaw angle (heading) (1D array) [radians]
%       yd          - yaw rate (yaw_dot) (1D array) [rad/s]
%       ydd         - yaw acceleration (yaw_ddot) (极1D array) [rad/s²]
%       waypoints   - waypoint matrix (3xN) where rows are [x; y; z] coordinates

    % Layout parameters
    fig_width = 1200;
    fig_height = 600;
    ax1_pos = [0.05, 0.1, 0.45, 0.85];        % 3D plot: left half
    control_panel_pos = [0.05, -0.01, 0.45, 0.08]; % Controls: bottom of 3D plot
    ax2_pos = [0.55, 0.72, 0.4, 0.23];         % Yaw plot
    ax3_pos = [0.55, 0.39, 0.4, 0.23];         % Yaw rate plot
    ax4_pos = [0.55, 0.10, 0.4, 0.18];         % Yaw acceleration plot

    % Create figure
    fig = figure('Color', 'white', 'Position', [100 100 fig_width fig_height]);

    % Helper function for safe axis limits
    function lims = safe_limits(data)
        if isempty(data)
            lims = [0, 1];
        else
            min_val = min(data);
            max_val = max(data);
            if min_val == max_val
                % Handle constant data
                lims = [min_val-0.1, min_val+0.1];
            else
                % Add 10% margin
                margin = 0.1 * (max_val - min_val);
                lims = [min_val - margin, max_val + margin];
            end
        end
    end

    % Main 3D trajectory plot
    ax1 = axes('Parent', fig, 'Position', ax1_pos);
    hold(ax1, 'on');
    
    % Plot waypoints (red circles with red fill)
    waypointPlot = plot3(ax1, waypoints(1,:), waypoints(2,:), waypoints(3,:), 'or', ...
          'MarkerSize', 10, 'MarkerFaceColor', 'r', 'DisplayName', 'Waypoints');
    
    % Plot static reference frame I with LaTeX interpreter
    frame_size = 0.5;
    quiver3(ax1, 0, 0, 0, frame_size, 0, 0, 'r', 'LineWidth', 3, 'MaxHeadSize', 0.3, 'DisplayName', '');
    quiver3(ax1, 0, 0, 0, 0, frame_size, 0, 'g', 'LineWidth', 3, 'MaxHeadSize', 0.3, 'DisplayName', '');
    quiver3(ax1, 0, 0, 0, 0, 0, frame_size, 'b', 'LineWidth', 3, 'MaxHeadSize', 0.3, 'DisplayName', '');
    text(ax1, frame_size*1.1, 0, 0, '$\mathbf{I}$', 'Interpreter', 'latex', ...
        'FontSize', 14, 'FontWeight', 'bold');
    
    % Initialize animated trajectory and moving frame J
    trajPlot = plot3(ax1, NaN, NaN, NaN, 'k-', 'LineWidth', 2, 'DisplayName', 'Trajectory');
    
    % Moving frame J with LaTeX interpreter
    frameJ_x = quiver3(ax1, NaN, NaN, NaN, NaN, NaN, NaN, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.4, 'DisplayName', '');
    frameJ_y = quiver3(ax1, NaN, NaN, NaN, NaN, NaN, NaN, 'g', 'LineWidth', 2, 'MaxHeadSize', 0.4, 'DisplayName', '');
    frameJ_z = quiver3(ax1, NaN, NaN, NaN, NaN, NaN, NaN, 'b', 'LineWidth', 2, 'MaxHeadSize', 0.4, 'DisplayName', '');
    frameJ_text = text(ax1, NaN, NaN, NaN, '$\mathbf{J}$', 'Interpreter', 'latex', ...
        'FontSize', 12, 'FontWeight', 'bold');
    
    % Axis settings (flipped Z-axis)
    xlabel(ax1, 'X', 'FontWeight', 'bold')
    ylabel(ax1, 'Y', 'FontWeight', 'bold')
    zlabel(ax1, 'Z', 'FontWeight', 'bold')
    ax1.YDir = 'reverse';  % Flip Y-axis
    ax1.ZDir = 'reverse';  % Flip Z-axis (negative Z is up)
    axis(ax1, 'equal')
    grid(ax1, 'on')
    
    % Set fixed axis limits for 3D plot
    all_x = [x(:); waypoints(1,:)'];
    all_y = [y(:); waypoints(2,:)'];
    all_z = [z(:); waypoints(3,:)'];
    axis_margin = 0.2;
    xlim(ax1, [min(all_x)-axis_margin, max(all_x)+axis_margin]);
    ylim(ax1, [min(all_y)-axis_margin, max(all_y)+axis_margin]);
    zlim(ax1, [min(all_z)-axis_margin, max(all_z)+axis_margin]);
    
    view(ax1, 3);
    light(ax1, 'Position', [1 1 1], 'Style', 'infinite');
    lighting(ax1, 'gouraud')
    material(ax1, 'dull')
    ax1.BoxStyle = 'full';
    ax1.DataAspectRatioMode = 'manual';
    ax1.DataAspectRatio = [1 1 1];
    titleHandle = title(ax1, 'NED Trajectory with Frames, t=0.00, speed=1.0x', 'FontSize', 14);
    legend(ax1, [waypointPlot, trajPlot], {'Waypoints', 'Trajectory'}, 'Location', 'best', 'FontSize', 10)

    % Controls panel
    controlPanel = uipanel(fig, 'Title', 'Controls', 'TitlePosition', 'centertop', ...
        'BackgroundColor', [0.94 0.94 0.94], 'Position', control_panel_pos);

    % Button layout (5 buttons, centered)
    panelWidth = fig_width * control_panel_pos(3);
    buttonWidth = 80;
    buttonHeight = 30;
    numButtons = 5;
    totalButtonWidth = numButtons * buttonWidth;
    spacing = (panelWidth - totalButtonWidth) / (numButtons + 1);

    btnStart = uicontrol(controlPanel, 'Style', 'pushbutton', 'String', 'Start', ...
        'Position', [spacing, 5, buttonWidth, buttonHeight]);
    btnPause = uicontrol(controlPanel, 'Style', 'togglebutton', 'String', 'Pause', ...
        'Position', [spacing*2 + buttonWidth, 5, buttonWidth, buttonHeight]);
    btnStop = uicontrol(controlPanel, 'Style', 'pushbutton', 'String', 'Stop', ...
        'Position', [spacing*3 + buttonWidth*2, 5, buttonWidth, buttonHeight]);
    btnSlow = uicontrol(controlPanel, 'Style', 'pushbutton', 'String', 'Slow Down', ...
        'Position', [spacing*4 + buttonWidth*3, 5, buttonWidth, buttonHeight]);
    btnFast = uicontrol(controlPanel, 'Style', 'pushbutton', 'String', 'Speed Up', ...
        'Position', [spacing*5 + buttonWidth*4, 5, buttonWidth, buttonHeight]);

    % Create yaw plots with safe limits
    ax2 = axes('Parent', fig, 'Position', ax2_pos); hold(ax2, 'on');
    yawLine = plot(ax2, NaN, NaN, 'k', 'LineWidth', 2);
    ylabel(ax2, 'Yaw [rad]', 'FontWeight', 'bold');
    grid(ax2, 'on');
    xlim(ax2, safe_limits(t));
    ylim(ax2, safe_limits(yaw));

    ax3 = axes('Parent', fig, 'Position', ax3_pos); hold(ax3, 'on');
    yawDotLine = plot(ax3, NaN, NaN, 'k', 'LineWidth', 2);
    ylabel(ax3, 'Yaw Rate [rad/s]', 'FontWeight', 'bold');
    grid(ax3, 'on');
    xlim(ax3, safe_limits(t));
    ylim(ax3, safe_limits(yd));

    ax4 = axes('Parent', fig, 'Position', ax4_pos); hold(ax4, 'on');
    yawDDotLine = plot(ax4, NaN, NaN, 'k', 'LineWidth', 2);
    ylabel(ax4, 'Yaw Accel [rad/s²]', 'FontWeight', 'bold');
    xlabel(ax4, 'Time (s)', 'FontWeight', 'bold');
    grid(ax4, 'on');
    xlim(ax4, safe_limits(t));
    ylim(ax4, safe_limits(ydd));

    % Link x-axes of yaw plots
    linkaxes([ax2, ax3, ax4], 'x');
    
    % Single legend below yaw plots
    legend_padding = 0.025;
    legend_height = 0.05;
    legend_pos = [ax4_pos(1), ax4_pos(2)-legend_height-legend_padding, ax4_pos(3), legend_height];
    axLegend = axes('Parent', fig, 'Position', legend_pos, 'Visible', 'off');
    legend(axLegend, yawLine, {'Yaw Data'}, 'Orientation', 'horizontal', 'Location', 'southoutside', 'Box', 'off');
    set(findobj(gcf,'Type','Legend'),'FontWeight','bold','FontSize',11,'Color','none');

    % Animation control variables
    isRunning = false;
    isPaused = false;
    currentIndex = 1;
    numPoints = length(t);
    animationStartTime = [];
    trajectoryStartTime = t(1);
    speedFactor = 1.0;
    currentTrajectoryTime = trajectoryStartTime;

    % Function to update moving frame J orientation and position
    function updateMovingFrame(pos_x, pos_y, pos_z, yaw_angle)
        frame_scale = 0.3;
        cos_yaw = cos(yaw_angle);
        sin_yaw = sin(yaw_angle);
        x_axis = [cos_yaw; sin_yaw; 0] * frame_scale;
        y_axis = [-sin_yaw; cos_yaw; 0] * frame_scale;
        z_axis = [0; 0; 1] * frame_scale;
        set(frameJ_x, 'XData', pos_x, 'YData', pos_y, 'ZData', pos_z, ...
                      'UData', x_axis(1), 'VData', x_axis(2), 'WData', x_axis(3));
        set(frameJ_y, 'XData', pos_x, 'YData', pos_y, 'ZData', pos_z, ...
                      'UData', y_axis(1), 'VData', y_axis(2), 'WData', y_axis(3));
        set(frameJ_z, 'XData', pos_x, 'YData', pos_y, 'ZData', pos_z, ...
                      'UData', z_axis(1), 'VData', z_axis(2), 'WData', z_axis(3));
        set(frameJ_text, 'Position', [pos_x + frame_scale*1.2, pos_y, pos_z]);
    end

    % Animation update function
    function updatePlot()
        if ~isRunning || isPaused
            return;
        end
        if isempty(animationStartTime)
            animationStartTime = tic;
        end
        elapsedRealTime = toc(animationStartTime);
        currentTrajectoryTime = trajectoryStartTime + (elapsedRealTime * speedFactor);
        validIndices = find(t <= currentTrajectoryTime);
        if ~isempty(validIndices)
            newIndex = validIndices(end);
            if newIndex ~= currentIndex
                currentIndex = newIndex;
                set(trajPlot, 'XData', x(1:currentIndex), 'YData', y(1:currentIndex), 'ZData', z(1:currentIndex));
                updateMovingFrame(x(currentIndex), y(currentIndex), z(currentIndex), yaw(currentIndex));
                set(yawLine, 'XData', t(1:currentIndex), 'YData', yaw(1:currentIndex));
                set(yawDotLine, 'XData', t(1:currentIndex), 'YData', yd(1:currentIndex));
                set(yawDDotLine, 'XData', t(1:currentIndex), 'YData', ydd(1:currentIndex));
                drawnow;
            end
        end
        set(titleHandle, 'String', sprintf('NED Trajectory with Frames, t=%.2f, speed=%.2fx', ...
            currentTrajectoryTime, speedFactor));
        if currentTrajectoryTime >= t(end)
            isRunning = false;
            set(btnStart, 'Enable', 'on');
        end
    end

    function timerCallback(~,~)
        updatePlot();
    end

    animationTimer = timer('ExecutionMode', 'fixedRate', 'Period', 0.01, ...
                           'TimerFcn', @timerCallback);

    btnStart.Callback = @(src,event) startAnimation();
    btnPause.Callback = @(src,event) pauseAnimation();
    btnStop.Callback = @(src,event) stopAnimation();
    btnSlow.Callback = @(src,event) slowDown();
    btnFast.Callback = @(src,event) speedUp();

    function startAnimation()
        if ~isRunning
            isRunning = true;
            isPaused = false;
            set(btnPause, 'Value', 0);
            set(btnStart, 'Enable', 'off');
            trajectoryStartTime = currentTrajectoryTime;
            animationStartTime = tic;
            start(animationTimer);
        end
    end

    function pauseAnimation()
        isPaused = get(btnPause, 'Value');
    end

    function stopAnimation()
        isRunning = false;
        isPaused = false;
        currentIndex = 1;
        currentTrajectoryTime = t(1);
        trajectoryStartTime = t(1);
        animationStartTime = [];
        set(trajPlot, 'XData', NaN, 'YData', NaN, 'ZData', NaN);
        set([frameJ_x, frameJ_y, frameJ_z], 'XData', NaN, 'YData', NaN, 'ZData', NaN, ...
                                            'UData', NaN, 'VData', NaN, 'WData', NaN);
        set(frameJ_text, 'Position', [NaN, NaN, NaN]);
        set([yawLine, yawDotLine, yawDDotLine], 'XData', NaN, 'YData', NaN);
        set(titleHandle, 'String', sprintf('NED Trajectory with Frames, t=%.2f, speed=%.2fx', ...
            currentTrajectoryTime, speedFactor));
        set(btnStart, 'Enable', 'on');
        set(btnPause, 'Value', 0);
        stop(animationTimer);
    end

    function slowDown()
        speedFactor = speedFactor / 1.5;
        if isRunning && ~isempty(animationStartTime)
            trajectoryStartTime = currentTrajectoryTime;
            animationStartTime = tic;
        end
        set(titleHandle, 'String', sprintf('NED Trajectory with Frames, t=%.2f, speed=%.2fx', ...
            currentTrajectoryTime, speedFactor));
    end

    function speedUp()
        speedFactor = speedFactor * 1.5;
        if isRunning && ~isempty(animationStartTime)
            trajectoryStartTime = currentTrajectoryTime;
            animationStartTime = tic;
        end
        set(titleHandle, 'String', sprintf('NED Trajectory with Frames, t=%.2f, speed=%.2fx', ...
            currentTrajectoryTime, speedFactor));
    end

    autoStartTimer = timer('StartDelay', 2, 'TimerFcn', @(~,~) startAnimation());
    start(autoStartTimer);

    fig.CloseRequestFcn = @(src,event) closeFigure();
    function closeFigure()
        if isvalid(animationTimer)
            stop(animationTimer);
            delete(animationTimer);
        end
        if isvalid(autoStartTimer)
            stop(autoStartTimer);
            delete(autoStartTimer);
        end
        delete(fig);
    end
end
