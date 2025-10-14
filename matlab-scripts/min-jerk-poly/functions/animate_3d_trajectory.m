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
% File:       animate_3d_trajectory.m                                                   
% Author:     Giri Mugundan Kumar
% Date:       June 20, 2025
% For info:   Andrea L'Afflitto                                               
%             a.lafflitto@vt.edu                                              
%                                                                             
% Description: This function plots generated trajectory and waypoints.
%
% GitHub: https://github.com/girimugundankumar/qrbp-unwinding-constraints.git   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [] = animate_3d_trajectory(traj)
    % Layout parameters
    fig_width = 1200;
    fig_height = 600;
    ax1_pos = [0.05, 0.1, 0.45, 0.85];        % 3D plot: left half
    control_panel_pos = [0.05, -0.01, 0.45, 0.08]; % Controls: bottom of 3D plot
    ax2_pos = [0.55, 0.72, 0.4, 0.23];         % Position plot
    ax3_pos = [0.55, 0.39, 0.4, 0.23];        % Velocity plot
    ax4_pos = [0.55, 0.10, 0.4, 0.18];        % Acceleration plot

    % Create figure
    fig = figure('Color', 'white', 'Position', [100 100 fig_width fig_height]);

    % Main 3D trajectory plot
    ax1 = axes('Parent', fig, 'Position', ax1_pos);
    hold(ax1, 'on');
    plot3(ax1, traj.wpts(1,:), traj.wpts(2,:), traj.wpts(3,:), 'or', ...
          'MarkerSize', 10, 'MarkerFaceColor', 'r', 'DisplayName', 'Waypoints');
    trajPlot = plot3(ax1, NaN, NaN, NaN, '.k', 'MarkerSize', 8, 'DisplayName', 'Trajectory');
    xlabel(ax1, 'X', 'FontWeight', 'bold')
    ylabel(ax1, 'Y', 'FontWeight', 'bold')
    zlabel(ax1, 'Z', 'FontWeight', 'bold')
    ax1.YDir = 'reverse';
    ax1.ZDir = 'reverse';
    axis(ax1, 'equal')
    grid(ax1, 'on')
    view(ax1, 3);
    light(ax1, 'Position', [1 1 1], 'Style', 'infinite');
    lighting(ax1, 'gouraud')
    material(ax1, 'dull')
    ax1.BoxStyle = 'full';
    ax1.DataAspectRatioMode = 'manual';
    ax1.DataAspectRatio = [1 1 1];
    titleHandle = title(ax1, 'Minimum Jerk Trajectory, t=0.00, speed=1.0x', 'FontSize', 14);
    legend(ax1, 'Location', 'best', 'FontSize', 10)

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

    % Create component plots with axis-specific colors
    ax2 = axes('Parent', fig, 'Position', ax2_pos); hold(ax2, 'on');
    posXLine = plot(ax2, NaN, NaN, 'r', 'LineWidth', 1.5, 'DisplayName', 'X');
    posYLine = plot(ax2, NaN, NaN, 'g', 'LineWidth', 1.5, 'DisplayName', 'Y');
    posZLine = plot(ax2, NaN, NaN, 'b', 'LineWidth', 1.5, 'DisplayName', 'Z');
    posNormLine = plot(ax2, NaN, NaN, 'k', 'LineWidth', 2, 'DisplayName', 'Norm');
    ylabel(ax2, 'Position', 'FontWeight', 'bold');
    set(ax2, 'FontWeight', 'bold');
    grid(ax2, 'minor');
    xlim(ax2, [min(traj.tsamples), max(traj.tsamples)]);
    legend(ax2, 'off');

    ax3 = axes('Parent', fig, 'Position', ax3_pos); hold(ax3, 'on');
    velXLine = plot(ax3, NaN, NaN, 'r', 'LineWidth', 1.5, 'DisplayName', 'X');
    velYLine = plot(ax3, NaN, NaN, 'g', 'LineWidth', 1.5, 'DisplayName', 'Y');
    velZLine = plot(ax3, NaN, NaN, 'b', 'LineWidth', 1.5, 'DisplayName', 'Z');
    velNormLine = plot(ax3, NaN, NaN, 'k', 'LineWidth', 2, 'DisplayName', 'Norm');
    ylabel(ax3, 'Velocity', 'FontWeight', 'bold');
    set(ax3, 'FontWeight', 'bold');
    grid(ax3, 'minor');
    xlim(ax3, [min(traj.tsamples), max(traj.tsamples)]);
    legend(ax3, 'off');

    ax4 = axes('Parent', fig, 'Position', ax4_pos); hold(ax4, 'on');
    accXLine = plot(ax4, NaN, NaN, 'r', 'LineWidth', 1.5, 'DisplayName', 'X');
    accYLine = plot(ax4, NaN, NaN, 'g', 'LineWidth', 1.5, 'DisplayName', 'Y');
    accZLine = plot(ax4, NaN, NaN, 'b', 'LineWidth', 1.5, 'DisplayName', 'Z');
    accNormLine = plot(ax4, NaN, NaN, 'k', 'LineWidth', 2, 'DisplayName', 'Norm');
    ylabel(ax4, 'Acceleration', 'FontWeight', 'bold');
    xlabel(ax4, 'Time (s)', 'FontWeight', 'bold');
    set(ax4, 'FontWeight', 'bold');
    grid(ax4, 'minor');
    xlim(ax4, [min(traj.tsamples), max(traj.tsamples)]);
    legend(ax4, 'off');

    % Link x-axes for synchronized zooming
    linkaxes([ax2, ax3, ax4], 'x');

    % --- Add a single legend below the acceleration plot, aligned with left plots ---
    legend_padding = 0.025; % vertical padding
    legend_height = 0.05;   % height of the legend area
    legend_pos = [ax4_pos(1), ax4_pos(2)-legend_height-legend_padding, ax4_pos(3), legend_height];
    axLegend = axes('Parent', fig, 'Position', legend_pos, 'Visible', 'off');
    legend(axLegend, [accXLine, accYLine, accZLine, accNormLine], {'X','Y','Z','Norm'}, ...
        'Orientation', 'horizontal', 'Location', 'southoutside', 'Box', 'off');
    set(findobj(gcf,'Type','Legend'),'FontWeight','bold','FontSize',11,'Color','none');

    % Animation control variables
    isRunning = false;
    isPaused = false;
    currentIndex = 1;
    numPoints = length(traj.tsamples);
    animationStartTime = [];
    trajectoryStartTime = traj.tsamples(1);
    speedFactor = 1.0;
    currentTrajectoryTime = trajectoryStartTime;

    % --- Animation update function ---
    function updatePlot()
        if ~isRunning || isPaused
            return;
        end
        if isempty(animationStartTime)
            animationStartTime = tic;
        end
        elapsedRealTime = toc(animationStartTime);
        currentTrajectoryTime = trajectoryStartTime + (elapsedRealTime * speedFactor);
        validIndices = find(traj.tsamples <= currentTrajectoryTime);
        if ~isempty(validIndices)
            newIndex = validIndices(end);
            if newIndex ~= currentIndex
                currentIndex = newIndex;
                set(trajPlot, 'XData', traj.pos(1,1:currentIndex), ...
                              'YData', traj.pos(2,1:currentIndex), ...
                              'ZData', traj.pos(3,1:currentIndex));
                set(posXLine, 'XData', traj.tsamples(1:currentIndex), 'YData', traj.pos(1,1:currentIndex));
                set(posYLine, 'XData', traj.tsamples(1:currentIndex), 'YData', traj.pos(2,1:currentIndex));
                set(posZLine, 'XData', traj.tsamples(1:currentIndex), 'YData', traj.pos(3,1:currentIndex));
                set(posNormLine, 'XData', traj.tsamples(1:currentIndex), 'YData', vecnorm(traj.pos(:,1:currentIndex), 2, 1));
                set(velXLine, 'XData', traj.tsamples(1:currentIndex), 'YData', traj.vel(1,1:currentIndex));
                set(velYLine, 'XData', traj.tsamples(1:currentIndex), 'YData', traj.vel(2,1:currentIndex));
                set(velZLine, 'XData', traj.tsamples(1:currentIndex), 'YData', traj.vel(3,1:currentIndex));
                set(velNormLine, 'XData', traj.tsamples(1:currentIndex), 'YData', vecnorm(traj.vel(:,1:currentIndex), 2, 1));
                set(accXLine, 'XData', traj.tsamples(1:currentIndex), 'YData', traj.acc(1,1:currentIndex));
                set(accYLine, 'XData', traj.tsamples(1:currentIndex), 'YData', traj.acc(2,1:currentIndex));
                set(accZLine, 'XData', traj.tsamples(1:currentIndex), 'YData', traj.acc(3,1:currentIndex));
                set(accNormLine, 'XData', traj.tsamples(1:currentIndex), 'YData', vecnorm(traj.acc(:,1:currentIndex), 2, 1));
                drawnow;
            end
        end
        set(titleHandle, 'String', sprintf('Minimum Jerk Trajectory, t=%.2f, speed=%.2fx', ...
            currentTrajectoryTime, speedFactor));
        if currentTrajectoryTime >= traj.tsamples(end)
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
        currentTrajectoryTime = traj.tsamples(1);
        trajectoryStartTime = traj.tsamples(1);
        animationStartTime = [];
        set(trajPlot, 'XData', NaN, 'YData', NaN, 'ZData', NaN);
        set([posXLine, posYLine, posZLine, posNormLine], 'XData', NaN, 'YData', NaN);
        set([velXLine, velYLine, velZLine, velNormLine], 'XData', NaN, 'YData', NaN);
        set([accXLine, accYLine, accZLine, accNormLine], 'XData', NaN, 'YData', NaN);
        set(titleHandle, 'String', sprintf('Minimum Jerk Trajectory, t=%.2f, speed=%.2fx', ...
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
        set(titleHandle, 'String', sprintf('Minimum Jerk Trajectory, t=%.2f, speed=%.2fx', ...
            currentTrajectoryTime, speedFactor));
    end

    function speedUp()
        speedFactor = speedFactor * 1.5;
        if isRunning && ~isempty(animationStartTime)
            trajectoryStartTime = currentTrajectoryTime;
            animationStartTime = tic;
        end
        set(titleHandle, 'String', sprintf('Minimum Jerk Trajectory, t=%.2f, speed=%.2fx', ...
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
