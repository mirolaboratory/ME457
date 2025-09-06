% Create a serial object
Port = 'COM4'; % Replace with your port name
baudRate = 115200; % Set your baud rate
s = serialport(Port, baudRate);

% Prepare for real-time plotting
figure;
hData = animatedline('Color', 'r', 'LineWidth', 2);
pl = gca;
pl.YGrid = 'on';
pl.XGrid = 'on';
xlabel('Sample Number');
ylabel('Sensor Value');
title('Real-time Serial Data Plot');

XLim = 10; % Show last 10 seconds of data
% Set axis limits (adjust as needed)
pl.YLim = [0, 5]; % Adjust Y-axis limits based on your data range

% Matrix for storing all data (columns: timestamp, value)
allData = [];
dataIndex = 1;

% Buffer for storing data used for plot (size of buffer is 50)
plotBuffer = nan(50, 1); % Preallocate buffer with NaNs for single value

% Read data in a loop (you can modify the loop condition as needed)
try
    % Reduce latency by configuring buffer sizes
    configureTerminator(s, 'LF');
    flush(s);
    
    while true
        % Check for a key press to stop the loop
        if get(gcf, 'CurrentCharacter') == 'q'
            disp('Stopping the data reading loop...');
            break;
        end
        
        % Read a line of data
        if s.NumBytesAvailable > 0
            data = readline(s);
            
            % Split data into components
            splitData = strsplit(data, ',');
            
            if length(splitData) == 2  % Expecting timestamp,value
                % Extract timestamp and single value
                timestamp = str2double(splitData{1})/1000;
                value = str2double(splitData{2});
                
                % Store the data in allData matrix if valid
                if ~isnan(timestamp) && ~isnan(value)
                    allData = [allData; timestamp, value];
                    
                    % Update the plot buffer by shifting and appending new data
                    plotBuffer(1:end-1) = plotBuffer(2:end); % Shift data up
                    plotBuffer(end) = value; % Add new data to the end
                    
                    % Add point to animated line
                    addpoints(hData, timestamp, value);
                    
                    % Keep X-axis limit increasing to show scrolling effect
                    pl.XLim = [max(0, timestamp - XLim), timestamp];
                    
                    % Use 'limitrate' to improve performance and reduce lag
                    drawnow limitrate;
                    
                    % Clear input buffer periodically
                    if mod(dataIndex, 20) == 0
                        flush(s);
                    end
                    
                    dataIndex = dataIndex + 1;
                    
                    % Display current value every 50 samples
                    if mod(dataIndex, 50) == 0
                        fprintf('Sample %d: Time = %.3f\n', dataIndex, timestamp);
                    end
                end
            else
                % Handle different data formats or display warning
                fprintf('Warning: Expected 2 values, got %d: %s\n', length(splitData), data);
            end
        end
        
        % Small pause for stability
        pause(0.01);
    end
    
catch exception
    % Handle errors and clean up
    disp('An error occurred:');
    disp(exception.message);
end

% Display summary
if ~isempty(allData)
    fprintf('\n=== Data Collection Summary ===\n');
    fprintf('Total samples collected: %d\n', size(allData, 1));
    fprintf('Value range: %.3f to %.3f\n', min(allData(:,2)), max(allData(:,2)));
    fprintf('Average value: %.3f\n', mean(allData(:,2)));
    
    % Ask if user wants to save data
    saveChoice = input('Save data to file? (y/n): ', 's');
    if strcmpi(saveChoice, 'y')
        timestamp_str = datestr(now, 'yyyy-mm-dd_HH-MM-SS');
        filename = sprintf('serial_data_%s.csv', timestamp_str);
        
        % Save with headers
        csvwrite(filename, allData);
        fprintf('Data saved to: %s\n', filename);
    end
end

% Close the serial port
clear s;
disp('Serial port closed.');
