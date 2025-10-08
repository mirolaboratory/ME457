clear all
close all

% Create a serial object
Port = 'COM5'; % Replace with your port name
baudRate = 115200;
s = serialport(Port, baudRate, 'Timeout', 10);

% Define buffer size (matches Arduino definition)
BUFFER_SIZE = 141;      % 1 marker + 5 timestamps + 10 samples
SYNC_MARKER = 255;      % 0xFF sync marker
NUM_TIMESTAMPS = 5;     % 5 timestamps in each buffer
NUM_SAMPLES = 10;       % 10 samples in each buffer

% Scale factors
ACCEL_SCALE = 16384.0;  % ±2g range
GYRO_SCALE = 131.0;     % ±250°/s range

% Prepare for real-time plotting
figure('Name', 'Real-Time IMU Data', 'NumberTitle', 'off');

subplot(2,1,1);
hAccelX = animatedline('Color', 'r', 'LineWidth', 1.5, 'DisplayName', 'Accel X');
hold on;
hAccelY = animatedline('Color', 'g', 'LineWidth', 1.5, 'DisplayName', 'Accel Y');
hAccelZ = animatedline('Color', 'b', 'LineWidth', 1.5, 'DisplayName', 'Accel Z');
hold off;
grid on;
xlabel('Sample Index');
ylabel('Acceleration (g)');
title('Accelerometer Data');
legend('show');
ylim([-2, 2]);

subplot(2,1,2);
hGyroX = animatedline('Color', 'r', 'LineWidth', 1.5, 'DisplayName', 'Gyro X');
hold on;
hGyroY = animatedline('Color', 'g', 'LineWidth', 1.5, 'DisplayName', 'Gyro Y');
hGyroZ = animatedline('Color', 'b', 'LineWidth', 1.5, 'DisplayName', 'Gyro Z');
hold off;
grid on;
xlabel('Sample Index');
ylabel('Angular Velocity (°/s)');
title('Gyroscope Data');
legend('show');
ylim([-250, 250]);

XLim = 200; % Show last 200 samples

% Data storage
allData = [];  % Columns: timestamp, AcX, AcY, AcZ, GyX, GyY, GyZ
sampleCount = 0;

% Read data in a loop
flush(s);
fprintf('Waiting for data... Press Ctrl+C to stop\n');

try
    while true
        % Check for key press to stop
        if ~isempty(get(gcf, 'CurrentCharacter')) && get(gcf, 'CurrentCharacter') == 'q'
            disp('Stopping...');
            break;
        end
        
        % Wait for enough bytes
        if s.NumBytesAvailable >= BUFFER_SIZE
            % Read buffer
            rawData = read(s, BUFFER_SIZE, 'uint8');
            
            % Check sync marker
            if rawData(1) ~= SYNC_MARKER
                warning('Sync marker not found (got 0x%02X), skipping', rawData(1));
                flush(s);
                continue;
            end
            
            % Parse buffer: [0xFF][T1][S1][S2][T2][S3][S4][T3][S5][S6][T4][S7][S8][T5][S9][S10]
            index = 2;  % Start after marker
            
            for tsNum = 1:NUM_TIMESTAMPS
                % Read timestamp (4 bytes, big-endian)
                timestamp = swapbytes(typecast(uint8(rawData(index:index+3)), 'uint32'));
                timestamp = double(timestamp) / 1000.0;  % Convert ms to seconds
                index = index + 4;
                
                % Read 2 samples that share this timestamp
                for sampleNum = 1:2
                    % Read Accelerometer (6 bytes, big-endian)
                    accelX = double(swapbytes(typecast(uint8(rawData(index:index+1)), 'int16'))) / ACCEL_SCALE;
                    accelY = double(swapbytes(typecast(uint8(rawData(index+2:index+3)), 'int16'))) / ACCEL_SCALE;
                    accelZ = double(swapbytes(typecast(uint8(rawData(index+4:index+5)), 'int16'))) / ACCEL_SCALE;
                    index = index + 6;
                    
                    % Read Gyroscope (6 bytes, big-endian)
                    gyroX = double(swapbytes(typecast(uint8(rawData(index:index+1)), 'int16'))) / GYRO_SCALE;
                    gyroY = double(swapbytes(typecast(uint8(rawData(index+2:index+3)), 'int16'))) / GYRO_SCALE;
                    gyroZ = double(swapbytes(typecast(uint8(rawData(index+4:index+5)), 'int16'))) / GYRO_SCALE;
                    index = index + 6;
                    
                    sampleCount = sampleCount + 1;
                    
                    % Store data: First sample gets timestamp, second gets NaN
                    if sampleNum == 1
                        allData = [allData; timestamp, accelX, accelY, accelZ, gyroX, gyroY, gyroZ];
                    else
                        allData = [allData; NaN, accelX, accelY, accelZ, gyroX, gyroY, gyroZ];
                    end
                    
                    % Update plots using sample index
                    subplot(2,1,1);
                    addpoints(hAccelX, sampleCount, accelX);
                    addpoints(hAccelY, sampleCount, accelY);
                    addpoints(hAccelZ, sampleCount, accelZ);
                    if sampleCount > XLim
                        xlim([sampleCount - XLim, sampleCount]);
                    else
                        xlim([0, XLim]);
                    end
                    
                    subplot(2,1,2);
                    addpoints(hGyroX, sampleCount, gyroX);
                    addpoints(hGyroY, sampleCount, gyroY);
                    addpoints(hGyroZ, sampleCount, gyroZ);
                    if sampleCount > XLim
                        xlim([sampleCount - XLim, sampleCount]);
                    else
                        xlim([0, XLim]);
                    end
                end
            end
            
            % Update display
            drawnow limitrate;
            
            % Display progress
            if mod(sampleCount, 100) == 0
                fprintf('Samples: %d\n', sampleCount);
            end
        end
        
        pause(0.001);
    end
    
catch exception
    disp('Error occurred:');
    disp(exception.message);
end

% Display summary
if ~isempty(allData)
    fprintf('\n=== Data Collection Summary ===\n');
    fprintf('Total samples: %d\n', size(allData, 1));
    fprintf('Time range: %.3f to %.3f s\n', min(allData(:,1)), max(allData(:,1)));
    fprintf('Accel X range: %.3f to %.3f g\n', min(allData(:,2)), max(allData(:,2)));
    fprintf('Accel Y range: %.3f to %.3f g\n', min(allData(:,3)), max(allData(:,3)));
    fprintf('Accel Z range: %.3f to %.3f g\n', min(allData(:,4)), max(allData(:,4)));
    
    % Save option
    saveChoice = input('Save data to file? (y/n): ', 's');
    if strcmpi(saveChoice, 'y')
        timestamp_str = datestr(now, 'yyyy-mm-dd_HH-MM-SS');
        filename = sprintf('imu_data_%s.csv', timestamp_str);
        headers = {'Time(s)', 'AcX(g)', 'AcY(g)', 'AcZ(g)', 'GyX(dps)', 'GyY(dps)', 'GyZ(dps)'};
        T = array2table(allData, 'VariableNames', headers);
        writetable(T, filename);
        fprintf('Data saved to: %s\n', filename);
    end
end

% Close serial port
clear s;
disp('Serial port closed.');
