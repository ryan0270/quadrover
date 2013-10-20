clear
disp('start chadding')

%% script defining phone log ids
log_ids; 

%%
phoneFile = 'runData/phoneLog.txt';
phoneData = importdata(phoneFile,'\t');
phoneData = phoneData(1:end-1,:);

syncIndex = find(phoneData(:,2) == -500,1,'last');

gyroIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == LOG_ID_GYRO);
gyroTime = phoneData(gyroIndices,1)'/1000;
gyro = phoneData(gyroIndices,4:end)';
gyro_dt = mean(diff(gyroTime));

% magIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == LOG_ID_MAGNOMETER);
% magTime = phoneData(magIndices,1)'/1000;
% mag = phoneData(magIndices,4:end)';
% mag_dt = mean(diff(magTime));

% accelIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == LOG_ID_ACCEL);
% accelTime = phoneData(accelIndices,3)'/1000;
% accel = phoneData(accelIndices,4:6)';
% accel_dt = mean(diff(accelTime));


%%
if exist('mag','var') && ~isempty(mag)
    baseFigMag = 100;
    labelsMag = {'Mag x [\muT]' 'Mag y [\muT]' 'Mag z [\muT]'};

    figure(baseFigMag+5);
%     set(gcf,'Units','Inches');
%     curPos = get(gcf,'Position'); figSize = [6 4];
%     set(gcf,'PaperSize',figSize,'PaperPosition',[0 0 figSize],'Position',[curPos(1:2) figSize]);
    resetIndex = [];
    for i=1:3
        subplot(3,1,i)
        plot(magTime, mag(i,:)); hold all
        hold off
        title(labelsMag(i));
    end
end
%%
if exist('gyro','var') && ~isempty(gyro)
	labelsAngle = {'x [rad]','y [rad]','z [rad]'};
    labelsGyro = {'Gyro x [rad/s]' 'Gyro y [rad/s]' 'Gyro z [rad/s]'};
    baseFig = 20;

    figure(baseFig+2);set(gcf,'Units','Inches');
    % curPos = get(gcf,'Position'); figSize = [6 4];
    % set(gcf,'PaperSize',figSize,'PaperPosition',[0 0 figSize],'Position',[curPos(1:2) figSize]);
    for i=1:3
        figure(baseFig+i-1)
		subplot(2,1,1);
        plot(gyroTime, gyro(i,:));
		xlabel('Time [s]');
		ylabel(labelsGyro{i});
		subplot(2,1,2)
		plot(gyroTime, cumtrapz(gyroTime,gyro(i,:)));
        xlabel('Time [s]');
		ylabel(labelsAngle{i});
    end

end

%%
if exist('accel','var') && ~isempty(accel)
    labelsAccel = {'Accel x [m/s^2]' 'Accel y [m/s^2]' 'Accel z [m/s^2]'};
    baseFig = 30;
        
    figure(baseFig+2);set(gcf,'Units','Inches');
%     curPos = get(gcf,'Position'); figSize = [6 4];
%     set(gcf,'PaperSize',figSize,'PaperPosition',[0 0 figSize],'Position',[curPos(1:2) figSize]);
    for i=1:3
        subplot(3,1,i)
        plot(accelTime, accel(i,:)); hold all
%         plot(accelTime, accelFilt(i,:));
        hold off;
        ax = axis;
        axis([accelTime(1) accelTime(end) -20 20]);
        title(labelsAccel(i));
    end
end



%%
disp('chad accomplished')