clear
disp('start chadding')

%%
phoneFile = 'runData/phoneLog.txt';
phoneData = importdata(phoneFile,'\t');
phoneData = phoneData(1:end-1,:);

syncIndex = find(phoneData(:,2) == -500,1,'last');

angleStateRefIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == -1001);
angleStateRefTime = phoneData(angleStateRefIndices,1)'/1000;
angleStateRef = phoneData(angleStateRefIndices,3:8)';

angleStateIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == -1002);
angleStateTime = phoneData(angleStateIndices,1)'/1000;
angleState = phoneData(angleStateIndices,3:8)';

tranStateRefIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == -1011);
tranStateRefTime = phoneData(tranStateRefIndices,1)'/1000;
tranStateRef = phoneData(tranStateRefIndices,3:8)';

tranStateIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == -1012);
tranStateTime = phoneData(tranStateIndices,1)'/1000;
tranState = phoneData(tranStateIndices,3:8)';

if ~isempty(tranStateRef)
	tranStateRefInterp = interp1(tranStateRefTime,tranStateRef',angleStateRefTime)';
	tranStateInterp = interp1(tranStateTime,tranState',angleStateTime)';
else
	tranStateRefInterp = zeros(size(angleStateRef));
	tranStateInterp = zeros(size(angleState));
end
stateRefTime = angleStateRefTime;
stateRef = [angleStateRef; tranStateRefInterp];
stateTime = angleStateTime;
state = [angleState; tranStateInterp];
state_dt = mean(diff(stateTime));

gyroIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == 2);
gyroTime = phoneData(gyroIndices,1)'/1000;
gyro = phoneData(gyroIndices,3:end)';
gyro_dt = mean(diff(gyroTime));

magIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == 3);
magTime = phoneData(magIndices,1)'/1000;
mag = phoneData(magIndices,3:end)';
mag_dt = mean(diff(magTime));

accelIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == 1);
accelTime = phoneData(accelIndices,1)'/1000;
accel = phoneData(accelIndices,3:end)';
accel_dt = mean(diff(accelTime));

pressureIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == 4);
pressureTime = phoneData(pressureIndices,1)'/1000;
pressure = phoneData(pressureIndices,3:6)';

motorIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == -1000);
motorTime = phoneData(motorIndices,1)'/1000;
motorCmd = phoneData(motorIndices,3:6)';
cntlCalcTime = phoneData(motorIndices,8)'/1000;
motor_dt = mean(diff(motorTime));

mainRunTimeIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == -400);
mainRunTimeTime = phoneData(mainRunTimeIndices,1)'/1000;
mainRunTime = phoneData(mainRunTimeIndices,3:6)';

visionRunTimeIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == -401);
visionRunTimeTime = phoneData(visionRunTimeIndices,1)'/1000;
visionRunTime = phoneData(visionRunTimeIndices,3:8)';

imageGrabberRunTimeIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == -402);
imageGrabberRunTimeTime = phoneData(imageGrabberRunTimeIndices,1)'/1000;
imageGrabberRunTime = phoneData(imageGrabberRunTimeIndices,3:8)';

throttleIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == -2000);
throttleTime = phoneData(throttleIndices,1)'/1000;
throttle = phoneData(throttleIndices,3)';

attBiasIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == -710);
attBiasTime = phoneData(attBiasIndices,1)'/1000;
attBias = phoneData(attBiasIndices,3:5)';

forceScalingIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == -711);
forceScalingTime = phoneData(forceScalingIndices,1)'/1000;
forceScaling = phoneData(forceScalingIndices,3)';

% cpuUsageIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == -2000);
% cpuUsageTime = phoneData(cpuUsageIndices,1)'/1000;
% cpuUsage = phoneData(cpuUsageIndices,3:end)';

phoneTempIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == 500);
phoneTempTime = phoneData(phoneTempIndices,1)'/1000;
phoneTemp = phoneData(phoneTempIndices,3:6)';

%%
if exist('cpuUsage','var') && ~isempty(cpuUsage)
	figure(2000); set(gcf,'Name','CPU Usage')
	plot(cpuUsageTime,cpuUsage(1,:)');
	xlabel('Time [s]');
	ylabel('Usage ratio');
% 	legend('total','cpu0','cpu1','cpu2','cpu2')
	axis([0 20 0 0.5])
end

%%
if exist('phoneTemp','var') && ~isempty(phoneTemp)
	figure(500); set(gcf,'Name','Batt Temp');
	plot(phoneTempTime, phoneTemp(1,:)); hold all
	plot(phoneTempTime, phoneTemp(2,:)); hold all
	plot(phoneTempTime, phoneTemp(3,:)); hold all
	plot(phoneTempTime, phoneTemp(4,:)); hold all
	hold off
	xlabel('Time [s]');
	ylabel('Temp [degC]');
	legend('Batt','SEC','Fuelgauge','TMU');
end

%%
if ~isempty(state)
    baseFigState = 10;
    labels = {'Roll [rad]' 'Pitch [rad]' 'Yaw [rad]' 'Roll Rate [rad/s]' 'Pitch Rate [rad/s]' 'Yaw Rate [rad/s]' ...
              'x [m]' 'y [m]' 'z [m]' 'x vel [m/s]' 'y vel [m/s]' 'z vel [m/s]'};
    % figure(3); set(gcf,'Units','Inches');
    % curPos = get(gcf,'Position'); figSize = [5 5];
    % set(gcf,'PaperSize',figSize,'PaperPosition',[0 0 figSize],'Position',[curPos(1:2) figSize]);
    figure(baseFigState+0)
    for i=1:12
        subplot(4,3,i)
		if ~isempty(stateRef)
			plot(stateRefTime, stateRef(i,:)); hold all
		end
        plot(stateTime,state(i,:)); hold off
        xlabel('Time [s]');
        ylabel(labels(i));

    	ax = axis; axis([stateTime(1) stateTime(end) ax(3) ax(4)])
    % 	if i == 4 || i == 5
    % 		axis([0 40 -.75 .75]);
    % 	elseif i == 6
    % 		axis([0 40 -.75 .75]);
    % 	end
    end
    legend('Commanded','Measured');
end
 
%%
attBiasLabels = {'roll [rad]', 'pitch [rad]', 'yaw [rad]'};
if exist('attBias','var') && ~isempty(attBias)
    baseFigAttBias=700;
    figure(baseFigAttBias+10); set(gcf,'name','Att Bias')
    for i=1:3
        subplot(3,1,i)
        plot(attBiasTime, attBias(i,:));
        xlabel('Time [s]');
        ylabel(attBiasLabels(i));
    end
end

%%
if exist('forceScaling','var') && ~isempty(forceScaling)
    figure(711);
    plot(forceScalingTime, forceScaling);
    xlabel('Time [s]');
    ylabel('force scaling [N/cmd]');
end

%%
if ~isempty(mag)
    baseFigMag = 100;
    labelsMag = {'Mag x [\muT]' 'Mag y [\muT]' 'Mag z [\muT]'};

    figure(baseFigMag+5);
    set(gcf,'Units','Inches');
    curPos = get(gcf,'Position'); figSize = [6 4];
    set(gcf,'PaperSize',figSize,'PaperPosition',[0 0 figSize],'Position',[curPos(1:2) figSize]);
    resetIndex = [];
    for i=1:3
        subplot(3,1,i)
        plot(magTime, mag(i,:)); hold all
        hold off
        title(labelsMag(i));
    end

end
%%
if ~isempty(gyro)
    labelsGyro = {'Gyro x [rad/s]' 'Gyro y [rad/s]' 'Gyro z [rad/s]'};
    baseFig = 20;

    figure(baseFig+2);set(gcf,'Units','Inches');
    % curPos = get(gcf,'Position'); figSize = [6 4];
    % set(gcf,'PaperSize',figSize,'PaperPosition',[0 0 figSize],'Position',[curPos(1:2) figSize]);
    for i=1:3
        subplot(3,1,i)
        plot(gyroTime, gyro(i,:));
        ax = axis;
        axis([gyroTime(1) gyroTime(end) ax(3) ax(4)]);
        title(labelsGyro(i));
    end

end

%%
if isfinite(accel_dt)
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
if exist('pressure','var') && ~isempty(pressure)
	baseFig = 60;
	figure(baseFig);
	plot(pressureTime, pressure(1,:));
	xlabel('Time [s]');
	ylabel('Pressure [mBar]');
	
	Rstar = 8.31432; % N·m /(mol·K)
	Tb = 288.15; % K
	g0 = 9.80665; % m/s^2
	M = 0.0289644; % kg/mol
	Pb = 1013.25; % milliBar
	h0 = -Rstar*Tb/g0/M*log(pressure(1,1)/Pb)*0;
	h = -Rstar*Tb/g0/M*log(pressure(1,:)/Pb) - h0;
	
	tempInterp = interp1(phoneTempTime, phoneTemp(4,:), pressureTime,[],'extrap');
	k = (999.5-1000)/(45-37);
	pressComp = pressure(1,:)-k*(tempInterp-37);
	hComp = -Rstar*Tb/g0/M*log(pressComp/Pb) - h0;
	figure(baseFig+1);
	plot(pressureTime, h); hold all
	plot(pressureTime, hComp);
	hold off
	xlabel('Time [s]');
	ylabel('Height [m]');
	legend('Raw','Temp Comp')
end

%%
if isfinite(motor_dt)
    labelsCntl = {'Throttle' 'Cntl Roll' 'Cntl Pitch' 'Cntl Yaw'};
    labelsMotor = {'Motor North' 'Motor East' 'Motor South' 'Motor West'};
    baseFig = 1000;

    figure(baseFig+2);set(gcf,'Units','Inches');
    % curPos = get(gcf,'Position'); figSize = [6 6];
    % set(gcf,'PaperSize',figSize,'PaperPosition',[0 0 figSize],'Position',[curPos(1:2) figSize]);
    for i=1:4
        subplot(2,2,i)
        plot(motorTime, motorCmd(i,:));
        title(labelsMotor(i));
    end
end

