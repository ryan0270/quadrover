clear
disp('start chadding')

%%
phoneFile = 'phoneLog.txt';
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

gyroIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == 4);
gyroTime = phoneData(gyroIndices,1)'/1000;
gyro = phoneData(gyroIndices,3:end)';
gyro_dt = mean(diff(gyroTime));

magIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == 2);
magTime = phoneData(magIndices,1)'/1000;
mag = phoneData(magIndices,3:end)';
mag_dt = mean(diff(magTime));

accelIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == 1);
accelTime = phoneData(accelIndices,1)'/1000;
accel = phoneData(accelIndices,3:end)';
accel_dt = mean(diff(accelTime));

pressureIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == 6);
pressureTime = phoneData(pressureIndices,1)'/1000;
pressure = phoneData(pressureIndices,3:6)';

motorIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == -1000);
motorTime = phoneData(motorIndices,1)'/1000;
motorCmd = phoneData(motorIndices,3:6)';
cntlCalcTime = phoneData(motorIndices,8)'/1000;
motor_dt = mean(diff(motorTime));

imgProcTimeIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == -600);
imgProcTimeTime = phoneData(imgProcTimeIndices,1)'/1000;
imgProcTime = phoneData(imgProcTimeIndices,3)';
imgProc_dt = mean(diff(imgProcTimeTime));

imgBoxPosIndices = syncIndex-1+find(phoneData(syncIndex:end-1,2) == -601);
imgBoxPosTime =  phoneData(imgBoxPosIndices,1)'/1000;
imgBoxPos = phoneData(imgBoxPosIndices,3:end)';
imgBoxPos_dt = mean(diff(imgBoxPosTime));

imgCentroidIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == -610);
imgCentroidTime = phoneData(imgCentroidIndices,1)'/1000;
imgCentroid = phoneData(imgCentroidIndices,3:5)';
imgCentroid_dt = mean(diff(imgCentroidTime));

% imgFlowCamIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == -611);
% imgFlowCamTime = phoneData(imgFlowCamIndices,1)'/1000;
% imgFlowCam = phoneData(imgFlowCamIndices,3:end)';

imgPhoneStateIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == -612);
imgPhoneStateTime = phoneData(imgPhoneStateIndices,1)'/1000;
imgPhoneState = phoneData(imgPhoneStateIndices,3:8)';
imgPhoneState_dt = mean(diff(imgPhoneStateTime));

imgDesFlowIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == -614);
imgDesFlowTime = phoneData(imgDesFlowIndices, 1)'/1000;
imgDesFlow = phoneData(imgDesFlowIndices, 3:5)';
imgDesFlow_dt = mean(imgDesFlowTime);

rotErrIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == -620);
rotErrTime = phoneData(rotErrIndices, 1)'/1000;
rotErr = phoneData(rotErrIndices, 3:5)';

vIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == -630);
vTime = phoneData(vIndices,1)'/1000;
v = phoneData(vIndices,3:4)';

flowIntIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == -616);
flowIntTime = phoneData(flowIntIndices,1)'/1000;
flowInt = phoneData(flowIntIndices,3:5)';

uCamIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == -611);
uCamTime = phoneData(uCamIndices,1)'/1000;
uCam = phoneData(uCamIndices,3:end)';

yawResetIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == -805);
yawResetTime = phoneData(yawResetIndices,1)'/1000;
yawResetMag = phoneData(yawResetIndices,3:end)';

mainRunTimeIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == -400);
mainRunTimeTime = phoneData(mainRunTimeIndices,1)'/1000;
mainRunTime = phoneData(mainRunTimeIndices,3:6)';

visionRunTimeIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == -401);
visionRunTimeTime = phoneData(visionRunTimeIndices,1)'/1000;
visionRunTime = phoneData(visionRunTimeIndices,3:8)';

imageGrabberRunTimeIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == -402);
imageGrabberRunTimeTime = phoneData(imageGrabberRunTimeIndices,1)'/1000;
imageGrabberRunTime = phoneData(imageGrabberRunTimeIndices,3:8)';

mainOuterTimeIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == -405);
mainOuterTimeTime = phoneData(mainOuterTimeIndices,1)'/1000;
mainOuterTime = phoneData(mainOuterTimeIndices,3:end)';

mainInnerTimeIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == -406);
mainInnerTimeTime = phoneData(mainInnerTimeIndices,1)'/1000;
mainInnerTime = phoneData(mainInnerTimeIndices,3:end)';

throttleIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == -2000);
throttleTime = phoneData(throttleIndices,1)'/1000;
throttle = phoneData(throttleIndices,3)';

attBiasIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == -710);
attBiasTime = phoneData(attBiasIndices,1)'/1000;
attBias = phoneData(attBiasIndices,3:5)';

forceScalingIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == -711);
forceScalingTime = phoneData(forceScalingIndices,1)'/1000;
forceScaling = phoneData(forceScalingIndices,3)';

thrustIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == -12345);
thrustTime = phoneData(thrustIndices,1)'/1000;
thrust = phoneData(thrustIndices,3)';

accelEstIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == -1234);
accelEstTime = phoneData(accelEstIndices,1)'/1000;
accelEst = phoneData(accelEstIndices,3:5)';

cpuUsageIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == -2000);
cpuUsageTime = phoneData(cpuUsageIndices,1)'/1000;
cpuUsage = phoneData(cpuUsageIndices,3:end)';

phoneTempIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == 500);
phoneTempTime = phoneData(phoneTempIndices,1)'/1000;
phoneTemp = phoneData(phoneTempIndices,3:6)';

%%
if ~isempty(cpuUsage)
	figure(2000); set(gcf,'Name','CPU Usage')
	plot(cpuUsageTime,cpuUsage(1,:)');
	xlabel('Time [s]');
	ylabel('Usage ratio');
% 	legend('total','cpu0','cpu1','cpu2','cpu2')
	axis([0 20 0 0.5])
end

%%
if ~isempty(phoneTemp)
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
        plot(stateRefTime, stateRef(i,:)); hold all
        plot(stateTime,state(i,:)); hold off
        xlabel('Time [s]');
        ylabel(labels(i));

    % 	ax = axis; axis([stateTime(1) stateTime(end) ax(3) ax(4)])
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
if ~isempty(attBiasTime)
    baseFigAttBias=700;
    figure(baseFigAttBias+10);
    for i=1:3
        subplot(3,1,i)
        plot(attBiasTime, attBias(i,:));
        xlabel('Time [s]');
        ylabel(attBiasLabels(i));
    end
end

%%
if ~isempty(forceScalingTime)
    figure(711);
    plot(forceScalingTime, forceScaling);
    xlabel('Time [s]');
    ylabel('force scaling [N/cmd]');
end

%%
if ~isempty(mag)
    baseFigMag = 100;
    labelsMag = {'Mag x [\muT]' 'Mag y [\muT]' 'Mag z [\muT]'};

    [bMag aMag] = butter(3,0.5*2*mag_dt);
    magFilt = filtfilt(bMag, aMag, mag')';

    figure(baseFigMag+5);
    set(gcf,'Units','Inches');
    curPos = get(gcf,'Position'); figSize = [6 4];
    set(gcf,'PaperSize',figSize,'PaperPosition',[0 0 figSize],'Position',[curPos(1:2) figSize]);
    resetIndex = [];
    for j=1:length(yawResetTime)
        [temp resetIndex(j)] = min(abs(magTime-yawResetTime(j)));
    end
    for i=1:3
        subplot(3,1,i)
        plot(magTime, mag(i,:)); hold all
        for j=1:length(resetIndex)
            plot(magTime(resetIndex(j)),mag(i,resetIndex(j)),'r.','MarkerSize',30);
        end
        hold off
        title(labelsMag(i));
    end

end
%%
if ~isempty(gyroTime)
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
if ~isempty(pressure)
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
	
	tempInterp = interp1(phoneTempTime, phoneTemp(2,:), pressureTime,[],'extrap');
	k = (993.9-994.4)/(37-31);
	pressComp = pressure(1,:)-k*(tempInterp-30.8);
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

%% Image processing
if ~isempty(imgBoxPosTime)
	baseFig = 600;
	figure(baseFig);
	% set(gcf,'Units','Inches'); curPos = get(gcf,'Position'); figSize = [4 4];
	% set(gcf,'PaperSize',figSize,'PaperPosition',[0 0 figSize],'Position',[curPos(1:2) figSize]);
	% hold all
	plot(imgBoxPosTime, [0 diff(imgBoxPosTime)*1000], imgProcTimeTime,imgProcTime/1000);
	hold off
	xlabel('Time [s]');
	ylabel('Img Proc Time [ms]');
	
	figure(baseFig+1); clf
	% set(gcf,'Units','Inches'); curPos = get(gcf,'Position'); figSize = [4 4];
	% set(gcf,'PaperSize',figSize,'PaperPosition',[0 0 figSize],'Position',[curPos(1:2) figSize]);
	plot3(imgCentroid(1,:), imgCentroid(2,:), imgCentroid(3,:),'LineWidth',2); hold on
	plot3(imgCentroid(1,1), imgCentroid(2,1), imgCentroid(3,1),'ro');
	myFun = @(x,y,r) sqrt(r.^2-x.^2-y.^2);
	h = ezsurf(@(x,y) myFun(x,y,norm(imgCentroid(:,1))),[-4 4 -4 4]);
	% h = ezsurf(@(x,y) myFun(x,y,3.87),[-4 4 -4 4]);
	set(h,'FaceColor',[0.5 0.5 0.5],'LineStyle','none','FaceAlpha',0.3);
	hold off
	axis(4*[-1 1 -1 1 0 1])
	ax = axis;
	set(gca,'zdir','reverse')
	xlabel('centroid x');
	ylabel('centroid y');
	zlabel('centroid z');
	
	% figure(baseFig+2); clf
	% % set(gcf,'Units','Inches'); curPos = get(gcf,'Position'); figSize = [4 4];
	% % set(gcf,'PaperSize',figSize,'PaperPosition',[0 0 figSize],'Position',[curPos(1:2) figSize]);
	% hold all
	% for i=1:4
	% 	plot(imgBoxPos(2*(i-1)+1,:), imgBoxPos(2*i,:));
	% end
	% hold off
	% axis([0 320 0 240]);
	% set(gca,'YDir','reverse');
	% axis square
	% xlabel('Box x pos [px]');
	% ylabel('Box y pos [px]');
	%
	% figure(baseFig+3); clf
	% % set(gcf,'Units','Inches'); curPos = get(gcf,'Position'); figSize = [4 4];
	% % set(gcf,'PaperSize',figSize,'PaperPosition',[0 0 figSize],'Position',[curPos(1:2) figSize]);
	% imgCentroidLabels = {'x', 'y', 'z'};
	% for i=1:3
	%     subplot(3,1, i)
	%     plot(imgDesFeatTime, imgDesFeat(i,:), imgCentroidTime, imgCentroid(i,:));
	%     xlabel('Time [s]');
	%     ylabel(imgCentroidLabels{i});
	%     if i==1
	%         title('Img Feature Tracking');
	%     end
	% end
	
	figure(baseFig+4); clf
	% set(gcf,'Units','Inches'); curPos = get(gcf,'Position'); figSize = [4 4];
	% set(gcf,'PaperSize',figSize,'PaperPosition',[0 0 figSize],'Position',[curPos(1:2) figSize]);
	imgVelLabels = {'x', 'y', 'z'};
	for i=1:3
		subplot(3,1,i)
		plot(imgDesFlowTime, imgDesFlow(i,:)); hold all
		plot(imgPhoneStateTime, imgPhoneState(i+3,:));
		line(imgDesFlowTime([1 end]),[0 0],'Color','k','LineStyle','--')
		%     plot(imgFlowCamTime, imgFlowCam(i,:));
		hold off
		ax = axis;
		axis([imgPhoneStateTime(1) imgPhoneStateTime(end) -1 1]);
		xlabel('Time [s]');
		ylabel(imgVelLabels{i});
		if i==1
			title('Img Flow Tracking');
		end
	end
	%%
	
	% figure(baseFig+5); clf
	% % set(gcf,'Units','Inches'); curPos = get(gcf,'Position'); figSize = [4 4];
	% % set(gcf,'PaperSize',figSize,'PaperPosition',[0 0 figSize],'Position',[curPos(1:2) figSize]);
	% rotErrLabels = {'x', 'y', 'z'};
	% for i=1:3
	%     subplot(3,1,i)
	%     plot(rotErrTime, rotErr(i,:));
	%     xlabel('Time [s]');
	%     ylabel(rotErrLabels{i});
	%     if i==1
	%         title('rotErr');
	%     end
	% end
	
	figure(baseFig+6); clf
	stateLabels = {'roll [rad]' 'pitch [rad]' 'yaw [rad]'};
	for i=1:3
		subplot(3,1,i)
		plot(stateRefTime, stateRef(i,:), 'LineWidth',3); hold all
		plot(stateTime,state(i,:), ':','MarkerSize',3); hold off
		ax = axis;
		axis([imgPhoneStateTime(1) imgPhoneStateTime(end) -0.11 0.11]);
		xlabel('Time [s]');
		ylabel(stateLabels(i));
	end
	
	% figure(baseFig+7); clf
	% mask = v(1,:)>v(2,:);
	% plot(vTime, v(2,:)-v(1,:)); hold on;
	% plot(vTime(mask), v(2,mask)-v(1,mask), 'rs','MarkerSize',5);
	% hold off;
	% xlabel('Time [s]');
	% ylabel('v2-v1 []')
	% title('Lyapunov component comparison')
	% % legend('diff', 'v2');
	
	% figure(baseFig+8); clf
	% plot(flowIntTime, flowInt(1:2,:));
	% xlabel('Time [s]');
	% ylabel('Bias []');
	% legend('x', 'y');
	
	% figure(baseFig+9); clf
	% plot(imgBoxPos([1 3 5 7],:)'-160,imgBoxPos([2 4 6 8],:)'-120); hold all
	% plot(imgBoxPos([1 3 5 7],1)'-160,imgBoxPos([2 4 6 8])-120,'k.','MarkerSize',20,'LineWidth',3);
	% axis([-160 160 -120 120])
	% hold off
	
	% figure(baseFig+10); clf
	% nuTildeLabels = {'\nu_x', '\nu_y', '\nu_z'};
	% for i=1:3
	%     subplot(3,1,i)
	%     plot(nuTildeTime, nuTilde(i,:)); hold all
	% %     plot(uCamTime, uCam(i,:));
	%     hold off
	%     xlabel('Time [s]');
	%     ylabel(nuTildeLabels(i));
	% end
	
	% figure(baseFig+11); clf
	% subplot(2,3,1)
	% plot(mainRunTimeTime, mainRunTime,'.');
	% legend('t1','t2','t3','t4')
	% title('main run()');
	%
	% subplot(2,3,2)
	% plot(visionRunTimeTime, visionRunTime,'.');
	% legend('t1','t2','t3','t4','t5','t6','t7','t8')
	% title('vision run()');
	%
	% subplot(2,3,5)
	% plot(imageGrabberRunTimeTime, imageGrabberRunTime,'.')
	% legend('t1','t2','t3','t4','t5','t6','t7','t8')
	% title('image grabber run()');
	%
	% subplot(2,3,3)
	% plot(mainOuterTimeTime, mainOuterTime,'.');
	% legend('t1','t2','t3','t4','t5','t6','t7','t8','t9','t10')
	% title('main outer')
	%
	% subplot(2,3,6)
	% plot(mainInnerTimeTime, mainInnerTime,'.');
	% legend('t1','t2','t3','t4','t5','t6','t7','t8','t9','t10')
	% title('main inner');
	
	% figure(baseFig+12); clf
	% plot(throttleTime, throttle);
	% title('Throttle');
	
end

