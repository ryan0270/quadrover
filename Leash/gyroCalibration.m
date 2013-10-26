clear
disp('start chadding')

%% script defining phone log ids
log_ids; 

%%
phone = 's4';
files = {['runData/' phone 'bias_xPos.txt'] ['runData/' phone 'bias_yPos.txt'] ['runData/' phone 'bias_zPos.txt'];
		 ['runData/' phone 'bias_xNeg.txt'] ['runData/' phone 'bias_yNeg.txt'] ['runData/' phone 'bias_zNeg.txt']};

if strcmp(phone,'s4')
	bias = [0.011; -0.009; -0.009]; % s4
else
	bias = [-0.009; -0.011; -0.002]; % s3
end
scale = ones(2,3);
for fileNum = 1:length(files(:))
	fileI = floor((fileNum-1)/3)+1;
	fileJ = mod(fileNum-1,3)+1;
	phoneFile = files{fileI, fileJ};
	phoneData = importdata(phoneFile,'\t');
	phoneData = phoneData(1:end-1,:);

	syncIndex = find(phoneData(:,2) == -500,1,'last');

	gyroIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == LOG_ID_GYRO);
	gyroTime = phoneData(gyroIndices,1)'/1000;
	gyro = phoneData(gyroIndices,4:end)';
	gyro_dt = mean(diff(gyroTime));

	%%
	labelsAngle = {'x [rad]','y [rad]','z [rad]'};
	labelsGyro = {'Gyro x [rad/s]' 'Gyro y [rad/s]' 'Gyro z [rad/s]'};
	baseFig = 20;

	numRot = 10;
	
	figure(baseFig+fileNum-1)
	subplot(2,1,1);
	plot(gyroTime, gyro(fileJ,:));
	xlabel('Time [s]');
	ylabel(labelsGyro{fileJ});
	subplot(2,1,2)
	gyroInt =  cumtrapz(gyroTime,gyro(fileJ,:)-bias(fileJ));
	plot(gyroTime, gyroInt); hold all
	scale(fileI, fileJ) = abs(gyroInt(end))/(2*pi*numRot);
	plot(gyroTime, gyroInt/scale(fileI, fileJ)); hold all
	hold off
	xlabel('Time [s]');
	ylabel(labelsAngle{fileJ});
end
	
disp('scale');
disp(scale);

%%
disp('chad accomplished')