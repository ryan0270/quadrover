clear
disp('start chadding')

%% Jun5_3 data
numPoints = [3.4 6.9 17 44 68 106 160 212 250];

rmsErrKF(3,:) = 129/10000*ones(size(numPoints));
rmsErrKF(4,:) = 843/10000*ones(size(numPoints));
rmsErrKF(5,:) = 927/10000*ones(size(numPoints));
rmsErrKF(6,:) = 1107/10000*ones(size(numPoints));

rmsErrORB(3,:) = [127 126 120 154 121 121 121 121 121]/1e4;
rmsErrORB(4,:) = [841 846 834 771 736 724 719 713 710]/1e4;
rmsErrORB(5,:) = [923 910 866 1205 1072 883 842 832 824]/1e4;
rmsErrORB(6,:) = [1081 1060 980 2066 1075 1064 1011 1020 1015]/1e4;

rmsErrNEW3s(3,:) = [123 121 119 116 115 114 114 114 115]/1e4;
rmsErrNEW3s(4,:) = [700 670 658 676 678 673 660 650 645]/1e4;
rmsErrNEW3s(5,:) = [800 753 749 742 739 733 726 735 731]/1e4;
rmsErrNEW3s(6,:) = [1019 999 966 924 909 884 886 891 901]/1e4;

calcTimeORB =   [20 21 22 25 28 30 38 42 46];
calcTimeNEW3s = [11 11 12 15 15 18 25 33 39];

submaskORB = [2 3 4 7];
subtimesORB{9} = [5.093231 8.501371 6.667587 21.657850 0.100163 2.466769 1.915463]/900*1000;
subtimesORB{8} = [4.959563 8.239150 4.118910 19.751402 0.086182 2.193029 1.753116]/900*1000;
subtimesORB{7} = [5.307986 9.298542 3.109867 19.526512 0.079862 2.092331 1.635826]/900*1000;
subtimesORB{6} = [4.623528 8.174824 2.159965 15.351160 0.053385 1.400807 1.191697]/900*1000;
subtimesORB{5} = [4.346855 8.214351 1.726711 13.922307 0.035145 0.982858 0.985451]/900*1000;
subtimesORB{4} = [3.794916 8.050122 1.330228 12.557511 0.025903 0.636806 0.822111]/900*1000;
subtimesORB{3} = [3.765687 7.553811 1.120617 10.452869 0.030854 0.277885 0.589398]/900*1000;
subtimesORB{2} = [3.828047 7.489265 1.034406 9.583567  0.025926 0.183575 0.234285]/900*1000;
subtimesORB{1} = [3.990292 7.775563 0.957414 8.725406 0.016590 0.133937 0.075274]/900*1000;

subtimesNEW3s{9} = [3.496167 9.016846 7.032567 3.321249 9.442634 5.779049 0.124063]/900*1000;
subtimesNEW3s{8} = [3.551595 8.810774 4.968175 2.923827 7.194047 4.759780 0.124810]/900*1000;
subtimesNEW3s{7} = [3.462271 9.040854 3.081107 2.257525 4.408489 3.347277 0.115066]/900*1000;
subtimesNEW3s{6} = [3.088361 8.213346 2.106607 1.573936 2.207960 2.126036 0.098326]/900*1000;
subtimesNEW3s{5} = [2.801478 7.961893 1.589075 1.117612 1.160221 1.387852 0.085665]/900*1000;
subtimesNEW3s{4} = [3.044804 9.119461 1.466580 0.982428 0.756883 1.107847 0.093989]/900*1000;
subtimesNEW3s{3} = [2.768490 7.799527 1.089369 0.449310 0.351953 0.645463 0.083705]/900*1000;
subtimesNEW3s{2} = [2.773169 7.771474 1.033151 0.196942 0.200338 0.626315 0.087097]/900*1000;
subtimesNEW3s{1} = [2.824145 7.790476 0.952583 0.125353 0.131078 0.568960 0.090726]/900*1000;

stateLabels = {'x [m]', 'y [m]', 'z [m]', 'x vel [m/s]', 'y vel [m/s]', 'z vel [m/s]'};
for st=3:6
	figure(10+st); clf
	set(gcf,'Units','Inches');
	curPos = get(gcf,'Position'); figSize = [5 5];
	set(gcf,'PaperSize',figSize,'PaperPosition',[0 0 figSize],'Position',[curPos(1:2) figSize]);

	plot(numPoints, rmsErrKF(st,:),'b--'); hold all
	plot(numPoints, rmsErrORB(st,:), '.-','Color',[0 0.5 0]); hold all
	plot(numPoints, rmsErrNEW3s(st,:), 'rx-'); hold all
	if st == 3
		axis([0 numPoints(end) 0.00 0.015]);
	elseif st == 4 || st == 5
		axis([0 numPoints(end) 0.0 0.11]);
	else
		axis([0 numPoints(end) 0.0 0.12]);
	end
	hold off
	xlabel('# feature points');
	ylabel(['rms err ' stateLabels{st}]);
	legend('KF only','ORB Hard C','Soft C','location','best')
end

figure(20); clf
set(gcf,'Units','Inches');
curPos = get(gcf,'Position'); figSize = [5 5];
set(gcf,'PaperSize',figSize,'PaperPosition',[0 0 figSize],'Position',[curPos(1:2) figSize]);
plot(numPoints, calcTimeORB, '.-','Color', [0 0.5 0]); hold all
plot(numPoints, calcTimeNEW3s, 'rx-'); hold all
hold off
axis([0 numPoints(end) 0 50]);
xlabel('# feature points');
ylabel('Calc Time [ms]');
legend('ORB Hard C', 'Soft C 3\sigma','location','best');

%%
sublabelsORB = 	{'KF update' 
				 sprintf('%s\n%s','Img Cnvt','and Blur') 
				 sprintf('%s\n%s','Find','Points') 
				 sprintf('%s\n%s','Feature Extract','and Match') 
				 'filter matched featers' 
				 'LS calc' 
				 sprintf('%s\n%s','Compute','MAP Estimate')};

figure(40); clf
set(gcf,'Units','Inches');
curPos = get(gcf,'Position'); figSize = [6 4];
set(gcf,'PaperSize',figSize,'PaperPosition',[0 0 figSize],'Position',[curPos(1:2) figSize]);
idx = 6;
bar( subtimesORB{idx}(submaskORB) );
ylabel('Processing time [ms]');
ax = axis; axis([ax(1) ax(2) 0 30]);
set(gca,'XTickLabel',[])
for i=1:length(submaskORB);
	ht = text(i, subtimesORB{idx}(submaskORB(i))+0.0005, sublabelsORB(submaskORB(i)),'interpreter','tex','HorizontalAlignment','center','VerticalAlignment','bottom');
end

%%
sublabelsNEW = {'KF update' 
				sprintf('%s\n%s','Img Cnvt','and Blur') 
				sprintf('%s\n%s','Find','Points') 
				sprintf('%s\n%s','Compute','Priors') 
				sprintf('%s\n%s','Compute','C') 
				sprintf('%s\n%s','Compute','MAP Estimate') 
				'misc'};
submaskNEW = [2 3 4 5 6];
figure(41); clf
set(gcf,'Units','Inches');
curPos = get(gcf,'Position'); figSize = [6 4];
set(gcf,'PaperSize',figSize,'PaperPosition',[0 0 figSize],'Position',[curPos(1:2) figSize]);
idx = 6;
bar( subtimesNEW3s{idx}(submaskNEW) );
ax = axis; axis([ax(1) ax(2) 0 30]);
set(gca,'XTickLabel',[])
for i=1:length(submaskNEW);
	text(i, subtimesNEW3s{idx}(submaskNEW(i))+0.0005, sublabelsNEW(submaskNEW(i)),'interpreter','tex','HorizontalAlignment','center','VerticalAlignment','bottom');
end


%%
disp('chad acomplished');