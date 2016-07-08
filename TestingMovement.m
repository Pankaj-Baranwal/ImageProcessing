%%
colorDevice = imaq.VideoDevice('kinect',1);
depthDevice = imaq.VideoDevice('kinect',2);
step(colorDevice);
step(depthDevice);

colorImage = step(colorDevice);
depthImage = step(depthDevice);

ptCloud = pcfromkinect(depthDevice,depthImage,colorImage);

player = pcplayer(ptCloud.XLimits,ptCloud.YLimits,ptCloud.ZLimits,...
	'VerticalAxis','y','VerticalAxisDir','down');

xlabel(player.Axes,'X (m)');
ylabel(player.Axes,'Y (m)');
zlabel(player.Axes,'Z (m)');
 
ptCloud = pcfromkinect(depthDevice,depthImage,colorImage, 'depthCentric');

view(player,ptCloud);
%%
maxDistance = 0.02;
referenceVector = [0,1,0];
maxAngularDistance = 5;
[~,inlierIndices,~] = pcfitplane(ptCloud,maxDistance,referenceVector,maxAngularDistance);
plane1 = select(ptCloud,inlierIndices);
% remainPtCloud = select(ptCloud,outlierIndices);
% 
% roi = [-inf,inf;0.4,inf;-inf,inf];
% sampleIndices = findPointsInROI(ptCloud,roi);
% 
% [model2,inlierIndices,outlierIndices] = pcfitplane(remainPtCloud,maxDistance,'SampleIndices',sampleIndices);
% plane2 = select(remainPtCloud,inlierIndices);
% remainPtCloud = select(remainPtCloud,outlierIndices);

figure
pcshow(plane1)
title('First Plane')

% figure
% pcshow(plane2)
% title('Second Plane') 
% 
% figure
% pcshow(remainPtCloud)
% title('Remaining Point Cloud')

release(colorDevice);
release(depthDevice);

%%

zCoord = plane1.Location(:, 3, 1);
[~, zloc] = sort(zCoord);
SortedLocation = plane1.Location(gather(zloc), :, 1);

%%
init = (SortedLocation(1, 3, 1)*6)/7;
%% Move forward by init.
cell = 0;
for i = 1:length(SortedLocation)
    
    if SortedLocation(i, 3, 1)<0.25+init
        cell = i;
    else
%         xCoord = SortedLocation(1:cell, 1, 1);
%         [~, xloc] = sort(xCoord);
%         UpdatedLocation = SortedLocation(gather(xloc), :, 1);
countG = 0;
countL = 0;
countM = 0;
thresh = 4500;
moved = 0;
        for j = 1:cell
            
            if abs(SortedLocation(j, 1, 1))<0.13
                countM = countM+1;
            end
            if (countM>thresh)
                %%Move right by 35 cm.
                moved = 1;
                disp('Middle 30 cm')
                break;
            end
            if SortedLocation(j, 1, 1)<0.25 && SortedLocation(j, 1, 1)>0
                countG = countG + 1;
            end
            if (countG>thresh)
                %%Move right by 35 cm.
                moved = 1;
                disp('Right 30 cm 20 degrees')
                break;
            end
            if abs(SortedLocation(j, 1, 1))<0.25 && SortedLocation(j, 1, 1)<0
                countL = countL + 1;
            end
            if (countL>thresh)
                %%Move right by 35 cm.
                moved = 1;
                disp('Left')
                break;
            end
        end
        if (moved == 0)
                %Keep turning right by 30 degrees until path is found.
                disp('Turn right by 20 degrees and re run the program.');
        end
        countL
        countG
        countM
        break;
    end
end