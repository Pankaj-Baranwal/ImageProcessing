%%
release(colorDevice);
release(depthDevice);
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
colorImage = step(colorDevice);  
depthImage = step(depthDevice);
 
ptCloud = pcfromkinect(depthDevice,depthImage,colorImage, 'depthCentric');
 
view(player,ptCloud);
%%
maxDistance = 0.02;
referenceVector = [0,1,0];
maxAngularDistance = 5;
[model1,inlierIndices,outlierIndices] = pcfitplane(ptCloud,maxDistance,referenceVector,maxAngularDistance);
plane1 = select(ptCloud,inlierIndices);
remainPtCloud = select(ptCloud,outlierIndices);
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
in = 1;
LocationConc = [0 0 0];
movement  = [];
threshz = 10;
threshx = 10;
while in<length(SortedLocation)
    count = 0;
    coordz = SortedLocation(:,3,1)+0.2;
    for i = in:length(SortedLocation)
        if SortedLocation(i, 3, 1)<coordz
            LocationConc = [LocationConc; SortedLocation(i, :, 1)]; %#ok<AGROW>
        elseif length(LocationConc)>threshz
            xCoord = SortedLocation(in:i-1, 1, 1);
            [~, xloc] = sort(xCoord);
            LocationConc = SortedLocation(gather(xloc), :, 1);
            coordx = LocationConc(1, 1, 1)+0.2;
            
        else
            in = i;
            break;
        end
    end
    xCoord = SortedLocation(:, 1, 1);
    [~, xloc] = sort(xCoord);
    LocationConc = SortedLocation(gather(xloc), :, 1);
    coorx = LocationConc(1, 1, 1)+0.2;
    for j = 1:length(LocationConc(:, 1, 1))
        if(LocationConc(j, 1, 1)<coorx)
            count=count+1;
        else
            if count>threshx
                movement = [movement LocationConc(1, :, 1)];
                break;
            end
        end
    end
    break;
end
%%
cell = 1;
count = 0;
% Dist = distance of point from origin
dist = sqrt((SortedLocation(1, 1, 1)^2)+(SortedLocation(1,3,1)^2));
% Loop 
for i=2:250
    if sqrt((SortedLocation(i, 1, 1)^2)+(SortedLocation(i,3,1)^2))<dist
        dist = sqrt((SortedLocation(i, 1, 1)^2)+(SortedLocation(i,3,1)^2));
        cell = i;
    end
end
coordz = SortedLocation(cell, 3, 1)+0.2;
LocationConc = zeros(15, 2);
for j = in:length(SortedLocation)
    if SortedLocation(j, 3, 1)<coordz
        LocationConc(count, :) = SortedLocation(i, 1:2:3, 1);
    else break;
    end
end
%%
thresh = 20000;
locations = plane1.Location;
% in is to check if next position of bot is possible here.
in = [0;0];
% Loop to continue traversing the location matrix untli next bot position
% is found.
for multip = 1:20:length(locations)
    % idx = location of 5*multip nearest neighbors of (0, 0) in x-z plane.
    5*multip
    idx = knnsearch(locations(:,1:2:3,1), [0, 0], 'k', 5*multip);
    % coords = Coordinates of idx points.
    coords = locations(idx, 1:2:3, 1);
    % dista = Distance of last point of idx from origin.
    for ij = length(idx):-1:1
    dista = sqrt((coords(ij, 1)^2)+(coords(ij, 2)^2));
    % Loop to find center of circle of bot's next position.
    for i = 1:100:length(SortedLocation)
        if SortedLocation(i, 3, 1)>coords(ij, 2)+.35
            if abs(SortedLocation(i, 1, 1))<0.2
                cell = i;
                break;
            end
        end
    end
    % count stores number of points within the circle.
    count = 0;
    % Loop to count number of points within circle.
    for i = 1:length(SortedLocation)
        if SortedLocation(i, 3, 1)>SortedLocation(cell, 3, 1)+0.35
            cell_max = i;
            break;
        end
        if sqrt(((SortedLocation(i, 1, 1)-SortedLocation(cell, 1, 1))^2)+((SortedLocation(i, 3, 1)-SortedLocation(cell, 3, 1))^2))<0.35
            count = count+1;
        end
    end
    % Check if number of points is greater than number of points required.
    % thresh is min num of points in circle for which it would be said to
    % be navigable position.
    if count>thresh
        in = [cell; cell_max];
        % bot_mov stores bot's next possible position.
        bot_move(1, :) = SortedLocation(cell, 1:2:3, 1);
        
        break;
    end
    end
    if in(1)~= 0
        break;
    end
end
if in(1) == 0 && in(2) == 0
    disp('turn right by 45 degrees and compute again.')
end