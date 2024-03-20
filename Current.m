%% Akakzatok létrehozása
x = [-40; -38; -21; -13; 2; 22; 7; -3; -14];
y = [-40; -25; -18; -20; -23; 10; 5; 12; 10];
w = [80; 15; 6; 6; 10; 18; 10; 11; 9];
h = [80; 5; 10; 3; 15; 7; 2; 14; 5;];

Pos = [x, y, w, h];

%%
clf;

for i=[1 3 5 7 9]
    rectangle('Position', Pos(i,:)) 
end

for i=[2 4 6 8]
    rectangle('Position', Pos(i,:),'Curvature',0.5) 
end

rectangle('Position', [-15 -33 3 4],'Curvature', 1)

rectangle('Position', [-35 -35 5 5],'Curvature',[1 1])

rectangle('Position', [34 34 2 2],'Curvature',[1 1])
r_r = 0.2; % robot sugara
axis off;
%% Mentés utáni kép beolvasása
I = imread('Map.bmp');
%% bináris kép létrehozása
IGBW = im2bw(I);
IGBW = ~IGBW;   % az Occupancy Map fordított logikát használ
%% Térkép létrehozása
map = binaryOccupancyMap(IGBW,13);
show(map)
inflate(map,r_r)

%% Lidar init
lidar = LidarSensor;
pose = [26; 19; pi/2];                  % Robot pozíció és irány

lidar.mapName = 'map';
lidar.sensorOffset = [0,0];
lidar.scanAngles = linspace(-pi,pi,360); 
lidar.maxRange = 15;

goal= [15; 17];
ranges = lidar(pose);                   % LiDAR adatok kiolvasása
plot(lidar.scanAngles,ranges,'o-');     % Diagramm megjelenítés

%% vizualizáció
viz = Visualizer2D;
viz.mapName = 'map';
attachLidarSensor(viz,lidar)
viz(pose,ranges)

%%
goalDir=atan2(goal(2)-pose(2),goal(1)-pose(1))-pose(3);
if goalDir < -pi
    goalDir=goalDir+2*pi;
end

%% Search for gaps' side
PL= [0 0 0 0];
PR= [0 0 0 0];
k=1;
l=1;
for i= 1:359
    dis=ranges(i)-ranges(i+1);
    if or(dis > 0.4,and(isnan(ranges(i)), ~isnan(ranges(i+1))))
       if k==1
        PL(k,:)=[lidar.scanAngles(i+1) , ranges(i+1), 1, abs(lidar.scanAngles(i+1)-goalDir)];
        k=k+1;
       elseif PL(k-1)~=lidar.scanAngles(i)
        PL(k,:)=[lidar.scanAngles(i+1) , ranges(i+1), 1, abs(lidar.scanAngles(i+1)-goalDir)];
        k=k+1;
       end
    elseif or(dis < -0.4,and(isnan(ranges(i+1)), ~isnan(ranges(i))))
       if l==1
        PR(l,:)=[lidar.scanAngles(i) , ranges(i),0,abs(lidar.scanAngles(i)-goalDir)];
        l=l+1;
       elseif PR(l-1)==lidar.scanAngles(i-1)
        PR(l-1,:)=[lidar.scanAngles(i) , ranges(i), 0,abs(lidar.scanAngles(i)-goalDir)];
       else
        PR(l,:)=[lidar.scanAngles(i) , ranges(i), 0,abs(lidar.scanAngles(i)-goalDir) ];
        l=l+1;
       end
    end
end

%%
Gaps=[PL;PR];
Gaps=sortrows(Gaps,1);
%% Find gaps
i=1;
p=1;
while i < size(Gaps,1)
    if Gaps(i,3)==Gaps(i+1,3)
        if Gaps(i,2)<Gaps(i+1,2)
            Gaps(i+1,:)=[];
            i=i+1;
        else
            Gaps(i,:)=[];
        end

    else
        i=i+1;
    end
end
if Gaps(1,3)==Gaps(i,3)
    if Gaps(1,2)<Gaps(i,2)
        Gaps(i,:)=[];
    else
        Gaps(1,:)=[];
    end
end
if Gaps(1,3)==1
    Gaps(end+1,:)=Gaps(1,:);
    Gaps(1,:)=[];
end


%% Choose a gap
i=1;
goalGap=[0 0 0 0];
while i < size(Gaps,1)
    if and(goalDir > Gaps(i,1), goalDir < Gaps(i+1,3))
        goalGap=Gaps(i,:);
    end
    i=i+2;
end
if goalGap==[0 0 0 0]
    if and(goalDir>Gaps(end-1,1),goalDir < Gaps(end,1))
        goalGap=Gaps(i,:);
    end
end
if goalGap==[0 0 0 0]
    minD=min(Gaps(:,4));
    i=1;
    while i<=size(Gaps,1)
        if Gaps(i,4)==minD
            goalGap=Gaps(i,:);
            break
        end
        i=i+1;
    end
end
%% start the robot


