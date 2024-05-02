map=binaryOccupancyMap(30,30,5);
Cx=[10;20];
Cy=[15;15];
setOccupancy(map, [Cx Cy], 1);
inflate(map, 1);
Sx=15;
Sy=15;
theta = linspace(0,2*pi,200);
x5 = 14.0+0.1+cos(theta); % x circle coordinates
y5 = 6.0+sin(theta); % y circle coordinates
% setOccupancy(map, [x5; y5]', 1);

%% Lidar init
lidar = LidarSensor;
pose = [15; 10; pi/2];                  % Robot pozíció és irány
goal= [20 25];
waypoints=goal;
lidar.mapName = 'map';
lidar.sensorOffset = [0,0];
lidar.scanAngles = linspace(-pi,pi,360);
lidar.maxRange = 10;
X=pose(1);
Y=pose(2);



D=sqrt((goal(1)-pose(1))^2+(goal(2)-pose(2))^2);
choosenGaps{1}{1}={0,0}; %!!!!!!! magyarázat
choosenGaps{1}{2}={0,0};
nextGap{1}{1}={nan,nan};
h=0;
g=0;
%%
while D > 0.4
    Sx=Sx+0.2;
    Sy=Sy+0.2;
    setOccupancy(map, [Sx Sy], ones(5,10));
    x5 = x5+0.1; % x circle coordinates
    y5 = y5; % y circle coordinates
    setOccupancy(map, [x5; y5]', 1);

    lidar = LidarSensor;
    lidar.mapName = 'map';
    lidar.sensorOffset = [0,0];
    lidar.scanAngles = linspace(-pi,pi,360);
    lidar.maxRange = 10;

    viz = Visualizer2D;
    viz.mapName = 'map';
    viz.hasWaypoints=true;
    attachLidarSensor(viz,lidar)

    ranges = lidar(pose);                   % LiDAR adatok kiolvasása
    %subplot(1,2,1);
    %plot(lidar.scanAngles,ranges,'o-');     % Diagramm megjelenítés

    %% vizualizáció
    % subplot(1,2,2);
    X(end+1)=pose(1);
    Y(end+1)=pose(2);

    viz(pose,waypoints,ranges)
    hold on;
    plot(X,Y,'r.-')

    %%
    goalDir(1)=atan2(goal(2)-pose(2),goal(1)-pose(1))-pose(3);
    if goalDir(1) < -pi
        goalDir(1)=goalDir(1)+2*pi;
    end
    %% Search for gaps' side
    PL= [0 0 0 0];
    PR= [0 0 0 0];
    k=1;
    l=1;

    for i= 1:359
        dis=ranges(i)-ranges(i+1);
        if and(abs(lidar.scanAngles(i)-goalDir(1))<pi/180,or(ranges(i)>D,isnan(ranges(i))))
            g=1;
            break;
        end
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
    if g~=1
        Gaps=[PL;PR];
        Gaps=sortrows(Gaps,1);
        %% Find gaps
        i=1;
        while i < size(Gaps,1)
            if Gaps(i,3)==Gaps(i+1,3)
                if abs(Gaps(i,1)-Gaps(i+1,1))>0.03501
                    if Gaps(i,2)<Gaps(i+1,2)
                        Gaps(i+1,:)=[];
                    else
                        Gaps(i,:)=[];
                    end
                else
                    if Gaps(i,2)>Gaps(i+1,2)
                        Gaps(i+1,:)=[];
                    else
                        Gaps(i,:)=[];
                    end
                end
            else
                i=i+1;
            end
        end
        if Gaps(1,3)==Gaps(end,3)
            if abs(Gaps(1,1)-Gaps(end,1))>0.03501
                if Gaps(1,2)<Gaps(end,2)
                    Gaps(end,:)=[];
                else
                    Gaps(1,:)=[];
                end
            else
                if Gaps(1,2)>Gaps(end,2)
                    Gaps(end,:)=[];
                else
                    Gaps(1,:)=[];
                end
            end
        end
        if Gaps(1,3)==1
            Gaps(end+1,:)=Gaps(1,:);
            Gaps(1,:)=[];
        end


        %% Choose a gap
        i=1;
        while i < size(Gaps,1)
            if and(goalDir > Gaps(i,1), goalDir < Gaps(i+1,1))
                if g>1
                    if m>3

                        nextGap{1}{1}={Gaps(i,2)*cos(Gaps(i,1)+pose(3))+pose(1),Gaps(i,2)*sin(Gaps(i,1)+pose(3))+pose(2)};
                        nextGap{1}{2}={Gaps(i+1,2)*cos(Gaps(i+1,1)+pose(3))+pose(1),Gaps(i+1,2)*sin(Gaps(i+1,1)+pose(3))+pose(2)};
                        if or([0.5 0.5]>abs(cell2mat(nextGap{1}{1})-cell2mat(choosenGaps{end}{1})),[0.5 0.5]>abs(cell2mat(nextGap{1}{1})-cell2mat(choosenGaps{end}{1})))
                        else
                            choosenGaps{end+1}=nextGap{1};
                            g=1;

                            break
                        end
                    else
                        m=m+1;
                    end
                end
                g=1;


            end

            i=i+2;
        end
        if g==0
            if and(goalDir>Gaps(end-1,1),goalDir < Gaps(end,1))
                nextGap{1}{1}={Gaps(end-1,2)*cos(Gaps(end-1,1)+pose(3))+pose(1),Gaps(end-1,2)*sin(Gaps(end-1,1)+pose(3))+pose(2)};
                nextGap{1}{2}={Gaps(end,2)*cos(Gaps(end,1)+pose(3))+pose(1),Gaps(end,2)*sin(Gaps(end,1)+pose(3))+pose(2)};
                if or([0.5 0.5]>abs(cell2mat(nextGap{1}{1})-cell2mat(choosenGaps{end}{1})),[0.5 0.5]>abs(cell2mat(nextGap{1}{1})-cell2mat(choosenGaps{end}{1})))
                else   choosenGaps{end+1}=nextGap{1};
                end

                g=1;
            end
        end
        if g==0
            minD=min(Gaps(:,4));
            i=1;
            while i<=size(Gaps,1)
                if Gaps(i,4)==minD
                    if Gaps(i,2)>2
                        h=0;
                    else
                        h=1;
                    end

                    if Gaps(i,3)==0 %job oldali pont
                        nextGap{1}{1}={Gaps(i,2)*cos(Gaps(i,1)+pose(3))+pose(1),Gaps(i,2)*sin(Gaps(i,1)+pose(3))+pose(2)};
                        nextGap{1}{2}={Gaps(i+1,2)*cos(Gaps(i+1,1)+pose(3))+pose(1),Gaps(i+1,2)*sin(Gaps(i+1,1)+pose(3))+pose(2)};
                        g=2;
                    else %bal oldali pont
                        nextGap{1}{1}={Gaps(i-1,2)*cos(Gaps(i-1,1)+pose(3))+pose(1),Gaps(i-1,2)*sin(Gaps(i-1,1)+pose(3))+pose(2)};
                        nextGap{1}{2}={Gaps(i,2)*cos(Gaps(i,1)+pose(3))+pose(1),Gaps(i,2)*sin(Gaps(i,1)+pose(3))+pose(2)};
                        g=3;
                    end
                    if or([0.5 0.5]>abs(cell2mat(nextGap{1}{1})-cell2mat(choosenGaps{end}{1})),[0.5 0.5]>abs(cell2mat(nextGap{1}{1})-cell2mat(choosenGaps{end}{1})))
                    else
                        choosenGaps{end+1}=nextGap{1};
                    end
                    break
                end
                i=i+1;
            end
        end
    end
    %% start the robot
    vmax=0.3;
    if g==1
        if and(goalDir>-pi/4,goalDir<pi/4) % ha látszik a cél
            pose=[vmax*cos(atan2(goal(2)-pose(2),goal(1)-pose(1)))+pose(1)
                vmax*sin(atan2(goal(2)-pose(2),goal(1)-pose(1)))+pose(2)
                goalDir+pose(3)];
        elseif goalDir>pi/4
            pose(3)=pi/4; %EZT MÉG MEG KELL VARIÁLNI
        else
            pose(3)=-pi/4;
        end
    elseif g==2 %job oldali
        if h==0
            safeA=5*pi/180; % ha túl megy a gapen akkor kéne valami más...
        else
            safeA=10*pi/180;
        end
        R_D=atan2((choosenGaps{end}{1}{2}-pose(2)),(choosenGaps{1}{1}{1}-pose(1)));

        if and(R_D-pose(3)>-pi/4,R_D-pose(3)<pi/4)
            pose=[vmax*cos(R_D+safeA)+pose(1)
                vmax*sin(R_D+safeA)+pose(2)
                R_D+safeA];
        elseif R_D-pose(3)>pi/4
            pose(3)=pose(3)+pi/4;
        else
            pose(3)=pose(3)-pi/4;
        end

    elseif g==3%Bal oldali
        if h==0
            safeA=5*pi/180;
        else
            safeA=10*pi/180;
        end
        R_D=atan2((choosenGaps{end}{2}{2}-pose(2)),(choosenGaps{end}{2}{1}-pose(1)));
        if and(R_D-pose(3)>-pi/4,R_D-pose(3)<pi/4)
            pose=[vmax*cos(R_D-safeA)+pose(1)
                vmax*sin(R_D-safeA)+pose(2)
                R_D-safeA];
        elseif R_D-pose(3)>pi/4
            pose(3)=pose(3)+pi/4;
        else
            pose(3)=pose(3)-pi/4;
        end
    end
    D=sqrt((goal(1)-pose(1))^2+(goal(2)-pose(2))^2);
    setOccupancy(map, [x5; y5]', 0);
    setOccupancy(map, [Sx Sy], zeros(5,10));
end
viz(pose,waypoints,ranges)