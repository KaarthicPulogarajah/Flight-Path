function flightPath
    null = [];
    origin = [0,0,0];
    
    %97.9 kN of thrust per CFM Intl. CFM56-3B2 engine (2)
    thrust = 97900*2;
    mass = input('\nEnter mass(kg) of Boeing 737-300 to generate \ntakeoff flight path\nMin: 33000 kg\nMax: 63000 kg\n');
    
    %function to fit mass to takeoff rotation speed in knots
    VrKnots = 1.707*1e-3*mass+45.78;
    
    %takeoff rotation speed in m/s
    Vr = VrKnots*463/900;

    planeStart = vpa(runway(Vr,mass,thrust),5);

    %point where plane leaves runway
    takeoffPoint = [-3.5*Vr,-3.5*Vr,0];
    
    %angle of constant takeoff ascent
    angle = 17;

    % parameters for vectors
    r = 2800;
    r = r/sqrt(2);
    u = 600;
    
    %establish vectors in parameter form
    runwayVect = origin + [-1, -1, 0]*r;  
    
    %angle of ascent increase
    dAngle = 16/7;
    
    syms t;
    
    %integrate function for slopes at takeoff to give function for takeoff path 
    takeoffSlope = tan((dAngle*(t+6.5*Vr)+(15/7)*Vr)/Vr*(pi/180));
    takeoffFunc = int(takeoffSlope,t);
    t = -3.5*Vr;
    takeoffFunc = takeoffFunc - subs(takeoffFunc);

    t=0;
    vpa(subs(takeoffFunc),5);
    clearXY = vpa(solve(takeoffFunc == 10.7, t),5);    %GIVES THAT 10.7m IS CLEARED AT 3.84s, CONSISTENT WITH "target liftoff
    %attitude reached after 3-4s"
    
    a = 1;
    for i = -3.5*Vr:(3.5*Vr/100):0
        t = i;
        takeoffX(a) = t;
        takeoffZ(a) = subs(takeoffFunc);
        a = a+1;
    end
    takeoffCurve = [takeoffX;takeoffX;takeoffZ];
    size(takeoffX);
    vpa(takeoffZ(end),4);  

    %35ft above ground that plane has to clear at end of runway
    clearPoint = [clearXY(2),clearXY(2),10.7];
    flightStart = [0,0,takeoffZ(end)];
    flightVect = flightStart + [1,1,sin(angle*pi/180)]*u;
    
    
    %PointA is arbitrary point
    PointA = [752,274,0];
    
    VrPoint = [-6.5*Vr, -6.5*Vr, 0];
    curveEndPoint = [0,0,takeoffZ(end)];
   
    [shortestDistance,closestPoint] = shortestDist(flightStart, flightVect, PointA);
    vpa(shortestDistance,4);
    vpa(closestPoint,4);
    
    %individual points to plot
    plotPoints = [PointA;closestPoint;planeStart;VrPoint;takeoffPoint;clearPoint];
    
    %identify vectors to plot by start and end points, and put in 3d matrix
    plotVectors = [origin;runwayVect];
    plotVectors(:,:,2) = [flightStart;flightVect];

    %identify curves to plot and put in 3d matrix
    plotCurves = [takeoffCurve];

    legend = {'My Position','Closest Point','Aircraft Initial Position','Vr Point','Take Off Point','Clear Point (35 ft)','Runway', 'Flight Vector', 'Flight Curve'};

    width = 800;
    f1 = figure('Name','example1','NumberTitle', 'off', 'Position', [0,0,width*4/3,width]);
    plot3d(plotPoints, plotVectors, plotCurves, legend);
   
    
%     f2 = figure('Name','example2','NumberTitle', 'off');
%     plot3d(null, null, plotCurves);

end

function plot3d(points,vectors,curves,names)
    
    hold on;
    view([15,15]);

    if size(points) ~= 0
        %plot points
        [rowP,colP] = size(points);
        for i = 1:rowP
            plot3(points(i,1),points(i,2),points(i,3),'*');
        end
    end
    
    if size(vectors) ~= 0
        %plot vectors by iterating through x and y columns, then the next vector
        [height,width,depth] = size(vectors);
        for i = 1:depth
            plot3(vectors(:,1,i),vectors(:,2,i),vectors(:,3,i),'-');
        end
    end
    
    if size(curves) ~= 0
        %plot curves by iterating through x and y columns, then the next curve
        [height,width,depth] = size(curves);
        for i = 1:depth
            plot3(curves(1,:,i),curves(2,:,i),curves(3,:,i),'m-');
        end
    end
 
    axis([-2700 1500 -2700 1500 0 2000]);
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    box on;
    h = rotate3d;
    h.Enable = 'on';
    h.RotateStyle = 'orbit';
    l = legend(names(:));
    l.Location = 'bestoutside';

end

function [shortestDist,closestPoint] = shortestDist(vectStart, vectEnd, pointArb)

%determine flight path vector and vector from start point to an arbitrary
%point "Point A" in 3d space
flightVector = vectEnd - vectStart;
pointVector = vectStart - pointArb;

%distance from start point to Point A
pointVectorMag = norm(pointVector);

%use scalar projection to find distance along flight path vector projected
%by the pointVector
flightVectorProj = abs(dot(pointVector,flightVector)/norm(flightVector));

%use pythagorean theorem to determine shortest (perpendicular) distance
%from Point A to vector
shortestDist = vpa(sqrt(pointVectorMag^2 - flightVectorProj^2),4)

%distance along the flight path vector from start point to point
%closest to Point A is already known: flightVectorProj
%use an arbitrary parameter t to determine the change in x,y,and z to go
%from start point to point closest to Point A
%solve for t parameter by allowing the magnitude of the components of the
%flight vector to equal the projection distance
syms t;
tvalues = solve(sqrt((flightVector(1)*t)^2+(flightVector(2)*t)^2+(flightVector(3)*t)^2) == flightVectorProj,t);

%there will be 2 values for t, and the correct one must be found
for i = 1:2
    for j = 1:3
        %find the point on the flight vector corresponding to each t value
        tPoints(i,j) = vectStart(j)+ flightVector(j)*tvalues(i);
    end
    %find the dist from Point A to each tPoint
    tDists(i) = sqrt((tPoints(i,1)-pointArb(1))^2+(tPoints(i,2)-pointArb(2))^2+(tPoints(i,3)-pointArb(3))^2);
end

%smaller tDist corresponds to the correct tPoint (the closest point)
if tDists(1) < tDists(2)
    closestPoint = [tPoints(1,1), tPoints(1,2), tPoints(1,3)];
else
    closestPoint = [tPoints(2,1), tPoints(2,2), tPoints(2,3)];
end

end

function [planeStart] = runway(Vr, mass, thrust)

    %lift-induced drag is highest when airspeed is low - during takeoff
    drag = 0.5*thrust;
    %calculate acceleration using Newton's Second Law
    acceleration = (thrust-drag)/mass;
    
    %calculate distance using kinematics: v^2 = v0^2 + 2ad
    syms d;
    runwayDist = vpa(solve(Vr^2 == 2*acceleration*d,d));
    
    %calculate expected V2 value given acceleration using kinematics
    V2 = sqrt(2*acceleration*(6.5*Vr+runwayDist));

    %display Vr and V2 in knots
    VrKnots = vpa(Vr*900/463,4)
    VRunwayEndKnots = vpa(V2*900/463,4)
    
    %split distance into x and y lengths where the angle is 45deg
    runwayDist = runwayDist/sqrt(2);
    planeStart = [-6.5*Vr-runwayDist, -6.5*Vr-runwayDist, 0];

end
