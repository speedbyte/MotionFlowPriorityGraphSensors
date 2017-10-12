function [ fileName ] = init(speed,nr)

%%This script initializes the movement.
%First, the path is calculated.
%Then objects specs are set
%Then start points of the first and second object are set

tic;
%Creating a movement path. The path is stored in a x and y vector
scale = speed;
theta = 1:scale:360;
for x=1:360/scale-1
    xPos(x)=600+round(500*cos(theta(x)*3.14/180)/(1+power(sin(theta(x)*3.14/180),2)));
    yPos(x)= 150+round(55*(cos(theta(x)*3.14/180)*sin(theta(x)*3.14/180))/(0.2+power(sin(theta(x)*3.14/180),2)));
end

%object specs
width = 30;
height = 100;


%Start is somewhere on the path
start = 60/scale;
xOrigin = xPos(start);
yOrigin = yPos(start);

%Start of the second object is somewhere on the path
secondStart = 240/scale;
secondXOrigin = xPos(secondStart);
secondYOrigin = yPos(secondStart);

%for moving the objects later
actualX = xOrigin;
actualY = yOrigin;
secondActualX = secondXOrigin;
secondActualY = secondYOrigin;


%how many interations(frames)?
maxIteration = (360/scale)-1;


%save this workspace to call it from the other functions
if nr == 1
fileName = 'InitializeSlow.mat';
end
if nr == 2
    fileName ='InitializeNormal.mat';
end
if nr == 3
    fileName='InitializeFast.mat'
end

save(fileName,'xPos','yPos','width','height','start','xOrigin','yOrigin','secondStart',...
    'secondXOrigin','secondYOrigin','actualX','actualY','secondActualX','secondActualY','maxIteration');
time = toc;
disp(time);






end

