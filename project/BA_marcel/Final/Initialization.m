%%This script initializes the movement.
%First, the path is calculated.
%Then objects specs are set
%Then start points of the first and second object are set

clear all;

theta = 1:0.5:360;
for x=1:719
    xPos(x)=600+round(500*cos(theta(x)*3.14/180)/(1+power(sin(theta(x)*3.14/180),2)));
    yPos(x)= 150+round(100*(cos(theta(x)*3.14/180)*sin(theta(x)*3.14/180))/(0.2+power(sin(theta(x)*3.14/180),2)));
end

%object specs
width = 30;
height = 80;

start = 300;

%Start is somewhere on the path
xOrigin = xPos(start);
yOrigin = yPos(start);

secondStart = 719;

%Start of the second object is somewhere on the path
secondXOrigin = xPos(secondStart);
secondYOrigin = yPos(secondStart);

%for moving the objects later
actualX = xOrigin;
actualY = yOrigin;

secondActualX = secondXOrigin;
secondActualY = secondYOrigin;


%how many interations?
maxIteration = 1000;


fileName = 'Initialize.mat';
save(fileName,'xPos','yPos','width','height','start','xOrigin','yOrigin','secondStart',...
    'secondXOrigin','secondYOrigin','actualX','actualY','secondActualX','secondActualY','maxIteration');
