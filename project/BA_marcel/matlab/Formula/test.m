
% theta [ 0 .. 360 ]
%         x = 1*cos(theta[i]*PI/180.0)/(1.0+pow(sin(theta[i]*PI/180.0), 2))) ;
%         y = 1*(cos(theta[i]*PI/180.0)*sin(theta[i]*PI/180.0))/(0.2+pow(sin(theta[i]*CV_PI/180.0), 2))) ;
%     \]
    
    theta = 1:360;
    
    for x=1:360
    t(x)=round(150*cos(theta(x)*3.14/180)/(1+power(sin(theta(x)*3.14/180),2)));
    y(x)= round(150*(cos(theta(x)*3.14/180)*sin(theta(x)*3.14/180))/(0.2+power(sin(theta(x)*3.14/180),2)));
    end
    
    plot(t,y);