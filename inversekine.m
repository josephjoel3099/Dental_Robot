%Initializing all the links (DH)
L(1)=Link([0    0.05  0   pi/2]);
L(2)=Link([0    0  0.3   0]);
L(3)=Link([0    0   0  pi/2]);
L(4)=Link([0    0.265   0  0]);
L(2).offset = ([pi/2]);

%Linking all the links
R = SerialLink(L);
R.name = ('Dental Robot');

%For sim
%Sim variables
data_log = [];
time = 0;

%Trajectory variables (Semi-circle)
x = 0.4; y = 0; z = 0.3 ; r = 0.2;
th = [linspace(-3*pi/2,-pi/2,50),linspace(-pi/2,-3*pi/2,50),linspace(-3*pi/2,-pi/2,50),linspace(-pi/2,-3*pi/2,50)];
x_circle = r * cos(th) + x;
y_circle = r * sin(th) + y;

%Trajectory plot
plot3(x_circle, y_circle, linspace(z,z,length(th)));

%Loop for simulation
for i = 1:length(th)
    
    %homogeneous matrix of position in cartesian space
    T = transl(x_circle(i),y_circle(i),z);
    
    %Inverse kinematics
    I = R.ikine(T, 'mask',[1, 1, 1, 0, 0, 0]);
    
    %Joint variables
    j1 = I(1);
    j2 = I(2);
    j3 = I(3);
    j4 = I(4);
    
    %storing data
    time = time + 0.05;
    data_log = [data_log;time,x_circle(1,i),y_circle(1,i),z,j1,j2,j3,j4];
    
    %robot visualizer
    R.plot([j1 j2 j3 j4]);
    
    pause(0.05);
    
end