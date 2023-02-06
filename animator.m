function animator(Q_in_animator, Time,t1,t2,t3,t4)

figure();
set(gcf,'Visible', 'on');
v = VideoWriter('jaco_gravity.mp4','MPEG-4');

open(v)
alpha = [pi/8 pi/4 pi/2 pi; pi/8 pi/4 pi/2 pi; pi/8 pi/6 pi/4 pi/2; pi/8 pi/6 pi/4 pi/2];

for time = 1:100:length(Time)
    q = Q_in_animator(:,time);
    aa=(11*pi)/72;
    ca=cos(aa);
    sa=sin(aa);
    c2a=cos(2*aa);
    s2a=sin(2*aa);
    d4b=0.2073+((sa/s2a)*0.0743);
    d5b=((sa/s2a)*0.0743)+((sa/s2a)*0.0743);
    d6b=((sa/s2a)*0.0743)+0.1687;
    alphaa = [90,180,90,55,55,180]; % this is the alpha value for all  the link
    a=[0,0.41, 0, 0, 0, 0]; % Length of the Link
    d=[0.2755,0,-0.0098,-d4b,-d5b,-d6b]; %Offset
    Q=q'; % joint angle variation
    %Q_column=[Q1;Q2;Q3;Q4];
%Q=[180,180,180,180,180,180];
%% 
% Txy is the transformation from frame x to frame y.

%% Transformation Matrices
    for i = 1:6
    switch i
        case 1
            T01= [cos(Q(1,i)),-sin(Q(1,i))*cosd(alphaa(1,i)),sind(alphaa(1,i))*sin(Q(1,i)),a(1,i)*cos(Q(1,i));sin(Q(1,i)),cos(Q(1,i)).*cosd(alphaa(1,i)),-sind(alphaa(1,i))*cos(Q(1,i)),sin(Q(1,i))*a(1,i);0,sind(alphaa(1,i)),cosd(alphaa(1,i)),d(1,i);0,0,0,1];
        case 2
            T12= [cos(Q(1,i)),-sin(Q(1,i))*cosd(alphaa(1,i)),sind(alphaa(1,i))*sin(Q(1,i)),a(1,i)*cos(Q(1,i));sin(Q(1,i)),cos(Q(1,i)).*cosd(alphaa(1,i)),-sind(alphaa(1,i))*cos(Q(1,i)),sin(Q(1,i))*a(1,i);0,sind(alphaa(1,i)),cosd(alphaa(1,i)),d(1,i);0,0,0,1];
        case 3
            T23= [cos(Q(1,i)),-sin(Q(1,i))*cosd(alphaa(1,i)),sind(alphaa(1,i))*sin(Q(1,i)),a(1,i)*cos(Q(1,i));sin(Q(1,i)),cos(Q(1,i)).*cosd(alphaa(1,i)),-sind(alphaa(1,i))*cos(Q(1,i)),sin(Q(1,i))*a(1,i);0,sind(alphaa(1,i)),cosd(alphaa(1,i)),d(1,i);0,0,0,1];
        case 4
            T34= [cos(Q(1,i)),-sin(Q(1,i))*cosd(alphaa(1,i)),sind(alphaa(1,i))*sin(Q(1,i)),a(1,i)*cos(Q(1,i));sin(Q(1,i)),cos(Q(1,i)).*cosd(alphaa(1,i)),-sind(alphaa(1,i))*cos(Q(1,i)),sin(Q(1,i))*a(1,i);0,sind(alphaa(1,i)),cosd(alphaa(1,i)),d(1,i);0,0,0,1];
%     case 5
%         T45= [cos(Q(1,i)),-sin(Q(1,i))*cosd(alphaa(1,i)),sind(alphaa(1,i))*sin(Q(1,i)),a(1,i)*cos(Q(1,i));sin(Q(1,i)),cos(Q(1,i)).*cosd(alphaa(1,i)),-sind(alphaa(1,i))*cos(Q(1,i)),sin(Q(1,i))*a(1,i);0,sind(alphaa(1,i)),cosd(alphaa(1,i)),d(1,i);0,0,0,1];
%     case 6
%          T56= [cos(Q(1,i)),-sin(Q(1,i))*cosd(alphaa(1,i)),sind(alphaa(1,i))*sin(Q(1,i)),a(1,i)*cos(Q(1,i));sin(Q(1,i)),cos(Q(1,i)).*cosd(alphaa(1,i)),-sind(alphaa(1,i))*cos(Q(1,i)),sin(Q(1,i))*a(1,i);0,sind(alphaa(1,i)),cosd(alphaa(1,i)),d(1,i);0,0,0,1];    
    end
    end
%% 
% These are the individual tranformation matrices

    T01;
    T12;
    T23;
    T34;
%T45;
% T56;
% transformation_matrix= T01*T12*T23*T34*T45*T56;
    T02 = T01*T12;
    T03 = T01*T12*T23;
    T04 = T01*T12*T23*T34;
%T05 = T01*T12*T23*T34*T45;
% T06 = T01*T12*T23*T34*T45*T56;
    z0 = [0; 0; 1];
    z1 = T01(1:3,3);
    z2 = T02(1:3,3);
    z3 = T03(1:3,3);
    z4 = T04(1:3,3);
% z5 = T05(1:3,3);
    t1= T01(1:3, 4);
    t2= T02(1:3, 4);
    t3= T03(1:3, 4);
    t4= T04(1:3, 4);
    p0 = [0;0;0];
    p1=t1;
    p2=t2;
    p3=t3;
    p4=t4;
    X = [p0(1),p1(1),p2(1),p3(1),p4(1)];
    Y = [p0(2),p1(2),p2(2),p3(2),p4(2)];
    Z = [p0(3),p1(3),p2(3),p3(3),p4(3)];
    x1  = [p0(1); p1(1)];
    y1  = [p0(2); p1(2)];
    z1  = [p0(3); p1(3)];

    x2  = [p1(1); p2(1)];
    y2  = [p1(2); p2(2)];
    z2  = [p1(3); p2(3)];
    
    x3  = [p2(1); p3(1)];
    y3  = [p2(2); p3(2)];
    z3  = [p2(3); p3(3)];

    x4  = [p3(1); p4(1)];
    y4  = [p3(2); p4(2)];
    z4  = [p3(3); p4(3)];

    b = bezier(alpha,time/20);
    T_04 = ee_pos(b);
    tb = T_04(1:3, 4);

    xb = tb(1); yb = tb(2); zb = tb(3);
    

    clf;
    hold on


    plot3(x1,y1,z1,'LineWidth',3,'Color','r')
    plot3(x2,y2,z2,'LineWidth',3,'Color','g')
    plot3(x3,y3,z3,'LineWidth',3,'Color','r')
    plot3(x4,y4,z4,'LineWidth',3,'Color','b')
    plot3(xb,yb,zb,'LineWidth',30,'Color','k')
    view([1,1,1])
    axis equal;

    xlim([-0.8 0.8]);
    ylim([-0.8 0.8]);
    zlim([-0.8 0.8]);
    xlabel 'x'; ylabel 'y'; zlabel 'z';
    title(sprintf('time = %f seconds',Time(time)));
    frame = getframe(gcf);
    writeVideo(v,frame);
    drawnow;
%     pause(0.05);
end

close(v);
end

