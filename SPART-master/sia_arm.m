% 这里使用我所自主开发的模块化六轴机械臂 SIA_ARM_Trotsky
clear; clc;
% load SPART
SPART2path;
% import SIA-ARM
filename='siaarm.urdf';
[robot,robot_keys] = urdf2robot(filename);

%% Kinematics
    %Base-link position and orientation
    R0=eye(3);  %Rotation from base-link with respect to the inertial CCS.
    r0=[0;0;0]; %Position of the base-link with respect to the origin of the inertial frame, projected in the inertial CCS.
    qm=[0;0;0;0;0;0];%Joint displacements
    %Kinematics
    [RJ,RL,rJ,rL,e,g]=Kinematics(R0,r0,qm,robot);

    % 输出符号值
    %     %Base-link position
    %     r0=sym('r0',[3,1],'real');
    % 
    %     %Base-link orientation
    %     Euler_Ang=sym('Euler_Ang',[3,1],'real');
    %     R0 = Angles321_DCM(Euler_Ang)';
    % 
    %     %Joint displacements
    %     qm=sym('qm',[robot.n_q,1],'real');
    % 
    %     %Kinematics
    %     [RJ,RL,rJ,rL,e,g]=Kinematics(R0,r0,qm,robot);
    
%% 微分运动
    R0=eye(3);  %Rotation from base-link with respect to the inertial CCS.
    r0=[0;0;0]; %Position of the base-link with respect to the origin of the inertial frame, projected in the inertial CCS.
    qm=[0;0;0;0;0;0];%Joint displacements
    %Differential kinematics
	[Bij,Bi0,P0,pm]=DiffKinematics(R0,r0,rL,e,g,robot);

%% 雅可比
    %Jacobian of a point p in the ith link
	%rp is the position of the point p, projected in the inertial CCS -- as a [3x1] matrix.
	[J0p, Jmp]=Jacob(rp,r0,rL,P0,pm,i,robot);
    
%% 加速度
