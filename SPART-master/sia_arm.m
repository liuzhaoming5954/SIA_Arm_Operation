% ����ʹ����������������ģ�黯�����е�� SIA_ARM_Trotsky
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

    % �������ֵ
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
    
%% ΢���˶�
    R0=eye(3);  %Rotation from base-link with respect to the inertial CCS.
    r0=[0;0;0]; %Position of the base-link with respect to the origin of the inertial frame, projected in the inertial CCS.
    qm=[0;0;0;0;0;0];%Joint displacements
    %Differential kinematics
	[Bij,Bi0,P0,pm]=DiffKinematics(R0,r0,rL,e,g,robot);

%% �ſɱ�
    %Jacobian of a point p in the ith link
	%rp is the position of the point p, projected in the inertial CCS -- as a [3x1] matrix.
	[J0p, Jmp]=Jacob(rp,r0,rL,P0,pm,i,robot);
    
%% ���ٶ�
