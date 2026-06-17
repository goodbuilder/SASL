close all;
clear all;
clc;

% test_20260324_Stereo_Camera_LSQ_Attitude_KF_20260409 코드에서 출발

%%%%% 저장된 데이터 경로
% Data = readmatrix("Gazebo_Sim\time (csv)\4x\pitch0_yaw0.csv");
% Data = readmatrix("Gazebo_Sim\time (csv)\4x\yaw0_pitch1.0.csv");
% Data2 = readmatrix("Gazebo_Sim\trajectory_full\4x\yaw0_pitch1.0.csv");
% Data = readmatrix("Gazebo_Sim\time (csv)\4x\pitch0_yaw0.4.csv");
% Data2 = readmatrix("Gazebo_Sim\trajectory_full\4x\pitch0_yaw0.4.csv");
% Data = readmatrix("Gazebo_Sim\time (csv)\8x\pitch0_yaw0.csv");
% Data2 = readmatrix("Gazebo_Sim\trajectory_full\8x\pitch0_yaw0.csv");
% Data = readmatrix("Gazebo_Sim\time (csv)\8x\yaw0_pitch1.0.csv");
% Data2 = readmatrix("Gazebo_Sim\trajectory_full\8x\yaw0_pitch1.0.csv");
% Data = readmatrix("Gazebo_Sim\time (csv)\8x\pitch0_yaw1.0.csv");
% Data2 = readmatrix("Gazebo_Sim\trajectory_full\8x\pitch0_yaw1.0.csv");

%%%%% Experimental or Simulation Data Load %%%%%
DataLoc01 = "Gazebo_Sim\time (csv)\8x\pitch0_yaw0.csv";
DataLoc02 = "Gazebo_Sim\trajectory_full\8x\pitch0_yaw0.csv";
[time_A, u_A, time_B, u_B, time_eVTOL, Position_eVTOL] = Dataload(DataLoc01, DataLoc02);

% % % % % figure;
% % % % % subplot(2,2,1)
% % % % % plot(time_A, u_A(1,:))
% % % % % subplot(2,2,2)
% % % % % plot(time_A, u_A(2,:))
% % % % % subplot(2,2,3)
% % % % % plot(time_B, u_B(1,:))
% % % % % subplot(2,2,4)
% % % % % plot(time_B, u_B(1,:))
% % % % % figure;
% % % % % subplot(3,1,1)
% % % % % plot(time_eVTOL, Position_eVTOL(1,:))
% % % % % subplot(3,1,2)
% % % % % plot(time_eVTOL, Position_eVTOL(2,:))
% % % % % subplot(3,1,3)
% % % % % plot(time_eVTOL, Position_eVTOL(3,:))

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% Camera Information %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%% Intrinsic Parameter %%%%%
res = [1920; 1080]; % width; height | pixel
sensor_size_diag = 6.46; % diagonal length | mm
sensor_size = [sqrt(sensor_size_diag^2/((res(2)/res(1))^2+1));
    sqrt(sensor_size_diag^2/((res(1)/res(2))^2+1))]; % width; height | mm
f_0 = [4.35; 4.35]; % width; height | mm
zoom = 8; % times %%%%%

%%%%% Extrinsic Parameter %%%%%
Loc_A = [-12500; 0; 0]; % X; Y; Z | mm
Att_A = [70; 0; 180]/180*pi; % Roll; Pitch; Yaw | rad
Loc_B = [12500; 0; 0]; % X; Y; Z | mm
Att_B = [70; 0; 180]/180*pi; % Roll; Pitch; Yaw | rad

%%%%% Camera Model Generation %%%%%
A = Stereo_Camera(res, sensor_size, f_0, zoom, Loc_A, Att_A);
B = Stereo_Camera(res, sensor_size, f_0, zoom, Loc_B, Att_B);

%%%%% Ideal Stereo Camera Image Plane Data Generation from Position_eVTOL %%%%%
% Extrinsic parameter 오차, bbox 중심점에 따른 오차 고려X
[time_A_ideal, u_A_ideal, time_B_ideal, u_B_ideal] = Ideal_StereoCamera_Data_Gen(time_eVTOL, Position_eVTOL, A, B);

% % % % % figure;
% % % % % subplot(2,2,1)
% % % % % plot(time_A_ideal, u_A_ideal(1,:))
% % % % % ylim([0 1920])
% % % % % subplot(2,2,2)
% % % % % plot(time_A_ideal, u_A_ideal(2,:))
% % % % % ylim([0 1080])
% % % % % subplot(2,2,3)
% % % % % plot(time_B_ideal, u_B_ideal(1,:))
% % % % % ylim([0 1920])
% % % % % subplot(2,2,4)
% % % % % plot(time_B_ideal, u_B_ideal(2,:))
% % % % % ylim([0 1080])

%%%%% Ideal 3D Position Reconstruction from Ideal Stereo Camera Image Plane Data %%%%%
% 이상적으로 측정되어야 할 위치, camera sensor의 discretization에 따른 오차만 반영
P_pred_ideal = [];
for i=1:size(time_A_ideal,2)
    [P_pred_ideal(:,i)] = Position_Reconst_from_SteroCamera_Data(u_A_ideal(:,i), u_B_ideal(:,i), A, B);
end
time_P_pred_ideal = time_A_ideal;

% % % % % figure;
% % % % % subplot(3,1,1)
% % % % % plot(time_P_pred_ideal, P_pred_ideal(1,:))
% % % % % subplot(3,1,2)
% % % % % plot(time_P_pred_ideal, P_pred_ideal(2,:))
% % % % % subplot(3,1,3)
% % % % % plot(time_P_pred_ideal, P_pred_ideal(3,:))

%%%%% 3D Position Reconstruction from Gazebo Camera Image Plane Data (YOLO) %%%%%
% Extrinsic parameter 오차 보정X, Gazebo 내에서 YOLO 알고리즘에 따른 bbox 중심점 정보 기반으로 측정한 위치
P_pred = [];
for i=1:size(time_A,2)
    [P_pred(:,i)] = Position_Reconst_from_SteroCamera_Data(u_A(:,i), u_B(:,i), A, B);
end
time_P_pred = time_A;

% % % % % figure;
% % % % % subplot(3,1,1)
% % % % % plot(time_P_pred, P_pred(1,:))
% % % % % subplot(3,1,2)
% % % % % plot(time_P_pred, P_pred(2,:))
% % % % % subplot(3,1,3)
% % % % % plot(time_P_pred, P_pred(3,:))


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% Time Sync and Matching
% 높은 샘플링 주파수의 연속적인 데이터인 ideal 값
% 낮은 샘플링 주파수의 불연속적(일 수 있는) 데이터인 Yolo bbox 중점값
% Time Sync 및 matching
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
delta_T = 0.7;
time_A = time_A + delta_T;
time_B = time_B + delta_T;
time_P_pred = time_A;
% 연속적이고 균일한 이상적 값을 불연속적/불균일한 Yolo값에 그리드 매칭
[u_A_grid] = meas_grid_mapping(time_A, time_A_ideal, u_A_ideal);
[u_B_grid] = meas_grid_mapping(time_B, time_B_ideal, u_B_ideal);

% % % % % figure;
% % % % % subplot(2,2,1)
% % % % % plot(time_A_ideal, u_A_ideal(1,:), 'r-')
% % % % % hold on;
% % % % % plot(time_A, u_A(1,:), 'bs')
% % % % % plot(time_A, u_A_grid(1,:), 'g^')
% % % % % ylim([0 1920])
% % % % % subplot(2,2,2)
% % % % % plot(time_A_ideal, u_A_ideal(2,:), 'r-')
% % % % % hold on;
% % % % % plot(time_A, u_A(2,:), 'bs')
% % % % % plot(time_A, u_A_grid(2,:), 'g^')
% % % % % ylim([0 1080])
% % % % % subplot(2,2,3)
% % % % % plot(time_B_ideal, u_B_ideal(1,:), 'r-')
% % % % % hold on;
% % % % % plot(time_B, u_B(1,:), 'bs')
% % % % % plot(time_B, u_B_grid(1,:), 'g^')
% % % % % ylim([0 1920])
% % % % % subplot(2,2,4)
% % % % % plot(time_B_ideal, u_B_ideal(2,:), 'r-')
% % % % % hold on;
% % % % % plot(time_B, u_B(2,:), 'bs')
% % % % % plot(time_B, u_B_grid(2,:), 'g^')
% % % % % ylim([0 1080])


% %%%%% Bounding Box center 측정 오차 보정 값
Adj_A = u_A_grid - u_A;
Adj_B = u_B_grid - u_B;


Adj_2D_A = zeros(2,2);
for i=1:2
    Adj_2D_A(i,:) = polyfit(u_A(i,:), u_A_grid(i,:), 1);
end
% Adj_2D_A(:,1) = Adj_2D_A(:,1) + [1; 1];
u_A_Adj(1,:) = Adj_2D_A(1,1)*u_A(1,:) + Adj_2D_A(1,2);
u_A_Adj(2,:) = Adj_2D_A(2,1)*u_A(2,:) + Adj_2D_A(2,2);
u_A_Adj(3,:) = ones(1,size(u_A,2));

Adj_2D_B = zeros(2,2);
for i=1:2
    Adj_2D_B(i,:) = polyfit(u_B(i,:), u_B_grid(i,:), 1);
end
% Adj_2D_B(:,1) = Adj_2D_B(:,1) + [1; 1];
u_B_Adj(1,:) = Adj_2D_B(1,1)*u_B(1,:) + Adj_2D_B(1,2);
u_B_Adj(2,:) = Adj_2D_B(2,1)*u_B(2,:) + Adj_2D_B(2,2);
u_B_Adj(3,:) = ones(1,size(u_B,2));

% % % % % figure;
% % % % % subplot(2,4,1)
% % % % % plot(u_A(1,:), Adj_A(1,:), 'b.', 'LineWidth', 2)
% % % % % hold on;
% % % % % p(1,:) = polyfit(u_A(1,:), Adj_A(1,:), 1);
% % % % % plot(u_A(1,:), p(1,1)*u_A(1,:)+p(1,2), 'r-')
% % % % % 
% % % % % subplot(2,4,2)
% % % % % plot(time_A, u_A(1,:), 'bs')
% % % % % hold on;
% % % % % plot(time_A, u_A_grid(1,:), 'g^')
% % % % % plot(time_A, u_A_Adj(1,:), 'r.', 'LineWidth', 2)
% % % % % 
% % % % % subplot(2,4,3)
% % % % % plot(u_A(2,:), Adj_A(2,:), 'b.', 'LineWidth', 2)
% % % % % hold on;
% % % % % p(1,:) = polyfit(u_A(2,:), Adj_A(2,:), 1);
% % % % % plot(u_A(2,:), p(1,1)*u_A(2,:)+p(1,2), 'r-')
% % % % % 
% % % % % subplot(2,4,4)
% % % % % plot(time_A, u_A(2,:), 'bs')
% % % % % hold on;
% % % % % plot(time_A, u_A_grid(2,:), 'g^')
% % % % % plot(time_A, u_A_Adj(2,:), 'r.', 'LineWidth', 2)
% % % % % 
% % % % % subplot(2,4,5)
% % % % % plot(u_B(1,:), Adj_B(1,:), 'b.', 'LineWidth', 2)
% % % % % hold on;
% % % % % p(1,:) = polyfit(u_B(1,:), Adj_B(1,:), 1);
% % % % % plot(u_B(1,:), p(1,1)*u_B(1,:)+p(1,2), 'r-')
% % % % % 
% % % % % subplot(2,4,6)
% % % % % plot(time_B, u_B(1,:), 'bs')
% % % % % hold on;
% % % % % plot(time_B, u_B_grid(1,:), 'g^')
% % % % % plot(time_B, u_B_Adj(1,:), 'r.', 'LineWidth', 2)
% % % % % 
% % % % % subplot(2,4,7)
% % % % % plot(u_B(2,:), Adj_B(2,:), 'b.', 'LineWidth', 2)
% % % % % hold on;
% % % % % p(1,:) = polyfit(u_B(2,:), Adj_B(2,:), 1);
% % % % % plot(u_B(2,:), p(1,1)*u_B(2,:)+p(1,2), 'r-')
% % % % % 
% % % % % subplot(2,4,8)
% % % % % plot(time_B, u_B(2,:), 'bs')
% % % % % hold on;
% % % % % plot(time_B, u_B_grid(2,:), 'g^')
% % % % % plot(time_B, u_B_Adj(2,:), 'r.', 'LineWidth', 2)

%%%%% 3D Position Reconstruction from Gazebo Camera Image Plane Data (YOLO) + bbox adjustment %%%%%
% Extrinsic parameter 오차 보정X, Gazebo 내에서 YOLO 알고리즘에 따른 bbox 중심점 정보 기반
% Bbox 오차 보정한 결과 바탕으로 측정한 위치
P_pred_Adj = [];
for i=1:size(time_A,2)
    [P_pred_Adj(:,i)] = Position_Reconst_from_SteroCamera_Data(u_A_Adj(:,i), u_B_Adj(:,i), A, B);
end
time_P_pred_Adj = time_A;


[Position_eVTOL_grid] = meas_grid_mapping(time_A, time_eVTOL, Position_eVTOL);




figure;
subplot(3,2,1)
plot(time_eVTOL, Position_eVTOL(1,:)/1000, 'k-')
hold on;
plot(time_P_pred, P_pred(1,:)/1000, 'g^')
plot(time_P_pred_Adj, P_pred_Adj(1,:)/1000, 'bo')

subplot(3,2,2)
plot(time_eVTOL, zeros(1,size(time_eVTOL,2)), 'k-')
hold on;
plot(time_P_pred, (P_pred(1,:) - Position_eVTOL_grid(1,:))/1000, 'g^')
plot(time_P_pred_Adj, (P_pred_Adj(1,:) - Position_eVTOL_grid(1,:))/1000, 'bo')
title(sprintf('max error : %0.2f m', max((P_pred_Adj(1,:) - Position_eVTOL_grid(1,:))/1000)))

subplot(3,2,3)
plot(time_eVTOL, Position_eVTOL(2,:)/1000, 'k-')
hold on;
plot(time_P_pred, P_pred(2,:)/1000, 'g^')
plot(time_P_pred_Adj, P_pred_Adj(2,:)/1000, 'bo')

subplot(3,2,4)
plot(time_eVTOL, zeros(1,size(time_eVTOL,2)), 'k-')
hold on;
plot(time_P_pred, (P_pred(2,:) - Position_eVTOL_grid(2,:))/1000, 'g^')
plot(time_P_pred_Adj, (P_pred_Adj(2,:) - Position_eVTOL_grid(2,:))/1000, 'bo')
title(sprintf('max error : %0.2f m', max((P_pred_Adj(2,:) - Position_eVTOL_grid(2,:))/1000)))

subplot(3,2,5)
plot(time_eVTOL, Position_eVTOL(3,:), 'k-')
hold on;
plot(time_P_pred, P_pred(3,:), 'g^')
plot(time_P_pred_Adj, P_pred_Adj(3,:), 'bo')

subplot(3,2,6)
plot(time_eVTOL, zeros(1,size(time_eVTOL,2)), 'k-')
hold on;
plot(time_P_pred, (P_pred(3,:) - Position_eVTOL_grid(3,:))/1000, 'g^')
plot(time_P_pred_Adj, (P_pred_Adj(3,:) - Position_eVTOL_grid(3,:))/1000, 'bo')
title(sprintf('max error : %0.2f m', max((P_pred_Adj(3,:) - Position_eVTOL_grid(3,:))/1000)))



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%% 함수모음 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [time_A, u_A_gazebo, time_B, u_B_gazebo, time_eVTOL, Position_eVTOL] = Dataload(DataLoc01, DataLoc02)
    %%%%% Input
    % DataLoc01 : Gazebo 시뮬레이션 결과 - 스테레오카메라 
    % DataLoc02 : Gazebo 시뮬레이션 결과 - 비행체 연속 위치정보
    % DataLoc01과 02는 분리될 필요 없고 통합되면 좋음) 
    %%%%% Output
    % time_A : Stereo Camera #1 영상에 매칭되는 시간정보
    % u_A_gazebo : Stereo Camera #1 영상에 Yolo 적용한 Bbox 중심점 [u; v; 1] | pixel
    % time_B : Stereo Camera #1 영상에 매칭되는 시간정보
    % u_B_gazebo : Stereo Camera #2 영상에 Yolo 적용한 Bbox 중심점 [u; v; 1] | pixel
    % time_eVTOL : eVTOL의 자체 위치정보에 매칭되는 시간정보
    % Position_eVTOL : eVTOL의 자체 위치정보 [X; Y; Z] | mm

    %%%%% Data Load
    Data = readmatrix(DataLoc01);
    Data2 = readmatrix(DataLoc02);

    time_A = Data(:,1)'; % time | sec
    time_B = Data(:,1)'; % time | sec
    % P_real = Data(:,2:4)'*1000; % X; Y; Z | mm
    % P_meas_gazebo = Data(:,5:7)'*1000; % X; Y; Z | mm -> Gazebo에서 측정한값
    u_A_gazebo = Data(:,8:9)'; % u_x; u_y | pixel
    u_A_gazebo(1,:) = 1920-u_A_gazebo(1,:); % 좌표 방향 Gazebo -> Matlab 변환
    u_A_gazebo(2,:) = 1080-u_A_gazebo(2,:); % 좌표 방향 Gazebo -> Matlab 변환
    u_A_gazebo(3,:) = ones(1,size(time_A,2)); % 벡터 사이즈 맞춤
    u_B_gazebo = Data(:,10:11)'; % u_x; u_y | pixel
    u_B_gazebo(1,:) = 1920-u_B_gazebo(1,:); % 좌표 방향 Gazebo -> Matlab 변환
    u_B_gazebo(2,:) = 1080-u_B_gazebo(2,:); % 좌표 방향 Gazebo -> Matlab 변환
    u_B_gazebo(3,:) = ones(1,size(time_B,2)); % 벡터 사이즈 맞춤
    
    % % % % % % % % % % % % u_A_gazebo(1,:) = u_A_gazebo(1,:)-(0.0674*u_A_gazebo(1,:)-61.3967); % Bbox Center 오차 보정
    % % % % % % % % % % % % u_A_gazebo(2,:) = u_A_gazebo(2,:)-(0.0696*u_A_gazebo(2,:)-29.5846); % Bbox Center 오차 보정
    % % % % % % % % % % % % u_B_gazebo(1,:) = u_B_gazebo(1,:)-(0.0633*u_B_gazebo(1,:)-56.6175); % Bbox Center 오차 보정
    % % % % % % % % % % % % u_B_gazebo(2,:) = u_B_gazebo(2,:)-(0.0708*u_B_gazebo(2,:)-30.5182); % Bbox Center 오차 보정
    
    time_eVTOL = Data2(:,1)'; % time (continuous) | sec
    Position_eVTOL = Data2(:,2:4)'*1000; % X; Y; Z (continuous) | mm

end

function [time_A_ideal, u_A_ideal, time_B_ideal, u_B_ideal] = Ideal_StereoCamera_Data_Gen(time, Position, CamA, CamB)
    % 3차원 위치정보, 스테레오 카메라 모델을 통해 각 카메라의 이미지평면 상 이상적인 위치 데이터 생성
    u_A_ideal = [];
    u_B_ideal = [];
    for i=1:1:size(Position,2)
        % Stereo Camera Signal Generation
        r_pcA_cA = CamA.Rx'*CamA.Ry'*CamA.Rz'*(Position(:,i) - CamA.Position);
        r_pcB_cB = CamB.Rx'*CamB.Ry'*CamB.Rz'*(Position(:,i) - CamB.Position);
    
        % Round for Sensor Pixel!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        u_A_ideal = [u_A_ideal round(1/r_pcA_cA(3)*CamA.K*r_pcA_cA)];
        u_B_ideal = [u_B_ideal round(1/r_pcB_cB(3)*CamB.K*r_pcB_cB)];
    end
    time_A_ideal = time;
    time_B_ideal = time;
end

function [Position_Reconst] = Position_Reconst_from_SteroCamera_Data(u_A, u_B, CamA, CamB)
    %%%%% 3D Position Reconstruction %%%%%
    dA_cA = CamA.K\u_A; % inv(A.K)*u_A
    dB_cB = CamB.K\u_B; % inv(B.K)*u_B
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    dA = CamA.Rz*CamA.Ry*CamA.Rx*dA_cA;
    dB = CamB.Rz*CamB.Ry*CamB.Rx*dB_cB;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    B0 = [-dot(CamB.Position - CamA.Position, dA); -dot(CamB.Position - CamA.Position, dB)];
    A0 = [-dot(dA, dA) dot(dB, dA); -dot(dA, dB) dot(dB, dB)];
    s0 = A0\B0;

    P_A = CamA.Position + s0(1)*dA;
    P_B = CamB.Position + s0(2)*dB;

    Position_Reconst = (P_A + P_B)/2;

end

function [grid] = meas_grid_mapping(time_ref, time_target, data_target)
    % measurement-to-grid mapping:
    % each actual measurement is assigned to nearest grid index
    grid = nan(3,size(time_ref, 2));
    % mask = false(1,size(time_ref, 2));
    
    for i = 1:size(time_ref,2)
        [~, k] = min(abs(time_ref(i) - time_target));
        grid(:,i) = data_target(:,k);
        % mask(k) = true;
    end

end



% % % % % % 
% % % % % % %%%%% Camera A의 2D Image Plane 상의 bbox 중심점의 픽셀 좌표에 대한 Residual 계산
% % % % % % % 자세각 업데이트 (Lease Square Method)
% % % % % % Target_Var_0 = [Att_A; 0; 0; 0; 0];
% % % % % % Position = Loc_A;
% % % % % % K = A.K;
% % % % % % u = u_A_gazebo;
% % % % % % Target_Var_A_lsq = lsqnonlin(@(Target_Var) myResidual01(Target_Var, Position, K, P_real, u), Target_Var_0);
% % % % % % Att_A_lsq = Target_Var_A_lsq(1:3,1)*180/pi
% % % % % % Delta_2D_A_lsq = [Target_Var_A_lsq(4,1) Target_Var_A_lsq(5,1); Target_Var_A_lsq(6,1) Target_Var_A_lsq(7,1)]
% % % % % % Att_A*180/pi
% % % % % % 
% % % % % % %%%%% Camera B의 2D Image Plane 상의 bbox 중심점의 픽셀 좌표에 대한 Residual 계산
% % % % % % % 자세각 업데이트 (Lease Square Method)
% % % % % % Target_Var_0 = [Att_B; 0; 0; 0; 0];
% % % % % % Position = Loc_B;
% % % % % % K = B.K;
% % % % % % u = u_B_gazebo;
% % % % % % Target_Var_B_lsq = lsqnonlin(@(Target_Var) myResidual01(Target_Var, Position, K, P_real, u), Target_Var_0);
% % % % % % Att_B_lsq = Target_Var_B_lsq(1:3,1)*180/pi
% % % % % % Delta_2D_B_lsq = [Target_Var_B_lsq(4,1) Target_Var_B_lsq(5,1); Target_Var_B_lsq(6,1) Target_Var_B_lsq(7,1)]
% % % % % % Att_B*180/pi
% % % % % % 
% % % % % % %%%%% Stereo Camera (A, B) 각각의 2D Image Plane 상의 bbox 중심점의 픽셀 좌표에 대한 Residual 계산
% % % % % % % 자세각 동시 업데이트 (Lease Square Method)
% % % % % % Attitude0 = [Att_A_lsq; Att_B_lsq];
% % % % % % Position = [Loc_A; Loc_B];
% % % % % % K = [A.K; B.K];
% % % % % % u = [u_A_gazebo; u_B_gazebo];
% % % % % % Att_lsq = lsqnonlin(@(Attitude) myResidual02(Attitude, Position, K, P_real, u), Attitude0);
% % % % % % Att_A*180/pi
% % % % % % Att_B*180/pi
% % % % % % Att_lsq*180/pi
% % % % % % 
% % % % % % %%%%% 자세각 업데이트에 따른 Camera Model Generation Update
% % % % % % A = Stereo_Camera(res, sensor_size, f_0, zoom, Loc_A, Att_lsq(1:3,1));
% % % % % % B = Stereo_Camera(res, sensor_size, f_0, zoom, Loc_B, Att_lsq(4:6,1));
% % % % % % 
% % % % % % 
% % % % % % %%%%% Updated Attitude를 바탕으로 Gazebo 3D Position Reconstruction %%%%%
% % % % % % % 새로 업데이트 된 Attitude + Gazebo 탐지 bbox 기반으로 측정한 위치
% % % % % % P_pred_gazebo_lsq = [];
% % % % % % for i=1:1:size(P_real,2)
% % % % % %     dA_cA = A.K\u_A_gazebo(:,i); % inv(A.K)*u_A
% % % % % %     dB_cB = B.K\u_B_gazebo(:,i); % inv(B.K)*u_B
% % % % % %     %%%%%%%%%%%%%%%%%%%%%%%%%%%% 설치 각도 오차 넣어볼 수 있음
% % % % % %     dA = A.Rz*A.Ry*A.Rx*dA_cA;
% % % % % %     dB = B.Rz*B.Ry*B.Rx*dB_cB;
% % % % % %     %%%%%%%%%%%%%%%%%%%%%%%%%%%% 설치 위치 오차 넣어볼 수 있음
% % % % % %     B0 = [-dot(B.Position - A.Position, dA); -dot(B.Position - A.Position, dB)];
% % % % % %     A0 = [-dot(dA, dA) dot(dB, dA); -dot(dA, dB) dot(dB, dB)];
% % % % % %     s0 = A0\B0;
% % % % % % 
% % % % % %     P_A = A.Position + s0(1)*dA;
% % % % % %     P_B = B.Position + s0(2)*dB;
% % % % % % 
% % % % % %     P_pred_gazebo_lsq = [P_pred_gazebo_lsq (P_A + P_B)/2];
% % % % % % end
% % % % % % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % % % % % 
% % % % % % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % % % % % %%%%% 2D Plane Data에 Kalman Filter 적용
% % % % % % time(65)
% % % % % % time(220)
% % % % % % sigma_z0(1) = std(u_A_gazebo(1,65:220)-mean(u_A_gazebo(1,65:220)));
% % % % % % sigma_z0(2) = std(u_A_gazebo(2,65:220)-mean(u_A_gazebo(2,65:220)));
% % % % % % sigma_z0(3) = std(u_B_gazebo(1,65:220)-mean(u_B_gazebo(1,65:220)));
% % % % % % sigma_z0(4) = std(u_B_gazebo(2,65:220)-mean(u_B_gazebo(2,65:220)));
% % % % % % siama_z0_mean = mean(sigma_z0) 
% % % % % % [t_grid, u_A_filt] = ImagePlaneKF(time, u_A_gazebo(1:2,:));
% % % % % % [t_grid, u_B_filt] = ImagePlaneKF(time, u_B_gazebo(1:2,:));
% % % % % % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % % % % % 
% % % % % % %%%%% Updated Attitude를 바탕으로, Kalman Filter 적용된 Gazebo bbox 기반으로 Gazebo 3D Position Reconstruction %%%%%
% % % % % % % 새로 업데이트 된 Attitude + Kalman Filter + Gazebo 탐지 bbox 기반으로 측정한 위치 및 속도
% % % % % % P_pred_gazebo_lsq_KF = [];
% % % % % % V_pred_gazebo_lsq_KF = [];
% % % % % % u_A_gazebo_KF = [u_A_filt(1:2,:); ones(1, length(t_grid))];
% % % % % % u_B_gazebo_KF = [u_B_filt(1:2,:); ones(1, length(t_grid))];
% % % % % % for i=1:1:size(t_grid,1)
% % % % % %     dA_cA = A.K\u_A_gazebo_KF(:,i); % inv(A.K)*u_A
% % % % % %     dB_cB = B.K\u_B_gazebo_KF(:,i); % inv(B.K)*u_B
% % % % % %     %%%%%%%%%%%%%%%%%%%%%%%%%%%% 설치 각도 오차 넣어볼 수 있음
% % % % % %     dA = A.Rz*A.Ry*A.Rx*dA_cA;
% % % % % %     dB = B.Rz*B.Ry*B.Rx*dB_cB;
% % % % % %     %%%%%%%%%%%%%%%%%%%%%%%%%%%% 설치 위치 오차 넣어볼 수 있음
% % % % % %     B0 = [-dot(B.Position - A.Position, dA); -dot(B.Position - A.Position, dB)];
% % % % % %     A0 = [-dot(dA, dA) dot(dB, dA); -dot(dA, dB) dot(dB, dB)];
% % % % % %     s0 = A0\B0;
% % % % % % 
% % % % % %     P_A = A.Position + s0(1)*dA;
% % % % % %     P_B = B.Position + s0(2)*dB;
% % % % % % 
% % % % % %     P_pred_gazebo_lsq_KF = [P_pred_gazebo_lsq_KF (P_A + P_B)/2];
% % % % % % end
% % % % % % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % % % % % 
% % % % % % % %%%%% Bounding Box center 측정 오차 보정 값
% % % % % % % Adj_A = (mean(u_A_gazebo')-mean(u_A_ideal'))'
% % % % % % % Adj_B = (mean(u_B_gazebo')-mean(u_B_ideal'))'
% % % % % % 
% % % % % % Adj_A = u_A_gazebo - u_A_ideal;
% % % % % % Adj_B = u_B_gazebo - u_B_ideal;
% % % % % % 
% % % % % % figure;
% % % % % % subplot(2,2,1)
% % % % % % plot(u_A_gazebo(1,:), Adj_A(1,:), 'b.')
% % % % % % hold on;
% % % % % % p(1,:) = polyfit(u_A_gazebo(1,:), Adj_A(1,:), 1);
% % % % % % plot(u_A_gazebo(1,:), p(1,1)*u_A_gazebo(1,:)+p(1,2), 'r-')
% % % % % % subplot(2,2,2)
% % % % % % plot(u_A_gazebo(2,:), Adj_A(2,:), 'b.')
% % % % % % hold on;
% % % % % % p(2,:) = polyfit(u_A_gazebo(2,:), Adj_A(2,:), 1);
% % % % % % plot(u_A_gazebo(2,:), p(2,1)*u_A_gazebo(2,:)+p(2,2), 'r-')
% % % % % % 
% % % % % % subplot(2,2,3)
% % % % % % plot(u_B_gazebo(1,:), Adj_B(1,:), 'b.')
% % % % % % hold on;
% % % % % % p(3,:) = polyfit(u_B_gazebo(1,:), Adj_B(1,:), 1);
% % % % % % plot(u_B_gazebo(1,:), p(3,1)*u_B_gazebo(1,:)+p(3,2), 'r-')
% % % % % % subplot(2,2,4)
% % % % % % plot(u_B_gazebo(2,:), Adj_B(2,:), 'b.')
% % % % % % hold on;
% % % % % % p(4,:) = polyfit(u_B_gazebo(2,:), Adj_B(2,:), 1);
% % % % % % plot(u_B_gazebo(2,:), p(4,1)*u_B_gazebo(2,:)+p(4,2), 'r-')
% % % % % % 
% % % % % % 
% % % % % % figure;
% % % % % % subplot(2,4,1)
% % % % % % plot(time, u_A_ideal(1,:), 'bo')
% % % % % % hold on;
% % % % % % plot(time, u_A_gazebo(1,:), 'm^')
% % % % % % plot(t_grid', u_A_filt(1,:), 'r-', 'LineWidth', 1.5)
% % % % % % xlim([0 120])
% % % % % % ylim([0 1920])
% % % % % % xlabel('Time (s)')
% % % % % % ylabel('u_x  (Pixel)')
% % % % % % title('Camera A - X pixel')
% % % % % % legend('Pred Ideal', 'Meas Gazebo', 'Kalman Filter')
% % % % % % 
% % % % % % 
% % % % % % 
% % % % % % subplot(2,4,2)
% % % % % % plot(time, u_A_ideal(2,:), 'bo')
% % % % % % hold on;
% % % % % % plot(time, u_A_gazebo(2,:), 'm^')
% % % % % % plot(t_grid', u_A_filt(2,:), 'r-', 'LineWidth', 1.5)
% % % % % % xlim([0 120])
% % % % % % ylim([0 1080])
% % % % % % xlabel('Time (s)')
% % % % % % ylabel('u_y  (Pixel)')
% % % % % % title('Camera A - Y pixel')
% % % % % % 
% % % % % % subplot(2,4,3)
% % % % % % plot(time, u_B_ideal(1,:), 'bo')
% % % % % % hold on;
% % % % % % plot(time, u_B_gazebo(1,:), 'm^')
% % % % % % plot(t_grid', u_B_filt(1,:), 'r-', 'LineWidth', 1.5)
% % % % % % xlim([0 120])
% % % % % % ylim([0 1920])
% % % % % % xlabel('Time (s)')
% % % % % % ylabel('u_x  (Pixel)')
% % % % % % title('Camera B - X pixel')
% % % % % % 
% % % % % % subplot(2,4,4)
% % % % % % plot(time, u_B_ideal(2,:), 'bo')
% % % % % % hold on;
% % % % % % plot(time, u_B_gazebo(2,:), 'm^')
% % % % % % plot(t_grid', u_B_filt(2,:), 'r-', 'LineWidth', 1.5)
% % % % % % xlim([0 120])
% % % % % % ylim([0 1080])
% % % % % % xlabel('Time (s)')
% % % % % % ylabel('u_y  (Pixel)')
% % % % % % title('Camera B - Y pixel')
% % % % % % 
% % % % % % subplot(2,4,5)
% % % % % % plot(t_grid', u_A_filt(3,:), 'r-', 'LineWidth', 1.5)
% % % % % % xlabel('Time (s)')
% % % % % % ylabel('udot_x  (Pixel/s)')
% % % % % % title('Camera A - X_dot pixel')
% % % % % % 
% % % % % % subplot(2,4,6)
% % % % % % plot(t_grid', u_A_filt(4,:), 'r-', 'LineWidth', 1.5)
% % % % % % xlabel('Time (s)')
% % % % % % ylabel('udot_y  (Pixel/s)')
% % % % % % title('Camera A - Y_dot pixel')
% % % % % % 
% % % % % % subplot(2,4,7)
% % % % % % plot(t_grid', u_B_filt(3,:), 'r-', 'LineWidth', 1.5)
% % % % % % xlabel('Time (s)')
% % % % % % ylabel('udot_x  (Pixel/s)')
% % % % % % title('Camera B - X_dot pixel')
% % % % % % 
% % % % % % subplot(2,4,8)
% % % % % % plot(t_grid', u_B_filt(4,:), 'r-', 'LineWidth', 1.5)
% % % % % % xlabel('Time (s)')
% % % % % % ylabel('udot_y  (Pixel/s)')
% % % % % % title('Camera B - Y_dot pixel')
% % % % % % 
% % % % % % 
% % % % % % 
% % % % % % 
% % % % % % figure;
% % % % % % subplot(3,2,1);
% % % % % % plot(time, P_real(1,:)/1000, 'r.')
% % % % % % hold on;
% % % % % % plot(time_cont, P_real_cont(1,:)/1000, 'r-')
% % % % % % plot(time, P_pred_ideal(1,:)/1000, 'bo')
% % % % % % plot(time, P_pred_gazebo(1,:)/1000, 'gs')
% % % % % % plot(time, P_meas_gazebo(1,:)/1000, 'm^')
% % % % % % plot(time, P_pred_gazebo_lsq(1,:)/1000, 'k*')
% % % % % % plot(t_grid', P_pred_gazebo_lsq_KF(1,:)/1000, 'k-', 'LineWidth', 1.5)
% % % % % % xlabel('Time (s)')
% % % % % % ylabel('Position X  (m)')
% % % % % % legend('Real', 'Real Cont.', 'Pred Ideal', 'Pred Gazebo', 'Meas Gazebo', 'Pred Gazebo LeastSquare', 'Pred Gazebo LeastSquare KF')
% % % % % % 
% % % % % % subplot(3,2,2);
% % % % % % plot(time, P_pred_ideal(1,:)/1000-P_real(1,:)/1000, 'bo')
% % % % % % hold on;
% % % % % % plot(time, P_pred_gazebo(1,:)/1000-P_real(1,:)/1000, 'gs')
% % % % % % plot(time, P_meas_gazebo(1,:)/1000-P_real(1,:)/1000, 'm^')
% % % % % % plot(time, P_pred_gazebo_lsq(1,:)/1000-P_real(1,:)/1000, 'k*')
% % % % % % xlabel('Time (s)')
% % % % % % ylabel('Position Error X  (m)')
% % % % % % max(P_pred_gazebo_lsq(1,:)/1000-P_real(1,:)/1000)
% % % % % % 
% % % % % % subplot(3,2,3);
% % % % % % plot(time, P_real(2,:)/1000, 'r.')
% % % % % % hold on;
% % % % % % plot(time_cont, P_real_cont(2,:)/1000, 'r-')
% % % % % % plot(time, P_pred_ideal(2,:)/1000, 'bo')
% % % % % % plot(time, P_pred_gazebo(2,:)/1000, 'gs')
% % % % % % plot(time, P_meas_gazebo(2,:)/1000, 'm^')
% % % % % % plot(time, P_pred_gazebo_lsq(2,:)/1000, 'k*')
% % % % % % plot(t_grid', P_pred_gazebo_lsq_KF(2,:)/1000, 'k-', 'LineWidth', 1.5)
% % % % % % xlabel('Time (s)')
% % % % % % ylabel('Position Y  (m)')
% % % % % % 
% % % % % % subplot(3,2,4);
% % % % % % plot(time, P_pred_ideal(2,:)/1000-P_real(2,:)/1000, 'bo')
% % % % % % hold on;
% % % % % % plot(time, P_pred_gazebo(2,:)/1000-P_real(2,:)/1000, 'gs')
% % % % % % plot(time, P_meas_gazebo(2,:)/1000-P_real(2,:)/1000, 'm^')
% % % % % % plot(time, P_pred_gazebo_lsq(2,:)/1000-P_real(2,:)/1000, 'k*')
% % % % % % xlabel('Time (s)')
% % % % % % ylabel('Position Error Y  (m)')
% % % % % % max(P_pred_gazebo_lsq(2,:)/1000-P_real(2,:)/1000)
% % % % % % 
% % % % % % subplot(3,2,5);
% % % % % % plot(time, P_real(3,:)/1000, 'r.')
% % % % % % hold on;
% % % % % % plot(time_cont, P_real_cont(3,:)/1000, 'r-')
% % % % % % plot(time, P_pred_ideal(3,:)/1000, 'bo')
% % % % % % plot(time, P_pred_gazebo(3,:)/1000, 'gs')
% % % % % % plot(time, P_meas_gazebo(3,:)/1000, 'm^')
% % % % % % plot(time, P_pred_gazebo_lsq(3,:)/1000, 'k*')
% % % % % % plot(t_grid', P_pred_gazebo_lsq_KF(3,:)/1000, 'k-', 'LineWidth', 1.5)
% % % % % % xlabel('Time (s)')
% % % % % % ylabel('Position Z  (m)')
% % % % % % 
% % % % % % subplot(3,2,6);
% % % % % % plot(time, P_pred_ideal(3,:)/1000-P_real(3,:)/1000, 'bo')
% % % % % % hold on;
% % % % % % plot(time, P_pred_gazebo(3,:)/1000-P_real(3,:)/1000, 'gs')
% % % % % % plot(time, P_meas_gazebo(3,:)/1000-P_real(3,:)/1000, 'm^')
% % % % % % plot(time, P_pred_gazebo_lsq(3,:)/1000-P_real(3,:)/1000, 'k*')
% % % % % % xlabel('Time (s)')
% % % % % % ylabel('Position Error Z  (m)')
% % % % % % max(P_pred_gazebo_lsq(3,:)/1000-P_real(3,:)/1000)
% % % % % % 
% % % % % % 
% % % % % % 
% % % % % % 
% % % % % % 
% % % % % % 
% % % % % % 
% % % % % % 
% % % % % % 
% % % % % % 
% % % % % % 
% % % % % % 
% % % % % % 
% % % % % % 
% % % % % % 
% % % % % % 
% % % % % % 
% % % % % % 
% % % % % % 
% % % % % % 
% % % % % % 
% % % % % % 
% % % % % % 
% % % % % % 
% % % % % % 
% % % % % % 
% % % % % % 
% % % % % % function [t_grid, x_filt] = ImagePlaneKF(time, img_plane_2D)
% % % % % % 
% % % % % % 
% % % % % %     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % % % % %     %%%%% 1. build full time grid %%%%%
% % % % % %     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % % % % %     t_meas = (time - time(1))';
% % % % % %     z_meas = img_plane_2D';
% % % % % %     dt_grid = 0.01;
% % % % % %     sigma_a = 1.5; % Process Noise for Acceleration
% % % % % %     sigma_z = 1.1389; % Measurement Noise for Position
% % % % % % 
% % % % % %     t_grid = (t_meas(1):dt_grid:t_meas(end))';
% % % % % %     M = length(t_grid);
% % % % % % 
% % % % % %     % measurement-to-grid mapping:
% % % % % %     % each actual measurement is assigned to nearest grid index
% % % % % %     z_grid = nan(M,2);
% % % % % %     meas_mask = false(M,1);
% % % % % % 
% % % % % %     for i = 1:length(t_meas)
% % % % % %         [~, k] = min(abs(t_grid - t_meas(i)));
% % % % % %         z_grid(k,:) = z_meas(i,:);
% % % % % %         meas_mask(k) = true;
% % % % % %     end
% % % % % % 
% % % % % %     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % % % % %     %%%%% 2. model definition %%%%%
% % % % % %     % state = [x; y; vx; vy]
% % % % % %     % measurement = [x; y]
% % % % % %     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % % % % %     H = [1 0 0 0;
% % % % % %          0 1 0 0];
% % % % % %     R = (sigma_z^2) * eye(2);
% % % % % % 
% % % % % % 
% % % % % %     %%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % % % % %     %%%%% 3. initial state %%%%%
% % % % % %     %%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % % % % %     x0 = zeros(4,1);
% % % % % %     x0(1:2) = z_meas(1,:).';
% % % % % % 
% % % % % %     if length(t_meas) >= 2
% % % % % %         dt0 = max(t_meas(2) - t_meas(1), eps);
% % % % % %         vx0 = (z_meas(2,1) - z_meas(1,1)) / dt0;
% % % % % %         vy0 = (z_meas(2,2) - z_meas(1,2)) / dt0;
% % % % % %         x0(3:4) = [vx0; vy0];
% % % % % %     else
% % % % % %         x0(3:4) = [0;0];
% % % % % %     end
% % % % % % 
% % % % % %     P0 = diag([10^2, 10^2, 10^2, 10^2]);
% % % % % % 
% % % % % % 
% % % % % % 
% % % % % %     %%%%%%%%%%%%%%%%%%%%%%
% % % % % %     %%%%% 4. storage %%%%%
% % % % % %     %%%%%%%%%%%%%%%%%%%%%%
% % % % % %     x_pred = zeros(4,M);
% % % % % %     P_pred = zeros(4,4,M);
% % % % % % 
% % % % % %     x_filt = zeros(4,M);
% % % % % %     P_filt = zeros(4,4,M);
% % % % % % 
% % % % % %     F_all = zeros(4,4,M);
% % % % % %     Q_all = zeros(4,4,M);
% % % % % % 
% % % % % % 
% % % % % %     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % % % % %     %%%%% 5. forward Kalman filter %%%%%
% % % % % %     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % % % % %     x_prev = x0;
% % % % % %     P_prev = P0;
% % % % % % 
% % % % % %     for k = 1:M
% % % % % %         if k == 1
% % % % % %             dt = dt_grid;
% % % % % %         else
% % % % % %             dt = t_grid(k) - t_grid(k-1);
% % % % % %         end
% % % % % % 
% % % % % %         [F, Q] = getCVModel2D(dt, sigma_a);
% % % % % %         F_all(:,:,k) = F;
% % % % % %         Q_all(:,:,k) = Q;
% % % % % % 
% % % % % %         % predict
% % % % % %         x_minus = F * x_prev;
% % % % % %         P_minus = F * P_prev * F.' + Q;
% % % % % % 
% % % % % %         x_pred(:,k) = x_minus;
% % % % % %         P_pred(:,:,k) = P_minus;
% % % % % % 
% % % % % %         % update only if measurement exists
% % % % % %         if meas_mask(k)
% % % % % %             z = z_grid(k,:).';
% % % % % %             y = z - H*x_minus;
% % % % % %             S = H*P_minus*H.' + R;
% % % % % %             K = P_minus*H.'/S;
% % % % % % 
% % % % % %             x_plus = x_minus + K*y;
% % % % % %             P_plus = (eye(4) - K*H) * P_minus;
% % % % % %         else
% % % % % %             x_plus = x_minus;
% % % % % %             P_plus = P_minus;
% % % % % %         end
% % % % % % 
% % % % % %         x_filt(:,k) = x_plus;
% % % % % %         P_filt(:,:,k) = P_plus;
% % % % % % 
% % % % % %         x_prev = x_plus;
% % % % % %         P_prev = P_plus;
% % % % % %     end
% % % % % % end
% % % % % % 
% % % % % % 
% % % % % % 
% % % % % % 
% % % % % % 
% % % % % % 
% % % % % % 
% % % % % % 
% % % % % % % % % % % %%%%%%%%%%%%%%%%%%%%%%%%%%%
% % % % % % % % % % % %%%%% 6. RTS smoother %%%%%
% % % % % % % % % % % %%%%%%%%%%%%%%%%%%%%%%%%%%%
% % % % % % % 예를들어 0.2초 지연송출 하는 대신 훨씬 정확하게
% % % % % % % 지금 당장은 필요 없음
% % % % % % % % % % % x_smooth = zeros(4,M);
% % % % % % % % % % % P_smooth = zeros(4,4,M);
% % % % % % % % % % % 
% % % % % % % % % % % x_smooth(:,M) = x_filt(:,M);
% % % % % % % % % % % P_smooth(:,:,M) = P_filt(:,:,M);
% % % % % % % % % % % 
% % % % % % % % % % % for k = M-1:-1:1
% % % % % % % % % % %     Fkp1 = F_all(:,:,k+1);
% % % % % % % % % % %     Pk = P_filt(:,:,k);
% % % % % % % % % % %     Pk1_minus = P_pred(:,:,k+1);
% % % % % % % % % % % 
% % % % % % % % % % %     Ck = Pk * Fkp1.' / Pk1_minus;
% % % % % % % % % % % 
% % % % % % % % % % %     x_smooth(:,k) = x_filt(:,k) + Ck * (x_smooth(:,k+1) - x_pred(:,k+1));
% % % % % % % % % % %     P_smooth(:,:,k) = P_filt(:,:,k) + Ck * (P_smooth(:,:,k+1) - Pk1_minus) * Ck.';
% % % % % % % % % % % end
% % % % % % 
% % % % % % 
% % % % % % 
% % % % % % 
% % % % % % 
% % % % % % 
% % % % % % function [F, Q] = getCVModel2D(dt, sigma_a)
% % % % % % % Constant velocity model in 2D
% % % % % % % state = [x; y; vx; vy]
% % % % % % %
% % % % % % % x_{k+1} = F x_k + w_k
% % % % % % %
% % % % % % % continuous white acceleration noise -> discrete Q
% % % % % % 
% % % % % %     F = [1 0 dt 0;
% % % % % %          0 1 0 dt;
% % % % % %          0 0 1  0;
% % % % % %          0 0 0  1];
% % % % % % 
% % % % % %     q1 = [dt^4/4, dt^3/2;
% % % % % %           dt^3/2, dt^2];
% % % % % % 
% % % % % %     Q = sigma_a^2 * [q1(1,1) 0       q1(1,2) 0;
% % % % % %                      0       q1(1,1) 0       q1(1,2);
% % % % % %                      q1(2,1) 0       q1(2,2) 0;
% % % % % %                      0       q1(2,1) 0       q1(2,2)];
% % % % % % end
% % % % % % 
% % % % % % 
% % % % % % 
% % % % % % 
% % % % % % 
% % % % % % 
% % % % % % 
% % % % % % 
% % % % % % 
% % % % % % 
% % % % % % %%%%% 개별 Cam의 2D 평면 Residual 구하는 함수
% % % % % % function [r] = myResidual01(Target_Var, Position, K, P_real, u)
% % % % % % 
% % % % % %     % Delta_2D = [D2_11 D2_12; D2_21 D2_22] : bbox center 위치에 따른 보정계수 4개
% % % % % %     Attitude = Target_Var(1:3,1);
% % % % % %     Delta_2D = [Target_Var(4) Target_Var(5); Target_Var(6) Target_Var(7)];
% % % % % %     Rx = [1 0 0; 0 cos(Attitude(1)) -sin(Attitude(1)); 0 sin(Attitude(1)) cos(Attitude(1))];
% % % % % %     Ry = [cos(Attitude(2)) 0 sin(Attitude(2)); 0 1 0; -sin(Attitude(2)) 0 cos(Attitude(2))];
% % % % % %     Rz = [cos(Attitude(3)) -sin(Attitude(3)) 0; sin(Attitude(3)) cos(Attitude(3)) 0; 0 0 1];
% % % % % % 
% % % % % %     r_pcX_cX = Rx'*Ry'*Rz'*(P_real - Position);
% % % % % %     u_X_ideal = K*r_pcX_cX./r_pcX_cX(3,:);
% % % % % %     u_X_ideal(1,:) = u_X_ideal(1,:) - (Delta_2D(1,1)*u_X_ideal(1,:) + Delta_2D(1,2));
% % % % % %     u_X_ideal(2,:) = u_X_ideal(2,:) - (Delta_2D(2,1)*u_X_ideal(2,:) + Delta_2D(2,2));
% % % % % %     r = (u_X_ideal-u).*(u_X_ideal-u);
% % % % % % end
% % % % % % 
% % % % % % 
% % % % % % 
% % % % % % %%%%% 스테레오 Cam의 2D 평면 Residual을 통합하여 구하는 함수
% % % % % % function [r] = myResidual02(Attitude, Position, K, P_real, u)
% % % % % %     Rx_A = [1 0 0; 0 cos(Attitude(1)) -sin(Attitude(1)); 0 sin(Attitude(1)) cos(Attitude(1))];
% % % % % %     Ry_A = [cos(Attitude(2)) 0 sin(Attitude(2)); 0 1 0; -sin(Attitude(2)) 0 cos(Attitude(2))];
% % % % % %     Rz_A = [cos(Attitude(3)) -sin(Attitude(3)) 0; sin(Attitude(3)) cos(Attitude(3)) 0; 0 0 1];
% % % % % %     Rx_B = [1 0 0; 0 cos(Attitude(4)) -sin(Attitude(4)); 0 sin(Attitude(4)) cos(Attitude(4))];
% % % % % %     Ry_B = [cos(Attitude(5)) 0 sin(Attitude(5)); 0 1 0; -sin(Attitude(5)) 0 cos(Attitude(5))];
% % % % % %     Rz_B = [cos(Attitude(6)) -sin(Attitude(6)) 0; sin(Attitude(6)) cos(Attitude(6)) 0; 0 0 1];
% % % % % % 
% % % % % %     r_pcA_cA = Rx_A'*Ry_A'*Rz_A'*(P_real - Position(1:3,1));
% % % % % %     r_pcB_cB = Rx_B'*Ry_B'*Rz_B'*(P_real - Position(4:6,1));
% % % % % %     u_A_ideal = K(1:3,:)*r_pcA_cA./r_pcA_cA(3,:);
% % % % % %     u_B_ideal = K(4:6,:)*r_pcB_cB./r_pcB_cB(3,:);
% % % % % % 
% % % % % %     r = (u_A_ideal-u(1:3,:)).*(u_A_ideal-u(1:3,:)) + (u_B_ideal-u(4:6,:)).*(u_B_ideal-u(4:6,:));
% % % % % % end
% % % % % % 
% % % % % % 
% % % % % % 
% % % % % % 

