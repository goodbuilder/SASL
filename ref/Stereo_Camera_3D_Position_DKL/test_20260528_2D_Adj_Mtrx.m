close all;
clear all;
clc;

% test_20260324_Stereo_Camera_LSQ_Attitude_KF_20260409 코드에서 출발


%% 영상센서 2D 평면 위에서 Centroid 상과 Bbox center의 차이 보정에 사용되는 파일의 경로
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

%%%%% Start Time Sync %%%%%
delta_T = 0.7;
time_A = time_A + delta_T;
time_B = time_B + delta_T;

% % % % % figure;
% % % % % subplot(2,2,1)
% % % % % plot(time_A, u_A(1,:))
% % % % % subplot(2,2,2)
% % % % % plot(time_A, u_A(2,:))
% % % % % subplot(2,2,3)
% % % % % plot(time_B, u_B(1,:))
% % % % % subplot(2,2,4)
% % % % % plot(time_B, u_B(1,:))
% % % % % 
% % % % % figure;
% % % % % subplot(3,1,1)
% % % % % plot(time_eVTOL, Position_eVTOL(1,:))
% % % % % subplot(3,1,2)
% % % % % plot(time_eVTOL, Position_eVTOL(2,:))
% % % % % subplot(3,1,3)
% % % % % plot(time_eVTOL, Position_eVTOL(3,:))


%% Camera Information %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%% Intrinsic Parameter %%%%%
res = [1920; 1080]; % width; height | pixel
sensor_size_diag = 6.46; % diagonal length | mm
sensor_size = [sqrt(sensor_size_diag^2/((res(2)/res(1))^2+1));
    sqrt(sensor_size_diag^2/((res(1)/res(2))^2+1))]; % width; height | mm
f_0 = [4.35; 4.35]; % width; height | mm
zoom = 8; % Optical Zoom | times

%%%%% Extrinsic Parameter %%%%%
Loc_A = [-12500; 0; 0]; % X; Y; Z | mm
Att_A = [70; 0; 180]/180*pi; % Roll; Pitch; Yaw | rad
Loc_B = [12500; 0; 0]; % X; Y; Z | mm
Att_B = [70; 0; 180]/180*pi; % Roll; Pitch; Yaw | rad

%%%%% Camera Model Generation %%%%%
A = Stereo_Camera(res, sensor_size, f_0, zoom, Loc_A, Att_A);
B = Stereo_Camera(res, sensor_size, f_0, zoom, Loc_B, Att_B);


%% Data Generation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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


%%%%% Time Sync and Matching %%%%%
% 높은 샘플링 주파수의 연속적인 데이터인 ideal 값
% 낮은 샘플링 주파수의 불연속적(일 수 있는) 데이터인 Yolo bbox 중점값
% Time Sync 및 matching
[u_A_grid] = meas_grid_mapping(time_A, time_A_ideal, u_A_ideal);
[u_B_grid] = meas_grid_mapping(time_B, time_B_ideal, u_B_ideal);
[Position_eVTOL_grid] = meas_grid_mapping(time_A, time_eVTOL, Position_eVTOL);

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


%% Centroid의 2D 위치와 Bbox center 위치 오차 보정 %%%%%%%%%%%%%%%%%%%%%%
[Mtrx1_A, Mtrx2_A] = Centroid_Bbox_Adjustment(u_A, u_A_grid);
[Mtrx1_B, Mtrx2_B] = Centroid_Bbox_Adjustment(u_B, u_B_grid);

%%%%% 위치 오차 보정 Matrix 저장 %%%%%
Mtrx_for_save = [Mtrx1_A Mtrx2_A; Mtrx1_B Mtrx2_B];
writematrix(Mtrx_for_save, 'Cent_Bbox_Adj_Mtrx.csv');


%%%%% 3D Position Reconstruction from Gazebo Camera Image Plane Data (YOLO) + bbox adjustment %%%%%
% Extrinsic parameter 오차 보정X, Gazebo 내에서 YOLO 알고리즘에 따른 bbox 중심점 정보 기반
% Bbox 오차 보정한 결과 바탕으로 측정한 위치
%%%%% 위치 오차 보정 Matrix 불러오기 %%%%%
Mtrx_Adj = readmatrix('Cent_Bbox_Adj_Mtrx.csv');

P_pred_Adj = [];
for i=1:size(time_A,2)
    u_A(:,i) = Mtrx_Adj(1:3,1:3)*u_A(:,i) + Mtrx_Adj(1:3,4);
    u_B(:,i) = Mtrx_Adj(4:6,1:3)*u_B(:,i) + Mtrx_Adj(4:6,4);
    [P_pred_Adj(:,i)] = Position_Reconst_from_SteroCamera_Data(u_A(:,i), u_B(:,i), A, B);
end
time_P_pred_Adj = time_A;

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
    
    % eVTOL 3D 연속데이터 로드    
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

function [Mtrx1, Mtrx2] = Centroid_Bbox_Adjustment(u_bbox, u_cent)
    Mtrx_polyfit = zeros(2,2);
    for i=1:2
        Mtrx_polyfit(i,:) = polyfit(u_bbox(i,:), u_cent(i,:), 1);
    end
    Mtrx1 = [Mtrx_polyfit(1,1) 0 0; 0 Mtrx_polyfit(2,1) 0; 0 0 1];
    Mtrx2 = [Mtrx_polyfit(:,2); 0];
end