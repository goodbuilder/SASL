function [Camera] = Stereo_Camera(Resolution, Sensor_Size, Focal_Length, Zoom, Position, Attitude)
    % Resolution ----- width; height | pixel
    % Sensor_Size ----- width; height | mm
    % Focal_Length ----- width; height @ 1x zoom | mm
    % Zoom ----- Optical | times
    % Pos
    % Att

    % % Sensor
    Camera.Resolution = Resolution;
    Pixel_Size = Sensor_Size./Resolution; % width; height | mm/pixel
    c = Resolution/2; % width; height | pixel

    % % Optical 
    f = Focal_Length*Zoom; % width, height | mm
    fc = f./Pixel_Size; % width, height | pixel

    % % Intrinsic Matrix
    Camera.K = [fc(1) 0 c(1); 0 fc(2) c(2); 0 0 1];


    % % Installation
    % Position
    Camera.Position = Position;
    % Attitude
    Camera.Attitude = Attitude;
    % Rotation Matrix (Euler Angle)
    Camera.Rx = [1 0 0; 0 cos(Attitude(1)) -sin(Attitude(1)); 0 sin(Attitude(1)) cos(Attitude(1))];
    Camera.Ry = [cos(Attitude(2)) 0 sin(Attitude(2)); 0 1 0; -sin(Attitude(2)) 0 cos(Attitude(2))];
    Camera.Rz = [cos(Attitude(3)) -sin(Attitude(3)) 0; sin(Attitude(3)) cos(Attitude(3)) 0; 0 0 1];
    % % ROI Vector w.r.t. Sensor Frame
    ROI_Vector = [-Sensor_Size(1)/2/f(1) Sensor_Size(1)/2/f(1)...
        Sensor_Size(1)/2/f(1) -Sensor_Size(1)/2/f(1);
        -Sensor_Size(2)/2/f(2) -Sensor_Size(2)/2/f(2)...
        Sensor_Size(2)/2/f(2) Sensor_Size(2)/2/f(2);
        1 1 1 1]; % ROI Vector w.r.t. Sensor Frame for 1 mm Depth
    Camera.ROI = Camera.Rz*Camera.Ry*Camera.Rx*ROI_Vector; % ROI Vector w.r.t. Ground Frame for 1 mm Depth

end