%% Initialization
   
redThresh = 0.25; % Threshold for red detection
% vidDevice = cv.VideoCapture; % Used by Webcam
vidDevice = cv.VideoCapture('tcp://192.168.1.1:5555'); %Used by ArDrone

hblob = vision.BlobAnalysis('AreaOutputPort', false, ... % Set blob analysis handling
                                'CentroidOutputPort', true, ... 
                                'BoundingBoxOutputPort', true', ...
                                'MinimumBlobArea', 1000, ...
                                'MaximumBlobArea', 30000, ...
                                'MaximumCount', 1);
hshapeinsRedBox = vision.ShapeInserter( 'Fill', true, ...
                                        'FillColor', 'Custom', ...
                                        'CustomFillColor', [1 0 0], ...
                                        'Opacity', 0.4);
htextins = vision.TextInserter('Text', 'Number of Red Object: %2d', ... % Set text for number of blobs
                                    'Location',  [7 2], ...
                                    'Color', [1 0 0], ... // red color
                                    'FontSize', 12);
htextinsCent = vision.TextInserter('Text', '+      X:%4d, Y:%4d', ... % set text for centroid
                                    'LocationSource', 'Input port', ...
                                    'Color', [1 1 0], ... // yellow color
                                    'FontSize', 14);
hVideoIn = vision.VideoPlayer('Name', 'Red Object Detection', ... % Output video player
                                'Position', [100 100 640+20 360+30]);
nFrame = 0; % Frame number initialization

    Kp = 0.3;
    SetPoint= 0;
    Previous = 0;


%% Processing Loop
while(1)
  
    rgbFrame = vidDevice.read; % Decodes and returns the grabbed video frame.
    J = rgb2gray(rgbFrame);
    diffFrame = imsubtract(rgbFrame(:,:,1) , J ); % Get red component of the image
    diffFrame = medfilt2(diffFrame, [3 3]); % Filter out the noise by using median filter
    binFrame = im2bw(diffFrame, redThresh); % Convert the image into binary image with the red objects as white
    [centroid, blobs] = step(hblob, binFrame); % Get the centroids and bounding boxes of the blobs
    centroid = int16(centroid); % Convert the centroids into Integer for further steps 
    rgbFrame(1:20,1:165,:) = 0; % put a black region on the output stream
    vidIn = step(hshapeinsRedBox, rgbFrame, blobs); % Instert the red box

    for object = 1:1:length(blobs(:,1)) % Write the corresponding centroids
        centX = centroid(object,1); centY = centroid(object,2);
        vidIn = step(htextinsCent, vidIn, [centX-320 180-centY], [centX-6 centY-9]); 
    end
    vidIn = step(htextins, vidIn, uint8(length(blobs(:,1)))); % Count the number of blobs
    step(hVideoIn, vidIn); % Output video stream

    %% PD Control
    x = centroid(object,1)-320;
    %' Calculate error.
    error = SetPoint - x;
    %' Calculate proportional term.
    p = Kp * error;
    % Calculate output.
    Yaw = p;
    A = single(Yaw);
    Yaw_control = A*0.01;
    Previous = error;
    if -70<=Yaw<=70
%         Move3D(controlChannel,0.02,0,0,Yaw_control)
    else
        continue
    end
end
%% Clearing Memory
release(hVideoIn); % Release all memory and buffer used
release(vidDevice);
fclose(instrfindall);
clear all;
clc;

