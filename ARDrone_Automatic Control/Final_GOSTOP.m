%% Initialization
redThresh = 0.25; % Threshold for red detection
vidDevice = cv.VideoCapture('tcp://192.168.1.1:5555');

hblob = vision.BlobAnalysis('AreaOutputPort', false, ... % Set blob analysis handling
                                'CentroidOutputPort', true, ... 
                                'BoundingBoxOutputPort', true', ...
                                'MinimumBlobArea', 300, ...
                                'MaximumBlobArea', 30000000, ...
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
                                'Position', [100 100 640+20 480+30]);
nFrame = 0; % Frame number initialization

%% Processing Loop
while(1)

    rgbFrame = vidDevice.read; % Acquire single frame
    diffFrame = imsubtract(rgbFrame(:,:,1) , cv.cvtColor(rgbFrame, 'RGB2GRAY') ); % Get red component of the image
    diffFrame = medfilt2(diffFrame, [3 3]); % Filter out the noise by using median filter
    binFrame = im2bw(diffFrame, redThresh); % Convert the image into binary image with the red objects as white
    [centroid, blobs] = step(hblob, binFrame); % Get the centroids and bounding boxes of the blobs
    centroid = int16(centroid); % Convert the centroids into Integer for further steps 
    rgbFrame(1:20,1:165,:) = 0; % put a black region on the output stream
    vidIn = step(hshapeinsRedBox, rgbFrame, blobs); % Instert the red box

    for object = 1:1:length(blobs(:,1)) % Write the corresponding centroids
        centX = centroid(object,1); centY = centroid(object,2);
        A = [centX-320 240-centY];
        vidIn = step(htextinsCent, vidIn, [centX-320 240-centY], [centX-6 centY-9]); 
    end
    vidIn = step(htextins, vidIn, uint8(length(blobs(:,1)))); % Count the number of blobs
    step(hVideoIn, vidIn); % Output video stream
    if size(blobs)==[1,4];
        c = blobs(1,3);
        d = blobs(1,4);
        area  = c*d
    else
        continue
        
    end
     
    
   if area <=17000;
       Move3D(controlChannel, 0.17,0, 0 , 0)
       continue
   else
       Move3D(controlChannel, 0, 0, 0 , 0)
       continue
   end
end
%% Clearing Memory
release(hVideoIn); % Release all memory and buffer used
release(vidDevice);
clear all;
clc;