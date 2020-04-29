function varargout = Ground_Control_Station(varargin)

% Ground_Control_Station MATLAB code for Ground_Control_Station.fig
%      Ground_Control_Station, by itself,
%      creates a new Ground_Control_Station or raises the existing
%      singleton*.
%
%      H = Ground_Control_Station returns the handle to a new
%          Ground_Control_Station or the handle to the existing singleton*.
%
%      Ground_Control_Station('CALLBACK',hObject,eventData,handles,...)
%      calls the local
%      function named CALLBACK in Ground_Control_Station.M
%      with the given input arguments.
%
%      Ground_Control_Station('Property','Value',...)
%      creates a new Ground_Control_Station or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Ground_Control_Station_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Ground_Control_Station_OpeningFcn
%      via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Ground_Control_Station

% Last Modified by GUIDE v2.5 11-Nov-2014 19:13:59

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Ground_Control_Station_OpeningFcn, ...
                   'gui_OutputFcn',  @Ground_Control_Station_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end



% --- Executes just before Ground_Control_Station is made visible.
function Ground_Control_Station_OpeningFcn(hObject, eventdata, handles, varargin)
% End initialization code - DO NOT EDIT
axes(handles.axes3)
imshow('Mark.jpg','parent',handles.axes3)
axes(handles.axes4)
imshow('DSC01817.jpg','parent', handles.axes4)
axes(handles.axes1)
% AltitudeViewer;
% ah = axes('Tag','axes3','unit', 'normalized');
% 
% % import the background image and show it on the axes
% 
% bg = imread('image2.jpg'); 
% imagesc(bg);
% 
% % prevent plotting over the background and turn the axis off
% 
% set(ah,'handlevisibility','off','visible','off')

% making sure the background is behind all the other uicontrols

% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Ground_Control_Station (see VARARGIN)

% Choose default command line output for Ground_Control_Station
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);



         

   
% UIWAIT makes Ground_Control_Station wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = Ground_Control_Station_OutputFcn(~, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



% --- Executes on button press in boeing.
function boeing_Callback(hObject, eventdata, handles)
axes(handles.axes1)
AltitudeViewer;

% hObject    handle to boeing (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in hovering.
function hovering_Callback(hObject, eventdata, handles)
global controlChannel;
global stateChannel;

axes(handles.axes1)


 WayPoint = [0, 0, 193, 2
            0, 0, 193, 2
             0, 0, 193, 2
             0, 0, 193, 2];
 

PointNum = size(WayPoint);
PointNum = PointNum(1);

SequenceNumber = tic;

SequenceNumber = TakeOff(SequenceNumber, controlChannel, stateChannel);

Duration_in_second = 30;
if Duration_in_second <=0;
    return
end

 catch excp
    disp('failed to open udp channels.');
    disp(excp.message)
     return
 end




    sequenceNumber = tic;
    t_ = 0;
    t_0 = clock;
    while(t_ < Duration_in_second);
       global v;
 
        if v == 1;
           break;
        
        elseif v == 2;
            Sig = sprintf('AT*REF=%d,290717952\r',tic); %(00010001010101000000000100000000)
            fprintf(controlChannel, Sig);

        end
         [~, ~, sequenceNumber] = Ask4DroneState (sequenceNumber, controlChannel, stateChannel, 1);
 t_ = etime(clock,t_0);
 
    end       
if SequenceNumber ~= -1
    
    
%  pause(1);


%   1      2       3         4          5          6
%  flag,LR_tilt,FB_tilt,VerticalVel,AngularVel  command duration
% [SequenceNumber ~] = TrackRelativeDisplacement ( SequenceNumber, controlChannel, stateChannel, WayPoint,0, 'indoor');

% pause(1);
 
SequenceNumber = Land(SequenceNumber, controlChannel,stateChannel);
end
    
% hObject    handle to hovering (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


function control = gui_control(AltitudeViewer)

if isa(AltitudeViewer, 'UIControl')
    control = AltitudeViewer.Control;  % query representative control
else
    control = AltitudeViewer;
end


% --- Executes on button press in motoron.
function motoron_Callback(hObject, eventdata, handles)
global ARc;
 PWM = sprintf('AT*PWM=1,1,0,0,0\r');  
 fprintf(ARc, PWM);


% Closing UDP port
%   fclose(ARc);
%  
% hObject    handle to motoron (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in motoroff.
function motoroff_Callback(hObject, eventdata, handles)
global ARc;
 PWM = sprintf('AT*PWM=1,0,0,0,0\r');  
 fprintf(ARc, PWM);
% hObject    handle to motoroff (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in portopen.
function portopen_Callback(hObject, eventdata, handles)
global ARc;
 ARc = udp('192.168.1.1', 5556, 'LocalPort', 5556);
 assignin('base','ARc',ARc);
  fopen(ARc);
% fopen(ARc);
% assignin('base','ARc',ARc)

% hObject    handle to portopen (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in portclose.
function portclose_Callback(hObject, eventdata, handles)
fclose(instrfindall);
% hObject    handle to portclose (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in waypoint.
function waypoint_Callback(hObject, eventdata, handles)
WayPoint;
% hObject    handle to waypoint (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --- Executes on button press in ARDRONE.
function ARDRONE_Callback(hObject, eventdata, handles)
axes(handles.axes2);
ARDrone;


% addpath('.\simulation\MatlabAPI')
% hObject    handle to ARDRONE (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in add.
function add_Callback(hObject, eventdata, handles)
addDroneControl;
% hObject    handle to add (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in Landing.
function Landing_Callback(hObject, eventdata, handles)
global v;
v = 1;
% hObject    handle to Landing (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in Emergency.
function Emergency_Callback(hObject, eventdata, handles)
global v;
v = 2;

%  normal landing
% Sig = sprintf('AT*REF=%d,290717696\r',tic); %(00010001010101000000000000000000)

% emergency landing
% stop the drone engine
% Sig = sprintf('AT*REF=%d,290717952\r',tic); %(00010001010101000000000100000000)
% fprintf(controlChannel, Sig);
% hObject    handle to Emergency (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in Clear.
function Clear_Callback(hObject, eventdata, handles)
clear all;
% hObject    handle to Clear (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in Close.
function Close_Callback(hObject, eventdata, handles)
close all;
% hObject    handle to Close (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in UDPOOO.
function UDPOOO_Callback(hObject, eventdata, handles)
global controlChannel;
global stateChannel;

controlChannel = udp('192.168.1.1', 5556, 'LocalPort', 5556);
 assignin('base','controlChannel',controlChannel);
  fopen(controlChannel);
 stateChannel = udp('192.168.1.1', 5554, 'LocalPort', 5554);
 assignin('base','stateChannel',stateChannel);
 fopen(stateChannel);

% hObject    handle to UDPOOO (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of UDPOOO


% --- Executes on button press in CLC.
function CLC_Callback(hObject, eventdata, handles)
clc;
% hObject    handle to CLC (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in GOSTOP.
function GOSTOP_Callback(hObject, eventdata, handles)
global controlChannel;
global stateChannel;

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
% hObject    handle to GOSTOP (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in Yawing.
function Yawing_Callback(hObject, eventdata, handles)
  global controlChannel;
  global stateChannel;

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
        Move3D(controlChannel,0.02,0,0,Yaw_control)
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

% hObject    handle to Yawing (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in land2.
function land2_Callback(hObject, eventdata, handles)
global controlChannel;
global stateChannel;

AR_LAND   = sprintf('AT*REF=%d,%d\r',tic,290717696);
fprintf(controlChannel, AR_LAND);
% hObject    handle to land2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in TAKEOFF.
function TAKEOFF_Callback(hObject, eventdata, handles)
global controlChannel;
global stateChannel;

AR_TRIM = sprintf('AT*FTRIM=%d,\r',1);
fprintf(controlChannel, AR_TRIM);

% Start
BASIC_CODE = 2^18 + 2^20 + 2^22 + 2^24 + 2^28;
START_CODE = BASIC_CODE + 2^9;
AR_START   = sprintf('AT*REF=%d,%d\r',2,START_CODE);
fprintf(controlChannel, AR_START);
% hObject    handle to TAKEOFF (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in Roll.
function Roll_Callback(hObject, eventdata, handles)
  global controlChannel;
global stateChannel;
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
    Roll = p;
    A = single(Roll);
    Roll_control = A*0.01;
    Previous = error;
    if -70<=Roll<=70
        Move3D(controlChannel,0.02,Roll_control,0,0)
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


% hObject    handle to Roll (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in Avoidance.
function Avoidance_Callback(hObject, eventdata, handles)
global controlChannel;
global stateChannel;
redThresh = 0.25; % Threshold for red detection
% vidDevice = cv.VideoCapture;
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
        area  = c*d;
    else
        continue
        
    end
     
    
    if area <=17000;
       Move3D(controlChannel, 0.2, 0, 0 , 0)
   else
       for i= 1:3000
       Move3D(controlChannel, 0, 0, 0 , 0)
       end
       for i= 1:5000
       Move3D(controlChannel, 0, 0.2, 0 , 0)
       end
       for i= 1:3000
       Move3D(controlChannel, 0, 0, 0 , 0)
       end
       for i= 1:9000
       Move3D(controlChannel, 0.2, 0, 0 , 0)
       end
       for i= 1:3000
       Move3D(controlChannel, 0, 0, 0 , 0)
       end
       for i= 1:7000
       Move3D(controlChannel, 0, -0.2, 0 , 0)
       end
       for i= 1:3000
       Move3D(controlChannel, 0, 0, 0 , 0)
       end
       AR_LAND   = sprintf('AT*REF=%d,%d\r',tic,290717696);
       fprintf(controlChannel, AR_LAND);
    end
    
end
%% Clearing Memory
release(hVideoIn); % Release all memory and buffer used
release(vidDevice);
clear all;
clc;
% hObject    handle to Avoidance (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
