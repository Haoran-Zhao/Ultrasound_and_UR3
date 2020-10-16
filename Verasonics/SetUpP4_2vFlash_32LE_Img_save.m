% Notice:
%   This file is provided by Verasonics to end users as a programming
%   example for the Verasonics Vantage Research Ultrasound System.
%   Verasonics makes no claims as to the functionality or intended
%   application of this program and the user assumes all responsibility
%   for its use.
%
% File name: SetUpP4_2vFlash_32LE.m - Example of phased array imaging with
%                                flash transmit scan
% Description:
%   Sequence programming file for P4-2v phased array in virtual apex
%   format, using a plane wave flash transmit for all acquisitions. 64
%   transmit and 32 receive channels are active for each acquisition. Two
%   synthetic aperture acquisitions are used to capture receive data from
%   all 64 elements.  The curvature of the transmit pulse is set to match a
%   circle with the same P.radius as the distance to the virtual apex.
%   Processing is asynchronous with respect to acquisition. Note: The
%   P4-2v is a 64 element transducer, with elements 1-64 wired to the probe
%   connector element signals 33-96.
%
% Last update:
% 12/12/2018 Modified for synthetic aperture receive acquisition.

clear all
P.iter = 1;
P.startDepth = 5;
P.endDepth = 160;   % Acquisition depth in wavelengths
P.record = 0;
% Define system parameters.
Resource.Parameters.speedOfSound = 1540;
Resource.Parameters.speedCorrectionFactor = 1.0;
Resource.Parameters.verbose = 2;
Resource.Parameters.initializeOnly = 0;
Resource.Parameters.simulateMode = 0;
%  Resource.Parameters.simulateMode = 1 forces simulate mode, even if hardware is present.
%  Resource.Parameters.simulateMode = 2 stops sequence and processes RcvData continuously.
Resource.System.SoftwareVersion = [4 0 0]; % Minimum software release for this script.
Resource.System.UTA = '260-MUX'; % This script requires the 260-MUX UTA.

% Specify Trans structure array.
Trans.name = 'P4-2v';
Trans.units = 'wavelengths'; % Explicit declaration avoids warning message when selected by default
Trans = computeTrans(Trans);
Trans.maxHighVoltage = 50;  % set maximum high voltage limit for pulser supply.
Trans = computeUTAMux64(Trans); % Add HVMux field for use with UTA 260-Mux
P.mm2wl = Trans.frequency/(Resource.Parameters.speedOfSound/1000);
P.wl2mm = Resource.Parameters.speedOfSound/1000/Trans.frequency;

P.theta = -pi/8;
P.rayDelta = 2*(-P.theta);
P.aperture = 64*Trans.spacing; % P.aperture in wavelengths
P.radius = (P.aperture/2)/tan(-P.theta); % dist. to virt. apex

% Set up PData structure.
PData(1).PDelta = [0.8, 0, 0.5]; %[Trans.spacing, 0, 0.5]
PData(1).Size(1) = 10 + ceil((P.endDepth-P.startDepth)/PData(1).PDelta(3));
PData(1).Size(2) = 10 + ceil(2*(P.endDepth + P.radius)*sin(-P.theta)/PData(1).PDelta(1));
P.pxl2mm = (P.endDepth-P.startDepth)*P.wl2mm/ceil((P.endDepth-P.startDepth)/PData(1).PDelta(3)); %pixel size in [mm]

PData(1).Size(3) = 1;
PData(1).Origin = [-(PData(1).Size(2)/2)*PData(1).PDelta(1),0,P.startDepth];
PData(1).Region = struct(...
            'Shape',struct('Name','SectorFT', ...
            'Position',[0,0,-P.radius], ...
            'z',P.startDepth, ...
            'r',P.radius+P.endDepth, ...
            'angle',P.rayDelta, ...
            'steer',0));
PData(1).Region = computeRegions(PData(1));

% Specify Media.
% Set up Media points
Media.MP(1,:) = [-10,0,69,1.0];

Media.numPoints = 1;
Media.attenuation = -0.5;
Media.function = 'circleMovement';

% Specify Resources.
Resource.RcvBuffer(1).datatype = 'int16';
Resource.RcvBuffer(1).rowsPerFrame = 2*4096; % two acquisitions per frame
Resource.RcvBuffer(1).colsPerFrame = Trans.numelements;
Resource.RcvBuffer(1).numFrames = 100;    % 100 frames used for RF cineloop.
Resource.InterBuffer(1).numFrames = 1;  % one intermediate buffer used.
Resource.ImageBuffer(1).numFrames = 10;
Resource.DisplayWindow(1).Title = 'P4-2vFlash_32LE';
Resource.DisplayWindow(1).pdelta = 0.35;
ScrnSize = get(0,'ScreenSize');
DwWidth = ceil(PData(1).Size(2)*PData(1).PDelta(1)/Resource.DisplayWindow(1).pdelta);
DwHeight = ceil(PData(1).Size(1)*PData(1).PDelta(3)/Resource.DisplayWindow(1).pdelta);
Resource.DisplayWindow(1).Position = [250,(ScrnSize(4)-(DwHeight+150))/2, ...  % lower left corner position
                                      DwWidth, DwHeight];
Resource.DisplayWindow(1).ReferencePt = [PData(1).Origin(1),0,PData(1).Origin(3)];   % 2D imaging is in the X,Z plane
Resource.DisplayWindow(1).Type = 'Verasonics';
Resource.DisplayWindow(1).numFrames = 20;
Resource.DisplayWindow(1).AxesUnits = 'mm';
Resource.DisplayWindow.Colormap = gray(256);

% Specify Transmit waveform structure.
TW.type = 'parametric';
TW.Parameters = [Trans.frequency,.67,2,1];

% Set up transmit delays in TX structure.
TX.waveform = 1;
TX.Origin = [0,0,0];            % set origin to 0,0,0 for flat focus.
TX.focus = -P.radius;     % set focus to negative for concave TX.Delay profile.
TX.Steer = [0,0];
TX.Apod = ones(1,Trans.numelements);  % set TX.Apod for 64 elements
TX.aperture = 1;
TX.Delay = computeTXDelays(TX);

% Specify Receive structure arrays.
maxAcqLength = ceil(sqrt(P.aperture^2 + P.endDepth^2 - 2*P.aperture*P.endDepth*cos(P.theta-pi/2)) - P.startDepth);
Receive = repmat(struct('Apod', [zeros(1,16),ones(1,32),zeros(1,16)], ...
                        'startDepth', P.startDepth, ...
                        'endDepth', P.startDepth + maxAcqLength, ...
                        'aperture', 1,...
                        'TGC', 1, ...
                        'bufnum', 1, ...
                        'framenum', 1, ...
                        'acqNum', 1, ...
                        'sampleMode', 'NS200BW', ...
                        'mode', 0, ...
                        'callMediaFunc',0),1,2*Resource.RcvBuffer(1).numFrames);
% - Set event specific Receive attributes.
for i = 1:Resource.RcvBuffer(1).numFrames
    Receive(2*i-1).framenum = i;
    Receive(2*i).framenum = i;
    Receive(2*i).acqNum = 2;
    Receive(2*i).Apod = [ones(1,16),zeros(1,32),ones(1,16)];
    Receive(2*i).callMediaFunc = 1; %active simulated media movement
end

% Specify TGC Waveform structure.
TGC.CntrlPts = [0,467,535,653,690,811,942,1023];
TGC.rangeMax = P.endDepth;
TGC.Waveform = computeTGCWaveform(TGC);

% Specify Recon structure arrays.
Recon = struct('senscutoff', 0.5, ...
               'pdatanum', 1, ...
               'rcvBufFrame', -1, ...
               'IntBufDest', [1,1], ...
               'ImgBufDest', [1,-1], ...
               'RINums', 1:2);

% Define ReconInfo structures.
ReconInfo(1) = struct('mode', 'replaceIQ', ...
                   'txnum', 1, ...
                   'rcvnum', 1, ...
                   'regionnum', 1);
ReconInfo(2) = struct('mode', 'accumIQ_replaceIntensity', ...
                   'txnum', 1, ...
                   'rcvnum', 2, ...
                   'regionnum', 1);

% Specify Process structure array.
pers = 20;
Process(1).classname = 'Image';
Process(1).method = 'imageDisplay';
Process(1).Parameters = {'imgbufnum',1,...   % number of buffer to process.
                         'framenum',-1,...   % (-1 => lastFrame)
                         'pdatanum',1,...    % number of PData structure to use
                         'pgain',1.0,...            % pgain is image processing gain
                         'reject',3,...      % reject level  50
                         'persistMethod','simple',...
                         'persistLevel',pers,...
                         'interpMethod','4pt',...
                         'grainRemoval','none',...
                         'processMethod','none',...
                         'averageMethod','none',...
                         'compressMethod','power',...
                         'compressFactor',100,...
                         'mappingMethod','full',...
                         'display',1,...      % display image after processing
                         'displayWindow',1};

Process(2).classname = 'External';
Process(2).method = 'myProcFunction';
Process(2).Parameters = {'srcbuffer','image',...
    'srcbufnum',1,...
    'srcframenum',-1,...
    'dstbuffer','none'};

% Specify SeqControl structure arrays.  Missing fields are set to NULL.
SeqControl(1).command = 'jump'; %  - Jump back to start.
SeqControl(1).argument = 1;
SeqControl(2).command = 'timeToNextAcq';  % set time between acquisitions
SeqControl(2).argument = 1000; % 1msec
SeqControl(3).command = 'timeToNextAcq';  % set time between frames
SeqControl(3).argument = 9000; % 10msec total frame iterval (~100fps)
SeqControl(4).command = 'returnToMatlab';
nsc = 5; % nsc is count of SeqControl objects

n = 1; % n is count of Events

% Acquire all frames defined in RcvBuffer
for i = 1:Resource.RcvBuffer(1).numFrames
    Event(n).info = 'center of receive aperture';
    Event(n).tx = 1;
    Event(n).rcv = 2*i-1;
    Event(n).recon = 0;
    Event(n).process = 0;
    Event(n).seqControl = 2; % acquisition PRI
    n = n+1;

    Event(n).info = 'edges of receive aperture';
    Event(n).tx = 1;
    Event(n).rcv = 2*i;
    Event(n).recon = 0;
    Event(n).process = 0;
    Event(n).seqControl = [3,nsc];
       SeqControl(nsc).command = 'transferToHost';
       nsc = nsc + 1;
    n = n+1;

    Event(n).info = 'Reconstruct';
    Event(n).tx = 0;
    Event(n).rcv = 0;
    Event(n).recon = 1;
    Event(n).process = 1;
    Event(n).seqControl = 0;
    n = n+1;
    
    Event(n).info = 'Call external Processing function';
    Event(n).tx = 0;
    Event(n).rcv = 0;
    Event(n).recon = 0;
    Event(n).process = 2;
    Event(n).SeqControl = 0;
    if floor(i/3) == i/3     % Exit to Matlab every 3rd frame
        Event(n).seqControl = 4;
    end
    n = n + 1;
end

Event(n).info = 'Jump back to first event';
Event(n).tx = 0;
Event(n).rcv = 0;
Event(n).recon = 0;
Event(n).process = 0;
Event(n).seqControl = 1;


% User specified UI Control Elements
% - Sensitivity Cutoff
UI(1).Control =  {'UserB7','Style','VsSlider','Label','Sens. Cutoff',...
                  'SliderMinMaxVal',[0,1.0,Recon(1).senscutoff],...
                  'SliderStep',[0.025,0.1],'ValueFormat','%1.3f'};
UI(1).Callback = text2cell('%SensCutoffCallback');

% - Range Change
MinMaxVal = [64,300,P.endDepth]; % default unit is wavelength
AxesUnit = 'wls';
if isfield(Resource.DisplayWindow(1),'AxesUnits')&&~isempty(Resource.DisplayWindow(1).AxesUnits)
    if strcmp(Resource.DisplayWindow(1).AxesUnits,'mm')
        AxesUnit = 'mm';
        MinMaxVal = MinMaxVal * (Resource.Parameters.speedOfSound/1000/Trans.frequency);
    end
end
UI(2).Control = {'UserA1','Style','VsSlider','Label',['Range (',AxesUnit,')'],...
                 'SliderMinMaxVal',MinMaxVal,'SliderStep',[0.1,0.2],'ValueFormat','%3.0f'};
UI(2).Callback = text2cell('%RangeChangeCallback');

UI(3).Control = {'UserB2','Style','VsSlider','Label','Record',...
                 'SliderMinMaxVal',[0,1,0],'SliderStep',[1,1],'ValueFormat','%3.0f'};
UI(3).Callback = text2cell('%RecordCallback');

EF(1).Function = text2cell('%EF#1');
% Specify factor for converting sequenceRate to frameRate.
frameRateFactor = 3;

% Save all the structures to a .mat file.
save('MatFiles/P4_2vFlash_32LE_Img_save');
return


% **** Callback routines to be converted by text2cell function. ****


%RecordCallback
P = evalin('base','P');
P.record = UIValue;
assignin('base','P', P);
%RecordCallback


%EF#1
myProcFunction(RData)
persistent myHandle
P = evalin('base','P');
% Resource = evalin('base','Resource');
% if isempty(myHandle)||~ishandle(myHandle)
%     figure;
%     myHandle = axes('XLim', [0,size(RData,2)], 'YLim', [0, size(RData,1)],...
%     'Ydir','reverse','NextPlot', 'replacechildren');
% end
%Plot the RF data

% set(myHandle,'XLim',[0,size(RData,2)],'YLim',[0,size(RData,1)])

% %%%% image process and plot bounding box
% level = 0.2;
% gray_img = mat2gray(RData);
% filted = medfilt2(gray_img);
% B_img = im2bw(filted,level);
% se = strel('rectangle', [3,3]);
% filled_img = imclose(B_img,se);
% 
% Iregion = regionprops(filled_img, 'centroid');
% [labeled, numObjects] = bwlabel(filled_img, 4);
% stats = regionprops(labeled,'Eccentricity','Area', 'BoundingBox');
% areas = [stats.Area];
% maximum = max(areas);
% [max_idx,~]=find(areas==maximum);
% eccentricities = [stats.Eccentricity];
% idxOfObj = find(eccentricities);
% statsDetects = stats(idxOfObj);
% 
% imagesc(myHandle,gray_img);
% colormap(gray(256))
% hold on
% h = rectangle('Position', statsDetects(max_idx).BoundingBox);
% center = [statsDetects(max_idx).BoundingBox(1)+0.5*statsDetects(max_idx).BoundingBox(3),statsDetects(max_idx).BoundingBox(2)+0.5*statsDetects(max_idx).BoundingBox(4)];
% string = sprintf('(%0.2f,%0.2f)', center(1)*P.pxl2mm,center(2)*P.pxl2mm); 
% text(statsDetects(max_idx).BoundingBox(1)-12,statsDetects(max_idx).BoundingBox(2)+statsDetects(max_idx).BoundingBox(4)-10,string,'Color','white','FontSize',5);
% set(h, 'EdgeColor', [0.75 0 0],'LineWidth', 1);
% % center = [];
% % for idx = 1 : length(idxOfObj)
% %     h = rectangle('Position', statsDetects(idx).BoundingBox);
% %     center = [center; statsDetects(idx).BoundingBox(1)+0.5*statsDetects(idx).BoundingBox(3),statsDetects(idx).BoundingBox(2)+0.5*statsDetects(idx).BoundingBox(4)];
% %     string = sprintf('(%0.2f,%0.2f)', center(idx,1)*P.pxl2mm,center(idx,2)*P.pxl2mm); 
% %     text(statsDetects(idx).BoundingBox(1)-16,statsDetects(idx).BoundingBox(2)+statsDetects(idx).BoundingBox(4)-10,string,'Color','white','FontSize',5);
% %     set(h, 'EdgeColor', [0.75 0 0],'LineWidth', 1);
% %     hold on;
% % end
% hold off
gray_img = mat2gray(RData);
% gray_img = imadjust(gray_img);
% gray_img = medfilt2(gray_img);

gray_img = imresize(gray_img, [500 NaN]);

% if isempty(myHandle)||~ishandle(myHandle)
%     figure;
%     myHandle = axes('XLim', [0,size(gray_img,2)], 'YLim', [0, size(gray_img,1)],...
%     'Ydir','reverse','NextPlot', 'replacechildren');
% end
% imagesc(myHandle,gray_img);
% colormap(gray(256))

% title(myHandle, sprintf('Pxl2mm = %0.3f',P.pxl2mm))

% keyboard % debug
% ImgDisplay = get(Resource.DisplayWindow(1).imageHandle,'CData');
% imagesc(ImgDisplay);
% colormap(gray);
% caxis([0 256]);
% 
if P.record
    imwrite(gray_img, [pwd '/Verasonics_Haoran/Images/Scene/scene',num2str(P.iter),'.png']);
    imwrite(gray_img, [pwd '/Verasonics_Haoran/Images/scene1.png']);
%     Image = getframe(myHandle);
%     imwrite(Image.cdata, [pwd '/Verasonics_Haoran/Images/scene',num2str(P.iter),'.png']);
    P.iter = P.iter+1;
    assignin('base','P', P);
end
% keyboard % debug
%EF#1


%SensCutoffCallback - Sensitivity cutoff change
ReconL = evalin('base', 'Recon');
for i = 1:size(ReconL,2)
    ReconL(i).senscutoff = UIValue;
end
assignin('base','Recon',ReconL);
Control = evalin('base','Control');
Control.Command = 'update&Run';
Control.Parameters = {'Recon'};
assignin('base','Control', Control);
return
%SensCutoffCallback

%RangeChangeCallback - Range change
simMode = evalin('base','Resource.Parameters.simulateMode');
% No range change if in simulate mode 2.
if simMode == 2
    set(hObject,'Value',evalin('base','P.endDepth'));
    return
end
Trans = evalin('base','Trans');
Resource = evalin('base','Resource');
scaleToWvl = Trans.frequency/(Resource.Parameters.speedOfSound/1000);

P = evalin('base','P');
P.endDepth = UIValue;
if isfield(Resource.DisplayWindow(1),'AxesUnits')&&~isempty(Resource.DisplayWindow(1).AxesUnits)
    if strcmp(Resource.DisplayWindow(1).AxesUnits,'mm')
        P.endDepth = UIValue*scaleToWvl;
    end
end
PData = evalin('base','PData');
PData(1).Size(1) = 10 + ceil((P.endDepth-P.startDepth)/PData(1).PDelta(3));
P.pxl2mm = (P.endDepth-P.startDepth)*P.wl2mm/ceil((P.endDepth-P.startDepth)/PData(1).PDelta(3)); %pixel size in [mm]
assignin('base','P',P);

PData(1).Region = struct(...
            'Shape',struct('Name','SectorFT', ...
            'Position',[0,0,-P.radius], ...
            'z',P.startDepth, ...
            'r',P.radius+P.endDepth, ...
            'angle',P.rayDelta, ...
            'steer',0));
PData(1).Region = computeRegions(PData(1));
assignin('base','PData',PData);
evalin('base','Resource.DisplayWindow(1).Position(4) = ceil(PData(1).Size(1)*PData(1).PDelta(3)/Resource.DisplayWindow(1).pdelta);');
Receive = evalin('base', 'Receive');
maxAcqLength = ceil(sqrt(P.aperture^2 + P.endDepth^2 - 2*P.aperture*P.endDepth*cos(P.theta-pi/2)) - P.startDepth);
wlsPer128 = 128/(4*2);
for i = 1:size(Receive,2)
    Receive(i).endDepth = P.startDepth + wlsPer128*ceil(maxAcqLength/wlsPer128);
end
assignin('base','Receive',Receive);
evalin('base','TGC.rangeMax = P.endDepth;');
evalin('base','TGC.Waveform = computeTGCWaveform(TGC);');
Control = evalin('base','Control');
Control.Command = 'update&Run';
Control.Parameters = {'PData','InterBuffer','ImageBuffer','DisplayWindow','Receive','TGC','Recon'};
assignin('base','Control', Control);
assignin('base', 'action', 'displayChange');
return
%RangeChangeCallback
