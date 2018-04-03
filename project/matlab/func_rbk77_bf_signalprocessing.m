%% Signal processing for Radarbook beamforming measurements
% 
% Revision
%   Version | Date          | Author         |
%   V1.0    | 11.01.2016    | Zora Slavik    |  initial version
%   V2.0    | 31.07.2017    | Zora Slavik    |  change reading of files
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function func_rbk77_bf_signalprocessing()
fs=10e6;
%--------------------------------------------------------------------------
% Define Constants
%--------------------------------------------------------------------------
c0 = 3e8; 
% 
% p='C:\Users\slavik\Documents\CS\Radarbox\Matlab\logdata\2017-07-30\16-33\';
% p='C:\Users\slavik\Documents\CS\Radarbox\16-00\';
% p='C:\Users\slavik\Documents\CS\Radarbox\Matlab\2017-09-29_MeasBF\';
% p='C:\Users\slavik\Documents\CS\Radarbox\2017-09-29\18-36\';
% p='C:\Users\slavik\Documents\CS\Radarbox\Matlab\logdata\2017-10-03\16-18\';

%% Path to data
% p='C:\Users\slavik\Documents\CS\Radarbox\Matlab\logdata\2017-10-26-2\14-58\';
p='/local/git/MotionFlowPriorityGraphSensors/datasets/radar_dataset/data/stereo_flow/2017-10-05/17-58/';
% p='C:\Users\slavik\Documents\CS\Radarbox\Matlab\logdata\2018-01-24\RCS\A3\17-37\';

pcam=strcat(p,'cam/')
% p=strcat(p1,path);
% p=path; 
%2017-07-30\16-23\';

%--------------------------------------------------------------------------
% Load Measurement and Calibration Data
%--------------------------------------------------------------------------
filenameTX = [p,'logTX.txt'];
Data = load(filenameTX);
[mdata,ndata] = size(Data);

filenameCalData = 'CalData.mat';
load(filenameCalData);
% CalData = hCal(:,1)+ 1i*hCal(:,2);
%--------------------------------------------------------------------------
% Configuration
%--------------------------------------------------------------------------
Cfg.fStrt       =   76e9;
Cfg.fStop       =   78e9;
Cfg.TRampUp     =   120e-6;
Cfg.TRampDo     =   60e-6;
Cfg.TInt        =   10e-3;
Cfg.Tp          =   360e-6;
Cfg.N           =   2^10;
Cfg.Np          =   1;
Cfg.NrFrms      =   100;
% Cfg.fStrt       =   76e9;
% Cfg.fStop       =   78e9;
% Cfg.TRampUp     =   120e-6;
% Cfg.TRampDo     =   60e-6;
% Cfg.TInt        =   100e-3;
% Cfg.Tp          =   360e-6;
% Cfg.N           =   2^7;
% Cfg.Np          =   1;
% Cfg.NrFrms      =   100;
Cfg.TxSeq       =   [1, 2, 3, 4];
Cfg.AdcChn      =   6;
Brd.FuSca       =   6.1035e-05; 
BW=Cfg.fStop - Cfg.fStrt;
fc=Cfg.fStrt+BW/2;


disp('################################ config done #######################')

%--------------------------------------------------------------------------
% Read Settings for N and fs
%--------------------------------------------------------------------------
% fs              =   10e6; %(20/3)*10^6; % fs=80MHz (6ADCs)/8 Rx antennas
N               =   Cfg.N; 
%--------------------------------------------------------------------------
% Configure Signal Processing
%--------------------------------------------------------------------------
% Processing of range profile
NFFT                =   2^12;                   % set FFT resolution in range direction
NV                  =   32-3;                   % Number of virtual elements

% Processing of range profile
Win2D               =   repmat(hann(N-11),[1 NV]);
ScaWin              =   sum(Win2D(:,1));
kf                  =   BW/Cfg.TRampUp;
vRange              =   (0:NFFT-1).'./NFFT.*fs.*c0/(2.*kf);
DataV               =   zeros(N-11,NV); 

%% set minimum and maximum range
RMin                =   1;         % min range for evaluation
RMax                =   70;         % max range for evaluation

[Val RMinIdx]       =   min(abs(vRange - RMin));
[Val RMaxIdx]       =   min(abs(vRange - RMax));
vRangeExt           =   vRange(RMinIdx:RMaxIdx);
%% Beamforming
% Window function for  NV virtual receive channels
NFFTAnt             =   2^10;
WinAnt              =   hann(NV);
ScaWinAnt           =   sum(WinAnt);
WinAnt2D            =   repmat(WinAnt.',numel(vRangeExt),1);
vAngDeg             =   (-NFFTAnt/2:NFFTAnt/2-1).'./NFFTAnt*180;

% Positions for polar plot of cost function
vU                  =   linspace(-1,1,NFFTAnt);
[mRange , mU]       =   ndgrid(vRangeExt,vU);
mX                  =   mRange.*mU;
mY                  =   mRange.*cos(asin(mU));

% Calibration data: Remove 8, 16, 24 measurement (overlappling ones)
AntIdx              =   [1:7, 9:15, 17:23, 25:32];
mCalData            =   repmat(CalData(AntIdx).',N-11,1);

% % ii=0, start scene, straight forward clutter removal
% DataClutter(:,1:7)    =   Data(12:N,1:7);
% DataClutter(:,8:14)   =   Data(12:N,9:15);
% DataClutter(:,15:21)  =   Data(12:N,17:23);
% DataClutter(:,22:29)  =   Data(12:N,25:32);
% load('DataClutter.mat')

for ii = 0:1:floor(mdata/(1*N))-1

    % TX1
     FrmCntr         =   Data(ii*N+1,AntIdx);
     frmno=FrmCntr(end);
     frmnr=num2str(FrmCntr(end));
    disp(frmnr);
    SeqId           =   Data(ii*N+2,AntIdx)*16;
    
%--------------------------------------------------------------------------
% Beamforming
%--------------------------------------------------------------------------
        DataV(:,1:7)    =   Data(ii*N+12:(ii+1)*N,1:7);
        DataV(:,8:14)   =   Data(ii*N+12:(ii+1)*N,9:15);
        DataV(:,15:21)  =   Data(ii*N+12:(ii+1)*N,17:23);
        DataV(:,22:29)  =   Data(ii*N+12:(ii+1)*N,25:32);
    
%     disp(['FrmCntr: ', num2str(FrmCntr)]);
%     disp(['SeqId: ', num2str(SeqId)]);
%     DataV=DataV-DataClutter;
    % Calculate range profile including calibration
    RP          =   fft(DataV.*Win2D.*mCalData,NFFT,1).*Brd.FuSca/ScaWin;
    RPExt       =   RP(RMinIdx:RMaxIdx,:);    

%     % calculate fourier transform over receive channels
    JOpt        =   fftshift(fft(RPExt.*WinAnt2D,NFFTAnt,2)/ScaWinAnt,2);
%     
%   % non-scaled range-azimuth map   
%     figure(33)
%     imagesc(vAngDeg, vRangeExt, 20.*log10(abs(JOpt)));
%     xlabel('Ang ()');
%     ylabel('R (m)');
%     colormap('jet');   
%     colorbar

%     % normalize cost function
%     JdB         =   20.*log10(abs(JOpt));
%     JMax        =   max(JdB(:));
%     JNorm       =   JdB - JMax;
%     JNorm(JNorm < -35)  =   -35;
%     
% 
%     
%     % normalized range-azimuth map
%     figure(44)
%     imagesc(vAngDeg, vRangeExt, JNorm);
%     xlabel('Ang (�)');
%     ylabel('R (m)');
%     colormap('jet');  
%     colorbar
%     
%     % generate polar plot, non-scaled range azimuth
%     figure(55);
%     surf(mX,mY, 20*log10(abs(JOpt))); 
%     shading flat;
%     view(0,90);
%     axis equal
%     xlabel('x (m)');
%     ylabel('y (m)');
%     colormap('jet');
%     drawnow; 
%     
    % plot image
    if frmno<100
        frmstr=strcat('frm',num2str(frmno));
    elseif frmno<1000
        frmstr=strcat('frm',num2str(frmno));
    else
        frmstr=strcat('frm',num2str(frmno));
    end
    p_im=strcat(pcam,frmstr,'.jpg');
    if exist(p_im,'file')>0
        cam_image=imread(p_im);
        cam_image=imrotate(cam_image,180);
        figure(66)
%         subplot(211)
        imshow(cam_image)
        figure(44)
%         subplot(2,1,2)
        imagesc(vAngDeg,vRangeExt, 20.*log10(abs(JOpt)));
        xlabel('Ang ()');
        ylabel('R (m)');
        colormap('jet');   
        colorbar
        pause(0.1)
    else 
        disp('p_im does not exist.')
    end

    
end


