%{
sizeA=[2 inf];
formatSpec = '%f %f';
fileID=fopen('odomRun.txt','r');
A=fscanf(fileID, formatSpec, [2 10000]);
A=A';
plot(A(:,1),A(:,2)) 
A=dlmread('odomRun.txt');
%}
clear all
close all

savevideo=0;

Odomrun=[];
Odomrunbytesize=0;
Odomrunlength=0;

Scan=[];
NewScanPlot=plot(0,0,'MarkerSize', 0.00001);
Scanbytesize=0;
Scanlength=0;

Features=[];
Featuresbytesize=0;
Featureslength=0;

knownFeatures=[];
knownFeaturesbytesize=0;
knownFeatureslength=0;
knownFeaturesPlot=plot(0,0,'MarkerSize', 0.00001);


Pxy=[];
Pxybytesize=0;
Pxylength=0;
Ellipseplot=plot(0,0,'MarkerSize', 0.00001);
currXY=[0;0];

fighandle=figure(1);
hold on;
dt=.1;

if savevideo
V=VideoWriter('video.avi');
V.FrameRate=1/dt;
open(V);
fighandle.Resize='off';
fighandle.Position=[0 0 1920 1080];
end


while(1)
    Odomrunfinfo=dir('../data/odom/odomRun.txt');
    if size(Odomrunfinfo,1)>0&&~isempty(Odomrunfinfo.bytes)
    if Odomrunfinfo.bytes>Odomrunbytesize
        newA=dlmread('../data/odom/odomRun.txt','',Odomrunlength,0);
        Odomrunbytesize=Odomrunfinfo.bytes;
        plot(newA(:,1),newA(:,2),'b*');
        %Odomrun=[Odomrun;newA];
        %Odomrunlength=size(Odomrun,1);
        Odomrunlength=Odomrunlength+size(newA,1);
        currXY=[newA(end,1);newA(end,2)];
    end
    end
     Scaninfo=dir('../data/scan/scanRun.txt');
    if size(Scaninfo,1)>0&&~isempty(Scaninfo.bytes)
     if Scaninfo.bytes>Scanbytesize
        newScan=dlmread('../data/scan/scanRun.txt','',Scanlength,0);
        Scanbytesize=Scaninfo.bytes;
        delete(NewScanPlot);
        plot(newScan(:,1),newScan(:,2),'k*');% old scans are black, current one is red
        NewScanPlot=plot(newScan(:,1),newScan(:,2),'r*');
        %Scan=[Scan;newScan];
        %Scanlength=size(Scanlength,1);
        Scanlength=Scanlength+size(newScan,1);
    end
    end
    
    
    Featuresinfo=dir('../data/features/featuresRun.txt');
    if size(Featuresinfo,1)>0&&~isempty(Featuresinfo.bytes)
    if Featuresinfo.bytes>Featuresbytesize
        newFeature=dlmread('../data/features/featuresRun.txt','',Featureslength,0);
        Featuresbytesize=Featuresinfo.bytes;
        plot(newFeature(:,1),newFeature(:,2),'g*');
        %Features=[Features;newFeature];
        %Featureslength=size(Featureslength,1);
        Featureslength=Featureslength+size(newFeature,1);
    end
    end    
    
    
    Pxyinfo=dir('../data/cov/covRun.txt');
    if  size(Pxyinfo,1)>0&&~isempty(Pxyinfo.bytes)
    if Pxyinfo.bytes>Pxybytesize
        %newA=dlmread('../data/odom/odomRun.txt','',Odomrunlength,0);
       %newA=dlmread('../data/odom/odomRun.txt','',Odomrunlength,0);
        NewPxy=dlmread('../data/cov/covRun.txt','',Pxylength,0);
        Pxy=[NewPxy(end,1),NewPxy(end,2);NewPxy(end,3),NewPxy(end,4)];
        
        Pxybytesize=Pxyinfo.bytes;
        
        %Odomrun=[Odomrun;newA];
        %Odomrunlength=size(Odomrun,1);
        Pxylength=Pxylength+size(NewPxy,1);
        delete( Ellipseplot)
       [EllipseX,EllipseY]=  plot_error_ellipse_plotting(currXY,Pxy);
       Ellipseplot=plot(EllipseX,EllipseY,'g');
    end
    end
    
%      knownFeaturesinfo=dir('../data/features/knownfeaturesRun.txt');
%     if size(knownFeaturesinfo,1)>0&& ~isempty(knownFeaturesinfo.bytes)
%     if knownFeaturesinfo.bytes>knownFeaturesbytesize
%         knownnewFeature=dlmread('../data/features/knownfeaturesRun.txt','',knownFeatureslength,0);
%         knownFeaturesbytesize=knownFeaturesinfo.bytes;
%         delete(knownFeaturesPlot);
%         knownFeaturesPlot=plot(knownnewFeature(:,1),knownnewFeature(:,2),'g*');
%         %Features=[Features;newFeature];
%         %Featureslength=size(Featureslength,1);
%         knownFeatureslength=knownFeatureslength+size(knownnewFeature,1);
%     end
%     end   
    
    pause(dt)
    if savevideo
    currFrame = getframe;
    writeVideo(V,currFrame)
    end
end
if savevideo
close(V)
end