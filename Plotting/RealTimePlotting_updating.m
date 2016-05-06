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
Scanbytesize=0;
Scanlength=0;

Features=[];
Featuresbytesize=0;
Featureslength=0;
if savevideo
V=VideoWriter('video.avi');
V.FrameRate=1;
open(V);
end

Pxy=[];
Pxybytesize=0;
Pxylength=0;
Ellipseplot=plot(0,0,'MarkerSize', 0.00001);

figure(1)
hold on;
while(1)
    Odomrunfinfo=dir('../data/odom/odomRun.txt');
    if Odomrunfinfo.bytes>Odomrunbytesize
        newA=dlmread('../data/odom/odomRun.txt','',Odomrunlength,0);
        
        Odomrunbytesize=Odomrunfinfo.bytes;
        plot(newA(:,1),newA(:,2),'b*');
        %Odomrun=[Odomrun;newA];
        %Odomrunlength=size(Odomrun,1);
        Odomrunlength=Odomrunlength+size(newA,1);
        currXY=[newA(end,1);newA(end,2)];
    end
    
     Scaninfo=dir('../data/scan/scanRun.txt');
    if Scaninfo.bytes>Scanbytesize
        newScan=dlmread('../data/scan/scanRun.txt','',Scanlength,0);
        Scanbytesize=Scaninfo.bytes;
        plot(newScan(:,1),newScan(:,2),'r*');
        %Scan=[Scan;newScan];
        %Scanlength=size(Scanlength,1);
        Scanlength=Scanlength+size(newScan,1);
    end
    
    Featuresinfo=dir('../data/features/featuresRun.txt');
    if Featuresinfo.bytes>Featuresbytesize
        newFeature=dlmread('../data/features/featuresRun.txt','',Scanlength,0);
        Featuresbytesize=Featuresinfo.bytes;
        plot(newFeature(:,1),newFeature(:,2),'g*');
        %Features=[Features;newFeature];
        %Featureslength=size(Featureslength,1);
        Featureslength=Featureslength+size(newFeature,1);
    end
        
    
    Pxyinfo=dir('../data/cov/covRun.txt');
    if Pxyinfo.bytes>Pxybytesize
        %newA=dlmread('../data/odom/odomRun.txt','',Odomrunlength,0);
        NewPxy=dlmread('odomRun.txt','',Pxylength,0);
        Pxy=[NewPxy(end,1),NewPxy(end,2);NewPxy(end,3),NewPxy(end,4)];
        
        Pxybytesize=Pxyinfo.bytes;
        
        %Odomrun=[Odomrun;newA];
        %Odomrunlength=size(Odomrun,1);
        Pxylength=Pxylength+size(NewPxy,1);
        delete( Ellipseplot)
       [EllipseX,EllipseY]=  plot_error_ellipse_plotting(currXY,Pxy,'y');
       Ellipseplot=plot(EllipseX,EllipseY,'r');
    end
    
    
    pause(1)
    if savevideo
    currFrame = getframe;
    writeVideo(V,currFrame)
    end
end
close(V)