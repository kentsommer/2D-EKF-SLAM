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
A=[];
bytesize=0;
Alength=0;
figure(1)
hold on;
while(1)
    finfo=dir('odomRun.txt');
    if finfo.bytes>bytesize
        newA=dlmread('odomRun.txt','',Alength,0);
        bytesize=finfo.bytes;
        plot(newA(:,1),newA(:,2),'b*');
        A=[A;newA];
        Alength=size(A,1);
        pause(.1)
    end

end