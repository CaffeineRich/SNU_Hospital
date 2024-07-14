% T모터 메시지 수신

clear;
clc;
format compact

global hcan
hcan = canChannel('PEAK-System','PCAN_USBBUS1')
if hcan.InitializationAccess
    configBusSpeed(hcan, 1000000)
end
start (hcan)
pause(0.001)

% timer remove
tims = timerfindall;
if ~isempty(tims)
    stop(tims);
    delete(tims);
end

global htim1
htim1 = timer;
htim1.ExecutionMode='fixedRate';
htim1.StartDelay=0.001;
htim1.Period=0.001;
htim1.TasksToExecute=3;
htim1.TimerFcn=@(~,~)TimerFcn1;
htim1.StopFcn=@(~,~)StopFcn1;
start(htim1);

function val=mit_output_data(a,b,c,d,e,f)
id=a;
pos=b*16^2+c;
vel=d*16+fix(e/16);
toq=mod(e,16)*16^2+f;
pos_a=round((pos-32767)/2621,3);
vel_a=round((vel-2047)/256,3);
toq_a=round((toq-2047)/14.22,3);
val=[id,pos_a,vel_a,toq_a];
end

function TimerFcn1()
global hcan
msg = receive(hcan, 2);
a=msg(1,2).Data;
d=double(a);
val=mit_output_data(d(1),d(2),d(3),d(4),d(5),d(6));
fprintf('ID: %d \t pos: %.3frad \t vel: %.3frad/s \t toq: %.3fNM\n',val(1),val(2),val(3),val(4))
end

function StopFcn1()
global hcan
global htim1
stop(hcan);
clear hcan;
disp('over')
delete(htim1)
end
