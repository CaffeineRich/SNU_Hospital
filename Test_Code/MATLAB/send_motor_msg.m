% T모터 메시지 송신

clear;
clc;
format compact

% motor setting
global hcan
motor_id=3;
pos_a=0;
vel_a=0;
kp_a=50;
kd_a=5;
toq_a=0;

% open CAN
hcan = canChannel('PEAK-System','PCAN_USBBUS1');
if hcan.InitializationAccess
    configBusSpeed(hcan, 1000000);
end
start(hcan)

% timer setting
tims = timerfindall;
if ~isempty(tims)
    stop(tims);
    delete(tims);
end

% motor run
global htim1
htim1 = timer;
set(htim1,'ExecutionMode','singleShot');
set(htim1,'StartDelay',0.001);
set(htim1,'TimerFcn',@(~,~)TimerFcn1(3));
set(htim1,'StopFcn',@(~,~)StopFcn1);

% motor position
global htim2
htim2 = timer;
set(htim2,'ExecutionMode','fixedRate');
set(htim2,'StartDelay',0.001);
set(htim2,'Period',0.001);
set(htim2,'TasksToExecute',10);
set(htim2,'TimerFcn',@(~,~)TimerFcn2(3));
set(htim2,'StopFcn',@(~,~)StopFcn2);

% motor activate
start(htim1);

% motor data structure(actual value to code)
function data = mit_input_data(pos_a,vel_a,kp_a,kd_a,toq_a)
pos=32767+2621*pos_a;
vel=2047+256*vel_a;
kp=8.19*kp_a;
kd=819*kd_a;
toq=2047+14.22*toq_a;
data=[];
data(1)=fix(pos/16^2);
data(2)=round(mod(pos,16^2));
data(3)=fix(vel/16);
data(4)=round(mod(vel,16))*16+fix(kp/16^2);
data(5)=round(mod(kp,16^2));
data(6)=fix(kd/16);
data(7)=round(mod(kd,16))*16+fix(toq/16^2);
data(8)=round(mod(toq,16^2));
end

% run motor
function TimerFcn1(motor_id)
global hcan
msg_exit = canMessage(motor_id,false,8);
msg_exit.Data = ([255 255 255 255 255 255 255 253]);
msg_enter = canMessage(motor_id,false,8);
msg_enter.Data = ([255 255 255 255 255 255 255 252]);
transmit(hcan,msg_enter);
end

% move on to timer2
function StopFcn1()
global htim2
start(htim2);
end

% rotate motor
function TimerFcn2(motor_id)
global hcan
msg = canMessage(motor_id,false,8);
msg.Data = mit_input_data(0,0,50,5,0);
transmit(hcan,msg);     
end

% close channel
function StopFcn2()
global hcan
stop(hcan);
clear hcan;
disp('over')
end
