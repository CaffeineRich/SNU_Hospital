% T모터 2개 2절링크 제어

clear;
clc;
close all
format compact

tims = timerfindall;                                            % Timer 초기화
if ~isempty(tims)
    stop(tims);
    delete(tims);
end

global hcan                                                     % 채널 설정
% 남아있는 채널 제거
stop(hcan) % 맨 처음 시작할 땐 주석처리 해야함
hcan = canChannel('PEAK-System','PCAN_USBBUS1');
if hcan.InitializationAccess
    configBusSpeed(hcan, 1000000);
end
% 채널 시작
start(hcan)

global last_target_1                                            % 최신 타겟 초기화
global last_target_2
global last_target_3
global last_target_4
last_target_1=0;
last_target_2=0;
last_target_3=0;
last_target_4=0;

control_start;                                                  % 메인

function control_start()                                        % 메인 시작
% 파라미터 범위
default=0;
pos_max=12.5;
pos_min=-12.5;
vel_max=8;
vel_min=-8;
kp_max=500;
kp_min=0;
kd_max=5;
kd_min=0;
toq_max=144;
toq_min=-144;
% arm 초기위치
x_def=0;
y_def=100;
% 명령 입력
key=input('Command(exit, enter, o, p, s, arm, stop): ','s');
    switch key
        case 'a' % Test
            fprintf('hello\n');
            control_start;
        case 'exit' % Exit
            motor_id=input('Motor ID: ');
            exit(motor_id);
            fprintf('Stop %d motor\n',motor_id);
            control_start;
        case 'enter' % Run
            motor_id=input('Motor ID: ');
            enter(motor_id);
            fprintf('Run ID%d motor\n',motor_id);
            control_start;
        case 'o' % Set Origin
            motor_id=input('Motor ID: ');
            origin(motor_id);
            fprintf('Set ID%d motor to 0rad\n',motor_id);
            control_start;
        case 'p' % Motor Setting
            motor_id=input('ID: ');
            if isempty(motor_id)|motor_id<1 control_start; end
            pos_a=input('Position: ');
            if isempty(pos_a)|pos_a<pos_min|pos_a>pos_max pos_a=default; end
            vel_a=input('Velocity: ');
            if isempty(vel_a)|vel_a<vel_min|vel_a>vel_max vel_a=default; end
            kp_a=input('KP: ');
            if isempty(kp_a)|kp_a<kp_min|kp_a>kp_max kp_a=default; end
            kd_a=input('KD: ');
            if isempty(kd_a)|kd_a<kd_min|kd_a>kd_max kd_a=default; end
            toq_a=input('Torque: ');
            if isempty(toq_a)|toq_a<toq_min|toq_a>toq_max toq_a=default; end
            % Rotate
            move_motor(motor_id,pos_a,vel_a,kp_a,kd_a,toq_a);
            control_start;
        case 's' % Status
            motor_id=input('ID: ');
            status(motor_id);
            control_start;
        case 'arm' % Arm Control
            x=input('x: ');
            if isempty(x) x=x_def; end
            y=input('y: ');
            if isempty(y)|y<y_def y=y_def; end
            arm_start(x,y);
            control_start;
        case 'stop' % Finish
            control_stop;
        otherwise % Blank
            control_start;
    end
end

function exit(motor_id)                                         % Exit
global hcan
msg = canMessage(motor_id,false,8);
msg.Data = ([255 255 255 255 255 255 255 253]);
transmit(hcan,msg);
end

function enter(motor_id)                                        % Run
global hcan
msg = canMessage(motor_id,false,8);
msg.Data = ([255 255 255 255 255 255 255 252]);
transmit(hcan,msg);
end

function origin(motor_id)                                       % Set Origin
global hcan
msg_s = canMessage(motor_id,false,8);
msg_s.Data = mit_input_data(0,0,0,0,0);
transmit(hcan,msg_s);
pause(0.1)
msg_s.Data = ([255 255 255 255 255 255 255 254]);
transmit(hcan,msg_s);
pause(0.1)
msg_s.Data = ([255 255 255 255 255 255 255 252]);
for i=1:100
    transmit(hcan,msg_s);
    pause(0.01)
end
msg_r = receive(hcan, Inf);
a=msg_r(1,end).Data;
d=double(a);
val=mit_output_data(d(1),d(2),d(3),d(4),d(5),d(6));
fprintf('\nID: %d \t pos: %.3frad \t vel: %.3frad/s \t toq: %.3fNM\n',val(1),val(2),val(3),val(4));
end

function move_motor(motor_id,pos_a,vel_a,kp_a,kd_a,toq_a)       % 모터 Timer 설정
global move
move = timer;
move.ExecutionMode='fixedRate';
move.StartDelay=0;
move.Period=0.01;
move.TasksToExecute=100;
move.TimerFcn=@(~,~)play(motor_id,pos_a,vel_a,kp_a,kd_a,toq_a);
move.StopFcn=@(~,~)stop_move;
start(move);
end

function position=play(motor_id,pos_a,vel_a,kp_a,kd_a,toq_a)    % 모터 메시지 송수신
global last_target_1
global last_target_2
global last_target_3
global last_target_4
global hcan
% send
msg_s = canMessage(motor_id,false,8);
msg_s.Data = mit_input_data(pos_a,vel_a,kp_a,kd_a,toq_a);
transmit(hcan,msg_s);
if motor_id==1 last_target_1=pos_a;
elseif motor_id==2 last_target_2=pos_a;
elseif motor_id==3 last_target_3=pos_a;
elseif motor_id==4 last_target_4=pos_a;
end
% receive
msg_r = receive(hcan, Inf);
a=msg_r(1,end).Data;
d=double(a);
val=mit_output_data(d(1),d(2),d(3),d(4),d(5),d(6));
fprintf('\nID: %d \t pos: %.3frad \t vel: %.3frad/s \t toq: %.3fNM\n',val(1),val(2),val(3),val(4));
position=val(2);
end

function data = mit_input_data(pos_a,vel_a,kp_a,kd_a,toq_a)     % 실제 입력값 -> 모터 코드값
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

function val=mit_output_data(a,b,c,d,e,f)                       % 모터 코드값 -> 실제 출력값
id=a;
pos=b*16^2+c;
vel=d*16+fix(e/16);
toq=mod(e,16)*16^2+f;
pos_a=round((pos-32767)/2621,3);
vel_a=round((vel-2047)/256,3);
toq_a=round((toq-2047)/14.22,3);
val=[id,pos_a,vel_a,toq_a];
end

function stop_move()                                            % 모터 Timer 종료
global move
delete(move);
end

function status(motor_id)                                       % Status
global hcan
msg_s = canMessage(motor_id,false,8);
msg_s.Data = ([255 255 255 255 255 255 255 252]);
for i=1:100
    transmit(hcan,msg_s);
    pause(0.01)
end
msg_r = receive(hcan,Inf);
a=msg_r(1,end).Data;
d=double(a);
val=mit_output_data(d(1),d(2),d(3),d(4),d(5),d(6));
fprintf('ID: %d \t pos: %.3frad \t vel: %.3frad/s \t toq: %.3fNM\n',val(1),val(2),val(3),val(4))
end

function arm_start(x,y)                                         % 모터 2개 암 Timer 설정
% 좌표 -> 회전각도
[rad_1,rad_2]=pos_to_ang(x,y);
% 최신 타겟
global last_target_1
global last_target_2
global last_target_3
global last_target_4
lt_1=last_target_1;
lt_2=last_target_2;
lt_3=last_target_3;
lt_4=last_target_4;
global arm
arm = timer;
arm.ExecutionMode='fixedRate';
arm.StartDelay=0;
arm.Period=0.001;
arm.TasksToExecute=200;
arm.TimerFcn=@(~,~)arm_move(rad_1,rad_2,lt_1,lt_2);
arm.StopFcn=@(~,~)arm_stop;
tic
start(arm);
toc
end

function [rad_1,rad_2]=pos_to_ang(x,y)                          % 좌표 -> 회전각도
x_def=0;
y_def=100;
leg_1=160;
leg_2=160;
D_def=(x_def^2+y_def^2-leg_1^2-leg_2^2)/(2*leg_1*leg_2);
ang2_def=acos(D_def);
ang1_def=atan(y_def/x_def)-atan(leg_2*sin(ang2_def)/(leg_1+leg_2*cos(ang2_def)));
D=(x^2+y^2-leg_1^2-leg_2^2)/(2*leg_1*leg_2);
ang_2=acos(D);
ang_1=atan(y/x)-atan(leg_2*sin(ang_2)/(leg_1+leg_2*cos(ang_2)));
rad_1=ang_1-ang1_def;
rad_2=ang2_def-ang_2;
end

function arm_move(rad_1,rad_2,lt_1,lt_2)                        % 모터 2개 암 모터 구동 및 플롯
global z
k1=rad_1-lt_1;
k2=rad_2-lt_2;
if k1>0
    pos_s1=k1*sin(pi/400*z)+lt_1;
    toq_s1=0.1;
else
    pos_s1=k1*sin(pi/400*z)+lt_1;
    toq_s1=-0.1;
end

if k2>0
    pos_s2=k2*sin(pi/400*z)+lt_2;
    toq_s2=0.1;
else
    pos_s2=k2*sin(pi/400*z)+lt_2;
    toq_s2=-0.1;
end
pos_r1=play(1,pos_s1,0,500,5,toq_s1);
pos_r2=play(2,pos_s2,0,500,5,toq_s2);
subplot(2,1,1)
plot(z,pos_s1,'r*',z,pos_r1,'b*');
hold on
subplot(2,1,2)
plot(z,pos_s2,'r*',z,pos_r2,'b*');
hold on
z=z+1;
end

function arm_stop()                                             % 모터 2개 암 Timer 종료
global arm
delete(arm)
end

function control_stop()                                         % 메인 종료
global hcan
stop(hcan);
disp('over');
end
