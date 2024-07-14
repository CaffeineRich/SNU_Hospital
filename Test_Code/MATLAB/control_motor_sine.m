% T모터 2개 Sine파 제어

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
stop(hcan)
hcan = canChannel('PEAK-System','PCAN_USBBUS1');
if hcan.InitializationAccess
    configBusSpeed(hcan, 1000000);
end
% 채널 시작
start(hcan)                                                     

global t                                                        % 사인파 x축 초기화
t=1;

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
% 명령 입력
key=input('Command(exit, enter, o, p, s, sine, two, stop): ','s');
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
        case 'sine' % Sine Wave
            motor_id=input('ID: ');
            kp_a=input('KP: ');
            if isempty(kp_a)|kp_a<kp_min|kp_a>kp_max kp_a=default; end
            kd_a=input('KD: ');
            if isempty(kd_a)|kd_a<kd_min|kd_a>kd_max kd_a=default; end
            sine_wave(motor_id,kp_a,kd_a);
            control_start;
        case 'two' % Two Sine Waves
            motor_id1=input('ID1: ');
            motor_id2=input('ID2: ');
            kp_a=input('KP: ');
            if isempty(kp_a)|kp_a<kp_min|kp_a>kp_max kp_a=default; end
            kd_a=input('KD: ');
            if isempty(kd_a)|kd_a<kd_min|kd_a>kd_max kd_a=default; end
            two_wave(motor_id1,motor_id2,kp_a,kd_a);
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
msg_s.Data = mit_input_data(0,0,0,0,0);
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
global hcan
% send
msg_s = canMessage(motor_id,false,8);
msg_s.Data = mit_input_data(pos_a,vel_a,kp_a,kd_a,toq_a);
transmit(hcan,msg_s);
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
msg_s.Data = mit_input_data(0,0,0,0,0);
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

function sine_wave(motor_id,kp_a,kd_a)                          % 사인파 Timer 설정
global wave_s
wave_s = timer;
wave_s.ExecutionMode='fixedRate';
wave_s.StartDelay=0;
wave_s.Period=0.001;
wave_s.TasksToExecute=1333;
wave_s.TimerFcn=@(~,~)sine_start(motor_id,kp_a,kd_a);
wave_s.StopFcn=@(~,~)sine_stop;
tic
start(wave_s);
toc
end

function sine_start(motor_id,kp_a,kd_a)                         % 사인파 모터 구동 및 플롯
global t
pos_s=sin(3*pi/1000*t);
pos_r=play(motor_id,pos_s,0,kp_a,kd_a,0);
plot(t,pos_s,'r*',t,pos_r,'b*');
hold on
t=t+1;
end

function sine_stop()                                            % 사인파 Timer 종료
global wave_s
delete(wave_s)
end

function two_wave(motor_id1,motor_id2,kp_a,kd_a)                % 모터 2개 사인파 Timer 설정
global wave_two
wave_two = timer;
wave_two.ExecutionMode='fixedRate';
wave_two.StartDelay=0;
wave_two.Period=0.001;
wave_two.TasksToExecute=800;
wave_two.TimerFcn=@(~,~)two_start(motor_id1,motor_id2,kp_a,kd_a);
wave_two.StopFcn=@(~,~)two_stop;
tic
start(wave_two);
toc
end

function two_start(motor_id1,motor_id2,kp_a,kd_a)               % 모터 2개 사인파 모터 구동 및 플롯
global t
pos_s1=sin(pi/200*t);
pos_s2=sin(pi/200*t);
pos_r1=play(motor_id1,pos_s1,0,kp_a,kd_a,0);
pos_r2=play(motor_id2,pos_s2,0,kp_a,kd_a,0);
subplot(2,1,1)
plot(t,pos_s1,'r*',t,pos_r1,'b*');
hold on
subplot(2,1,2)
plot(t,pos_s2,'r*',t,pos_r2,'b*');
hold on
t=t+1;
end

function two_stop()                                             % 모터 2개 사인파 Timer 종료
global wave_two
delete(wave_two)
end

function control_stop()                                         % 메인 종료
global hcan
stop(hcan);
disp('over');
end
