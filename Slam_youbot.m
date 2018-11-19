clear all 
close all
clc

addpath Vrep

disp('Program started');
% Use the following line if you had to recompile remoteApi
%vrep = remApi('remoteApi', 'extApi.h');
vrep = remApi('remoteApi');
vrep.simxFinish(-1);
clientID = vrep.simxStart('127.0.0.1', 19997, true, true, 2000, 5);

% If you get an error like: 
%   Remote API function call returned with error code: 64. Explanation: simxStart was not yet called.
% Make sure your code is within a function! You cannot call V-REP from a script. 

if clientID < 0
    disp('Failed connecting to remote API server. Exiting.');
    vrep.delete();
    return;
end
fprintf('Connection %d to remote API server open.\n', clientID);

vrep.simxSynchronous(clientID,true);

% Start Simulation 
vrep.simxStartSimulation(clientID,vrep.simx_opmode_blocking);

% Retrieve Base Handle
[returnCode_base,vrep_base_handle]=vrep.simxGetObjectHandle(clientID,'ME_Platfo2_sub1',vrep.simx_opmode_blocking);

% Retrieve Wheels Joints handles
[returnCode_fl,vrep_wheel_fl_handle]=vrep.simxGetObjectHandle(clientID,'rollingJoint_fl',vrep.simx_opmode_blocking);
[returnCode_fr,vrep_wheel_fr_handle]=vrep.simxGetObjectHandle(clientID,'rollingJoint_fr',vrep.simx_opmode_blocking);
[returnCode_rl,vrep_wheel_rl_handle]=vrep.simxGetObjectHandle(clientID,'rollingJoint_rl',vrep.simx_opmode_blocking);
[returnCode_rr,vrep_wheel_rr_handle]=vrep.simxGetObjectHandle(clientID,'rollingJoint_rr',vrep.simx_opmode_blocking);

% Retrieve Laser
[returnCode,laser]=vrep.simxGetObjectHandle(clientID,'SICK_S300_fast',vrep.simx_opmode_blocking);
[returnCode, measLaser] = vrep.simxGetStringSignal(clientID,'measuredDataAtThisTime', vrep.simx_opmode_streaming );

% Reset all wheels Velocity
vrep.simxSetJointTargetVelocity(clientID,vrep_wheel_fl_handle,0,vrep.simx_opmode_blocking);
vrep.simxSetJointTargetVelocity(clientID,vrep_wheel_fr_handle,0,vrep.simx_opmode_blocking);
vrep.simxSetJointTargetVelocity(clientID,vrep_wheel_rl_handle,0,vrep.simx_opmode_blocking);
vrep.simxSetJointTargetVelocity(clientID,vrep_wheel_rr_handle,0,vrep.simx_opmode_blocking);

disp('Press any key to Start Simulation');
pause()

% Get Base Position --- don't now if needed
[returnCode_base, vrep_pos_base_initial] = vrep.simxGetObjectPosition(clientID,vrep_base_handle,-1,vrep.simx_opmode_blocking);

% Get Base Orientation
[returnCode_base_t, vrep_orn_base_initial] = vrep.simxGetObjectOrientation(clientID,vrep_base_handle,-1,vrep.simx_opmode_blocking);

% Enable Streaming mode
vrep.simxGetObjectFloatParameter(clientID,vrep_wheel_fl_handle,vrep.sim_jointfloatparam_velocity,vrep.simx_opmode_streaming );
vrep.simxGetObjectFloatParameter(clientID,vrep_wheel_fr_handle,vrep.sim_jointfloatparam_velocity,vrep.simx_opmode_streaming );
vrep.simxGetObjectFloatParameter(clientID,vrep_wheel_rl_handle,vrep.sim_jointfloatparam_velocity,vrep.simx_opmode_streaming );
vrep.simxGetObjectFloatParameter(clientID,vrep_wheel_rr_handle,vrep.sim_jointfloatparam_velocity,vrep.simx_opmode_streaming );

vrep.simxGetObjectPosition(clientID,vrep_base_handle,-1,vrep.simx_opmode_streaming);
vrep.simxGetObjectOrientation(clientID,vrep_base_handle,-1,vrep.simx_opmode_streaming);
vrep.simxGetObjectVelocity(clientID,vrep_base_handle,vrep.simx_opmode_streaming);



disp(' ')
disp('Simulation Started')
disp(' ')

for i = 1:100
    fprintf('iteration num. %i \n', i);
    vrep.simxSetJointTargetVelocity(clientID,vrep_wheel_fl_handle,-5,vrep.simx_opmode_oneshot);
    vrep.simxSetJointTargetVelocity(clientID,vrep_wheel_fr_handle,-5,vrep.simx_opmode_oneshot);
    vrep.simxSetJointTargetVelocity(clientID,vrep_wheel_rl_handle,-5,vrep.simx_opmode_oneshot);
    vrep.simxSetJointTargetVelocity(clientID,vrep_wheel_rr_handle,-5,vrep.simx_opmode_oneshot);
    [returnCode, measLaser] = vrep.simxGetStringSignal(clientID,'measuredDataAtThisTime', vrep.simx_opmode_buffer);
    measLaser = vrep.simxUnpackFloats(measLaser);
    x = [];
    y = [];
    for k=0:(size(measLaser,2)/3-1)
        x = [x, -measLaser(k*3+2)]; % I've inverted x and y. Just for visualization
        y = [y, measLaser(k*3+1)];
    end
    
    plot(x, y);
    drawnow;
    vrep.simxSynchronousTrigger(clientID); % SUPER IMPORTANT!!!
end
    
[returnCode]=vrep.simxSetJointTargetVelocity(clientID,vrep_wheel_fl_handle,0,vrep.simx_opmode_blocking);

vrep.simxSynchronousTrigger(clientID);

% stop the simulation:
vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking);

% Now close the connection to V-REP:    
vrep.simxFinish(clientID);

disp('Simulation Finished')
    