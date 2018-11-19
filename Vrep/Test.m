%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Sapienza University of Rome                                             %
% Department of Computer, Control, and Management Engineering             %
% Autonomous and mobile robotics                                          %
% Cousre Project                                                          %
% By: Amr Afifi                                                           %
%     Paolo Fanello                                                       %
%     Youssef Abou Dorra                                                  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
clear all
close all
clc
%% Init V-rep Connection
k_final = 300;
pos_J0 = zeros(k_final,1);
pos_J1 = zeros(k_final,1);
pos_J2 = zeros(k_final,1);
pos_J3 = zeros(k_final,1);
pos_J4 = zeros(k_final,1);
pos_base = zeros(k_final,3);
orn_base = zeros(k_final,3);


vel_lin_J0 = zeros(k_final,3);
vel_ang_J0 = zeros(k_final,3);

% Using the prototype file (remoteApiProto.m)
vrep=remApi('remoteApi');

% Close all opened connections
vrep.simxFinish(-1);

% We try to connect on port 19997 where there should be a continuous remote
% API server service already running and pre-enabled for synchronous mode.
clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);

if (clientID>-1)
    disp('Connected')
    
    % enable the synchronous mode on the client:
     vrep.simxSynchronous(clientID,true);
    
    % retrieve Base Handle 
    [returnCode_base,base]=vrep.simxGetObjectHandle(clientID,'ME_Platfo2_sub1',vrep.simx_opmode_blocking); 
    
    [returnCode_base,ww]=vrep.simxGetObjectHandle(clientID,'intermediateLink_fr',vrep.simx_opmode_blocking); 
    % retrieve wheel joint handels
    [returnCode_fl,wheel_fl]=vrep.simxGetObjectHandle(clientID,'rollingJoint_fl',vrep.simx_opmode_blocking);
    [returnCode_fr,wheel_fr]=vrep.simxGetObjectHandle(clientID,'rollingJoint_fr',vrep.simx_opmode_blocking);
    [returnCode_rl,wheel_rl]=vrep.simxGetObjectHandle(clientID,'rollingJoint_rl',vrep.simx_opmode_blocking);
    [returnCode_rr,wheel_rr]=vrep.simxGetObjectHandle(clientID,'rollingJoint_rr',vrep.simx_opmode_blocking);
    
    % retrieve arm joints handels
    [returnCode_J0,arm_J0]=vrep.simxGetObjectHandle(clientID,'youBotArmJoint0',vrep.simx_opmode_blocking);
    [returnCode_J1,arm_J1]=vrep.simxGetObjectHandle(clientID,'youBotArmJoint1',vrep.simx_opmode_blocking);
    [returnCode_J2,arm_J2]=vrep.simxGetObjectHandle(clientID,'youBotArmJoint2',vrep.simx_opmode_blocking);
    [returnCode_J3,arm_J3]=vrep.simxGetObjectHandle(clientID,'youBotArmJoint3',vrep.simx_opmode_blocking);
    [returnCode_J4,arm_J4]=vrep.simxGetObjectHandle(clientID,'youBotArmJoint4',vrep.simx_opmode_blocking);
    
    % Reset all wheels Velocity
    vrep.simxSetJointTargetVelocity(clientID,wheel_fl,0,vrep.simx_opmode_blocking);
    vrep.simxSetJointTargetVelocity(clientID,wheel_fr,0,vrep.simx_opmode_blocking);
    vrep.simxSetJointTargetVelocity(clientID,wheel_rl,0,vrep.simx_opmode_blocking);
    vrep.simxSetJointTargetVelocity(clientID,wheel_rr,0,vrep.simx_opmode_blocking);
    
    % Reset all arm joints Velocity
    vrep.simxSetJointTargetVelocity(clientID,arm_J0,0,vrep.simx_opmode_blocking);
    vrep.simxSetJointTargetVelocity(clientID,arm_J1,0,vrep.simx_opmode_blocking);
    vrep.simxSetJointTargetVelocity(clientID,arm_J2,0,vrep.simx_opmode_blocking);
    vrep.simxSetJointTargetVelocity(clientID,arm_J3,0,vrep.simx_opmode_blocking);
    vrep.simxSetJointTargetVelocity(clientID,arm_J4,0,vrep.simx_opmode_blocking);
    
    % Start Simulation
    vrep.simxStartSimulation(clientID,vrep.simx_opmode_blocking);

  
    pause()
    
    % get initial configuration  
         
         % Get All Joints Position         
         [returnCode_J0, pos_J0_o] = vrep.simxGetJointPosition(clientID,arm_J0,vrep.simx_opmode_blocking);
         [returnCode_J1, pos_J1_o] = vrep.simxGetJointPosition(clientID,arm_J1,vrep.simx_opmode_blocking);
         [returnCode_J2, pos_J2_o] = vrep.simxGetJointPosition(clientID,arm_J2,vrep.simx_opmode_blocking);
         [returnCode_J3, pos_J3_o] = vrep.simxGetJointPosition(clientID,arm_J3,vrep.simx_opmode_blocking);
         [returnCode_J4, pos_J4_o] = vrep.simxGetJointPosition(clientID,arm_J4,vrep.simx_opmode_blocking);
         
         % Get Base Position
         [returnCode_base, pos_base_o] = vrep.simxGetObjectPosition(clientID,base,-1,vrep.simx_opmode_blocking); 
       
         % Get Base angle theta
         [returnCode_base_t, orn_base_o] = vrep.simxGetObjectOrientation(clientID,base,-1,vrep.simx_opmode_blocking); 
         
         L1 = 235.5/1000;
         L2 = 150.23/1000;
         R =  50/1000;
         
         
         
            L = (1/R)*[-1  +1 -(L1+L2);
                       -1  -1 +(L1+L2);
                       -1  -1 -(L1+L2);
                       -1  +1 +(L1+L2);]
    
    for k = 1:k_final
        
         [returnCode_base_t, orn_base(k,:)] = vrep.simxGetObjectOrientation(clientID,base,-1,vrep.simx_opmode_blocking);      
           
        
         % Get All Joints Position
         
         [returnCode_J0, pos_J0(k,:)] = vrep.simxGetJointPosition(clientID,arm_J0,vrep.simx_opmode_blocking);
         [returnCode_J1, pos_J1(k,:)] = vrep.simxGetJointPosition(clientID,arm_J1,vrep.simx_opmode_blocking);
         [returnCode_J2, pos_J2(k,:)] = vrep.simxGetJointPosition(clientID,arm_J2,vrep.simx_opmode_blocking);
         [returnCode_J3, pos_J3(k,:)] = vrep.simxGetJointPosition(clientID,arm_J3,vrep.simx_opmode_blocking);
         [returnCode_J4, pos_J4(k,:)] = vrep.simxGetJointPosition(clientID,arm_J4,vrep.simx_opmode_blocking);
         
         % Get Base Position
         [returnCode_base, pos_base(k,:)] = vrep.simxGetObjectPosition(clientID,base,-1,vrep.simx_opmode_blocking);          
         
         

          if k >200
              
          w = L*rotate(orn_base(k,3))*[0.1;0;0] ;   
              
          vrep.simxSetJointTargetVelocity(clientID,wheel_rr,w(1),vrep.simx_opmode_blocking);
          vrep.simxSetJointTargetVelocity(clientID,wheel_rl,w(2),vrep.simx_opmode_blocking);
          vrep.simxSetJointTargetVelocity(clientID,wheel_fr,w(3),vrep.simx_opmode_blocking);
          vrep.simxSetJointTargetVelocity(clientID,wheel_fl,w(4),vrep.simx_opmode_blocking);
          else
          if k>100
          w = L*rotate(orn_base(k,3))*[0;0;0.5] ;    
          vrep.simxSetJointTargetVelocity(clientID,wheel_rr,w(1),vrep.simx_opmode_blocking);
          vrep.simxSetJointTargetVelocity(clientID,wheel_rl,w(2),vrep.simx_opmode_blocking);
          vrep.simxSetJointTargetVelocity(clientID,wheel_fr,w(3),vrep.simx_opmode_blocking);
          vrep.simxSetJointTargetVelocity(clientID,wheel_fl,w(4),vrep.simx_opmode_blocking);
          
          else             
          w = L*rotate(orn_base(k,3))*[0.1;0;0];  
          vrep.simxSetJointTargetVelocity(clientID,wheel_rr,w(1),vrep.simx_opmode_blocking);
          vrep.simxSetJointTargetVelocity(clientID,wheel_rl,w(2),vrep.simx_opmode_blocking);
          vrep.simxSetJointTargetVelocity(clientID,wheel_fr,w(3),vrep.simx_opmode_blocking);
          vrep.simxSetJointTargetVelocity(clientID,wheel_fl,w(4),vrep.simx_opmode_blocking);
          end
          end
          vrep.simxSetJointTargetVelocity(clientID,arm_J0,-0.2,vrep.simx_opmode_blocking);
        
          vrep.simxSynchronousTrigger(clientID);
          
          
          
    end
    
    
          vrep.simxSetJointTargetVelocity(clientID,wheel_fl,0,vrep.simx_opmode_blocking);
          vrep.simxSetJointTargetVelocity(clientID,wheel_fr,0,vrep.simx_opmode_blocking);
          vrep.simxSetJointTargetVelocity(clientID,wheel_rl,0,vrep.simx_opmode_blocking);
          vrep.simxSetJointTargetVelocity(clientID,wheel_rr,0,vrep.simx_opmode_blocking);

          % Reset all arm joints Velocity
          vrep.simxSetJointTargetVelocity(clientID,arm_J0,0,vrep.simx_opmode_blocking);
          vrep.simxSetJointTargetVelocity(clientID,arm_J1,0,vrep.simx_opmode_blocking);
          vrep.simxSetJointTargetVelocity(clientID,arm_J2,0,vrep.simx_opmode_blocking);
          vrep.simxSetJointTargetVelocity(clientID,arm_J3,0,vrep.simx_opmode_blocking);
          vrep.simxSetJointTargetVelocity(clientID,arm_J4,0,vrep.simx_opmode_blocking);
          
          % stop the simulation:
          vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking);
          
          % Now close the connection to V-REP:    
          vrep.simxFinish(clientID);
    
    
else
    disp('Failed connecting to remote API server');
end
    vrep.delete(); % call the destructor!
    disp('Program ended');
