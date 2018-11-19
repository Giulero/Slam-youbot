function main()

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
    disp('Press any key to Start Simulation');
    pause()
    
    disp(' ')
    disp('Simulation Started')
    disp(' ')
    
    for i = 1:100
        fprintf('iteration num. %i \n', i);
    end
    
    vrep.simxSynchronousTrigger(clientID);

    % stop the simulation:
    vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking);

    % Now close the connection to V-REP:    
    vrep.simxFinish(clientID);
    
    disp('Simulation Finished')
    
end