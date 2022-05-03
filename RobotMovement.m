classdef RobotMovement < handle
    % This class will handle the movement of SerialLink Robots and Mesh
    % Objects in a defined workspace environment. It will log details to a
    % log file defined as a property of the class.
    
    properties
        L;      % LogFile
    end
    
    methods
        function self = RobotMovement()
           self.L = log4matlab('Robot_Movement_Log.log');
        end
        
        function [qOut] = MoveRobotToObject(self, robot, loc_T, dAbove, ...
                qGuess, steps)
            %This function takes a SerialLink robot, a desired 4x4 Transformation ...
            %Matrix to reach, and animates a calculated minimum jerk trajectory
            %to the configuration using waypoints.
            
            % TO BE ADDED: COLLISION AVOIDANCE/DETECTION
            
            % Defining and logging to log file for information
            funcName = 'MoveRobotToObject';
            self.L.mlog = {self.L.DEBUG,funcName,['RUNNING FUNCTION: ', ...
                funcName, char(13)]};
            
            % Getting the transform above the desired end location.
            % Define a rotation of PI/2 about X to face End Effector
            % sidewards.
            locA_T = loc_T*transl(0,0,dAbove)*trotx(pi/2)*trotz(pi);

            % Moving to Location
            % Getting required Joint Angles to reach position above using ...
            % Inverse Kinematics
            newQ = robot.ikcon(locA_T, qGuess);

            % Plannning a Trajectory using the Min. Jerk Method
            % (Quintic Polynomial Method)
            % jtraj(Starting Joint Angles, Target Joint Angles, Steps to Take)
            qpMatrix = jtraj(robot.getpos(), newQ, steps);

            % Moving to the spot above the desired location
            for i = 1:steps
               % Plot Robot Moving
               robot.animate(qpMatrix(i, :));
               drawnow();
            end

            % Debugging pose above desired position
            FK = robot.fkine(qpMatrix(end, :));
            [check, dist] = self.compareTwoPositions(FK, locA_T);
            
            self.L.mlog = {self.L.DEBUG,funcName,['The robot transform ' ...
                'above input location is: ',self.L.MatrixToString(FK)]};
            
            if check == true
                self.L.mlog = {self.L.DEBUG,funcName,['The robot has reached' ...
                    ' its goal position (within 0.01m)',char(13)]};
            else
                self.L.mlog = {self.L.WARN,funcName,['The robot has missed ' ...
                    'its goal position (', num2str(dist), ' > 0.01m)',char(13)]};
            end
            
            % Print desired transform
            self.L.mlog = {self.L.DEBUG,funcName,['The desired transform ' ...
                'was: ', self.L.MatrixToString(locA_T)]};

            % Moving towards desired pose/location
            % Define a rotation of PI about X to face End Effector downwards.
            % Adding a small offset above the desired point to prevent a
            % collision.
            loc_T = loc_T*transl(0,0,0.02)*trotx(pi/2)*trotz(pi);

            % Inverse Kinematics
            newQ_2 = robot.ikcon(loc_T, newQ);

            % Getting Trajectory (Min Jerk Method)
            qpMatrix = jtraj(newQ, newQ_2, steps);

            % Moving to the desired pose (For Loop)
            for i = 1:steps
               % Plot robot moving
               robot.animate(qpMatrix(i, :));
               drawnow();
            end

            % Debugging Position
            FK = robot.fkine(qpMatrix(end, :));
            [check, dist] = self.compareTwoPositions(FK, loc_T);
            
            self.L.mlog = {self.L.DEBUG,funcName,['The robot transform at ' ...
                'the desired pose is: ',self.L.MatrixToString(FK)]};

            if check == true
                self.L.mlog = {self.L.DEBUG,funcName,['The robot has reached' ...
                    ' its goal position (within 0.01m)',char(13)]};
            else
                self.L.mlog = {self.L.WARN,funcName,['The robot has missed ' ...
                    'its goal position (', num2str(dist), ' > 0.01m)',char(13)]};
            end
            
            % Print desired transform
            self.L.mlog = {self.L.DEBUG,funcName,['The desired transform ' ...
                'was: ', self.L.MatrixToString(loc_T)]};
            
            % Return the ending pose which could be used as a guess for
            % further movement of the robot from this position.
            qOut = qpMatrix(end, :);

        end
        
        function [qOut] = MoveRobotToObject2(self, robot, loc_T, qGuess, ...
                steps)
            %This function takes a SerialLink robot, a desired 4x4 Transformation ...
            %Matrix to reach, and animates a calculated minimum jerk trajectory
            %to the configuration.
            
            % TO BE ADDED: COLLISION AVOIDANCE/DETECTION
            
            % Defining and logging to log file for information
            funcName = 'MoveRobotToObject2';
            self.L.mlog = {self.L.DEBUG,funcName,['RUNNING FUNCTION: ', ...
                funcName, char(13)]};
            
            % Getting the transform above the desired end location.
            % Define a rotation of PI/2 about X to face End Effector
            % sidewards.
            loc_T = loc_T*trotx(pi/2)*trotz(pi);

            % Moving to Location
            % Getting required Joint Angles to reach position above using ...
            % Inverse Kinematics
            newQ = robot.ikcon(loc_T, qGuess);

            % Plannning a Trajectory using the Min. Jerk Method
            % (Quintic Polynomial Method)
            % jtraj(Starting Joint Angles, Target Joint Angles, Steps to Take)
            qpMatrix = jtraj(robot.getpos(), newQ, steps);

            % Moving to the spot above the desired location
            for i = 1:steps
               % Plot Robot Moving
               robot.animate(qpMatrix(i, :));
               drawnow();
            end

            % Debugging pose above desired position
            FK = robot.fkine(qpMatrix(end, :));
            [check, dist] = self.compareTwoPositions(FK, loc_T);
            
            self.L.mlog = {self.L.DEBUG,funcName,['The robot transform ' ...
                'above input location is: ',self.L.MatrixToString(FK)]};
            
            if check == true
                self.L.mlog = {self.L.DEBUG,funcName,['The robot has reached' ...
                    ' its goal position (within 0.01m)',char(13)]};
            else
                self.L.mlog = {self.L.WARN,funcName,['The robot has missed ' ...
                    'its goal position (', num2str(dist), ' > 0.01m)',char(13)]};
            end
            
            % Print desired transform
            self.L.mlog = {self.L.DEBUG,funcName,['The desired transform ' ...
                'was: ', self.L.MatrixToString(loc_T)]};
            
            % Return the ending pose which could be used as a guess for
            % further movement of the robot from this position.
            qOut = qpMatrix(end, :);

        end
        
        function [qOut] = MoveRobotWithObject(self, robot, end_T, dAbove, ...
                objMesh_h, objVertices, qGuess, steps)
            %This function takes a SerialLink robot, a desired 4x4 Transformation ...
            %Matrix to reach, and an object mesh and animates a calculated ...
            %minimum jerk trajectory to the desired pose using waypoints.
            
            % TO BE ADDED: COLLISION AVOIDANCE/DETECTION
            
            % Defining and logging to log file for information
            funcName = 'MoveRobotWithObject';
            self.L.mlog = {self.L.DEBUG,funcName,['RUNNING FUNCTION: ', ...
                funcName, char(13)]};
            
            % Getting the transform above the current location.
            % Define a rotation of PI about X to face End Effector downwards.
            loc_T = robot.fkine(robot.getpos());
            loc_T = loc_T*transl(0,0,dAbove);
            
            % Getting required Joint Angles to reach position using ...
            % Inverse Kinematics
            newQ = robot.ikcon(loc_T, robot.getpos());
            
            % Plannning a Trajectory using the Min. Jerk Method
            % (Quintic Polynomial Method)
            % jtraj(Starting Joint Angles, Target Joint Angles, Steps to Take)
            qpMatrix = jtraj(robot.getpos(), newQ, steps);
            
            % Moving to the spot above the object
            for i = 1:steps
               % Plot robot moving
               robot.animate(qpMatrix(i, :));
               % Get Robot Pose with Forward Kinematics
               robot_TR = robot.fkine(qpMatrix(i, :));
               % Transform Object to this Pose
               objTransformVertices = [objVertices,ones(size(objVertices,1),1)] ...
                   * robot_TR';
               set(objMesh_h, 'Vertices', objTransformVertices(:,1:3));
               
               drawnow();
            end
            
            % Debugging Position
            FK = robot.fkine(qpMatrix(end, :));
            [check, dist] = self.compareTwoPositions(FK, loc_T);
            
            self.L.mlog = {self.L.DEBUG,funcName,['The robot transform at ' ...
                'the pose above the object is: ',self.L.MatrixToString(FK)]};
            if check == true
                self.L.mlog = {self.L.DEBUG,funcName,['The robot has reached' ...
                    ' its goal position (within 0.01m)',char(13)]};
            else
                self.L.mlog = {self.L.WARN,funcName,['The robot has missed ' ...
                    'its goal position (', num2str(dist), ' > 0.01m)',char(13)]};
            end
            
            % Moving towards the goal/target pose
            finQ = robot.ikcon(endT, qGuess);
            % Getting Trajectory (Min Jerk Method)
            qpMatrix = jtraj(newQ, finQ, steps);
            
            % Moving the robot and object to the end/goal pose.
            for i = 1:steps
               % Plot robot moving
               robot.animate(qpMatrix(i, :));
               % Get Robot Pose with Forward Kinematics
               robot_TR = robot.fkine(qpMatrix(i, :));
               % Transform Object to this Pose
               objTransformVertices = [objVertices,ones(size(objVertices,1),1)] ...
                   * robot_TR';
               set(objMesh_h, 'Vertices', objTransformVertices(:,1:3));
               
               drawnow();
            end
            
            % Debugging Position
            FK = robot.fkine(qpMatrix(end, :));
            [check, dist] = self.compareTwoPositions(FK, end_T);
            
            self.L.mlog = {self.L.DEBUG,funcName,['The robot transform at ' ...
                'the goal pose is: ',self.L.MatrixToString(FK)]};
            if check == true
                self.L.mlog = {self.L.DEBUG,funcName,['The robot has reached' ...
                    ' its goal position (within 0.01m)',char(13)]};
            else
                self.L.mlog = {self.L.WARN,funcName,['The robot has missed ' ...
                    'its goal position (', num2str(dist), ' > 0.01m)',char(13)]};
            end
            
            % Return the ending pose which could be used as a guess for
            % further movement of the robot from this position.
            qOut = qpMatrix(end, :);
                     
        end
        
        function [qOut] = RMRC_7DOF(self, robot, start_T, end_T, time, ...
                plotTrail, plotData)
            % This function is modified from the exercise completed in the
            % Lab 9 Tutorial Questions for a 7DOF Robot (Hans Cute).
            
            % If no 'plotTrail' value input, set 'plotTrail' to 0 (do not ...
            % show trajectory trail).
            if nargin < 5
                plotTrail = 0;
            end
            
            % If no 'plotData' value input, set 'plotData' to 0 (do not ...
            % show plot data).
            if nargin < 6
                plotData = 0;
            end
            
            % Defining and logging to log file for information
            funcName = 'RMRC_7DOF';
            self.L.mlog = {self.L.DEBUG,funcName,['RUNNING FUNCTION: ', ...
                funcName, char(13)]};
            
            % Setting Parameters
            deltaT = 0.02;          % Control Freq. (Around 50Hz)
            steps = time/deltaT;    % No. of Steps
            delta = 2*pi/steps;     % Small angle change
            epsilon = 0.0015;         % Threshold value for manipulability/Damped Least Squares
            lambda_max = 0.05;      % Set Lambda_Max when attenuating the Damping Factor for DLS Method
            W = diag([1 1 1 0.1 0.1 0.1]);     % Weighting matrix for the velocity vector
            
            % Pre-allocating memory for required data/arrays
            MoM = zeros(steps,1);           % Array for Measure of Manipulability
            qMatrix = zeros(steps,7);       % Array for joint angles
            qDot = zeros(steps,7);          % Array for joint velocities
            theta = zeros(3,steps);         % Array for roll-pitch-yaw angles
            X = zeros(3,steps);             % Array for x-y-z trajectory
            
            positionError = zeros(3,steps); % For plotting trajectory error
            angleError = zeros(3,steps);    % For plotting trajectory error
            
            % Get X-Y-Z of Start and End Poses
            startXYZ = start_T(1:3, 4);
            finXYZ = end_T(1:3, 4);

            % Get Roll, Pitch, Yaw from Rotation Matrix of 4x4 TR Matrix
            RPY = tr2rpy(start_T);
            
            % Create straight line trajectory in X-Y-Z plane at current RPY
            s = lspb(0,1,steps);            % Trapezoidal Trajectory Scalar
            for i = 1:steps
                % XYZ Trajectory
                X(:,i) = startXYZ*(1-s(i)) + s(i)*finXYZ;
                % RPY Trajectory (constant)
                theta(1, i) = RPY(1);
                theta(2, i) = RPY(2);
                theta(3, i) = RPY(3);
            end      
            
            % Set the first state in qMatrix (the current joint state)
            qMatrix(1, :) = robot.getpos();
            
            % Perform RMRC
            for i = 1:steps-1
                T = robot.fkine(qMatrix(i,:));      % FK for current joint state
                deltaX = X(:,i+1) - T(1:3, 4);     % Get position error from next waypoint
                % Get next and current RPY angles as a 3x3 Rotation Matrix
                R_Next = rpy2r(theta(1, i+1), theta(2, i+1), theta(3, i+1));
                R_Curr = T(1:3, 1:3);
                
                % Calculate rotation matrix error (from Jon RMRC lecture)
                Rdot = (1/deltaT)*(R_Next - R_Curr);    
                
                % Rdot = Skew(w)*R, also: R(t+1) = R(t) + delta_t*Rdot
                % Therefore, Rdot = (R(t+1) - R(t))/delta_t
                % Sub in Rdot = Skew(w)*R -> Skew(w)*R = (R(t+1) - R(t))/delta_t
                % To solve for Skew(w), we can use the fact that R->SO(3), and R*R'=I
                % Therefore: Skew(w)*R*R' = R'*(R(t+1) - R(t))/delta_t
                % Therefore: Skew(w) = R(t)'*(R(t+1) - R(t))/delta_t
                % Knowing that Rdot = (R(t+1) - R(t))/delta_t, and R(t) = R_Curr:
                S = Rdot*R_Curr';
                % OR we can also define the Skew Symmetric Matrix as:
                S_M = (1/deltaT)*(R_Next*R_Curr' - eye(3)); % By expanding and simplifying the equation in Line 276
                
                % Calculate required linear and angular velocities to get
                % to next point in the trajectory
                linVel = (1/deltaT)*deltaX;
                % From the Skew Symmetric Matrix, the Angular Velocities are:
                % Roll = S(3,2), Pitch = S(1,3), Yaw = S(2,1) -> FROM JON RMRC LECTURE
                angVel = [S(3,2);S(1,3);S(2,1)];
                
                % Convert change in rotation matrix to RPY angles
                deltaTheta = tr2rpy(R_Next*R_Curr');
                
                % Calculate end-effector velocity to reach next waypoint.
                % (Try using a weighting matrix to (de)emphasize certain dimensions)
                Xdot = W*[linVel;
                          angVel]; 
                      
                J = robot.jacob0(qMatrix(i,:));   % Get Jacobian at current joint state
                MoM(i) = sqrt(det(J*J'));        % Record Measure of Manipulability
                
                if MoM(i) < epsilon  % If manipulability is less than given threshold, use DLS Method
                    lambda = (1-(MoM(i)/epsilon))*lambda_max;   % Damping coefficient (try scaling it)    
                else
                    lambda = 0;
                end
                invJ = pinv(J'*J + lambda*eye(7))*J'; % Apply Damped Least Squares pseudoinverse
                
                qDot(i,:) = (invJ*Xdot)';    % Solve the RMRC equation
                % Check if next joint is outside allowable joint limits.
                % If TRUE -> trigger E-STOP and STOP MOTOR (qDot = 0)
                for joint = 1:7
                    if qMatrix(i, joint) + deltaT*qDot(i, joint) ...
                            < robot.qlim(joint, 1) ...
                            || qMatrix(i, joint) + deltaT*qDot(i, joint) ...
                            > robot.qlim(joint, 2)
                        qDot(i, joint) = 0;
                        disp('Motors Stopped - RMRC!');
                    end
                end
                
                % Calculate next joint state given calculated joint
                % velocities
                qMatrix(i+1, :) = qMatrix(i, :) + deltaT*qDot(i, :);  
                
                positionError(:,i) = X(:,i) - T(1:3, 4);  % For plotting position error at current timestamp
                angleError(:,i) = deltaTheta; % For plotting angle error at current timestamp
            end
            
            % Animate through calculated joint states
            trail = nan(3, steps);
            for i = 1:steps
                % Get FK
                FK = robot.fkine(qMatrix(i, :));
                trail(:, i) = FK(1:3, 4);
                robot.animate(qMatrix(i, :));
                drawnow();
                pause(0.01);
            end
            
            % Plot trail data
            if plotTrail == 1
                trailData_h = plot3(trail(1,:), trail(2,:), trail(3,:), 'c*');
                drawnow();
            end
            
            % Check we have reached the desired final pose
            % Debugging Position
            FK = robot.fkine(robot.getpos());
            [check, dist] = self.compareTwoPositions(FK, end_T);
            
            self.L.mlog = {self.L.DEBUG,funcName,['The robot transform at ' ...
                'the goal pose is: ',self.L.MatrixToString(FK)]};
            if check == true
                self.L.mlog = {self.L.DEBUG,funcName,['The robot has reached' ...
                    ' its goal position (within 0.01m)',char(13)]};
            else
                self.L.mlog = {self.L.WARN,funcName,['The robot has missed ' ...
                    'its goal position (', num2str(dist), ' > 0.01m)',char(13)]};
            end
            
            % Print desired transform
            self.L.mlog = {self.L.DEBUG,funcName,['The desired transform ' ...
                'was: ', self.L.MatrixToString(end_T)]};
            
            % Return final joint state (as output)
            qOut = qMatrix(end, :);
           
            if plotData == 1
                % Plot Results (Testing)
                for i = 1:7
                    figure(2)
                    subplot(4,2,i)
                    plot(qMatrix(:,i),'k','LineWidth',1)
                    title(['Joint ', num2str(i)])
                    ylabel('Angle (rad)')
                    refline(0,robot.qlim(i,1));
                    refline(0,robot.qlim(i,2));

                    figure(3)
                    subplot(4,2,i)
                    plot(qDot(:,i),'k','LineWidth',1)
                    title(['Joint ',num2str(i)]);
                    ylabel('Velocity (rad/s)')
                    refline(0,0)
                end

                figure(4)
                subplot(2,1,1)
                plot(positionError'*1000,'LineWidth',1)
                refline(0,0)
                xlabel('Step')
                ylabel('Position Error (mm)')
                legend('X-Axis','Y-Axis','Z-Axis')

                subplot(2,1,2)
                plot(angleError','LineWidth',1)
                refline(0,0)
                xlabel('Step')
                ylabel('Angle Error (rad)')
                legend('Roll','Pitch','Yaw')
                figure(5)
                plot(MoM,'k','LineWidth',1)
                refline(0,epsilon)
                title('Manipulability')
            end     
        end
        
        function [qOut] = RMRC_7DOF_OBJ(self, robot, start_T, end_T, ...
                objMesh_h, objVertices, time, plotTrail, plotData)
            % This function is modified from the exercise completed in the
            % Lab 9 Tutorial Questions for a 7DOF Robot (Hans Cute). It is
            % the version which includes moving around a specified object
            % attached to the manipulator End Effector.
            
            % If no 'plotTrail' value input, set 'plotTrail' to 0 (do not ...
            % show trajectory trail).
            if nargin < 7
                plotTrail = 0;
            end
            
            % If no 'plotData' value input, set 'plotData' to 0 (do not ...
            % show plot data).
            if nargin < 8
                plotData = 0;
            end
            
            % Defining and logging to log file for information
            funcName = 'RMRC_7DOF_OBJ';
            self.L.mlog = {self.L.DEBUG,funcName,['RUNNING FUNCTION: ', ...
                funcName, char(13)]};
            
            % Setting Parameters
            deltaT = 0.02;          % Control Freq. (Around 50Hz)
            steps = time/deltaT;    % No. of Steps
            delta = 2*pi/steps;     % Small angle change
            epsilon = 0.0015;         % Threshold value for manipulability/Damped Least Squares
            lambda_max = 0.05;      % Set Lambda_Max when attenuating the Damping Factor for DLS Method
            W = diag([1 1 1 0.1 0.1 0.1]);     % Weighting matrix for the velocity vector
            
            % Pre-allocating memory for required data/arrays
            MoM = zeros(steps,1);           % Array for Measure of Manipulability
            qMatrix = zeros(steps,7);       % Array for joint angles
            qDot = zeros(steps,7);          % Array for joint velocities
            theta = zeros(3,steps);         % Array for roll-pitch-yaw angles
            X = zeros(3,steps);             % Array for x-y-z trajectory
            
            positionError = zeros(3,steps); % For plotting trajectory error
            angleError = zeros(3,steps);    % For plotting trajectory error
            
            % Get X-Y-Z of Start and End Poses
            startXYZ = start_T(1:3, 4);
            finXYZ = end_T(1:3, 4);

            % Get Roll, Pitch, Yaw from Rotation Matrix of 4x4 TR Matrix
            RPY = tr2rpy(start_T);
            
            % Create straight line trajectory in X-Y-Z plane at current RPY
            s = lspb(0,1,steps);            % Trapezoidal Trajectory Scalar
            for i = 1:steps
                % XYZ Trajectory
                X(:,i) = startXYZ*(1-s(i)) + s(i)*finXYZ;
                % RPY Trajectory (constant)
                theta(1, i) = RPY(1);
                theta(2, i) = RPY(2);
                theta(3, i) = RPY(3);
            end      
            
            % Set the first state in qMatrix (the current joint state)
            qMatrix(1, :) = robot.getpos();
            
            % Perform RMRC
            for i = 1:steps-1
                T = robot.fkine(qMatrix(i,:));      % FK for current joint state
                deltaX = X(:,i+1) - T(1:3, 4);     % Get position error from next waypoint
                % Get next and current RPY angles as a 3x3 Rotation Matrix
                R_Next = rpy2r(theta(1, i+1), theta(2, i+1), theta(3, i+1));
                R_Curr = T(1:3, 1:3);
                
                % Calculate rotation matrix error (from Jon RMRC lecture)
                Rdot = (1/deltaT)*(R_Next - R_Curr);    
                
                % Rdot = Skew(w)*R, also: R(t+1) = R(t) + delta_t*Rdot
                % Therefore, Rdot = (R(t+1) - R(t))/delta_t
                % Sub in Rdot = Skew(w)*R -> Skew(w)*R = (R(t+1) - R(t))/delta_t
                % To solve for Skew(w), we can use the fact that R->SO(3), and R*R'=I
                % Therefore: Skew(w)*R*R' = R'*(R(t+1) - R(t))/delta_t
                % Therefore: Skew(w) = R(t)'*(R(t+1) - R(t))/delta_t
                % Knowing that Rdot = (R(t+1) - R(t))/delta_t, and R(t) = R_Curr:
                S = Rdot*R_Curr';
                % OR we can also define the Skew Symmetric Matrix as:
                S_M = (1/deltaT)*(R_Next*R_Curr' - eye(3)); % By expanding and simplifying the equation in Line 276
                
                % Calculate required linear and angular velocities to get
                % to next point in the trajectory
                linVel = (1/deltaT)*deltaX;
                % From the Skew Symmetric Matrix, the Angular Velocities are:
                % Roll = S(3,2), Pitch = S(1,3), Yaw = S(2,1) -> FROM JON RMRC LECTURE
                angVel = [S(3,2);S(1,3);S(2,1)];
                
                % Convert change in rotation matrix to RPY angles
                deltaTheta = tr2rpy(R_Next*R_Curr');
                
                % Calculate end-effector velocity to reach next waypoint.
                % (Try using a weighting matrix to (de)emphasize certain dimensions)
                Xdot = W*[linVel;
                          angVel]; 
                      
                J = robot.jacob0(qMatrix(i,:));   % Get Jacobian at current joint state
                MoM(i) = sqrt(det(J*J'));        % Record Measure of Manipulability
                
                if MoM(i) < epsilon  % If manipulability is less than given threshold, use DLS Method
                    lambda = (1-(MoM(i)/epsilon))*lambda_max;   % Damping coefficient (try scaling it)    
                else
                    lambda = 0;
                end
                invJ = pinv(J'*J + lambda*eye(7))*J'; % Apply Damped Least Squares pseudoinverse
                
                qDot(i,:) = (invJ*Xdot)';    % Solve the RMRC equation
                % Check if next joint is outside allowable joint limits.
                % If TRUE -> trigger E-STOP and STOP MOTOR (qDot = 0)
                for joint = 1:7
                    if qMatrix(i, joint) + deltaT*qDot(i, joint) ...
                            < robot.qlim(joint, 1) ...
                            || qMatrix(i, joint) + deltaT*qDot(i, joint) ...
                            > robot.qlim(joint, 2)
                        qDot(i, joint) = 0;
                        disp('Motors Stopped - RMRC!');
                    end
                end
                
                % Calculate next joint state given calculated joint
                % velocities
                qMatrix(i+1, :) = qMatrix(i, :) + deltaT*qDot(i, :);  
                
                positionError(:,i) = X(:,i) - T(1:3, 4);  % For plotting position error at current timestamp
                angleError(:,i) = deltaTheta; % For plotting angle error at current timestamp
            end
            
            % Animate through calculated joint states
            trail = nan(3, steps);
            for i = 1:steps
                % Get FK
                FK = robot.fkine(qMatrix(i, :));
                editedFK = FK*trotx(pi/2);
                % Save FK value for trail
                trail(:, i) = FK(1:3, 4);
                % Animate manipulator to position
                robot.animate(qMatrix(i, :));
                % Transform held object to new EE location
                objTransformVertices = [objVertices,ones(size(objVertices,1),1)] ...
                   * editedFK';
                set(objMesh_h, 'Vertices', objTransformVertices(:,1:3));
                drawnow();
                pause(0.01);
            end
            
            % Plot trail data
            if plotTrail == 1
                trailData_h = plot3(trail(1,:), trail(2,:), trail(3,:), 'c*');
                drawnow();
            end
            
            % Check we have reached the desired final pose
            % Debugging Position
            FK = robot.fkine(robot.getpos());
            [check, dist] = self.compareTwoPositions(FK, end_T);
            
            self.L.mlog = {self.L.DEBUG,funcName,['The robot transform at ' ...
                'the goal pose is: ',self.L.MatrixToString(FK)]};
            if check == true
                self.L.mlog = {self.L.DEBUG,funcName,['The robot has reached' ...
                    ' its goal position (within 0.01m)',char(13)]};
            else
                self.L.mlog = {self.L.WARN,funcName,['The robot has missed ' ...
                    'its goal position (', num2str(dist), ' > 0.01m)',char(13)]};
            end
            
            % Print desired transform
            self.L.mlog = {self.L.DEBUG,funcName,['The desired transform ' ...
                'was: ', self.L.MatrixToString(end_T)]};
            
            % Return final joint state (as output)
            qOut = qMatrix(end, :);
           
            if plotData == 1
                % Plot Results (Testing)
                for i = 1:7
                    figure(2)
                    subplot(4,2,i)
                    plot(qMatrix(:,i),'k','LineWidth',1)
                    title(['Joint ', num2str(i)])
                    ylabel('Angle (rad)')
                    refline(0,robot.qlim(i,1));
                    refline(0,robot.qlim(i,2));

                    figure(3)
                    subplot(4,2,i)
                    plot(qDot(:,i),'k','LineWidth',1)
                    title(['Joint ',num2str(i)]);
                    ylabel('Velocity (rad/s)')
                    refline(0,0)
                end

                figure(4)
                subplot(2,1,1)
                plot(positionError'*1000,'LineWidth',1)
                refline(0,0)
                xlabel('Step')
                ylabel('Position Error (mm)')
                legend('X-Axis','Y-Axis','Z-Axis')

                subplot(2,1,2)
                plot(angleError','LineWidth',1)
                refline(0,0)
                xlabel('Step')
                ylabel('Angle Error (rad)')
                legend('Roll','Pitch','Yaw')
                figure(5)
                plot(MoM,'k','LineWidth',1)
                refline(0,epsilon)
                title('Manipulability')
            end          
        end
        
        
        function [check, dist] = compareTwoPositions(~, T1, T2)
            % This function will take two 4x4 Homogenous Transform matrices 
            % and check if their positions are equal.

            x = abs(T1(1,4)-T2(1,4));
            y = abs(T1(2,4)-T2(2,4));
            z = abs(T1(3,4)-T2(3,4));
            
            dist = sqrt(x^2 + y^2 + z^2);

            if dist < 0.01
                check = true;
            else
                check = false;
            end
        end
        
    end
            
end