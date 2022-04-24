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
        
        function [qOut] = MoveRobotToObject(self, robot, loc_T, qGuess, steps)
            %This function takes a SerialLink robot, a desired 4x4 Transformation ...
            %Matrix to reach, and animates a calculated minimum jerk trajectory
            %to the configuration using waypoints.
            
            % TO BE ADDED: COLLISION AVOIDANCE/DETECTION
            
            % Defining and logging to log file for information
            funcName = 'MoveRobotToObject';
            self.L.mlog = {self.L.DEBUG,funcName,['RUNNING FUNCTION: ', ...
                funcName, char(13)]};
            
            % Getting the transform 25cm above the desired end location.
            % Define a rotation of PI about X to face End Effector downwards.
            locA_T = loc_T*transl(0,0,0.25)*trotx(pi);

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
            self.L.mlog = {self.L.DEBUG,funcName,['The robot transform ' ...
                'above input location is: ',self.L.MatrixToString(FK)]};
            
            if self.compareTwoPositions(FK, locA_T) == true
                self.L.mlog = {self.L.DEBUG,funcName,['The robot has reached' ...
                    ' its goal position (within 0.01m)',char(13)]};
            else
                self.L.mlog = {self.L.WARN,funcName,['The robot has missed ' ...
                    'its goal position (>0.01m)',char(13)]};
            end

            % Moving towards desired pose/location
            % Define a rotation of PI about X to face End Effector downwards.
            % Adding a small offset above the desired point to prevent a
            % collision.
            loc_T = loc_T*transl(0,0,0.05)*trotx(pi);

            % Inverse Kinematics
            newQ_2 = robot.ikcon(loc_T, newQ);

            % Getting Trajectory (Min Jerk Method)
            qpMatrix = jtraj(newQ, newQ_2, 10);

            % Moving to the desired pose (For Loop)
            for i = 1:10
               % Plot robot moving
               robot.animate(qpMatrix(i, :));
               drawnow();
            end

            % Debugging Position
            FK = robot.fkine(qpMatrix(end, :));
            self.L.mlog = {self.L.DEBUG,funcName,['The robot transform at ' ...
                'the desired pose is: ',self.L.MatrixToString(FK)]};

            if self.compareTwoPositions(FK, loc_T) == true
                self.L.mlog = {self.L.DEBUG,funcName,['The robot has reached' ...
                    ' its goal position (within 0.01m)',char(13)]};
            else
                self.L.mlog = {self.L.WARN,funcName,['The robot has missed ' ...
                    'its goal position (>0.01m)',char(13)]};
            end
            
            % Return the ending pose which could be used as a guess for
            % further movement of the robot from this position.
            qOut = qpMatrix(end, :);

        end
        
        function [qOut] = MoveRobotWithObject(self, robot, end_T, objMesh_h, ...
                objVertices, qGuess, steps)
            %This function takes a SerialLink robot, a desired 4x4 Transformation ...
            %Matrix to reach, and an object mesh and animates a calculated ...
            %minimum jerk trajectory to the desired pose using waypoints.
            
            % TO BE ADDED: COLLISION AVOIDANCE/DETECTION
            
            % Defining and logging to log file for information
            funcName = 'MoveRobotWithObject';
            self.L.mlog = {self.L.DEBUG,funcName,['RUNNING FUNCTION: ', ...
                funcName, char(13)]};
            
            % Getting the transform 25cm above the current location.
            % Define a rotation of PI about X to face End Effector downwards.
            loc_T = robot.fkine(robot.getpos());
            loc_T = loc_T*transl(0,0,0.25);
            
            % Getting required Joint Angles to reach position using ...
            % Inverse Kinematics
            newQ = robot.ikcon(loc_T, robot.getpos());
            
            % Plannning a Trajectory using the Min. Jerk Method
            % (Quintic Polynomial Method)
            % jtraj(Starting Joint Angles, Target Joint Angles, Steps to Take)
            qpMatrix = jtraj(robot.getpos(), newQ, 10);
            
            % Moving to the spot above the object
            for i = 1:10
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
            self.L.mlog = {self.L.DEBUG,funcName,['The robot transform at ' ...
                'the pose above the object is: ',self.L.MatrixToString(FK)]};

            if self.compareTwoPositions(FK, loc_T) == true
                self.L.mlog = {self.L.DEBUG,funcName,['The robot has reached' ...
                    ' its goal position (within 0.01m)',char(13)]};
            else
                self.L.mlog = {self.L.WARN,funcName,['The robot has missed ' ...
                    'its goal position (>0.01m)',char(13)]};
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
            self.L.mlog = {self.L.DEBUG,funcName,['The robot transform at ' ...
                'the goal pose is: ',self.L.MatrixToString(FK)]};

            if self.compareTwoPositions(FK, end_T) == true
                self.L.mlog = {self.L.DEBUG,funcName,['The robot has reached' ...
                    ' its goal position (within 0.01m)',char(13)]};
            else
                self.L.mlog = {self.L.WARN,funcName,['The robot has missed ' ...
                    'its goal position (>0.01m)',char(13)]};
            end
            
            % Return the ending pose which could be used as a guess for
            % further movement of the robot from this position.
            qOut = qpMatrix(end, :);
                     
        end
        
        function [check] = compareTwoPositions(~, T1, T2)
            % This function will take two 4x4 Homogenous Transform matrices 
            % and check if their positions are equal.

            x = abs(T1(1,4)-T2(1,4));
            y = abs(T1(2,4)-T2(2,4));
            z = abs(T1(3,4)-T2(3,4));

            if [x,y,z] < 0.01
                check = true;
            else
                check = false;
            end
        end
        
    end
            
end