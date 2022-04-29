classdef HansCute < handle
    % Han's robotics cute robot class.
    % For use with Peter Corke's Robotics, Vision and Control Toolbox.
    % We inherit from handle because this allows us to modify the robots
    % state and keep those changes preserved in class methods (for example,
    % moveJ will actually change the robot's joints).
    
    properties (Constant)
        DHParams = ...  % DH Parameters describing the robot geometry
        [
        %   z       x       alpha       range       offset
            0.150   0       90.00       300.0       0
            0       0       -90.00      210.0       0
            0.125   0       90.00       300.0       0
            0       0.065   -90.00      210.0       90.00
            0       0.068   90.00       210.0       0
            0       0       90.00       210.0       90.00
            0.158   0       0.0         300.0       180.0
        ];    
        nJoints = 7;    % Number of joints in the robot;
        maxJointVel = ...       % Largest movement the robot can make in one time step
            (pi);
        q0 = ...                % Home Position 
            [0 0 0 0 0 0 0];    % (all zeroes)
        RMRCWeights = ...       % Weighting for the RMRC Solution
            diag([1 1 1 0.5 0.5 0.5]);
        linearSpeed = ...       % Linear speed during L trajectories
            0.2;                % in m/s.
        angularSpeed = ...      % Angular speed during L trajectories (tool rotation speed)
            pi/2;               % in rad/s.
        workspace = [-0.75 0.75 -0.75 0.75 0 0.75];   % Robot workspace
    end
    
    properties
        model      % SerialLink object describing the robot
        joints          % Joint Positions
        realRobotHAL    % HAL for the actual robot itself
        moveRealRobot   % Hardware State (simulation or actual)
        useRealController   % Controller State (keyboard or controller)
        moveJFrequency	% Rate at which joint moves should run
        moveLFrequency	% Rate at which tool moves should run
    end
    
    methods
        function plotModel(self)   %robot,workspace
            for linkIndex = 0:self.model.n
                [ faceData, vertexData, plyData{linkIndex + 1} ] =  ...
                    plyread(['HansCuteToShare/HansLink',num2str(linkIndex),'.ply'],'tri');
                
                self.model.faces{linkIndex + 1} = faceData;
                self.model.points{linkIndex + 1} = vertexData;
            end

            % Display robot
            self.model.plot3d(zeros(1,self.model.n),...
                'noarrow','workspace',self.workspace);
            hold all
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end  
            self.model.delay = 0;

            % Try to correctly colour the arm (if colours are in ply file data)
            for linkIndex = 0:self.model.n
                handles = findobj('Tag', self.model.name);
                h = get(handles,'UserData');
                try 
                    h.link(linkIndex+1).Children.FaceVertexCData = ...
                        [plyData{linkIndex+1}.vertex.red ...
                         plyData{linkIndex+1}.vertex.green ...
                         plyData{linkIndex+1}.vertex.blue]/255;
                    h.link(linkIndex+1).Children.FaceColor = 'interp';
                catch ME_1
                    disp(ME_1);
                    continue;
                end
            end
        end
        
        function set.joints(obj, joints)
            % Validate joints on assignment
            obj.validateJoints(joints);
            obj.joints = joints;
        end
        
        function valid = validateJoints(obj, joints, crash)
            % Validation that the new joint position is within joint limits
            % If no joints are provided the robot's current joints are
            % used. If crash is set to false, the validity of the joints
            % will be returned instead.
            if nargin < 2
                joints = obj.joints;
            end
            if nargin < 3
                crash = true;
            end
            if norm(double(abs(joints) > deg2rad(obj.DHParams(:,4))'/2))
                if crash
                    disp('Joint limit overshoot')
                    rad2deg(abs(joints) - deg2rad(obj.DHParams(:,4)'/2))
                    error('Joints exceed joint limits!');
                else
                    valid = false;
                    return
                end
            else
                valid = true;
                return
            end
        end
        
        function obj = HansCute(name)
            % Creates a new Cyton 300 Robot Object. Default pose is 0
            if nargin < 1
                name = 'Hans Cute Robot';
            end
            links = Link.empty(obj.nJoints, 0);
            for i = 1:size(obj.DHParams,1)
                links(i) = Link('d', obj.DHParams(i,1), ...
                    'a', obj.DHParams(i,2), 'alpha', deg2rad(obj.DHParams(i,3)), ...
                    'qlim', deg2rad([-obj.DHParams(i,4)/2, obj.DHParams(i,4)/2]), ...
                    'offset', deg2rad(obj.DHParams(i,5)));
            end           
            obj.model = SerialLink(links, 'name', name);
            %obj.plotModel;
            obj.joints = zeros(1, obj.nJoints);
            obj.moveRealRobot = false;
            obj.moveJFrequency = 15;
            obj.moveLFrequency = 15;
        end
               
        function teach(obj)
            % Opens a figure with the robot for teaching new poses
            obj.model.teach(obj.joints);
        end
        
        function transform = getEndEffectorTransform(obj, joints)
            % Uses forward kinematics to get the transform of the end
            % effector.
            if nargin < 2
                joints = obj.joints;
            end
            transform = obj.model.fkine(joints);
        end
        
        function position = getEndEffectorPosition(obj, joints)
            % Uses forward kinematics to get the position of the end
            % effector.
            if nargin < 2
                joints = obj.joints;
            end
            transform = obj.model.fkine(joints);
            position = transform(1:3,4);
        end
        
        function jacobian = getJacobian(obj, joints)
            % Gets the jacobian for a given robot joint position. 
            % If no position is supplied, the current pose is used.
            if nargin < 2
                joints = obj.joints;
            end
            jacobian = obj.model.jacob0(joints);
        end
        
        function plot(obj)
            % Plots the robot
            obj.model.plot(obj.joints);
        end
        
        function animate(obj)
            % Updates the plot of a robot
            obj.model.animate(obj.joints);
            pause(0.1)
        end
        
        function moveJTraj(obj, trajectory)
            % Moves the robot through a set of joint positions
            % The rate limiter means our robot will run at a realistic
            % speed
            rateLimiter = rateControl(obj.moveJFrequency);
            rateLimiter.OverrunAction = 'slip';
            obj.joints = trajectory(1,:);
            obj.plotModel();
            rateLimiter.reset();
            for i = 2:size(trajectory,1)
                obj.joints = trajectory(i,:);
                obj.animate();
                rateLimiter.waitfor();
            end
        end
                                     
    end    
end