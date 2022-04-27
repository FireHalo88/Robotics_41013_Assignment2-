classdef Dobot < handle
    %HANSCUTE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        model;
        workspace = [0 4 0 4 0 4]
    end
    
    methods
        function self = Dobot()
            self.GetDobotRobot();
        end
        
        function GetDobotRobot(self)

        pause(0.001);
        name = ['Dobot'];

        %DH params from:
        %

        L1 = Link('d',0.08,     'a',0,      'alpha',pi/2,   'qlim',deg2rad([-135 135]),     'offset',0);
        L2 = Link('d',0,        'a',0.135,  'alpha',0,      'qlim',deg2rad([0 85]),         'offset',0); 
        L3 = Link('d',0,        'a',0.16,   'alpha',0,      'qlim',deg2rad([-10 95]),       'offset',0);
        %L4 for end effector Yaw control via servo? Unsure of actual
        %limits/specs but can leave as placeholder
        L4 = Link('d',0,        'a',0,      'alpha',0,      'qlim',deg2rad([-90 90]),       'offset',0);
    
        self.model = SerialLink([L1 L2 L3],'name',name);
        end
        
%         function outputArg = method1(obj,inputArg)
%             outputArg = obj.Property1 + inputArg;
%         end
    end
end

