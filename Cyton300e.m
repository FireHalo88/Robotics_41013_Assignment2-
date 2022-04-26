classdef Cyton300e < handle
    %HANSCUTE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        model;
        workspace = [0 4 0 4 0 4]
    end
    
    methods
        function self = Cyton300e()
            self.GetCytonRobot();
        end
        
        function GetCytonRobot(self)

        pause(0.001);
        name = ['CytonATV'];

        %DH params from:
        %https://link.springer.com/content/pdf/10.1007/s00500-017-2975-y.pdf
        %based on Cyton Gamma 300 model. Seem to be same as Epsilon 300.

        L1 = Link('d',0.12,     'a',0,      'alpha',pi/2,   'qlim',deg2rad([0 300]), 'offset',30);
        L2 = Link('d',0,        'a',0,      'alpha',pi/2,   'qlim',deg2rad([0 210]), 'offset',70); 
        L3 = Link('d',0.1408,   'a',0,      'alpha',-pi/2,  'qlim',deg2rad([0 300]), 'offset',30);
        L4 = Link('d',0,        'a',0.718,  'alpha',-pi/2,  'qlim',deg2rad([0 210]), 'offset',70);
        L5 = Link('d',0,        'a',0.718,  'alpha',pi/2,   'qlim',deg2rad([0,210]), 'offset',70);
        L6 = Link('d',0,        'a',0,      'alpha',-pi/2,  'qlim',deg2rad([0,210]), 'offset',75);
        L7 = Link('d',0.1296,   'a',0,      'alpha',0,      'qlim',deg2rad([0,300]), 'offset',30);
    
        self.model = SerialLink([L1 L2 L3 L4 L5 L6 L7],'name',name);
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
    end
end

