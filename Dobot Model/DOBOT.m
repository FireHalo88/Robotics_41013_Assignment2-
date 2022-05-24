classdef DOBOT < handle
    
    properties
        model;
        workspace = [-.4 .4 -.4 .4 -0.3 .4];   
        
    end
    
    methods
        function self = DOBOT(useGripper)

            % robot = 
            self.GetDOBOTRobot();
            % robot = 
            self.PlotAndColourRobot();%robot,workspace);
        end
        
        function GetDOBOTRobot(self)
            pause(0.001);
            name = ['DOBOT_',datestr(now,'yyyymmddTHHMMSSFFF')];

            L1 = Link('d',0.12,'a',0.0,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-90),deg2rad(90)]);
            L2 = Link('d',0.0,'a',0.145,'alpha',0,'offset',0,'qlim',[deg2rad(-90),deg2rad(10)]);
            L3 = Link('d',0.0,'a',0.16,'alpha',0,'offset',0,'qlim',[deg2rad(0),deg2rad(120)]);
            L4 = Link('d',0.0,'a',-0.04,'alpha',-pi/2,'offset',pi,'qlim',[deg2rad(-90),deg2rad(0)]);
            %L5 = Link('d',0.093,'a',0,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);

            self.model = SerialLink([L1 L2 L3 L4],'name',name);%L1
        end
        
        function PlotAndColourRobot(self)%robot,workspace)
            for linkIndex = 0:self.model.n;
                [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['DOBOT_',num2str(linkIndex),'.PLY'],'tri');
                self.model.faces{linkIndex+1} = faceData;
                self.model.points{linkIndex+1} = vertexData;
            end

            % Display robot
            self.model.plot3d(zeros(1,self.model.n),'noarrow','workspace',self.workspace);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end  
            self.model.delay = 0;

            % Try to correctly colour the arm (if colours are in ply file data)
            for linkIndex = 0:self.model.n
                handles = findobj('Tag', self.model.name);
                h = get(handles,'UserData');
                try 
                    h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                                                                  , plyData{linkIndex+1}.vertex.green ...
                                                                  , plyData{linkIndex+1}.vertex.blue]/255;
                    h.link(linkIndex+1).Children.FaceColor = 'interp';
                catch ME_1
                    disp(ME_1);
                    continue;
                end
            end
        end        
    end
end

