function varargout = TracerGUI(varargin)
% TRACERGUI MATLAB code for TracerGUI.fig
%      TRACERGUI, by itself, creates a new TRACERGUI or raises the existing
%      singleton*.
%
%      H = TRACERGUI returns the handle to a new TRACERGUI or the handle to
%      the existing singleton*.
%
%      TRACERGUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in TRACERGUI.M with the given input arguments.
%
%      TRACERGUI('Property','Value',...) creates a new TRACERGUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before TracerGUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to TracerGUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help TracerGUI

% Last Modified by GUIDE v2.5 30-Apr-2022 01:08:22

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @TracerGUI_OpeningFcn, ...
                   'gui_OutputFcn',  @TracerGUI_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT

% --- Executes just before TracerGUI is made visible.
function TracerGUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to TracerGUI (see VARARGIN)

% Choose default command line output for TracerGUI
handles.output = hObject;

% Creation of the Cyton/Hans Cute Robot Class
workspace = [-0.75 0.75 -0.75 0.75 0 0.75];     % Workspace Definition
hansCute = HansCute('ATV', workspace);
myRobot = hansCute.model;

% Set initial angles
handles.q1.String = '0';
handles.q2.String = '0';
handles.q3.String = '0';
handles.q4.String = '0';
handles.q5.String = '0';
handles.q6.String = '0';
handles.q7.String = '0';

% Updating Handles
handles.myRobot = myRobot;

% Defining 4x4 Transforms for Environment Objects
table = transl(0.0, 0.0, 0.0); 
safetyBarrierPoint1 = transl(-0.47, -0.45, 0.0);
safetyBarrierPoint2 = transl(-0.47, 0.5, 0.0);
safetyBarrierPoint3 = transl(0.35, -0.45, 0.0);
safetyBarrierPoint4 = transl(0.35, 0.5, 0.0);
guard = transl(0.6, 0.4, 0)*trotz(pi/2);
fireExtinguisher = transl(0.3, 0.15, 0.7);

% Special Objects needing Handles
pen1 = transl(0.02, -0.3, 0.273);
pen2 = transl(0.07, -0.3, 0.273);
pen3 = transl(0.12, -0.3, 0.273);
pen4 = transl(0.17, -0.3, 0.273);
canvas = transl(-0.3, -0.2, 0.22);
handles.blackPen = pen1;
handles.redPen = pen2;
handles.greenPen = pen3;
handles.bluePen = pen4;
handles.canvas = canvas;

% Plot the Robot at default state
axes(handles.axes2)
hold on

q = zeros(1,7);
hansCute.model.base = transl(0.2, 0, 0.22);
hansCute.plotModel();

% Plot Environment
% Create Instance of Environment Class
environ = createEnvironment(workspace);
environ.placeObjectsBetter(canvas, table, pen1, pen2, pen3, pen4, ...
    safetyBarrierPoint1, safetyBarrierPoint2, safetyBarrierPoint3, ...
    safetyBarrierPoint4, guard, fireExtinguisher)

view(3);

% Calculate the Robot EE Position with FK
myRobot_TR = myRobot.fkine(q);
handles.ee_X.String = num2str(round(myRobot_TR(1,4), 3));
handles.ee_Y.String = num2str(round(myRobot_TR(2,4), 3));
handles.ee_Z.String = num2str(round(myRobot_TR(3,4), 3));
% Extract Rotation Portion of FK Matrix
rot = myRobot_TR(1:3, 1:3);
% Convert Rotation Matrix to Roll, Pitch, Yaw Values in Degrees
RPY = tr2rpy(rot, 'deg');
handles.ee_Roll.String = num2str(round(RPY(1), 3));
handles.ee_Pitch.String = num2str(round(RPY(2), 3));
handles.ee_Yaw.String = num2str(round(RPY(3), 3));

% Setting a cartesian increment
cartInc = 0.01;
% Adding handles
handles.cartInc = cartInc;
handles.cartKeepCurrentRPY = 1;
handles.colour = 'blackPen';
handles.drawing = [];
handles.drawingPath = '';
% Handle for Robot Movement Class
rMove = RobotMovement;
handles.rMove = rMove;

% Update handles structure
guidata(hObject, handles);

% This sets up the initial plot - only do when we are invisible
% so window can get raised using TracerGUI.
if strcmp(get(hObject,'Visible'),'off')
    plot(rand(5));
end

% UIWAIT makes TracerGUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = TracerGUI_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
axes(handles.axes1);
cla;

popup_sel_index = get(handles.popupmenu1, 'Value');
switch popup_sel_index
    case 1
        plot(rand(5));
    case 2
        plot(sin(1:0.01:25.99));
    case 3
        bar(1:.5:10);
    case 4
        plot(membrane);
    case 5
        surf(peaks);
end


% --------------------------------------------------------------------
function FileMenu_Callback(hObject, eventdata, handles)
% hObject    handle to FileMenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function OpenMenuItem_Callback(hObject, eventdata, handles)
% hObject    handle to OpenMenuItem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
file = uigetfile('*.fig');
if ~isequal(file, 0)
    open(file);
end

% --------------------------------------------------------------------
function PrintMenuItem_Callback(hObject, eventdata, handles)
% hObject    handle to PrintMenuItem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
printdlg(handles.figure1)

% --------------------------------------------------------------------
function CloseMenuItem_Callback(hObject, eventdata, handles)
% hObject    handle to CloseMenuItem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
selection = questdlg(['Close ' get(handles.figure1,'Name') '?'],...
                     ['Close ' get(handles.figure1,'Name') '...'],...
                     'Yes','No','Yes');
if strcmp(selection,'No')
    return;
end

delete(handles.figure1)


% --- Executes on selection change in popupmenu1.
function popupmenu1_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = get(hObject,'String') returns popupmenu1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu1


% --- Executes during object creation, after setting all properties.
function popupmenu1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
     set(hObject,'BackgroundColor','white');
end

set(hObject, 'String', {'plot(rand(5))', 'plot(sin(1:0.01:25))', 'bar(1:.5:10)', 'plot(membrane)', 'surf(peaks)'});


% --- Executes during object creation, after setting all properties.
function axes1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes1


% --- Executes on slider movement.
function slider1_Callback(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
% Get current slider value
newQ1 = round(get(hObject, 'Value'), 1);
% Set RHS text box to be this value
handles.q1.String = num2str(newQ1);
% Update X,Y,Z and R,P,Y values and animate robot to match new joint state.
allQ = handles.myRobot.getpos();
allQ(1) = newQ1*pi/180;
handles.myRobot.animate(allQ);
% Now we want to update the EE Position on the GUI
% Calculate the Robot EE Position with FK
myRobot_TR = handles.myRobot.fkine(allQ);
handles.ee_X.String = num2str(round(myRobot_TR(1,4), 3));
handles.ee_Y.String = num2str(round(myRobot_TR(2,4), 3));
handles.ee_Z.String = num2str(round(myRobot_TR(3,4), 3));
% Extract Rotation Portion of FK Matrix
rot = myRobot_TR(1:3, 1:3);
% Convert Rotation Matrix to Roll, Pitch, Yaw Values in Degrees
RPY = tr2rpy(rot, 'deg');
handles.ee_Roll.String = num2str(round(RPY(1), 3));
handles.ee_Pitch.String = num2str(round(RPY(2), 3));
handles.ee_Yaw.String = num2str(round(RPY(3), 3));


% --- Executes during object creation, after setting all properties.
function slider1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Define Min, Max and starting Angle for Q1 Slider
set(hObject,'Min',-150,'Max',150);
set(hObject, 'Value', 0);
% Set step sizes of 5 degrees when pressing slider buttons
step = [5, 5]/(get(hObject, 'Max') - get(hObject, 'Min'));
set(hObject, 'SliderStep', step);

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider2_Callback(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
% Get current slider value
newQ2 = round(get(hObject, 'Value'), 1);
% Set RHS text box to be this value
handles.q2.String = num2str(newQ2);
% Update X,Y,Z and R,P,Y values and animate robot to match new joint state.
allQ = handles.myRobot.getpos();
allQ(2) = newQ2*pi/180;
handles.myRobot.animate(allQ);
% Now we want to update the EE Position on the GUI
% Calculate the Robot EE Position with FK
myRobot_TR = handles.myRobot.fkine(allQ);
handles.ee_X.String = num2str(round(myRobot_TR(1,4), 3));
handles.ee_Y.String = num2str(round(myRobot_TR(2,4), 3));
handles.ee_Z.String = num2str(round(myRobot_TR(3,4), 3));
% Extract Rotation Portion of FK Matrix
rot = myRobot_TR(1:3, 1:3);
% Convert Rotation Matrix to Roll, Pitch, Yaw Values in Degrees
RPY = tr2rpy(rot, 'deg');
handles.ee_Roll.String = num2str(round(RPY(1), 3));
handles.ee_Pitch.String = num2str(round(RPY(2), 3));
handles.ee_Yaw.String = num2str(round(RPY(3), 3));


% --- Executes during object creation, after setting all properties.
function slider2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Define Min, Max and starting Angle for Q2 Slider
set(hObject,'Min',-105,'Max',105);
set(hObject, 'Value', 0);
% Set step sizes of 5 degrees when pressing slider buttons
step = [5, 5]/(get(hObject, 'Max') - get(hObject, 'Min'));
set(hObject, 'SliderStep', step);

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider3_Callback(hObject, eventdata, handles)
% hObject    handle to slider3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
% Get current slider value
newQ3 = round(get(hObject, 'Value'), 1);
% Set RHS text box to be this value
handles.q3.String = num2str(newQ3);
% Update X,Y,Z and R,P,Y values and animate robot to match new joint state.
allQ = handles.myRobot.getpos();
allQ(3) = newQ3*pi/180;
handles.myRobot.animate(allQ);
% Now we want to update the EE Position on the GUI
% Calculate the Robot EE Position with FK
myRobot_TR = handles.myRobot.fkine(allQ);
handles.ee_X.String = num2str(round(myRobot_TR(1,4), 3));
handles.ee_Y.String = num2str(round(myRobot_TR(2,4), 3));
handles.ee_Z.String = num2str(round(myRobot_TR(3,4), 3));
% Extract Rotation Portion of FK Matrix
rot = myRobot_TR(1:3, 1:3);
% Convert Rotation Matrix to Roll, Pitch, Yaw Values in Degrees
RPY = tr2rpy(rot, 'deg');
handles.ee_Roll.String = num2str(round(RPY(1), 3));
handles.ee_Pitch.String = num2str(round(RPY(2), 3));
handles.ee_Yaw.String = num2str(round(RPY(3), 3));

% --- Executes during object creation, after setting all properties.
function slider3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Define Min, Max and starting Angle for Q3 Slider
set(hObject,'Min',-150,'Max',150);
set(hObject, 'Value', 0);
% Set step sizes of 5 degrees when pressing slider buttons
step = [5, 5]/(get(hObject, 'Max') - get(hObject, 'Min'));
set(hObject, 'SliderStep', step);

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider4_Callback(hObject, eventdata, handles)
% hObject    handle to slider4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
% Get current slider value
newQ4 = round(get(hObject, 'Value'), 1);
% Set RHS text box to be this value
handles.q4.String = num2str(newQ4);
% Update X,Y,Z and R,P,Y values and animate robot to match new joint state.
allQ = handles.myRobot.getpos();
allQ(4) = newQ4*pi/180;
handles.myRobot.animate(allQ);
% Now we want to update the EE Position on the GUI
% Calculate the Robot EE Position with FK
myRobot_TR = handles.myRobot.fkine(allQ);
handles.ee_X.String = num2str(round(myRobot_TR(1,4), 3));
handles.ee_Y.String = num2str(round(myRobot_TR(2,4), 3));
handles.ee_Z.String = num2str(round(myRobot_TR(3,4), 3));
% Extract Rotation Portion of FK Matrix
rot = myRobot_TR(1:3, 1:3);
% Convert Rotation Matrix to Roll, Pitch, Yaw Values in Degrees
RPY = tr2rpy(rot, 'deg');
handles.ee_Roll.String = num2str(round(RPY(1), 3));
handles.ee_Pitch.String = num2str(round(RPY(2), 3));
handles.ee_Yaw.String = num2str(round(RPY(3), 3));

% --- Executes during object creation, after setting all properties.
function slider4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Define Min, Max and starting Angle for Q4 Slider
set(hObject,'Min',-105,'Max',105);
set(hObject, 'Value', 0);
% Set step sizes of 5 degrees when pressing slider buttons
step = [5, 5]/(get(hObject, 'Max') - get(hObject, 'Min'));
set(hObject, 'SliderStep', step);


% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider5_Callback(hObject, eventdata, handles)
% hObject    handle to slider5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
% Get current slider value
newQ5 = round(get(hObject, 'Value'), 1);
% Set RHS text box to be this value
handles.q5.String = num2str(newQ5);
% Update X,Y,Z and R,P,Y values and animate robot to match new joint state.
allQ = handles.myRobot.getpos();
allQ(5) = newQ5*pi/180;
handles.myRobot.animate(allQ);
% Now we want to update the EE Position on the GUI
% Calculate the Robot EE Position with FK
myRobot_TR = handles.myRobot.fkine(allQ);
handles.ee_X.String = num2str(round(myRobot_TR(1,4), 3));
handles.ee_Y.String = num2str(round(myRobot_TR(2,4), 3));
handles.ee_Z.String = num2str(round(myRobot_TR(3,4), 3));
% Extract Rotation Portion of FK Matrix
rot = myRobot_TR(1:3, 1:3);
% Convert Rotation Matrix to Roll, Pitch, Yaw Values in Degrees
RPY = tr2rpy(rot, 'deg');
handles.ee_Roll.String = num2str(round(RPY(1), 3));
handles.ee_Pitch.String = num2str(round(RPY(2), 3));
handles.ee_Yaw.String = num2str(round(RPY(3), 3));

% --- Executes during object creation, after setting all properties.
function slider5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Define Min, Max and starting Angle for Q5 Slider
set(hObject,'Min',-105,'Max',105);
set(hObject, 'Value', 0);
% Set step sizes of 5 degrees when pressing slider buttons
step = [5, 5]/(get(hObject, 'Max') - get(hObject, 'Min'));
set(hObject, 'SliderStep', step);


% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider6_Callback(hObject, eventdata, handles)
% hObject    handle to slider6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
% Get current slider value
newQ6 = round(get(hObject, 'Value'), 1);
% Set RHS text box to be this value
handles.q6.String = num2str(newQ6);
% Update X,Y,Z and R,P,Y values and animate robot to match new joint state.
allQ = handles.myRobot.getpos();
allQ(6) = newQ6*pi/180;
handles.myRobot.animate(allQ);
% Now we want to update the EE Position on the GUI
% Calculate the Robot EE Position with FK
myRobot_TR = handles.myRobot.fkine(allQ);
handles.ee_X.String = num2str(round(myRobot_TR(1,4), 3));
handles.ee_Y.String = num2str(round(myRobot_TR(2,4), 3));
handles.ee_Z.String = num2str(round(myRobot_TR(3,4), 3));
% Extract Rotation Portion of FK Matrix
rot = myRobot_TR(1:3, 1:3);
% Convert Rotation Matrix to Roll, Pitch, Yaw Values in Degrees
RPY = tr2rpy(rot, 'deg');
handles.ee_Roll.String = num2str(round(RPY(1), 3));
handles.ee_Pitch.String = num2str(round(RPY(2), 3));
handles.ee_Yaw.String = num2str(round(RPY(3), 3));

% --- Executes during object creation, after setting all properties.
function slider6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Define Min, Max and starting Angle for Q6 Slider
set(hObject,'Min',-105,'Max',105);
set(hObject, 'Value', 0);
% Set step sizes of 5 degrees when pressing slider buttons
step = [5, 5]/(get(hObject, 'Max') - get(hObject, 'Min'));
set(hObject, 'SliderStep', step);

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider7_Callback(hObject, eventdata, handles)
% hObject    handle to slider7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
% Get current slider value
newQ7 = round(get(hObject, 'Value'), 1);
% Set RHS text box to be this value
handles.q7.String = num2str(newQ7);
% Update X,Y,Z and R,P,Y values and animate robot to match new joint state.
allQ = handles.myRobot.getpos();
allQ(7) = newQ7*pi/180;
handles.myRobot.animate(allQ);
% Now we want to update the EE Position on the GUI
% Calculate the Robot EE Position with FK
myRobot_TR = handles.myRobot.fkine(allQ);
handles.ee_X.String = num2str(round(myRobot_TR(1,4), 3));
handles.ee_Y.String = num2str(round(myRobot_TR(2,4), 3));
handles.ee_Z.String = num2str(round(myRobot_TR(3,4), 3));
% Extract Rotation Portion of FK Matrix
rot = myRobot_TR(1:3, 1:3);
% Convert Rotation Matrix to Roll, Pitch, Yaw Values in Degrees
RPY = tr2rpy(rot, 'deg');
handles.ee_Roll.String = num2str(round(RPY(1), 3));
handles.ee_Pitch.String = num2str(round(RPY(2), 3));
handles.ee_Yaw.String = num2str(round(RPY(3), 3));

% --- Executes during object creation, after setting all properties.
function slider7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Define Min, Max and starting Angle for Q7 Slider
set(hObject,'Min',-150,'Max',150);
set(hObject, 'Value', 0);
% Set step sizes of 5 degrees when pressing slider buttons
step = [5, 5]/(get(hObject, 'Max') - get(hObject, 'Min'));
set(hObject, 'SliderStep', step);


% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function q1_Callback(hObject, eventdata, handles)
% hObject    handle to q1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of q1 as text
%        str2double(get(hObject,'String')) returns contents of q1 as a double
% Get new value
newQ1 = round(str2double(get(hObject,'String')), 1);
handles.q1.String = num2str(newQ1); % We only want to show to the first d.p.
% Get current values
allQ = handles.myRobot.getpos();

% Check new value is in joint range
if newQ1*pi/180 < handles.myRobot.qlim(1,1) || newQ1*pi/180 > handles.myRobot.qlim(1,2)
    % Not in joint limits, need to revert to old Q1
    handles.q1.String = num2str(round(allQ(1)*180/pi, 1));
else
    % Move slider for Q1 to this position
    set(handles.slider1,'Value',newQ1);
    % Adjust pose of robot with animate
    allQ = handles.myRobot.getpos();
    allQ(1) = newQ1*pi/180;
    handles.myRobot.animate(allQ);
    % Now we want to update the EE Position on the GUI
    % Calculate the Robot EE Position with FK
    myRobot_TR = handles.myRobot.fkine(allQ);
    handles.ee_X.String = num2str(round(myRobot_TR(1,4), 3));
    handles.ee_Y.String = num2str(round(myRobot_TR(2,4), 3));
    handles.ee_Z.String = num2str(round(myRobot_TR(3,4), 3));
    % Extract Rotation Portion of FK Matrix
    rot = myRobot_TR(1:3, 1:3);
    % Convert Rotation Matrix to Roll, Pitch, Yaw Values in Degrees
    RPY = tr2rpy(rot, 'deg');
    handles.ee_Roll.String = num2str(round(RPY(1), 3));
    handles.ee_Pitch.String = num2str(round(RPY(2), 3));
    handles.ee_Yaw.String = num2str(round(RPY(3), 3));
end


% --- Executes during object creation, after setting all properties.
function q1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function q2_Callback(hObject, eventdata, handles)
% hObject    handle to q2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of q2 as text
%        str2double(get(hObject,'String')) returns contents of q2 as a double
% Get new value
newQ2 = round(str2double(get(hObject,'String')), 1);
handles.q2.String = num2str(newQ2); % We only want to show to the first d.p.
% Get current values
allQ = handles.myRobot.getpos();

% Check new value is in joint range
if newQ2*pi/180 < handles.myRobot.qlim(2,1) || newQ2*pi/180 > handles.myRobot.qlim(2,2)
    % Not in joint limits, need to revert to old Q2
    handles.q2.String = num2str(round(allQ(2)*180/pi, 1));
else
    % Move slider for Q2 to this position
    set(handles.slider2,'Value',newQ2);
    % Adjust pose of robot with animate
    allQ = handles.myRobot.getpos();
    allQ(2) = newQ2*pi/180;
    handles.myRobot.animate(allQ);
    % Now we want to update the EE Position on the GUI
    % Calculate the Robot EE Position with FK
    myRobot_TR = handles.myRobot.fkine(allQ);
    handles.ee_X.String = num2str(round(myRobot_TR(1,4), 3));
    handles.ee_Y.String = num2str(round(myRobot_TR(2,4), 3));
    handles.ee_Z.String = num2str(round(myRobot_TR(3,4), 3));
    % Extract Rotation Portion of FK Matrix
    rot = myRobot_TR(1:3, 1:3);
    % Convert Rotation Matrix to Roll, Pitch, Yaw Values in Degrees
    RPY = tr2rpy(rot, 'deg');
    handles.ee_Roll.String = num2str(round(RPY(1), 3));
    handles.ee_Pitch.String = num2str(round(RPY(2), 3));
    handles.ee_Yaw.String = num2str(round(RPY(3), 3));
end

% --- Executes during object creation, after setting all properties.
function q2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function q3_Callback(hObject, eventdata, handles)
% hObject    handle to q3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of q3 as text
%        str2double(get(hObject,'String')) returns contents of q3 as a double
% Get new value
newQ3 = round(str2double(get(hObject,'String')), 1);
handles.q3.String = num2str(newQ3); % We only want to show to the first d.p.
% Get current values
allQ = handles.myRobot.getpos();

% Check new value is in joint range
if newQ3*pi/180 < handles.myRobot.qlim(3,1) || newQ3*pi/180 > handles.myRobot.qlim(3,2)
    % Not in joint limits, need to revert to old Q3
    handles.q3.String = num2str(round(allQ(3)*180/pi, 1));
else
    % Move slider for Q3 to this position
    set(handles.slider3,'Value',newQ3);
    % Adjust pose of robot with animate
    allQ = handles.myRobot.getpos();
    allQ(3) = newQ3*pi/180;
    handles.myRobot.animate(allQ);
    % Now we want to update the EE Position on the GUI
    % Calculate the Robot EE Position with FK
    myRobot_TR = handles.myRobot.fkine(allQ);
    handles.ee_X.String = num2str(round(myRobot_TR(1,4), 3));
    handles.ee_Y.String = num2str(round(myRobot_TR(2,4), 3));
    handles.ee_Z.String = num2str(round(myRobot_TR(3,4), 3));
    % Extract Rotation Portion of FK Matrix
    rot = myRobot_TR(1:3, 1:3);
    % Convert Rotation Matrix to Roll, Pitch, Yaw Values in Degrees
    RPY = tr2rpy(rot, 'deg');
    handles.ee_Roll.String = num2str(round(RPY(1), 3));
    handles.ee_Pitch.String = num2str(round(RPY(2), 3));
    handles.ee_Yaw.String = num2str(round(RPY(3), 3));
end

% --- Executes during object creation, after setting all properties.
function q3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function q4_Callback(hObject, eventdata, handles)
% hObject    handle to q4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of q4 as text
%        str2double(get(hObject,'String')) returns contents of q4 as a double
% Get new value
newQ4 = round(str2double(get(hObject,'String')), 1);
handles.q4.String = num2str(newQ4); % We only want to show to the first d.p.
% Get current values
allQ = handles.myRobot.getpos();

% Check new value is in joint range
if newQ4*pi/180 < handles.myRobot.qlim(4,1) || newQ4*pi/180 > handles.myRobot.qlim(4,2)
    % Not in joint limits, need to revert to old Q4
    handles.q2.String = num2str(round(allQ(4)*180/pi, 1));
else
    % Move slider for Q4 to this position
    set(handles.slider4,'Value',newQ4);
    % Adjust pose of robot with animate
    allQ = handles.myRobot.getpos();
    allQ(4) = newQ4*pi/180;
    handles.myRobot.animate(allQ);
    % Now we want to update the EE Position on the GUI
    % Calculate the Robot EE Position with FK
    myRobot_TR = handles.myRobot.fkine(allQ);
    handles.ee_X.String = num2str(round(myRobot_TR(1,4), 3));
    handles.ee_Y.String = num2str(round(myRobot_TR(2,4), 3));
    handles.ee_Z.String = num2str(round(myRobot_TR(3,4), 3));
    % Extract Rotation Portion of FK Matrix
    rot = myRobot_TR(1:3, 1:3);
    % Convert Rotation Matrix to Roll, Pitch, Yaw Values in Degrees
    RPY = tr2rpy(rot, 'deg');
    handles.ee_Roll.String = num2str(round(RPY(1), 3));
    handles.ee_Pitch.String = num2str(round(RPY(2), 3));
    handles.ee_Yaw.String = num2str(round(RPY(3), 3));
end

% --- Executes during object creation, after setting all properties.
function q4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function q5_Callback(hObject, eventdata, handles)
% hObject    handle to q5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of q5 as text
%        str2double(get(hObject,'String')) returns contents of q5 as a double
% Get new value
newQ5 = round(str2double(get(hObject,'String')), 1);
handles.q5.String = num2str(newQ5); % We only want to show to the first d.p.
% Get current values
allQ = handles.myRobot.getpos();

% Check new value is in joint range
if newQ5*pi/180 < handles.myRobot.qlim(5,1) || newQ5*pi/180 > handles.myRobot.qlim(5,2)
    % Not in joint limits, need to revert to old Q5
    handles.q5.String = num2str(round(allQ(5)*180/pi, 1));
else
    % Move slider for Q5 to this position
    set(handles.slider5,'Value',newQ5);
    % Adjust pose of robot with animate
    allQ = handles.myRobot.getpos();
    allQ(5) = newQ5*pi/180;
    handles.myRobot.animate(allQ);
    % Now we want to update the EE Position on the GUI
    % Calculate the Robot EE Position with FK
    myRobot_TR = handles.myRobot.fkine(allQ);
    handles.ee_X.String = num2str(round(myRobot_TR(1,4), 3));
    handles.ee_Y.String = num2str(round(myRobot_TR(2,4), 3));
    handles.ee_Z.String = num2str(round(myRobot_TR(3,4), 3));
    % Extract Rotation Portion of FK Matrix
    rot = myRobot_TR(1:3, 1:3);
    % Convert Rotation Matrix to Roll, Pitch, Yaw Values in Degrees
    RPY = tr2rpy(rot, 'deg');
    handles.ee_Roll.String = num2str(round(RPY(1), 3));
    handles.ee_Pitch.String = num2str(round(RPY(2), 3));
    handles.ee_Yaw.String = num2str(round(RPY(3), 3));
end

% --- Executes during object creation, after setting all properties.
function q5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function q6_Callback(hObject, eventdata, handles)
% hObject    handle to q6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of q6 as text
%        str2double(get(hObject,'String')) returns contents of q6 as a double
% Get new value
newQ6 = round(str2double(get(hObject,'String')), 1);
handles.q6.String = num2str(newQ6); % We only want to show to the first d.p.
% Get current values
allQ = handles.myRobot.getpos();

% Check new value is in joint range
if newQ6*pi/180 < handles.myRobot.qlim(6,1) || newQ6*pi/180 > handles.myRobot.qlim(6,2)
    % Not in joint limits, need to revert to old Q6
    handles.q6.String = num2str(round(allQ(6)*180/pi, 1));
else
    % Move slider for Q6 to this position
    set(handles.slider6,'Value',newQ6);
    % Adjust pose of robot with animate
    allQ = handles.myRobot.getpos();
    allQ(6) = newQ6*pi/180;
    handles.myRobot.animate(allQ);
    % Now we want to update the EE Position on the GUI
    % Calculate the Robot EE Position with FK
    myRobot_TR = handles.myRobot.fkine(allQ);
    handles.ee_X.String = num2str(round(myRobot_TR(1,4), 3));
    handles.ee_Y.String = num2str(round(myRobot_TR(2,4), 3));
    handles.ee_Z.String = num2str(round(myRobot_TR(3,4), 3));
    % Extract Rotation Portion of FK Matrix
    rot = myRobot_TR(1:3, 1:3);
    % Convert Rotation Matrix to Roll, Pitch, Yaw Values in Degrees
    RPY = tr2rpy(rot, 'deg');
    handles.ee_Roll.String = num2str(round(RPY(1), 3));
    handles.ee_Pitch.String = num2str(round(RPY(2), 3));
    handles.ee_Yaw.String = num2str(round(RPY(3), 3));
end

% --- Executes during object creation, after setting all properties.
function q6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function q7_Callback(hObject, eventdata, handles)
% hObject    handle to q7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of q7 as text
%        str2double(get(hObject,'String')) returns contents of q7 as a double
% Get new value
newQ7 = round(str2double(get(hObject,'String')), 1);
handles.q7.String = num2str(newQ7); % We only want to show to the first d.p.
% Get current values
allQ = handles.myRobot.getpos();

% Check new value is in joint range
if newQ7*pi/180 < handles.myRobot.qlim(7,1) || newQ7*pi/180 > handles.myRobot.qlim(7,2)
    % Not in joint limits, need to revert to old Q7
    handles.q7.String = num2str(round(allQ(7)*180/pi, 1));
else
    % Move slider for Q7 to this position
    set(handles.slider7,'Value',newQ7);
    % Adjust pose of robot with animate
    allQ = handles.myRobot.getpos();
    allQ(7) = newQ7*pi/180;
    handles.myRobot.animate(allQ);
    % Now we want to update the EE Position on the GUI
    % Calculate the Robot EE Position with FK
    myRobot_TR = handles.myRobot.fkine(allQ);
    handles.ee_X.String = num2str(round(myRobot_TR(1,4), 3));
    handles.ee_Y.String = num2str(round(myRobot_TR(2,4), 3));
    handles.ee_Z.String = num2str(round(myRobot_TR(3,4), 3));
    % Extract Rotation Portion of FK Matrix
    rot = myRobot_TR(1:3, 1:3);
    % Convert Rotation Matrix to Roll, Pitch, Yaw Values in Degrees
    RPY = tr2rpy(rot, 'deg');
    handles.ee_Roll.String = num2str(round(RPY(1), 3));
    handles.ee_Pitch.String = num2str(round(RPY(2), 3));
    handles.ee_Yaw.String = num2str(round(RPY(3), 3));
end

% --- Executes during object creation, after setting all properties.
function q7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in eStop.
function eStop_Callback(hObject, eventdata, handles)
% hObject    handle to eStop (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in up_X.
function up_X_Callback(hObject, eventdata, handles)
% hObject    handle to up_X (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Increase X value by 0.05m;
% Create a 4x4 Matrix to define new transform (Get FK, add 0.05 to X
% translation)
currentQ = handles.myRobot.getpos();
ee_TR = handles.myRobot.fkine(currentQ);

if handles.cartKeepCurrentRPY == 1
    ee_TR(1,4) = ee_TR(1,4) + handles.cartInc;
    % Use IK to get joint state for new desired EE pose
    newQ = handles.myRobot.ikcon(ee_TR, currentQ);
else
    XYZ = [ee_TR(1,4)+handles.cartInc, ee_TR(2,4), ee_TR(3,4)];
    TR = [eye(3)    XYZ';
          zeros(1,3) 1]
    % Use IK to get joint state for new desired EE pose
    newQ = handles.myRobot.ikcon(TR, currentQ);
end

% Update sliders with new joint states
handles.q1.String = num2str(round(rad2deg(newQ(1)),1));
set(handles.slider1,'Value',round(rad2deg(newQ(1)),1));
handles.q2.String = num2str(round(rad2deg(newQ(2)),1));
set(handles.slider2,'Value',round(rad2deg(newQ(2)),1));
handles.q3.String = num2str(round(rad2deg(newQ(3)),1));
set(handles.slider3,'Value',round(rad2deg(newQ(3)),1));
handles.q4.String = num2str(round(rad2deg(newQ(4)),1));
set(handles.slider4,'Value',round(rad2deg(newQ(4)),1));
handles.q5.String = num2str(round(rad2deg(newQ(5)),1));
set(handles.slider5,'Value',round(rad2deg(newQ(5)),1));
handles.q6.String = num2str(round(rad2deg(newQ(6)),1));
set(handles.slider6,'Value',round(rad2deg(newQ(6)),1));
handles.q7.String = num2str(round(rad2deg(newQ(7)),1));
set(handles.slider7,'Value',round(rad2deg(newQ(7)),1));
% Update robot pose
handles.myRobot.animate(newQ);
% Get new EE pose with FK and update XYZ, RPY values
myRobot_TR = handles.myRobot.fkine(newQ)
handles.ee_X.String = num2str(round(myRobot_TR(1,4), 3));
handles.ee_Y.String = num2str(round(myRobot_TR(2,4), 3));
handles.ee_Z.String = num2str(round(myRobot_TR(3,4), 3));
% Extract Rotation Portion of FK Matrix
rot = myRobot_TR(1:3, 1:3);
% Convert Rotation Matrix to Roll, Pitch, Yaw Values in Degrees
RPY = tr2rpy(rot, 'deg');
handles.ee_Roll.String = num2str(round(RPY(1), 3));
handles.ee_Pitch.String = num2str(round(RPY(2), 3));
handles.ee_Yaw.String = num2str(round(RPY(3), 3));


% --- Executes on button press in down_X.
function down_X_Callback(hObject, eventdata, handles)
% hObject    handle to down_X (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Decrease X value by 0.05m
% Create a 4x4 Matrix to define new transform (Get FK, sub 0.05 to X
% translation)
currentQ = handles.myRobot.getpos();
ee_TR = handles.myRobot.fkine(currentQ);

if handles.cartKeepCurrentRPY == 1
    ee_TR(1,4) = ee_TR(1,4) - handles.cartInc;
    % Use IK to get joint state for new desired EE pose
    newQ = handles.myRobot.ikcon(ee_TR, currentQ);
else
    XYZ = [ee_TR(1,4)-handles.cartInc, ee_TR(2,4), ee_TR(3,4)];
    TR = [eye(3)    XYZ';
          zeros(1,3) 1]
    % Use IK to get joint state for new desired EE pose
    newQ = handles.myRobot.ikcon(TR, currentQ);
end

% Update sliders with new joint states
handles.q1.String = num2str(round(rad2deg(newQ(1)),1));
set(handles.slider1,'Value',round(rad2deg(newQ(1)),1));
handles.q2.String = num2str(round(rad2deg(newQ(2)),1));
set(handles.slider2,'Value',round(rad2deg(newQ(2)),1));
handles.q3.String = num2str(round(rad2deg(newQ(3)),1));
set(handles.slider3,'Value',round(rad2deg(newQ(3)),1));
handles.q4.String = num2str(round(rad2deg(newQ(4)),1));
set(handles.slider4,'Value',round(rad2deg(newQ(4)),1));
handles.q5.String = num2str(round(rad2deg(newQ(5)),1));
set(handles.slider5,'Value',round(rad2deg(newQ(5)),1));
handles.q6.String = num2str(round(rad2deg(newQ(6)),1));
set(handles.slider6,'Value',round(rad2deg(newQ(6)),1));
handles.q7.String = num2str(round(rad2deg(newQ(7)),1));
set(handles.slider7,'Value',round(rad2deg(newQ(7)),1));
% Update robot pose
handles.myRobot.animate(newQ);
% Get new EE pose with FK and update XYZ, RPY values
myRobot_TR = handles.myRobot.fkine(newQ)
handles.ee_X.String = num2str(round(myRobot_TR(1,4), 3));
handles.ee_Y.String = num2str(round(myRobot_TR(2,4), 3));
handles.ee_Z.String = num2str(round(myRobot_TR(3,4), 3));
% Extract Rotation Portion of FK Matrix
rot = myRobot_TR(1:3, 1:3);
% Convert Rotation Matrix to Roll, Pitch, Yaw Values in Degrees
RPY = tr2rpy(rot, 'deg');
handles.ee_Roll.String = num2str(round(RPY(1), 3));
handles.ee_Pitch.String = num2str(round(RPY(2), 3));
handles.ee_Yaw.String = num2str(round(RPY(3), 3));


% --- Executes on button press in up_Y.
function up_Y_Callback(hObject, eventdata, handles)
% hObject    handle to up_Y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Increase Y value by 0.05m
% Create a 4x4 Matrix to define new transform (Get FK, add 0.05 to Y
% translation)
currentQ = handles.myRobot.getpos();
ee_TR = handles.myRobot.fkine(currentQ);

if handles.cartKeepCurrentRPY == 1
    XYZ = [ee_TR(1,4), ee_TR(2,4)+handles.cartInc, ee_TR(3,4)];
    TR = [eye(3)    XYZ';
          zeros(1,3) 1]
    % Use IK to get joint state for new desired EE pose
    newQ = handles.myRobot.ikcon(TR, currentQ);
else
    ee_TR(2,4) = ee_TR(2,4) + handles.cartInc;
    % Use IK to get joint state for new desired EE pose
    newQ = handles.myRobot.ikcon(ee_TR, currentQ);
end

% Update sliders with new joint states
handles.q1.String = num2str(round(rad2deg(newQ(1)),1));
set(handles.slider1,'Value',round(rad2deg(newQ(1)),1));
handles.q2.String = num2str(round(rad2deg(newQ(2)),1));
set(handles.slider2,'Value',round(rad2deg(newQ(2)),1));
handles.q3.String = num2str(round(rad2deg(newQ(3)),1));
set(handles.slider3,'Value',round(rad2deg(newQ(3)),1));
handles.q4.String = num2str(round(rad2deg(newQ(4)),1));
set(handles.slider4,'Value',round(rad2deg(newQ(4)),1));
handles.q5.String = num2str(round(rad2deg(newQ(5)),1));
set(handles.slider5,'Value',round(rad2deg(newQ(5)),1));
handles.q6.String = num2str(round(rad2deg(newQ(6)),1));
set(handles.slider6,'Value',round(rad2deg(newQ(6)),1));
handles.q7.String = num2str(round(rad2deg(newQ(7)),1));
set(handles.slider7,'Value',round(rad2deg(newQ(7)),1));
% Update robot pose
handles.myRobot.animate(newQ);
% Get new EE pose with FK and update XYZ, RPY values
myRobot_TR = handles.myRobot.fkine(newQ)
handles.ee_X.String = num2str(round(myRobot_TR(1,4), 3));
handles.ee_Y.String = num2str(round(myRobot_TR(2,4), 3));
handles.ee_Z.String = num2str(round(myRobot_TR(3,4), 3));
% Extract Rotation Portion of FK Matrix
rot = myRobot_TR(1:3, 1:3);
% Convert Rotation Matrix to Roll, Pitch, Yaw Values in Degrees
RPY = tr2rpy(rot, 'deg');
handles.ee_Roll.String = num2str(round(RPY(1), 3));
handles.ee_Pitch.String = num2str(round(RPY(2), 3));
handles.ee_Yaw.String = num2str(round(RPY(3), 3));


% --- Executes on button press in down_Y.
function down_Y_Callback(hObject, eventdata, handles)
% hObject    handle to down_Y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Decrease Y value by 0.05m
% Create a 4x4 Matrix to define new transform (Get FK, sub 0.05 to Y
% translation)
currentQ = handles.myRobot.getpos();
ee_TR = handles.myRobot.fkine(currentQ);

if handles.cartKeepCurrentRPY == 1
    ee_TR(2,4) = ee_TR(2,4) - handles.cartInc;
    % Use IK to get joint state for new desired EE pose
    newQ = handles.myRobot.ikcon(ee_TR, currentQ);
else
    XYZ = [ee_TR(1,4), ee_TR(2,4)-handles.cartInc, ee_TR(3,4)];
    TR = [eye(3)    XYZ';
          zeros(1,3) 1]
    % Use IK to get joint state for new desired EE pose
    newQ = handles.myRobot.ikcon(TR, currentQ);
end

% Update sliders with new joint states
handles.q1.String = num2str(round(rad2deg(newQ(1)),1));
set(handles.slider1,'Value',round(rad2deg(newQ(1)),1));
handles.q2.String = num2str(round(rad2deg(newQ(2)),1));
set(handles.slider2,'Value',round(rad2deg(newQ(2)),1));
handles.q3.String = num2str(round(rad2deg(newQ(3)),1));
set(handles.slider3,'Value',round(rad2deg(newQ(3)),1));
handles.q4.String = num2str(round(rad2deg(newQ(4)),1));
set(handles.slider4,'Value',round(rad2deg(newQ(4)),1));
handles.q5.String = num2str(round(rad2deg(newQ(5)),1));
set(handles.slider5,'Value',round(rad2deg(newQ(5)),1));
handles.q6.String = num2str(round(rad2deg(newQ(6)),1));
set(handles.slider6,'Value',round(rad2deg(newQ(6)),1));
handles.q7.String = num2str(round(rad2deg(newQ(7)),1));
set(handles.slider7,'Value',round(rad2deg(newQ(7)),1));
% Update robot pose
handles.myRobot.animate(newQ);
% Get new EE pose with FK and update XYZ, RPY values
myRobot_TR = handles.myRobot.fkine(newQ)
handles.ee_X.String = num2str(round(myRobot_TR(1,4), 3));
handles.ee_Y.String = num2str(round(myRobot_TR(2,4), 3));
handles.ee_Z.String = num2str(round(myRobot_TR(3,4), 3));
% Extract Rotation Portion of FK Matrix
rot = myRobot_TR(1:3, 1:3);
% Convert Rotation Matrix to Roll, Pitch, Yaw Values in Degrees
RPY = tr2rpy(rot, 'deg');
handles.ee_Roll.String = num2str(round(RPY(1), 3));
handles.ee_Pitch.String = num2str(round(RPY(2), 3));
handles.ee_Yaw.String = num2str(round(RPY(3), 3));


% --- Executes on button press in up_Z.
function up_Z_Callback(hObject, eventdata, handles)
% hObject    handle to up_Z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Increase Z value by 0.05m
% Create a 4x4 Matrix to define new transform (Get FK, add 0.05 to Z
% translation)
currentQ = handles.myRobot.getpos();
ee_TR = handles.myRobot.fkine(currentQ);

if handles.cartKeepCurrentRPY == 1
    ee_TR(3,4) = ee_TR(3,4) + handles.cartInc;
    % Use IK to get joint state for new desired EE pose
    newQ = handles.myRobot.ikcon(ee_TR, currentQ);
else
    XYZ = [ee_TR(1,4), ee_TR(2,4), ee_TR(3,4)+handles.cartInc];
    TR = [eye(3)    XYZ';
          zeros(1,3) 1]
    % Use IK to get joint state for new desired EE pose
    newQ = handles.myRobot.ikcon(TR, currentQ);
end

% Update sliders with new joint states
handles.q1.String = num2str(round(rad2deg(newQ(1)),1));
set(handles.slider1,'Value',round(rad2deg(newQ(1)),1));
handles.q2.String = num2str(round(rad2deg(newQ(2)),1));
set(handles.slider2,'Value',round(rad2deg(newQ(2)),1));
handles.q3.String = num2str(round(rad2deg(newQ(3)),1));
set(handles.slider3,'Value',round(rad2deg(newQ(3)),1));
handles.q4.String = num2str(round(rad2deg(newQ(4)),1));
set(handles.slider4,'Value',round(rad2deg(newQ(4)),1));
handles.q5.String = num2str(round(rad2deg(newQ(5)),1));
set(handles.slider5,'Value',round(rad2deg(newQ(5)),1));
handles.q6.String = num2str(round(rad2deg(newQ(6)),1));
set(handles.slider6,'Value',round(rad2deg(newQ(6)),1));
handles.q7.String = num2str(round(rad2deg(newQ(7)),1));
set(handles.slider7,'Value',round(rad2deg(newQ(7)),1));
% Update robot pose
handles.myRobot.animate(newQ);
% Get new EE pose with FK and update XYZ, RPY values
myRobot_TR = handles.myRobot.fkine(newQ)
handles.ee_X.String = num2str(round(myRobot_TR(1,4), 3));
handles.ee_Y.String = num2str(round(myRobot_TR(2,4), 3));
handles.ee_Z.String = num2str(round(myRobot_TR(3,4), 3));
% Extract Rotation Portion of FK Matrix
rot = myRobot_TR(1:3, 1:3);
% Convert Rotation Matrix to Roll, Pitch, Yaw Values in Degrees
RPY = tr2rpy(rot, 'deg');
handles.ee_Roll.String = num2str(round(RPY(1), 3));
handles.ee_Pitch.String = num2str(round(RPY(2), 3));
handles.ee_Yaw.String = num2str(round(RPY(3), 3));


% --- Executes on button press in down_Z.
function down_Z_Callback(hObject, eventdata, handles)
% hObject    handle to down_Z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Decrease Z value by 0.05m
% Create a 4x4 Matrix to define new transform (Get FK, sub 0.05 to Z
% translation)
currentQ = handles.myRobot.getpos();
ee_TR = handles.myRobot.fkine(currentQ);

if handles.cartKeepCurrentRPY == 1
    ee_TR(3,4) = ee_TR(3,4) - handles.cartInc;
    % Use IK to get joint state for new desired EE pose
    newQ = handles.myRobot.ikcon(ee_TR, currentQ);
else
    XYZ = [ee_TR(1,4), ee_TR(2,4), ee_TR(3,4)-handles.cartInc];
    TR = [eye(3)    XYZ';
          zeros(1,3) 1]
    % Use IK to get joint state for new desired EE pose
    newQ = handles.myRobot.ikcon(TR, currentQ);
end

% Update sliders with new joint states
handles.q1.String = num2str(round(rad2deg(newQ(1)),1));
set(handles.slider1,'Value',round(rad2deg(newQ(1)),1));
handles.q2.String = num2str(round(rad2deg(newQ(2)),1));
set(handles.slider2,'Value',round(rad2deg(newQ(2)),1));
handles.q3.String = num2str(round(rad2deg(newQ(3)),1));
set(handles.slider3,'Value',round(rad2deg(newQ(3)),1));
handles.q4.String = num2str(round(rad2deg(newQ(4)),1));
set(handles.slider4,'Value',round(rad2deg(newQ(4)),1));
handles.q5.String = num2str(round(rad2deg(newQ(5)),1));
set(handles.slider5,'Value',round(rad2deg(newQ(5)),1));
handles.q6.String = num2str(round(rad2deg(newQ(6)),1));
set(handles.slider6,'Value',round(rad2deg(newQ(6)),1));
handles.q7.String = num2str(round(rad2deg(newQ(7)),1));
set(handles.slider7,'Value',round(rad2deg(newQ(7)),1));
% Update robot pose
handles.myRobot.animate(newQ);
% Get new EE pose with FK and update XYZ, RPY values
myRobot_TR = handles.myRobot.fkine(newQ)
handles.ee_X.String = num2str(round(myRobot_TR(1,4), 3));
handles.ee_Y.String = num2str(round(myRobot_TR(2,4), 3));
handles.ee_Z.String = num2str(round(myRobot_TR(3,4), 3));
% Extract Rotation Portion of FK Matrix
rot = myRobot_TR(1:3, 1:3);
% Convert Rotation Matrix to Roll, Pitch, Yaw Values in Degrees
RPY = tr2rpy(rot, 'deg');
handles.ee_Roll.String = num2str(round(RPY(1), 3));
handles.ee_Pitch.String = num2str(round(RPY(2), 3));
handles.ee_Yaw.String = num2str(round(RPY(3), 3));


% --- Executes on button press in greenPen.
function greenPen_Callback(hObject, eventdata, handles)
% hObject    handle to greenPen (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of greenPen


% --- Executes on button press in bluePen.
function bluePen_Callback(hObject, eventdata, handles)
% hObject    handle to bluePen (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of bluePen


% --- Executes on button press in redPen.
function redPen_Callback(hObject, eventdata, handles)
% hObject    handle to redPen (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of redPen


% --- Executes on button press in blackPen.
function blackPen_Callback(hObject, eventdata, handles)
% hObject    handle to blackPen (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of blackPen


% --- Executes on button press in chooseDrawingBtn.
function chooseDrawingBtn_Callback(hObject, eventdata, handles)
% hObject    handle to chooseDrawingBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Search for an Image/Drawing
[filename, filepath] = uigetfile({'*.*'; '*.jpg*'; '*.jpeg*'; '*.png*'}, ...
                        'Search Drawing');
handles.drawingPath = [filepath filename];

% Read the Image
handles.drawing = imread(handles.drawingPath);
% Display the Image
axes(handles.axes3)
imshow(handles.drawing);
% Remove Axis Scale
axis off

% --- Executes on button press in startDrawingBtn.
function startDrawingBtn_Callback(hObject, eventdata, handles)
% hObject    handle to startDrawingBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Set handles.colour to be the colour pen chosen from the Button Group
h = get(handles.colourBtnGroup,'SelectedObject');
handles.colour = get(h, 'Tag');
% Switch statement to get 4x4 Transform Matrix of colour pen to pick up
switch handles.colour
    case 'blackPen'
        pen_T = handles.blackPen;
    case 'redPen'
        pen_T = handles.redPen;
    case 'greenPen'
        pen_T = handles.greenPen;
    case 'bluePen'
        pen_T = handles.bluePen;
end

% Move the Hans Cute to pick up the pen!
qGuess_Pen = [60 60 0 80 0 -50 90]*pi/180;
steps = 70;
hold on

usingRMRC = 1;
if usingRMRC == 0
    pen_T = pen_T*transl(-0.01, 0, 0); % Shifting target transform slightly so pen is centred in gripper
    %qGuess_Pen = [25 90 0 0 -65 0 90]*pi/180;
    %qGuess_Pen = [60 45 0 105 0 -60 90]*pi/180;

    qOut = handles.rMove.MoveRobotToObject(handles.myRobot, pen_T, 0.1, ...
        qGuess_Pen, steps);
    % Use RMRC to move the Hans Cute back up 10cm
    % Define starting and desired end transform
    start_T = handles.myRobot.fkine(qOut);
    end_T = pen_T*transl(0, -0.08, 0);  % Y-Axis pointing downwards
    % RMRC Parameters: Robot, Start 4x4, End 4x4, Time of Traj, ...
    % Plot Traj Trail?, Plot Traj Data?, Moving Mesh?
    qOut = handles.rMove.RMRC_7DOF(handles.myRobot, start_T, end_T, 1, 0, 0);
    
else
    % Move above the target point
    pen_T = pen_T*transl(-0.01, 0, 0.1);
    qOut = handles.rMove.MoveRobotToObject2(handles.myRobot, pen_T, ...
        qGuess_Pen, steps);
    
    % Use RMRC to move down towards the pen and then back up with it
    % RMRC Parameters: Robot, Start 4x4, End 4x4, Time of Traj, ...
    % Plot Traj Trail?, Plot Traj Data?, Moving Mesh?
    % Moving down:
    start_T = handles.myRobot.fkine(qOut);
    end_T = pen_T*transl(0, 0, -0.08);  % Y-Axis pointing downwards
    qOut = handles.rMove.RMRC_7DOF(handles.myRobot, start_T, end_T, 1, 1, 0);
    
    % Moving up:
    start_T = handles.myRobot.fkine(qOut);
    end_T = pen_T;
    qOut = handles.rMove.RMRC_7DOF(handles.myRobot, start_T, end_T, 1, 1, 0);
end
    
        

% --- Executes on button press in resumeBtn.
function resumeBtn_Callback(hObject, eventdata, handles)
% hObject    handle to resumeBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes when selected object is changed in uibuttongroup4.
function uibuttongroup4_SelectionChangedFcn(hObject, eventdata, handles)
% hObject    handle to the selected object in uibuttongroup4 
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
