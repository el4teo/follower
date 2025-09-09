function varargout = CarlosV_Control_Intarface(varargin)
% CARLOSV_CONTROL_INTARFACE MATLAB code for CarlosV_Control_Intarface.fig
%      CARLOSV_CONTROL_INTARFACE, by itself, creates a new CARLOSV_CONTROL_INTARFACE or raises the existing
%      singleton*.
%
%      H = CARLOSV_CONTROL_INTARFACE returns the handle to a new CARLOSV_CONTROL_INTARFACE or the handle to
%      the existing singleton*.
%
%      CARLOSV_CONTROL_INTARFACE('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in CARLOSV_CONTROL_INTARFACE.M with the given input arguments.
%
%      CARLOSV_CONTROL_INTARFACE('Property','Value',...) creates a new CARLOSV_CONTROL_INTARFACE or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before CarlosV_Control_Intarface_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to CarlosV_Control_Intarface_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help CarlosV_Control_Intarface

% Last Modified by GUIDE v2.5 27-Dec-2018 09:29:58

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @CarlosV_Control_Intarface_OpeningFcn, ...
                   'gui_OutputFcn',  @CarlosV_Control_Intarface_OutputFcn, ...
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


% --- Executes just before CarlosV_Control_Intarface is made visible.
function CarlosV_Control_Intarface_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to CarlosV_Control_Intarface (see VARARGIN)

% Choose default command line output for CarlosV_Control_Intarface
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% Crear directorios
if exist('Datos','dir') == 0
    mkdir('Datos');
end
if exist('Imágenes','dir') == 0
    mkdir('Imágenes');
end
        
% Reiniciar variables de control
ReiniciarVariablesDeControl(handles);

% Configurar Axes
hold(handles.AxSensor, 'on');
handles.AxSensor.XLabel.String = 'Tiempo ( s )';
handles.AxSensor.YLabel.String  = 'Izquierda                                             Derecha  ';
handles.AxSensor.Title.String  = 'SEGUIDOR DE LINEAS';

% Graficar línea del sensor
colorSensor = [0 0 0.5];
yyaxis (handles.AxSensor, 'left');
handles.AxSensor.YLabel.Color = colorSensor;
handles.AxSensor.YColor = colorSensor;
n = 501;
T = 10e-3;
x = linspace(-1* T * (n - 1), 0, n);
xlim([x(1) x(end)]);
y = zeros(n,1);
handles.figure1.UserData.h.sensor = plot(handles.AxSensor, x, y, 'LineWidth', 3, 'Color', colorSensor);
ylim([-100 100]);

% Graficar lineas de los motores
colorMotor1 = [0.5 0 0];
colorMotor2 = [0 0.5 0];
yyaxis (handles.AxSensor, 'right');
handles.AxSensor.YLabel.Color = [0 0 0];
handles.AxSensor.YColor = [0 0 0];
ylabel('Potencia de los motores ( % )');
handles.figure1.UserData.h.mIzq = plot(handles.AxSensor, x, y, '-','LineWidth', 2, 'Color', colorMotor1);
handles.figure1.UserData.h.mDer = plot(handles.AxSensor, x, y, '-', 'LineWidth', 2, 'Color', colorMotor2);
ylim([handles.figure1.UserData.PotMin handles.figure1.UserData.PotMax]);

% Legend
handles.figure1.UserData.h.legend = legend('Sensor de linea','Motor Izq.','Motor Der.');
 
% Deshabilitar botones
        handles.Calibrar.Enable = 'off';
        handles.CongelarImg.Enable = 'off';
        handles.Reiniciar.Enable = 'off';
        handles.ExpDatos.Enable = 'off';
        handles.ExpImagen.Enable = 'off';
        handles.Correr.Enable = 'off';
        handles.Parar.Enable = 'off';
global h
h =handles;

% UIWAIT makes CarlosV_Control_Intarface wait for user response (see UIRESUME)
% uiwait(handles.figure1);

function ReiniciarVariablesDeControl(handles)
% Leer ficheros existentes en los folders
    filesDatos = dir('Datos\*.mat');
    handles.figure1.UserData.nFilesDat = length(filesDatos);
    filesImg = dir('Imágenes\*.png');
    handles.figure1.UserData.nFilesImg = length(filesDatos);

    handles.figure1.UserData.esperandoConfirmacion = false;
    handles.figure1.UserData.bytesRecibidos = 0;
    if handles.CongelarImg.Value == 1
        handles.figure1.UserData.ActualizarAxes  = false;
    else
        handles.figure1.UserData.ActualizarAxes  = true;
    end
    handles.figure1.UserData.Reiniciando = false;
    handles.figure1.UserData.K = 0.25;
    handles.figure1.UserData.Ti = 1;
    handles.figure1.UserData.Td = 0.05;
    handles.figure1.UserData.T = 0.02;
    handles.figure1.UserData.PotMax = int16(80);
    handles.figure1.UserData.PotRef = int16(30);
    handles.figure1.UserData.PotMin = int16(-20);
    handles.K.String = num2str(handles.figure1.UserData.K);
    handles.Ti.String = num2str(handles.figure1.UserData.Ti);
    handles.Td.String = num2str(handles.figure1.UserData.Td);
    handles.T.String = num2str(handles.figure1.UserData.T);
    handles.PotMax.String = [num2str(handles.figure1.UserData.PotMax), ' %'];
    handles.PotRef.String = [num2str(handles.figure1.UserData.PotRef), ' %'];
    handles.PotMin.String = [num2str(handles.figure1.UserData.PotMin), ' %'];
        yyaxis (handles.AxSensor, 'right');
        ylim([handles.figure1.UserData.PotMin handles.figure1.UserData.PotMax]);


% --- Outputs from this function are returned to the command line.
function varargout = CarlosV_Control_Intarface_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes during object creation, after setting all properties.
function figure1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called



function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in MostrarMotores.
function MostrarMotores_Callback(hObject, eventdata, handles)
% hObject    handle to MostrarMotores (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of MostrarMotores


if strcmp(handles.figure1.UserData.h.mIzq.Visible, 'on')
    handles.figure1.UserData.h.mIzq.Visible = 'off';
    handles.figure1.UserData.h.mDer.Visible = 'off';
else
    handles.figure1.UserData.h.mIzq.Visible = 'on';
    handles.figure1.UserData.h.mDer.Visible = 'on';
end



% --- Executes on button press in MostrarSensor.
function MostrarSensor_Callback(hObject, eventdata, handles)
% hObject    handle to MostrarSensor (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of MostrarSensor

if strcmp(handles.figure1.UserData.h.sensor.Visible, 'on')
    handles.figure1.UserData.h.sensor.Visible = 'off';
else
    handles.figure1.UserData.h.sensor.Visible = 'on';
end



function Wn_Callback(hObject, eventdata, handles)
% hObject    handle to Wn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Wn as text
%        str2double(get(hObject,'String')) returns contents of Wn as a double


% --- Executes during object creation, after setting all properties.
function Wn_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Wn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Dzeta_Callback(hObject, eventdata, handles)
% hObject    handle to Dzeta (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Dzeta as text
%        str2double(get(hObject,'String')) returns contents of Dzeta as a double


% --- Executes during object creation, after setting all properties.
function Dzeta_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Dzeta (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function K_Callback(hObject, eventdata, handles)
% hObject    handle to K (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of K as text
%        str2double(get(hObject,'String')) returns contents of K as a double
try 
    myString = get(hObject,'String');
    newVal = single(str2double(myString));
    if newVal > 30
        error('K debe ser menor o igual que 30');
    end
    if newVal < 0.001
        error('K debe ser mayor o igual que 0.001');
    end
    if newVal ~= handles.figure1.UserData.K
        % Cambio normal de K
        handles.figure1.UserData.K = newVal;
        EnviarK(handles);
    end
    handles.K.String = num2str(handles.figure1.UserData.K);
catch e
    disp(['Error en K_Callback: ', e.message]);
    handles.K.String = num2str(handles.figure1.UserData.K);
    return;
end

% --- Executes during object creation, after setting all properties.
function K_CreateFcn(hObject, eventdata, handles)
% hObject    handle to K (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function PotMax_Callback(hObject, eventdata, handles)
% hObject    handle to PotMax (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of PotMax as text
%        str2double(get(hObject,'String')) returns contents of PotMax as a double
try 
    myString = get(hObject,'String');
    numStr = '';
    for i= 1:length(myString)
        myChar = myString(i);
        if myChar == '-' || (myChar >= 48 && myChar <= 57)
            numStr = [numStr, myChar];
        else
            break;
        end
    end
    newPotMax = str2double(numStr) ;
    if newPotMax < handles.figure1.UserData.PotMin
        error('PotMax debe ser mayor que PotMin');
    end
    if newPotMax > 100
        error('PotMax debe ser menor que 100');
    end
    if newPotMax < 10
        error('PotMax debe ser mayor que 10');
    end
    if newPotMax ~= handles.figure1.UserData.PotMax
        % Cambio normal de PotMax
        handles.figure1.UserData.PotMax = newPotMax;
        yyaxis (handles.AxSensor, 'right');
        ylim([handles.figure1.UserData.PotMin handles.figure1.UserData.PotMax]);
        EnviarPotMax(handles);
    end
    handles.PotMax.String = [num2str(handles.figure1.UserData.PotMax), ' %'];
catch e
    disp(['Error en PotMax_Callback: ', e.message]);
    handles.PotMax.String = [num2str(handles.figure1.UserData.PotMax), ' %'];
    return;
end

% --- Executes during object creation, after setting all properties.
function PotMax_CreateFcn(hObject, eventdata, handles)
% hObject    handle to PotMax (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Ti_Callback(hObject, eventdata, handles)
% hObject    handle to Ti (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Ti as text
%        str2double(get(hObject,'String')) returns contents of Ti as a double
try 
    myString = get(hObject,'String');
    newVal = single(str2double(myString));
    if newVal > 30
        error('Ti debe ser menor que 30');
    end
    if newVal < 0.001
        error('Ti debe ser mayor que 0.001');
    end
    if newVal ~= handles.figure1.UserData.Ti
        % Cambio normal de Ti
        handles.figure1.UserData.Ti = newVal;
        EnviarTi(handles);
    end
    handles.Ti.String = num2str(handles.figure1.UserData.Ti);
catch e
    disp(['Error en Ti_Callback: ', e.message]);
    handles.Ti.String = num2str(handles.figure1.UserData.Ti);
    return;
end

% --- Executes during object creation, after setting all properties.
function Ti_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Ti (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Td_Callback(hObject, eventdata, handles)
% hObject    handle to Td (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Td as text
%        str2double(get(hObject,'String')) returns contents of Td as a double
try 
    myString = get(hObject,'String');
    newVal = single(str2double(myString));
    if newVal > 30
        error('Td debe ser menor o igual que 30');
    end
    if newVal < 0.001
        error('Td debe ser mayor o igual que 0.001');
    end
    if newVal ~= handles.figure1.UserData.Td
        % Cambio normal de Td
        handles.figure1.UserData.Td = newVal;
        EnviarTd(handles);
    end
    handles.Td.String = num2str(handles.figure1.UserData.Td);
catch e
    disp(['Error en Td_Callback: ', e.message]);
    handles.Td.String = num2str(handles.figure1.UserData.Td);
    return;
end

% --- Executes during object creation, after setting all properties.
function Td_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Td (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in MostrarLeyendas.
function MostrarLeyendas_Callback(hObject, eventdata, handles)
% hObject    handle to MostrarLeyendas (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of MostrarLeyendas
if strcmp(handles.figure1.UserData.h.legend.Visible, 'on')
    handles.figure1.UserData.h.legend.Visible = 'off';
else
    handles.figure1.UserData.h.legend.Visible = 'on';
end


% --- Executes on button press in Conectar.
function Conectar_Callback(hObject, eventdata, handles)
% hObject    handle to Conectar (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
try
    handles.EstadoRobot.String = 'Conectando...';
    pause(0.001);
    handles.figure1.UserData.Bluetooth = Bluetooth('RobotCarlosV',1);
    handles.EstadoRobot.String = 'Abiendo bluetooth...';
    pause(0.001);
    fopen(handles.figure1.UserData.Bluetooth);
    if strcmp(handles.figure1.UserData.Bluetooth.Status, 'open')
        handles.Calibrar.Enable = 'on';
        handles.Conectar.Enable = 'off';
        handles.EstadoRobot.String = 'Bluetooth abierto';
        pause(0.001);
    else
        handles.EstadoRobot.String = 'Bluetooth cerrado';
        pause(0.001);
        handles.Calibrar.Enable = 'off';
        handles.CongelarImg.Enable = 'off';
        handles.Reiniciar.Enable = 'off';
        handles.ExpDatos.Enable = 'off';
        handles.ExpImagen.Enable = 'off';
        handles.Correr.Enable = 'off';
        handles.Parar.Enable = 'off';
    end
catch e
    handles.EstadoRobot.String = 'Error en Conectar_Callback';
    disp(['Error en Conectar_Callback: ', e.message]);
        handles.Calibrar.Enable = 'off';
        handles.CongelarImg.Enable = 'off';
        handles.Reiniciar.Enable = 'off';
        handles.ExpDatos.Enable = 'off';
        handles.ExpImagen.Enable = 'off';
        handles.Correr.Enable = 'off';
        handles.Parar.Enable = 'off';
end


% --- Executes on button press in Calibrar.
function Calibrar_Callback(hObject, eventdata, handles)
% hObject    handle to Calibrar (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
try
    handles.EstadoRobot.String = 'Calibrando...';
    pause(0.001);
% Enviar código
    fwrite(handles.figure1.UserData.Bluetooth, [255 255])
% Esperar respuesta de que la calibración ha terminado correctamente
    i = 0;
    while true
        if handles.figure1.UserData.Bluetooth.BytesAvailable > 0
            myByte = fread(handles.figure1.UserData.Bluetooth, 1);
            if myByte == 255
                break;
            end
        end
        pause(10e-3);
        i = i + 1;
        if i >= 500
            error('Timeout');
        end
    end
% Enviar valores de las variables de control
    handles.EstadoRobot.String = 'Enviando variables';
    pause(0.001);
        EnviarPotMax(handles);
        EnviarPotMin(handles);
        EnviarK(handles);
        EnviarTi(handles);
        EnviarTd(handles);
        EnviarPotRef(handles);
        EnviarT(handles); 
% Habilitar botones
        handles.Calibrar.Enable = 'off';
        handles.CongelarImg.Enable = 'on';
        handles.Reiniciar.Enable = 'on';
        handles.ExpDatos.Enable = 'on';
        handles.ExpImagen.Enable = 'on';
        handles.Correr.Enable = 'on';
        handles.Parar.Enable = 'on';
% Mantener a la escucha
    handles.EstadoRobot.String = 'Modo 0: Paro';
        BucleRecibirDatosDelRobot(handles);
catch e
        handles.Calibrar.Enable = 'on';
        handles.CongelarImg.Enable = 'off';
        handles.Reiniciar.Enable = 'off';
        handles.ExpDatos.Enable = 'off';
        handles.ExpImagen.Enable = 'off';
        handles.Correr.Enable = 'off';
        handles.Parar.Enable = 'off';
        
    handles.EstadoRobot.String = 'Error en Calibrar_Callback';
    disp(['Error en Calibrar_Callback: ', e.message]);
end

function EnviarPotMax(handles)
if strcmp(handles.figure1.UserData.Bluetooth.Status, 'close')
    handles.EstadoRobot.String = 'Error Bluetooth cerrado';
    return;
end
try
    handles.figure1.UserData.esperandoConfirmacion = true;
    myByte = uint8(handles.figure1.UserData.PotMax + 100);
    fwrite(handles.figure1.UserData.Bluetooth, [254 myByte])
% % Esperar respuesta de que el número ha sido recibido exitosamente
%     tic;
%     while true
%         if handles.figure1.UserData.Bluetooth.BytesAvailable > 0
%             myByte = fread(handles.figure1.UserData.Bluetooth, 1);
%             if myByte == 254
%                 break;
%             end
%         end
%         t = toc;
%         if t >= 5
%             error('Timeout');
%         end
%     end
catch e
    disp(['Error en EnviarPotMax: ', e.message]);
    handles.EstadoRobot.String = 'Error en EnviarPotMax';
end
    handles.figure1.UserData.esperandoConfirmacion = false;
        
function EnviarPotMin(handles)
if strcmp(handles.figure1.UserData.Bluetooth.Status, 'close')
    handles.EstadoRobot.String = 'Error Bluetooth cerrado';
    return;
end
try
    handles.figure1.UserData.esperandoConfirmacion = true;
    myByte = uint8(handles.figure1.UserData.PotMin + 100);
    fwrite(handles.figure1.UserData.Bluetooth, [253 myByte])
% % Esperar respuesta de que el número ha sido recibido exitosamente
%     tic;
%     while true
%         
%         if handles.figure1.UserData.Bluetooth.BytesAvailable > 0
%             myByte = fread(handles.figure1.UserData.Bluetooth, 1);
%             if myByte == 253
%                 break;
%             end
%         end
%         t = toc;
%         if t >= 5
%             error('Timeout');
%         end
%     end
catch e
    disp(['Error en EnviarPotMin: ', e.message]);
    handles.EstadoRobot.String = 'Error en EnviarPotMin';
end
    handles.figure1.UserData.esperandoConfirmacion = false;
        
function EnviarPotRef(handles)
if strcmp(handles.figure1.UserData.Bluetooth.Status, 'close')
    handles.EstadoRobot.String = 'Error Bluetooth cerrado';
    return;
end
try
    handles.figure1.UserData.esperandoConfirmacion = true;
    myByte = uint8(handles.figure1.UserData.PotRef + 100);
    fwrite(handles.figure1.UserData.Bluetooth, [249 myByte])
% % Esperar respuesta de que el número ha sido recibido exitosamente
%    tic;
%     while true
%         if handles.figure1.UserData.Bluetooth.BytesAvailable > 0
%             myByte = fread(handles.figure1.UserData.Bluetooth, 1);
%             if myByte == 249
%                 break;
%             end
%         end
%         t = toc;
%         if t >= 5
%             error('Timeout');
%         end
%     end
catch e
    disp(['Error en EnviarPotRef: ', e.message]);
    handles.EstadoRobot.String = 'Error en EnviarPotRef';
end
    handles.figure1.UserData.esperandoConfirmacion = false;

function EnviarK(handles)
if strcmp(handles.figure1.UserData.Bluetooth.Status, 'close')
    handles.EstadoRobot.String = 'Error Bluetooth cerrado';
    return;
end
try
    handles.figure1.UserData.esperandoConfirmacion = true;
    fwrite(handles.figure1.UserData.Bluetooth, 252)
%     fwrite(handles.figure1.UserData.Bluetooth, handles.figure1.UserData.K, 'single')
    myInt = uint32(handles.figure1.UserData.K * 1000);
    Y = typecast(myInt, 'uint8');
    pause(0.01);
    fwrite(handles.figure1.UserData.Bluetooth, Y(1))
%     pause(0.01);
    fwrite(handles.figure1.UserData.Bluetooth, Y(2))
%     pause(0.01);
    fwrite(handles.figure1.UserData.Bluetooth, Y(3))
%     pause(0.01);
    fwrite(handles.figure1.UserData.Bluetooth, Y(4))
% % Esperar respuesta de que el número ha sido recibido exitosamente
%     tic;
%     while true
%         if handles.figure1.UserData.Bluetooth.BytesAvailable > 0
%             myByte = fread(handles.figure1.UserData.Bluetooth, 1);
%             if myByte == 252
%                 break;
%             end
%         end
%         t = toc;
%         if t >= 5
%             error('Timeout');
%         end
%     end
%     
catch e
    disp(['Error en EnviarK: ', e.message]);
    handles.EstadoRobot.String = 'Error en EnviarK';
end
    handles.figure1.UserData.esperandoConfirmacion = false;

function EnviarTi(handles)
if strcmp(handles.figure1.UserData.Bluetooth.Status, 'close')
    handles.EstadoRobot.String = 'Error Bluetooth cerrado';
    return;
end
try
    handles.figure1.UserData.esperandoConfirmacion = true;
    fwrite(handles.figure1.UserData.Bluetooth, 251)
%     fwrite(handles.figure1.UserData.Bluetooth, handles.figure1.UserData.Ti, 'single')
    myInt = uint32(handles.figure1.UserData.Ti * 1000);
    Y = typecast(myInt, 'uint8');
    pause(0.01);
    fwrite(handles.figure1.UserData.Bluetooth, Y(1))
%     pause(0.01);
    fwrite(handles.figure1.UserData.Bluetooth, Y(2))
%     pause(0.01);
    fwrite(handles.figure1.UserData.Bluetooth, Y(3))
%     pause(0.01);
    fwrite(handles.figure1.UserData.Bluetooth, Y(4))
    
    
% % Esperar respuesta de que el número ha sido recibido exitosamente
%     tic;
%     while true
%         
%         if handles.figure1.UserData.Bluetooth.BytesAvailable > 0
%             myByte = fread(handles.figure1.UserData.Bluetooth, 1);
%             if myByte == 251
%                 break;
%             end
%         end
%         t = toc;
%         if t >= 5
%             error('Timeout');
%         end
%     end
catch e
    disp(['Error en EnviarTi: ', e.message]);
    handles.EstadoRobot.String = 'Error en EnviarTi';
end
    handles.figure1.UserData.esperandoConfirmacion = false;

function EnviarTd(handles)
try
    handles.figure1.UserData.esperandoConfirmacion = true;
    fwrite(handles.figure1.UserData.Bluetooth, 250)
%     fwrite(handles.figure1.UserData.Bluetooth, handles.figure1.UserData.Td, 'single')
    myInt = uint32(handles.figure1.UserData.Td * 1000);
    Y = typecast(myInt, 'uint8');
    pause(0.01);
    fwrite(handles.figure1.UserData.Bluetooth, Y(1))
%     pause(0.01);
    fwrite(handles.figure1.UserData.Bluetooth, Y(2))
%     pause(0.01);
    fwrite(handles.figure1.UserData.Bluetooth, Y(3))
%     pause(0.01);
    fwrite(handles.figure1.UserData.Bluetooth, Y(4))
% % Esperar respuesta de que el número ha sido recibido exitosamente
%     tic;
%     while true
%         
%         if handles.figure1.UserData.Bluetooth.BytesAvailable > 0
%             myByte = fread(handles.figure1.UserData.Bluetooth, 1);
%             if myByte == 250
%                 break;
%             end
%         end
%         t = toc;
%         if t >= 5
%             error('Timeout');
%         end
%     end
catch e
    disp(['Error en EnviarTd: ', e.message]);
    handles.EstadoRobot.String = 'Error en EnviarTd';
end
    handles.figure1.UserData.esperandoConfirmacion = false;
    
    
function EnviarT(handles)
try
    handles.figure1.UserData.esperandoConfirmacion = true;
    fwrite(handles.figure1.UserData.Bluetooth, 243)
    myInt = uint32(handles.figure1.UserData.T * 100);
    fwrite(handles.figure1.UserData.Bluetooth, uint8(myInt))
catch e
    disp(['Error en EnviarT: ', e.message]);
    handles.EstadoRobot.String = 'Error en EnviarT';
end
    handles.figure1.UserData.esperandoConfirmacion = false;
    
    
    
% --- Executes on button press in Correr.
function Correr_Callback(hObject, eventdata, handles)
% hObject    handle to Correr (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of Correr
switch handles.PanelTipoDeControl.SelectedObject.String
    case 'P'
        Modo_1(handles);
    case 'PI'
        Modo_2(handles);
    case 'PD'
        Modo_3(handles);
    case 'PID'
        Modo_4(handles);
end

% --- Executes on button press in Detener.
function Detener_Callback(hObject, eventdata, handles)
% hObject    handle to Detener (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --- Executes on button press in CongelarImg.
function CongelarImg_Callback(hObject, eventdata, handles)
% hObject    handle to CongelarImg (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of CongelarImg
if handles.CongelarImg.Value == 1
    handles.figure1.UserData.ActualizarAxes  = false;
else
    handles.figure1.UserData.ActualizarAxes  = true;
end

% --- Executes on button press in Reiniciar.
function Reiniciar_Callback(hObject, eventdata, handles)
% hObject    handle to Reiniciar (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
try
% Enviar código
    fwrite(handles.figure1.UserData.Bluetooth, [255 0])

% Esperar respuesta de que el número ha sido recibido exitosamente
    tic
    while true
        
        if handles.figure1.UserData.Bluetooth.BytesAvailable > 0
            myByte = fread(handles.figure1.UserData.Bluetooth, 1);
            if myByte == 255
                break;
            end
        end
        t = toc;
        if t >= 10
            error('Timeout');
        end
    end
    % Continuar con el reinicio
    ReiniciarVariablesDeControl(handles);
    handles.figure1.UserData.Reiniciando = true;
    handles.Calibrar.Enable = 'on';
    handles.CongelarImg.Enable = 'off';
    handles.Reiniciar.Enable = 'off';
    handles.ExpDatos.Enable = 'off';
    handles.ExpImagen.Enable = 'off';
    handles.Correr.Enable = 'off';
    handles.Parar.Enable = 'off';
catch e 
    disp(['Error en Reiniciar_Callback: ', e.message]);
    handles.EstadoRobot.String = 'Error en Reiniciar_Callback';
    % Continuar con el reinicio
    ReiniciarVariablesDeControl(handles);
    handles.Calibrar.Enable = 'on';
    handles.CongelarImg.Enable = 'off';
    handles.Reiniciar.Enable = 'off';
    handles.ExpDatos.Enable = 'off';
    handles.ExpImagen.Enable = 'off';
    handles.Correr.Enable = 'off';
    handles.Parar.Enable = 'off';
end


% --- Executes on button press in ExpDatos.
function ExpDatos_Callback(hObject, eventdata, handles)
% hObject    handle to ExpDatos (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

try 
    handles.figure1.UserData.nFilesDat = handles.figure1.UserData.nFilesDat + 1;
    filename = ['Datos\CarlosV_', num2str(handles.figure1.UserData.nFilesDat), '.mat'];
    time = handles.figure1.UserData.h.sensor.XData';
    linePos = handles.figure1.UserData.h.sensor.YData';
    motorLeft = handles.figure1.UserData.h.mIzq.YData';
    motorRight = handles.figure1.UserData.h.mDer.YData';
    save(filename,'time','linePos','motor*');
catch e
    handles.figure1.UserData.nFilesDat = handles.figure1.UserData.nFilesDat - 1;
    disp(['Error en ExpDatos_Callback: ', e.message]);
    handles.EstadoRobot.String = 'Error en ExpDatos_Callback';
end

% --- Executes on button press in ExpImagen.
function ExpImagen_Callback(hObject, eventdata, handles)
% hObject    handle to ExpImagen (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

try 
    handles.figure1.UserData.nFilesImg = handles.figure1.UserData.nFilesImg + 1;
    filename = ['Imágenes\CarlosV_', num2str(handles.figure1.UserData.nFilesImg), '.jpg'];
    saveas(handles.figure1,filename)
catch e
    handles.figure1.UserData.nFilesDat = handles.figure1.UserData.nFilesDat - 1;
    disp(['Error en ExpImagen_Callback: ', e.message]);
    handles.EstadoRobot.String = 'Error en ExpImagen_Callback';
end

% --- Executes on button press in Parar.
function Parar_Callback(hObject, eventdata, handles)
% hObject    handle to Parar (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

Modo_0(handles);

% --- Executes during object deletion, before destroying properties.
function figure1_DeleteFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
try
% Enviar código de reinicio
    fwrite(handles.figure1.UserData.Bluetooth, [255 0])

% % Esperar respuesta de que el número ha sido recibido exitosamente
%     i = 0;
%     while true
%         if handles.figure1.UserData.Bluetooth.BytesAvailable > 0
%             myByte = fread(handles.figure1.UserData.Bluetooth, 1);
%             if myByte == 255
%                 break;
%             end
%         end
%         pause(10e-3); % 10 ms
%         i = i + 1;
%         if i >= 500
%             error('Timeout');
%         end
%     end
    % Continuar con el reinicio
    ReiniciarVariablesDeControl(handles);
    handles.figure1.UserData.Reiniciando = true;
     fclose(handles.figure1.UserData.Bluetooth);
catch e 
    disp(['Error en figure1_DeleteFcn: ', e.message]);
    handles.EstadoRobot.String = 'Error en figure1_DeleteFcn';
    fclose(handles.figure1.UserData.Bluetooth);
end




function PotMin_Callback(hObject, eventdata, handles)
% hObject    handle to PotMin (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of PotMin as text
%        str2double(get(hObject,'String')) returns contents of PotMin as a double
try 
    myString = get(hObject,'String');
    numStr = '';
    for i= 1:length(myString)
        myChar = myString(i);
        if myChar == '-' || (myChar >= 48 && myChar <= 57)
            numStr = [numStr, myChar];
        else
            break;
        end
    end
    newPotMin= str2double(numStr) ;
    if newPotMin > handles.figure1.UserData.PotMax
        error('PotMin debe ser menor que PotMax');
    end
    if newPotMin > 80
        error('PotMin debe ser menor que 80');
    end
    if newPotMin < -100
        error('PotMin debe ser mayor que -100');
    end
    if newPotMin ~= handles.figure1.UserData.PotMin
        % Cambio normal de PotMin
        handles.figure1.UserData.PotMin = newPotMin;
        yyaxis (handles.AxSensor, 'right');
        ylim([handles.figure1.UserData.PotMin handles.figure1.UserData.PotMax]);
        EnviarPotMin(handles);
    end
    handles.PotMin.String = [num2str(handles.figure1.UserData.PotMin), ' %'];
catch e
    disp(['Error en PotMin_Callback: ', e.message]);
    handles.PotMin.String = [num2str(handles.figure1.UserData.PotMin), ' %'];
    return;
end

% --- Executes during object creation, after setting all properties.
function PotMin_CreateFcn(hObject, eventdata, handles)
% hObject    handle to PotMin (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function BucleRecibirDatosDelRobot(handles)
try
    n = 0;
    n_ciclos = 0;
    sensor = 0;
    motorIzq = 0;
    motorDer = 0;
    handles.figure1.UserData.bytesRecibidos = 0;
    
    timerVal = tic;
    while true
        % Señal de ruptura del bucle
        if handles.figure1.UserData.Reiniciando
            handles.EstadoRobot.String = 'Reiniciando...';
            pause(0.001);
            % Vaciar buffer del bluetooth
                while handles.figure1.UserData.Bluetooth.BytesAvailable > 0
                        myByte = fread(handles.figure1.UserData.Bluetooth, 1);
                end
                handles.figure1.UserData.Reiniciando = false;
                
                if strcmp(handles.figure1.UserData.Bluetooth.Status, 'open')
                    handles.EstadoRobot.String = 'Bluetooth abierto';
                else
                    handles.EstadoRobot.String = 'Bluetooth cerrado';
                end
                 pause(0.001);
                return;
        end
%         % Otra instrucción está leyendo los bytes
%         if  handles.figure1.UserData.esperandoConfirmacion
%             drawnow();
%             continue;
%         end
        % Hay algo en el buffer
        if handles.figure1.UserData.Bluetooth.BytesAvailable > 0
            myByte = fread(handles.figure1.UserData.Bluetooth, 1);
            handles.figure1.UserData.bytesRecibidos = handles.figure1.UserData.bytesRecibidos + 1;
            if myByte >= 201
                n = 0;
            else
                n = n + 1;
            end
            switch n
                case 1
                    sensor = myByte - 100;
                case 2
                    motorIzq = myByte -100;
                case 3
                    motorDer= myByte -100;
                    n_ciclos = n_ciclos + 1;
                    if handles.figure1.UserData.ActualizarAxes
                        handles.figure1.UserData.h.sensor.YData(1:end-1) = handles.figure1.UserData.h.sensor.YData(2:end);
                        handles.figure1.UserData.h.sensor.YData(end) = sensor;
                        handles.figure1.UserData.h.mIzq.YData(1:end-1) = handles.figure1.UserData.h.mIzq.YData(2:end);
                        handles.figure1.UserData.h.mIzq.YData(end) = motorIzq;
                        handles.figure1.UserData.h.mDer.YData(1:end-1) = handles.figure1.UserData.h.mDer.YData(2:end);
                        handles.figure1.UserData.h.mDer.YData(end) = motorDer;
                    end
                    if n_ciclos == 10
                        drawnow();
                        n_ciclos = 0;
                        timerVal = tic;
                    end
            end

        else
            % No ha habido ningún byte que recibir
            t = toc(timerVal);
            if (t >= 5)
                    timerVal = tic;
                    drawnow();
            end
        end
        
    end
catch e
    disp(['Error en BucleRecibirDatosDelRobot: ', e.message]);
    handles.EstadoRobot.String = 'Error en BucleRecibirDatosDelRobot';
end


function Modo_0(handles)
if strcmp(handles.figure1.UserData.Bluetooth.Status, 'close')
    handles.EstadoRobot.String = 'Error Bluetooth cerrado';
    return;
end
try
    handles.figure1.UserData.esperandoConfirmacion = true;
    fwrite(handles.figure1.UserData.Bluetooth, 248)
% % Esperar respuesta de que el número ha sido recibido exitosamente
%     tic;
%     while true
%         if handles.figure1.UserData.Bluetooth.BytesAvailable > 0
%             myByte = fread(handles.figure1.UserData.Bluetooth, 1);
%             if myByte == 248
%                 break;
%             end
%         end
%         t = toc;
%         if t >= 5
%             error('Timeout');
%         end
%     end
    handles.EstadoRobot.String = 'Modo 0: Paro';
catch e
    disp(['Error en Modo_0: ', e.message]);
    handles.EstadoRobot.String = 'Error en Modo_0';
end
handles.figure1.UserData.esperandoConfirmacion = false;

function Modo_1(handles)
if strcmp(handles.figure1.UserData.Bluetooth.Status, 'close')
    handles.EstadoRobot.String = 'Error Bluetooth cerrado';
    return;
end
try
    handles.figure1.UserData.esperandoConfirmacion = true;
    fwrite(handles.figure1.UserData.Bluetooth, 247)
% % Esperar respuesta de que el número ha sido recibido exitosamente
%     tic;
%     while true
%         if handles.figure1.UserData.Bluetooth.BytesAvailable > 0
%             myByte = fread(handles.figure1.UserData.Bluetooth, 1);
%             if myByte == 247
%                 break;
%             end
%         end
%         t = toc;
%         if t >= 5
%             error('Timeout');
%         end
%     end
    handles.EstadoRobot.String = 'Modo 1: P';
catch e
    disp(['Error en Modo_1: ', e.message]);
    handles.EstadoRobot.String = 'Error en Modo_1';
end
    handles.figure1.UserData.esperandoConfirmacion = false;
    
    

function Modo_2(handles)
if strcmp(handles.figure1.UserData.Bluetooth.Status, 'close')
    handles.EstadoRobot.String = 'Error Bluetooth cerrado';
    return;
end
try
    handles.figure1.UserData.esperandoConfirmacion = true;
    fwrite(handles.figure1.UserData.Bluetooth, 246)
% % Esperar respuesta de que el número ha sido recibido exitosamente
%     tic;
%     while true
%         if handles.figure1.UserData.Bluetooth.BytesAvailable > 0
%             myByte = fread(handles.figure1.UserData.Bluetooth, 1);
%             if myByte == 246
%                 break;
%             end
%         end
%         t = toc;
%         if t >= 5
%             error('Timeout');
%         end
%     end
    handles.EstadoRobot.String = 'Modo 2: PI';
catch e
    disp(['Error en Modo_2: ', e.message]);
    handles.EstadoRobot.String = 'Error en Modo_2';
end
    handles.figure1.UserData.esperandoConfirmacion = false;
    

function Modo_3(handles)
if strcmp(handles.figure1.UserData.Bluetooth.Status, 'close')
    handles.EstadoRobot.String = 'Error Bluetooth cerrado';
    return;
end
try
    handles.figure1.UserData.esperandoConfirmacion = true;
    fwrite(handles.figure1.UserData.Bluetooth, 245)
% % Esperar respuesta de que el número ha sido recibido exitosamente
%     tic;
%     while true
%         if handles.figure1.UserData.Bluetooth.BytesAvailable > 0
%             myByte = fread(handles.figure1.UserData.Bluetooth, 1);
%             if myByte == 245
%                 break;
%             end
%         end
%         t = toc;
%         if t >= 5
%             error('Timeout');
%         end
%     end
    handles.EstadoRobot.String = 'Modo 3: PD';
catch e
    disp(['Error en Modo_3: ', e.message]);
    handles.EstadoRobot.String = 'Error en Modo_3';
end
    handles.figure1.UserData.esperandoConfirmacion = false;
    
    
function Modo_4(handles)
if strcmp(handles.figure1.UserData.Bluetooth.Status, 'close')
    handles.EstadoRobot.String = 'Error Bluetooth cerrado';
    return;
end
try
    handles.figure1.UserData.esperandoConfirmacion = true;
    fwrite(handles.figure1.UserData.Bluetooth, 244)
% % Esperar respuesta de que el número ha sido recibido exitosamente
%     tic;
%     while true
%         if handles.figure1.UserData.Bluetooth.BytesAvailable > 0
%             myByte = fread(handles.figure1.UserData.Bluetooth, 1);
%             if myByte == 244
%                 break;
%             end
%         end
%         t = toc;
%         if t >= 5
%             error('Timeout');
%         end
%     end
    handles.EstadoRobot.String = 'Modo 4: PID';
catch e
    disp(['Error en Modo_4: ', e.message]);
    handles.EstadoRobot.String = 'Error en Modo_4';
end
    handles.figure1.UserData.esperandoConfirmacion = false;



function PotRef_Callback(hObject, eventdata, handles)
% hObject    handle to PotRef (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of PotRef as text
%        str2double(get(hObject,'String')) returns contents of PotRef as a double
try 
    myString = get(hObject,'String');
    numStr = '';
    for i= 1:length(myString)
        myChar = myString(i);
        if myChar == '-' || (myChar >= 48 && myChar <= 57)
            numStr = [numStr, myChar];
        else
            break;
        end
    end
    newPotRef= str2double(numStr) ;
    if newPotRef > handles.figure1.UserData.PotMax
        error('PotRef debe ser menor o igual que PotMax');
    end
    if newPotRef < handles.figure1.UserData.PotMin
        error('PotRef debe ser mayor o igual que PotMin');
    end
    if newPotRef ~= handles.figure1.UserData.PotRef
        % Cambio normal de PotRef
        handles.figure1.UserData.PotRef = newPotRef;
        EnviarPotRef(handles);
    end
    handles.PotRef.String = [num2str(handles.figure1.UserData.PotRef), ' %'];
catch e
    disp(['Error en PotRef_Callback: ', e.message]);
    handles.PotRef.String = [num2str(handles.figure1.UserData.PotRef), ' %'];
    return;
end


% --- Executes during object creation, after setting all properties.
function PotRef_CreateFcn(hObject, eventdata, handles)
% hObject    handle to PotRef (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes when selected object is changed in PanelTipoDeControl.
function PanelTipoDeControl_SelectionChangedFcn(hObject, eventdata, handles)
% hObject    handle to the selected object in PanelTipoDeControl 
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

switch handles.PanelTipoDeControl.SelectedObject.String
    case 'P'
        Modo_1(handles);
    case 'PI'
        Modo_2(handles);
    case 'PD'
        Modo_3(handles);
    case 'PID'
        Modo_4(handles);
end



function T_Callback(hObject, eventdata, handles)
% hObject    handle to T (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of T as text
%        str2double(get(hObject,'String')) returns contents of T as a double
try 
    myString = get(hObject,'String');
    newVal = str2double(myString);
    if newVal > 2
        error('T debe ser menor o igual que 2');
    end
    if newVal < 0.01
        error('T debe ser mayor o igual que 0.01');
    end
    if newVal ~= handles.figure1.UserData.T
        % Cambio normal de Td
        handles.figure1.UserData.T = newVal;
        EnviarT(handles);
    end
    handles.T.String = num2str(handles.figure1.UserData.T);
catch e
    disp(['Error en T_Callback: ', e.message]);
    handles.T.String = num2str(handles.figure1.UserData.T);
    return;
end


% --- Executes during object creation, after setting all properties.
function T_CreateFcn(hObject, eventdata, handles)
% hObject    handle to T (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on key press with focus on figure1 and none of its controls.
function figure1_KeyPressFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  structure with the following fields (see MATLAB.UI.FIGURE)
%	Key: name of the key that was pressed, in lower case
%	Character: character interpretation of the key(s) that was pressed
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)
