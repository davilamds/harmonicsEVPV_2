tic; clear; clc; close all;clear all;

%%Configuración
steps=144;
case_simulation=6;

enable_hosting=0;

if enable_hosting==1;
    enable_pvsystems=3; %0 disabled 1 enabled (dist) 2 enabled (single) 3 enabled (dist loads)
    enable_vehicles=2; %0 disabled 1 enabled (dist) 2 enabled (dist loads)
    enable_harmonics=1;
    EV_index=0;   %2
    PV_index=5;%13.5
end

if case_simulation==1
enable_pvsystems=0; %0 disabled 1 enabled (dist) 2 enabled (single) 3 enabled (dist loads)
enable_vehicles=0; %0 disabled 1 enabled (dist) 2 enabled (dist loads)
enable_harmonics=0;
EV_index=0;   %2
PV_index=0;%13.5
end

if case_simulation==2
enable_pvsystems=0; %0 disabled 1 enabled (dist) 2 enabled (single) 3 enabled (dist loads)
enable_vehicles=1; %0 disabled 1 enabled (dist) 2 enabled (dist loads)
enable_harmonics=0;
EV_index=1;   %2
PV_index=0;%13.5
end

if case_simulation==3
enable_pvsystems=1; %0 disabled 1 enabled (dist) 2 enabled (single) 3 enabled (dist loads)
enable_vehicles=1; %0 disabled 1 enabled (dist) 2 enabled (dist loads)
enable_harmonics=0;
EV_index=1;   %2
PV_index=5;%13.5
end

if case_simulation==4
enable_pvsystems=2; %0 disabled 1 enabled (dist) 2 enabled (single) 3 enabled (dist loads)
enable_vehicles=1; %0 disabled 1 enabled (dist) 2 enabled (dist loads)
enable_harmonics=0;
EV_index=1;   %2
PV_index=70;%13.5
end

if case_simulation==5
enable_pvsystems=3; %0 disabled 1 enabled (dist) 2 enabled (single) 3 enabled (dist loads)
enable_vehicles=2; %0 disabled 1 enabled (dist) 2 enabled (dist loads)
enable_harmonics=0;
EV_index=1;   %2
PV_index=5;%13.5
end

if case_simulation==6
enable_pvsystems=0; %0 disabled 1 enabled (dist) 2 enabled (single) 3 enabled (dist loads)
enable_vehicles=1; %0 disabled 1 enabled (dist) 2 enabled (dist loads)
enable_harmonics=1;
EV_index=1;   %2
PV_index=0;%13.5
end

if case_simulation==7
enable_pvsystems=1; %0 disabled 1 enabled (dist) 2 enabled (single) 3 enabled (dist loads)
enable_vehicles=1; %0 disabled 1 enabled (dist) 2 enabled (dist loads)
enable_harmonics=1;
EV_index=1;   %2
PV_index=5;%13.5
end

if case_simulation==8
enable_pvsystems=2; %0 disabled 1 enabled (dist) 2 enabled (single) 3 enabled (dist loads)
enable_vehicles=1; %0 disabled 1 enabled (dist) 2 enabled (dist loads)
enable_harmonics=1;
EV_index=1;   %2
PV_index=70;%13.5
end

if case_simulation==9
enable_pvsystems=3; %0 disabled 1 enabled (dist) 2 enabled (single) 3 enabled (dist loads)
enable_vehicles=2; %0 disabled 1 enabled (dist) 2 enabled (dist loads)
enable_harmonics=1;
EV_index=11;   %2 y 19
PV_index=5;%13.5
end
%Graficas
graph_bus_overvoltage_by_step=0; %este
enable_graphs=1;

%limits
voltage_variation_max=0.1;
voltage_variation_min=0.1; %0.06
Vloadbase=240;
v_max=Vloadbase*voltage_variation_max;
v_min=Vloadbase*voltage_variation_min;
V_THD=8; %percentage
V_3_VH=5; %voltage 3rd harmonic
V_5_VH=5; %voltage 5rd harmonic
V_7_VH=5; %voltage 7rd harmonic
V_9_VH=5; %voltage 9rd harmonic
V_11_VH=5; %voltage 11rd harmonic


%% **********************************************************************************************************
% Basic Definitions & Input Data

input.mydir = pwd; %>>>>>>Set Directoty
input.input_path = [input.mydir '\Input_data' ]; %>>>>>>Path of the folder where all input data are
input.name_circuit = 'circuit'; %>>>>>>Specify the name of the circuit to assess


if steps==144
    minutos=10;
end
if steps==1440
    minutos=1;
end
if steps==24
    minutos=60;
end
input.time_steps = steps; %>>>>>>Steps of the daily simulation (1 day = 1440 steps of 1min)
TIEMPO = 1/(steps/24):1/(steps/24):24;
%% Start up the DSS
[DSSCircObj, DSSText, gridpvPath] = DSSStartup;
DSSText.Command = 'clear';                                      %Clear previous session
DSSText.Command = ['Compile "' input.input_path '\' input.name_circuit '\Master.dss"'];
DSSText.command = 'solve';
warnSt = circuitCheck(DSSCircObj);
DSSCircuit = DSSCircObj.ActiveCircuit; %>>>>>>Set up the Circuit
DSSSolution = DSSCircuit.Solution; %>>>>>>Set up the Solution
ControlQueue = DSSCircuit.CtrlQueue; %>>>>>>Set up the Control
DSSLoad     = DSSCircuit.Loads;
DSSLines = DSSCircuit.Lines;
DSSActiveCktElement = DSSCircObj.ActiveCircuit.ActiveCktElement;
DSSCircObj.AllowForms = false; %>>>>>>Avoids getting OpenDSS messages
DSSMonitors=DSSCircuit.Monitors;



%% Place Monitors in every Line
line_data = importdata([input.input_path '\' input.name_circuit '\Lines.txt']);
for i_temp = 1:size(line_data,1)
    var_temp = line_data{i_temp,1};
    var_temp = strsplit(var_temp,' ');
    var_temp1 = strsplit(var_temp{1,2},'.');
    DSSText.Command = ['new monitor.' var_temp1{1,2} ' element=' var_temp{1,2} ' terminal=1 mode=0 ppolar=no'];
end
clear var_temp var_temp1 i_temp 

%% Place Monitors in every Tranformer
trafo_data = importdata([input.input_path '\' input.name_circuit '\Transformers.txt']);
i_temp = 1:size(trafo_data,1);
var_temp = trafo_data{i_temp,1};
var_temp = strsplit(var_temp,' '); %-------cambiar split por strsplit-----
var_temp1 = strsplit(var_temp{1,2},'.');
DSSText.Command = ['new monitor.' var_temp1{1,2} ' element=' var_temp{1,2} ' terminal=1 mode=1 ppolar=no'];
clear var_temp var_temp1 i_temp trafo_data

%% Place Monitors in every Load
line_data = importdata([input.input_path '\' input.name_circuit '\Loads.txt']);
for i_temp = 1:size(line_data,1)
    var_temp = line_data{i_temp,1};
    var_temp = strsplit(var_temp,' ');
    var_temp1 = strsplit(var_temp{1,2},'.');
    DSSText.Command = ['new monitor.' var_temp1{1,2} ' element=' var_temp{1,2} ' terminal=1 mode=0 ppolar=no'];
end
clear var_temp var_temp1 i_temp 

%tensiones en puntos de medicion de las cargas
DSSText.Command = 'new monitor.linea33 element=line.line33 terminal=1 mode=0';
DSSText.Command = 'new energymeter.line33 element=line.line33 terminal=1';
 
clear var_temp var_temp1 i_temp
%% *********************************************************************************************************

% SIMULATION CONTROL

if enable_vehicles==1
    DSSText.Command = 'Redirect vehicles.txt';
end

if enable_vehicles==2
    DSSText.Command = 'Redirect vehicles2.txt';
end

if enable_pvsystems==1
    DSSText.Command = 'Redirect Photovoltaic.txt';
end

if enable_pvsystems==2
    DSSText.Command = 'Redirect Photovoltaic2.txt';
end

if enable_pvsystems==3
    DSSText.Command = 'Redirect Photovoltaic3.txt';
end

%is_overcurrent=-1;
%while is_overcurrent<=0
%    EV_index=EV_index+1

if enable_vehicles>0;
    DSSActiveClass=DSSCircuit.ActiveClass;
    DSSCircuit.SetActiveClass('Storage');
    AllStorageNames=DSSActiveClass.AllNames;
    DSSCircuit.ActiveClass.First;
    for nevs=1:size(AllStorageNames)
        DSSCircuit.ActiveCktElement.Name;
        DSSCircuit.ActiveCktElement.Properties('kW').Val=num2str(EV_index);
        DSSCircuit.ActiveCktElement.Properties('kWrated').Val=num2str(EV_index);
        DSSCircuit.ActiveClass.Next;
    end
end

if enable_pvsystems>0;
    DSSActiveClass=DSSCircuit.ActiveClass;
    DSSCircuit.SetActiveClass('PVSystem');
    AllStorageNames=DSSActiveClass.AllNames;
    DSSCircuit.ActiveClass.First;
    for nevs=1:size(AllStorageNames)
        DSSCircuit.ActiveCktElement.Name;
        DSSCircuit.ActiveCktElement.Properties('kVA').Val=num2str(PV_index);
        DSSCircuit.ActiveCktElement.Properties('Pmpp').Val=num2str(PV_index);
        DSSCircuit.ActiveClass.Next;
    end
end



voltaje50=zeros(steps,1);
voltaje150=zeros(steps,1);
voltaje250=zeros(steps,1);
voltaje350=zeros(steps,1);
voltaje450=zeros(steps,1);
voltaje550=zeros(steps,1);
VthdPercent=zeros(steps,1);

corriente50=zeros(steps,1);
corriente150=zeros(steps,1);
corriente250=zeros(steps,1);
corriente350=zeros(steps,1);
corriente450=zeros(steps,1);
corriente550=zeros(steps,1);
IthdPercent=zeros(steps,1);

potencia_activa1=zeros(steps,1);
potencia_reactiva1=zeros(steps,1);
potencia_activa2=zeros(steps,1);
potencia_reactiva2=zeros(steps,1);
potencia_activa3=zeros(steps,1);
potencia_reactiva3=zeros(steps,1);

circuitlosses=zeros(steps,1);
linelosses=zeros(steps,1);
elementlosses=zeros(steps,1);
perdidasharmonics=zeros(steps,1);

circuitlossesh=zeros(steps,1);
linelossesh=zeros(steps,1);
elementlossesh=zeros(steps,1);
perdidasharmonicsh=zeros(steps,1);

V1 = zeros(steps,1);
Dist1 = zeros(steps,1);
V2 = zeros(steps,1);
Dist2 = zeros(steps,1);
V3 = zeros(steps,1);
Dist3 = zeros(steps,1);

V1h = zeros(steps,2721);
Dist1h = zeros(steps,1);
V2h = zeros(steps,1);
Dist2h = zeros(steps,1);
V3h = zeros(steps,1);
Dist3h = zeros(steps,1);


DSSText.Command = 'Set ControlMode = snapshot'; %>>>>>>Defines the control mode
DSSText.Command = 'Reset'; %>>>>>>Resets all energy meters and monitors
DSSText.Command = 'Solve'; %>>>>>>Defines the control mode

%%lineas
lineas=DSSCircuit.Lines.AllNames;
DSSCircuit.Lines.First;
for con=1:size(lineas);
    ttt(con,1)=DSSCircuit.Lines.NormAmps;
    DSSCircuit.Lines.Next;
end
%%buses
Name_buses=DSSCircuit.AllBusNames;
for con=1:size(Name_buses);
    DSSCircuit.SetActiveBusi(con);
    bttt(con,1)=DSSCircuit.ActiveBus.kVBase/DSSCircuit.ActiveBus.kVBase;
end
%loads
Name_loads=DSSLoad.AllNames;
for con=1:size(Name_loads);
    loadbttt(con,1)=(DSSLoad.kV/DSSLoad.kV)*Vloadbase;
end


L= readmatrix('Lineas.csv');
buscoordenates=readmatrix('Buscoords.csv');
test=[L ttt];

    
if enable_harmonics==0
    for time_simulation = 1:input.time_steps; %>>>>>>Starts time-series power flow
        DSSText.Command = 'Set ControlMode = time';
        t = sprintf('Set Mode=daily stepsize =%f' ,minutos);
        t2 = sprintf('m number=%f',time_simulation);
        DSSText.Command = [t t2];
        DSSSolution.Solve; %>>>>>>Solves power flow

        
        voltaje_cliente_1 = ExtractMonitorData(DSSCircuit,'load20');
        voltaje(time_simulation,1)=voltaje_cliente_1(time_simulation,3);
        corriente(time_simulation,1)=voltaje_cliente_1(time_simulation,7);
        corriente_linea33=ExtractMonitorData(DSSCircuit,'linea33');
                
         %PERDIDAS SIN ARMONICOS
        circuitlosses(time_simulation,1:2)=DSSCircuit.Losses;
        linelosses(time_simulation,1:2)=DSSCircuit.LineLosses;
        elementlossesnames=DSSCircuit.AllElementNames;
        elementlosses(time_simulation,1:size(elementlossesnames)*2)=DSSCircuit.AllElementLosses;
        
%%%%%%%%lines
        LMon = DSSLines.AllNames;
        DSSLines.First;
        for LMonc=1:size(LMon);
            strName=DSSLines.Name;
            line_monitor_data=ExtractMonitorData(DSSCircuit,strName);
            line_voltages50_1(time_simulation,LMonc,1)=line_monitor_data(time_simulation,3);     
            line_voltages50_2(time_simulation,LMonc,1)=line_monitor_data(time_simulation,5);
            line_voltages50_3(time_simulation,LMonc,1)=line_monitor_data(time_simulation,7);
            line_currents50_1(time_simulation,LMonc,1)=line_monitor_data(time_simulation,9);            
            line_currents50_2(time_simulation,LMonc,1)=line_monitor_data(time_simulation,11);           
            line_currents50_3(time_simulation,LMonc,1)=line_monitor_data(time_simulation,13);
            DSSLines.Next;
        end
        

        Currents_DSS_Lines_1=line_currents50_1;
        Currents_DSS_Lines_2=line_currents50_2;
        Currents_DSS_Lines_3=line_currents50_3;
        
        %ttt2=[ttt Currents_DSS_Lines_1' Currents_DSS_Lines_2' Currents_DSS_Lines_3'];
        
        ttt2(:,1)=ttt;
        ttt2(:,2)=Currents_DSS_Lines_1(time_simulation,:)';
        ttt2(:,3)=Currents_DSS_Lines_2(time_simulation,:)';
        ttt2(:,4)=Currents_DSS_Lines_3(time_simulation,:)';
        
        for i=1:size(ttt2);
            if ttt2(i,2)>ttt2(i,1) || ttt2(i,3)>ttt2(i,1) || ttt2(i,4)>ttt2(i,1);
                ttt2(i,5)=1;
                ttt3(i,time_simulation)=1;
            else
                ttt2(i,5)=0;
                ttt3(i,time_simulation)=0; %sobrecorrientes en todo el tiempo
            end
        end
        clear ttt2;
        %%sobrevoltajes
        
        % Get Voltage and Distances Array
        V1(time_simulation,1:907) = DSSCircuit.AllNodeVmagPUByPhase(1);
        Dist1(time_simulation,1:907) = DSSCircuit.AllNodeDistancesByPhase(1);
        V2(time_simulation,1:907) = DSSCircuit.AllNodeVmagPUByPhase(2);
        Dist2(time_simulation,1:907) = DSSCircuit.AllNodeDistancesByPhase(2);
        V3(time_simulation,1:907) = DSSCircuit.AllNodeVmagPUByPhase(3);
        Dist3(time_simulation,1:907) = DSSCircuit.AllNodeDistancesByPhase(3);
        
 %%%%%%%%loads
        iMon = DSSLoad.AllNames;
        DSSLoad.First;
        for iMonc=1:size(iMon);
            strName=DSSLoad.Name;
            load_monitor_data=ExtractMonitorData(DSSCircuit,strName);
            load_voltages50_1(time_simulation,iMonc,1)=load_monitor_data(time_simulation,3);            
            load_voltages50_2(time_simulation,iMonc,1)=load_monitor_data(time_simulation,5);            
            load_currents50_1(time_simulation,iMonc,1)=load_monitor_data(time_simulation,7);            
            load_currents50_2(time_simulation,iMonc,1)=load_monitor_data(time_simulation,9);                   
            DSSLoad.Next;
        end

        
        Voltages_DSS_Loads_1=load_voltages50_1;
        Voltages_DSS_Loads_2=load_voltages50_2;
        
        %bttt2=[loadbttt Voltages_DSS_Loads_1' Voltages_DSS_Loads_2'];
        bttt2(:,1)=loadbttt;
        bttt2(:,2)=Voltages_DSS_Loads_1(time_simulation,:)';
        bttt2(:,3)=Voltages_DSS_Loads_2(time_simulation,:)';
        
        for i=1:size(bttt2);
            if bttt2(i,2)>bttt2(i,1)+v_max;
                bttt2(i,5)=1;
            else
                bttt2(i,5)=0;
            end
            if bttt2(i,2)<bttt2(i,1)-v_min;
                bttt2(i,6)=2;
            else
                bttt2(i,6)=0;
            end
            
            bttt2(i,7)=bttt2(i,5)+bttt2(i,6);
            bttt3(i,time_simulation)=bttt2(i,7); %sobrevoltajes en todo el tiempo
        end
        clear bttt2;

%% conexiones de evs
        if enable_vehicles>0;
            DSSActiveClass=DSSCircuit.ActiveClass;
            DSSCircuit.SetActiveClass('Storage');
            AllStorageNames=DSSActiveClass.AllNames;
            DSSCircuit.ActiveClass.First;
            for nevs=1:size(AllStorageNames)
                estados=string(DSSCircuit.ActiveCktElement.Properties('state').Val);
                if estados=='CHARGING';
                    estados2(nevs,time_simulation)=1;
                elseif estados=='IDLING'
                    estados2(nevs,time_simulation)=0;
                end
                evbusnames(nevs,1)=DSSCircuit.ActiveElement.BusNames;
                evbusnames1(nevs,1)=round(cellfun(@str2num,evbusnames(nevs,1)));
                DSSCircuit.ActiveClass.Next;
            end
        end
    end
    
    Voltages_DSS_Loads_1_max=max(Voltages_DSS_Loads_1,[],2);
    Voltages_DSS_Loads_1_min=min(Voltages_DSS_Loads_1,[],2);
%%Check for violations

is_overcurrent=sum(ttt3,'all');
is_over_sub_voltage=sum(bttt3,'all');
%Potencia de cada fase en el lado de alta del tranformador de
%subestacion
DSSCircuit.Meters.Name = 'TR1';
potencia_lado_alta = ExtractMonitorData(DSSCircuit,'TR1');
    
%Tensiones y corrientes en el Cliente 1
DSSCircuit.Meters.Name = 'load1';
voltaje_cliente_1 = ExtractMonitorData(DSSCircuit,'load1');

%Tensiones y corrientes en el Cliente 2
DSSCircuit.Meters.Name = 'load32';
voltaje_cliente_2 = ExtractMonitorData(DSSCircuit,'load32');

%Tensiones y corrientes en el Cliente 3
DSSCircuit.Meters.Name = 'load53';
voltaje_cliente_3 = ExtractMonitorData(DSSCircuit,'load53');

if enable_graphs==1;
%%%%%%%%%%%%%%%%GRAFICAS SIN ARMONICOS      
    %% GRAFICA DE VOLTAJES 
    figure;
    plot(TIEMPO ,voltaje_cliente_1(:,3));
    hold on
    plot(TIEMPO ,voltaje_cliente_2(:,3), 'b');
    plot(TIEMPO ,voltaje_cliente_3(:,3), 'r');
    title('Load Voltages')
    xlabel('Time [Hour]')
    ylabel('Voltage [V]')
    xlim([0 24]) 
    legend('Load 1','Load 2','Load 3')
    hold off
    XTick = [0:24];
    grid on

%% voltage profile
    figure;
    plot(Dist1, V1,'k*');  % black *
    hold on;
    plot(Dist2, V2, 'r+');  % red +
    plot(Dist3, V3, 'bd');  % diamond Marker
    legend('phase A','phase B','phase C','Location','SouthEast'); %put the legend
    title('Voltage Profile Plot'); %plot title
        
    %ylim([230 280]);
    ylim([0.9 1.1]);
    ylabel('Volts(pu)');
    xlabel('Distance from Substation');
    
    hold off
    
     %% GRAFICA DE POTENCIAS
    PT=potencia_lado_alta(:,3)+potencia_lado_alta(:,5)+potencia_lado_alta(:,7);
    QT=potencia_lado_alta(:,4)+potencia_lado_alta(:,6)+potencia_lado_alta(:,8);
    figure;
    g1 = subplot (2,1,1);
    plot(g1, TIEMPO ,potencia_lado_alta(:,3));
    hold on
    plot(g1, TIEMPO, potencia_lado_alta(:,5), 'b');
    plot(g1, TIEMPO, potencia_lado_alta(:,7), 'r');
    title(g1,'Substation transformer power curve')
    xlabel(g1,'Time [Hour]')
    ylabel(g1,'Power [KW]')
    xlim([0 24]) 
    legend('Phase A','Phase B','Phase C')
    g1.XTick = [0:24];
    grid on
    
    g2 = subplot (2,1,2);
    plot(g2, TIEMPO ,PT);
    hold on
    plot(g2, TIEMPO ,QT);
    title(g2,'Substation transformer power curve (total)')
    xlabel(g2,'Time [Hour]')
    ylabel(g2,'Power [KW]')
    xlim([0 24])
    g2.XTick = [0:24];
    grid on
    hold off
end 
    %%vehiculos
    if enable_vehicles>0;
        for evnames=1:size(evbusnames1,1);
            for evnames2=1:size(buscoordenates,1);
                if evbusnames1(evnames,1)==buscoordenates(evnames2,1);
                    evbusnames1(evnames,2)=buscoordenates(evnames2,2);
                    evbusnames1(evnames,3)=buscoordenates(evnames2,3);
                    break;
                end
            end
        end
        vehiculos=[evbusnames1 estados2];
    end
    
    
    if graph_bus_overvoltage_by_step==1;
        figure;
        for j=1:steps;
            for i=1:size(test);
                if ttt3(i,j)==1;
                    plot([test(i,3) test(i,5)],[test(i,4) test(i,6)],'color','red','LineWidth',5)
                    hold on
                end
                if ttt2(i,5)==0;
                    plot([test(i,3) test(i,5)],[test(i,4) test(i,6)],'color','blue','LineWidth',2)
                    hold on
                end
            end
            for k=1:size(Name_buses)
                if bttt3(k,j)>0;
                    plot(buscoordenates(k,2),buscoordenates(k,3),'.','markersize',18,'color','red')

                end
                if bttt3(k,j)==0;
                    plot(buscoordenates(k,2),buscoordenates(k,3),'.','markersize',18,'color','blue')
                end
            end
            if enable_vehicles>0;
                for evit=1:size(vehiculos,1);
                    if vehiculos(evit,j+3)==1;
                        plot(vehiculos(evit,2), vehiculos(evit,3),'-p','markersize',18,'MarkerFaceColor','red')
                    end
                     if vehiculos(evit,j+3)==0;
                        plot(vehiculos(evit,2), vehiculos(evit,3),'-p','markersize',1,'MarkerFaceColor','yellow')
                    end
                end
            end
            hold off
            title(['time ',datestr((1/(steps/24)*j)/24,'HH:MM')])
            drawnow;     
        end
    end
    
    clear time_simulation g1 g2 g3 g4 g5 g6

end%termina sin armónicos


if enable_harmonics==1
    for time_simulation = 1:input.time_steps; %>>>>>>Starts time-series power flow
        DSSText.Command = 'Set ControlMode = time';
        t = sprintf('Set Mode=daily stepsize =%f' ,minutos);
        t2 = sprintf('m number=%f',time_simulation);
        DSSText.Command = [t t2];
        DSSSolution.Solve; %>>>>>>Solves power flow
        % Get Voltage and Distances Array
        V1(time_simulation,1:907) = DSSCircuit.AllNodeVmagPUByPhase(1);
        Dist1(time_simulation,1:907) = DSSCircuit.AllNodeDistancesByPhase(1);
        V2(time_simulation,1:907) = DSSCircuit.AllNodeVmagPUByPhase(2);
        Dist2(time_simulation,1:907) = DSSCircuit.AllNodeDistancesByPhase(2);
        V3(time_simulation,1:907) = DSSCircuit.AllNodeVmagPUByPhase(3);
        Dist3(time_simulation,1:907) = DSSCircuit.AllNodeDistancesByPhase(3);
        %-------------------------------HARMONICOS---------------------------
        %t = sprintf('solve mode=harmonicsT stepsize =%f' ,minutos);
        %t2 = sprintf('m number=%f',time_simulation);
        %DSSText.Command = [t t2];
        t = sprintf('Solve mode=harmonics')';
        DSSText.Command = t;
        %DSSSolution.Solve; %>>>>>>Solves power flow
        
        % Get Voltage and Distances Array
        V1h(time_simulation,1:907) = DSSCircuit.AllNodeVmagPUByPhase(1);
        Dist1h(time_simulation,1:907) = DSSCircuit.AllBusDistances;
        V2h(time_simulation,1:907) = DSSCircuit.AllNodeVmagPUByPhase(2);
        Dist2h(time_simulation,1:907) = DSSCircuit.AllNodeDistancesByPhase(2);
        V3h(time_simulation,1:907) = DSSCircuit.AllNodeVmagPUByPhase(3);
        Dist3h(time_simulation,1:907) = DSSCircuit.AllNodeDistancesByPhase(3);
        
        potencia_lado_altah = ExtractMonitorData(DSSCircuit,'TR1');
        potencia_activa1h(time_simulation,1)=potencia_lado_altah(1,3);
        potencia_reactiva1h(time_simulation,1)=potencia_lado_altah(1,4);
        potencia_activa2h(time_simulation,1)=potencia_lado_altah(1,5);
        potencia_reactiva2h(time_simulation,1)=potencia_lado_altah(1,6);
        potencia_activa3h(time_simulation,1)=potencia_lado_altah(1,7);
        potencia_reactiva3h(time_simulation,1)=potencia_lado_altah(1,8);
        %if you do control, here it comes
        
        
%%%%%%%%lines
        LMon = DSSLines.AllNames;
        DSSLines.First;
        for LMonc=1:size(LMon);
            strName=DSSLines.Name;
            line_monitor_data=ExtractMonitorData(DSSCircuit,strName);
            line_voltages50_1(time_simulation,LMonc,1)=line_monitor_data(1,3);
            line_voltages150_1(time_simulation,LMonc,1)=line_monitor_data(2,3);
            line_voltages250_1(time_simulation,LMonc,1)=line_monitor_data(3,3);
            line_voltages350_1(time_simulation,LMonc,1)=line_monitor_data(4,3);
            line_voltages450_1(time_simulation,LMonc,1)=line_monitor_data(5,3);
            line_voltages550_1(time_simulation,LMonc,1)=line_monitor_data(6,3);
            line_voltages650_1(time_simulation,LMonc,1)=line_monitor_data(7,3);
            
            line_voltages50_2(time_simulation,LMonc,1)=line_monitor_data(1,5);
            line_voltages150_2(time_simulation,LMonc,1)=line_monitor_data(2,5);
            line_voltages250_2(time_simulation,LMonc,1)=line_monitor_data(3,5);
            line_voltages350_2(time_simulation,LMonc,1)=line_monitor_data(4,5);
            line_voltages450_2(time_simulation,LMonc,1)=line_monitor_data(5,5);
            line_voltages550_2(time_simulation,LMonc,1)=line_monitor_data(6,5);
            line_voltages650_2(time_simulation,LMonc,1)=line_monitor_data(7,5);
            
            line_voltages50_3(time_simulation,LMonc,1)=line_monitor_data(1,7);
            line_voltages150_3(time_simulation,LMonc,1)=line_monitor_data(2,7);
            line_voltages250_3(time_simulation,LMonc,1)=line_monitor_data(3,7);
            line_voltages350_3(time_simulation,LMonc,1)=line_monitor_data(4,7);
            line_voltages450_3(time_simulation,LMonc,1)=line_monitor_data(5,7);
            line_voltages550_3(time_simulation,LMonc,1)=line_monitor_data(6,7);
            line_voltages650_3(time_simulation,LMonc,1)=line_monitor_data(7,7);
            
            line_currents50_1(time_simulation,LMonc,1)=line_monitor_data(1,9);
            line_currents150_1(time_simulation,LMonc,1)=line_monitor_data(2,9);
            line_currents250_1(time_simulation,LMonc,1)=line_monitor_data(3,9);
            line_currents350_1(time_simulation,LMonc,1)=line_monitor_data(4,9);
            line_currents450_1(time_simulation,LMonc,1)=line_monitor_data(5,9);
            line_currents550_1(time_simulation,LMonc,1)=line_monitor_data(6,9);
            line_currents650_1(time_simulation,LMonc,1)=line_monitor_data(7,9);
            
            line_currents50_2(time_simulation,LMonc,1)=line_monitor_data(1,11);
            line_currents150_2(time_simulation,LMonc,1)=line_monitor_data(2,11);
            line_currents250_2(time_simulation,LMonc,1)=line_monitor_data(3,11);
            line_currents350_2(time_simulation,LMonc,1)=line_monitor_data(4,11);
            line_currents450_2(time_simulation,LMonc,1)=line_monitor_data(5,11);
            line_currents550_2(time_simulation,LMonc,1)=line_monitor_data(6,11);
            line_currents650_2(time_simulation,LMonc,1)=line_monitor_data(7,11);
            
            line_currents50_3(time_simulation,LMonc,1)=line_monitor_data(1,13);
            line_currents150_3(time_simulation,LMonc,1)=line_monitor_data(2,13);
            line_currents250_3(time_simulation,LMonc,1)=line_monitor_data(3,13);
            line_currents350_3(time_simulation,LMonc,1)=line_monitor_data(4,13);
            line_currents450_3(time_simulation,LMonc,1)=line_monitor_data(5,13);
            line_currents550_3(time_simulation,LMonc,1)=line_monitor_data(6,13);
            line_currents650_3(time_simulation,LMonc,1)=line_monitor_data(7,13);
                   
            DSSLines.Next;
        end
        
        line_voltage_distortion_rms_1=sqrt(line_voltages150_1.^2+line_voltages250_1.^2+line_voltages350_1.^2+line_voltages450_1.^2+line_voltages550_1.^2+line_voltages650_1.^2);
        line_voltage_rms_1=sqrt(line_voltages50_1.^2+line_voltages150_1.^2+line_voltages250_1.^2+line_voltages350_1.^2+line_voltages450_1.^2+line_voltages550_1.^2+line_voltages650_1.^2);
        line_VTHD_1=(line_voltage_distortion_rms_1./line_voltage_rms_1)*100;

        line_current_distortion_rms_1=sqrt(line_currents150_1.^2+line_currents250_1.^2+line_currents350_1.^2+line_currents450_1.^2+line_currents550_1.^2+line_currents650_1.^2);
        line_current_rms_1=sqrt(line_currents50_1.^2+line_currents150_1.^2+line_currents250_1.^2+line_currents350_1.^2+line_currents450_1.^2+line_currents550_1.^2+line_currents650_1.^2);
        line_ITHD_1=(line_current_distortion_rms_1./line_current_rms_1)*100;
        
        line_voltage_distortion_rms_2=sqrt(line_voltages150_2.^2+line_voltages250_2.^2+line_voltages350_2.^2+line_voltages450_2.^2+line_voltages550_2.^2+line_voltages650_2.^2);
        line_voltage_rms_2=sqrt(line_voltages50_2.^2+line_voltages150_2.^2+line_voltages250_2.^2+line_voltages350_2.^2+line_voltages450_2.^2+line_voltages550_2.^2+line_voltages650_2.^2);
        line_VTHD_2=(line_voltage_distortion_rms_2./line_voltage_rms_2)*100;

        line_current_distortion_rms_2=sqrt(line_currents150_2.^2+line_currents250_2.^2+line_currents350_2.^2+line_currents450_2.^2+line_currents550_2.^2+line_currents650_2.^2);
        line_current_rms_2=sqrt(line_currents50_2.^2+line_currents150_2.^2+line_currents250_2.^2+line_currents350_2.^2+line_currents450_2.^2+line_currents550_2.^2+line_currents650_2.^2);
        line_ITHD_2=(line_current_distortion_rms_2./line_current_rms_2)*100;
        
        line_voltage_distortion_rms_3=sqrt(line_voltages150_3.^2+line_voltages250_3.^2+line_voltages350_3.^2+line_voltages450_3.^2+line_voltages550_3.^2+line_voltages650_3.^2);
        line_voltage_rms_3=sqrt(line_voltages50_3.^2+line_voltages150_3.^2+line_voltages250_3.^2+line_voltages350_3.^2+line_voltages450_3.^2+line_voltages550_3.^2+line_voltages650_3.^2);
        line_VTHD_3=(line_voltage_distortion_rms_3./line_voltage_rms_3)*100;

        line_current_distortion_rms_3=sqrt(line_currents150_3.^2+line_currents250_3.^2+line_currents350_3.^2+line_currents450_3.^2+line_currents550_3.^2+line_currents650_3.^2);
        line_current_rms_3=sqrt(line_currents50_3.^2+line_currents150_3.^2+line_currents250_3.^2+line_currents350_3.^2+line_currents450_3.^2+line_currents550_3.^2+line_currents650_3.^2);
        line_ITHD_3=(line_current_distortion_rms_3./line_current_rms_3)*100;

        Currents_DSS_Lines_h_1=line_current_rms_1;
        Currents_DSS_Lines_h_2=line_current_rms_2;
        Currents_DSS_Lines_h_3=line_current_rms_3;
        
        %ttt2h=[ttt Currents_DSS_Lines_h_1' Currents_DSS_Lines_h_2' Currents_DSS_Lines_h_3'];
        ttt2h(:,1)=ttt;
        ttt2h(:,2)=Currents_DSS_Lines_h_1(time_simulation,:)';
        ttt2h(:,3)=Currents_DSS_Lines_h_2(time_simulation,:)';
        ttt2h(:,4)=Currents_DSS_Lines_h_3(time_simulation,:)';
        
        for i=1:size(ttt2h);
            if ttt2h(i,2)>ttt2h(i,1) || ttt2h(i,3)>ttt2h(i,1) || ttt2h(i,4)>ttt2h(i,1);
                ttt2h(i,5)=1;
                ttt3h(i,time_simulation)=1;
            else
                ttt2h(i,5)=0;
                ttt3h(i,time_simulation)=0; %sobrecorrientes en todo el tiempo
            end
        end
    
        %%%%%%%%loads
        iMon = DSSLoad.AllNames;
        DSSLoad.First;
        for iMonc=1:size(iMon);
            strName=DSSLoad.Name;
            load_monitor_data=ExtractMonitorData(DSSCircuit,strName);
            load_voltages50_1(time_simulation,iMonc,1)=load_monitor_data(1,3);
            load_voltages150_1(time_simulation,iMonc,1)=load_monitor_data(2,3);
            load_voltages250_1(time_simulation,iMonc,1)=load_monitor_data(3,3);
            load_voltages350_1(time_simulation,iMonc,1)=load_monitor_data(4,3);
            load_voltages450_1(time_simulation,iMonc,1)=load_monitor_data(5,3);
            load_voltages550_1(time_simulation,iMonc,1)=load_monitor_data(6,3);
            load_voltages650_1(time_simulation,iMonc,1)=load_monitor_data(7,3);
            
            load_voltages50_2(time_simulation,iMonc,1)=load_monitor_data(1,5);
            load_voltages150_2(time_simulation,iMonc,1)=load_monitor_data(2,5);
            load_voltages250_2(time_simulation,iMonc,1)=load_monitor_data(3,5);
            load_voltages350_2(time_simulation,iMonc,1)=load_monitor_data(4,5);
            load_voltages450_2(time_simulation,iMonc,1)=load_monitor_data(5,5);
            load_voltages550_2(time_simulation,iMonc,1)=load_monitor_data(6,5);
            load_voltages650_2(time_simulation,iMonc,1)=load_monitor_data(7,5);
            
            load_currents50_1(time_simulation,iMonc,1)=load_monitor_data(1,7);
            load_currents150_1(time_simulation,iMonc,1)=load_monitor_data(2,7);
            load_currents250_1(time_simulation,iMonc,1)=load_monitor_data(3,7);
            load_currents350_1(time_simulation,iMonc,1)=load_monitor_data(4,7);
            load_currents450_1(time_simulation,iMonc,1)=load_monitor_data(5,7);
            load_currents550_1(time_simulation,iMonc,1)=load_monitor_data(6,7);
            load_currents650_1(time_simulation,iMonc,1)=load_monitor_data(7,7);
            
            load_currents50_2(time_simulation,iMonc,1)=load_monitor_data(1,9);
            load_currents150_2(time_simulation,iMonc,1)=load_monitor_data(2,9);
            load_currents250_2(time_simulation,iMonc,1)=load_monitor_data(3,9);
            load_currents350_2(time_simulation,iMonc,1)=load_monitor_data(4,9);
            load_currents450_2(time_simulation,iMonc,1)=load_monitor_data(5,9);
            load_currents550_2(time_simulation,iMonc,1)=load_monitor_data(6,9);
            load_currents650_2(time_simulation,iMonc,1)=load_monitor_data(7,9);
                   
            DSSLoad.Next;
        end
        
        load_voltage_distortion_rms_1=sqrt(load_voltages150_1.^2+load_voltages250_1.^2+load_voltages350_1.^2+load_voltages450_1.^2+load_voltages550_1.^2+load_voltages650_1.^2);
        load_voltage_rms_1=sqrt(load_voltages50_1.^2+load_voltages150_1.^2+load_voltages250_1.^2+load_voltages350_1.^2+load_voltages450_1.^2+load_voltages550_1.^2+load_voltages650_1.^2);
        load_VTHD_1=(load_voltage_distortion_rms_1./load_voltage_rms_1)*100;
        load_VTHD_1_3h=(load_voltages150_1./load_voltages50_1)*100;
        load_VTHD_1_5h=(load_voltages250_1./load_voltages50_1)*100;
        load_VTHD_1_7h=(load_voltages350_1./load_voltages50_1)*100;
        load_VTHD_1_9h=(load_voltages450_1./load_voltages50_1)*100;
        load_VTHD_1_11h=(load_voltages550_1./load_voltages50_1)*100;
        
        
        
        load_current_distortion_rms_1=sqrt(load_currents150_1.^2+load_currents250_1.^2+load_currents350_1.^2+load_currents450_1.^2+load_currents550_1.^2+load_currents650_1.^2);
        load_current_rms_1=sqrt(load_currents50_1.^2+load_currents150_1.^2+load_currents250_1.^2+load_currents350_1.^2+load_currents450_1.^2+load_currents550_1.^2+load_currents650_1.^2);
        load_ITHD_1=(load_current_distortion_rms_1./load_current_rms_1)*100;
        
        load_voltage_distortion_rms_2=sqrt(load_voltages150_2.^2+load_voltages250_2.^2+load_voltages350_2.^2+load_voltages450_2.^2+load_voltages550_2.^2+load_voltages650_2.^2);
        load_voltage_rms_2=sqrt(load_voltages50_2.^2+load_voltages150_2.^2+load_voltages250_2.^2+load_voltages350_2.^2+load_voltages450_2.^2+load_voltages550_2.^2+load_voltages650_2.^2);
        load_VTHD_2=(load_voltage_distortion_rms_2./load_voltage_rms_2)*100;

        load_current_distortion_rms_2=sqrt(load_currents150_2.^2+load_currents250_2.^2+load_currents350_2.^2+load_currents450_2.^2+load_currents550_2.^2+load_currents650_2.^2);
        load_current_rms_2=sqrt(load_currents50_2.^2+load_currents150_2.^2+load_currents250_2.^2+load_currents350_2.^2+load_currents450_2.^2+load_currents550_2.^2+load_currents650_2.^2);
        load_ITHD_2=(load_current_distortion_rms_2./load_current_rms_2)*100;
        
        Voltages_DSS_Loads_h_1=load_voltage_rms_1;
        Voltages_DSS_Loads_h_2=load_voltage_rms_2;
        
        %bttt2h=[loadbttt Voltages_DSS_Loads_h_1' Voltages_DSS_Loads_h_2'];
        bttt2h(:,1)=loadbttt;
        bttt2h(:,2)=Voltages_DSS_Loads_h_1(time_simulation,:)';
        %bttt2h(:,3)=Voltages_DSS_Loads_h_2(time_simulation,:)';
        
        for i=1:size(bttt2h);
            if bttt2h(i,2)>bttt2h(i,1)+v_max;
                bttt2h(i,5)=1;
            else
                bttt2h(i,5)=0;
            end
            if bttt2h(i,2)<bttt2h(i,1)-v_min;
                bttt2h(i,6)=2;
            else
                bttt2h(i,6)=0;
            end
            
            bttt2h(i,7)=bttt2h(i,5)+bttt2h(i,6);
            bttt3h(i,time_simulation)=bttt2h(i,7); %sobrevoltajes en todo el tiempo
        end

        %% conexiones de evs
        if enable_vehicles>0;
            DSSActiveClass=DSSCircuit.ActiveClass;
            DSSCircuit.SetActiveClass('Storage');
            AllStorageNames=DSSActiveClass.AllNames;
            DSSCircuit.ActiveClass.First;
            for nevs=1:size(AllStorageNames)
                estados=string(DSSCircuit.ActiveCktElement.Properties('state').Val);
                if estados=='CHARGING';
                    estados2(nevs,time_simulation)=1;
                elseif estados=='IDLING'
                    estados2(nevs,time_simulation)=0;
                end
                evbusnames(nevs,1)=DSSCircuit.ActiveElement.BusNames;
                evbusnames1(nevs,1)=round(cellfun(@str2num,evbusnames(nevs,1)));
                DSSCircuit.ActiveClass.Next;
            end
        end
        
        
    end
    
    
    Voltages_DSS_Loads_1h_max=max(Voltages_DSS_Loads_h_1,[],2);
    Voltages_DSS_Loads_1h_min=min(Voltages_DSS_Loads_h_1,[],2);
    load_VTHD_1_max=max(load_VTHD_1,[],2);
    
    %%Check for violations
   
   
    
    thdttt2=load_VTHD_1';
    for i=1:size(thdttt2,1);
        for j=1:size(thdttt2,2);
            if thdttt2(i,j)>V_THD;
                thdttt3(i,j)=1;
            else
                thdttt3(i,j)=0;
            end
        end
    end
    
    thdttt2_3h=load_VTHD_1_3h';
    for i=1:size(thdttt2_3h,1);
        for j=1:size(thdttt2_3h,2);
            if thdttt2_3h(i,j)>V_3_VH;
                thdttt3_3h(i,j)=1;
            else
                thdttt3_3h(i,j)=0;
            end
        end
    end
    
    thdttt2_5h=load_VTHD_1_5h';
    for i=1:size(thdttt2_5h,1);
        for j=1:size(thdttt2_5h,2);
            if thdttt2_5h(i,j)>V_5_VH;
                thdttt3_5h(i,j)=1;
            else
                thdttt3_5h(i,j)=0;
            end
        end
    end
    
    thdttt2_7h=load_VTHD_1_7h';
    for i=1:size(thdttt2_7h,1);
        for j=1:size(thdttt2_7h,2);
            if thdttt2_7h(i,j)>V_7_VH;
                thdttt3_7h(i,j)=1;
            else
                thdttt3_7h(i,j)=0;
            end
        end
    end
    
    thdttt2_9h=load_VTHD_1_9h';
    for i=1:size(thdttt2_9h,1);
        for j=1:size(thdttt2_9h,2);
            if thdttt2_9h(i,j)>V_9_VH;
                thdttt3_9h(i,j)=1;
            else
                thdttt3_9h(i,j)=0;
            end
        end
    end
    
    thdttt2_11h=load_VTHD_1_11h';
    for i=1:size(thdttt2_11h,1);
        for j=1:size(thdttt2_11h,2);
            if thdttt2_11h(i,j)>V_11_VH;
                thdttt3_11h(i,j)=1;
            else
                thdttt3_11h(i,j)=0;
            end
        end
    end
    
    is_overcurrent_h=sum(ttt3h,'all');
    is_over_sub_voltage_h=sum(bttt3h,'all');
    is_over_V_THD=sum(thdttt3,'all');
    is_over_V_3_VH=sum(thdttt3_3h,'all');
    is_over_V_5_VH=sum(thdttt3_5h,'all');
    is_over_V_7_VH=sum(thdttt3_7h,'all');
    is_over_V_9_VH=sum(thdttt3_9h,'all');
    is_over_V_11_VH=sum(thdttt3_11h,'all');

    %Potencia de cada fase en el lado de alta del tranformador de
    %subestacion
    DSSCircuit.Meters.Name = 'TR1';
    potencia_lado_altah = ExtractMonitorData(DSSCircuit,'TR1');

    %Tensiones y corrientes en el Cliente 1
    DSSCircuit.Meters.Name = 'load20';
    voltaje_cliente_1h = ExtractMonitorData(DSSCircuit,'load20');

    %Tensiones y corrientes en el Cliente 2
    DSSCircuit.Meters.Name = 'load32';
    voltaje_cliente_2h = ExtractMonitorData(DSSCircuit,'load32');

    %Tensiones y corrientes en el Cliente 3
    DSSCircuit.Meters.Name = 'load53';
    voltaje_cliente_3h = ExtractMonitorData(DSSCircuit,'load53');

    %%tensiones en cargas
    voltajef=[load_voltage_rms_1 load_voltage_rms_2];
    corrientef=[load_current_rms_1 load_current_rms_2];
    potencia_activaf=[potencia_activa1h potencia_activa2h potencia_activa3h];
    potencia_reactivaf=[potencia_reactiva1h potencia_reactiva2h potencia_reactiva3h];
    
if enable_graphs==1;
    %% GRAFICA DE POTENCIAS
    figure;
    g1 = subplot (2,1,1);
    plot(g1, TIEMPO ,potencia_activaf(:,1));
    hold on
    plot(g1, TIEMPO, potencia_activaf(:,2), 'b');
    plot(g1, TIEMPO, potencia_activaf(:,3), 'r');
    title(g1,'Substation transformer power curve')
    xlabel(g1,'Time [Hour]')
    ylabel(g1,'Power [KW]')
    xlim([0 24]) 
    legend('Phase A','Phase B','Phase C')
    g1.XTick = [0:24];
    grid on
    potencia=potencia_activaf(:,1)+potencia_activaf(:,2)+potencia_activaf(:,3);
    preactiva=potencia_reactivaf(:,1)+potencia_reactivaf(:,2)+potencia_reactivaf(:,3);
    g2 = subplot (2,1,2);
    plot(g2, TIEMPO ,potencia');
    hold on
    plot(g2, TIEMPO ,preactiva');
    title(g2,'Substation transformer power curve (total)')
    xlabel(g2,'Time [Hour]')
    ylabel(g2,'Power [KW]')
    xlim([0 24])
    g2.XTick = [0:24];
    grid on
    hold off
    
    %% GRAFICA DE VOLTAJES 
    figure;
    plot(TIEMPO ,load_voltage_rms_1(:,1));
    hold on
    for clv=2:55;
        plot(TIEMPO , load_voltage_rms_1(:,clv));
    end
    %plot(TIEMPO,voltaje(:,1));
    title('Load Voltages')
    xlabel('Time [Hour]')
    ylabel('Voltage [V]')
    xlim([0 24]) 
    legend('Load 349')
    hold off
    XTick = [0:24];
    grid on
    
    %% GRAFICA DE corrientes 
    figure;
    plot(TIEMPO ,sqrt(corrientef(:,1).^2+corrientef(:,2).^2+corrientef(:,3).^2+corrientef(:,4).^2+corrientef(:,5).^2+corrientef(:,6).^2));
    hold on
    %plot(TIEMPO , corriente(:,1));
    title('Load Currents')
    xlabel('Time [Hour]')
    ylabel('Current [I]')
    xlim([0 24]) 
    legend('Load 349')
    hold off
    XTick = [0:24];
    grid on
    
    %% voltage profile armonicos
    figure;
    plot(Dist1h, V1h(:,1:3:end)+V1,'k*');  % black *
    hold on;
    plot(Dist1h, V1h(:,2:3:end)+V2, 'r+');  % red +
    plot(Dist1h, V1h(:,3:3:end)+V3, 'bd');  % diamond Marker
    legend('phase A','phase B','phase C','Location','SouthEast'); %put the legend
    title('Voltage Profile Plot'); %plot title
    ylim([0.9 1.1]);
    ylabel('Volts(pu)');
    xlabel('Distance from Substation');
    hold off
    
    %% GRAFICA DE THD 
    figure;
    plot(TIEMPO , load_VTHD_1(:,1));
    hold on
    for clvthd=2:55;
        plot(TIEMPO , load_VTHD_1(:,clvthd));
    end
    %plot(TIEMPO , load_VTHD_1(:,2));
    
    title('Load THDs')
    xlabel('Time [Hour]')
    ylabel('THD [%]')
    xlim([0 24]) 
    hold off
    XTick = [0:24];
    grid on
    
    figure;
    x=1:1:144;
    x=TIEMPO;
    y=[load_voltages150_1(:,1)';load_voltages250_1(:,1)';load_voltages350_1(:,1)';load_voltages450_1(:,1)';load_voltages550_1(:,1)';load_voltages650_1(:,1)'];
    bar(x,y)
    grid on
end
end



if enable_hosting==1
    is_over_V_THD=0;
    EV_index=0;
    PV_index=0;
    for EV_index=1:25;
        EV_index
        is_over_V_THD
        for time_simulation = 1:input.time_steps; %>>>>>>Starts time-series power flow
            DSSText.Command = 'Set ControlMode = time';
            t = sprintf('Set Mode=daily stepsize =%f' ,minutos);
            t2 = sprintf('m number=%f',time_simulation);
            DSSText.Command = [t t2];
            DSSSolution.Solve; %>>>>>>Solves power flow
            t = sprintf('Solve mode=harmonics')';
            DSSText.Command = t;

            potencia_lado_altah = ExtractMonitorData(DSSCircuit,'TR1');
            potencia_activa1h(time_simulation,1)=potencia_lado_altah(1,3);
            potencia_reactiva1h(time_simulation,1)=potencia_lado_altah(1,4);
            potencia_activa2h(time_simulation,1)=potencia_lado_altah(1,5);
            potencia_reactiva2h(time_simulation,1)=potencia_lado_altah(1,6);
            potencia_activa3h(time_simulation,1)=potencia_lado_altah(1,7);
            potencia_reactiva3h(time_simulation,1)=potencia_lado_altah(1,8);

    %%%%%%%%lines
            LMon = DSSLines.AllNames;
            DSSLines.First;
            for LMonc=1:size(LMon);
                strName=DSSLines.Name;
                line_monitor_data=ExtractMonitorData(DSSCircuit,strName);
                line_voltages50_1(time_simulation,LMonc,1)=line_monitor_data(1,3);
                line_voltages150_1(time_simulation,LMonc,1)=line_monitor_data(2,3);
                line_voltages250_1(time_simulation,LMonc,1)=line_monitor_data(3,3);
                line_voltages350_1(time_simulation,LMonc,1)=line_monitor_data(4,3);
                line_voltages450_1(time_simulation,LMonc,1)=line_monitor_data(5,3);
                line_voltages550_1(time_simulation,LMonc,1)=line_monitor_data(6,3);
                line_voltages650_1(time_simulation,LMonc,1)=line_monitor_data(7,3);

                line_voltages50_2(time_simulation,LMonc,1)=line_monitor_data(1,5);
                line_voltages150_2(time_simulation,LMonc,1)=line_monitor_data(2,5);
                line_voltages250_2(time_simulation,LMonc,1)=line_monitor_data(3,5);
                line_voltages350_2(time_simulation,LMonc,1)=line_monitor_data(4,5);
                line_voltages450_2(time_simulation,LMonc,1)=line_monitor_data(5,5);
                line_voltages550_2(time_simulation,LMonc,1)=line_monitor_data(6,5);
                line_voltages650_2(time_simulation,LMonc,1)=line_monitor_data(7,5);

                line_voltages50_3(time_simulation,LMonc,1)=line_monitor_data(1,7);
                line_voltages150_3(time_simulation,LMonc,1)=line_monitor_data(2,7);
                line_voltages250_3(time_simulation,LMonc,1)=line_monitor_data(3,7);
                line_voltages350_3(time_simulation,LMonc,1)=line_monitor_data(4,7);
                line_voltages450_3(time_simulation,LMonc,1)=line_monitor_data(5,7);
                line_voltages550_3(time_simulation,LMonc,1)=line_monitor_data(6,7);
                line_voltages650_3(time_simulation,LMonc,1)=line_monitor_data(7,7);

                line_currents50_1(time_simulation,LMonc,1)=line_monitor_data(1,9);
                line_currents150_1(time_simulation,LMonc,1)=line_monitor_data(2,9);
                line_currents250_1(time_simulation,LMonc,1)=line_monitor_data(3,9);
                line_currents350_1(time_simulation,LMonc,1)=line_monitor_data(4,9);
                line_currents450_1(time_simulation,LMonc,1)=line_monitor_data(5,9);
                line_currents550_1(time_simulation,LMonc,1)=line_monitor_data(6,9);
                line_currents650_1(time_simulation,LMonc,1)=line_monitor_data(7,9);

                line_currents50_2(time_simulation,LMonc,1)=line_monitor_data(1,11);
                line_currents150_2(time_simulation,LMonc,1)=line_monitor_data(2,11);
                line_currents250_2(time_simulation,LMonc,1)=line_monitor_data(3,11);
                line_currents350_2(time_simulation,LMonc,1)=line_monitor_data(4,11);
                line_currents450_2(time_simulation,LMonc,1)=line_monitor_data(5,11);
                line_currents550_2(time_simulation,LMonc,1)=line_monitor_data(6,11);
                line_currents650_2(time_simulation,LMonc,1)=line_monitor_data(7,11);

                line_currents50_3(time_simulation,LMonc,1)=line_monitor_data(1,13);
                line_currents150_3(time_simulation,LMonc,1)=line_monitor_data(2,13);
                line_currents250_3(time_simulation,LMonc,1)=line_monitor_data(3,13);
                line_currents350_3(time_simulation,LMonc,1)=line_monitor_data(4,13);
                line_currents450_3(time_simulation,LMonc,1)=line_monitor_data(5,13);
                line_currents550_3(time_simulation,LMonc,1)=line_monitor_data(6,13);
                line_currents650_3(time_simulation,LMonc,1)=line_monitor_data(7,13);

                DSSLines.Next;
            end

            line_voltage_distortion_rms_1=sqrt(line_voltages150_1.^2+line_voltages250_1.^2+line_voltages350_1.^2+line_voltages450_1.^2+line_voltages550_1.^2+line_voltages650_1.^2);
            line_voltage_rms_1=sqrt(line_voltages50_1.^2+line_voltages150_1.^2+line_voltages250_1.^2+line_voltages350_1.^2+line_voltages450_1.^2+line_voltages550_1.^2+line_voltages650_1.^2);
            line_VTHD_1=(line_voltage_distortion_rms_1./line_voltage_rms_1)*100;

            line_current_distortion_rms_1=sqrt(line_currents150_1.^2+line_currents250_1.^2+line_currents350_1.^2+line_currents450_1.^2+line_currents550_1.^2+line_currents650_1.^2);
            line_current_rms_1=sqrt(line_currents50_1.^2+line_currents150_1.^2+line_currents250_1.^2+line_currents350_1.^2+line_currents450_1.^2+line_currents550_1.^2+line_currents650_1.^2);
            line_ITHD_1=(line_current_distortion_rms_1./line_current_rms_1)*100;

            line_voltage_distortion_rms_2=sqrt(line_voltages150_2.^2+line_voltages250_2.^2+line_voltages350_2.^2+line_voltages450_2.^2+line_voltages550_2.^2+line_voltages650_2.^2);
            line_voltage_rms_2=sqrt(line_voltages50_2.^2+line_voltages150_2.^2+line_voltages250_2.^2+line_voltages350_2.^2+line_voltages450_2.^2+line_voltages550_2.^2+line_voltages650_2.^2);
            line_VTHD_2=(line_voltage_distortion_rms_2./line_voltage_rms_2)*100;

            line_current_distortion_rms_2=sqrt(line_currents150_2.^2+line_currents250_2.^2+line_currents350_2.^2+line_currents450_2.^2+line_currents550_2.^2+line_currents650_2.^2);
            line_current_rms_2=sqrt(line_currents50_2.^2+line_currents150_2.^2+line_currents250_2.^2+line_currents350_2.^2+line_currents450_2.^2+line_currents550_2.^2+line_currents650_2.^2);
            line_ITHD_2=(line_current_distortion_rms_2./line_current_rms_2)*100;

            line_voltage_distortion_rms_3=sqrt(line_voltages150_3.^2+line_voltages250_3.^2+line_voltages350_3.^2+line_voltages450_3.^2+line_voltages550_3.^2+line_voltages650_3.^2);
            line_voltage_rms_3=sqrt(line_voltages50_3.^2+line_voltages150_3.^2+line_voltages250_3.^2+line_voltages350_3.^2+line_voltages450_3.^2+line_voltages550_3.^2+line_voltages650_3.^2);
            line_VTHD_3=(line_voltage_distortion_rms_3./line_voltage_rms_3)*100;

            line_current_distortion_rms_3=sqrt(line_currents150_3.^2+line_currents250_3.^2+line_currents350_3.^2+line_currents450_3.^2+line_currents550_3.^2+line_currents650_3.^2);
            line_current_rms_3=sqrt(line_currents50_3.^2+line_currents150_3.^2+line_currents250_3.^2+line_currents350_3.^2+line_currents450_3.^2+line_currents550_3.^2+line_currents650_3.^2);
            line_ITHD_3=(line_current_distortion_rms_3./line_current_rms_3)*100;

            Currents_DSS_Lines_h_1=line_current_rms_1;
            Currents_DSS_Lines_h_2=line_current_rms_2;
            Currents_DSS_Lines_h_3=line_current_rms_3;

            ttt2h(:,1)=ttt;
            ttt2h(:,2)=Currents_DSS_Lines_h_1(time_simulation,:)';
            ttt2h(:,3)=Currents_DSS_Lines_h_2(time_simulation,:)';
            ttt2h(:,4)=Currents_DSS_Lines_h_3(time_simulation,:)';

            for i=1:size(ttt2h);
                if ttt2h(i,2)>ttt2h(i,1) || ttt2h(i,3)>ttt2h(i,1) || ttt2h(i,4)>ttt2h(i,1);
                    ttt2h(i,5)=1;
                    ttt3h(i,time_simulation)=1;
                else
                    ttt2h(i,5)=0;
                    ttt3h(i,time_simulation)=0; %sobrecorrientes en todo el tiempo
                end
            end

            %%%%%%%%loads
            iMon = DSSLoad.AllNames;
            DSSLoad.First;
            for iMonc=1:size(iMon);
                strName=DSSLoad.Name;
                load_monitor_data=ExtractMonitorData(DSSCircuit,strName);
                load_voltages50_1(time_simulation,iMonc,1)=load_monitor_data(1,3);
                load_voltages150_1(time_simulation,iMonc,1)=load_monitor_data(2,3);
                load_voltages250_1(time_simulation,iMonc,1)=load_monitor_data(3,3);
                load_voltages350_1(time_simulation,iMonc,1)=load_monitor_data(4,3);
                load_voltages450_1(time_simulation,iMonc,1)=load_monitor_data(5,3);
                load_voltages550_1(time_simulation,iMonc,1)=load_monitor_data(6,3);
                load_voltages650_1(time_simulation,iMonc,1)=load_monitor_data(7,3);

                load_voltages50_2(time_simulation,iMonc,1)=load_monitor_data(1,5);
                load_voltages150_2(time_simulation,iMonc,1)=load_monitor_data(2,5);
                load_voltages250_2(time_simulation,iMonc,1)=load_monitor_data(3,5);
                load_voltages350_2(time_simulation,iMonc,1)=load_monitor_data(4,5);
                load_voltages450_2(time_simulation,iMonc,1)=load_monitor_data(5,5);
                load_voltages550_2(time_simulation,iMonc,1)=load_monitor_data(6,5);
                load_voltages650_2(time_simulation,iMonc,1)=load_monitor_data(7,5);

                load_currents50_1(time_simulation,iMonc,1)=load_monitor_data(1,7);
                load_currents150_1(time_simulation,iMonc,1)=load_monitor_data(2,7);
                load_currents250_1(time_simulation,iMonc,1)=load_monitor_data(3,7);
                load_currents350_1(time_simulation,iMonc,1)=load_monitor_data(4,7);
                load_currents450_1(time_simulation,iMonc,1)=load_monitor_data(5,7);
                load_currents550_1(time_simulation,iMonc,1)=load_monitor_data(6,7);
                load_currents650_1(time_simulation,iMonc,1)=load_monitor_data(7,7);

                load_currents50_2(time_simulation,iMonc,1)=load_monitor_data(1,9);
                load_currents150_2(time_simulation,iMonc,1)=load_monitor_data(2,9);
                load_currents250_2(time_simulation,iMonc,1)=load_monitor_data(3,9);
                load_currents350_2(time_simulation,iMonc,1)=load_monitor_data(4,9);
                load_currents450_2(time_simulation,iMonc,1)=load_monitor_data(5,9);
                load_currents550_2(time_simulation,iMonc,1)=load_monitor_data(6,9);
                load_currents650_2(time_simulation,iMonc,1)=load_monitor_data(7,9);

                DSSLoad.Next;
            end

            load_voltage_distortion_rms_1=sqrt(load_voltages150_1.^2+load_voltages250_1.^2+load_voltages350_1.^2+load_voltages450_1.^2+load_voltages550_1.^2+load_voltages650_1.^2);
            load_voltage_rms_1=sqrt(load_voltages50_1.^2+load_voltages150_1.^2+load_voltages250_1.^2+load_voltages350_1.^2+load_voltages450_1.^2+load_voltages550_1.^2+load_voltages650_1.^2);
            load_VTHD_1=(load_voltage_distortion_rms_1./load_voltage_rms_1)*100;
            load_VTHD_1_3h=(load_voltages150_1./load_voltages50_1)*100;
            load_VTHD_1_5h=(load_voltages250_1./load_voltages50_1)*100;
            load_VTHD_1_7h=(load_voltages350_1./load_voltages50_1)*100;
            load_VTHD_1_9h=(load_voltages450_1./load_voltages50_1)*100;
            load_VTHD_1_11h=(load_voltages550_1./load_voltages50_1)*100;



            load_current_distortion_rms_1=sqrt(load_currents150_1.^2+load_currents250_1.^2+load_currents350_1.^2+load_currents450_1.^2+load_currents550_1.^2+load_currents650_1.^2);
            load_current_rms_1=sqrt(load_currents50_1.^2+load_currents150_1.^2+load_currents250_1.^2+load_currents350_1.^2+load_currents450_1.^2+load_currents550_1.^2+load_currents650_1.^2);
            load_ITHD_1=(load_current_distortion_rms_1./load_current_rms_1)*100;

            load_voltage_distortion_rms_2=sqrt(load_voltages150_2.^2+load_voltages250_2.^2+load_voltages350_2.^2+load_voltages450_2.^2+load_voltages550_2.^2+load_voltages650_2.^2);
            load_voltage_rms_2=sqrt(load_voltages50_2.^2+load_voltages150_2.^2+load_voltages250_2.^2+load_voltages350_2.^2+load_voltages450_2.^2+load_voltages550_2.^2+load_voltages650_2.^2);
            load_VTHD_2=(load_voltage_distortion_rms_2./load_voltage_rms_2)*100;

            load_current_distortion_rms_2=sqrt(load_currents150_2.^2+load_currents250_2.^2+load_currents350_2.^2+load_currents450_2.^2+load_currents550_2.^2+load_currents650_2.^2);
            load_current_rms_2=sqrt(load_currents50_2.^2+load_currents150_2.^2+load_currents250_2.^2+load_currents350_2.^2+load_currents450_2.^2+load_currents550_2.^2+load_currents650_2.^2);
            load_ITHD_2=(load_current_distortion_rms_2./load_current_rms_2)*100;

            Voltages_DSS_Loads_h_1=load_voltage_rms_1;
            Voltages_DSS_Loads_h_2=load_voltage_rms_2;

            bttt2h(:,1)=loadbttt;
            bttt2h(:,2)=Voltages_DSS_Loads_h_1(time_simulation,:)';

            for i=1:size(bttt2h);
                if bttt2h(i,2)>bttt2h(i,1)+v_max;
                    bttt2h(i,5)=1;
                else
                    bttt2h(i,5)=0;
                end
                if bttt2h(i,2)<bttt2h(i,1)-v_min;
                    bttt2h(i,6)=2;
                else
                    bttt2h(i,6)=0;
                end

                bttt2h(i,7)=bttt2h(i,5)+bttt2h(i,6);
                bttt3h(i,time_simulation)=bttt2h(i,7); %sobrevoltajes en todo el tiempo
            end
        end


        Voltages_DSS_Loads_1h_max=max(Voltages_DSS_Loads_h_1,[],2);
        Voltages_DSS_Loads_1h_min=min(Voltages_DSS_Loads_h_1,[],2);
        load_VTHD_1_max=max(load_VTHD_1,[],2);

        %%Check for violations

        thdttt2=load_VTHD_1';
        for i=1:size(thdttt2,1);
            for j=1:size(thdttt2,2);
                if thdttt2(i,j)>V_THD;
                    thdttt3(i,j)=1;
                else
                    thdttt3(i,j)=0;
                end
            end
        end

        thdttt2_3h=load_VTHD_1_3h';
        for i=1:size(thdttt2_3h,1);
            for j=1:size(thdttt2_3h,2);
                if thdttt2_3h(i,j)>V_3_VH;
                    thdttt3_3h(i,j)=1;
                else
                    thdttt3_3h(i,j)=0;
                end
            end
        end

        thdttt2_5h=load_VTHD_1_5h';
        for i=1:size(thdttt2_5h,1);
            for j=1:size(thdttt2_5h,2);
                if thdttt2_5h(i,j)>V_5_VH;
                    thdttt3_5h(i,j)=1;
                else
                    thdttt3_5h(i,j)=0;
                end
            end
        end

        thdttt2_7h=load_VTHD_1_7h';
        for i=1:size(thdttt2_7h,1);
            for j=1:size(thdttt2_7h,2);
                if thdttt2_7h(i,j)>V_7_VH;
                    thdttt3_7h(i,j)=1;
                else
                    thdttt3_7h(i,j)=0;
                end
            end
        end

        thdttt2_9h=load_VTHD_1_9h';
        for i=1:size(thdttt2_9h,1);
            for j=1:size(thdttt2_9h,2);
                if thdttt2_9h(i,j)>V_9_VH;
                    thdttt3_9h(i,j)=1;
                else
                    thdttt3_9h(i,j)=0;
                end
            end
        end

        thdttt2_11h=load_VTHD_1_11h';
        for i=1:size(thdttt2_11h,1);
            for j=1:size(thdttt2_11h,2);
                if thdttt2_11h(i,j)>V_11_VH;
                    thdttt3_11h(i,j)=1;
                else
                    thdttt3_11h(i,j)=0;
                end
            end
        end

        is_overcurrent_h=sum(ttt3h,'all');
        is_over_sub_voltage_h=sum(bttt3h,'all');
        is_over_V_THD=sum(thdttt3,'all');
        is_over_V_3_VH=sum(thdttt3_3h,'all');
        is_over_V_5_VH=sum(thdttt3_5h,'all');
        is_over_V_7_VH=sum(thdttt3_7h,'all');
        is_over_V_9_VH=sum(thdttt3_9h,'all');
        is_over_V_11_VH=sum(thdttt3_11h,'all');

        %%tensiones en cargas
        voltajef=[load_voltage_rms_1 load_voltage_rms_2];
        corrientef=[load_current_rms_1 load_current_rms_2];
        potencia_activaf=[potencia_activa1h potencia_activa2h potencia_activa3h];
        potencia_reactivaf=[potencia_reactiva1h potencia_reactiva2h potencia_reactiva3h];
        
        if is_over_V_THD>0
            break
        end
    end
end
