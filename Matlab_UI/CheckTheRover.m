clear
close all
clc
%%
ConnectedPorts = serialportlist';
for i = 1:length(ConnectedPorts)
    fprintf(string(i)+":" +ConnectedPorts(i)+"\n")
end
idev = input("Which port number?\n");
%%
dev = serialport(ConnectedPorts(idev),38400,"Timeout",1);
configureTerminator(dev,"CR");
%%
Camp = [-8.5934 -71.7248];
figure,
ax1 = subplot(3,10,[1:6,11:16,21:26]);
ax2 = subplot(3,10,[7:10]);
ax3 = subplot(3,10,[17:20]);
ax4 = subplot(3,10,[27:30]);
pCamp = plot(ax1,Camp(1),Camp(2),'^k','MarkerFaceColor','y','MarkerSize',15);
hold(ax1,'on')
plot(ax2,0,0)
hold(ax2,'on')
plot(ax3,0,0)
hold(ax3,'on')
plot(ax4,0,0)
hold(ax4,'on')
ax1.Color = [0.25 0.25 0.25];
title(ax1,"Rover Position")
title(ax2,"GPS Speed")
title(ax3,"Altitude")
title(ax4,"Battery Voltage")
ylim(ax2,[0 4])
ylim(ax4,[45 60])
ax2.XTickLabel = '';
ax3.XTickLabel = '';
%%
n = 360; % 30 minutes data for 5 sec telemetry interval
pRoverTail = [];
pRoverPose = [];
pSpeed = [];
pAlt = [];
pBatt = [];
t = [];
rp = [];
v = [];
h = [];
b = [];
while true
    [RadioString,TEL] = RecRadioStr(dev);
    if TEL == true
        RoverTEL = ReadTEL(RadioString);
        time = RoverTEL.Time;
        pose = RoverTEL.Pose;
        speed = RoverTEL.speed;
        altitude = RoverTEL.altitude;
        battery = RoverTEL.battery;
        t = [t;time];
        rp = [rp;pose];
        v = [v;speed];
        h = [h;altitude];
        b = [b;battery];
        if size(rp,1)==n+1
            t(1,:) = [];
            rp(1,:) = [];
            v(1,:) = [];
            h(1,:) = [];
            b(1,:) = [];
        end
        
        delete(pRoverTail)
        delete(pRoverPose)
        delete(pSpeed)
        delete(pAlt)
        delete(pBatt)
        
        pRoverTail = plot(ax1,rp(:,1),rp(:,2),'.-g','MarkerSize',15,'LineWidth',2);
        pRoverPose = plot(ax1,rp(end,1),rp(end,2),'sr','MarkerSize',15);
        
        pSpeed = plot(ax2,1:size(rp,1),v,'.-r','MarkerSize',15);
        pAlt = plot(ax3,1:size(rp,1),h,'.-b','MarkerSize',15);
        pBatt = plot(ax4,1:size(rp,1),b,'.-k','MarkerSize',15);
        if length(t)>1
            xlim(ax2,[1,n])
            xlim(ax3,[1,n])
            xlim(ax4,[1,n])
            ax4.XTick = [1,size(rp,1)];
            ax4.XTickLabel = string([t(1),t(end)]);
            ax1.XTickLabelRotation = 90;
            ax4.XTickLabelRotation = 90;
        end
    end
end

function [RadioString,TEL] = RecRadioStr(dev)
    RadioString = "";
    TEL = false;
    NBA = dev.NumBytesAvailable;
    if NBA ~= 0
        NewRadioString = readline(dev);
        flush(dev,"input")
        try
            if contains(NewRadioString,"#")
                NewRadioString = char(NewRadioString);
                i_in = find(NewRadioString=='#');
                i_te = find(NewRadioString=='$');
                NewRadioString = NewRadioString(i_in:i_te);
                NewRadioString = erase(NewRadioString,"#");
                NewRadioString = erase(NewRadioString,"$");
                RadioString = string(NewRadioString);
                fprintf("-------------------\n")
                fprintf("-------------------\n")
                fprintf(RadioString+'\n')
                fprintf("-------------------\n")
                fprintf("-------------------\n")
                if contains(RadioString,"TELEMETRY")
                    TEL = true;
                end
            end
        end
    end
end

function RoverTEL = ReadTEL(RadioString)
    tel = RadioString;
    tel = erase(tel,'TELEMETRY,');
    sepTEL = strsplit(tel,',');
    RoverTEL.Time = round(str2double(sepTEL(2)),0);
    RoverTEL.Pose = [str2double(sepTEL(4)) str2double(sepTEL(3))];
    RoverTEL.speed = str2double(sepTEL(6));
    RoverTEL.altitude = str2double(sepTEL(5));
    RoverTEL.battery = str2double(sepTEL(8));
    fprintf("********** TELEMETRY **********\n"+...
            "GPS age: "+sepTEL(1)+"\n"+...
            "Time: "+sepTEL(2)+"\n"+...
            "Latitude: "+sepTEL(3)+"\n"+...
            "Longitude: "+sepTEL(4)+"\n"+...
            "Altitude: "+sepTEL(5)+"\n"+...
            "Speed: "+sepTEL(6)+"\n"+...
            "Course: "+sepTEL(7)+"\n"+...
            "Battery: "+sepTEL(8)+"\n"+...
            "V speed: "+sepTEL(9)+"\n"+...
            "W speed: "+sepTEL(10)+"\n"+...
            "Next goal number: "+sepTEL(11)+"\n"+...
            "Next goal lat: "+sepTEL(12)+"\n"+...
            "Next goal lon: "+sepTEL(13)+"\n"+...
            "Bearing to goal: "+sepTEL(14)+"\n"+...
            "Distance to goal: "+sepTEL(15)+"\n"+...
            "Distance from ref: "+sepTEL(16)+"\n"+...
            "GPS Quality: "+sepTEL(17)+"\n")
end