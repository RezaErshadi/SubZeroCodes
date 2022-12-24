clear
close all
clc
%%
ConnectedPorts = serialportlist';
iport = 8;
device = serialport(ConnectedPorts(iport),38400,"Timeout",1);
configureTerminator(device,"CR")
while true
%     NBA = device.NumBytesAvailable;
%     disp(NBA)
    temp1 = readline(device);
    disp(temp1)
end