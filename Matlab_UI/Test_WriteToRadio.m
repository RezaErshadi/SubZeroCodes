clear
close all
clc
%%
ConnectedPorts = serialportlist';
iport = 8;
device = serialport(ConnectedPorts(iport),38400,"Timeout",1);
configureTerminator(device,"CR")
while true
    writeline(device,"hello")
    pause(1)
end