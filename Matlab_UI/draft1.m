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

while true
    [RadioString,TEL] = RecRadioStr(dev);
    
end