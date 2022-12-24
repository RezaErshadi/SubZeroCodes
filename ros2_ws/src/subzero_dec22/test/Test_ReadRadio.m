clear
close all
clc
ConnectedPorts = serialportlist';
%%
% iport = contains(ConnectedPorts,"USB0");
iport = 8;
device = serialport(ConnectedPorts(iport),38400,"Timeout",1);
configureTerminator(device,"CR")
% flush(device)
% i = 1;
while true
    NBA = device.NumBytesAvailable;
    disp(NBA)
    temp1 = readline(device);
    disp(temp1)
end
% while true
%     dta = readline(device)
% %     dta = read(device,4,"string");
%     flush(device)
%     if ~isempty(dta)
% %         disp(dta)
%         RecDta(i,1) = dta;
%         i = i+1;
%     end
% %     flush(device)
% end

% Initiator = '#';
% Terminator = '$';
% Data = [];
% while true
%     c_temp = read(device,1,"string");
%     if c_temp == Initiator
%         c = c_temp;
%         while c_temp == Initiator
%             c_temp2 = read(device,1,"string");
%             c = c+c_temp2;
%             if c_temp2 == Terminator
%                 flush(device)
%                 disp(c)
%                 Data = [Data ; c];
%                 c_temp = '$';
%                 break 
%             end
%         end
%     end
% end