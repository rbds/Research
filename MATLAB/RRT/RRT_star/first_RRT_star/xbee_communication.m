%XBee- Matlab communication testing
clear
close all

robot.p = [0 0]'; %robot position
robot.r = 1; %robot radius
robot.t = 0; %robot angle in radians
prev_enc_right = 0;
prev_enc_left = 0;

straight_gain = 1/450; %[inches/count]
straight_std = 30; %[counts]

close all
figure(1)
hold on
axis([-10 25 -10 25])
r = draw_robot(robot);
 
s1 = serial('COM5');        %set up COM port.
s1.BaudRate = 9600;
set(s1, 'terminator', 'LF');
fopen(s1);


s1.ReadAsyncMode = 'Continuous';
while 1
b = s1.BytesAvailable;
while b < 2
    %While there is no data available, keep waiting for data.
    b = s1.BytesAvailable;
end
msg = fgetl(s1);

if isempty(msg)
   break 
end

    switch msg
        case 'encoder'
            b = s1.BytesAvailable;
                while b < 8
                    %do nothing
                end
            enc_data_right = fgetl(s1); %read in the data from one encoder.
            enc_data_left = fgetl(s1);

            %Do some processing with encoder data
            left = enc_data_left - prev_enc_left;
            right = enc_data_right - prev_enc_right;

            distance = (left + right)/2 * straight_gain;    %calculate distance travelled.
            new_pos(1) = robot.p(1) + distance*cos(robot.t);    
            new_pos(2) = robot.p(2) + distance*sin(robot.t);
            plot([robot.p(1); new_pos(1)],[robot.p(2); new_pos(2)], 'g', 'LineWidth', 3)

            robot.p = new_pos';
            set(h, 'visible', off)
            h = draw_robot;
            set(h, 'visible', on)
            prev_enc_left = enc_data_left;
            prev_enc_right = enc_data_right;

        case 'ping'
            b = s1.BytesAvailable;
                while b < 8
                    %do nothing
                end        
            dst = fgetl(s1);
            angle = fgetl(s1);
            
            x2 = dist*cos(angle) + x1;      
            y2 = dist*sin(angle) + y1;
            plot(x2, y2, 'r*', 'LineWidth', 3);    %plot the coordinates of the sensed point.
        case 'heading'
            b = s1.BytesAvailable;
                while b < 4
                    %do nothing
                end        
            heading = fgetl(s1);
            robot.t = heading;
            set(h, 'visible', off)
            h = draw_robot;
            set(h, 'visible', on)            
            
        case 'voltage'
            b = s1.BytesAvailable;
                while b < 8
                    %do nothing
                end        
            volts = fgetl(s1); 
            
            
    end


end



fclose(s1);





