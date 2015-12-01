%Voltage measurement from the Arduino

s1 = serial('COM5');        %set up COM port.
s1.BaudRate = 9600;
set(s1, 'terminator', 'CR/LF');
fopen(s1);
voltage(0) = 13.9; %Fully charged voltage

s1.ReadAsyncMode = 'Continuous';
while 1
    
 if s1.BytesAvailable
    voltage(end + 1) = fgetl(s1);   
    
    if isempty(voltage)
       break 
    end
 end
 
end
fclose(s1);

t = 60*(0: 1: length(voltage));
initial_charge = 2.8; %[Ah] correct value
I = 1; %[A] This is whatever constant current we set to.
charge_consumed = I* t; %charge is integral of current over time (with constant current, I*t)
SOC = 1 - (charge_consumed/initial_charge);

plot(voltage, SOC);
