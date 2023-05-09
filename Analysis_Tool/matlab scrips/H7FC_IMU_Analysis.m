Data = load('C:\Users\HUAV\Desktop\8B!T0\H7FC_V0.02\Analysis_Tool\Log2Txt\logfile\imu.txt');

% Data = Data(1:10370,:);

Fs = 2000;
T = [0:1/Fs:2000];
DataLen = length(Data);

UsRt = Data(:,1);
UsRtDiff = UsRt(2:end) - UsRt(1:end-1);
GyrX = Data(:,2);
GyrY = Data(:,3);
GyrZ = Data(:,4);
AccX = Data(:,5);
AccY = Data(:,6);
AccZ = Data(:,7);

figure(1);
plot(GyrX);grid on;hold on;legend('GX');
plot(GyrY);grid on;hold on;legend('GY');
plot(GyrZ);grid on;hold on;legend('GZ');

figure(2);
plot(AccX);grid on;hold on;legend('AX');
plot(AccY);grid on;hold on;legend('AY');
plot(AccZ);grid on;hold on;legend('AZ');

figure(3);
plot(UsRtDiff);grid on;hold on;legend('UsRtDiff');

% FFT
FFT_GyrX = fft(GyrX);
FFT_GyrX_Abs = abs(FFT_GyrX(1:1:DataLen/2))*2/DataLen;
Freq_GyrX = (0:DataLen/2 - 1)'*Fs/DataLen;

figure(4)
plot(Freq_GyrX,FFT_GyrX_Abs);grid on;hold on;
