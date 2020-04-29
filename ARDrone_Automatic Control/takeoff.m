controlChannel = udp('192.168.1.1',5556,'LocalPort',5556);
stateChannel = udp('192.168.1.1',5554,'LocalPort',5554);
fopen(controlChannel);
fopen(stateChannel);

BASIC_CODE = 2^18 + 2^20 + 2^22 + 2^24 + 2^28;
START_CODE = BASIC_CODE + 2^9;
AR_START   = sprintf('AT*REF=%d,%d\r',tic,START_CODE);
fprintf(controlChannel, AR_START);