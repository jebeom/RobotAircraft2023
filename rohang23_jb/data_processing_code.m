%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Title  : Data Processing for Robot Aircraft Competition
% Author : Jebeom Chae
% Date   : 2023-09-02 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
clc; clear; close all;

addpath("C:\Users\s_cowpqja3302\Desktop\rohang_jb\Functions");
addpath('C:\Users\s_cowpqja3302\Desktop\rohang_jb\Data')     %Edit before run
%% Data read Edit before run

% 3~4m 간격으로 픽스호크가 데이터 송출

vehicle_global_position_0 = readmatrix('Data_vehicle_global_position_0.csv');
commander_state_0 = readmatrix('Data_commander_state_0.csv');
sensor_gps_0 = readmatrix('Data_sensor_gps_0.csv');


lat = vehicle_global_position_0(:,3);
lon = vehicle_global_position_0(:,4);
alt = vehicle_global_position_0(:,5); % 해발고도

time_utc = sensor_gps_0(:,2); % real flight
% time_utc = sensor_gps_0(:,3); % SITL
time_stamp_sensor = sensor_gps_0(:,1); % GPS Time
time_stamp_vehicle = vehicle_global_position_0(:,1);
flight_type = commander_state_0(:,3);


%% WPT 

% 실제 대회 좌표 나중에 수정필요
% 실제 사용 3d - wpt 1, 3, 5, 6  / 2d - wpt 2, 4
format long
base = [36.6607034, 126.3420265, 0]; 
WPT1 = [36.660804, 126.342046, 50]; 
WPT2 = [36.661176, 126.343689, 50]; 
WPT3 = [36.662515, 126.341660, 50];
WPT4 = [36.661521, 126.342140, 10]; 
WPT5 = [36.660154, 126.342902, 10]; 
WPT6 = [36.660154, 126.342902, 50];

%% frame transform (LLA to ENU)
wgs84 = wgs84Ellipsoid; % 단위를 meter 로

lat0 = lat(1);
lon0 = lon(1);
alt0 = alt(1); % ?

% LLA to ENU
format shortG
for j=1:1:length(lat) %flight
    [xE(j),yN(j),zD(j)] = geodetic2enu(lat(j),lon(j),alt(j),lat0,lon0,alt0,wgs84); 
end 

% WayPoint ENU 변환
[xEb, yNb, zDb] = geodetic2enu(base(1),base(2),base(3),lat0,lon0,alt0,wgs84);
[xE1, yN1, zD1] = geodetic2enu(WPT1(1),WPT1(2),WPT1(3),lat0,lon0,alt0,wgs84);
[xE2, yN2, zD2] = geodetic2enu(WPT2(1),WPT2(2),WPT2(3),lat0,lon0,alt0,wgs84);
[xE3, yN3, zD3] = geodetic2enu(WPT3(1),WPT3(2),WPT3(3),lat0,lon0,alt0,wgs84);
[xE4, yN4, zD4] = geodetic2enu(WPT4(1),WPT4(2),WPT4(3),lat0,lon0,alt0,wgs84);
[xE5, yN5, zD5] = geodetic2enu(WPT5(1),WPT5(2),WPT5(3),lat0,lon0,alt0,wgs84);
[xE6, yN6, zD6] = geodetic2enu(WPT6(1),WPT6(2),WPT6(3),lat0,lon0,alt0,wgs84);

zDb=0;
zD1=50;
zD2=50;
zD3=50;
zD4=10;
zD5=10;
zD6=50;
%% flight mode recognize


for i= 1:1:length(flight_type)
    if flight_type(i) == 3 || flight_type(i) == 7 || flight_type(i) == 11

       flight_type(i) = 1;
    else 
        flight_type(i) = 0;
    end
end

%% resizing matrix

for i= 1:1:length(flight_type)-1
    if flight_type(i) > flight_type(i+1)
        cp = i;    
    else 
        cp = 0;     % 자동비행 
    end
end

if cp == 0  % 빈 부분 채워주기
    for i = length(flight_type):1:length(lat)
        flight_type(i) = 1;
    end
end % 실패하면 0으로 넣기
%% time data processing

% GPS time to utc time
utc_start = time_utc(1) - time_stamp_sensor(1); % UTC time에서 GPS time 빼서 leap second 구함
time_real_utc = time_stamp_vehicle + utc_start ;  %
time_real_utc_korea = time_real_utc + 32400 * 1000000;
% time_real_gps = time_stamp
DT3 = datetime(time_real_utc_korea,'ConvertFrom','epochtime','TicksPerSecond',1000000,'TimeZone','UTC');
utcSeconds = posixtime(DT3);
gpsConstantOffset = 315964800;
time_real_gps = utcSeconds - gpsConstantOffset;

% GPS 시간을 초로 변환
gpsSeconds = time_real_gps;

% % UTC 시간 변환 (간단한 예제이므로 보정값은 고려하지 않음)
% gpsConstantOffset = 315964800; % 1980년 1월 6일 00:00:00에 해당하는 GPS 시간 (초 단위)
% utcTime = time_real_gps + gpsConstantOffset;
% 
% % 변환된 UTC 시간을 datetime 형식으로 변환
% DT2 = datetime(utcTime,'ConvertFrom','epochtime','TicksPerSecond',1000000);

%% time convert ( UTC version )

DT = datetime(time_real_utc_korea,'ConvertFrom','epochtime','TicksPerSecond',1000000);
out = datestr(DT, 'yyyy, mm, dd, HH, MM, SS, FFF');

year = str2num(out(:,1:4));
month = str2num(out(:,7:8));
day = str2num(out(:,11:12));
hour = str2num(out(:,15:16));
minute = str2num(out(:,19:20));
second = str2num(out(:,23:24));
m_second = str2num(out(:,27:29));



%% Finding close point ( 2D ) 
% 고도 고려하느냐 안하느냐 미션에 따라 수정필요

close_point_Base_2d = disT(xE, yN, xEb, yNb);
close_point_WPT1_2d = disT(xE, yN, xE1, yN1);
close_point_WPT2_2d = disT(xE, yN, xE2, yN2);
close_point_WPT3_2d = disT(xE, yN, xE3, yN3);
close_point_WPT4_2d = disT(xE, yN, xE4, yN4);
close_point_WPT5_2d = disT(xE, yN, xE5, yN5); 
close_point_WPT6_2d = disT(xE, yN, xE6, yN6);

%% Finding close point ( 3D )
close_point_WPT1_3d = disT_3d(xE, yN, zD, xE1, yN1, zD1);
close_point_WPT3_3d = disT_3d(xE, yN, zD, xE3, yN3, zD3);
close_point_WPT5_3d = disT_3d(xE, yN, zD, xE5, yN5, zD5);
close_point_WPT6_3d = disT_3d(xE, yN, zD, xE6, yN6, zD6);
close_point_Land = disT(lat,lon, base(1), base(2));


%% Finding index ( 2D )
[Base_dist_min_2d, Base_dist_index_2d] = min(close_point_Base_2d(1:300));
Base_index_in_original = find(close_point_Base_2d == Base_dist_min_2d, 1);
Base_dist_index_2d = Base_index_in_original;

% base 근처로 돌아오면서 WPT1 과 가까워 지는 부분이 생겨 300까지 설정
[WPT1_dist_min_2d, WPT1_dist_index_2d] = min(close_point_WPT1_2d(Base_dist_index_2d+1:Base_dist_index_2d+400));
WPT1_index_in_original = find(close_point_WPT1_2d == WPT1_dist_min_2d, 1);
WPT1_dist_index_2d = WPT1_index_in_original;

% [WPT1_dist_min, WPT1_dist_index] = min(close_point_WPT1);i

% [WPT2_dist_min_2d, WPT2_dist_index_2d] = min(close_point_WPT2_2d);

[WPT3_dist_min_2d, WPT3_dist_index_2d] = min(close_point_WPT3_2d);

[WPT4_dist_min_2d, WPT4_dist_index_2d] = min(close_point_WPT4_2d);

[WPT5_dist_min_2d, WPT5_dist_index_2d] = min(close_point_WPT5_2d(WPT4_dist_index_2d+1:WPT4_dist_index_2d+100));
WPT5_index_in_original = find(close_point_WPT5_2d == WPT5_dist_min_2d, 1);
WPT5_dist_index_2d = WPT5_index_in_original;

[WPT6_dist_min_2d, WPT6_dist_index_2d] = min(close_point_WPT6_2d);

[Land_dist_min_2d, Land_dist_index_2d] = min(close_point_Land(length(lat)-50:length(lat)));
Land_index_in_original = find(close_point_Land == Land_dist_min_2d, 1);
Land_dist_index_2d=Land_index_in_original;

%% Find index ( 3D )

[WPT1_dist_min_3d, WPT1_dist_index_3d] = min(close_point_WPT1_3d);
WPT1_index_in_original = find(close_point_WPT1_3d == WPT1_dist_min_3d, 1);
WPT1_dist_index_3d = WPT1_index_in_original;

[WPT2_dist_min_2d, WPT2_dist_index_2d] = min(close_point_WPT2_2d(WPT1_dist_index_3d+1:WPT3_dist_index_2d-1));
WPT2_index_in_original = find(close_point_WPT2_2d == WPT2_dist_min_2d, 1);
WPT2_dist_index_2d = WPT2_index_in_original;

[WPT3_dist_min_3d, WPT3_dist_index_3d] = min(close_point_WPT3_3d);
WPT3_index_in_original = find(close_point_WPT3_3d == WPT3_dist_min_3d, 1);
WPT3_dist_index_3d = WPT3_index_in_original;


% [WPT3_dist_min_3d, WPT3_dist_index_3d] = min(close_point_WPT3_3d(WPT2_dist_index_2d+1:WPT4_dist_index_2d-1));
% WPT3_index_in_original = find(close_point_WPT3_3d == WPT3_dist_min_3d, 1);
% WPT3_dist_index_3d = WPT3_index_in_original;

[WPT5_dist_min_3d, WPT5_dist_index_3d] = min(close_point_WPT5_3d);
WPT5_index_in_original = find(close_point_WPT5_3d == WPT5_dist_min_3d, 1);
WPT5_dist_index_3d = WPT5_index_in_original;

[WPT6_dist_min_3d, WPT6_dist_index_3d] = min(close_point_WPT6_3d(WPT5_dist_index_3d+1:length(lat)));
WPT6_index_in_original = find(close_point_WPT6_3d == WPT6_dist_min_3d, 1);
WPT6_dist_index_3d = WPT6_index_in_original;




%% Fine WPT num
WPT_flag = [];
WPT_flag(1:WPT1_dist_index_3d) = 1; 
WPT_flag(WPT1_dist_index_3d+1:WPT2_dist_index_2d) = 2; 
WPT_flag(WPT2_dist_index_2d+1:WPT3_dist_index_3d) = 3;
WPT_flag(WPT3_dist_index_3d+1:WPT4_dist_index_2d) = 4;
WPT_flag(WPT4_dist_index_2d+1:WPT5_dist_index_3d) = 5; 
WPT_flag(WPT5_dist_index_3d+1:WPT6_dist_index_3d) = 6;
WPT_flag(WPT6_dist_index_3d+1:length(lat)) = 0;


% WPT_flag = [];
% WPT_flag(1:Base_dist_index_2d) = 1;
% WPT_flag(Base_dist_index_2d+1:WPT1_dist_index_2d-1) = 2;
% WPT_flag(WPT1_dist_index_2d) = 3;
% WPT_flag(WPT1_dist_index_2d+1:WPT2_dist_index_2d-1) = 4;
% WPT_flag(WPT2_dist_index_2d) = 5;
% WPT_flag(WPT2_dist_index_2d+1:WPT3_dist_index_2d-1) = 6;
% WPT_flag(WPT3_dist_index_2d) = 7;
% WPT_flag(WPT3_dist_index_2d+1:WPT4_dist_index_2d-1) = 8;
% WPT_flag(WPT4_dist_index_2d) = 9;
% WPT_flag(WPT4_dist_index_2d+1:WPT5_dist_index_3d-1) = 10;
% WPT_flag(WPT5_dist_index_3d) = 11;
% WPT_flag(WPT5_dist_index_3d+1:WPT6_dist_index_3d-1) = 12;
% WPT_flag(WPT6_dist_index_3d) = 13;
% WPT_flag(WPT6_dist_index_3d+1:Land_dist_index_2d-1) = 14;
% WPT_flag(Land_dist_index_2d:length(time_real_utc_korea)) =15;


%%  Print result
WPT_flag = WPT_flag.';    % .' mean 전치행렬(tranpose)
alt = zD.';

% 결과를 저장할 문자열 배열 초기화
lat_format = strings(length(lat), 1); 
lon_format = strings(length(lon), 1);
alt_format = strings(length(alt), 1);

for i=1:length(lat)
    lat_format(i) = sprintf('%.6f', lat(i));
    lon_format(i) = sprintf('%.6f', lon(i));
    alt_format(i) = sprintf('%.1f',alt(i));
end

% 문자열을 다시 숫자로 변환
lat_converted = str2double(lat_format);
lon_converted = str2double(lon_format);
alt_converted = str2double(alt_format);



flight_data_table = table(flight_type, WPT_flag, time_real_gps, lat_converted, lon_converted, alt_converted);
flight_data_table.flight_type = string(flight_type);
flight_data_table.WPT_flag = string(WPT_flag);
flight_data_table.Properties.VariableNames("flight_type") = "Auto,Manual Flag";
flight_data_table.Properties.VariableNames("WPT_flag") = "Event Flag";
flight_data_table.Properties.VariableNames("time_real_gps") = "GPS Time";
flight_data_table.Properties.VariableNames("lat_converted") = "Latitude (deg)";
flight_data_table.Properties.VariableNames("lon_converted") = "Longitude (deg)";
flight_data_table.Properties.VariableNames("alt_converted") = "Altitude (m)";


% flight_data_table = table(flight_type, lat_converted, lon_converted, alt_converted, year, month, day, hour, minute, second, m_second, WPT_flag);
% flight_data_table.flight_type = string(flight_type);
% flight_data_table.WPT_flag = string(WPT_flag);
% flight_data_table.Properties.VariableNames("flight_type") = "Auto,Manual";
% flight_data_table.Properties.VariableNames("lat_converted") = "Latitude (deg)";
% flight_data_table.Properties.VariableNames("lon_converted") = "Longitude (deg)";
% flight_data_table.Properties.VariableNames("alt_converted") = "Altitude (m)";
% flight_data_table.Properties.VariableNames("year") = "UTC Year";
% flight_data_table.Properties.VariableNames("month") = "UTC Month";
% flight_data_table.Properties.VariableNames("day") = "UTC Day";
% flight_data_table.Properties.VariableNames("hour") = "UTC Hour";
% flight_data_table.Properties.VariableNames("minute") = "UTC Min";
% flight_data_table.Properties.VariableNames("second") = "UTC Sec";
% flight_data_table.Properties.VariableNames("m_second") = "UTC ms";
% flight_data_table.Properties.VariableNames("WPT_flag") = "WPT number";

writetable(flight_data_table, 'data_Mechatron.csv', 'Delimiter', 'comma');

%% Create Struct ( for horizon score )

% WPT6_dist_min_2d = 
% score.base_hori = Base_dist_min_2d;
score.wpt1_hori = WPT1_dist_min_2d;
score.wpt2_hori = WPT2_dist_min_2d;
score.wpt3_hori = WPT3_dist_min_2d;
score.wpt4_hori = WPT4_dist_min_2d;
score.wpt5_hori = WPT5_dist_min_2d;
score.wpt6_hori = WPT6_dist_min_2d;
% score.land = Land_dist_index;

save('score.mat','score')



%% Create Sturct ( for vertical score )

index.wpt1_verti = WPT1_dist_index_3d;
index.wpt2_verti = WPT2_dist_index_2d;
index.wpt3_verti = WPT3_dist_index_3d;
index.wpt4_verti = WPT4_dist_index_2d;
index.wpt5_verti = WPT5_dist_index_3d;
index.wpt6_verti = WPT6_dist_index_3d;

save('index.mat','index')

%% Create Struct ( for enu altitude )

enu_alt = zD;
save('enu_alt.mat','enu_alt');


