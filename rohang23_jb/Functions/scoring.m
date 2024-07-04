clc; clear; close all;

addpath("C:\Users\s_cowpqja3302\Desktop\rohang_jb\Functions");
addpath('C:\Users\s_cowpqja3302\Desktop\rohang_jb\test')     %Edit before run

load('score.mat');
load('index.mat');
load('enu_alt.mat');

%% Horizon Score 
score_hori = [score.wpt1_hori, score.wpt2_hori, score.wpt3_hori, ...
              score.wpt4_hori, score.wpt5_hori, score.wpt6_hori];

score_h = 0; % 초기화된 test 생성
score_v = 0;

for i = 1:length(score_hori)
    if (score_hori(i) < 2)
        score_h = score_h + 10;

    elseif (2 <= score_hori(i) && score_hori(i) < 4)
        score_h = score_h + 6;

    elseif (4 <= score_hori(i) && score_hori(i) < 6)
        score_h = score_h + 3;

    else 
        score_h = score_h + 0;
    end
end

%% Vertical Score 

score_verti = [enu_alt(index.wpt1_verti), enu_alt(index.wpt2_verti), enu_alt(index.wpt3_verti), ...
               enu_alt(index.wpt4_verti), enu_alt(index.wpt5_verti), enu_alt(index.wpt6_verti)];


for i = 1:3
    score_verti(i) = abs(score_verti(i) - 50);
end
score_verti(4) = score_verti(4) - 10;
score_verti(5) = score_verti(5) - 10;
score_verti(6) = score_verti(6) - 50; % 얘는 고도가 60m로 날아서

for i = 1:length(score_verti)
    if (score_verti(i) < 4)
        score_v = score_v + 10;

    elseif (4 <= score_verti(i) && score_verti(i) < 8)
        score_v = score_v + 6;

    elseif (8 <= score_verti(i) && score_verti(i) < 12)
        score_v = score_v + 3;

    else 
        score_v = score_v + 0;
    end
end

%% Result

score = score_h + score_v;

