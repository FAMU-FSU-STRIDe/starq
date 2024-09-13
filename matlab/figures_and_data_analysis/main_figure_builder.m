clc

close all

% a script for processing onr data and getting good figures for ICRA 2025
% submission
% started by Derek Vasquez

%% Loading data
WalkingData = struct;

WalkingData.buoy.f0p1.files = ["TrackerResults/Buoyant/WalkBuoy_0p1_T1.txt"...
    "TrackerResults/Buoyant/WalkBuoy_0p1_T2.txt"...
    "TrackerResults/Buoyant/WalkBuoy_0p1_T3.txt"];

WalkingData.buoy.f0p2.files = ["TrackerResults/Buoyant/WalkBuoy_0p2_T1.txt"...
    "TrackerResults/Buoyant/WalkBuoy_0p2_T2.txt"...
    "TrackerResults/Buoyant/WalkBuoy_0p2_T3.txt"];

WalkingData.buoy.f0p5.files = ["TrackerResults/Buoyant/WalkBuoy_0p5_T1.txt"...
    "TrackerResults/Buoyant/WalkBuoy_0p5_T2.txt"...
    "TrackerResults/Buoyant/WalkBuoy_0p5_T3.txt"];

WalkingData.buoy.f0p25.files = ["TrackerResults/Buoyant/WalkBuoy_0p25_T1.txt"...
    "TrackerResults/Buoyant/WalkBuoy_0p25_T2.txt"...
    "TrackerResults/Buoyant/WalkBuoy_0p25_T3.txt"];

WalkingData.buoy.f0p35.files = ["TrackerResults/Buoyant/WalkBuoy_0p35_T1.txt"...
    "TrackerResults/Buoyant/WalkBuoy_0p35_T2.txt"...
    "TrackerResults/Buoyant/WalkBuoy_0p35_T3.txt"];


WalkingData.heavy.f0p5.files = ["TrackerResults/Heavy/Walk0p5_OldGains_Test1"...
    "TrackerResults/Heavy/Walk0p5_OldGains_Test2"...
    "TrackerResults/Heavy/Walk0p5_OldGains_Test3"];

WalkingData.heavy.f0p75.files = ["TrackerResults/Heavy/Walk0p75_OldGains_Test1"...
    "TrackerResults/Heavy/Walk0p75_OldGains_Test2"...
    "TrackerResults/Heavy/Walk0p75_OldGains_Test3"];

WalkingData.heavy.f1p0.files = [...
    % "TrackerResults/Heavy/Walk1p0_OldGains_Test1"...
    "TrackerResults/Heavy/Walk1p0_OldGains_Test2"...
    "TrackerResults/Heavy/Walk1p0_OldGains_Test3"...
    "TrackerResults/Heavy/Walk1p0_OldGains_Test1_Redo"];

WalkingData.heavy.f1p5.files = ["TrackerResults/Heavy/Walk1p5_OldGains_Test1"...
    "TrackerResults/Heavy/Walk1p5_OldGains_Test2"...
    "TrackerResults/Heavy/Walk1p5_OldGains_Test3"];

WalkingData.heavy.f1p25.files = ["TrackerResults/Heavy/Walk1p25_OldGains_Test1"...
    "TrackerResults/Heavy/Walk1p25_OldGains_Test2"...
    "TrackerResults/Heavy/Walk1p25_OldGains_Test3"];

JumpingData.buoy.f0p1.files = [...
    "TrackerResults/Buoyant/JumpBuoy_0p1_T1"...
    "TrackerResults/Buoyant/JumpBuoy_0p1_T2"...
    "TrackerResults/Buoyant/JumpBuoy_0p1_T3"...
    ];

JumpingData.buoy.f0p15.files = [...
    "TrackerResults/Buoyant/JumpBuoy_0p15_T1"...
    "TrackerResults/Buoyant/JumpBuoy_0p15_T2"...
    "TrackerResults/Buoyant/JumpBuoy_0p15_T3"...
    ];

JumpingData.buoy.f0p2.files = [...
    "TrackerResults/Buoyant/JumpBuoy_0p2_T1"...
    "TrackerResults/Buoyant/JumpBuoy_0p2_T2"...
    "TrackerResults/Buoyant/JumpBuoy_0p2_T3"...
    ];

JumpingData.buoy.f0p25.files = [...
    "TrackerResults/Buoyant/JumpBuoy_0p25_T1"...
    "TrackerResults/Buoyant/JumpBuoy_0p25_T2"...
    "TrackerResults/Buoyant/JumpBuoy_0p25_T3"...
    ];

JumpingData.buoy.f0p4.files = [...
    "TrackerResults/Buoyant/JumpBuoy_0p4_T1"...
    "TrackerResults/Buoyant/JumpBuoy_0p4_T2"...
    "TrackerResults/Buoyant/JumpBuoy_0p4_T3"...
    ];

JumpingData.buoy.f0p5.files = [...
    "TrackerResults/Buoyant/JumpBuoy_0p5_T1"...
    "TrackerResults/Buoyant/JumpBuoy_0p5_T2"...
    "TrackerResults/Buoyant/JumpBuoy_0p5_T3"...
    ];

JumpingData.buoy.f0p75.files = [...
    % "TrackerResults/Buoyant/JumpBuoy_0p75_T1"...
    "TrackerResults/Buoyant/JumpBuoy_0p75_T2"...
    "TrackerResults/Buoyant/JumpBuoy_0p75_T3"...
    ];

JumpingData.heavy.f0p5.files = [...
    "TrackerResults/Heavy/Jump0p5_75G_Test1"...
    "TrackerResults/Heavy/Jump0p5_75G_Test2"...
    "TrackerResults/Heavy/Jump0p5_75G_Test3"...
    ];

JumpingData.heavy.f0p25.files = [...
    "TrackerResults/Heavy/Jump0p25_75G_Test1"...
    "TrackerResults/Heavy/Jump0p25_75G_Test2"...
    "TrackerResults/Heavy/Jump0p25_75G_Test3"...
    ];

JumpingData.heavy.f0p75.files = [...
    "TrackerResults/Heavy/Jump0p75_75G_Test1"...
    "TrackerResults/Heavy/Jump0p75_75G_Test2"...
    "TrackerResults/Heavy/Jump0p75_75G_Test3"...
    ];

JumpingData.heavy.f1p0.files = [...
    % "TrackerResults/Heavy/Jump1p0_75G_Test1"...
    "TrackerResults/Heavy/Jump1p0_75G_Test2"...
    "TrackerResults/Heavy/Jump1p0_75G_Test3"...
    ];

SwimmingData.v1a.files = [...
    "TrackerResults/Swimming/Swim_V1a_T1"...
    "TrackerResults/Swimming/Swim_V1a_T2"...
    % "TrackerResults/Swimming/Swim_V1a_T3"...
    ];

SwimmingData.v1b.files = [...
    "TrackerResults/Swimming/Swim_V1b_T1"...
    "TrackerResults/Swimming/Swim_V1b_T2"...
    ];

SwimmingData.v1cb.files = [...
    "TrackerResults/Swimming/Swim_V1CB_T1"...
    ];


%% Processing
buoyWalkingFields = fieldnames(WalkingData.buoy);
for i = 1:length(buoyWalkingFields)
    WalkingData.buoy.(buoyWalkingFields{i}) = getAvgVels(WalkingData.buoy.(buoyWalkingFields{i}));
end

heavyWalkingFields = fieldnames(WalkingData.heavy);
for i = 1:length(heavyWalkingFields)
    WalkingData.heavy.(heavyWalkingFields{i}) = getAvgVels(WalkingData.heavy.(heavyWalkingFields{i}));
end

% getting walking speeds
buoyWalkingFreqs = [0.1 0.2 0.25 0.35 0.5];
heavyWalkingFreqs = [ 0.5 0.75 1.0 1.25 1.5];

sh = WalkingData.buoy; % sh for shorthand
buoyWalkingVels = [sh.f0p1.avgVelOverall...
    sh.f0p2.avgVelOverall...
    sh.f0p25.avgVelOverall...
    sh.f0p35.avgVelOverall...
    sh.f0p5.avgVelOverall];

sh = WalkingData.heavy;
heavyWalkingVels = [sh.f0p5.avgVelOverall...
    sh.f0p75.avgVelOverall...
    sh.f1p0.avgVelOverall...
    sh.f1p25.avgVelOverall...
    sh.f1p5.avgVelOverall];

figure('Name',"ComparingWalkingVels")
hold on
plot(buoyWalkingFreqs,buoyWalkingVels,'bo--','LineWidth',3)
plot(heavyWalkingFreqs,heavyWalkingVels,'ko--','LineWidth',3)
xlabel("Gait Frequency (Hz)")
ylabel("Walking speed (m/s)")
legend("Increased Buoyancy","Nominal Weight","Location","northwest")
ylim([0 0.16])
xlim([0 1.6])
FormatPlot(gca)

% getting jump heights
buoyJumpingFields = fieldnames(JumpingData.buoy);
for i = 1:length(buoyJumpingFields)
    JumpingData.buoy.(buoyJumpingFields{i}) = getMaxHeight(JumpingData.buoy.(buoyJumpingFields{i}));
end

heavyJumpingFields = fieldnames(JumpingData.heavy);
for i = 1:length(heavyJumpingFields)
    JumpingData.heavy.(heavyJumpingFields{i}) = getMaxHeight(JumpingData.heavy.(heavyJumpingFields{i}));
end

buoyJumpingFreqs = [0.1 0.15 0.2 0.25 0.4 0.5 0.75];
heavyJumpingFreqs = [ 0.25 0.5 0.75 1.0];

sh = JumpingData.buoy; % sh for shorthand
buoyJumpHeights = [sh.f0p1.meanMaxHeight...
    sh.f0p15.meanMaxHeight...
    sh.f0p2.meanMaxHeight...
    sh.f0p25.meanMaxHeight...
    sh.f0p4.meanMaxHeight...
    sh.f0p5.meanMaxHeight...
    sh.f0p75.meanMaxHeight];

sh = JumpingData.heavy;
heavyJumpHeights = [sh.f0p25.meanMaxHeight...
    sh.f0p5.meanMaxHeight...
    sh.f0p75.meanMaxHeight...
    sh.f1p0.meanMaxHeight];

figure('Name',"ComparingJumpHeights")
hold on
plot(buoyJumpingFreqs,buoyJumpHeights,'bo--','LineWidth',3)
plot(heavyJumpingFreqs,heavyJumpHeights,'ko--','LineWidth',3)
xlabel("Gait Frequency (Hz)")
ylabel("Jump Apex Height (m)")
legend("Increased Buoyancy","Nominal Weight","Location","northeast")
xlim([0 1.1])
ylim([0 0.3])
FormatPlot(gca)

% swimming
swimmingFields = fieldnames(SwimmingData);
for i = 1:length(swimmingFields)
    SwimmingData.(swimmingFields{i}) = getAvgVels(SwimmingData.(swimmingFields{i}));
end

swimAreas = [1 2 3];


sh = SwimmingData; % sh for shorthand
swimmingVels = [sh.v1a.avgVelOverall...
    sh.v1b.avgVelOverall...
    sh.v1cb.avgVelOverall...
    ];

figure('Name',"Swimming")
hold on
plot(swimAreas,swimmingVels,'ko--','LineWidth',3)
xlabel("Total Fin Area (mm^2)")
ylabel("Swimming speed (m/s)")
% ylim([0 0.16])
xlim([0 4])
FormatPlot(gca)

function dataStruct = getAvgVels(dataStruct)
avgVels = [];
for i = 1:length(dataStruct.files)
    data = readmatrix(dataStruct.files(i));

    t1 = data(:,1); %time in s
    x1 = data(:,2); %x direction in ft
    y1 = data(:,3); %y direction in ft
    r1 = data(:,4); %relative postion in ft
    vx1 = data(:,5); %x velocity in ft/s
    vy1 = data(:,6); %y velocity in ft/s
    v1 = data(:,7); %relative velocity in ft/s
    ax1 = data(:,8);
    ay1 = data(:,9);
    a1 = data(:,10);

    avg1 = mean(v1(2:end-1),'omitnan');
    if isnan(avg1)
        keyboard
    end
    if avg1<0
        keyboard
    end
    avgVels = [avgVels avg1];
end
if isnan(mean(avgVels))
    keyboard
end
dataStruct.avgVels = avgVels;
dataStruct.avgVelOverall = mean(avgVels);

end

function FormatPlot(ax)
ax.FontSize = 12;
ax.Title.FontSize = 18;
ax.XLabel.FontSize = 16;
ax.YLabel.FontSize = 16;
grid on
ax.GridLineStyle = "--";
ax.LineWidth = 1.5;
end

function dataStruct = getMaxHeight(dataStruct)
avgHeight = [];
for i = 1:length(dataStruct.files)
    data = readmatrix(dataStruct.files(i));

    t1 = data(:,1); %time in s
    x1 = data(:,2); %x direction in ft
    y1 = data(:,3); %y direction in ft
    r1 = data(:,4); %relative postion in ft
    vx1 = data(:,5); %x velocity in ft/s
    vy1 = data(:,6); %y velocity in ft/s
    v1 = data(:,7); %relative velocity in ft/s
    ax1 = data(:,8);
    ay1 = data(:,9);
    a1 = data(:,10);

    height = max(y1);
    if isnan(height)
        keyboard
    end
    if height<0
        keyboard
    end
    avgHeight = [avgHeight height];
end
if isnan(mean(avgHeight))
    keyboard
end
dataStruct.heights = avgHeight;
dataStruct.meanMaxHeight = mean(avgHeight);

end