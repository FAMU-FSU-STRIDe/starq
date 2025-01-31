clc
clear
close all

traj = readmatrix("walk_test_2.txt");

x = traj(1:500,5);
z = traj(1:500,7);

traj = [x,z];
freq = 2.5; %Hz

ldesFunc(0.000,traj,freq)
qBdesFunc(0,traj,freq)

function ldes = ldesFunc(t,traj,freq)
    idx = ceil(t*length(traj(:,1))*freq);
    if(idx==0)
        idx = 1;
    end
    xdes = traj(idx,1);
    zdes = traj(idx,2);
    ldes = sqrt(xdes^2+zdes^2);
end

function qBdes = qBdesFunc(t,traj,freq)
    idx = ceil(t*length(traj(:,1))*freq);
    if(idx==0)
        idx = 1;
    end
    xdes = traj(idx,1);
    zdes = traj(idx,2);
    qBdes = atan2(zdes,xdes);
end

function dldes = dldesFunc(t,traj,freq)
    idx = ceil(t*length(traj(:,1))*freq);
    if(idx==0)
        idx = 1;
    end
    xdes = traj(idx,1);
    zdes = traj(idx,2);
    ldes = sqrt(xdes^2+zdes^2);

    xdes1 = traj(idx+1,1);
    zdes1 = traj(idx+1,2);
    ldes1 = sqrt(xdes1^2+zdes1^2);

    dt = 1/freq/length(traj(:,1));

    dldes = (ldes1-ldes)/dt;

end

function dqBdes = dqBdesFunc(t,traj,freq)
    idx = ceil(t*length(traj(:,1))*freq);
    if(idx==0)
        idx = 1;
    end
    xdes = traj(idx,1);
    zdes = traj(idx,2);
    qBdes = atan2(zdes,xdes);

    xdes1 = traj(idx+1,1);
    zdes1 = traj(idx+1,2);
    qBdes1 = atan2(zdes1,xdes1);

    dt = 1/freq/length(traj(:,1));
    dqBdes = (qBdes1-qBdes)/dt;
end
