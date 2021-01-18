% generate example data for motionmodel

% from docs motionmodel.cpp:
%
% reads in a csv file that describes the motion model using spline interpolation points
%          * * the first column contains the progress variable s
%          * * the subsequent columns contain for each mode, the spline corresponding to the different joints, i.e.  first mode 1 of joint 1, then  mode 1 of joint 2, then mode 1 of joint 3,....
%          *   and only then mode 2 of joint 1, etc....
%          * * the last columns contains the b vector for joint 1, joint 2 , ....
%          * * the number of modes is deduced from the number of columns in the file, as well as the number of interpolation points, which is deduced from
%          *   the number of rows.

% the modes are just sines with a phase shift for each joint and a
% frequency change for each different mode
%
nrofmodes=2;
nrofjoints=3;
nrofrows=100;

s = linspace(0.0, 1.0, nrofrows)';
basefreq = 2*pi;
phaseshift = 50/180.0*pi;
A = zeros(nrofrows,(nrofmodes+1)*nrofjoints+1);
A(:,1)=s;

% H:
for mode=1:nrofmodes
    for joint=1:nrofjoints
        A(:,1+(mode-1)*nrofjoints+joint) =  sin( basefreq*s*mode + phaseshift*(joint-1));
    end
end

% b:
A(:,nrofjoints*nrofmodes+2) = 0.0*ones(nrofrows,1);
A(:,nrofjoints*nrofmodes+3) = 0.2*ones(nrofrows,1);
A(:,nrofjoints*nrofmodes+4) = -0.2*ones(nrofrows,1);

figure(10);
subplot(2,1,1);
plot([ A(:,2);A(:,3);A(:,4)],'.')
title('mode 1 for joint 1,2 and 3');
subplot(2,1,2);
plot([ A(:,5);A(:,6);A(:,7)],'.')
title('mode 2 for joint 1,2 and 3');


dlmwrite('../motionmodel.csv',A,'delimiter',',','precision','%24.16f')