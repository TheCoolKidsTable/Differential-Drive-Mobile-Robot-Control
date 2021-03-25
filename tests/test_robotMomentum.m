%% Main function to generate tests
function tests = test_robotMomentum
tests = functiontests(localfunctions);
end

%% Test Functions
function testDontInvertMatrix(testCase)
    global param
    pr = [1;2];
    p = robotMomentum(pr,param);
    % No assertions needed since errors in the exploding fakes will fail the test
end

%% Optional file fixtures  
function setupOnce(testCase)  % do not change function name
    addpath ../
    warning('off','MATLAB:dispatcher:nameConflict');
    addpath mocks
    global param
    param = robotParameters();
end

function teardownOnce(testCase)  % do not change function name
    clearvars -global param
    rmpath mocks
    warning('on','MATLAB:dispatcher:nameConflict');
    rmpath ../
end

%% Optional fresh fixtures  
function setup(testCase)  % do not change function name
end

function teardown(testCase)  % do not change function name
end