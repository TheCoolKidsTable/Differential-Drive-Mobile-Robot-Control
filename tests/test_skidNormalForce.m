%% Main function to generate tests
function tests = test_skidNormalForce
tests = functiontests(localfunctions);
end

%% Test Functions
function testSkidForcePositive(testCase)
    global param
    actual = skidNormalForce(param);
    floor = 0;
    assertGreaterThan(testCase, actual, floor);
end

function testSkidForceLessThanTotalWeight(testCase)
    global param
    actual = skidNormalForce(param);
    ceiling = param.m*param.g;
    assertLessThan(testCase, actual, ceiling);
end

%% Optional file fixtures  
function setupOnce(testCase)  % do not change function name
    addpath ../
    global param
    param = robotParameters();
end

function teardownOnce(testCase)  % do not change function name
    clearvars -global param
    rmpath ../
end

%% Optional fresh fixtures  
function setup(testCase)  % do not change function name
end

function teardown(testCase)  % do not change function name
end