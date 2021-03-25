%% Main function to generate tests
function tests = test_robotConstraints
tests = functiontests(localfunctions);
end

%% Test Functions
function testAnnihilatorNull(testCase)
    global param
    [Bc,Bcp] = robotConstraints(param);
    residual = Bcp*Bc;
    actual = norm(residual);
    expected = 0;
    assertEqual(testCase, actual, expected, 'AbsTol', 1e-12);
end

function testAnnihilatorFullRank(testCase)
    global param
    [~,Bcp] = robotConstraints(param);
    actual = rank(Bcp);
    expected = min(size(Bcp));
    assertEqual(testCase, actual, expected, 'AbsTol', 1e-12);
end

%% Optional file fixtures  
function setupOnce(testCase)  % do not change function name
    global param
    param.l = 42;
    addpath ../
end

function teardownOnce(testCase)  % do not change function name
    rmpath ../
    clearvars -global param
end

%% Optional fresh fixtures  
function setup(testCase)  % do not change function name
end

function teardown(testCase)  % do not change function name
end