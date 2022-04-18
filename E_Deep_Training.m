clc
close all
clear

%%

fdyn_data = readmatrix("dataset/fdyn_data.csv");
idyn_data = readmatrix("dataset/idyn_data.csv");

% size(fdyn_data)
% size(idyn_data)

trainset_fdyn = fdyn_data(1:5000,:); % Testing
trainset_idyn = idyn_data(1:5000,:);

testset_fdyn = fdyn_data(5001:end,:); % Testing

size(trainset_fdyn)
%%
% trainnet_fdyn = fitnet(16);
train_algo = 'trainbfg'
trainnet_fdyn = fitnet([32,64,128,32], train_algo);
trainnet_idyn = fitnet([32,64,128,32], train_algo);

epochs = 5000;
goal = 0;

trainnet_fdyn.trainParam.epochs= epochs;
trainnet_fdyn.trainParam.goal = goal;
trainnet_idyn.trainParam.epochs= epochs;
trainnet_idyn.trainParam.goal = goal;
% trainnet_fdyn = feedforwardnet(10, 'trainlm');

% view(trainnet_fdyn)
%%
% data = [path(:x3) tau(:x3)] 
% net_fdyn = train(trainnet_fdyn,path', Tau');
% net_fdyn = train(trainnet_fdyn, trainset_fdyn(:,1:3)', trainset_fdyn(:,4:end)','useGpu','yes','useParallel','yes','showResources','yes');
net_idyn = train(trainnet_idyn, trainset_idyn(:,1:3)', trainset_idyn(:,4:end)','useGpu','yes','useParallel','yes','showResources','yes');
%%
% net = trainNetwork(Q, path,layers,options);
%
% save net_fdyn
% save net_idyn

% load net_fdyn

predict_tau_fdyn = net_fdyn(testset_fdyn(:,1:3)')'
perf_fdyn = perform(net_fdyn, predict_tau_fdyn', testset_fdyn(:,4:end)');
disp(perf_fdyn);
%%
error_fdyn = abs(predict_tau_fdyn-testset_fdyn(:,4:end));
figure(1)
for i=1:3
    subplot(3,1,i)
    plot(error_fdyn(:, i))
    if i == 1; title("algorithm = " + train_algo + ",  epochs = " + string(epochs)); end
    ylabel(["\theta_" + string(i)])
end
saveas(1,"train_resuls.svg","svg")
% predicted_Theta_1 = networkModel_1(test_partition_1(:,1:3)');
% perf_1 = perform(networkModel_1, predicted_Theta_1 ,test_partition_1(:,4)');
% disp(perf_1);

genFunction(net_fdyn, 'fdynNet');
genFunction(net_idyn, 'idynNet');
