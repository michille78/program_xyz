%% Plot -> Ìí¼Óµ½ Cpp ÖÐ

function  DrawResult_InCpp( InertialPositionCompensate )

    figure('name','InertialPositionCompensate');
    subplot(2,1,1);
    plot(InertialPositionCompensate(1,:));
    subplot(2,1,2);
    plot(InertialPositionCompensate(2,:));
