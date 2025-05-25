function [heave,roll,pitch] = swash(s_1_top, s_1_bot,s_2_top, s_2_bot,s_3_top, s_3_bot)

    % Should return the collective pitch, roll angle and yaw angle

    % S.one_top = position of top servo 1
    % S.one_top = position of top servo 2
    % S.one_top = position of top servo 3
    % S.one_bottom = position of top servo 1
    % S.one_bottom = position of top servo 2
    % S.one_bottom = position of top servo 3

    % Returns the collective, theta_roll, theta_pitch

    heave = mean([s_1_top, s_1_bot]);
    roll = mean([s_2_top, s_2_bot]);
    pitch = mean([s_3_top, s_3_bot]);




end