function [sol_2, FLAG_terminate] = ROB535_ControlProject_part2_Team4(TestTrack,Xobs_seen,curr_state)
    center = [TestTrack.cline, [1480;840]];

    right = [TestTrack.br, [1490;840]];
    track.br = right;

    left = [TestTrack.bl, [1470;840]];
    track.bl = left;

    theta = TestTrack.theta ;
    theta = [theta theta(end)];
    track.theta = theta;

    last_state = curr_state;
    vel_err = 0;
    T = 0:0.01:0.5;

    for i = 1:length(T)
        % initialize the track to be at center for each iteration and then
        % choose the expected lane to avoid obstacle
        track.cline = center;
        expected_lane = ChangeLane(track, curr_state, Xobs_seen);
        track.cline = expected_lane;
    
        % calculate desired Fx and delta values
        [Fx, vel_err] = forward(curr_state, vel_err);
        [delta, FLAG_terminate] = direction(track, curr_state);

        % call forwardIntegrate it to estimate the next state
        [next, ~] = forwardIntegrateControlInput([delta Fx; delta Fx], last_state');
        last_state = next(end,:)';

        sol_2(i, :) = [delta Fx];
    end
end

function [fx, vel_err] = forward(state, vel_err)
    % Estimate the required velocity
    d_head = abs(state(6));
    vel_req = mapfun(d_head, 0.0, 0.3, 20, 5);
    
    % Calculate error in velocity
    err = vel_req - state(2);
    vel_err = [vel_err err];
    
    % Estimate the fx value and clamp it to the bounds
    fx = sat(20 * err, -5000, 5000);
end

function [delta, FLAG_terminate] = direction(track, state)
    % calculating heading error
    pos = [state(1) + 1.35 * cos(state(5)); state(3) + 1.35 * sin(state(5))];
    [head, cross_err, ~, FLAG_terminate] = whereIscar(track, state, pos);
    error = head - state(5);

    % control law
    delta = error + atan2(2.5 * cross_err, (1 + state(2)));
    delta = wrap(delta);
    delta = sat(delta, -0.5, 0.5);
end

function [desired_heading, cross_err, side, FLAG_terminate] = whereIscar(track, state, curr_pos)

    % initial definitions 
    center = track.cline;   
    theta = track.theta;    

    % calculate heading error
    [idx, d1] = knnsearch(center', [curr_pos(1) curr_pos(2)]);
    flag = "go"; 
    
    % Finding the crosstrack error
    if idx == length(center(1,:)) 
        FLAG_terminate = 1;     
        d2 =  sqrt((curr_pos(1) - center(1, idx - 1))^2 + (curr_pos(2) -  center(2, idx - 1))^2);  
        desired_heading = theta(idx);                           
        cross_err = calculateError([center(1, idx - 1); center(2, idx - 1)], [center(1, idx); center(2, idx)], d1, d2);           
        if ~isreal(cross_err)   
                cross_err = 0;  
        end
        side = RelObsPos(idx, center', state);   % determining the side of the track the car is on
        if side(3) > 0 % if greater than zero, the car is on the left side
            cross_err = -cross_err; % negate the crosstrack error
        end
    else
        % getting the heading
        FLAG_terminate = 0; % binary flag is zero since we haven't finished yet
        d2 =  sqrt((curr_pos(1) - center(1, idx + 1))^2 + (curr_pos(2) -  center(2, idx + 1))^2); 
        if idx ~= 1 && idx ~= length(center(1,:))  
            [idx_check, d2] = knnsearch([center(:, idx - 1)'; center(:, idx + 1)'], [curr_pos(1) curr_pos(2)]); 
            if idx_check == 1   
                flag = "after"; % first index means we are behind the closest point
                desired_heading = theta(idx); 
            else
                flag = "before"; % second index means that we are in front of the closest point
                desired_heading = theta(idx + 1); 
            end
        else
            desired_heading = theta(idx); 
        end
    
        % calculate crosstrack error
        if flag == "before"  || flag == "go"
            cross_err = calculateError([center(1, idx); center(2, idx)], [center(1, idx + 1); center(2, idx + 1)], d1, d2);    % computing the cross_err
            if ~isreal(cross_err)   % checking if the crosstrack error is real
                cross_err = 0;  % setting the crosstrack error to zero if it's not real
            end
            side = RelObsPos(idx, center', state);  
            if side(3) > 0 
                cross_err = -cross_err; 
            end
        end
    
        if flag == "after"
            cross_err = calculateError([center(1, idx - 1); center(2, idx - 1)], [center(1, idx); center(2, idx)], d1, d2);    % computing the crosstrack error
            if ~isreal(cross_err)  
                cross_err = 0; 
            end
            side = RelObsPos(idx, center', state); 
            if side(3) > 0 
                cross_err = -cross_err; 
            end
        end
    end
end

function side = RelObsPos(id, pt_c, state)
    if id <= length(pt_c) - 1
        side = cross([pt_c(id+1, 1) - pt_c(id, 1), pt_c(id+1, 2) - pt_c(id, 2), 0], ...
            [state(1) - pt_c(id, 1), state(3) - pt_c(id, 2), 0]);
    else
        side = [0 0 0];
    end
end

function new_angle = wrap(angle)
    new_angle = wrapToPi(angle);
end

function var = sat(var, amin, amax)
    var = max(min(var, amax), amin);
end

function cross_err = calculateError(point1, point2, d1, d2)
        assert(d1 > 0, "d1: distance can not be < 0");
        assert(d2 > 0, "d2: distance can not be < 0");
        dist = sqrt((point2(1) - point1(1))^2 + (point2(2) - point1(2))^2);
        perim = (d1 + d2 + dist) / 2;
        cross_err = 2 * sqrt(perim * (perim - d1) * (perim - d2) * (perim - dist)) / dist;
end

function output = mapfun(value,fL,fH,tL,tH)
    % Maps x from the range [in_min, in_max] to the range [out_min, out_max]
    output = (value - fL) .* (tH - tL) ./ (fH - fL) + tL;
end

function lane = ChangeLane(track, state, Xobs_seen)
    state = state';
    center = track.cline;
    right = track.br;
    left = track.bl;
    lane = center;

    mid_r = (right + center)/2;
    mid_l = (left + center)/2;

    next_obs =[];
    for i=1:length(Xobs_seen)
        Obs = mean(Xobs_seen{i});
        est1 = state(1) + 10*cos(state(5));
        est2 = state(3) + 10*sin(state(5));
        p1 = [est1, est2, 0];
        p2 = [state(1), state(3), 0];
        p3 = [Obs(1), Obs(2), 0];

        var1 = norm(cross(p1 - p2, p3 - p2));
        var2 = dot(p1 -p2, p3 - p2);
        theta = atan2(var1, var2);
        
        if abs(theta) < pi/2
          next_obs = [next_obs; [norm([state(1), state(3)] - [Obs(1), Obs(2)]), Obs(1), Obs(2), est1, est2, theta]];
        end
    end
            
    if(next_obs)
        [~,idx] = min(next_obs(:,1));
        obs_next = next_obs(idx, 2:3);
        [cidx ,~] = knnsearch(center', obs_next);
        obs_next(3) = next_obs(idx, 3);
        obs_pos = RelObsPos(cidx, center', obs_next);
        
        lane = mid_l';
        if (obs_pos(3) >= 0)
            lane = mid_r';
        end
    end
    if size(lane, 1) ~= 2
        lane = lane';
    end
end



   