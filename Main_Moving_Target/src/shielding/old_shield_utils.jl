# shield.jl

# TO-DO: add Minkowski sum for human radius to each set
#   - Julia crashes when trying to use minkowski_sum() within shielding function
#   - gen_FRS() function works fine with mink_sum() when used separately in command line

# TO-DO: add finer time discretion using x_subpaths

function shield_action_set(x_k1, nearby_human_positions, Dt_obs_to_k1, Dt_plan, get_actions::Function, veh, human_goal_positions, v_human_max, safe_HJB_value_lim, m)
    Dv_max = m.vehicle_action_delta_speed

    # generate each human FRS sequence from t_k2 to t_stop_max
    actions_k1, ia_k1_set, _ = get_actions(x_k1, Dt_plan, veh, m)

    #Compute Maximum possible vehicle velocity
    v_k2_max = x_k1[4] + maximum(getindex.(actions_k1, 2))
    kd_max = ceil(Int, (0.0 - v_k2_max)/(-Dv_max)) - 1

    F_all_body_seq = generate_F_all_seq(nearby_human_positions, Dt_obs_to_k1, Dt_plan, v_human_max, human_goal_positions, kd_max)

    # perform reachability check on all actions in standard POMDP action set
    ia_k1_safe_set = []

    for ia_k1 in ia_k1_set
        ia_k1_safe = false

        # propagate vehicle state to state x_k2
        a_k1 = actions_k1[ia_k1]
        x_k2, _ = propagate_state(x_k1, a_k1, Dt_plan, veh)

        # calculate time needed for divert path from new state
        kd_stop = ceil(Int, (0.0 - x_k2[4])/(-Dv_max)) - 1

        # println("\nshield: ia_k1 = $ia_k1, a_k1 = $a_k1, x_k2 = $x_k2, kd_stop = $kd_stop")

        # iterate through divert steering angles
        #= Run this loop 3 times. WHY?=#
        for dpath in shuffle([1,2,3])
            dpath_safe = true

            # println("shield: a_k1 = $a_k1, dpath = $dpath")

            x_kd = x_k2
            for kd in 0:kd_stop

                #=
                ISSUE: can get stuck in HJB chokepoints
                  - POMDP leads vehicle to right of rightmost obstacle,
                      then shield says all divert states have val_x<500.0
                  - should probably lower threshold some more
                  - can do value check only on last state in divert path

                  - could just generate human-free HJB path to goal to check state validity
                      - would probably need to use POMDP state propagator, collision checker
                      - should basically be how upper bound rollouts are conducted
                          - does calculate_upper_bound() function conduct full rollout sim?
                              - yes, but set up to take m and b as inputs

                  - can use calculate_upper_bound(m, b) to assess whether goal can be reached from divert stop point
                      - have m unchanged, need to create b for x_stop state
                          - b::TreeSearchScenarioParameters() - used on line 196 in simulator.jl
                          - needs vehicle_state, vehicle_params, exp_details,

                  - might be easier to make copy of upper_bound() function that takes state as input (since don't need other stuff in b)
                  - b uses in upper_bound():
                      - if(b.depth == 100) - line 612
                      - if b.depth+i < 100 - line 622
                      - s = first(particles(b)) - line 615

                  - is s even needed, or can just pass x directly?
                      - nope, can pass x directly

                  - UBx seems to work well (for actions it actually calls)

                # check if divert stop point is in static unsafe set (OLD)
                if kd == kd_stop
                    val_x_kd = interp_value(x_kd, m.rollout_guide.value_array, m.rollout_guide.state_grid)

                    if val_x_kd < safe_HJB_value_lim
                        println("shield: ia_k1 = $ia_k1, dpath = $dpath, kd = $kd, x_kd = $x_kd has unsafe HJB value = $val_x_kd")
                        dpath_safe = false
                        break
                    end
                end

                (?): did a_k1 = (0.0, -delta_speed) get skipped when v_k1=0.5 m/s?
                  - did it get removed somewhere?
                  - last two divert actions also seem to be skipped (make sure not just this context)
                  - check get_actions() stuff
                  - these are the actions with -Dv, is it an issue with kd_stop?
                      - yep, kd_stop = -1 when v_k1=0.5 m/s and Dv_k1=-0.5 m/s (hmm...)

                  - kd_stop=-1 kinda makes sense
                  - when choosing action a_k1, need to check if landing spot x_k2 is safe -> check kd=0
                  - if vehicle is already stopped at t_k1, don't need to check x_k2 (since x_k2 = x_k1)
                      -  x_k1 has already been proven statically safe, since you're in it

                  - think everything is working
                      - test theta conversions by setting initial angle to something in fourth quadrant -> works
                      - test UBx collision checking when x_stop in static unsafe set -> works

                NOTE: can remove all safe_HJB_value_lim
                =#

                # check if divert stop point is in static unsafe set (NEW)
                if kd == kd_stop
                    val_x_stop = calculate_upper_bound_x(m, x_kd)
                    # println("shield: val_x_stop = ", val_x_stop)

                    if val_x_stop < 0.0
                        println("shield: ia_k1 = $ia_k1, dpath = $dpath, kd = $kd, x_kd = $x_kd has unsafe UB value = $val_x_stop")
                        dpath_safe = false
                        break
                    end
                end

                # (?): should this be checking for direct collisions with static obstacles as well?

                # check for collisions with each human
                veh_body_cir_kd = state_to_body_circle(x_kd, veh)

                humans_safe = true
                for ih in axes(nearby_human_positions, 1)
                    #=WHY 3+kd?=#
                    F_ih_body_kd = F_all_body_seq[ih][3+kd]

                    if isdisjoint(veh_body_cir_kd, F_ih_body_kd) == false
                        humans_safe = false
                        break
                    end
                end

                if humans_safe == false
                    dpath_safe = false
                    break
                end

                # propagate vehicle to next step along divert path
                #= WHY some random action is being chosen here?=#
                actions_kd, _, ia_divert_set = get_actions(x_kd, Dt_plan, veh, m)
                ia_d = ia_divert_set[dpath]
                a_d = actions_kd[ia_d]

                x_kd1, _ = propagate_state(x_kd, a_d, Dt_plan, veh)

                # pass state to next step
                x_kd = x_kd1
            end

            if dpath_safe == true
                ia_k1_safe = true
                break
            end
        end

        # action is safe
        if ia_k1_safe == true
            # println("shield: ia_k1 = $ia_k1 is safe")
            push!(ia_k1_safe_set, ia_k1)
        else
            # println("shield: ia_k1 = $ia_k1 is not safe")
        end
    end

    return actions_k1[ia_k1_safe_set], ia_k1_safe_set
end

# generate an FRS sequence for all nearby humans
function generate_F_all_seq(nearby_human_positions, Dt_obs_to_k1, Dt_plan, v_human_max, goal_positions, kd_max)
    FR_body_set_seq_all_humans = []
    # generate FRS sequences for each nearby human
    for human in nearby_human_positions
        FR_set_seq, FR_body_set_seq = generate_human_FR_seq(human, Dt_obs_to_k1, Dt_plan, v_human_max, goal_positions, kd_max)
        push!(FR_body_set_seq_all_humans, FR_body_set_seq)
    end
    return FR_body_set_seq_all_humans
end

# generate an FRS sequence for a given human position and time horizon
function generate_human_FR_seq(x_ih_obs, Dt_obs_to_k1, Dt_plan, v_human_max, goal_positions, kd_max)
    # FR_set_seq = Vector{VPolygon{Float64, SVector{2, Float64}}}()
    # FR_body_set_seq = Vector{VPolygon{Float64, SVector{2, Float64}}}()

    FR_set_seq = []
    FR_body_set_seq = []
    h_body = VPolyCircle((0.0, 0.0), 0.5)

    # create initial polygon from observed position
    human_points = [SVector(x_ih_obs)]
    FR_set = VPolygon([SVector(x_ih_obs)])
    push!(FR_set_seq, FR_set)

    FR_body_set = FR_set
    # F_ih_body_ks = minkowski_sum(F_ih_ks, h_body)
    push!(FR_body_set_seq, FR_body_set)

    # propagate set through time steps
    for ks1 in 1:(2+kd_max)
        new_human_points = Vector{SVector{2,Float64}}()
        ks1 == 1 ? Dt = Dt_obs_to_k1 : Dt = Dt_plan

        # apply all actions to each vertex of current set polygon
        for point in human_points
            for ig in axes(goal_positions, 1)
                new_point = propagate_human(point, ig, Dt, v_human_max, goal_positions)
                push!(new_human_points, SVector(new_point))
            end
        end

        # create set polygons from propagated states
        # println(length(new_human_points))
        FR_set = VPolygon(new_human_points)
        push!(FR_set_seq, FR_set)

        FR_body_set = minkowski_sum(FR_set, h_body)
        push!(FR_body_set_seq, FR_body_set)

        # pass states to next time step
        human_points = new_human_points
    end

    return FR_set_seq, FR_body_set_seq
end

function propagate_human(human_pos, goal_index, Dt, v_human, goal_positions)
    # break out current state
    xp_ih_k = human_pos[1]
    yp_ih_k = human_pos[2]

    # pull out chosen goal location
    xpg = goal_positions[goal_index].x
    ypg = goal_positions[goal_index].y

    # calculate derivative at current state
    C_x = ((xpg-xp_ih_k)^2 + (ypg-yp_ih_k)^2)^(-1/2)

    xp_ih_dot_k = v_human * C_x * (xpg-xp_ih_k)
    yp_ih_dot_k = v_human * C_x * (ypg-yp_ih_k)

    # calculate next state
    xp_ih_k1 = xp_ih_k + (xp_ih_dot_k * Dt)
    yp_ih_k1 = yp_ih_k + (yp_ih_dot_k * Dt)

    # reassemble state vector
    x_ih_k1 = (xp_ih_k1, yp_ih_k1)

    return x_ih_k1
end


function lsp_shield_action_set(x_k1, nearby_human_positions, Dt_obs_to_k1, Dt_plan, get_actions::Function,
                        veh, human_goal_positions, v_human_max, m)

    Dv_max = m.vehicle_action_delta_speed

    #Compute Maximum possible vehicle velocity
    action_set, index_action_set = get_actions(x_k1, Dt_plan, veh, m)
    v_k2_max = x_k1[4] + maximum(action_set)
    kd_max = ceil(Int, (0.0 - v_k2_max)/(-Dv_max)) - 1

    # generate each human FRS sequence from t_k2 to t_stop_max
    F_all_body_seq = generate_F_all_seq(nearby_human_positions, Dt_obs_to_k1, Dt_plan, v_human_max, human_goal_positions, kd_max)

    # perform reachability check on all actions in standard POMDP action set
    # safe_action_set = []
    index_safe_action_set = Int64[]
    path = m.rollout_guide.controls_sequence

    for id in index_action_set

        controls_sequence = deepcopy(path)
        action = action_set[id]
        action_safe = true

        # propagate vehicle state to state x_k2

        x_k2, num_segments = lsp_propagate_state(x_k1, action, Dt_plan, controls_sequence, m)
        controls_sequence = controls_sequence[num_segments+1:end]

        #Now need to check if x_k2 is a safe state or not

        #num time steps for the vehicle to come to a stop on that path
        num_steps_to_stop = ceil(Int, (0.0 - x_k2[4])/(-Dv_max)) - 1
        curr_vehicle_state = x_k2

        for step in 0:num_steps_to_stop

            # println("\nshield: ia_k1 = $ia_k1, a_k1 = $a_k1, x_k2 = $x_k2, kd_stop = $kd_stop")

            # check for collisions with each human
            veh_body_cir_kd = state_to_body_circle(curr_vehicle_state, veh)

            humans_safe = true
            for ih in axes(nearby_human_positions, 1)
                F_ih_body_kd = F_all_body_seq[ih][3+step]
                if isdisjoint(veh_body_cir_kd, F_ih_body_kd) == false
                    humans_safe = false
                    break
                end
            end

            if humans_safe == false
                action_safe = false
                break
            end

            # propagate vehicle for next time step along the A* path by decreasing its speed
            a_d = -Dv_max
            new_vehicle_state, num_segments = lsp_propagate_state(curr_vehicle_state, a_d, Dt_plan, controls_sequence, m)

            # pass state to next step
            curr_vehicle_state = new_vehicle_state
            controls_sequence = controls_sequence[num_segments+1:end]
        end

        # action is safe
        if action_safe == true
            push!(index_safe_action_set, id)
        end
    end

    return action_set,index_safe_action_set
    # return actions_k1[ia_k1_safe_set], ia_k1_safe_set
end


function lsp_propagate_state(vehicle_state, action, time_duration, controls_sequence, m)

    xpos = vehicle_state[1]
    ypos = vehicle_state[2]
    theta = vehicle_state[3]
    speed = vehicle_state[4]

    delta_speed = action
    max_vehicle_speed = m.max_vehicle_speed

    new_vehicle_speed = clamp(speed+delta_speed, 0.0, max_vehicle_speed)
    num_segments_in_one_time_step = Int(new_vehicle_speed/m.vehicle_action_delta_speed)

    time_duration_per_segment = time_duration/num_segments_in_one_time_step

    num_segments = min(num_segments_in_one_time_step, length(controls_sequence))
    current_x, current_y, current_theta = xpos, ypos, theta

    for i in 1:num_segments
        # println(s.index_vehicle_controls_sequence , " ", s.index_vehicle_controls_sequence+i-1, " ", m.rollout_guide.len)
        # println(s)
        # println("SA is : ", steering_angle)
        # if(steering_angle == 0.0)
        #     new_theta = current_theta
        #     new_x = current_x + new_vehicle_speed*cos(current_theta)*(m.one_time_step/num_segments_in_one_time_step)
        #     new_y = current_y + new_vehicle_speed*sin(current_theta)*(m.one_time_step/num_segments_in_one_time_step)
        # else
        #     new_theta = current_theta + (new_vehicle_speed * tan(steering_angle) * (m.one_time_step/num_segments_in_one_time_step) / m.vehicle_wheelbase)
        #     new_theta = wrap_between_0_and_2Pi(new_theta)
        #     new_x = current_x + ((m.vehicle_wheelbase / tan(steering_angle)) * (sin(new_theta) - sin(current_theta)))
        #     new_y = current_y + ((m.vehicle_wheelbase / tan(steering_angle)) * (cos(current_theta) - cos(new_theta)))
        # end
        steering_angle = controls_sequence[i]
        # println(s.index_vehicle_controls_sequence+i-1, " SA: ", steering_angle)
        new_x,new_y,new_theta = move_vehicle(current_x,current_y,current_theta,m.vehicle_wheelbase,
                                        steering_angle,new_vehicle_speed,time_duration_per_segment)
        # push!(vehicle_path,(new_x,new_y,new_theta))
        current_x,current_y,current_theta = new_x,new_y,new_theta
    end

    return SVector(current_x,current_y,current_theta,new_vehicle_speed),num_segments
end


function calculate_D_shield(curr_vehicle_speed, )

    curr_vehicle_speed = 0.5
    t_before_action_executed =
    delta_speed = 
    max_human_speed =
    t_stop = abs(curr_speed/delta_speed)
    @assert isinteger(t_stop) "$curr_speed divided by $delta_speed is not an integer, so t_stop is not an Integer"

    max_dist_before_planning_starts = curr_speed*t_before_action_executed
    max_vehicle_dist = sum(curr_speed:delta_speed:0.0)
    max_human_dist = max_human_speed*t_stop
    D_shield = max_vehicle_dist + max_human_dist + max_dist_before_planning_starts
    return D_shield
end
