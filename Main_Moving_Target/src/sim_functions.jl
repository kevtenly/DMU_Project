using Distributions


function run_experiment_moving_target!(current_sim_obj::NavigationSimulator{VehicleParametersLSPlanner}, path_planning_details::PathPlanningDetails,
                                                pomdp_details::POMPDPlanningDetails, exp_details::ExperimentDetails, 
                                                shield_utils, output::Output,
                                                sudden_break_flag = true,
                                                run_shield = false,
                                                run_with_trials=false,
                                                print_logs = true)

    # initialize variables
    current_time_value = 0.0
    current_vehicle_speed = 0.0
    vehicle_steer_actions = get_vehicle_steer_actions(pomdp_details.max_vehicle_steering_angle,21)
    current_action = ActionLimitedSpacePOMDP(0.0)
    output.vehicle_actions[current_time_value] = current_action
    output.sim_objects[current_time_value] = current_sim_obj
    nbh = NearbyHumans(HumanState[], Int64[], HumanGoalsBelief[])
    output.nearby_humans[current_time_value] = nbh
    expected_vehicle_path = get_future_vehicle_trajectory(OrderedDict(current_time_value=>current_sim_obj.vehicle),
                                                    current_sim_obj.vehicle_params,1,path_planning_details,exp_details)
    output.vehicle_expected_trajectory[current_time_value] = expected_vehicle_path[current_time_value]
    modified_vehicle_params = copy(current_sim_obj.vehicle_params)
    next_action = nothing
    info = nothing
    predicted_vehicle_pos_in_goal = false
    debug = false
    print_despot_time = true:POMPDPlanningDetails, exp_details::ExperimentDetails, 
    shield_utils, output::Output,
    sudden_break_flag = true,
    run_shield = false,
    run_with_trials=false,
    print_logs = true
    print_shielding_time = true
    despot_start_time = 0.0
    store_tree=true
    start_time = time()
    LS_pomdp = nothing

    #1. Get initial target beleif (μ,Σ) -> initial_belief
    μ0 = SVector[exp_details.target_state.x, exp_details.target_state.y, exp_details.target_state.vx, exp_details.target_state.vy] 
    Σ0 = Diagonal(@SVector [0.2,0.2,0.2,0.2])
    initial_belief = MyNormal(μ0, Σ0)
    #2. output.target_belief[current_time_value] = initial_belief 
    output.target_belief[current_time_value] = initial_belief 
    #3. Define a new variable called mean_target_estimate and set it to the mean of the initial belief.
    mean_target_estimate = μ0
    covariance_target_estimate = Σ0
    :POMPDPlanningDetails, exp_details::ExperimentDetails, 
    shield_utils, output::Output,
    sudden_break_flag = true,
    run_shield = false,
    run_with_trials=false,
    print_logs = true
    #=
    Psuedocode
        while(!stop_condition)
            1) Print vehicle state, action and current time step
            2) Find vehicle trajectory using this action for one_time_step seconds
            3) Simulate humans for (one_time_step - pomdp_planning_time - buffer_time - path_planning_time) seconds
            4) a) Observe current human positions and their belief
               b) Predict future vehicle position
               c) Generate hybrid A* path
               d) Initialize scenarios
               e) Feed scenarios and hybrid A* path to the LS planner to get vehicle action
            5) Simulate humans for (pomdp_planning_time + buffer_time + path_planning_time) seconds
            6) Run shielding
            7) Set parameters for next cycle and store/modify required variables
        end
    =#

    # Run simulation
    # try
    while(!stop_simulation!(current_sim_obj,current_time_value,exp_details,output))

        exp_details.target_history[current_time_value] = Location(exp_details.target_state.x, exp_details.target_state.y)
        state_array = (current_sim_obj.vehicle.x, current_sim_obj.vehicle.y, current_sim_obj.vehicle.theta*180/pi, current_sim_obj.vehicle.v)
        action_array = (current_sim_obj.vehicle_params.controls_sequence[1]*180/pi, current_action.delta_speed)
        if(print_logs)
            println("\nTime_k = ", current_time_value,
                  "\t State_k = ", round.(state_array, digits=3),
                  "\t Action_k = ", round.(action_array, digits=3))
        end
        #=
        Generate vehicle trajectory using current action
        =#
        vehicle_trajectory, latest_controls_sequence_index = propogate_vehicle(current_sim_obj.vehicle, current_sim_obj.vehicle_params,current_vehicle_speed,
                                                            current_time_value, exp_details.one_time_step, path_planning_details, exp_details)
        vehicle_expected_goal_paths = get_future_vehicle_trajectory(vehicle_trajectory, current_sim_obj.vehicle_params,latest_controls_sequence_index+1,
                                                            path_planning_details,exp_details)

        #=
        Simulate for (one_time_step - path_planning_time - pomdp_planning_time - buffer_time) seconds
        =#
        if(debug)
            println("Simulating for one_time_step - path_planning_time - pomdp_planning_time - buffer_time seconds : " ,
                                exp_details.one_time_step - (path_planning_details.planning_time + pomdp_details.planning_time + exp_details.buffer_time))
            start_time = time()
        end
        time_duration_until_planning_begins = round(exp_details.one_time_step - (path_planning_details.planning_time +
                                                        pomdp_details.planning_time + exp_details.buffer_time), digits=1)
        current_sim_obj = simulate_vehicle_and_humans!(current_sim_obj, vehicle_trajectory, vehicle_expected_goal_paths,
                                                current_time_value, time_duration_until_planning_begins, exp_details, output)
        current_time_value = round(current_time_value+time_duration_until_planning_begins, digits=1)
        if(debug)
            println("Finished simulation")
            time_taken = time() - start_time
            println("Time Taken : ", time_taken)
        end
        
        #=
        Planning for the next cycle
        =#
        time_duration_until_pomdp_action_determined = round(pomdp_details.planning_time + exp_details.buffer_time, digits=1)
        predicted_vehicle_state = collect(values(vehicle_trajectory))[end]
        predicted_vehicle_center_x = predicted_vehicle_state.x + current_sim_obj.vehicle_params.dist_origin_to_center*cos(predicted_vehicle_state.theta)
        predicted_vehicle_center_y = predicted_vehicle_state.y + current_sim_obj.vehicle_params.dist_origin_to_center*sin(predicted_vehicle_state.theta)
        # println("Predicted Vehicle State : ", predicted_vehicle_state)
        # println("Predicted Vehicle Center : ", predicted_vehicle_center_x, " , ", predicted_vehicle_center_y)
        # Check if the predicted vehicle position is in the goal region
        if(is_within_range(predicted_vehicle_center_x,predicted_vehicle_center_y,current_sim_obj.vehicle_params.goal.x,current_sim_obj.vehicle_params.goal.y,exp_details.radius_around_vehicle_goal))
            println("Predicted vehicle state is in goal. Experiment Finished")
            next_pomdp_action = ActionLimitedSpacePOMDP(0.0)
            output.nearby_humans[current_time_value] = nbh
            output.b_roomean_target_estimatet[current_time_value] = nothing
            output.despot_trees[current_time_value] = nothing
            predicted_vehicle_pos_in_goal = true
        # If it is not, then take observation from the environment and plan for the next action
        else
            
            nbh = get_nearby_humans(current_sim_obj,pomdp_details.num_nearby_humans,2.0,pomdp_details.cone_half_angle)
            if(debug)
                println("Starting Hybrid A* path planning")
                start_time = time()
            end
            vehicle_steering_coehintrols = hybrid_astar_search(current_sim_obj.env, predicted_vehicle_state, current_sim_obj.vehicle_params,
                                                    vehicle_steer_actions, nbh, path_planning_details)
            if(debug)
                println("Finished Hybrid A* path planning")
                time_taken = time() - start_time
                println("Time Taken : ", time_taken)KF(old_belief, F,H,Q,R,o)

            end

            if(length(vehicle_steering_controls) == 0)
                if(print_logs)
                    println("Hybrid A* path not found within the given time limit. Reusing old path")
                end
                #Modify the steering_controls_sequence to reuse the old path
                modified_vehicle_params = modify_vehicle_params(current_sim_obj.vehicle_params, current_vehicle_speed, path_planning_details.veh_path_planning_v)
            else
                #Modify vehicle params
                modified_vehicle_params = modify_vehicle_params(current_sim_obj.vehicle_params, vehicle_steering_controls)
            end

            ng = mean_target_estimate
            modified_vehicle_params = modify_vehicle_goal(modified_vehicle_params, ng)


            if(debug)
                println("Starting POMDP planning")
                start_time = time()
            end
            b = TreeSearchScenarioParameters(predicted_vehicle_state.x,predicted_vehicle_state.y,predicted_vehicle_state.theta,predicted_vehicle_state.v,
                                modified_vehicle_params, exp_details.human_goal_locations, length(nbh.position_data), nbh.position_data, nbh.belief,
                                current_sim_obj.env.length,current_sim_obj.env.breadth,time_duration_until_pomdp_action_determined)
            output.nearby_humans[current_time_value] = nbh
            output.b_root[current_time_value] = b

            #=
            Define POMDP, POMDP Solver and POMDP Planner
            =#
            rollout_guide = HybridAStarPolicy(modified_vehicle_params.controls_sequence,length(modified_vehicle_params.controls_sequence))
            LS_pomdp = LimitedSpacePOMDP(pomdp_details,current_sim_obj.env,modified_vehicle_params,rollout_guide,sudden_break_flag)
            lower_bound_func = DefaultPolicyLB(
                                    FunctionPolicy(b->calculate_lower_bound(LS_pomdp, b)),
                                    max_depth=pomdp_details.tree_search_max_depth
                                    )
            upper_bound_func = calculate_upper_bound
            pomdp_solver = DESPOTSolver(
                                bounds=IndependentBounds(lower_bound_func,upper_bound_func,check_terminal=true,consistency_fix_thresh=1e-5),
                                K=pomdp_details.num_scenarios,D=pomdp_details.tree_search_max_depth,
                                T_max = run_with_trials ? Inf : pomdp_details.planning_time,
                                max_trials = run_with_trials ? pomdp_details.max_num_trials : 1000,
                                tree_in_info=true,
                                default_action= (s, rng) -> ActionLimitedSpacePOMDP(0.0),
                                rng = MersenneTwister(19)
                                )
            planner = POMDPs.solve(pomdp_solver, LS_pomdp);

            # Run DESPOTmean_target_estimate

            next_pomdp_action, info = action_info(planner, b)

            if(print_despot_time)
                println("Finished Tree Construction. Action selected")
                time_taken = time() - despot_start_time
                println("Time Taken : ", time_taken)
            end

            if(debug)
                println("Finished POMDP planning. Action selected")
                time_taken = time() - start_time
                println("Time Taken : ", time_taken)
            end
            if(store_tree)
                output.despot_trees[current_time_value] = info
            else
                output.despot_trees[current_time_value] = nothing
            end
        end

        #=
        Simulate for (path_planning_time + pomdp_planning_time + buffer_time) seconds
        =#
        if(debug)
            println("Simulating for path_planning_time + pomdp_planning_time + buffer_time seconds : " , 
                        path_planning_details.planning_time + pomdp_details.planning_time + exp_details.buffer_time)
            start_time = time()
        end
        time_duration_until_next_action_is_applied = round(path_planning_details.planning_time + 
                                                    pomdp_details.planning_time + exp_details.buffer_time, digits=1)
        current_sim_obj = simulate_vehicle_and_humans!(current_sim_obj, vehicle_trajectory, vehicle_expected_goal_paths,
                                                current_time_value, time_duration_until_next_action_is_applied, exp_details, output)
        if(debug)
            println("Finished simulation")
            time_taken = time() - start_time
            println("Time Taken : ", time_taken)
        end

        #1. Move target 
            #=
            target_state = exp_details.target_s
            new_target_state = your_func
            noise = sample_noise()
            new_target_state += noise
            exp_details.target_s = new_target_state
            =# 
        target_state = exp_details.target_s
        Δt = exp_details.simulator_time_step
        F = @SMatrix [1.0 0.0 Δt 0.0;
                        0.0 1.0 0.0 Δt;
                        0.0 0.0 1.0 0.0;
                        0.0 0.0 0.0 1.0] 
        new_target_state = F * target_state
        exp_details.target_s = new_target_state
        exp_details.target_history[CURRENT_TIME_VALUE] = Location(new_target_state.x , new_target_state.y)
        H_matrix = @SMatrix [1.0 0.0 0.0 0.0;
                                0.0 1.0 0.0 0.0]
        #2. Simulate an observation
            #=
                o = get_observation(new_target_state) H*x
                noise = sample_noise()
                o += noise
                =#
        o = H_matrix *  SVector(new_target_state.x, new_target_state.y, new_target_state.vx, new_target_state.vy)
        #o += rand()
        
        Q = Diagonal(@SVector [0.01, 0.01,0.1,0.1])
        R = Diagonal(@SVector [0.05, 0.05])
        #. new_belief = KF(old_belief, F,H,Q,R,o)
        μ_pred = F * mean_target_estimate
        Σ_pred = F * covariance_target_estimate * transpose(F) + Q
        Y_hat = o - H_matrix * μ_pred
        S = H_matrix * Σ_pred * transpose(H_matrix) + R
        K = Σ_pred * transpose(H_matrix) * inv(S)

        μ_new = μ_pred + K* Y_hat
        L = (I - K * H_matrix) * Σ_pred
        #4. Set mean_target_estimate to the mean of the new belief.
        mean_target_estimate = μ_new
        covariance_target_estimate = Σ_new

        #=
        Shielding
        =#
        if(run_shield)
            if(print_logs)
                println("POMDP requested action = ", (next_pomdp_action.delta_speed))
            end

            if(print_shielding_time)
                println("Started Shielding")
                shielding_start_time = time()
            end

            shield_radius_humans = find_shield_radius_humans(shield_utils, LS_pomdp, current_sim_obj)
            # Check if the predicted vehicle position is in the goal region
            if(predicted_vehicle_pos_in_goal)
                next_action = next_pomdp_action
            # Else, check if the pomdp tree is empty (root node took default action)
            elseif length(info[:tree].children[1]) == 0
                next_action = next_pomdp_action
            # Else, run shield and return the best safe action
            else
                veh_body = output.vehicle_body
                new_path_flag, next_action = get_best_safe_action(LS_pomdp, shield_utils, shield_radius_humans, 
                                                                current_sim_obj.vehicle, info[:tree])
                if(new_path_flag == false)
                    modified_vehicle_params = modify_vehicle_params(current_sim_obj.vehicle_params, current_vehicle_speed, 
                                                    path_planning_details.veh_path_planning_v)
                end
            end

            if(print_shielding_time)
                println("Finished Shielding. Action selected")
                time_taken = time() - shielding_start_time
                println("Time Taken : ", time_taken)
            end

            if(print_logs)
                println("Shield selected action = ", (next_action.delta_speed))
            end
        else
            next_action = next_pomdp_action
        end
    
        #=
        Set parameters for the next cycle
        =#
        current_action = next_action
        current_vehicle_speed = clamp((current_vehicle_speed + current_action.delta_speed), 0.0, pomdp_details.max_vehicle_speed)
        current_time_value = round(current_time_value+time_duration_until_next_action_is_applied,digits=1)
        current_sim_obj = copy(current_sim_obj,modified_vehicle_params)
        output.sim_objects[current_time_value] = current_sim_obj
        output.target_belief[current_time_value] = new_belief
        old_belief = new_belief

        #=
        Store relevant values in exp_details
        =#
        output.vehicle_actions[current_time_value] = current_action
        if(current_action.delta_speed == -10.0)
            output.number_sudden_stops += 1
        end
    end

    # catch e
    #     println("\n Things failed during the simulation. \n The error message is : \n ")
    #     println(e)
    # end
    output.time_taken = current_time_value
end



function modify_vehicle_goal(mvp, ng)

    new_vehicle_params = VehicleParametersLSPlanner(
        mvp.wheelbase,
        mvp.length,
        mvp.breadth,
        mvp.dist_origin_to_center,
        mvp.radius,
        mvp.max_speed,
        mvp.max_steering_angle,
        ng,
        mvp.controls_sequence
        )

    return new_vehicle_params
end