using BellmanPDEs
using JLD2
using Revise
include("struct_definition.jl")
include("environment.jl")
include("utils.jl")
include("hybrid_astar.jl")
include("LS_POMDP_Planner.jl")
include("ES_POMDP_Planner.jl")
include("belief_tracker.jl")
include("simulator.jl")
include("simulator_utils.jl")
include("parser.jl")
include("visualization.jl")
include("HJB_wrappers.jl")
include("shielding/shield_utils.jl")
include("shielding/shield.jl")


function run_extended_space_planner_experiment_HJB_rollout(input_config, rollout_guide,
                                        sudden_break_flag, run_shield_flag, run_with_trials, 
                                        print_logs=true, create_gif=false)

    #Define experiment details and POMDP planning details
    pomdp_details = POMPDPlanningDetails(input_config)
    exp_details = ExperimentDetails(input_config)
    output = OutputObj()
    #Define environment
    env = generate_environment(input_config.env_length,input_config.env_breadth,input_config.obstacles)
    exp_details.env = env
    exp_details.human_goal_locations = get_human_goals(env)
    #Define Vehicle
    veh = Vehicle(input_config.veh_start_x, input_config.veh_start_y, input_config.veh_start_theta, input_config.veh_start_v)
    veh_sensor_data = VehicleSensor(HumanState[],Int64[],HumanGoalsBelief[])
    veh_goal = Location(input_config.veh_goal_x,input_config.veh_goal_y)
    r = sqrt( (0.5*input_config.veh_length)^2 + (0.5*input_config.veh_breadth)^2 )
    veh_params = VehicleParametersESPlanner(input_config.veh_wheelbase,input_config.veh_length,
                    input_config.veh_breadth,input_config.veh_dist_origin_to_center, r,
                    input_config.veh_max_speed,input_config.veh_max_steering_angle,veh_goal)
    vehicle_body = get_vehicle_body((veh_params.length,veh_params.length), (veh_params.dist_origin_to_center,0.0))
    output.vehicle_body = vehicle_body
    #Define Humans
    env_humans, env_humans_params = generate_humans(env,veh,exp_details.human_start_v,exp_details.human_goal_locations,exp_details.num_humans_env,
                                            exp_details.simulator_time_step, exp_details.user_defined_rng)
    #Create sim object
    initial_sim_obj = NavigationSimulator(env,veh,veh_params,veh_sensor_data,env_humans,env_humans_params,exp_details.simulator_time_step)

    #Define POMDP, POMDP Solver and POMDP Planner
    sol_rng = MersenneTwister(19)
    extended_space_pomdp = ExtendedSpacePOMDP(pomdp_details,env,veh_params,rollout_guide,sudden_break_flag)
    lower_bound_func = DefaultPolicyLB(
                            FunctionPolicy(b->calculate_lower_bound(extended_space_pomdp, b)),
                            max_depth=pomdp_details.tree_search_max_depth)
    upper_bound_func = calculate_upper_bound
    pomdp_solver = DESPOTSolver(
                        bounds=IndependentBounds(lower_bound_func,upper_bound_func,check_terminal=true,consistency_fix_thresh=1e-5),
                        K=pomdp_details.num_scenarios,D=pomdp_details.tree_search_max_depth,
                        tree_in_info=true,
                        T_max = run_with_trials ? Inf : pomdp_details.planning_time,
                        max_trials = run_with_trials ? pomdp_details.max_num_trials : 100,
                        default_action=get_default_action,
                        rng=sol_rng
                        )
    pomdp_planner = POMDPs.solve(pomdp_solver, extended_space_pomdp);
    shield_utils = ShieldUtils(exp_details, pomdp_details, veh_params.wheelbase)

    #Run the experiment
    try
        run_experiment!(initial_sim_obj, pomdp_planner, lower_bound_func, upper_bound_func,
                        pomdp_details, exp_details, shield_utils, output, run_shield_flag, 
                        run_with_trials, print_logs)
        #Create Gif
        if(create_gif)
            generate_gif(output, exp_details)
        end
    catch ex
        println("Experiment Failed. Coding Error")
        # println(ex)
    end
    return output
end


function run_limited_space_planner_experiment(input_config, sudden_break_flag,
                    run_shield_flag, run_with_trials, print_logs=true, create_gif=false)

    #Define experiment details and POMDP planning details
    pomdp_details = POMPDPlanningDetails(input_config)
    pomdp_details.planning_time = input_config.LS_pomdp_planning_time
    pomdp_details.max_num_trials = input_config.LS_max_num_trials
    exp_details = ExperimentDetails(input_config)
    output = OutputObj()
    #Define environment
    env = generate_environment(input_config.env_length,input_config.env_breadth,input_config.obstacles)
    exp_details.env = env
    exp_details.human_goal_locations = get_human_goals(env)
    #Define path planning details
    path_planning_details = PathPlanningDetails(input_config, env)
    #Define Vehicle
    veh = Vehicle(input_config.veh_start_x, input_config.veh_start_y, input_config.veh_start_theta, input_config.veh_start_v)
    veh_sensor_data = VehicleSensor(HumanState[],Int64[],HumanGoalsBelief[])
    veh_goal = Location(input_config.veh_goal_x,input_config.veh_goal_y)
    r = sqrt( (0.5*input_config.veh_length)^2 + (0.5*input_config.veh_breadth)^2 )
    temp_veh_params = VehicleParametersLSPlanner(input_config.veh_wheelbase,input_config.veh_length,
                    input_config.veh_breadth,input_config.veh_dist_origin_to_center, r,
                    input_config.veh_max_speed,input_config.veh_max_steering_angle,veh_goal,Float64[])

    #Find hybrid A* path for the given environment and vehicle.
    nbh = NearbyHumans(HumanState[], Int64[], HumanGoalsBelief[])
    vehicle_delta_angle_actions = get_vehicle_actions(45,5)
    given_planning_time = path_planning_details.planning_time
    path_planning_details.planning_time = 10.0
    vehicle_controls_sequence = hybrid_astar_search(env,veh,temp_veh_params,vehicle_delta_angle_actions,nbh,path_planning_details);
    path_planning_details.planning_time = given_planning_time
    veh_params = VehicleParametersLSPlanner(input_config.veh_wheelbase,input_config.veh_length,
                    input_config.veh_breadth,input_config.veh_dist_origin_to_center, r,
                    input_config.veh_max_speed,input_config.veh_max_steering_angle,veh_goal,vehicle_controls_sequence)
    vehicle_body = get_vehicle_body((veh_params.length,veh_params.length), (veh_params.dist_origin_to_center,0.0))
    output.vehicle_body = vehicle_body

    #Define Humans
    env_humans, env_humans_params = generate_humans(env,veh,exp_details.human_start_v,exp_details.human_goal_locations,exp_details.num_humans_env,
                                            exp_details.simulator_time_step, exp_details.user_defined_rng)

    #Create sim object
    initial_sim_obj = NavigationSimulator(env,veh,veh_params,veh_sensor_data,env_humans,env_humans_params,
                        exp_details.simulator_time_step)

    shield_utils = ShieldUtils(exp_details, pomdp_details, veh_params.wheelbase)                    
    #Run the experiment
    try
        run_experiment!(initial_sim_obj, path_planning_details, pomdp_details, exp_details, 
                    shield_utils, output, sudden_break_flag, run_shield_flag, run_with_trials,
                    print_logs)
        #Create Gif
        if(create_gif)
            generate_gif(output, exp_details)
        end
    catch ex
        println("Experiment Failed. Coding Error")
        println(ex)
    end
    return output
end

function run_extended_space_planner_experiment_random_rollout(input_config, rollout_guide,
                                    sudden_break_flag, run_shield_flag, run_with_trials,
                                    print_logs=true, create_gif=false)

    #Define experiment details and POMDP planning details
    pomdp_details = POMPDPlanningDetails(input_config)
    exp_details = ExperimentDetails(input_config)
    output = OutputObj()
    #Define environment
    env = generate_environment(input_config.env_length,input_config.env_breadth,input_config.obstacles)
    exp_details.env = env
    exp_details.human_goal_locations = get_human_goals(env)
    #Define Vehicle
    veh = Vehicle(input_config.veh_start_x, input_config.veh_start_y, input_config.veh_start_theta, input_config.veh_start_v)
    veh_sensor_data = VehicleSensor(HumanState[],Int64[],HumanGoalsBelief[])
    veh_goal = Location(input_config.veh_goal_x,input_config.veh_goal_y)
    r = sqrt( (0.5*input_config.veh_length)^2 + (0.5*input_config.veh_breadth)^2 )
    veh_params = VehicleParametersESPlanner(input_config.veh_wheelbase,input_config.veh_length,
                    input_config.veh_breadth,input_config.veh_dist_origin_to_center, r,
                    input_config.veh_max_speed,input_config.veh_max_steering_angle,veh_goal)
    vehicle_body = get_vehicle_body((veh_params.length,veh_params.length), (veh_params.dist_origin_to_center,0.0))
    output.vehicle_body = vehicle_body
    #Define Humans
    env_humans, env_humans_params = generate_humans(env,veh,exp_details.human_start_v,exp_details.human_goal_locations,exp_details.num_humans_env,
                                            exp_details.simulator_time_step, exp_details.user_defined_rng)
    #Create sim object
    initial_sim_obj = NavigationSimulator(env,veh,veh_params,veh_sensor_data,env_humans,env_humans_params,exp_details.simulator_time_step)

    #Define POMDP, POMDP Solver and POMDP Planner
    rollout_guide = Tuple(true)
    extended_space_pomdp = ExtendedSpacePOMDP(pomdp_details,env,veh_params,rollout_guide,sudden_break_flag)
    sol_rng = MersenneTwister(19)
    lower_bound_func = DefaultPolicyLB(RandomPolicy(extended_space_pomdp));
    upper_bound_func = calculate_upper_bound
    pomdp_solver = DESPOTSolver(
                        bounds=IndependentBounds(lower_bound_func,upper_bound_func,check_terminal=true,consistency_fix_thresh=1e-5),
                        K=pomdp_details.num_scenarios,D=pomdp_details.tree_search_max_depth,
                        tree_in_info=true,
                        T_max = run_with_trials ? Inf : pomdp_details.planning_time,
                        max_trials = run_with_trials ? pomdp_details.max_num_trials : 100,
                        default_action=get_default_action,
                        rng=sol_rng
                        )
    pomdp_planner = POMDPs.solve(pomdp_solver, extended_space_pomdp);
    shield_utils = ShieldUtils(exp_details, pomdp_details, veh_params.wheelbase)

    #Run the experiment
    try
        run_experiment!(initial_sim_obj, pomdp_planner, lower_bound_func, upper_bound_func,
                        pomdp_details, exp_details, shield_utils, output, run_shield_flag,
                        run_with_trials, print_logs)
        #Create Gif
        if(create_gif)
            generate_gif(output, exp_details)
        end
    catch ex
        println("Experiment Failed. Coding Error")
        println(ex)
    end
    return output
end

function run_extended_space_planner_experiment_straight_line_rollout(input_config, rollout_guide,
                                            sudden_break_flag, run_shield_flag, run_with_trials,
                                            print_logs=true, create_gif=false)

    #Define experiment details and POMDP planning details
    pomdp_details = POMPDPlanningDetails(input_config)
    exp_details = ExperimentDetails(input_config)
    output = OutputObj()
    #Define environment
    env = generate_environment(input_config.env_length,input_config.env_breadth,input_config.obstacles)
    exp_details.env = env
    exp_details.human_goal_locations = get_human_goals(env)
    #Define Vehicle
    veh = Vehicle(input_config.veh_start_x, input_config.veh_start_y, input_config.veh_start_theta, input_config.veh_start_v)
    veh_sensor_data = VehicleSensor(HumanState[],Int64[],HumanGoalsBelief[])
    veh_goal = Location(input_config.veh_goal_x,input_config.veh_goal_y)
    r = sqrt( (0.5*input_config.veh_length)^2 + (0.5*input_config.veh_breadth)^2 )
    veh_params = VehicleParametersESPlanner(input_config.veh_wheelbase,input_config.veh_length,
                    input_config.veh_breadth,input_config.veh_dist_origin_to_center, r,
                    input_config.veh_max_speed,input_config.veh_max_steering_angle,veh_goal)
    vehicle_body = get_vehicle_body((veh_params.length,veh_params.length), (veh_params.dist_origin_to_center,0.0))
    output.vehicle_body = vehicle_body
    #Define Humans
    env_humans, env_humans_params = generate_humans(env,veh,exp_details.human_start_v,exp_details.human_goal_locations,exp_details.num_humans_env,
                                            exp_details.simulator_time_step, exp_details.user_defined_rng)
    #Create sim object
    initial_sim_obj = NavigationSimulator(env,veh,veh_params,veh_sensor_data,env_humans,env_humans_params,exp_details.simulator_time_step)

    #Define POMDP, POMDP Solver and POMDP Planner
    rollout_guide = (true,true)
    extended_space_pomdp = ExtendedSpacePOMDP(pomdp_details,env,veh_params,rollout_guide,sudden_break_flag)
    sol_rng = MersenneTwister(19)
    lower_bound_func = DefaultPolicyLB(
                            FunctionPolicy(b->calculate_lower_bound_straight_line(extended_space_pomdp, b)),
                            max_depth=pomdp_details.tree_search_max_depth)
    upper_bound_func = calculate_upper_bound
    pomdp_solver = DESPOTSolver(
                        bounds=IndependentBounds(lower_bound_func,upper_bound_func,check_terminal=true,consistency_fix_thresh=1e-5),
                        K=pomdp_details.num_scenarios,D=pomdp_details.tree_search_max_depth,
                        tree_in_info=true,
                        T_max = run_with_trials ? Inf : pomdp_details.planning_time,
                        max_trials = run_with_trials ? pomdp_details.max_num_trials : 100,
                        default_action=get_default_action,
                        rng=sol_rng
                        )
    pomdp_planner = POMDPs.solve(pomdp_solver, extended_space_pomdp);
    shield_utils = ShieldUtils(exp_details, pomdp_details, veh_params.wheelbase)

    #Run the experiment
    try
        run_experiment!(initial_sim_obj, pomdp_planner, lower_bound_func, upper_bound_func,
                        pomdp_details, exp_details, shield_utils, output, run_shield_flag, 
                        run_with_trials, print_logs)
        #Create Gifs
        if(create_gif)
            generate_gif(output, exp_details)
        end
    catch ex
        println("Experiment Failed. Coding Error")
        println(ex)
    end
    return output
end


function run_esp_experiment_HJB_rollout_without_SB_no_shielding(input_config, rollout_guide, create_gif=false)
    sudden_break_flag = false
    run_shield_flag = false
    return run_extended_space_planner_experiment_HJB_rollout(input_config, rollout_guide,
                                            sudden_break_flag, run_shield_flag, create_gif)
end

function run_esp_experiment_HJB_rollout_with_SB_no_shielding(input_config, rollout_guide)
    sudden_break_flag = true
    run_shield_flag = false
    return run_extended_space_planner_experiment_HJB_rollout(input_config, rollout_guide,
                                            sudden_break_flag, run_shield_flag, create_gif)
end

function run_esp_experiment_HJB_rollout_with_shielding(input_config, rollout_guide)
    #This by default means no sudden break action
    sudden_break_flag = false
    run_shield_flag = true
    return run_extended_space_planner_experiment_HJB_rollout(input_config, rollout_guide,
                                            sudden_break_flag, run_shield_flag, create_gif)
end


function run_lsp_experiment_without_SB_no_shielding(input_config, create_gif=false)
    sudden_break_flag = false
    run_shield_flag = false
    return run_limited_space_planner_experiment(input_config,sudden_break_flag,run_shield_flag,create_gif)
end

function run_lsp_experiment_with_SB_no_shielding(input_config,create_gif=false)
    sudden_break_flag = true
    run_shield_flag = false
    return run_limited_space_planner_experiment(input_config,sudden_break_flag,run_shield_flag,create_gif)
end

function run_lsp_experiment_with_shielding(input_config, rollout_guide)
    #This by default means no sudden break action
    sudden_break_flag = false
    run_shield_flag = true
    return run_limited_space_planner_experiment(input_config,sudden_break_flag,run_shield_flag,create_gif)
end



#=
Code to run things!

lsp_output = run_limited_space_planner_experiment(input_config);
s = load("./src/HJB_rollout_guide.jld2")
rollout_guide = s["rollout_guide"];
esp_output = run_extended_space_planner_experiment(input_config, rollout_guide);

=#
