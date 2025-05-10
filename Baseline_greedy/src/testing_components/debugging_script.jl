t1 = 38.0
tree1 = output.despot_trees[t1][:tree]
inchrome(D3Tree(tree1))
tree1.ba_children
scenario_num = 10
tree1.scenarios[scenario_num][1][2]

scens1 = tree1.scenarios[scenario_num];
DEP1 = tree1.Delta[scenario_num];
scenario_paths, rollout_actions = generate_rollout( scens1, pomdp_planner, DEP1);

N_array = [i[1] for i in scenario_paths[1]]

N = 1
P = extract_one_scenario_path(scenario_paths,N);
visualize_scenario_path(env,veh_params,output,P)
length(P)

s1 = SVector(38.210618777024784, 37.21386657005148, wrap_between_negative_pi_to_pi(2.893941838895035), 0.0)
BPDE.reactive_controller_HJB_policy(extended_space_pomdp.rollout_guide,s1,0.5)
my_reactive_controller_HJB_policy(extended_space_pomdp.rollout_guide,s1,0.5)



t2 = 40.5
tree2 = output.despot_trees[t2][:tree]
inchrome(D3Tree(tree2))
tree2.ba_children
scenario_num2 = 6
tree2.scenarios[scenario_num2][1][2]

scens2 = tree2.scenarios[scenario_num2];
DEP2 = tree2.Delta[scenario_num2];
scenario_paths, rollout_actions = generate_rollout( scens2, pomdp_planner, DEP2);

N_array = [i[1] for i in scenario_paths[1]]

N = 1
P = extract_one_scenario_path(scenario_paths,N);
visualize_scenario_path(env,veh_params,output,P)
length(P)

38.21061877702478, 37.21386657005148, 2.893941838895036, 0.0

s2 = SVector(38.21061877702478, 37.21386657005148, wrap_between_negative_pi_to_pi(2.893941838895036), 0.0)
BPDE.reactive_controller_HJB_policy(extended_space_pomdp.rollout_guide,s2,0.5)
my_reactive_controller_HJB_policy(extended_space_pomdp.rollout_guide,s2,0.5)

function my_find_best_action(planner, s, actions, velocity_set)  
    
    #=
    Find the best action out of all the actions for the current state
    where the velocity lie in the velocity_set 
    =#

    (;grid, V_values, Δt) = planner.solver
    (;veh, cost) = planner.problem
    
    best_value = -Inf
    best_action_index = -1

    #=
    Iterate through all actions to find the best one with velocity in
    the given velocity_set
    =#
    for i in eachindex(actions)
        a = actions[i]
        if(a[2] in velocity_set)
            sp = BPDE.propagate_state(s, a, Δt, veh)
            r = cost(s, a, Δt, veh)
            V_sp = r + my_interpolate_value(grid,V_values,sp)
            println("A : $a; V_sp = $V_sp")
            if(V_sp > best_value)
                best_value = V_sp
                best_action_index = i
            end
        end
    end

    return best_value, best_action_index
end

function my_reactive_controller_HJB_policy(planner,state,velocity_reactive_controller)
    
    #=
    Given the velocity chosen by the reactive controller, find the best action
    out of the actions with that velocity.
    If the reactive controller's action is not valid, then find the best action
    out of all the actions for the current state.
    =#

    (;Δt) = planner.solver
    (;δstate, veh, controls) = planner.problem
    δv = δstate[4]
    SVL = planner.safe_value_limit 

    #Get actions for current state
    actions = controls(state, Δt, veh)
    velocity_set = Tuple(velocity_reactive_controller)

    #Find the best action with reactive controller velocity
    best_value, best_action_index = my_find_best_action(planner,state,actions,velocity_set)

    #Check if this action is a valid action in static environment ---
    if ( best_value >= SVL || (state[4] == 0.0 && velocity_reactive_controller == δv) ) 
        best_action = actions[best_action_index]
        return best_action
    end

    #=
    If the action determined using reactive controller velocity action is not valid, then
    find the best action from all the actions for the current state
    =#
    #=
    NOTE: This can be improved. We find the best action with velocity_set (0.0, -δv) and then Check
    if the returned value is greater than previously returned best_value and make the decision
    accordingly. That will save computation time.
    =#
    println("Later Part")
    velocity_set = (-δv,0.0,δv)
    best_value, best_action_index = my_find_best_action(planner,state,actions,velocity_set)
    best_action = actions[best_action_index]
    return best_action
end

function my_interpolate_value(grid::RectangleGrid, value_array::Vector{Float64}, x::AbstractVector)
    #Check if current state is within the state space
    for d in eachindex(x)
        if x[d] < grid.cutPoints[d][1] || x[d] > grid.cutPoints[d][end]
            val_x = -1e6
            return val_x
        end
    end
    #Interpolate value at given state
    val_x = GridInterpolations.interpolate(grid, value_array, x)
    return val_x
end


function get_correct_delta_angle(delta_angle)
    #=
    Get the correct delta angle between -π to π
    =#
    # delta_angle = desired_orientation - current_orientation
    if(delta_angle > 180)
        delta_angle -= 360
    elseif(delta_angle < -180)
        delta_angle += 360
    end
    return delta_angle

end



function get_sl_correct_delta_angle(delta_angle)
    #=
    Get the correct delta angle between -π to π
    =#
    # delta_angle = desired_orientation - current_orientation
    abs_delta_angle = abs(delta_angle)
    if(abs_delta_angle <= 180)
        return delta_angle
    else
        if(delta_angle > 0.0)
            return delta_angle - 360
        else
            return delta_angle + 360
        end
    end
end


for i in -360:0.01:360
    # println(i)
    if( get_correct_delta_angle(i) != get_sl_correct_delta_angle(i) )
        println("i = $i; get_correct_delta_angle(i) = $(get_correct_delta_angle(i)); get_sl_correct_delta_angle(i) = $(get_sl_correct_delta_angle(i))")
    end
end


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
sudden_break_flag = true
run_with_trials = true
rollout_guide = (true,true)
extended_space_pomdp = ExtendedSpacePOMDP(pomdp_details,env,veh_params,rollout_guide,sudden_break_flag)
sol_rng = MersenneTwister(19)
lower_bound_func = DefaultPolicyLB(
                        FunctionPolicy(b->calculate_lower_bound_straight_line(extended_space_pomdp, b)),
                        max_depth=pomdp_details.tree_search_max_depth)
upper_bound_func = old_calculate_upper_bound
pomdp_solver = DESPOTSolver(
                    bounds=IndependentBounds(lower_bound_func,upper_bound_func,check_terminal=true,consistency_fix_thresh=1e-5),
                    # K=pomdp_details.num_scenarios,D=pomdp_details.tree_search_max_depth,
                    K=1,
                    tree_in_info=true,
                    T_max = run_with_trials ? Inf : pomdp_details.planning_time,
                    # max_trials = run_with_trials ? pomdp_details.max_num_trials : 100,
                    max_trials = 1,
                    default_action=get_default_action,
                    rng=sol_rng
                    )
pomdp_planner = POMDPs.solve(pomdp_solver, extended_space_pomdp);
shield_utils = ShieldUtils(exp_details, pomdp_details, veh_params.wheelbase)


ti = 9.5
b = output.b_root[ti]
next_pomdp_action, info = action_info(pomdp_planner, b);
println(next_pomdp_action)
inchrome(D3Tree(info[:tree]))
tree = info[:tree]
@profiler action_info(pomdp_planner, b)


scenario_num = 1
scens = tree.scenarios[scenario_num];
DEP = tree.Delta[scenario_num];
scenario_paths, rollout_actions = generate_rollout( scens, pomdp_planner, DEP);

N_array = [i[1] for i in scenario_paths[1]]

N = 1
P = extract_one_scenario_path(scenario_paths,N);
visualize_scenario_path(env,veh_params,output,P)
length(P)

