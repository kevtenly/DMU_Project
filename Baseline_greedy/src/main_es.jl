using JLD2
using Revise
include("struct_definition.jl")
include("environment.jl")
include("utils.jl")
include("ES_POMDP_Planner.jl")
include("belief_tracker.jl")
include("simulator.jl")
include("simulator_utils.jl")
include("parser.jl")
include("visualization.jl")
include("HJB_wrappers.jl")
include("shielding/shield_utils.jl")
include("shielding/shield.jl")

#=
Possible Environment Names

no_obstacles_25x25
small_obstacles_25x25
big_obstacle_25x25
L_shape_25x25
indoor_tables_25x25

no_obstacles_50x50
small_obstacles_50x50
many_small_obstacles_50x50
big_obstacle_50x50
L_shape_50x50

aspen
aspen2
small_obstacles_20x20
small_obstacles_30x30
L_shape_30x30
=#


#=
Initialization
=#
# environment_name = "no_obstacles_25x25"
# environment_name = "small_obstacles_25x25"
# environment_name = "big_obstacle_25x25"
# environment_name = "L_shape_25x25"

# environment_name = "no_obstacles_50x50"
environment_name = "small_obstacles_50x50"
# environment_name = "many_small_obstacles_50x50"
# environment_name = "big_obstacle_50x50"
# environment_name = "L_shape_50x50"

# environment_name = "small_obstacles_20x20"
# environment_name = "L_shape_100x100"
# environment_name = "indoor_tables_25x25"
# environment_name = "aspen"

filename = "configs/"*environment_name*".jl"
include(filename)
rollout_guide_filename = "./src/rollout_guides/HJB_rollout_guide_"*environment_name*".jld2"


#=
Define experiment details and POMDP planning details
=#
pomdp_details = POMPDPlanningDetails(input_config)
exp_details = ExperimentDetails(input_config)
output = OutputObj()

#=
Define environment
=#
env = generate_environment(input_config.env_length,input_config.env_breadth,input_config.obstacles)
exp_details.env = env
exp_details.human_goal_locations = get_human_goals(env)

#=
Define Vehicle
=#
veh = Vehicle(input_config.veh_start_x, input_config.veh_start_y, input_config.veh_start_theta, input_config.veh_start_v)
veh_sensor_data = VehicleSensor(HumanState[],Int64[],HumanGoalsBelief[])
veh_goal = Location(input_config.veh_goal_x,input_config.veh_goal_y)
r = sqrt( (0.5*input_config.veh_length)^2 + (0.5*input_config.veh_breadth)^2 )
veh_params = VehicleParametersESPlanner(input_config.veh_wheelbase,input_config.veh_length,
                input_config.veh_breadth,input_config.veh_dist_origin_to_center, r,
                input_config.veh_max_speed,input_config.veh_max_steering_angle,veh_goal)
#=
body_dims = SVector(veh_params.length, veh_params.breadth)
origin_to_cent = SVector(veh_params.dist_origin_to_center, 0.0)
veh_body = get_vehicle_body(body_dims, origin_to_cent)
=#
vehicle_body = get_vehicle_body((veh_params.length,veh_params.length), (veh_params.dist_origin_to_center,0.0))
output.vehicle_body = vehicle_body


#=
Define Humans
=#
env_humans, env_humans_params = generate_humans(env,veh,exp_details.human_start_v,exp_details.human_goal_locations,exp_details.num_humans_env,
                                        exp_details.simulator_time_step, exp_details.user_defined_rng)

#=
Create sim object
=#
initial_sim_obj = NavigationSimulator(env,veh,veh_params,veh_sensor_data,env_humans,env_humans_params,exp_details.simulator_time_step)

#=
Solve HJB equation for the given environment and vehicle
=#
Dt = 0.5
max_solve_steps = 200
Dval_tol = 0.1
HJB_planning_details = HJBPlanningDetails(Dt, max_solve_steps, Dval_tol, veh_params.max_steering_angle, veh_params.max_speed)
solve_HJB = true
solve_HJB = false
if(solve_HJB)
    println("Solving HJB equation for given environment ....")
    start_time = time()
    rollout_guide = get_HJB_rollout_guide(HJB_planning_details, exp_details, veh_params);
    println("Finish time : ", time()-start_time)
    d = Dict("rollout_guide"=>rollout_guide);
    save(rollout_guide_filename,d);
else
    s = load(rollout_guide_filename);
    rollout_guide = s["rollout_guide"];
end

#=
Define POMDP, POMDP Solver and POMDP Planner
=#
sol_rng = MersenneTwister(19)
SB_flag = false  #Apply Sudden Break Flag
SB_flag = true  #Apply Sudden Break Flag
extended_space_pomdp = ExtendedSpacePOMDP(pomdp_details,env,veh_params,rollout_guide,SB_flag);
lower_bound_func = DefaultPolicyLB(
                        FunctionPolicy(b->calculate_lower_bound(extended_space_pomdp, b)),
                        max_depth=pomdp_details.tree_search_max_depth
                        )
upper_bound_func = calculate_upper_bound
run_with_trials = false
run_with_trials = true
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

#=
Define Shield Utils
=#
RS_flag = true  #Run shield Flag
RS_flag = false  #Run shield Flag
shield_utils = ShieldUtils(exp_details, pomdp_details, veh_params.wheelbase)


#=
Run the experiment
=#
c = run_experiment!(initial_sim_obj, pomdp_planner, lower_bound_func, upper_bound_func,
                    pomdp_details, exp_details, shield_utils, output, RS_flag, run_with_trials)

#=
Create Gif
=#
create_gif = true
create_gif = false
if(create_gif)
    vehicle_executed_trajectory = []
    anim = @animate for k ∈ keys(output.sim_objects)
        observe(output, exp_details, k, vehicle_executed_trajectory);
    end
    gif(anim, "es_planner.gif", fps = 10)
end


# nbh = NearbyHumans(HumanState[], Int64[], HumanGoalsBelief[])
# vsd = VehicleSensor(HumanState[], Int64[], HumanGoalsBelief[])
# get_plot(env, veh, veh_params, nbh, vsd, 0.0, exp_details)

#=

for i in 0.0:exp_details.one_time_step:output.time_taken
    println("Time = $i; Vehicle : ", output.sim_objects[i].vehicle)
end

=#