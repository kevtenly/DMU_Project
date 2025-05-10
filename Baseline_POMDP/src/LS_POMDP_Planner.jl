
include("pomdp_planning_utils.jl")
#=
Added current_target_belief field to track belief during planning

Maintained original state structure while adding belief tracking


Incorporated exp_details in POMDP model for access to true target

Modified constructor to maintain compatibility with existing code

Updated terminal state check to use true target location

Modified generative model to verify against true goal

Enhanced upper bound calculation with belief-aware heuristics
=#

#=
Struct for POMDP State
=#
struct StateLimitedSpacePOMDP
    vehicle_x::Float64
    vehicle_y::Float64
    vehicle_theta::Float64
    vehicle_v::Float64
    index_vehicle_controls_sequence::Int64
    nearby_humans::Array{HumanState,1}
    current_target_belief::TargetBelief  # Added belief tracking
end

#=
Struct for POMDP Action
=#
struct ActionLimitedSpacePOMDP
    delta_speed::Float64
end

#=
Struct for Hybrid A* policy
=#
struct HybridAStarPolicy
    controls_sequence::Array{Float64,1}
    len::Int64
end

#=
Struct for POMDP
=#
struct LimitedSpacePOMDP{P,N} <: POMDPs.POMDP{StateLimitedSpacePOMDP,ActionLimitedSpacePOMDP,Array{Location,1}}
    discount_factor::Float64
    min_safe_distance_from_human::Float64
    human_collision_penalty::Float64
    radius_around_vehicle_goal::Float64
    goal_reached_reward::Float64
    vehicle_wheelbase::Float64
    vehicle_length::Float64
    vehicle_breadth::Float64
    vehicle_D::Float64
    vehicle_R::Float64
    max_vehicle_speed::Float64
    vehicle_action_delta_speed::Float64
    vehicle_goal::Location
    one_time_step::Float64
    num_segments_in_one_time_step::Int64
    observation_discretization_length::Float64
    d_near::Float64
    d_far::Float64
    sudden_break_flag::Bool
    world::ExperimentEnvironment
    exp_details::ExperimentDetails # added
    vehicle_path_cache::MVector{N, Tuple{Float64,Float64,Float64}}
    rollout_guide::P
end

function LimitedSpacePOMDP(pomdp_details,env,vehicle_params,rollout_guide,sudden_break_flag,exp_details)

    return LimitedSpacePOMDP(
        pomdp_details.discount_factor,
        pomdp_details.min_safe_distance_from_human,
        pomdp_details.human_collision_penalty,
        pomdp_details.radius_around_vehicle_goal,
        pomdp_details.goal_reached_reward,
        vehicle_params.wheelbase,
        vehicle_params.length,
        vehicle_params.breadth,
        vehicle_params.dist_origin_to_center,
        vehicle_params.radius,
        pomdp_details.max_vehicle_speed,
        pomdp_details.action_delta_speed,
        vehicle_params.goal,
        pomdp_details.one_time_step,
        pomdp_details.num_segments_in_one_time_step,
        pomdp_details.observation_discretization_length,
        pomdp_details.d_near,
        pomdp_details.d_far,
        sudden_break_flag,
        env,
        exp_details,
        MVector{pomdp_details.num_segments_in_one_time_step+1,Tuple{Float64,Float64,Float64}}(undef),
        rollout_guide
        )
end


# #=
# Function to check terminal state
# =#
# function is_terminal_state(s,terminal_state)
#     if(terminal_state.x == s.vehicle_x && terminal_state.y == s.vehicle_y)
#         return true
#     else
#         return false
#     end
# end

# Updated terminal state check using true target
function is_terminal_state(m::LimitedSpacePOMDP, s::StateLimitedSpacePOMDP)
    vehicle_loc = Location(s.vehicle_x, s.vehicle_y)
    target_loc = m.exp_details.true_target
    return is_within_range(vehicle_loc, target_loc, m.radius_around_vehicle_goal)
end

#=
************************************************************************************************
Generate Initial POMDP state based on the scenario provided by random operator.
=#
function Base.rand(rng::AbstractRNG, scenario_params::TreeSearchScenarioParameters{VehicleParametersLSPlanner})
    humans = Vector{HumanState}()
    for i in 1:scenario_params.num_nearby_humans
        sampled_goal = rand(rng, SparseCat(scenario_params.human_goals, scenario_params.nearby_humans_belief[i].pdf))
        new_human = HumanState(scenario_params.nearby_humans[i].x, scenario_params.nearby_humans[i].y, scenario_params.nearby_humans[i].v, sampled_goal)
        noise = (rand(rng) - 0.5) * new_human.v * scenario_params.time_duration * 0.2
        new_human = update_human_position(new_human, scenario_params.world_length, scenario_params.world_breadth, scenario_params.time_duration, noise)
        push!(humans, new_human)
    end
    
    # Create a target belief using the target_locations field
    # This uses the fields that actually exist in scenario_params
    target_belief = TargetBelief(ones(length(scenario_params.target_locations)) / length(scenario_params.target_locations))
    
    return StateLimitedSpacePOMDP(
        scenario_params.vehicle_x,
        scenario_params.vehicle_y,
        scenario_params.vehicle_theta,
        scenario_params.vehicle_v,
        1,
        humans,
        target_belief  # Include the target belief in the constructor
    )
end


#=
************************************************************************************************
Simulate the vehicle one step forward in POMDP planning
=#

function update_vehicle_position(m::LimitedSpacePOMDP, s, new_vehicle_speed, num_path_steps)
    current_x, current_y, current_theta = s.vehicle_x, s.vehicle_y, s.vehicle_theta
    curr_vehicle = (current_x, current_y, current_theta)
    N = m.num_segments_in_one_time_step
    vehicle_path = m.vehicle_path_cache
    vehicle_path[1] = curr_vehicle

    # println("S index : $(s.index_vehicle_controls_sequence); num_path_steps : $num_path_steps")
    if(new_vehicle_speed == 0.0)
        for i in 2:N+1
            vehicle_path[i] = curr_vehicle
        end
    else
        curr_vehicle_path_index = 1 
        curr_num_steps = 0  # Will range from 1 to num_path_steps*N
        time_duration = m.one_time_step/num_path_steps/N
        for i in 1:num_path_steps
            index = max(1, s.index_vehicle_controls_sequence+i-1)
            index = min(index, length(m.rollout_guide.controls_sequence))
            steering_angle = m.rollout_guide.controls_sequence[index]
            for j in 1:N
                new_x,new_y,new_theta = move_vehicle(current_x,current_y,current_theta,m.vehicle_wheelbase,
                                                        steering_angle,new_vehicle_speed,time_duration)
                current_x,current_y,current_theta = new_x,new_y,new_theta
                # new_x,new_y,new_theta = move(curr_vehicle,(steering_angle,new_vehicle_speed),m.one_time_step/N,m.vehicle_wheelbase)
                curr_vehicle = (new_x,new_y,new_theta)
                curr_num_steps += 1
                if(curr_num_steps % num_path_steps == 0)
                    curr_vehicle_path_index += 1 
                    @inbounds vehicle_path[curr_vehicle_path_index] = curr_vehicle
                end
            end
            if(s.index_vehicle_controls_sequence+i > m.rollout_guide.len)
                for k in curr_vehicle_path_index+1:N+1
                    @inbounds vehicle_path[k] = curr_vehicle
                end
                return
            end
        end
    end
end

#=
************************************************************************************************
POMDP Generative Model
=#

#parent = Dict()
function POMDPs.gen(m::LimitedSpacePOMDP, s, a, rng)

    vehicle_reached_goal = false
    collision_with_human = false
    collision_with_obstacle = false
    immediate_stop = false
    next_human_states = HumanState[]
    observed_positions = Location[]

    #=
        Check if current vehicle position collides with any nearby human.
        Check if current vehicle position is in the goal region.
        Apply given action on the vehicle and get vehicle path.
        Propogate humans and get their paths.
        Check if vehicle path collides with the path of any nearby human and with any static obstacle.
        Generate the new state accordingly.
        Generate corrsponding observation and reward.
    =#

    vehicle_center_x = s.vehicle_x + m.vehicle_D*cos(s.vehicle_theta)
    vehicle_center_y = s.vehicle_y + m.vehicle_D*sin(s.vehicle_theta)

    for human in s.nearby_humans
        if(s.vehicle_v != 0.0 && a.delta_speed != -10.0 )
            if( is_within_range(vehicle_center_x, vehicle_center_y, human.x, human.y, m.min_safe_distance_from_human+m.vehicle_R) )
                # println("Collision with this human " ,s.nearby_humans[human_index] , " ", time_index )
                # println("Vehicle's position is " ,vehicle_path[time_index] , "\nHuman's position is ", intermediate_human_location )
                new_vehicle_position = (-100.0, -100.0, -100.0)
                collision_with_human = true
                observed_positions = Location[ Location(-50.0,-50.0) ]
                sp = StateLimitedSpacePOMDP(new_vehicle_position[1],new_vehicle_position[2],new_vehicle_position[3],s.vehicle_v,-1,next_human_states,s.current_target_belief)
                r = human_collision_penalty(collision_with_human, m.human_collision_penalty)
                return (sp=sp, o=observed_positions, r=r)
            end
        end
    end

    true_target = m.exp_details.true_target
    if(is_within_range(vehicle_center_x, vehicle_center_y, true_target.x, true_target.y, m.radius_around_vehicle_goal))
        # println("Goal reached")
        new_vehicle_position = (-100.0, -100.0, -100.0)
        vehicle_reached_goal = true
        observed_positions = Location[ Location(-50.0,-50.0) ]
        sp = StateLimitedSpacePOMDP(new_vehicle_position[1],new_vehicle_position[2],new_vehicle_position[3],s.vehicle_v,-1,next_human_states,s.current_target_belief)
        r = vehicle_goal_reached_reward(vehicle_reached_goal, m.goal_reached_reward)
        return (sp=sp, o=observed_positions, r=r)
    end

    if(a.delta_speed == -10.0)
        immediate_stop = true
    end
    new_vehicle_speed = clamp(s.vehicle_v + a.delta_speed, 0.0, m.max_vehicle_speed)
    num_path_steps = Int(new_vehicle_speed/m.vehicle_action_delta_speed)
    update_vehicle_position(m, s, new_vehicle_speed, num_path_steps)
    vehicle_path = m.vehicle_path_cache
    new_vehicle_position = vehicle_path[end]

    for human in s.nearby_humans
        scaling_factor_for_noise = 0.2
        noise = (rand(rng) - 0.5)*human.v*m.one_time_step*scaling_factor_for_noise
        modified_human_state = update_human_position(human,m.world.length,m.world.breadth,m.one_time_step,noise)
        if(new_vehicle_speed!=0.0)
            human_path_x = LinRange(human.x,modified_human_state.x,m.num_segments_in_one_time_step+1)
            human_path_y = LinRange(human.y,modified_human_state.y,m.num_segments_in_one_time_step+1)
            for time_index in 2:m.num_segments_in_one_time_step+1
                vehicle_center_x = vehicle_path[time_index][1] + m.vehicle_D*cos(vehicle_path[time_index][3])
                vehicle_center_y = vehicle_path[time_index][2] + m.vehicle_D*sin(vehicle_path[time_index][3])
                if( is_within_range(vehicle_center_x,vehicle_center_y,human_path_x[time_index],human_path_y[time_index],m.min_safe_distance_from_human+m.vehicle_R) )
                    # println("Collision with this human " ,human , "at time ", time_index )
                    # println("Vehicle's position is " ,vehicle_path[time_index] , "\nHuman's position is ", (human_path_x[time_index],human_path_y[time_index]) )
                    new_vehicle_position = (-100.0, -100.0, -100.0)
                    collision_with_human = true
                    next_human_states = HumanState[]
                    observed_positions = Location[ Location(-50.0,-50.0) ]
                    sp = StateLimitedSpacePOMDP(new_vehicle_position[1],new_vehicle_position[2],new_vehicle_position[3],new_vehicle_speed,-1,next_human_states,s.current_target_belief)
                    r = human_collision_penalty(collision_with_human, m.human_collision_penalty)
                    return (sp=sp, o=observed_positions, r=r)
                end
            end
        end
        if(modified_human_state.x!=modified_human_state.goal.x && modified_human_state.y!=modified_human_state.goal.y)
            push!(next_human_states, modified_human_state)
            discrete_new_x = floor(modified_human_state.x/m.observation_discretization_length) * m.observation_discretization_length
            discrete_new_y = floor(modified_human_state.y/m.observation_discretization_length) * m.observation_discretization_length
            observed_location = Location(discrete_new_x, discrete_new_y)
            push!(observed_positions, observed_location)
        end
    end


    # If the code reaches here, then both s and sp are safe states. Define corresponding new POMDP state.
    sp = StateLimitedSpacePOMDP(new_vehicle_position[1],new_vehicle_position[2],new_vehicle_position[3],new_vehicle_speed,
                                    s.index_vehicle_controls_sequence+num_path_steps,next_human_states, s.current_target_belief)

    # R(s,a): Reward for being at state s and taking action a
    #Penalize if collision with human
    r = low_speed_penalty(s.vehicle_v, m.max_vehicle_speed)
    #println("Reward from not traveling at max_speed ", r)
    #Penalize if had to apply sudden brakes
    r += immediate_stop_penalty(immediate_stop, m.human_collision_penalty)
    #println("Reward if you had to apply immediate brakes", r)
    #Penalize unsmooth paths
    r += unsmooth_motion_penalty(a)
    #Penalty to avoid long paths
    r += -1.0

    # if( sp.vehicle_x<0.0+sp.vehicle_L || sp.vehicle_y<0.0+sp.vehicle_L || sp.vehicle_x>m.world.length-sp.vehicle_L || sp.vehicle_y>m.world.breadth-sp.vehicle_L )
    #     println(s)
    #     println(sp)
    #     println(r)
    # end
    # println("REWARD is : ",r)
    return (sp=sp, o=observed_positions, r=r)
end
#=
unit_test_h1 = human_state(10.0,10.0,1.0,location(0.0,0.0),1)
unit_test_h2 = human_state(10.0,10.0,1.0,location(100.0,0.0),2)
unit_test_nearby_humans = human_state[unit_test_h1,unit_test_h2]
unit_test_es_planner_state = state_Limited_space_POMDP_planner(2.0,2.0,0.0,1.0,1.0,location(99.0,74.0),unit_test_nearby_humans)
unit_test_es_planner_action = ActionLimitedSpacePOMDP(pi/12,1.0)
unit_test_env = experiment_environment(100.0,100.0,obstacle_location[])
unit_test_es_pomdp = Limited_space_POMDP_planner(0.99,1.0,-100.0,1.0,-100.0,1.0,100.0,3.0,1.0,10,1.0,unit_test_env)
POMDPs.gen( unit_test_es_pomdp,unit_test_es_planner_state,unit_test_es_planner_action,MersenneTwister(7) )
=#


#=
************************************************************************************************
Upper bound value function for DESPOT
=#

function is_human_collision_state(m::LimitedSpacePOMDP, s::StateLimitedSpacePOMDP, vehicle_center_x, vehicle_center_y)
    if(s.vehicle_v != 0.0)
        for human in s.nearby_humans
            if( is_within_range(vehicle_center_x,vehicle_center_y,human.x,human.y,m.min_safe_distance_from_human+m.vehicle_R) )
                # println("Collision with this human " ,human)
                return true
            end
        end
    end
    return false
end

#This is not accurate for HV or NHV, especially when static obstacles are present. Can we get a better and tighter upper bound?
function time_to_goal(m::LimitedSpacePOMDP,s::StateLimitedSpacePOMDP)
    remaining_path_length = m.rollout_guide.len - s.index_vehicle_controls_sequence
    return floor(remaining_path_length/(m.max_vehicle_speed/m.one_time_step))
end

function calculate_upper_bound(m::LimitedSpacePOMDP, b)

    # lower = lbound(DefaultPolicyLB(FunctionPolicy(b->calculate_lower_bound(m, b)),max_depth=100),m,b)
    value_sum = 0.0
    debug = false
    for (s, w) in weighted_particles(b)
        if(s.vehicle_x == -100.0 && s.vehicle_y == -100.0)
            value_sum += 0.0
        else
            vehicle_center_x = s.vehicle_x + m.vehicle_D*cos(s.vehicle_theta)
            vehicle_center_y = s.vehicle_y + m.vehicle_D*sin(s.vehicle_theta)
            if(is_human_collision_state(m,s,vehicle_center_x,vehicle_center_y))
                value_sum += w*m.human_collision_penalty
                if(debug)
                    println("Upper bound PA is :", value_sum)
                end

            elseif(is_within_range(vehicle_center_x,vehicle_center_y, m.vehicle_goal.x, m.vehicle_goal.y,
                    m.radius_around_vehicle_goal))
                value_sum += w*m.goal_reached_reward
                if(debug)
                    println("Upper bound PB is :", value_sum)
                end
            else
                #############
                map_goal = get_map_target(m.exp_details)
                dist_to_goal = sqrt((s.vehicle_x - map_goal.x)^2 + (s.vehicle_y - map_goal.y)^2)
                value_sum += w * discount(m)^(dist_to_goal/m.max_vehicle_speed) * m.goal_reached_reward
                if(debug)
                    println("Upper bound PC is :", value_sum)
                end
            end
        end
    end
    # println("Upper bound is :", value_sum)
    # u = (value_sum)/weight_sum(b)
    # if lower > value_sum + 1
    #     push!(bad, (lower,value_sum,b))
    #     @show("While debugging ",lower,value_sum,b.depth)
    # end
    return value_sum
end
#@code_warntype calculate_upper_bound_value(golfcart_pomdp(), initialstate_distribution(golfcart_pomdp()))


#=
************************************************************************************************
Lower bound policy function for DESPOT
=#
function calculate_lower_bound(m::LimitedSpacePOMDP,b)
    #Implement a reactive controller for your lower bound
    delta_speed = m.vehicle_action_delta_speed
    vehicle_center_x = 0.0
    vehicle_center_y = 0.0
    #This bool is also used to check if all the states in the belief are terminal or not.
    first_execution_flag = true
    debug = true
    debug=false
    for (s, w) in weighted_particles(b)
        if(s.vehicle_x == -100.0 && s.vehicle_y == -100.0)
            continue
        else
            if(first_execution_flag)
                first_execution_flag = false
                vehicle_center_x = s.vehicle_x + m.vehicle_D*cos(s.vehicle_theta)
                vehicle_center_y = s.vehicle_y + m.vehicle_D*sin(s.vehicle_theta)
                if(debug)
                    println(s)
                end
            end
            dist_to_closest_human = 20000.0  #Some really big infeasible number (not Inf to avoid the type mismatch error)
            for human in s.nearby_humans
                predicted_human_state = update_human_position(human,m.world.length,m.world.breadth,m.one_time_step,0.0)
                euclidean_distance = sqrt((vehicle_center_x - predicted_human_state.x)^2 + (vehicle_center_y - predicted_human_state.y)^2)
                if(euclidean_distance < 2.0 && s.vehicle_v !=0.0 && m.sudden_break_flag)
                    return ActionLimitedSpacePOMDP(-10.0)
                end
                if(euclidean_distance < dist_to_closest_human)
                    dist_to_closest_human = euclidean_distance
                end
                if(dist_to_closest_human < m.d_near)
                    delta_speed = -m.vehicle_action_delta_speed
                    if(debug)
                        println("Near a Human $dist_to_closest_human")
                        println("Inside Delta speed is ", delta_speed)
                        println(ActionLimitedSpacePOMDP(delta_speed))
                    end
                    return ActionLimitedSpacePOMDP(delta_speed)
                end
            end
            if(dist_to_closest_human > m.d_far)
                chosen_delta_speed = delta_speed
            else
                if(s.vehicle_v == 0.0)
                    chosen_delta_speed = delta_speed
                else
                    chosen_delta_speed = 0.0
                end
            end
            if(chosen_delta_speed < delta_speed)
                delta_speed = chosen_delta_speed
            end
        end
    end

    #This condition is true only when all the states in the belief are terminal. In that case, just return (0.0,0.0)
    if(first_execution_flag)
        if(debug)
            println("All are terminal")
            println(ActionLimitedSpacePOMDP(delta_speed))
        end
        return ActionLimitedSpacePOMDP(0.0)
    end
    if(debug)
        println(ActionLimitedSpacePOMDP(delta_speed))
    end
    return ActionLimitedSpacePOMDP(delta_speed)
end

#=
************************************************************************************************
Action Function for the POMDP
=#
function get_actions_with_SB(m::LimitedSpacePOMDP,b)
    # println("************************With SB******************************")
    pomdp_state = first(particles(b))
    if(pomdp_state.vehicle_v == 0.0)
        return (ActionLimitedSpacePOMDP(0.0),
                ActionLimitedSpacePOMDP(m.vehicle_action_delta_speed),
                )
    elseif(pomdp_state.vehicle_v == m.max_vehicle_speed)
        return (ActionLimitedSpacePOMDP(-m.vehicle_action_delta_speed),
                ActionLimitedSpacePOMDP(0.0),
                ActionLimitedSpacePOMDP(-10.0)
                )
    else
        return (ActionLimitedSpacePOMDP(-m.vehicle_action_delta_speed),
                ActionLimitedSpacePOMDP(0.0),
                ActionLimitedSpacePOMDP(m.vehicle_action_delta_speed),
                ActionLimitedSpacePOMDP(-10.0)
                )
    end
end

function get_actions_without_SB(m::LimitedSpacePOMDP,b)
    # println("************************Without SB******************************")
    pomdp_state = first(particles(b))
    if(pomdp_state.vehicle_v == 0.0)
        return (ActionLimitedSpacePOMDP(0.0),
                ActionLimitedSpacePOMDP(m.vehicle_action_delta_speed),
                )
    elseif(pomdp_state.vehicle_v == m.max_vehicle_speed)
        return (ActionLimitedSpacePOMDP(-m.vehicle_action_delta_speed),
                ActionLimitedSpacePOMDP(0.0),
                )
    else
        return (ActionLimitedSpacePOMDP(-m.vehicle_action_delta_speed),
                ActionLimitedSpacePOMDP(0.0),
                ActionLimitedSpacePOMDP(m.vehicle_action_delta_speed),
                )
    end
end

function get_actions(m::LimitedSpacePOMDP,b)
    if(m.sudden_break_flag)
        return get_actions_with_SB(m,b)
    else
        return get_actions_without_SB(m,b)
    end
end

discount(m::LimitedSpacePOMDP) = m.discount_factor

# Modified to use POMDP parameters
function is_terminal_state(m::LimitedSpacePOMDP, s::StateLimitedSpacePOMDP)
    vehicle_center_x = s.vehicle_x + m.vehicle_D*cos(s.vehicle_theta)
    vehicle_center_y = s.vehicle_y + m.vehicle_D*sin(s.vehicle_theta)
    true_target = m.exp_details.true_target
    return is_within_range(vehicle_center_x, vehicle_center_y, 
                          true_target.x, true_target.y, 
                          m.radius_around_vehicle_goal)
end
isterminal(m::LimitedSpacePOMDP, s::StateLimitedSpacePOMDP) = is_terminal_state(m, s)

actions(m::LimitedSpacePOMDP,b) = get_actions(m,b)

# function get_default_action(m::LimitedSpacePOMDP, b::TreeSearchScenarioParameters, ex)
#     # Access POMDP parameters directly from model
#     if m.exp_details.target_belief.pdf[1] > 0.5  # Example: Use MAP probability
#         return ActionLimitedSpacePOMDP(0.0)  # Neutral action
#     else
#         return ActionLimitedSpacePOMDP(-m.vehicle_action_delta_speed)  # Slow down
#     end
# end


function make_default_action(model::LimitedSpacePOMDP)

    goal_radius   = model.radius_around_vehicle_goal
    max_speed     = model.max_vehicle_speed
    dv_max        = model.vehicle_action_delta_speed
    cruise_target = 0.60 * max_speed
    goal          = model.vehicle_goal                 # Location(x, y)

    return (s, _rng) -> begin
        # s :: TreeSearchScenarioParameters
        # positional layout: 1=x, 2=y, 3=θ, 4=v, 5=vehicle_params, …
        x = getfield(s, 1)
        y = getfield(s, 2)
        v = getfield(s, 4)

        dist_to_goal = hypot(goal.x - x, goal.y - y)

        if dist_to_goal ≤ goal_radius
            return ActionLimitedSpacePOMDP(-dv_max)            # brake
        elseif v < cruise_target
            dv = clamp(cruise_target - v, -dv_max, dv_max)     # accelerate
            return ActionLimitedSpacePOMDP(dv)
        else
            return ActionLimitedSpacePOMDP(0.0)                # hold speed
        end
    end
end
