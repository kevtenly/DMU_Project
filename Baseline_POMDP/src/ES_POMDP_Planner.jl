include("pomdp_planning_utils.jl")

#=
Struct for POMDP State
=#
struct StateExtendedSpacePOMDP
    vehicle_x::Float64
    vehicle_y::Float64
    vehicle_theta::Float64
    vehicle_v::Float64
    nearby_humans::Array{HumanState,1}
end

#=
Struct for POMDP Action
=#
struct ActionExtendedSpacePOMDP
    steering_angle::Float64
    delta_speed::Float64
end


#=
Struct for POMDP
=#
struct ExtendedSpacePOMDP{P,N} <: POMDPs.POMDP{StateExtendedSpacePOMDP,ActionExtendedSpacePOMDP,Array{Location,1}}
    discount_factor::Float64
    min_safe_distance_from_human::Float64
    human_collision_penalty::Float64
    min_safe_distance_from_obstacle::Float64
    obstacle_collision_penalty::Float64
    radius_around_vehicle_goal::Float64
    goal_reached_reward::Float64
    vehicle_wheelbase::Float64
    vehicle_length::Float64
    vehicle_breadth::Float64
    vehicle_D::Float64
    vehicle_R::Float64
    max_vehicle_speed::Float64
    max_vehicle_steering_angle::Float64
    vehicle_action_max_delta_heading_angle::Float64
    vehicle_action_delta_speed::Float64
    vehicle_goal::Location
    one_time_step::Float64
    num_segments_in_one_time_step::Int64
    observation_discretization_length::Float64
    d_near::Float64
    d_far::Float64
    sudden_break_flag::Bool
    world::ExperimentEnvironment
    vehicle_path_cache::MVector{N, Tuple{Float64,Float64,Float64}}
    rollout_guide::P
end

function ExtendedSpacePOMDP(pomdp_details,env,vehicle_params,rollout_guide,sudden_break_flag)

    return ExtendedSpacePOMDP(
        pomdp_details.discount_factor,
        pomdp_details.min_safe_distance_from_human,
        pomdp_details.human_collision_penalty,
        pomdp_details.min_safe_distance_from_obstacle,
        pomdp_details.obstacle_collision_penalty,
        pomdp_details.radius_around_vehicle_goal,
        pomdp_details.goal_reached_reward,
        vehicle_params.wheelbase,
        vehicle_params.length,
        vehicle_params.breadth,
        vehicle_params.dist_origin_to_center,
        vehicle_params.radius,
        vehicle_params.max_speed,
        vehicle_params.max_steering_angle,
        pomdp_details.action_max_delta_heading_angle,
        pomdp_details.action_delta_speed,
        vehicle_params.goal,
        pomdp_details.one_time_step,
        pomdp_details.num_segments_in_one_time_step,
        pomdp_details.observation_discretization_length,
        pomdp_details.d_near,
        pomdp_details.d_far,
        sudden_break_flag,
        env,
        MVector{pomdp_details.num_segments_in_one_time_step+1,Tuple{Float64,Float64,Float64}}(undef),
        rollout_guide
        )
end

function Base.rand(rng::AbstractRNG, scenario_params::TreeSearchScenarioParameters{VehicleParametersESPlanner})
    humans = Array{HumanState,1}()
    # return StateExtendedSpacePOMDP(scenario_params.vehicle_x,scenario_params.vehicle_y,scenario_params.vehicle_theta,scenario_params.vehicle_v,humans)
    for i in 1:scenario_params.num_nearby_humans
        sampled_goal = Distributions.rand(rng, SparseCat(scenario_params.human_goals,scenario_params.nearby_humans_belief[i].pdf))
        new_human = HumanState(scenario_params.nearby_humans[i].x,scenario_params.nearby_humans[i].y,scenario_params.nearby_humans[i].v,sampled_goal)
        noise = (rand(rng) - 0.5)*new_human.v*scenario_params.time_duration*0.2
        new_human = update_human_position(new_human,scenario_params.world_length,scenario_params.world_breadth,scenario_params.time_duration,noise)
        push!(humans, new_human)
    end
    return StateExtendedSpacePOMDP(scenario_params.vehicle_x,scenario_params.vehicle_y,scenario_params.vehicle_theta,scenario_params.vehicle_v,humans)
end

#=
************************************************************************************************
Simulate the vehicle one step forward in POMDP planning
=#

function move(vehicle::Tuple{Float64,Float64,Float64},a::Tuple{Float64,Float64},time,vehicle_wheelbase)
    current_x,current_y,current_theta = vehicle
    steering_angle,speed = a
    # vehicle_wheelbase = 1.0

    if(steering_angle == 0.0)
        new_theta = current_theta
        new_x = current_x + speed*cos(current_theta)*time
        new_y = current_y + speed*sin(current_theta)*time
    else
        new_theta = current_theta + (speed * tan(steering_angle) * (time) / vehicle_wheelbase)
        new_theta = wrap_between_0_and_2Pi(new_theta)
        new_x = current_x + ((vehicle_wheelbase / tan(steering_angle)) * (sin(new_theta) - sin(current_theta)))
        new_y = current_y + ((vehicle_wheelbase / tan(steering_angle)) * (cos(current_theta) - cos(new_theta)))
    end
    return (new_x,new_y,new_theta)
end

function update_vehicle_position(m::ExtendedSpacePOMDP, s, steering_angle, new_vehicle_speed)
    current_x, current_y, current_theta = s.vehicle_x, s.vehicle_y, s.vehicle_theta
    curr_vehicle = (current_x, current_y, current_theta)
    N = m.num_segments_in_one_time_step
    vehicle_path = m.vehicle_path_cache
    vehicle_path[1] = curr_vehicle

    if(new_vehicle_speed == 0.0)
        for i in 2:N+1
            vehicle_path[i] = curr_vehicle
        end
    else
        for i in 2:N+1
            new_x,new_y,new_theta = move(curr_vehicle,(steering_angle,new_vehicle_speed),m.one_time_step/N,m.vehicle_wheelbase)
            @inbounds vehicle_path[i] = (new_x,new_y,new_theta)
            current_x,current_y,current_theta = new_x,new_y,new_theta
            curr_vehicle = (current_x, current_y, current_theta)
            vehicle_center_x = current_x + m.vehicle_D*cos(current_theta)
            vehicle_center_y = current_y + m.vehicle_D*sin(current_theta)
            if( intersection((vehicle_center_x,vehicle_center_y),(m.vehicle_goal.x,m.vehicle_goal.y),m.radius_around_vehicle_goal) )
            # if(is_within_range(vehicle_center_x,vehicle_center_y,m.vehicle_goal.x,m.vehicle_goal.y,m.radius_around_vehicle_goal))
                for j in i+1:N+1
                    @inbounds vehicle_path[j] = (current_x, current_y, current_theta)
                end
                return
                # return SVector{N+1,Tuple{Float64,Float64,Float64}}(vehicle_path)
            end
            if( vehicle_center_x<0.0+m.vehicle_R || vehicle_center_y<0.0+m.vehicle_R ||
                vehicle_center_x>m.world.length-m.vehicle_R || vehicle_center_y>m.world.breadth-m.vehicle_R )
                for j in i+1:N+1
                    @inbounds vehicle_path[j] = (current_x, current_y, current_theta)
                end
                return
                # return SVector{N+1,Tuple{Float64,Float64,Float64}}(vehicle_path)
            end
        end
    end
    # return SVector{N+1,Tuple{Float64,Float64,Float64}}(vehicle_path)
end

#=
************************************************************************************************
POMDP Generative Model
=#

function POMDPs.gen(m::ExtendedSpacePOMDP, s, a, rng)

    vehicle_reached_goal = false
    collision_with_human = false
    collision_with_obstacle = false
    immediate_stop = false
    next_human_states = HumanState[]
    observed_positions = Location[]

    # println(s)
    #=
        Check if current vehicle position collides with any nearby human.
        Check if current vehicle position collides with any static obstacle.
        Check if current vehicle position is outside the environment boundary.
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
        if(s.vehicle_v != 0.0 && a.delta_speed != -10.0)
            if( is_within_range(vehicle_center_x, vehicle_center_y, human.x, human.y, m.min_safe_distance_from_human+m.vehicle_R) )
                # println("Collision with this human " ,s.nearby_humans[human_index] , " ", time_index )
                # println("Vehicle's position is " ,vehicle_path[time_index] , "\nHuman's position is ", intermediate_human_location )
                new_vehicle_position = (-100.0, -100.0, -100.0)
                collision_with_human = true
                observed_positions = Location[ Location(-50.0,-50.0) ]
                # observed_positions = Svector{1,Location}(Location(-50.0,-50.0))
                # sp = StateExtendedSpacePOMDP(new_vehicle_position[1],new_vehicle_position[2],new_vehicle_position[3],s.vehicle_v,next_human_states)
                sp = StateExtendedSpacePOMDP(new_vehicle_position[1],new_vehicle_position[2],new_vehicle_position[3],s.vehicle_v,s.nearby_humans)
                r = human_collision_penalty(collision_with_human, m.human_collision_penalty)
                return (sp=sp, o=observed_positions, r=r)
            end
        end
    end

    for obstacle in m.world.obstacles
        if( is_within_range(vehicle_center_x,vehicle_center_y,obstacle.x,obstacle.y,obstacle.r+m.min_safe_distance_from_obstacle+m.vehicle_R) )
            # println("Collision with this obstacle " ,obstacle)
            new_vehicle_position = (-100.0, -100.0, -100.0)
            collision_with_obstacle = true
            observed_positions = Location[ Location(-50.0,-50.0) ]
            sp = StateExtendedSpacePOMDP(new_vehicle_position[1],new_vehicle_position[2],new_vehicle_position[3],s.vehicle_v,next_human_states)
            r = obstacle_collision_penalty(collision_with_obstacle, m.obstacle_collision_penalty)
            return (sp=sp, o=observed_positions, r=r)
        end
    end

    if( vehicle_center_x<0.0+m.vehicle_R || vehicle_center_y<0.0+m.vehicle_R || vehicle_center_x>m.world.length-m.vehicle_R || vehicle_center_y>m.world.breadth-m.vehicle_R )
        # println("Running into boundary wall")
        # println(s.vehicle_x," ",s.vehicle_y," ",s.vehicle_theta*180/pi," ",s.vehicle_v)
        # println(vehicle_center_x," ",vehicle_center_y," ",s.vehicle_theta*180/pi," ",s.vehicle_v)
        new_vehicle_position = (-100.0, -100.0, -100.0)
        collision_with_obstacle = true
        observed_positions = Location[ Location(-50.0,-50.0) ]
        sp = StateExtendedSpacePOMDP(new_vehicle_position[1],new_vehicle_position[2],new_vehicle_position[3],s.vehicle_v,next_human_states)
        r = obstacle_collision_penalty(collision_with_obstacle, m.obstacle_collision_penalty)
        # println(s)
        # println(r)
        return (sp=sp, o=observed_positions, r=r)
    end

    if(is_within_range(vehicle_center_x,vehicle_center_y, m.vehicle_goal.x, m.vehicle_goal.y, m.radius_around_vehicle_goal))
        # println("Goal reached")
        new_vehicle_position = (-100.0, -100.0, -100.0)
        vehicle_reached_goal = true
        observed_positions = Location[ Location(-50.0,-50.0) ]
        sp = StateExtendedSpacePOMDP(new_vehicle_position[1],new_vehicle_position[2],new_vehicle_position[3],s.vehicle_v,next_human_states)
        r = vehicle_goal_reached_reward(vehicle_reached_goal, m.goal_reached_reward)
        # println(r)
        return (sp=sp, o=observed_positions, r=r)
    end

    if(a.delta_speed == -10.0)
        immediate_stop = true
    end
    new_vehicle_speed = clamp(s.vehicle_v + a.delta_speed, 0.0, m.max_vehicle_speed)
    steering_angle = a.steering_angle
    update_vehicle_position(m, s, steering_angle, new_vehicle_speed)
    vehicle_path = m.vehicle_path_cache
    new_vehicle_position = vehicle_path[m.num_segments_in_one_time_step+1]

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
                # println(vehicle_center_x,vehicle_center_y)
                if( is_within_range(vehicle_center_x,vehicle_center_y,human_path_x[time_index],human_path_y[time_index],m.min_safe_distance_from_human+m.vehicle_R) )
                    # println("Collision with this human " ,human , "at time ", time_index )
                    # println("Vehicle's position is " ,vehicle_path[time_index] , "\nHuman's position is ", (human_path_x[time_index],human_path_y[time_index]) )
                    new_vehicle_position = (-100.0, -100.0, -100.0)
                    collision_with_human = true
                    next_human_states = HumanState[]
                    observed_positions = Location[ Location(-50.0,-50.0) ]
                    sp = StateExtendedSpacePOMDP(new_vehicle_position[1],new_vehicle_position[2],new_vehicle_position[3],new_vehicle_speed,next_human_states)
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

    if(new_vehicle_speed!=0.0)
        for time_index in 2:m.num_segments_in_one_time_step+1
            vehicle_center_x = vehicle_path[time_index][1] + m.vehicle_D*cos(vehicle_path[time_index][3])
            vehicle_center_y = vehicle_path[time_index][2] + m.vehicle_D*sin(vehicle_path[time_index][3])
            # println(vehicle_center_x,vehicle_center_y)
            if( vehicle_center_x<0.0+m.vehicle_R || vehicle_center_y<0.0+m.vehicle_R || vehicle_center_x>m.world.length-m.vehicle_R || vehicle_center_y>m.world.breadth-m.vehicle_R )
                # println("Running into boundary wall")
                # println(s.vehicle_x," ",s.vehicle_y," ",s.vehicle_theta*180/pi," ",s.vehicle_v)
                # println(vehicle_center_x," ",vehicle_center_y," ",s.vehicle_theta*180/pi," ",s.vehicle_v)
                new_vehicle_position = (-100.0, -100.0, -100.0)
                collision_with_obstacle = true
                next_human_states = HumanState[]
                observed_positions = Location[ Location(-50.0,-50.0) ]
                sp = StateExtendedSpacePOMDP(new_vehicle_position[1],new_vehicle_position[2],new_vehicle_position[3],s.vehicle_v,next_human_states)
                r = obstacle_collision_penalty(collision_with_obstacle, m.obstacle_collision_penalty)
                return (sp=sp, o=observed_positions, r=r)
            end
            for obstacle in m.world.obstacles
                if( is_within_range(vehicle_center_x,vehicle_center_y,obstacle.x,obstacle.y,m.min_safe_distance_from_obstacle+m.vehicle_R+obstacle.r) )
                    # println("Collision with this obstacle " ,obstacle)
                    new_vehicle_position = (-100.0, -100.0, -100.0)
                    collision_with_obstacle = true
                    next_human_states = HumanState[]
                    observed_positions = Location[ Location(-50.0,-50.0) ]
                    sp = StateExtendedSpacePOMDP(new_vehicle_position[1],new_vehicle_position[2],new_vehicle_position[3],s.vehicle_v,next_human_states)
                    r = obstacle_collision_penalty(collision_with_obstacle, m.obstacle_collision_penalty)
                    return (sp=sp, o=observed_positions, r=r)
                end
            end
        end
    end

    # If the code reaches here, then both s and sp are safe states. Define corresponding new POMDP state.
    sp = StateExtendedSpacePOMDP(new_vehicle_position[1],new_vehicle_position[2],new_vehicle_position[3],new_vehicle_speed,next_human_states)

    # R(s,a): Reward for being at state s and taking action a
    #Penalize if collision with human
    r = low_speed_penalty(s.vehicle_v, m.max_vehicle_speed)
    #println("Reward from not traveling at max_speed ", r)
    #Penalize if had to apply sudden brakes
    r += immediate_stop_penalty(immediate_stop, m.human_collision_penalty)
    #println("Reward if you had to apply immediate brakes", r)
    #Penalize if vehicle's heading angle changes
    r += heading_angle_change_penalty(sp.vehicle_v,a.steering_angle)
    #Penalty to avoid long paths
    r += -1.0
    # println(r)

    # println("State is : $s")
    # println("Next State is $sp")
    # println("Reward is $r")
    # println("***************************************")
    return (sp=sp, o=observed_positions, r=r)
end



#=
************************************************************************************************
Upper bound value function for DESPOT
=#

function is_human_collision_state(m::ExtendedSpacePOMDP, s::StateExtendedSpacePOMDP, vehicle_center_x, vehicle_center_y)
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

function is_obstacle_collision_state(m::ExtendedSpacePOMDP, vehicle_center_x, vehicle_center_y)
    if( vehicle_center_x<0.0+m.vehicle_R || vehicle_center_y<0.0+m.vehicle_R || 
        vehicle_center_x>m.world.length-m.vehicle_R || vehicle_center_y>m.world.breadth-m.vehicle_R )
        return true
    end
    for obstacle in m.world.obstacles
        if( is_within_range(vehicle_center_x,vehicle_center_y,obstacle.x,obstacle.y,m.min_safe_distance_from_obstacle+obstacle.r+m.vehicle_R) )
            # println("Collision with this obstacle " ,obstacle)
            return true
        end
    end
    return false
end

#This is not accurate for HV or NHV, especially when static obstacles are present. Can we get a better and tighter upper bound?
function time_to_goal(m::ExtendedSpacePOMDP, vehicle_center_x, vehicle_center_y)
    vehicle_distance_to_goal = sqrt( (vehicle_center_x-m.vehicle_goal.x)^2 + (vehicle_center_y-m.vehicle_goal.y)^2 )
    # println("Distance is :", vehicle_distance_to_goal)
    # println(s)
    # println("Time is :", floor(vehicle_distance_to_goal/m.max_vehicle_speed))
    # return floor(vehicle_distance_to_goal/m.max_vehicle_speed/m.one_time_step)
    return floor(vehicle_distance_to_goal/m.max_vehicle_speed)
end


function calculate_upper_bound(m::ExtendedSpacePOMDP, b)

    # lower = lbound(DefaultPolicyLB(FunctionPolicy(b->calculate_lower_bound(extended_space_pomdp, b)),max_depth=100),m,b)
    # lower = lbound(DefaultPolicyLB(FunctionPolicy(b->calculate_lower_bound(m, b)),max_depth=pomdp_details.tree_search_max_depth),m,b)
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
            elseif(is_obstacle_collision_state(m,vehicle_center_x,vehicle_center_y))
                value_sum += w*m.obstacle_collision_penalty
                if(debug)
                    println("Upper bound PB is :", value_sum)
                end
            elseif(is_within_range(vehicle_center_x,vehicle_center_y,m.vehicle_goal.x, m.vehicle_goal.y, 
                    m.radius_around_vehicle_goal))
                value_sum += w*m.goal_reached_reward
                if(debug)
                    println("Upper bound PC is :", value_sum)
                end
            else
                value_sum += w*((discount(m)^time_to_goal(m,vehicle_center_x,vehicle_center_y))*m.goal_reached_reward)
                if(debug)
                    println("Upper bound PC is :", value_sum)
                end
            end
        end
    end
    # println("Upper bound is :", value_sum)
    # u = (value_sum)/weight_sum(b)
    # if lower > value_sum
    #     push!(bad, (lower,value_sum,b))
    #     @show("While debugging ",lower,value_sum,b.depth)
    # end
    return value_sum
end
# @code_warntype calculate_upper_bound_value(golfcart_pomdp(), initialstate_distribution(golfcart_pomdp()))
#=
b = output.b_root[16.0]
s = rand(b)
root_scenarios = [i=>s for i in 1:50];
belief = ScenarioBelief(root_scenarios,pomdp_planner.rs, 1, missing);
calculate_upper_bound(extended_space_pomdp,belief)
=#


#=
************************************************************************************************
Lower bound policy function for DESPOT
=#
function calculate_lower_bound(m::ExtendedSpacePOMDP,b)
    #Implement a reactive controller for your lower bound
    #=
    Predict the vehicle position forward at same speed and after increasing the speed.
    If there is a collision of the predicted position with any human, return the SB action.
    If not, move according to the usual reactive controller policy.
    =#
    delta_speed = m.vehicle_action_delta_speed
    vehicle_center_x = 0.0
    vehicle_center_y = 0.0
    #This bool is also used to check if all the states in the belief are terminal or not.
    first_execution_flag = true
    # if(length(b.scenarios)==1)
    #     println("Tree Depth : ", b.depth)
    #     println(b.scenarios)
    # end
    debug = true
    debug = false
    if(debug)
        fp = first(particles(b))
        println([fp.vehicle_x,fp.vehicle_y,fp.vehicle_theta,fp.vehicle_v])
    end
    for (s, w) in weighted_particles(b)
        if(s.vehicle_x == -100.0 && s.vehicle_y == -100.0)
            continue
        else
            if(first_execution_flag)
                first_execution_flag = false
                vehicle_center_x = s.vehicle_x + m.vehicle_D*cos(s.vehicle_theta)
                vehicle_center_y = s.vehicle_y + m.vehicle_D*sin(s.vehicle_theta)
            end
            dist_to_closest_human = Inf 
            for human in s.nearby_humans
                predicted_human_state = update_human_position(human,m.world.length,m.world.breadth,m.one_time_step,0.0)
                euclidean_distance = sqrt((vehicle_center_x - predicted_human_state.x)^2 + (vehicle_center_y - predicted_human_state.y)^2)
                # if(debug)
                #     println("Current Euclidean Distance is $euclidean_distance")
                # end
                if(euclidean_distance < 2.0 && s.vehicle_v != 0.0 && m.sudden_break_flag)
                    return ActionExtendedSpacePOMDP(-10.0,-10.0)
                end
                if(euclidean_distance < dist_to_closest_human)
                    dist_to_closest_human = euclidean_distance
                end
                # if(dist_to_closest_human < 2.0)
                # if(dist_to_closest_human < 3.2)
                #     # return ActionExtendedSpacePOMDP(-10.0,-10.0)
                #     if(s.vehicle_v > m.vehicle_action_delta_speed)
                #         return ActionExtendedSpacePOMDP(-10.0,-10.0)
                #     elseif(s.vehicle_v == m.vehicle_action_delta_speed)
                #         return ActionExtendedSpacePOMDP(0.0,-delta_speed)
                #     else
                #         return ActionExtendedSpacePOMDP(0.0,0.0)
                #     end
                # end
                if(dist_to_closest_human < m.d_near)
                    delta_speed = -m.vehicle_action_delta_speed
                    state = SVector(s.vehicle_x,s.vehicle_y,wrap_between_negative_pi_to_pi(s.vehicle_theta),s.vehicle_v)
                    a = BPDE.reactive_controller_HJB_policy(m.rollout_guide,state,delta_speed)
                    if(debug)
                        println("Near a Human $dist_to_closest_human")
                        println("Inside Delta speed is ", delta_speed)
                        println( ActionExtendedSpacePOMDP(a[1],delta_speed) )
                    end
                    return ActionExtendedSpacePOMDP(a[1],delta_speed)
                end
            end
            if(dist_to_closest_human > m.d_far)
                chosen_delta_speed = m.vehicle_action_delta_speed
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
            println(ActionExtendedSpacePOMDP(0.0,0.0))
        end
        return ActionExtendedSpacePOMDP(0.0,0.0)
    end
    if(debug)
        println("Outside (final) Delta speed is ", delta_speed)
    end
    s = first(particles(b))
    state = SVector(s.vehicle_x,s.vehicle_y,wrap_between_negative_pi_to_pi(s.vehicle_theta),s.vehicle_v)
    a = BPDE.reactive_controller_HJB_policy(m.rollout_guide,state,delta_speed)
    if(debug)
        @show(delta_speed)
        println( ActionExtendedSpacePOMDP(a[1],a[2]) )
    end
    return ActionExtendedSpacePOMDP(a[1],a[2])
end

#=
************************************************************************************************
Action Function for the POMDP
=#

function get_actions_without_SB_HJB_rollout(m::ExtendedSpacePOMDP,b)
    max_steering_angle = m.max_vehicle_steering_angle
    max_delta_angle = m.vehicle_action_max_delta_heading_angle
    delta_speed = m.vehicle_action_delta_speed
    pomdp_state = first(particles(b))
    if(pomdp_state.vehicle_v == 0.0)
        steering_angle = clamp(get_steering_angle(m.vehicle_wheelbase,max_delta_angle,delta_speed,m.one_time_step),
                                    0.0,max_steering_angle)
        return (ActionExtendedSpacePOMDP(-steering_angle,delta_speed),
                ActionExtendedSpacePOMDP(-2*steering_angle/3,delta_speed),
                ActionExtendedSpacePOMDP(-steering_angle/3,delta_speed),
                ActionExtendedSpacePOMDP(0.0,delta_speed),
                ActionExtendedSpacePOMDP(0.0,0.0),
                ActionExtendedSpacePOMDP(steering_angle/3,delta_speed),
                ActionExtendedSpacePOMDP(2*steering_angle/3,delta_speed),
                ActionExtendedSpacePOMDP(steering_angle,delta_speed)
                )
    elseif(pomdp_state.vehicle_v == m.max_vehicle_speed)
        steering_angle = clamp(get_steering_angle(m.vehicle_wheelbase,max_delta_angle,pomdp_state.vehicle_v,m.one_time_step),
                                0.0,max_steering_angle)
        steering_angle_n = clamp(get_steering_angle(m.vehicle_wheelbase, max_delta_angle, pomdp_state.vehicle_v-delta_speed, m.one_time_step),
                                0.0,max_steering_angle)
        state = SVector(pomdp_state.vehicle_x,pomdp_state.vehicle_y,wrap_between_negative_pi_to_pi(pomdp_state.vehicle_theta),
                            pomdp_state.vehicle_v)
        rollout_action = BPDE.reactive_controller_HJB_policy(m.rollout_guide,state,delta_speed)                        
        # rollout_action = better_reactive_policy(SVector(pomdp_state.vehicle_x,pomdp_state.vehicle_y,wrap_between_negative_pi_to_pi(pomdp_state.vehicle_theta),pomdp_state.vehicle_v),
        #                         delta_speed,750.0,m.rollout_guide.get_actions, m.rollout_guide.get_cost,m.one_time_step,m.rollout_guide.q_value_array,
        #                         m.rollout_guide.value_array,m.rollout_guide.veh,m.rollout_guide.state_grid)
        if( rollout_action[2] == -delta_speed &&
            (rollout_action[1]!=0.0 || rollout_action[1]!=steering_angle_n || rollout_action[1]!=-steering_angle_n) )
            return (
                    ActionExtendedSpacePOMDP(-steering_angle,0.0),
                    ActionExtendedSpacePOMDP(-2*steering_angle/3,0.0),
                    ActionExtendedSpacePOMDP(-steering_angle/3,0.0),
                    ActionExtendedSpacePOMDP(0.0,-delta_speed),
                    ActionExtendedSpacePOMDP(0.0,0.0),
                    ActionExtendedSpacePOMDP(steering_angle/3,0.0),
                    ActionExtendedSpacePOMDP(2*steering_angle/3,0.0),
                    ActionExtendedSpacePOMDP(steering_angle,0.0),
                    ActionExtendedSpacePOMDP(rollout_action[1],rollout_action[2]),
                    ActionExtendedSpacePOMDP(steering_angle_n,-delta_speed),
                    ActionExtendedSpacePOMDP(-steering_angle_n,-delta_speed),
                    )
        else
            return (
                    ActionExtendedSpacePOMDP(-steering_angle,0.0),
                    ActionExtendedSpacePOMDP(-2*steering_angle/3,0.0),
                    ActionExtendedSpacePOMDP(-steering_angle/3,0.0),
                    ActionExtendedSpacePOMDP(0.0,-delta_speed),
                    ActionExtendedSpacePOMDP(0.0,0.0),
                    ActionExtendedSpacePOMDP(steering_angle/3,0.0),
                    ActionExtendedSpacePOMDP(2*steering_angle/3,0.0),
                    ActionExtendedSpacePOMDP(steering_angle,0.0),
                    ActionExtendedSpacePOMDP(steering_angle_n,-delta_speed),
                    ActionExtendedSpacePOMDP(-steering_angle_n,-delta_speed),
                    )
        end
    else
        steering_angle = clamp(get_steering_angle(m.vehicle_wheelbase,max_delta_angle,pomdp_state.vehicle_v,m.one_time_step),
                            0.0,max_steering_angle)
        steering_angle_n = clamp(get_steering_angle(m.vehicle_wheelbase, max_delta_angle, pomdp_state.vehicle_v-delta_speed, m.one_time_step),
                            0.0, max_steering_angle)
        state = SVector(pomdp_state.vehicle_x,pomdp_state.vehicle_y,wrap_between_negative_pi_to_pi(pomdp_state.vehicle_theta),
                            pomdp_state.vehicle_v)
        rollout_action = BPDE.reactive_controller_HJB_policy(m.rollout_guide,state,delta_speed)
        # rollout_action = better_reactive_policy(SVector(pomdp_state.vehicle_x,pomdp_state.vehicle_y,wrap_between_negative_pi_to_pi(pomdp_state.vehicle_theta),pomdp_state.vehicle_v),
        #                         delta_speed,750.0,m.rollout_guide.get_actions, m.rollout_guide.get_cost,m.one_time_step,m.rollout_guide.q_value_array,
        #                         m.rollout_guide.value_array,m.rollout_guide.veh,m.rollout_guide.state_grid)
        if(
                ( rollout_action[2] == -delta_speed && (rollout_action[1] != steering_angle_n
                    || rollout_action[1] != -steering_angle_n) || rollout_action[1] != 0.0 ) ||
                ( rollout_action[2] == delta_speed && rollout_action[1] != 0.0 )
            )
            return (ActionExtendedSpacePOMDP(-steering_angle,0.0),
                    ActionExtendedSpacePOMDP(-2*steering_angle/3,0.0),
                    ActionExtendedSpacePOMDP(-steering_angle/3,0.0),
                    ActionExtendedSpacePOMDP(0.0,-delta_speed),
                    ActionExtendedSpacePOMDP(0.0,0.0),
                    ActionExtendedSpacePOMDP(0.0,delta_speed),
                    ActionExtendedSpacePOMDP(steering_angle/3,0.0),
                    ActionExtendedSpacePOMDP(2*steering_angle/3,0.0),
                    ActionExtendedSpacePOMDP(steering_angle,0.0),
                    ActionExtendedSpacePOMDP(rollout_action[1],rollout_action[2]),
                    ActionExtendedSpacePOMDP(steering_angle_n,-delta_speed),
                    ActionExtendedSpacePOMDP(-steering_angle_n,-delta_speed),
                    )
        else
            return (ActionExtendedSpacePOMDP(-steering_angle,0.0),
                    ActionExtendedSpacePOMDP(-2*steering_angle/3,0.0),
                    ActionExtendedSpacePOMDP(-steering_angle/3,0.0),
                    ActionExtendedSpacePOMDP(0.0,-delta_speed),
                    ActionExtendedSpacePOMDP(0.0,0.0),
                    ActionExtendedSpacePOMDP(0.0,delta_speed),
                    ActionExtendedSpacePOMDP(steering_angle/3,0.0),
                    ActionExtendedSpacePOMDP(2*steering_angle/3,0.0),
                    ActionExtendedSpacePOMDP(steering_angle,0.0),
                    ActionExtendedSpacePOMDP(steering_angle_n,-delta_speed),
                    ActionExtendedSpacePOMDP(-steering_angle_n,-delta_speed),
                    )
        end
    end
end

function get_actions_with_SB_HJB_rollout(m::ExtendedSpacePOMDP,b)
    max_steering_angle = m.max_vehicle_steering_angle
    max_delta_angle = m.vehicle_action_max_delta_heading_angle
    delta_speed = m.vehicle_action_delta_speed
    pomdp_state = first(particles(b))
    if(pomdp_state.vehicle_v == 0.0)
        steering_angle = clamp(get_steering_angle(m.vehicle_wheelbase,max_delta_angle,delta_speed,m.one_time_step),0.0,max_steering_angle)
        return (ActionExtendedSpacePOMDP(-steering_angle,delta_speed),
                ActionExtendedSpacePOMDP(-2*steering_angle/3,delta_speed),
                ActionExtendedSpacePOMDP(-steering_angle/3,delta_speed),
                ActionExtendedSpacePOMDP(0.0,delta_speed),
                ActionExtendedSpacePOMDP(0.0,0.0),
                ActionExtendedSpacePOMDP(steering_angle/3,delta_speed),
                ActionExtendedSpacePOMDP(2*steering_angle/3,delta_speed),
                ActionExtendedSpacePOMDP(steering_angle,delta_speed)
                )
    elseif(pomdp_state.vehicle_v == m.max_vehicle_speed)
        steering_angle = clamp(get_steering_angle(m.vehicle_wheelbase,max_delta_angle,pomdp_state.vehicle_v,m.one_time_step),0.0,max_steering_angle)
        state = SVector(pomdp_state.vehicle_x,pomdp_state.vehicle_y,wrap_between_negative_pi_to_pi(pomdp_state.vehicle_theta),
                            pomdp_state.vehicle_v)
        rollout_action = BPDE.reactive_controller_HJB_policy(m.rollout_guide,state,delta_speed)
        # rollout_action = better_reactive_policy(SVector(pomdp_state.vehicle_x,pomdp_state.vehicle_y,wrap_between_negative_pi_to_pi(pomdp_state.vehicle_theta),pomdp_state.vehicle_v),
        #                         delta_speed,750.0,m.rollout_guide.get_actions, m.rollout_guide.get_cost,m.one_time_step,m.rollout_guide.q_value_array,
        #                         m.rollout_guide.value_array,m.rollout_guide.veh,m.rollout_guide.state_grid)
        if(rollout_action[1]!=0.0 && rollout_action[2] == -delta_speed)
            return (
                    ActionExtendedSpacePOMDP(-steering_angle,0.0),
                    ActionExtendedSpacePOMDP(-2*steering_angle/3,0.0),
                    ActionExtendedSpacePOMDP(-steering_angle/3,0.0),
                    ActionExtendedSpacePOMDP(0.0,-delta_speed),
                    ActionExtendedSpacePOMDP(0.0,0.0),
                    ActionExtendedSpacePOMDP(steering_angle/3,0.0),
                    ActionExtendedSpacePOMDP(2*steering_angle/3,0.0),
                    ActionExtendedSpacePOMDP(steering_angle,0.0),
                    ActionExtendedSpacePOMDP(rollout_action[1],rollout_action[2]),
                    ActionExtendedSpacePOMDP(-10.0,-10.0)
                    )
        else
            return (
                    ActionExtendedSpacePOMDP(-steering_angle,0.0),
                    ActionExtendedSpacePOMDP(-2*steering_angle/3,0.0),
                    ActionExtendedSpacePOMDP(-steering_angle/3,0.0),
                    ActionExtendedSpacePOMDP(0.0,-delta_speed),
                    ActionExtendedSpacePOMDP(0.0,0.0),
                    ActionExtendedSpacePOMDP(steering_angle/3,0.0),
                    ActionExtendedSpacePOMDP(2*steering_angle/3,0.0),
                    ActionExtendedSpacePOMDP(steering_angle,0.0),
                    ActionExtendedSpacePOMDP(-10.0,-10.0)
                    )
        end
    else
        steering_angle = clamp(get_steering_angle(m.vehicle_wheelbase,max_delta_angle,pomdp_state.vehicle_v,m.one_time_step),0.0,max_steering_angle)
        state = SVector(pomdp_state.vehicle_x,pomdp_state.vehicle_y,wrap_between_negative_pi_to_pi(pomdp_state.vehicle_theta),
                            pomdp_state.vehicle_v)
        rollout_action = BPDE.reactive_controller_HJB_policy(m.rollout_guide,state,delta_speed)
        # rollout_action= better_reactive_policy(SVector(pomdp_state.vehicle_x,pomdp_state.vehicle_y,wrap_between_negative_pi_to_pi(pomdp_state.vehicle_theta),pomdp_state.vehicle_v),
        #                         delta_speed,750.0,m.rollout_guide.get_actions, m.rollout_guide.get_cost,m.one_time_step,m.rollout_guide.q_value_array,
        #                         m.rollout_guide.value_array,m.rollout_guide.veh,m.rollout_guide.state_grid)
        if(rollout_action[1]!=0.0 && rollout_action[2] != 0.0 )
            return (ActionExtendedSpacePOMDP(-steering_angle,0.0),
                    ActionExtendedSpacePOMDP(-2*steering_angle/3,0.0),
                    ActionExtendedSpacePOMDP(-steering_angle/3,0.0),
                    ActionExtendedSpacePOMDP(0.0,-delta_speed),
                    ActionExtendedSpacePOMDP(0.0,0.0),
                    ActionExtendedSpacePOMDP(0.0,delta_speed),
                    ActionExtendedSpacePOMDP(steering_angle/3,0.0),
                    ActionExtendedSpacePOMDP(2*steering_angle/3,0.0),
                    ActionExtendedSpacePOMDP(steering_angle,0.0),
                    ActionExtendedSpacePOMDP(rollout_action[1],rollout_action[2]),
                    ActionExtendedSpacePOMDP(-10.0,-10.0)
                    )
        else
            return (ActionExtendedSpacePOMDP(-steering_angle,0.0),
                    ActionExtendedSpacePOMDP(-2*steering_angle/3,0.0),
                    ActionExtendedSpacePOMDP(-steering_angle/3,0.0),
                    ActionExtendedSpacePOMDP(0.0,-delta_speed),
                    ActionExtendedSpacePOMDP(0.0,0.0),
                    ActionExtendedSpacePOMDP(0.0,delta_speed),
                    ActionExtendedSpacePOMDP(steering_angle/3,0.0),
                    ActionExtendedSpacePOMDP(2*steering_angle/3,0.0),
                    ActionExtendedSpacePOMDP(steering_angle,0.0),
                    ActionExtendedSpacePOMDP(-10.0,-10.0)
                    )
        end
    end
end

function get_actions_without_SB_random_rollout(m::ExtendedSpacePOMDP,b)
    max_steering_angle = m.max_vehicle_steering_angle
    max_delta_angle = m.vehicle_action_max_delta_heading_angle
    delta_speed = m.vehicle_action_delta_speed
    pomdp_state = first(particles(b))
    if(pomdp_state.vehicle_v == 0.0)
        steering_angle = clamp(get_steering_angle(m.vehicle_wheelbase,max_delta_angle,delta_speed,m.one_time_step),0.0,max_steering_angle)
        return (ActionExtendedSpacePOMDP(-steering_angle,delta_speed),
                ActionExtendedSpacePOMDP(-2*steering_angle/3,delta_speed),
                ActionExtendedSpacePOMDP(-steering_angle/3,delta_speed),
                ActionExtendedSpacePOMDP(0.0,delta_speed),
                ActionExtendedSpacePOMDP(0.0,0.0),
                ActionExtendedSpacePOMDP(steering_angle/3,delta_speed),
                ActionExtendedSpacePOMDP(2*steering_angle/3,delta_speed),
                ActionExtendedSpacePOMDP(steering_angle,delta_speed)
                )
    elseif(pomdp_state.vehicle_v == m.max_vehicle_speed)
        steering_angle = clamp(get_steering_angle(m.vehicle_wheelbase,max_delta_angle,pomdp_state.vehicle_v,m.one_time_step),
                                0.0,max_steering_angle)
        steering_angle_n = clamp(get_steering_angle(m.vehicle_wheelbase, max_delta_angle, pomdp_state.vehicle_v-delta_speed, m.one_time_step),
                                0.0,max_steering_angle)
        return (
                ActionExtendedSpacePOMDP(-steering_angle,0.0),
                ActionExtendedSpacePOMDP(-2*steering_angle/3,0.0),
                ActionExtendedSpacePOMDP(-steering_angle/3,0.0),
                ActionExtendedSpacePOMDP(0.0,-delta_speed),
                ActionExtendedSpacePOMDP(0.0,0.0),
                ActionExtendedSpacePOMDP(steering_angle/3,0.0),
                ActionExtendedSpacePOMDP(2*steering_angle/3,0.0),
                ActionExtendedSpacePOMDP(steering_angle,0.0),
                ActionExtendedSpacePOMDP(steering_angle_n,-delta_speed),
                ActionExtendedSpacePOMDP(-steering_angle_n,-delta_speed),
                )
    else
        steering_angle = clamp(get_steering_angle(m.vehicle_wheelbase,max_delta_angle,pomdp_state.vehicle_v,m.one_time_step),
                            0.0,max_steering_angle)
        steering_angle_n = clamp(get_steering_angle(m.vehicle_wheelbase, max_delta_angle, pomdp_state.vehicle_v-delta_speed, m.one_time_step),
                            0.0, max_steering_angle)
        return (ActionExtendedSpacePOMDP(-steering_angle,0.0),
                ActionExtendedSpacePOMDP(-2*steering_angle/3,0.0),
                ActionExtendedSpacePOMDP(-steering_angle/3,0.0),
                ActionExtendedSpacePOMDP(0.0,-delta_speed),
                ActionExtendedSpacePOMDP(0.0,0.0),
                ActionExtendedSpacePOMDP(0.0,delta_speed),
                ActionExtendedSpacePOMDP(steering_angle/3,0.0),
                ActionExtendedSpacePOMDP(2*steering_angle/3,0.0),
                ActionExtendedSpacePOMDP(steering_angle,0.0),
                ActionExtendedSpacePOMDP(steering_angle_n,-delta_speed),
                ActionExtendedSpacePOMDP(-steering_angle_n,-delta_speed),
                )
    end
end

function get_actions_with_SB_random_rollout(m::ExtendedSpacePOMDP,b)
    max_steering_angle = m.max_vehicle_steering_angle
    max_delta_angle = m.vehicle_action_max_delta_heading_angle
    delta_speed = m.vehicle_action_delta_speed
    pomdp_state = first(particles(b))
    if(pomdp_state.vehicle_v == 0.0)
        steering_angle = clamp(get_steering_angle(m.vehicle_wheelbase,max_delta_angle,delta_speed,m.one_time_step),0.0,max_steering_angle)
        return (ActionExtendedSpacePOMDP(-steering_angle,delta_speed),
                ActionExtendedSpacePOMDP(-2*steering_angle/3,delta_speed),
                ActionExtendedSpacePOMDP(-steering_angle/3,delta_speed),
                ActionExtendedSpacePOMDP(0.0,delta_speed),
                ActionExtendedSpacePOMDP(0.0,0.0),
                ActionExtendedSpacePOMDP(steering_angle/3,delta_speed),
                ActionExtendedSpacePOMDP(2*steering_angle/3,delta_speed),
                ActionExtendedSpacePOMDP(steering_angle,delta_speed)
                )
    elseif(pomdp_state.vehicle_v == m.max_vehicle_speed)
        steering_angle = clamp(get_steering_angle(m.vehicle_wheelbase,max_delta_angle,pomdp_state.vehicle_v,m.one_time_step),0.0,max_steering_angle)
        return (
                ActionExtendedSpacePOMDP(-steering_angle,0.0),
                ActionExtendedSpacePOMDP(-2*steering_angle/3,0.0),
                ActionExtendedSpacePOMDP(-steering_angle/3,0.0),
                ActionExtendedSpacePOMDP(0.0,-delta_speed),
                ActionExtendedSpacePOMDP(0.0,0.0),
                ActionExtendedSpacePOMDP(steering_angle/3,0.0),
                ActionExtendedSpacePOMDP(2*steering_angle/3,0.0),
                ActionExtendedSpacePOMDP(steering_angle,0.0),
                ActionExtendedSpacePOMDP(-10.0,-10.0)
                )
    else
        steering_angle = clamp(get_steering_angle(m.vehicle_wheelbase,max_delta_angle,pomdp_state.vehicle_v,m.one_time_step),0.0,max_steering_angle)
        return (ActionExtendedSpacePOMDP(-steering_angle,0.0),
                ActionExtendedSpacePOMDP(-2*steering_angle/3,0.0),
                ActionExtendedSpacePOMDP(-steering_angle/3,0.0),
                ActionExtendedSpacePOMDP(0.0,-delta_speed),
                ActionExtendedSpacePOMDP(0.0,0.0),
                ActionExtendedSpacePOMDP(0.0,delta_speed),
                ActionExtendedSpacePOMDP(steering_angle/3,0.0),
                ActionExtendedSpacePOMDP(2*steering_angle/3,0.0),
                ActionExtendedSpacePOMDP(steering_angle,0.0),
                ActionExtendedSpacePOMDP(-10.0,-10.0)
                )
    end
end

function get_actions_without_SB_straight_line_rollout(m::ExtendedSpacePOMDP,b)
    max_steering_angle = m.max_vehicle_steering_angle
    max_delta_angle = m.vehicle_action_max_delta_heading_angle
    delta_speed = m.vehicle_action_delta_speed
    pomdp_state = first(particles(b))
    if(pomdp_state.vehicle_v == 0.0)
        steering_angle = clamp(get_steering_angle(m.vehicle_wheelbase,max_delta_angle,delta_speed,m.one_time_step),0.0,max_steering_angle)
        return (ActionExtendedSpacePOMDP(-steering_angle,delta_speed),
                ActionExtendedSpacePOMDP(-2*steering_angle/3,delta_speed),
                ActionExtendedSpacePOMDP(-steering_angle/3,delta_speed),
                ActionExtendedSpacePOMDP(0.0,delta_speed),
                ActionExtendedSpacePOMDP(0.0,0.0),
                ActionExtendedSpacePOMDP(steering_angle/3,delta_speed),
                ActionExtendedSpacePOMDP(2*steering_angle/3,delta_speed),
                ActionExtendedSpacePOMDP(steering_angle,delta_speed)
                )
    elseif(pomdp_state.vehicle_v == m.max_vehicle_speed)
        steering_angle = clamp(get_steering_angle(m.vehicle_wheelbase,max_delta_angle,pomdp_state.vehicle_v,m.one_time_step),
                                0.0,max_steering_angle)
        steering_angle_n = clamp(get_steering_angle(m.vehicle_wheelbase, max_delta_angle, pomdp_state.vehicle_v-delta_speed, m.one_time_step),
                                0.0,max_steering_angle)
        rollout_delta_angle = get_straight_line_action(m, pomdp_state) #Just an angle value
        rollout_steering_angle = clamp(get_steering_angle(m.vehicle_wheelbase,rollout_delta_angle,pomdp_state.vehicle_v,m.one_time_step),-max_steering_angle,max_steering_angle)
        rollout_action = (rollout_steering_angle,0.0)
        if( rollout_action[1]!=0.0 && rollout_action[1]!= steering_angle && rollout_action[1]!=-steering_angle )
            return (
                    ActionExtendedSpacePOMDP(-steering_angle,0.0),
                    ActionExtendedSpacePOMDP(-2*steering_angle/3,0.0),
                    ActionExtendedSpacePOMDP(-steering_angle/3,0.0),
                    ActionExtendedSpacePOMDP(0.0,-delta_speed),
                    ActionExtendedSpacePOMDP(0.0,0.0),
                    ActionExtendedSpacePOMDP(steering_angle/3,0.0),
                    ActionExtendedSpacePOMDP(2*steering_angle/3,0.0),
                    ActionExtendedSpacePOMDP(steering_angle,0.0),
                    ActionExtendedSpacePOMDP(rollout_action[1],rollout_action[2]),
                    ActionExtendedSpacePOMDP(steering_angle_n,-delta_speed),
                    ActionExtendedSpacePOMDP(-steering_angle_n,-delta_speed),
                    )
        else
            return (
                    ActionExtendedSpacePOMDP(-steering_angle,0.0),
                    ActionExtendedSpacePOMDP(-2*steering_angle/3,0.0),
                    ActionExtendedSpacePOMDP(-steering_angle/3,0.0),
                    ActionExtendedSpacePOMDP(0.0,-delta_speed),
                    ActionExtendedSpacePOMDP(0.0,0.0),
                    ActionExtendedSpacePOMDP(steering_angle/3,0.0),
                    ActionExtendedSpacePOMDP(2*steering_angle/3,0.0),
                    ActionExtendedSpacePOMDP(steering_angle,0.0),
                    ActionExtendedSpacePOMDP(steering_angle_n,-delta_speed),
                    ActionExtendedSpacePOMDP(-steering_angle_n,-delta_speed),
                    )
        end
    else
        steering_angle = clamp(get_steering_angle(m.vehicle_wheelbase,max_delta_angle,pomdp_state.vehicle_v,m.one_time_step),
                            0.0,max_steering_angle)
        steering_angle_n = clamp(get_steering_angle(m.vehicle_wheelbase, max_delta_angle, pomdp_state.vehicle_v-delta_speed, m.one_time_step),
                            0.0, max_steering_angle)
        rollout_delta_angle = get_straight_line_action(m, pomdp_state) #Just an angle value
        rollout_steering_angle = clamp(get_steering_angle(m.vehicle_wheelbase,rollout_delta_angle,pomdp_state.vehicle_v,m.one_time_step),-max_steering_angle,max_steering_angle)
        rollout_action = (rollout_steering_angle,0.0)
        if( rollout_action[1]!=0.0 && rollout_action[1]!=steering_angle && rollout_action[1]!=-steering_angle )
            return (ActionExtendedSpacePOMDP(-steering_angle,0.0),
                    ActionExtendedSpacePOMDP(-2*steering_angle/3,0.0),
                    ActionExtendedSpacePOMDP(-steering_angle/3,0.0),
                    ActionExtendedSpacePOMDP(0.0,-delta_speed),
                    ActionExtendedSpacePOMDP(0.0,0.0),
                    ActionExtendedSpacePOMDP(0.0,delta_speed),
                    ActionExtendedSpacePOMDP(steering_angle/3,0.0),
                    ActionExtendedSpacePOMDP(2*steering_angle/3,0.0),
                    ActionExtendedSpacePOMDP(steering_angle,0.0),
                    ActionExtendedSpacePOMDP(rollout_action[1],rollout_action[2]),
                    ActionExtendedSpacePOMDP(steering_angle_n,-delta_speed),
                    ActionExtendedSpacePOMDP(-steering_angle_n,-delta_speed),
                    )
        else
            return (ActionExtendedSpacePOMDP(-steering_angle,0.0),
                    ActionExtendedSpacePOMDP(-2*steering_angle/3,0.0),
                    ActionExtendedSpacePOMDP(-steering_angle/3,0.0),
                    ActionExtendedSpacePOMDP(0.0,-delta_speed),
                    ActionExtendedSpacePOMDP(0.0,0.0),
                    ActionExtendedSpacePOMDP(0.0,delta_speed),
                    ActionExtendedSpacePOMDP(steering_angle/3,0.0),
                    ActionExtendedSpacePOMDP(2*steering_angle/3,0.0),
                    ActionExtendedSpacePOMDP(steering_angle,0.0),
                    ActionExtendedSpacePOMDP(steering_angle_n,-delta_speed),
                    ActionExtendedSpacePOMDP(-steering_angle_n,-delta_speed),
                    )
        end
    end
end

function get_actions_with_SB_straight_line_rollout(m::ExtendedSpacePOMDP,b)
    max_steering_angle = m.max_vehicle_steering_angle
    max_delta_angle = m.vehicle_action_max_delta_heading_angle
    delta_speed = m.vehicle_action_delta_speed
    pomdp_state = first(particles(b))
    if(pomdp_state.vehicle_v == 0.0)
        steering_angle = clamp(get_steering_angle(m.vehicle_wheelbase,max_delta_angle,delta_speed,m.one_time_step),0.0,max_steering_angle)
        return (ActionExtendedSpacePOMDP(-steering_angle,delta_speed),
                ActionExtendedSpacePOMDP(-2*steering_angle/3,delta_speed),
                ActionExtendedSpacePOMDP(-steering_angle/3,delta_speed),
                ActionExtendedSpacePOMDP(0.0,delta_speed),
                ActionExtendedSpacePOMDP(0.0,0.0),
                ActionExtendedSpacePOMDP(steering_angle/3,delta_speed),
                ActionExtendedSpacePOMDP(2*steering_angle/3,delta_speed),
                ActionExtendedSpacePOMDP(steering_angle,delta_speed)
                )
    elseif(pomdp_state.vehicle_v == m.max_vehicle_speed)
        steering_angle = clamp(get_steering_angle(m.vehicle_wheelbase,max_delta_angle,pomdp_state.vehicle_v,m.one_time_step),0.0,max_steering_angle)
        rollout_delta_angle = get_straight_line_action(m, pomdp_state) #Just an angle value
        rollout_steering_angle = clamp(get_steering_angle(m.vehicle_wheelbase,rollout_delta_angle,pomdp_state.vehicle_v,m.one_time_step),-max_steering_angle,max_steering_angle)
        rollout_action = (rollout_steering_angle,0.0)
        if(rollout_action[1]!=0.0 && rollout_action[1] != -steering_angle && rollout_action[1]!=-steering_angle)
            return (
                    ActionExtendedSpacePOMDP(-steering_angle,0.0),
                    ActionExtendedSpacePOMDP(-2*steering_angle/3,0.0),
                    ActionExtendedSpacePOMDP(-steering_angle/3,0.0),
                    ActionExtendedSpacePOMDP(0.0,-delta_speed),
                    ActionExtendedSpacePOMDP(0.0,0.0),
                    ActionExtendedSpacePOMDP(steering_angle/3,0.0),
                    ActionExtendedSpacePOMDP(2*steering_angle/3,0.0),
                    ActionExtendedSpacePOMDP(steering_angle,0.0),
                    ActionExtendedSpacePOMDP(rollout_action[1],rollout_action[2]),
                    ActionExtendedSpacePOMDP(-10.0,-10.0)
                    )
        else
            return (
                    ActionExtendedSpacePOMDP(-steering_angle,0.0),
                    ActionExtendedSpacePOMDP(-2*steering_angle/3,0.0),
                    ActionExtendedSpacePOMDP(-steering_angle/3,0.0),
                    ActionExtendedSpacePOMDP(0.0,-delta_speed),
                    ActionExtendedSpacePOMDP(0.0,0.0),
                    ActionExtendedSpacePOMDP(steering_angle/3,0.0),
                    ActionExtendedSpacePOMDP(2*steering_angle/3,0.0),
                    ActionExtendedSpacePOMDP(steering_angle,0.0),
                    ActionExtendedSpacePOMDP(-10.0,-10.0)
                    )
        end
    else
        steering_angle = clamp(get_steering_angle(m.vehicle_wheelbase,max_delta_angle,pomdp_state.vehicle_v,m.one_time_step),0.0,max_steering_angle)
        rollout_delta_angle = get_straight_line_action(m, pomdp_state) #Just an angle value
        rollout_steering_angle = clamp(get_steering_angle(m.vehicle_wheelbase,rollout_delta_angle,pomdp_state.vehicle_v,m.one_time_step),-max_steering_angle,max_steering_angle)
        rollout_action = (rollout_steering_angle,0.0)
        if(rollout_action[1]!=0.0 && rollout_action[1] != -steering_angle && rollout_action[1]!=-steering_angle )
            return (ActionExtendedSpacePOMDP(-steering_angle,0.0),
                    ActionExtendedSpacePOMDP(-2*steering_angle/3,0.0),
                    ActionExtendedSpacePOMDP(-steering_angle/3,0.0),
                    ActionExtendedSpacePOMDP(0.0,-delta_speed),
                    ActionExtendedSpacePOMDP(0.0,0.0),
                    ActionExtendedSpacePOMDP(0.0,delta_speed),
                    ActionExtendedSpacePOMDP(steering_angle/3,0.0),
                    ActionExtendedSpacePOMDP(2*steering_angle/3,0.0),
                    ActionExtendedSpacePOMDP(steering_angle,0.0),
                    ActionExtendedSpacePOMDP(rollout_action[1],rollout_action[2]),
                    ActionExtendedSpacePOMDP(-10.0,-10.0)
                    )
        else
            return (ActionExtendedSpacePOMDP(-steering_angle,0.0),
                    ActionExtendedSpacePOMDP(-2*steering_angle/3,0.0),
                    ActionExtendedSpacePOMDP(-steering_angle/3,0.0),
                    ActionExtendedSpacePOMDP(0.0,-delta_speed),
                    ActionExtendedSpacePOMDP(0.0,0.0),
                    ActionExtendedSpacePOMDP(0.0,delta_speed),
                    ActionExtendedSpacePOMDP(steering_angle/3,0.0),
                    ActionExtendedSpacePOMDP(2*steering_angle/3,0.0),
                    ActionExtendedSpacePOMDP(steering_angle,0.0),
                    ActionExtendedSpacePOMDP(-10.0,-10.0)
                    )
        end
    end
end

function get_straight_line_action(m, pomdp_state)
    desired_orientation = get_heading_angle( m.vehicle_goal.x, m.vehicle_goal.y, pomdp_state.vehicle_x, pomdp_state.vehicle_y)
    # println(required_orientation*180/pi)
    delta_angle = get_delta_angle_straight_line_policy(desired_orientation,pomdp_state.vehicle_theta)
    return delta_angle
end

#get_actions function without considering other rollout policies
function get_actions_only_HJB_rollouts(m::ExtendedSpacePOMDP,b)
    if(m.sudden_break_flag)
        return get_actions_with_SB_HJB_rollout(m,b)
    else
        return get_actions_without_SB_HJB_rollout(m,b)
    end
end

#=
Please note that instead of doing it this way, I can also do multiple dispatch
on type of m where it is gonna be different based on the different rollout_guide
type and that should fix this problem too.
But stick to this for now!
=#
function get_actions(m::ExtendedSpacePOMDP,b)

    #This condition is used for random rollouts
    if(typeof(m.rollout_guide) == Tuple{Bool})
        if(m.sudden_break_flag)
            return get_actions_with_SB_random_rollout(m,b)
        else
            return get_actions_without_SB_random_rollout(m,b)
        end
    #This condition is used for straight line rollouts
    elseif(typeof(m.rollout_guide) == Tuple{Bool,Bool})
        if(m.sudden_break_flag)
            return get_actions_with_SB_straight_line_rollout(m,b)
        else
            return get_actions_without_SB_straight_line_rollout(m,b)
        end
    #This condition is used for HJB rollouts
    else
        if(m.sudden_break_flag)
            return get_actions_with_SB_HJB_rollout(m,b)
        else
            return get_actions_without_SB_HJB_rollout(m,b)
        end
    end
end


function get_default_action(m::ExtendedSpacePOMDP,b,ex)
    # println(ex)
    if(m.sudden_break_flag)
        return ActionExtendedSpacePOMDP(-10.0,-10.0)
    else
        return ActionExtendedSpacePOMDP(0.0,-0.5)
    end
end

discount(m::ExtendedSpacePOMDP) = m.discount_factor
isterminal(m::ExtendedSpacePOMDP, s::StateExtendedSpacePOMDP) = is_terminal_state(s,Location(-100.0,-100.0));
actions(m::ExtendedSpacePOMDP,b) = get_actions(m,b)


function get_delta_angle_straight_line_policy(desired_orientation,current_orientation)
    delta_angle = desired_orientation - current_orientation
    if(delta_angle>=π)
        delta_angle -= 2*π
    elseif(delta_angle<=-π)
        delta_angle += 2*π
    end
    return delta_angle
end

#Implement a straight line reactive controller policy for lower bound
function calculate_lower_bound_straight_line(m::ExtendedSpacePOMDP,b)

    delta_speed = m.vehicle_action_delta_speed
    vehicle_center_x = 0.0
    vehicle_center_y = 0.0
    delta_angle = 0.0

    debug=true
    debug=false

    #This bool is also used to check if all the states in the belief are terminal or not.
    first_execution_flag = true

    for (s, w) in weighted_particles(b)
        # println("State: $s")
        if(s.vehicle_x == -100.0 && s.vehicle_y == -100.0)
            continue
        else
            if(first_execution_flag)
                first_execution_flag = false
                vehicle_center_x = s.vehicle_x + m.vehicle_D*cos(s.vehicle_theta)
                vehicle_center_y = s.vehicle_y + m.vehicle_D*sin(s.vehicle_theta)
                desired_orientation = get_heading_angle( m.vehicle_goal.x, m.vehicle_goal.y,
                                            s.vehicle_x, s.vehicle_y)
                delta_angle = get_delta_angle_straight_line_policy(desired_orientation,s.vehicle_theta)
            end
            dist_to_closest_human = Inf  #Some really big infeasible number
            for human in s.nearby_humans
                # println("HUMAN : $human")
                predicted_human_state = update_human_position(human,m.world.length,m.world.breadth,m.one_time_step,0.0)
                euclidean_distance = sqrt((vehicle_center_x - predicted_human_state.x)^2 + (vehicle_center_y - predicted_human_state.y)^2)
                # println("!!!!!!!!!!!!!!euclidean_distance: $euclidean_distance !!!!!!!!!!!!!!")
                # println("!!!!!!Vehicle V : $(s.vehicle_v) !!!!!!!")
                if(euclidean_distance < 2.0 && s.vehicle_v != 0.0 && m.sudden_break_flag)
                    return ActionExtendedSpacePOMDP(-10.0,-10.0)
                end
                if(euclidean_distance < dist_to_closest_human)
                    dist_to_closest_human = euclidean_distance
                end
                # println("Distance to closest human: $dist_to_closest_human")
                if(dist_to_closest_human < m.d_near)
                    new_vehicle_speed = s.vehicle_v - m.vehicle_action_delta_speed
                    ϕ = get_steering_angle(m.vehicle_wheelbase, delta_angle, new_vehicle_speed, m.one_time_step)
                    ϕ = clamp(ϕ,-m.max_vehicle_steering_angle,m.max_vehicle_steering_angle)
                    if(debug)
                        println("Human too close. Steering angle: $ϕ, Delta Speed: $delta_speed")
                    end
                    return ActionExtendedSpacePOMDP(ϕ,-m.vehicle_action_delta_speed)
                end
            end
            if(dist_to_closest_human > m.d_far)
                chosen_delta_speed = m.vehicle_action_delta_speed
            else
                if(s.vehicle_v == 0.0)
                    chosen_delta_speed = delta_speed
                else
                    chosen_delta_speed = 0.0
                end
                # chosen_delta_speed = 0.0
            end
            if(chosen_delta_speed < delta_speed)
                delta_speed = chosen_delta_speed
            end
        end
    end

    #This condition is true only when all the states in the belief are terminal. In that case, just return (0.0,0.0)
    if(first_execution_flag)
        if(debug)
            println("All states are terminal")
        end
        return ActionExtendedSpacePOMDP(0.0,0.0)
    end

    s = first(particles(b))
    new_vehicle_speed = s.vehicle_v + delta_speed
    ϕ = get_steering_angle(m.vehicle_wheelbase, delta_angle, new_vehicle_speed, m.one_time_step)
    ϕ = clamp(ϕ,-m.max_vehicle_steering_angle,m.max_vehicle_steering_angle)
    if(debug)
        println("Normal Case. Steering angle: $ϕ, Delta Speed: $delta_speed")
    end
    return ActionExtendedSpacePOMDP(ϕ,delta_speed)

    # #This means all humans are away and you can accelerate.
    # if(delta_speed == m.vehicle_action_delta_speed)
    #     #@show(0.0,speed_change_to_be_returned)
    #     return ActionExtendedSpacePOMDP(delta_angle,delta_speed)
    # end

    # #If code has reached this point, then the best action is to maintain your current speed.
    # #We have already found the best steering angle to take.
    # #@show(best_delta_angle,0.0)
    # return ActionExtendedSpacePOMDP(delta_angle,0.0)
end

