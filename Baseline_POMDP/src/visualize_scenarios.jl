#=
Function to plot a scenario
=#

function visualize_given_scenario(b)
    s = rand(b)
    root_scenarios = [i=>s for i in 1:1];
    belief = ScenarioBelief(root_scenarios,pomdp_planner.rs, 1, missing);

    current_vehicle = Vehicle(s.vehicle_x,vehicle_y,vehicle_theta,vehicle_v)

    sim_time_step = 0.1
    one_time_step = 0.5
    num_sim_steps = Int64(one_time_step/sim_time_step)
    num_time_steps = 1

    vehicle_path = Vehicle[]
    humans_path = Array{Array{HumanState,1},1}(humans)
    sensor_data = []
    nearby_humans = []

    while(!is_within_range(current_vehicle.x, current_vehicle.y, vehicle_params.goal.x, vehicle_params.goal.y, exp_details.radius_around_vehicle_goal))
        #=
        Find vehicle action using lower bound rollout policy.
        Execute that action for one time step on the vehicle.
        Move humans for one time step.
        Generate plot for that situation.
        =#
        vehicle_action = lower_bound_func(m,belief)
        steering_angle = vehicle_action.steering_angle
        delta_speed = vehicle_action.delta_speed
        new_vehicle_speed = clamp(current_vehicle.v+delta_speed, 0.0, m.max_vehicle_speed)
        for i in 1:num_sim_steps
            new_vehicle = propogate_vehicle(current_vehicle, vehicle_params, steering_angle, new_vehicle_speed, sim_time_step)
            push!(vehicle_path,new_vehicle)
            new_humans = HumanState[]
            for j in 1:length(humans)
                nh = propogate_human(humans[j],humans_params[j])
                humans_params[j].path_index += 1
                push!(new_humans, nh)
            end
            push!(humans_path, new_humans)
        end
        current_vehicle = new_vehicle
        humans = new_humans
    end
    get_plot(env,vehicle,vehicle_params,humans,humans_params,sensor_data,nearby_humans,time_value,exp_details)
end

# function gen_rand!(r::MemorizingRNG{MersenneTwister}, n::Integer)
#     len = length(r.memory)
#     if len < r.idx + n
#         resize!(r.memory, len+MT_CACHE_F)
#         Random.gen_rand(r.source) # could be faster to use dsfmt_fill_array_close1_open2
#         r.memory[len+1:end] = r.source.vals
#         Random.mt_setempty!(r.source)
#     end
#     r.finish = length(r.memory)
#     return r
# end
# mr_avail(r::MemorizingRNG) = r.finish - r.idx
# reserve(r::MemorizingRNG, n::Integer) = mr_avail(r) < n && gen_rand!(r, n)
# function get_rng_copy(s::MemorizingSource, scenario::Int, depth::Int)
#     rng = s.rngs[depth+1, scenario]
#     if rng.finish == 0
#         rng.start = s.furthest+1
#         rng.idx = rng.start - 1
#         rng.finish = s.furthest
#         reserve(rng, s.min_reserve)
#     end
#     rng.idx = rng.start - 1
#     return rng
# end

rollout_policy_function(x,y) = calculate_lower_bound(x, y) 
# rollout_policy_function(x,y) = calculate_lower_bound_straight_line(extended_space_pomdp, x) 

function generate_rollout(scenarios, planner, depth)

    copy_scenarios = deepcopy(scenarios)
    b = ScenarioBelief(copy_scenarios,planner.rs, depth, missing);
    pomdp = planner.pomdp
    index, s = first(copy_scenarios)
    current_vehicle = Vehicle(s.vehicle_x,s.vehicle_y,s.vehicle_theta,s.vehicle_v)
    scenario_paths = [copy_scenarios]
    rollout_actions = []
    S = typeof(s)
    iter = depth

    while(!is_within_range(current_vehicle.x, current_vehicle.y, veh_params.goal.x, veh_params.goal.y, exp_details.radius_around_vehicle_goal))
        #=
        Find vehicle action using lower bound rollout policy.
        Execute that action for one time step on the vehicle.
        Move humans for one time step.
        Generate plot for that situation.
        =#
        # a = action(lower_bound_func.policy,b)
        a = rollout_policy_function(pomdp,b)
        push!(rollout_actions,a)
        new_scenarios =  Vector{ Pair{Int, S} }()
        for (k, s) in b.scenarios
            if !isterminal(pomdp, s)
                rng = ARDESPOT.get_rng(b.random_source, k, b.depth)
                sp, o, r = @gen(:sp, :o, :r)(pomdp, s, a, rng)
                push!(new_scenarios, k=>sp)
            end
        end
        if( isempty(new_scenarios) || iter>=planner.sol.D)
            break
        end
        iter +=1
        k,sp = new_scenarios[1]
        current_vehicle = Vehicle(sp.vehicle_x,sp.vehicle_y,sp.vehicle_theta,sp.vehicle_v)
        push!(scenario_paths, new_scenarios)
        b = ScenarioBelief(new_scenarios, b.random_source, b.depth+1, missing)
        # break
    end

    return scenario_paths, rollout_actions
end

function extract_one_scenario_path(scenario_paths, scenario_num)
    path = []
    for paths in scenario_paths
        pos = findfirst( x-> x[1] == scenario_num, paths)
        if(isnothing(pos))
            break
        else
            push!(path, paths[pos][2])
        end
    end
    return path
end

function visualize_scenario_path(env,vehicle_params,output,path)
    VP = vehicle_params
    E = env
    VB = output.vehicle_body
    for i in 1:length(path)
        point = path[i]
        V = Vehicle(point.vehicle_x,point.vehicle_y,point.vehicle_theta,point.vehicle_v)
        if(V.x == -100.0)
            break
        end
        H = point.nearby_humans
        u = rollout_plot(E, V, VP, VB, H)
        # sleep(1)
    end
end

function visualize_individual_scenario(scenario,env,vehicle_params,output)
    VP = vehicle_params
    E = env
    VB = output.vehicle_body
    point = scenario
    V = Vehicle(point.vehicle_x,point.vehicle_y,point.vehicle_theta,point.vehicle_v)
    if(V.x == -100.0)
        println("Terminal State in Scenario : ", scenario)
    end
    H = point.nearby_humans
    u = rollout_plot(E, V, VP, VB, H)
end

function rollout_plot(env, vehicle, vehicle_params, vehicle_body, humans)

    l = env.length
    b = env.breadth
    snapshot = plot(aspect_ratio=:equal,size=(800,800), dpi=300,
        # axis=([], false)
        xticks=0:2:l, yticks=0:2:b,
        # xlabel="x-axis [m]", ylabel="y-axis [m]",
        # legend=:bottom,
        # legend=false
        )

    #Plot the rectangular environment
    turf_color = :green
    plot!(snapshot, rectangleShape(0.0,0.0,env.length,env.breadth),opacity=0.1,color=turf_color,linewidth=2.0,linecolor=:black,label="")

    #Plot External Anchors
    anchor_dist = 0.5
    scatter!(snapshot,
        [0.0 - anchor_dist, env.length + anchor_dist, env.length + anchor_dist, 0.0 - anchor_dist],
        [0.0 - anchor_dist, 0.0 - anchor_dist, env.breadth + anchor_dist, env.breadth + anchor_dist],
        markeralpha=0.0, fillalpha=0.0, label="")

    #Plot Obstacles
    obstacle_color = :black
    for obs in env.obstacles
        plot!(snapshot, circleShape(obs.x,obs.y,obs.r), color=obstacle_color, linewidth=2.0, linecolor = obstacle_color,
            legend=false, fillalpha=0.4, aspect_ratio=1, label="", seriestype = [:shape,])
        Plots.annotate!(snapshot,obs.x, obs.y, text("Obs", obstacle_color, :center, 10))
        # scatter!([obs.x],[obs.y],color="black",shape=:circle,msize=plot_size*obs.r/env.length)
    end

    #Plot Human Goals
    human_goals = get_human_goals(env)
    offset = 0.125 * 0
    human_goal_color = :green
    for i in 1:length(human_goals)
        goal_name = "G"*string(i)
        human_goals[i].x > 1/2*env.length ? x_dir = 1 : x_dir = -1
        human_goals[i].y > 1/2*env.breadth ? y_dir = 1 : y_dir = -1
        Plots.annotate!(human_goals[i].x + x_dir*offset, human_goals[i].y + y_dir*offset, text(goal_name, :black, :center, 10))
        plot!(snapshot, circleShape(human_goals[i].x + x_dir*offset,human_goals[i].y + y_dir*offset,exp_details.max_risk_distance),
                    color=human_goal_color, linewidth=2.0, linecolor = :black, fillalpha=0.2, aspect_ratio=1, label="", seriestype = [:shape,])
    end

    #=
    Plot humans!
    Go through all the humans in the current sim object
    If that human is a nearby human, display it in red
    If that human is in sensor data, display it in blue
    If neither of that is true, it is outside the lidar range, ignore it!
    =#
    for i in 1:length(humans)
        human = humans[i]
        human_reached_goal = (human.x == human.goal.x) && (human.y == human.goal.y)
        if(human_reached_goal)
            continue
        end
        scatter!([human.x],[human.y],color="red")
        # Plots.annotate!(human.x, human.y, text(string(human_id), :purple, :right, 15))
        plot!(snapshot,circleShape(human.x,human.y,exp_details.max_risk_distance), lw=0.5, linecolor = :black,
                                                legend=false, fillalpha=0.2, aspect_ratio=1,c= :red, seriestype = [:shape,])
    end

    #Plot Vehicle Goal
    vehicle_goal_color = :yellow
    Plots.annotate!(snapshot,vehicle_params.goal.x, vehicle_params.goal.y, text("G", :darkgreen, :center, 10))
    plot!(snapshot, circleShape(vehicle_params.goal.x, vehicle_params.goal.y, exp_details.radius_around_vehicle_goal),
        color=vehicle_goal_color, fillalpha=0.2, linecolor=:black, linewidth=2.0, label="", seriestype = [:shape,])

    #Plot Vehicle
    scatter!(snapshot, [vehicle.x], [vehicle.y],markershape=:circle, markersize=3, markerstrokewidth=0, markercolor=:black,label="")
    vehicle_shape = get_vehicle_shape(vehicle,vehicle_body)
    plot!(snapshot, vehicle_shape,color=:black, alpha=0.125, linecolor=:black, linewidth=2.0, linealpha=1.0,label="Vehicle")
    vehicle_center_x = vehicle.x + vehicle_params.dist_origin_to_center*cos(vehicle.theta)
    vehicle_center_y = vehicle.y + vehicle_params.dist_origin_to_center*sin(vehicle.theta)
    plot!(circleShape(vehicle_center_x,vehicle_center_y,vehicle_params.radius), linewidth=0.5, linecolor = :black,
                            legend=false, fillalpha=0.2, aspect_ratio=1,c= :grey, seriestype = [:shape,])

    display(snapshot)
    return snapshot
end



#=
tree = info[:tree]
scenario_num = 38
scens = tree.scenarios[scenario_num];
DEP = tree.Delta[scenario_num];
scenario_paths, rollout_actions = generate_rollout( scens, pomdp_planner, DEP);

N_array = [i[1] for i in scenario_paths[1]]

N = 3
P = extract_one_scenario_path(scenario_paths,N);
visualize_scenario_path(env,veh_params,output,P)
length(P)

=#


function helper(B_node, scenario_num, path, all_paths)

    index = findfirst(i-> i[1]==scenario_num, B_node.scenarios)
    if(isnothing(index))
        return
    else
        desired_scenario = B_node.scenarios[index]
        push!(path, (B_node.Delta => desired_scenario) )
    end

    #If B_node is a leaf node, time to add this path to all the paths
    if( isempty(B_node.children) )
        push!(all_paths, deepcopy(path))
        pop!(path)
        return
    end

    for action_node in B_node.children
        for new_B_node in action_node.children
            helper(new_B_node,scenario_num,path,all_paths)
        end
    end
    pop!(path)
    return
end

function all_paths_recursive_tree(root,scenario_num)
    all_paths = []
    path = []
    helper(root,scenario_num,path,all_paths)
    return all_paths
end

function visualize_all_paths(env,vehicle_params,output,all_paths)

    l = env.length
    b = env.breadth
    snapshot = plot(aspect_ratio=:equal,size=(1800,1800), dpi=300,
        # axis=([], false)
        xticks=0:2:l, yticks=0:2:b,
        # xlabel="x-axis [m]", ylabel="y-axis [m]",
        # legend=:bottom,
        # legend=false
        )

    #Plot the rectangular environment
    turf_color = :green
    plot!(snapshot, rectangleShape(0.0,0.0,env.length,env.breadth),opacity=0.1,color=turf_color,linewidth=2.0,linecolor=:black,label="")

    #Plot External Anchors
    anchor_dist = 0.5
    scatter!(snapshot,
        [0.0 - anchor_dist, env.length + anchor_dist, env.length + anchor_dist, 0.0 - anchor_dist],
        [0.0 - anchor_dist, 0.0 - anchor_dist, env.breadth + anchor_dist, env.breadth + anchor_dist],
        markeralpha=0.0, fillalpha=0.0, label="")

    # #Plot Obstacles
    # obstacle_color = :black
    # for obs in env.obstacles
    #     plot!(snapshot, circleShape(obs.x,obs.y,obs.r), color=obstacle_color, linewidth=2.0, linecolor = obstacle_color,
    #         legend=false, fillalpha=0.4, aspect_ratio=1, label="", seriestype = [:shape,])
    #     Plots.annotate!(snapshot,obs.x, obs.y, text("Obs", obstacle_color, :center, 10))
    #     # scatter!([obs.x],[obs.y],color="black",shape=:circle,msize=plot_size*obs.r/env.length)
    # end

    #Plot Human Goals
    human_goals = get_human_goals(env)
    offset = 0.125 * 0
    human_goal_color = :green
    for i in 1:length(human_goals)
        goal_name = "G"*string(i)
        human_goals[i].x > 1/2*env.length ? x_dir = 1 : x_dir = -1
        human_goals[i].y > 1/2*env.breadth ? y_dir = 1 : y_dir = -1
        Plots.annotate!(human_goals[i].x + x_dir*offset, human_goals[i].y + y_dir*offset, text(goal_name, :black, :center, 10))
        plot!(snapshot, circleShape(human_goals[i].x + x_dir*offset,human_goals[i].y + y_dir*offset,exp_details.max_risk_distance),
                    color=human_goal_color, linewidth=2.0, linecolor = :black, fillalpha=0.2, aspect_ratio=1, label="", seriestype = [:shape,])
    end

    #Plot Vehicle Goal
    vehicle_goal_color = :yellow
    Plots.annotate!(snapshot,vehicle_params.goal.x, vehicle_params.goal.y, text("G", :darkgreen, :center, 10))
    plot!(snapshot, circleShape(vehicle_params.goal.x, vehicle_params.goal.y, exp_details.radius_around_vehicle_goal),
        color=vehicle_goal_color, fillalpha=0.2, linecolor=:black, linewidth=2.0, label="", seriestype = [:shape,])


    for p in all_paths
        visualize_path_recursive_tree(snapshot,vehicle_params,output,p)
    end

end

function visualize_path_recursive_tree(snapshot,vehicle_params,output,path)
    modified_path = []
    for p in path
        push!(modified_path,(p[1]=>p[2][2]))
    end

    N = length(modified_path)
    opacity_levels = collect(1:-1/N:0)

    VP = vehicle_params
    VB = output.vehicle_body
    for i in 1:length(modified_path)
        point = modified_path[i][2]
        V = Vehicle(point.vehicle_x,point.vehicle_y,point.vehicle_theta,point.vehicle_v)
        if(V.x == -100.0)
            break
        end
        H = point.nearby_humans
        opacity = opacity_levels[i]
        u = visualize_tree_path(snapshot, V, VP, VB, H, opacity)
        # sleep(1)
    end

    return modified_path
    # visualize_scenario_path(env,vehicle_params,output,modified_path)
end

function visualize_tree_path(snapshot, vehicle, vehicle_params, vehicle_body, humans, opacity)

    #=
    Plot humans!
    =#
    for i in 1:length(humans)
        human = humans[i]
        human_reached_goal = (human.x == human.goal.x) && (human.y == human.goal.y)
        if(human_reached_goal)
            continue
        end
        scatter!([human.x],[human.y],color="red")
        # Plots.annotate!(human.x, human.y, text(string(human_id), :purple, :right, 15))
        plot!(snapshot,circleShape(human.x,human.y,exp_details.max_risk_distance), lw=0.5, linecolor = :black,
                                                legend=false, fillalpha=opacity, aspect_ratio=1,c= :red, seriestype = [:shape,])
    end


    #Plot Vehicle
    scatter!(snapshot, [vehicle.x], [vehicle.y],markershape=:circle, markersize=3, markerstrokewidth=0, markercolor=:black,label="")
    vehicle_shape = get_vehicle_shape(vehicle,vehicle_body)
    plot!(snapshot, vehicle_shape,color=:black, alpha=0.125, linecolor=:black, linewidth=2.0, linealpha=1.0,label="Vehicle")
    vehicle_center_x = vehicle.x + vehicle_params.dist_origin_to_center*cos(vehicle.theta)
    vehicle_center_y = vehicle.y + vehicle_params.dist_origin_to_center*sin(vehicle.theta)
    plot!(circleShape(vehicle_center_x,vehicle_center_y,vehicle_params.radius), linewidth=0.5, linecolor = :black,
                            legend=false, fillalpha=opacity, aspect_ratio=1,c= :lightgreen, seriestype = [:shape,])

    display(snapshot)
    return snapshot
end
