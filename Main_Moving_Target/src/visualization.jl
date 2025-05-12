using Plots
using UnicodePlots
using D3Trees

#Function to display a circle
function circleShape(h,k,r)
    theta = LinRange(0,2*pi,100)
    h .+ r*sin.(theta), k .+ r*cos.(theta)
end

function rectangleShape(x,y,width,height)
    s = Shape(x .+ [0,width,width,0], y.+ [0,0,height,height])
    return s
end

function get_vehicle_shape(vehicle,vehicle_body_origin)
    #First rotate the vehicle to align with vehicle's current theta
    rotation_matrix = [cos(vehicle.theta) -sin(vehicle.theta); sin(vehicle.theta) cos(vehicle.theta) ]
    vehicle_shape = linear_map(rotation_matrix,vehicle_body_origin)

    #Translate vehicle body to its current x and y coordinates
    LazySets.translate!(vehicle_shape,[vehicle.x,vehicle.y])
    return vehicle_shape
end


function get_plot(env,vehicle,vehicle_params,humans,humans_params,sensor_data,nearby_humans,time_value,exp_details)

    plot_size = 1000; #number of pixels
    boundary_padding = 0.5
    # p = plot(legend=false,grid=false)
    p = plot(legend=false,grid=false,axis=([], false))

    #Plot the rectangular environment
    plot!([0.0, env.length],[0.0,0.0], color="grey", lw=2)
    plot!([env.length, env.length],[0.0,env.breadth], color="grey", lw=2)
    plot!([0.0, env.length],[env.breadth,env.breadth], color="grey", lw=2)
    plot!([0.0, 0.0],[0.0,env.breadth], color="grey", lw=2)
    scatter!([0.0-boundary_padding],[0.0-boundary_padding], color="white",markersize = 0)
    scatter!([env.length+boundary_padding],[0.0-boundary_padding], color="white",markersize = 0)
    scatter!([env.length+boundary_padding],[env.breadth+boundary_padding], color="white",markersize = 0)
    scatter!([0.0-boundary_padding],[env.breadth+boundary_padding], color="white",markersize = 0)

    #Plot Human Goals
    human_goals = get_human_goals(env)
    for i in 1:length(human_goals)
        goal_name = "G"*string(i)
        Plots.annotate!(human_goals[i].x, human_goals[i].y,text(goal_name, :purple, :center, 15))
    end

    #Plot Obstacles
    for obs in env.obstacles
        plot!(circleShape(obs.x,obs.y,obs.r), lw=0.5, linecolor = :black,
                                            legend=false, fillalpha=1.0, aspect_ratio=1,c= :black, seriestype = [:shape,])
        # scatter!([obs.x],[obs.y],color="black",shape=:circle,msize=plot_size*obs.r/env.length)
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
        human_id = humans_params[i].id
        human_reached_goal = (human.x == human.goal.x) && (human.y == human.goal.y)
        if(human_reached_goal)
            continue
        end
        is_nearby_human = !(length(findall(x->x==human_id, nearby_humans.ids)) == 0)
        if(is_nearby_human)
            scatter!([human.x],[human.y],color="red")
            Plots.annotate!(human.x, human.y, text(string(human_id), :purple, :right, 15))
            plot!(circleShape(human.x,human.y,exp_details.max_risk_distance), lw=0.5, linecolor = :black,
                                                legend=false, fillalpha=0.2, aspect_ratio=1,c= :red, seriestype = [:shape,])
            # human_heading_angle = get_heading_angle(human.goal.x,human.goal.y,human.x,human.y)
            # quiver!([human.x],[human.y],quiver=([cos(human_heading_angle)],[sin(human_heading_angle)]), color="red")
            continue
        end
        is_sensor_data_human = !(length(findall(x->x==human_id, sensor_data.ids)) == 0)
        if(is_sensor_data_human)
            scatter!([human.x],[human.y],color="lightblue")
            # annotate!(human.x, human.y, text(string(human_id), :purple, :right, 15))
            plot!(circleShape(human.x,human.y,exp_details.max_risk_distance), lw=0.5, linecolor = :black,
                                                legend=false, fillalpha=0.2, aspect_ratio=1,c= :lightblue, seriestype = [:shape,])
            # human_heading_angle = get_heading_angle(human.goal.x,human.goal.y,human.x,human.y)
            # quiver!([human.x],[human.y],quiver=([cos(human_heading_angle)],[sin(human_heading_angle)]), color="green")
            continue
        end
    end

    #Plot Vehicle
    # scatter!([vehicle.x], [vehicle.y], shape=:circle, color="grey")
    vehicle_center_x = vehicle.x + vehicle_params.dist_origin_to_center*cos(vehicle.theta)
    vehicle_center_y = vehicle.y + vehicle_params.dist_origin_to_center*sin(vehicle.theta)
    plot!(circleShape(vehicle_center_x,vehicle_center_y,vehicle_params.radius), lw=0.5, linecolor = :black,
                            legend=false, fillalpha=0.2, aspect_ratio=1,c= :grey, seriestype = [:shape,])
    quiver!([vehicle_center_x],[vehicle_center_y],quiver=([cos(vehicle.theta)],[sin(vehicle.theta)]), color="grey")

    #Plot Vehicle Goal
    Plots.annotate!(vehicle_params.goal.x, vehicle_params.goal.y, text("G", :darkgreen, :right, 10))
    plot!(circleShape(vehicle_params.goal.x, vehicle_params.goal.y, exp_details.radius_around_vehicle_goal), lw=0.5, linecolor = :black,
                                        legend=false, fillalpha=0.2, aspect_ratio=1,c= :darkgreen, seriestype = [:shape,])

    #Add the time value to the plot
    Plots.annotate!(env.length/2, env.breadth+0.5, text(string(round(time_value,digits=1)), :purple, :right, 20))
    plot!(size=(plot_size,plot_size))
    # display(p)
    return p
end
#=
t = 11
p = get_plot( output.sim_objects[t].env, output.sim_objects[t].vehicle, output.sim_objects[t].vehicle_params, output.nearby_humans[t], output.sim_objects[t].vehicle_sensor_data, t, exp_details)
display(p)
=#


function get_plot(env, vehicle, vehicle_body, vehicle_params, nearby_humans, sensor_data, time_value, exp_details, vehicle_traj)

    l = env.length
    b = env.breadth
    point_width = 5
    snapshot = plot(aspect_ratio=:equal,size=(1000,1000), dpi=300,
        # axis=([], false)
        xticks=0:point_width:l, yticks=0:point_width:b,
        # xlabel="x-axis [m]", ylabel="y-axis [m]",
        # legend=:bottom,
        # legend=false
        )

    #Plot Workspace
    turf_color = :green
    # plot!(snapshot, [0.0, env.length], [0.0,0.0], linecolor=boundary_color, linewidth=0.5, label="")
    # plot!(snapshot, [env.length, env.length], [0.0,env.breadth], linecolor=boundary_color, linewidth=0.5, label="")
    # plot!(snapshot, [0.0, env.length], [env.breadth,env.breadth], linecolor=boundary_color, linewidth=0.5, label="")
    # plot!(snapshot, [0.0, 0.0], [0.0,env.breadth], linecolor=boundary_color, linewidth=0.5, label="")
    plot!(snapshot, rectangleShape(0.0,0.0,env.length,env.breadth),opacity=0.1,color=turf_color,linewidth=2.0,linecolor=:black,label="")


    #Plot External Anchors
    anchor_dist = 0.5
    scatter!(snapshot,
        [0.0 - anchor_dist, env.length + anchor_dist, env.length + anchor_dist, 0.0 - anchor_dist],
        [0.0 - anchor_dist, 0.0 - anchor_dist, env.breadth + anchor_dist, env.breadth + anchor_dist],
        markeralpha=0.0, fillalpha=0.0, label="")

    #Plot Obstacles
    obstacle_color = :black
    for (i,obs) in enumerate(env.obstacles)
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

    #Plot Humans
    nearby_human_color = :red
    faraway_human_color = :cyan
    for i in 1:length(sensor_data.lidar_data)
        human = sensor_data.lidar_data[i]
        human_id = sensor_data.ids[i]
        human_reached_goal = (human.x == human.goal.x) && (human.y == human.goal.y)
        if(human_reached_goal)
            continue
        end
        is_nearby_human = !(length(findall(x->x==human_id, nearby_humans.ids)) == 0)
        if(is_nearby_human)
            # scatter!(snapshot, [human.x], [human.y], color=nearby_human_color, label="")
            plot!(snapshot, circleShape(human.x, human.y, exp_details.max_risk_distance),color=nearby_human_color,
                        fillalpha=0.2, linecolor=:black, linewidth=2.0, label="", seriestype = [:shape,])
            # Plots.annotate!(human.x, human.y, text(string(human_id), :purple, :center, 15))
            # Plots.annotate!(human.x, human.y, text("H",nearby_human_color, :center, :black, 10))
            continue
        end
        is_sensor_data_human = !(length(findall(x->x==human_id, sensor_data.ids)) == 0)
        if(is_sensor_data_human)
            # scatter!(snapshot, [human.x], [human.y], color=faraway_human_color, label="")
            plot!(snapshot, circleShape(human.x, human.y, exp_details.max_risk_distance),color=faraway_human_color,
                fillalpha=0.2,linecolor=:black, linewidth=2.0, label="", seriestype = [:shape,])
            # Plots.annotate!(human.x, human.y, text(string(human_id), :purple, :center, 15))
            # Plots.annotate!(human.x, human.y, text("H", faraway_human_color , :center, :black, 10))
            continue
        end
    end
    
    p = floor(time_value)
    p_upper = p+0.5
    if(time_value<p_upper)
        pos_t = p
    else
        pos_t = p_upper
    end

    pos = exp_details.target_history[pos_t]
    scatter!(snapshot, [pos.x], [pos.y]; 
                marker = :rect, ms=8, mc = :yellow, label="Target")

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

    #Plot vehicle's executed trajectory
    velocity_lim = vehicle_params.max_speed + 0.5
    vehicle_velocity_list = zeros(length(vehicle_traj))
    for i in 1:(length(vehicle_traj)-1)
        vehicle_velocity_list[i] = vehicle_traj[i+1].v
    end
    vehicle_traj_x_path = getfield.(vehicle_traj, :x)
    vehicle_traj_y_path = getfield.(vehicle_traj, :y)
    plot!(snapshot, vehicle_traj_x_path, vehicle_traj_y_path,
        linez=vehicle_velocity_list, clim=(0,velocity_lim), colorbar_title="V [m/s]",
        linewidth=1,label="")
    # plot!(snapshot, vehicle_traj_x_path, vehicle_traj_y_path,
    #     linez=vehicle_velocity_list, clim=(0,velocity_lim), colorbar_title="Velocity [m/s]",
    #     linewidth=2,label="")

    # annotate time value
    t_round = round(time_value, digits=1)
    Plots.annotate!(snapshot, 0.5*env.length, env.breadth+0.5, text("t = $t_round sec", :black, :center, 14))

    return snapshot
end


#Function to display the environment, vehicle and humans
function observe(output,exp_details,time_value,vehicle_executed_trajectory)
    e = output.sim_objects[time_value].env
    v = output.sim_objects[time_value].vehicle
    vp = output.sim_objects[time_value].vehicle_params
    vb = output.vehicle_body
    h = output.sim_objects[time_value].humans
    hp = output.sim_objects[time_value].humans_params
    sd = output.sim_objects[time_value].vehicle_sensor_data
    nbh = output.nearby_humans[time_value]
    push!(vehicle_executed_trajectory, v)

    p = get_plot(e, v, vb, vp, nbh, sd, time_value, exp_details, vehicle_executed_trajectory)
    # p = get_plot(e,v,vp,h,hp,sd,nbh,time_value,exp_details)

    if(hasfield(typeof(vp),:controls_sequence))
        expected_path = output.vehicle_expected_trajectory[time_value]
        plot!(expected_path[1], expected_path[2], line=(3,:dot,:blue))
    end
    # annotate!(env.length/2, env.breadth/2, text("HG", :purple, :right, 20))
    display(p)
end

function generate_gif(output, exp_details)
    vehicle_executed_trajectory = []
    anim = @animate for k ∈ keys(output.sim_objects)
        observe(output, exp_details, k, vehicle_executed_trajectory);
    end
    gif(anim, "es_planner.gif", fps = 10)
end
