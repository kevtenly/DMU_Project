# function get_intro_plot(env, vehicle, vehicle_body, vehicle_params, nearby_humans, sensor_data, time_value, exp_details, vehicle_traj)

function EllipseShape(h,k,a,b)
    theta = LinRange(0,2*pi,100)
    h .+ a*cos.(theta), k .+ b*sin.(theta)
end

function get_intro_plot(vehicle_body)

    l = 10.0
    b = 10.0
    obstacles = [ObstacleLocation(5,6,1)]
    goal = Location(8,9)
    radius_around_vehicle_goal = 0.3
    vehicle = Vehicle(4,1,pi/2,1.0)

    snapshot = plot(aspect_ratio=:equal,size=(1000,1000), dpi=300,
        # axis=([], false),
        xticks=0:2:l, yticks=0:2:b,
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
    plot!(snapshot, rectangleShape(0.0,0.0,l,b),opacity=0.1,color=turf_color,linewidth=2.0,linecolor=:black,label="")


    #Plot External Anchors
    anchor_dist = 0.5
    scatter!(snapshot,
        [0.0 - anchor_dist, l + anchor_dist, l + anchor_dist, 0.0 - anchor_dist],
        [0.0 - anchor_dist, 0.0 - anchor_dist, b + anchor_dist, b + anchor_dist],
        markeralpha=0.0, fillalpha=0.0, label="")

    #Plot Obstacles
    obstacle_color = :brown
    for (i,obs) in enumerate(obstacles)
        plot!(snapshot, circleShape(obs.x,obs.y,obs.r), color=obstacle_color, linewidth=2.0, linecolor = obstacle_color,
            legend=false, fillalpha=0.4, aspect_ratio=1, label="", seriestype = [:shape,])
        Plots.annotate!(snapshot,obs.x, obs.y, text("Obstacle", obstacle_color, :center, 10))
        # scatter!([obs.x],[obs.y],color="black",shape=:circle,msize=plot_size*obs.r/env.length)
    end


    #Plot Human Goals
    # human_goals = get_human_goals(env)
    # offset = 0.125 * 0
    # human_goal_color = :green
    # for i in 1:length(human_goals)
    #     goal_name = "G"*string(i)
    #     human_goals[i].x > 1/2*env.length ? x_dir = 1 : x_dir = -1
    #     human_goals[i].y > 1/2*env.breadth ? y_dir = 1 : y_dir = -1
    #     Plots.annotate!(human_goals[i].x + x_dir*offset, human_goals[i].y + y_dir*offset, text(goal_name, :black, :center, 10))
    #     plot!(snapshot, circleShape(human_goals[i].x + x_dir*offset,human_goals[i].y + y_dir*offset,exp_details.max_risk_distance),
    #                 color=human_goal_color, linewidth=2.0, linecolor = :black, fillalpha=0.2, aspect_ratio=1, label="", seriestype = [:shape,])
    # end

    #Plot Humans
    # nearby_human_color = :red
    # faraway_human_color = :cyan
    # for i in 1:length(sensor_data.lidar_data)
    #     human = sensor_data.lidar_data[i]
    #     human_id = sensor_data.ids[i]
    #     human_reached_goal = (human.x == human.goal.x) && (human.y == human.goal.y)
    #     if(human_reached_goal)
    #         continue
    #     end
    #     is_nearby_human = !(length(findall(x->x==human_id, nearby_humans.ids)) == 0)
    #     if(is_nearby_human)
    #         # scatter!(snapshot, [human.x], [human.y], color=nearby_human_color, label="")
    #         plot!(snapshot, circleShape(human.x, human.y, exp_details.max_risk_distance),color=nearby_human_color,
    #                     fillalpha=0.2, linecolor=:black, linewidth=2.0, label="", seriestype = [:shape,])
    #         # Plots.annotate!(human.x, human.y, text(string(human_id), :purple, :center, 15))
    #         # Plots.annotate!(human.x, human.y, text("H",nearby_human_color, :center, :black, 10))
    #         continue
    #     end

    faraway_human_color = :cyan
    hg = Location(10,10)
    h1 = HumanState(2,4,1,hg)
    h2 = HumanState(7,6,1,hg)
    humans = [h1,h2]
    for (i,human) in enumerate(humans)
        # scatter!(snapshot, [human.x], [human.y], color=faraway_human_color, label="")
        plot!(snapshot, circleShape(human.x, human.y, 0.3),color=faraway_human_color,
            fillalpha=0.5,linecolor=:black, linewidth=2.0, label="", seriestype = [:shape,])
        Plots.annotate!(human.x, human.y, text("H"*string(i), faraway_human_color , :center, :black, 10))
    end

    h1 = HumanState(2.7,3.7,1,hg)
    humans_tplus1 = [h1]
    for (i,human) in enumerate(humans_tplus1)
        # scatter!(snapshot, [human.x], [human.y], color=faraway_human_color, label="")
        plot!(snapshot, EllipseShape(human.x, human.y, 0.5, 0.3),color=faraway_human_color,
            fillalpha=0.2,linecolor=:black, linewidth=2.0, label="", seriestype = [:shape,])
        Plots.annotate!(human.x, human.y, text("H"*string(i), faraway_human_color , :center, :black, 10))
    end



    #Plot Vehicle Goal
    vehicle_goal_color = :yellow
    Plots.annotate!(snapshot,goal.x, goal.y, text("G", :darkgreen, :center, 10))
    plot!(snapshot, circleShape(goal.x, goal.y, radius_around_vehicle_goal),
        color=vehicle_goal_color, fillalpha=0.2, linecolor=:black, linewidth=2.0, label="", seriestype = [:shape,])

    #Plot Vehicle
    scatter!(snapshot, [vehicle.x], [vehicle.y],markershape=:circle, markersize=3, markerstrokewidth=0, markercolor=:black,label="")
    vehicle_shape = get_vehicle_shape(vehicle,vehicle_body)
    plot!(snapshot, vehicle_shape,color=:black, alpha=0.5, linecolor=:black, linewidth=2.0, linealpha=1.0,label="Vehicle")

    vehicle_tplus1 = Vehicle(4,3,pi/2,1.0)
    vehicle = vehicle_tplus1
    scatter!(snapshot, [vehicle.x], [vehicle.y],markershape=:circle, markersize=3, markerstrokewidth=0, markercolor=:black,label="")
    vehicle_shape = get_vehicle_shape(vehicle,vehicle_body)
    plot!(snapshot, vehicle_shape,color=:black, alpha=0.3, linecolor=:black, linewidth=2.0, linealpha=1.0,label="Vehicle")

    return snapshot
end
