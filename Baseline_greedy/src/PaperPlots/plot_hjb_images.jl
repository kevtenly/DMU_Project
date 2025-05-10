function BellmanPDEs.propagate_state(x_k, a_k, Dt, veh)
   # define number of substeps used for integration
   x_k1 = discrete_time_EoM(x_k, a_k, Dt, veh)
   return old_propagate_state(x_k, a_k, Dt, veh)
end



function hg_plot_HJB_path(x_path_list, x_subpath_list, env, veh, linez_clim, label_list)
    p_planner = plot(aspect_ratio=:equal,
        size=(800,800), dpi=300,
        xticks=0:4:20, yticks=0:4:20,
        xlabel="x-axis [m]", ylabel="y-axis [m]",
        legend=:bottomright,
        top_margin = -36*Plots.mm,
        bottom_margin = -8*Plots.mm,
        left_margin = 8*Plots.mm,
        right_margin = 6*Plots.mm)

    # workspace
    plot!(p_planner, env.workspace,
        alpha=0.0,
        linecolor=:black, linewidth=2, linealpha=1.0,
        label="")

    # goal
    plot!(p_planner, env.goal,
        color=:green, alpha=0.125,
        linecolor=:green, linewidth=2, linealpha=1.0,
        label="Goal")

    # obstacles
    if isempty(env.obstacle_list) == false
        plot!(p_planner, env.obstacle_list[1],
            color=:red, alpha=0.125,
            linecolor=:red, linewidth=2, linealpha=1.0,
            label="Obstacle")

        for obstacle in env.obstacle_list[2:end]
            plot!(p_planner, obstacle,
                color=:red, alpha=0.125,
                linecolor=:red, linewidth=2, linealpha=1.0,
                label="")
        end
    end

    for ip in axes(x_path_list, 1)
        x_path = x_path_list[ip]
        x_subpath = x_subpath_list[ip]
        # shift velocity up one step to make line_z look right
        linez_velocity = zeros(length(x_subpath))
        for kk in 1:(length(x_subpath)-1)
            linez_velocity[kk] = x_subpath[kk+1][4]
        end
        # subpath lines
        println(ip)
        plot!(p_planner, getindex.(x_subpath,1), getindex.(x_subpath,2),
            linez=linez_velocity, clim=(0, linez_clim), colorbar_title="Velocity [m/s]",
            linewidth = 2,
            label="")

        # path points
        plot!(p_planner, getindex.(x_path,1), getindex.(x_path,2),
            linewidth = 0, linealpha=0.0,
            markershape=:circle, markersize=3.0, markerstrokewidth=0,
            label=label_list[ip])

        # start position
        plot!(p_planner, [x_path[1][1]], [x_path[1][2]],
            markershape=:circle, markersize=3, markerstrokewidth=0,
            label="")

        veh_body = state_to_body(x_path[1], veh)
        plot!(p_planner, veh_body, alpha=0.0,
            linecolor=:black, linewidth=2, linealpha=1.0, label="")

        # end position
        plot!(p_planner, [x_path[end][1]], [x_path[end][2]],
            markershape=:circle, markersize=3, markerstrokewidth=0,
            label="")

        veh_body = state_to_body(x_path[end], veh)
        plot!(p_planner, veh_body, alpha=0.0,
            linecolor=:black, linewidth=2, linealpha=1.0, label="")
    end

    display("image/png", p_planner)
end


x0 = SVector(10,3,pi/2,0.0)
q1,w1,e,r = plan_path(x0,HJB_policy,750,extended_space_pomdp.rollout_guide.get_actions,
                          extended_space_pomdp.rollout_guide.get_cost,extended_space_pomdp.one_time_step,extended_space_pomdp.rollout_guide.q_value_array,
                          extended_space_pomdp.rollout_guide.value_array,extended_space_pomdp.rollout_guide.env,extended_space_pomdp.rollout_guide.veh,extended_space_pomdp.rollout_guide.state_grid,100)

x0 = SVector(3,1,2*pi/3,0.0)
q2,w2,e,r = plan_path(x0,HJB_policy,750,extended_space_pomdp.rollout_guide.get_actions,
                          extended_space_pomdp.rollout_guide.get_cost,extended_space_pomdp.one_time_step,extended_space_pomdp.rollout_guide.q_value_array,
                          extended_space_pomdp.rollout_guide.value_array,extended_space_pomdp.rollout_guide.env,extended_space_pomdp.rollout_guide.veh,extended_space_pomdp.rollout_guide.state_grid,100)

x0 = SVector(14,9,0.0,0.0)
q3,w3,e,r = plan_path(x0,HJB_policy,750,extended_space_pomdp.rollout_guide.get_actions,
                          extended_space_pomdp.rollout_guide.get_cost,extended_space_pomdp.one_time_step,extended_space_pomdp.rollout_guide.q_value_array,
                          extended_space_pomdp.rollout_guide.value_array,extended_space_pomdp.rollout_guide.env,extended_space_pomdp.rollout_guide.veh,extended_space_pomdp.rollout_guide.state_grid,100)

x0 = SVector(16,4,-pi/4,0.0)
q4,w4,e,r = plan_path(x0,HJB_policy,750,extended_space_pomdp.rollout_guide.get_actions,
                        extended_space_pomdp.rollout_guide.get_cost,extended_space_pomdp.one_time_step,extended_space_pomdp.rollout_guide.q_value_array,
                        extended_space_pomdp.rollout_guide.value_array,extended_space_pomdp.rollout_guide.env,extended_space_pomdp.rollout_guide.veh,extended_space_pomdp.rollout_guide.state_grid,100)

x0 = SVector(1.5,14,8*pi/9,0.0)
q5,w5,e,r = plan_path(x0,HJB_policy,750,extended_space_pomdp.rollout_guide.get_actions,
                        extended_space_pomdp.rollout_guide.get_cost,extended_space_pomdp.one_time_step,extended_space_pomdp.rollout_guide.q_value_array,
                        extended_space_pomdp.rollout_guide.value_array,extended_space_pomdp.rollout_guide.env,extended_space_pomdp.rollout_guide.veh,extended_space_pomdp.rollout_guide.state_grid,100)


hg_plot_HJB_path([q1,q2,q3,q4,q5],[w1,w2,w3,w4,w5],rollout_guide.env, rollout_guide.veh, 2.5, ["","","","",""])
