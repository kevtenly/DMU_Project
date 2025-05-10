extended_space_pomdp = ExtendedSpacePOMDP(pomdp_details,env,veh_params,rollout_guide);
pomdp_solver = DESPOTSolver(bounds=IndependentBounds(DefaultPolicyLB(FunctionPolicy(b->calculate_lower_bound(extended_space_pomdp, b)),max_depth=pomdp_details.tree_search_max_depth),
                    calculate_upper_bound,check_terminal=true,consistency_fix_thresh=1e-5),K=pomdp_details.num_scenarios,D=pomdp_details.tree_search_max_depth,
                    T_max=0.4,tree_in_info=true);#,default_action=default_es_pomdp_action)
pomdp_solver = DESPOTSolver(bounds=IndependentBounds(DefaultPolicyLB(FunctionPolicy(b->calculate_lower_bound(extended_space_pomdp, b)),max_depth=pomdp_details.tree_search_max_depth),
                    calculate_upper_bound,check_terminal=true,consistency_fix_thresh=1e-5),K=pomdp_details.num_scenarios,D=pomdp_details.tree_search_max_depth,
                    max_trials=20,tree_in_info=true,lambda=0);#,default_action=default_es_pomdp_action)
pomdp_solver = DESPOTSolver(bounds=IndependentBounds(DefaultPolicyLB(FunctionPolicy(b->calculate_lower_bound(extended_space_pomdp, b)),max_depth=pomdp_details.tree_search_max_depth),
                    old_calculate_upper_bound,check_terminal=true,consistency_fix_thresh=1e-5),K=pomdp_details.num_scenarios,D=pomdp_details.tree_search_max_depth,
                    T_max=0.3,max_trials=1,tree_in_info=true,rng=sol_rng);#,default_action=default_es_pomdp_action)
pomdp_solver = DESPOTSolver(bounds=IndependentBounds(DefaultPolicyLB(FunctionPolicy(b->calculate_lower_bound(extended_space_pomdp, b)),max_depth=pomdp_details.tree_search_max_depth),
                    old_calculate_upper_bound,check_terminal=true,consistency_fix_thresh=1e-5),K=pomdp_details.num_scenarios,D=pomdp_details.tree_search_max_depth,
                    max_trials=20,tree_in_info=true,rng=sol_rng,lambda=0);#,default_action=default_es_pomdp_action)

pomdp_planner = POMDPs.solve(pomdp_solver, extended_space_pomdp);
ti = 5.0
b = output.b_root[ti]
next_pomdp_action, info = action_info(pomdp_planner, b);
println(next_pomdp_action)
inchrome(D3Tree(info[:tree]))
tree = info[:tree]
@profiler action_info(pomdp_planner, b)


pomdp_solver = DESPOTSolver(bounds=IndependentBounds(DefaultPolicyLB(FunctionPolicy(b->calculate_lower_bound(extended_space_pomdp, b)),max_depth=pomdp_details.tree_search_max_depth),
                    calculate_upper_bound,check_terminal=true,consistency_fix_thresh=1e-5),K=1,D=pomdp_details.tree_search_max_depth,
                    max_trials=20,tree_in_info=true);#,default_action=default_es_pomdp_action)


function find_tree_depth(tree)
    max_depth = -Inf
    max_depth_scenario_num = -Inf
    for (i,ele) in enumerate(tree.parent_b)
        curr_scenario_num = i
        parent_scenario_num = ele
        curr_scenario_depth = 0
        while parent_scenario_num!=0
            curr_scenario_depth +=1
            new_parent_scenario_num = tree.parent_b[parent_scenario_num]
            parent_scenario_num = new_parent_scenario_num
        end
        if(curr_scenario_depth > max_depth)
            max_depth = curr_scenario_depth
            max_depth_scenario_num = curr_scenario_num
        end
    end
    println("Max Node Depth in this tree is : ", max_depth)
    println("Scenario with Max Depth is at index : ", max_depth_scenario_num)

    return max_depth, max_depth_scenario_num
    #=
    Here is a better way to do it which I just didn't know before.
    SO FREAKING STUPID OF ME!
    max_depth, max_depth_scenario_num = findmax(tree.Delta)
    =#
end

max_d,max_d_sce_num = find_tree_depth(info[:tree])

k = info[:tree].parent_b[max_d_sce_num];
println(k," ", info[:tree].scenarios[k][1], "\n")
k = info[:tree].parent_b[k];

for i in 33:0.1:38
    observe(output, exp_details,i, [])
end


q,w,e,r = plan_path(x0,HJB_policy,750,extended_space_pomdp.rollout_guide.get_actions,
                   extended_space_pomdp.rollout_guide.get_cost,extended_space_pomdp.one_time_step,extended_space_pomdp.rollout_guide.q_value_array,
                   extended_space_pomdp.rollout_guide.value_array,extended_space_pomdp.rollout_guide.env,extended_space_pomdp.rollout_guide.veh,extended_space_pomdp.rollout_guide.state_grid,100)



scenario_num=10
belief = ScenarioBelief(tree.scenarios[scenario_num],pomdp_planner.rs, 1, missing);
s = belief.scenarios[1][2]
println(s)
a = calculate_lower_bound(extended_space_pomdp, belief)
println(a)

POMDPs.gen(extended_space_pomdp,s,a,MersenneTwister(1))