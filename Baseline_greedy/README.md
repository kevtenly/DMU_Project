# target_tracking_dynaminc_envs

To get started, you first need to install all the required packages.
1. Go the code's folder
2. Activate the current environment using the command Pkg.activate("./")
3. Run Pkg.precompile(). It will fail and tell you that BellmanPDEs.jl isn't registered.
4. That is expected. Add it from its github repo using the command Pkg.add(url = "https://github.com/himanshugupta1009/BellmanPDEs.jl").
5. Run Pkg.precompile() again.


Now, there are two main files: main_ls.jl and main_es.jl
1. Run main_ls.jl to run the code for the limited space planning approach.
2. Run main_es.jl to run the code for the extended space planning approach.
    2.a) When you run this file for the first time, comment out the HJB_flag = false line.
    2.b) This ensures that a rollout guide is created for the current environment. 
    2.c) This rollout guide needs to be created for every new static environment that you wish to test on.
    2.d) After that uncomment the HJB_flag = false line.
3. Both of these files call other files inside them and that should give you an idea about the helper files needed for each approach.
   
# DMU_Project
