# printing utility for benchmarking

def summarize_data(pd,sep_str=
    "\n============================================================================================"):
    moet=pd[['wcet']].mean()     
    meanet=pd[['mean_reach_time']].mean()
    sum_times = pd['time_taken_lec']+pd['time_taken_safety_controller']
    lec_percentage = pd['time_taken_lec']/sum_times
    safety_controller_percentage = pd['time_taken_safety_controller']/sum_times
    mean_experiment_time = sum_times.mean()
    num_experiments = len(sum_times)
    mean_lec_percentage =  lec_percentage.mean()
    mean_safety_controller_percentage = safety_controller_percentage.mean()
    print("\nlec usage %:",round(100*round(mean_lec_percentage,4),2)," | safety_controller usage %:",
        round(100*round(mean_safety_controller_percentage,4),2),"| moet: ",round(1000* round(moet.values[0],4),2),
        "| mean_et:",round(1000*round(meanet.values[0],4),2),sep_str,"| num_experiments: ",num_experiments)
    
    return num_experiments