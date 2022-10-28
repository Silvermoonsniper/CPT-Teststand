rem subroutie for model controller
rem input args:
rem     vel_local: velocity data
rem     datasize_train: size of velocity data array
rem     predict_distance_array[]: predicted distance array
rem     pre_dist_p: pointer for predicted distance array
rem     weg_ingoutarray[]: controller output array (2 consecutive values)
rem     enter_cycle_number: number of closed loop control cycles
rem     desired_dist: desired distance
rem     intervalp: number of points for velocity calculation
rem     linear_coff[]: estimated parameters:
rem     Abtastfrequenz: sampling frequency
rem     Sinusfrequenz_Vibro: current set vibration frequency
rem     Vibro_internal_counter: counter for vibration motion
rem     total_shiftdist: initial phase of desired distance profile
rem     WegInd_RegOut: controller output
rem     sollvel: desired velocity
rem     initialWegInd_Digits: distance data when we switch on model controller
rem     predict_dist: predicted distance
rem     estimatedWegInd_RegOut: estimated controller output
rem     total_shift: number of points for time delay
rem     switch_flag: flag to switch
rem     WegInd_Digits: position of piston in digital value
rem     vel_deviation: velocity deviation
rem     dist_err: distance error
rem     proportion_str: constant for proportional control
  
sub model_control(vel_local,datasize_train,predict_distance_array[],pre_dist_p, weg_ingoutarray[],enter_cycle_number,desired_dist,intervalp,linear_coff[],Abtastfrequenz,Sinusfrequenz_Vibro,Vibro_internal_counter,total_shiftdist, WegInd_RegOut,sollvel,initialWegInd_Digits,predict_dist,estimatedWegInd_RegOut,total_shift,switch_flag,global_pointer, WegInd_Digits,vel_deviation, dist_err, proportion_str) as float
  
  rem  STORE estimated controller output and real one 
  DIM initial_desired_velocity as float
  rem define factor 
  DIM factor as long
  rem define model quality factor
  DIM q_m as float

  '##############################################################################
  '##############################################################################
  rem switch when distance cross zero
  rem shift points for get same phase between desired and measured distance
  
  total_shiftdist = round(0.25*Abtastfrequenz/Sinusfrequenz_Vibro) + 15
  if ((Vibro_internal_counter = total_shiftdist) and (switch_flag = 0)) then
    switch_flag = 1
    rem record velocity at switch point
    desired_distance(desired_dist,WegInd_Digits,switch_flag,total_shiftdist, original_vib_freq,local_index,penetration_vel,global_pointer,desired_distpointer,total_shiftdist,initialWegInd_Digits,fulldata_number,Abtastfrequenz,0.65*damping_amplitude,Sinusfrequenz_Vibro,Vibro_internal_counter)
 
    initial_desired_velocity = (desired_dist - WegInd_Digits) / intervalp
   
    
   
  endif
 
  
  if (switch_flag = 1) then
    
    inc(enter_cycle_number)
   
    rem calculate difference between desired and predict distance
     
       
    rem calculate difference between desired and predicted distance
    
   
    dec(pre_dist_p )
    total_shift = total_shift
    if (pre_dist_p > total_shift) then
      dist_err = desired_dist - predict_distance_array[pre_dist_p - total_shift]
    else
      dist_err = desired_dist - predict_distance_array[datasize_train + pre_dist_p - total_shift]
    endif 
    
    
    ' adaptively modify controller output
    
    rem model quality factor
    if (pre_dist_p > total_shift) then
      q_m = (desired_dist - predict_distance_array[pre_dist_p - total_shift]) / desired_dist   
    else
      q_m = (desired_dist - predict_distance_array[datasize_train + pre_dist_p - total_shift]) / desired_dist   
    endif
 
    rem calculate velocity deviation at current step
    
    vel_deviation = 1*dist_err/ total_shift
        
    
  
    rem desired velocity 
  
    sollvel =  (desired_dist - WegInd_Digits) / intervalp
    
    rem controller output from model
    factor = round(14*(0.5*damping_amplitude-1)) + 1
   
    '    dec(local_c)
    '    if (local_c > total_shift) then
    '      factor = (1/(q_m*(sollvel-vel_deviation)))*(2- (controlleroutput[local_c-total_shift] *linear_coff[1]+ linear_coff[2]))
    '    else
    '      factor = (1/(q_m*(sollvel-vel_deviation)))*(2- (controlleroutput[data_sizetrain + local_c-total_shift] *linear_coff[1]+ linear_coff[2]))
    '    
    '          
    '    endif
    
    estimatedWegInd_RegOut = ( (1-factor*q_m)*vel_deviation + (1+ factor*q_m)*sollvel - linear_coff[2])/(linear_coff[1])
         
    
                         
 
   
    rem assign real controller output
   
    WegInd_RegOut = estimatedWegInd_RegOut
  
    
   
  endif
 
  

  
  if (enter_cycle_number <= 2) then
    weg_ingoutarray[enter_cycle_number] = WegInd_RegOut
  else
    weg_ingoutarray[1] = weg_ingoutarray[2]
    weg_ingoutarray[2] = WegInd_RegOut
  endif
 
endsub
