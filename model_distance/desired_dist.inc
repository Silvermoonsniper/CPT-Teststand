rem function to calculate time shifted desired distance data at current step
rem      switch_vibro_dist_p_1: depth when we switch to third vibro mode
rem      virboswitch_flag_1: flag to indicate we switch to third vibro mode
rem      virboswitch_flag: flag to indicate we switch to second vibro mode
rem      switch_vibro_dist_p: depth we want to switch to second vibro mode
rem      desired_dist: desired distance in digits
rem      WegInd_Digits: smoothed distance
rem      switch_flag: flag to switch to model controller
rem      total_shiftdist: number of datapoints for phase we switch to model controller
rem      original_vib_freq: original vibration frequency
rem      local_index: index to store distance data when switch to one vibro mode
rem      penetration_vel: background velocity
rem      global_pointer: pointer to update number of cycles we enter into vibro mode
rem      total_shift: number of data points for time delay
rem      initialWegInd_Digits:distance sensor measurement at controller starting point
rem      Amplitude_Vibro: vibration amplitude for desired sine+linear cone penetration movement
rem      Sinusfrequenz_Vibro: vibration frequency
rem      Vibro_internal_counter: number for calculate sine movement value, cyclically repeated by sine period
rem      Programm_counter: measurement cycles from programm  start
rem      penetration_vel:linear penetration velocity in physical unit (mm)
rem      controlstart_step: controller starting step

sub desired_distance(enter_cycle_number,switch_vibro_dist_p_1,virboswitch_flag_1,virboswitch_flag,switch_vibro_dist_p,desired_dist,WegInd_Digits,switch_flag,total_shiftdist, original_vib_freq,local_index,penetration_vel,global_pointer,desired_distpointer,total_shift,initialWegInd_Digits,localstepval,Abtastfrequenz,Amplitude_Vibro,Sinusfrequenz_Vibro,Vibro_internal_counter) as float
  
 
  rem desired trajectory is defined with combination of linear penetration movment of cone and sinsuidal damping
  Dim desired_distance_phy as float
  
  if ( switch_flag = 1) then
    total_shiftdist = round(0.25*Abtastfrequenz/Sinusfrequenz_Vibro) + 15
    if (local_index = 1)  then
               
      rem record original vibration frequency
      original_vib_freq = Sinusfrequenz_Vibro
      rem record initial piston position  
                                                                                            
      initialWegInd_Digits = WegInd_Digits
      global_pointer = 0
                
      inc(local_index)
    else
      inc(local_index)
    endif
    '  
    if (virboswitch_flag = 1) then
            
      if ( switch_vibro_dist_p = 0) then
        rem reinitialize switch flag and initial distance when we switch to another vibro mode 
        local_index = 1
        switch_flag = 0
        final_switch = 0
        enter_cycle_number = 0
      endif
      inc( switch_vibro_dist_p)
    endif
                    
    if (virboswitch_flag_1 = 1) then
      if ( switch_vibro_dist_p_1 = 0) then 
        rem reinitialize switch flag and initial distance when we switch to another vibro mode 
     
        local_index = 1
        switch_flag = 0
        final_switch = 0
        enter_cycle_number = 0
      endif
      inc( switch_vibro_dist_p_1)
    endif
    rem desired distance in physical unit (mm)

    
    rem calculate initial distance shifted back by total_shift steps in mm rem
   
    desired_distance_phy = (-420/33076) * initialWegInd_Digits+623 + Amplitude_Vibro * sin(2*3.14159*(Sinusfrequenz_Vibro/Abtastfrequenz)* (Vibro_internal_counter - total_shiftdist +1) ) + ((penetration_vel*0.366)/Abtastfrequenz)*global_pointer
    
    
    inc(global_pointer)
    
    rem transform into digital value
    rem final desired digital distance at current step is distance measurement at controller starting point plus distance difference
    desired_dist = (desired_distance_phy-623)*(-33076/420)  
   
  endif
 

endsub
