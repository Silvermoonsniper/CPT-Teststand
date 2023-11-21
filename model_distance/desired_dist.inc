rem function to calculate time shifted desired distance data at current step

rem      control_cycle: the number of cylces that we enter into control
rem      final_switch: flag to finally switch on model control
rem      final_switch_flag: flag to finally switch on model control for reset background shift
rem      current_p_vel: current background velocity
rem      amplify_factor_m: amplification factor for model para m
rem      enter_cycle_number: number of cycles we enter into control
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

sub desired_distance(Vibro_mode_2_background_vel,Vibro_mode_3_background_vel,control_cycle,final_switch,final_switch_flag,current_p_vel,amplify_factor_m,enter_cycle_number,switch_vibro_dist_p_1,virboswitch_flag_1,virboswitch_flag,switch_vibro_dist_p,desired_dist,WegInd_Digits,switch_flag,total_shiftdist, original_vib_freq,local_index,penetration_vel,global_pointer,desired_distpointer,total_shift,initialWegInd_Digits,localstepval,Abtastfrequenz,Amplitude_Vibro,Sinusfrequenz_Vibro,Vibro_internal_counter) as float
  
 
  rem desired trajectory is defined with combination of linear penetration movment of cone and sinsuidal damping
  Dim desired_distance_phy as float
  DIM amplitdue_amplification as float
  if ( switch_flag = 1) then
    '    total_shiftdist = 1
    total_shiftdist = round(0.25*Abtastfrequenz/Sinusfrequenz_Vibro) + 15

    'rem reinitialization for vibro mode 2  
    if (virboswitch_flag = 1) then
            
      if (( switch_vibro_dist_p = 0) ) then
        rem reinitialize switch flag and initial distance when we switch to another vibro mode 
        local_index = 1
        switch_flag = 0
        final_switch = 0
        control_cycle = 0
        enter_cycle_number = 0
        current_p_vel = Vibro_mode_2_background_vel  
        final_switch_flag = 0
      endif
      inc( switch_vibro_dist_p)
    endif
    rem reinitialization for vibro mode 3                  
    if (virboswitch_flag_1 = 1) then
      if ( (switch_vibro_dist_p_1 = 0) ) then 
        rem reinitialize switch flag and initial distance when we switch to another vibro mode 
     
        local_index = 1
        switch_flag = 0
        final_switch = 0
        control_cycle = 0
        enter_cycle_number = 0
        current_p_vel = Vibro_mode_3_background_vel  
        final_switch_flag = 0
        
      endif
      inc( switch_vibro_dist_p_1)
    endif
    '    
   
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
    
    if (final_switch = 1) then
      
     
      
      
      inc(final_switch_flag)
      if (final_switch_flag = 1)then
        global_pointer = 0
            
      endif
      
      current_p_vel =      Fpar_24 
      rem assign current background velocity 
      if (virboswitch_flag = 1) then
      
        current_p_vel = Vibro_mode_2_background_vel 
      
      endif
      
      if (virboswitch_flag_1 = 1) then
      
        current_p_vel = Vibro_mode_3_background_vel 
      
      endif
      
    endif    
    
    
    
    
    rem desired distance in physical unit (mm)
    
    
    rem calculate initial distance shifted back by total_shift steps in mm rem
    if (switch_flag = 1) then
      
      amplitdue_amplification = 1/amplify_factor_m
          
      if (0.5*Amplitude_Vibro > 2) then
        desired_distance_phy = (-420/33076) * initialWegInd_Digits+623 + amplitdue_amplification*1*Amplitude_Vibro * cos(2*3.14159*(Sinusfrequenz_Vibro/Abtastfrequenz)* (Vibro_internal_counter - total_shiftdist +1) + 1.5*3.14159) + ((current_p_vel)/Abtastfrequenz)*global_pointer 
      else
        desired_distance_phy = (-420/33076) * initialWegInd_Digits+623 + amplitdue_amplification*1*Amplitude_Vibro * cos(2*3.14159*(Sinusfrequenz_Vibro/Abtastfrequenz)* (Vibro_internal_counter - total_shiftdist +1) + 1.5*3.14159) + ((current_p_vel)/Abtastfrequenz)*global_pointer 
      endif
        
      '      
      '    -  amplitdue_amplification*Amplitude_Vibro *cos(3.14159)  
   
    
    
    
    
      rem transmitted desired distance
    
      desired_distance_phy_transmit =  1*((-420/33076) * initialWegInd_Digits+623 + 0.5*Amplitude_Vibro * cos(2*3.14159*(Sinusfrequenz_Vibro/Abtastfrequenz)* (Vibro_internal_counter - total_shiftdist +1) + 1.5*3.14159 ) + ((current_p_vel)/Abtastfrequenz)*global_pointer)
  
  
    
      inc(global_pointer)
    
      rem transform into digital value
      rem final desired digital distance at current step is distance measurement at controller starting point plus distance difference
      desired_dist = (desired_distance_phy-623)*(-33076/420)  
    else
      
      desired_distance_phy_transmit =  (-420/33076) * initialWegInd_Digits+623
    endif
  
  else
  
    desired_distance_phy_transmit = (-420/33076) *WegInd_Digits  +623
  endif
 

endsub
