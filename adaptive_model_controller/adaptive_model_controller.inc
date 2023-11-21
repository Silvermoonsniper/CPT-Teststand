rem subroutie for model controller
rem input args:
rem     control_cycle: number of control cycle
rem     Kp_Position: Kp const fo pi control
rem     ZERO_OFFSET: digital zero voltage value
rem     MAX_OUTPUT: Max digiatl output
rem     WegInd_Delta: error term in PI control
rem     Soll_Position_Regelungswert: desired digital distance
rem     amplify_factor_m: amplify factor
rem     current_p_vel: current background velocity
rem     switch_vibro_dist_p: index in order to  reinitialize model controller when we switch to second vibro mode
rem     switch_vibro_dist_p_1: index in order to  reinitialize model controller when we switch to third vibro mode
rem     controlleroutput[]: controller output array
rem     local_c: index for controller output array
rem     final_switch: switch flag for final switch to model control
rem     penetration_vel: background velocity of CPT test , mm/s
rem     virboswitch_flag: flag indicats switch to second vibro mode
rem     virboswitch_flag_1: flag indicats switch to third vibro mode
rem     damping_amplitude: vibration amplitude
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
  
sub model_control(Vibro_mode_2_background_vel,Vibro_mode_3_background_vel,control_cycle,Kp_Position,ZERO_OFFSET,MAX_OUTPUT,WegInd_Delta,Soll_Position_Regelungswert,amplify_factor_m,current_p_vel,switch_vibro_dist_p,switch_vibro_dist_p_1,controlleroutput[],local_c,final_switch,penetration_vel,virboswitch_flag, virboswitch_flag_1, damping_amplitude,vel_local,datasize_train,predict_distance_array[],pre_dist_p, weg_ingoutarray[],enter_cycle_number,desired_dist,intervalp,linear_coff[],Abtastfrequenz,Sinusfrequenz_Vibro,Vibro_internal_counter,total_shiftdist, WegInd_RegOut,sollvel,initialWegInd_Digits,predict_dist,estimatedWegInd_RegOut,total_shift,switch_flag,global_pointer, WegInd_Digits,vel_deviation, dist_err, proportion_str) as float
  
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
  
  '  total_shiftdist = 1
  total_shiftdist = round(0.25*Abtastfrequenz/Sinusfrequenz_Vibro) + 15
  if ((Vibro_internal_counter = total_shiftdist) and (switch_flag = 0)) then
 
    total_shiftdist = Vibro_internal_counter
    if ((-420/33076) * WegInd_Digits + 623 < (Switch_freq_depth-2)) then
    
      switch_flag = 1
    else
      if ((virboswitch_flag = 1) or (virboswitch_flag_1 = 1)) then
        switch_flag = 1
      
      endif
      
    endif 
    rem record velocity at switch point
 
    '    if (final_switch = 1) then
    if (switch_flag = 1) then
      desired_distance(Vibro_mode_2_background_vel,Vibro_mode_3_background_vel,control_cycle,final_switch,final_switch_flag,current_p_vel,amplify_factor_m,enter_cycle_number,switch_vibro_dist_p_1,virboswitch_flag_1,virboswitch_flag,switch_vibro_dist_p,desired_dist,WegInd_Digits,switch_flag,total_shiftdist, original_vib_freq,local_index,penetration_vel,global_pointer,desired_distpointer,total_shiftdist,initialWegInd_Digits,fulldata_number,Abtastfrequenz,0.65*damping_amplitude,Sinusfrequenz_Vibro,Vibro_internal_counter)
 
      initial_desired_velocity = (desired_dist - WegInd_Digits) / intervalp
    else
      desired_dist = Soll_Position_Regelungswert
      
    
      
    endif
   
  endif
  
 
  
       
  
  if (switch_flag = 1) then
    
    inc(enter_cycle_number)
   
    rem calculate difference between desired and predict distance
     
       
    rem calculate difference between desired and predicted distance
    
    
    
    dec(pre_dist_p )
    
   
    
    
    total_shift = total_shift
    if (pre_dist_p > total_shift) then
      dist_err = desired_dist - predict_distance_array[pre_dist_p - 0]
    else
      dist_err = desired_dist - predict_distance_array[datasize_train + pre_dist_p - 0]
    endif 
    
    
    ' adaptively modify controller output
    
    rem model quality factor
    if (pre_dist_p > total_shift) then
      q_m = (desired_dist - predict_distance_array[pre_dist_p - total_shift]) / desired_dist   
    else
      q_m = (desired_dist - predict_distance_array[datasize_train + pre_dist_p - total_shift]) / desired_dist   
    endif
 
    rem calculate velocity deviation at current step
    
    '    if (final_switch = 1) then
    '      vel_deviation = 1*dist_err/ round(0.769*total_shift)
    '    else
    '      
    '      vel_deviation = 1*dist_err/ round(0.769*total_shift)
    '    endif
    
    if (final_switch = 1) then
      vel_deviation =amplify_factor_m*( 1*dist_err/ round((1)*total_shift))
    else
      
      vel_deviation = amplify_factor_m*( 1*dist_err/ round((1)*total_shift))
    endif
    rem desired velocity 
  
    sollvel =  (desired_dist - WegInd_Digits) / intervalp
  
    
    if ((virboswitch_flag = 1) or (virboswitch_flag_1 = 1)) then q_m = 0
    
    if (penetration_vel > 0) then q_m = 0
    
    
    rem controller output from model
    
    if (damping_amplitude < 2) then
      '      dec(local_c)
      if (local_c > total_shift) then
        factor = (1/(q_m*(sollvel-vel_deviation)))*(2- (controlleroutput[local_c-total_shift] *linear_coff[1]+ linear_coff[2]))
      else
        factor = (1/(q_m*(sollvel-vel_deviation)))*(2- (controlleroutput[datasize_train + local_c-total_shift] *linear_coff[1]+ linear_coff[2]))
                          
                                
      endif
    else
      factor = round(14*(0.5*damping_amplitude-1)) + 1
    endif
    
    rem for very low frequency, adaptive control is deactivated for safety control 
    if (Sinusfrequenz_Vibro < 1) then
      q_m = 0
    endif
   
   
    q_m = 0
    
   
      
      
      
    estimatedWegInd_RegOut = (( (1-factor*q_m)*vel_deviation + (1+ factor*q_m)*sollvel - linear_coff[2])/(linear_coff[1]))
    
   
      
                   
    rem check controller output before switch and cmpare it with estimated controller output
                
 
    '    if (absf(controlleroutput[local_c] - estimatedWegInd_RegOut ) < 30) then
                                 
    rem assign real controller output
                                 
    final_switch = 1
                                  
    '  endif
                    
    rem for very small frequency
      
    rem switch to final model controller when estimated control output (model) is sufficiently close to 
    rem actual control output, this is in order to smooth the controller and piston motion.                
    if (final_switch = 1) then 
      WegInd_RegOut = estimatedWegInd_RegOut
    else
      
      
      
      rem hold the position of piston when model controller mode is not engaged.                      
      '      if (virboswitch_flag = 0)  then          
      WegInd_Delta = WegInd_Digits - Soll_Position_Regelungswert 'auf letzte Position geregelt
      WegInd_RegOut = (Kp_Position * WegInd_Delta)
                                                                    
      If (WegInd_RegOut < -MAX_OUTPUT)  Then WegInd_RegOut = -MAX_OUTPUT
      If (WegInd_RegOut > MAX_OUTPUT)  Then WegInd_RegOut = MAX_OUTPUT
                                                              
      WegInd_RegOut= WegInd_RegOut + ZERO_OFFSET
              
      '    else
      '      WegInd_RegOut= 32668 + 100*(current_p_vel)  
      '                      
      '    endif     
      '                    
      '                    
      '    if (virboswitch_flag_1 = 0)  then          
      '      WegInd_Delta = WegInd_Digits - Soll_Position_Regelungswert 'auf letzte Position geregelt
      '      WegInd_RegOut = (Kp_Position * WegInd_Delta)
      '                                                                          
      '      If (WegInd_RegOut < -MAX_OUTPUT)  Then WegInd_RegOut = -MAX_OUTPUT
      '      If (WegInd_RegOut > MAX_OUTPUT)  Then WegInd_RegOut = MAX_OUTPUT
      '                                                                    
      '      WegInd_RegOut= WegInd_RegOut + ZERO_OFFSET
      '                    
      '    else
      '      WegInd_RegOut= 32668 + 100*(current_p_vel)  
      '                    
      '    endif                 
     
       
                         
    endif
                     
   
  endif
 
 

  
  if (enter_cycle_number <= 2) then
    weg_ingoutarray[enter_cycle_number] = WegInd_RegOut
  else
    weg_ingoutarray[1] = weg_ingoutarray[2]
    weg_ingoutarray[2] = WegInd_RegOut
  endif
  
endsub
