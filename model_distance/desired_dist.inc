rem function to calculate desired distance data at current step
rem      initialWegInd_Digits:distance sensor measurement at controller starting point
rem      Amplitude_Vibro: vibration amplitude for desired sine+linear cone penetration movement
rem      Sinusfrequenz_Vibro: vibration frequency
rem      Vibro_internal_counter: number for calculate sine movement value, cyclically repeated by sine period
rem      Programm_counter: measurement cycles from programm  start
rem      penetration_vel:linear penetration velocity in physical unit (mm)
rem      controlstart_step: controller starting step
sub desired_distance(desired_dist,WegInd_Digits,switch_flag,total_shiftdist, original_vib_freq,local_index,penetration_vel,global_pointer,desired_distpointer,total_shift,initialWegInd_Digits,localstepval,Abtastfrequenz,Amplitude_Vibro,Sinusfrequenz_Vibro,Vibro_internal_counter) as float
  
 
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

    rem desired distance in physical unit (mm)
    rem specify penetration velocity
    penetration_vel = fpar_24
  
    rem calculate initial distance shifted back by total_shift steps in mm rem
    if (desired_distpointer <=(Abtastfrequenz/Sinusfrequenz_Vibro) ) then
      desired_distance_phy = (-420/33076) * initialWegInd_Digits+623 + Amplitude_Vibro * sin(2*3.14159*(Sinusfrequenz_Vibro/Abtastfrequenz)* (Vibro_internal_counter - total_shift +1) ) + (penetration_vel/Abtastfrequenz)*global_pointer
    
      desired_distpointer = desired_distpointer + 1
      inc(global_pointer)
    else
      desired_distpointer = 1
      inc(global_pointer)  
    endif
    rem transform into digital value
    rem final desired digital distance at current step is distance measurement at controller starting point plus distance difference
    desired_dist = (desired_distance_phy-623)*(-33076/420)  
   
  endif
  

endsub
