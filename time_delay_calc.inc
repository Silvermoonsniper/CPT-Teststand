rem calculate time offset steps for distance and controller output data
rem  Input args:
rem        original_vib_freq: initial vibration frequency
rem        Sinusfrequenz_Vibro: current vibration frequency
rem        correlation_control_button: button for starting correlation
rem        fulldata_number: number of datapoints for one period
rem        corr_pointerdist: pointer
rem        total_shiftdist: time delay between distance and controller output
rem        time_offsetdist[]: time delay array
rem        total_shift_maxdist: final time delay between distance and controller output
sub timeoffset_distance(original_vib_freq,Sinusfrequenz_Vibro,correlation_control_button,fulldata_number,corr_pointerdist,total_shiftdist,time_offsetdist[],total_shift_maxdist) as float
  
  rem calculate shifted time steps
  if(correlation_control_button = 1) then
    rem update control cycle
    inc(control_cycle)
    total_shift_maxdist = fpar_64
    if (corr_pointerdist <= 2) then
      time_offsetdist[corr_pointerdist] = total_shift_maxdist
      inc(corr_pointerdist)
    else
      '      if (corr_pointerdist <= fulldata_number) then
      rem check if time delay shows spike
       
      time_offsetdist[1] = time_offsetdist[2]
      time_offsetdist[2] = total_shift_maxdist
       
      if (Sinusfrequenz_Vibro < original_vib_freq ) then  
        time_offsetdist[2] = max_float(time_offsetdist[2],time_offsetdist[1])
      else
        time_offsetdist[2] = min_float(time_offsetdist[2],time_offsetdist[1])
      endif
      rem assign maximum step
      fpar_66 = time_offsetdist[2]
      total_shiftdist = time_offsetdist[2]
      '    endif
      inc(corr_pointerdist)
    endif
   
  
  
  endif
endsub




rem calculate time offset steps
rem  original_vib_freq:  original vibration frequency
rem  Sinusfrequenz_Vibro: current set vibration frequency
rem  correlation_control_button: button to start correlation
rem  fulldata_number: number of datapoints for one period
rem  corr_pointer: correlation pointer
rem  total_shift: time delay
rem  time_offset[]:time delay array
rem  total_shift_max: final time delay

sub timeoffset(control_cycle,original_vib_freq,Sinusfrequenz_Vibro,correlation_control_button,fulldata_number,corr_pointer,total_shift,time_offset[],total_shift_max) as float
  rem update control cycle
  inc(control_cycle)
  rem calculate shifted time steps
  if(correlation_control_button = 1) then
   
    total_shift_max = par_53
    if (corr_pointer <= 2) then
      time_offset[corr_pointer] = total_shift_max
      inc(corr_pointer)
    else
      rem if (corr_pointer <= fulldata_number) then
      time_offset[1] = time_offset[2]
      time_offset[2] = total_shift_max
      rem endif
      inc(corr_pointer)
    endif
     
    if (Sinusfrequenz_Vibro <= original_vib_freq ) then  
      total_shift_max = max_float(time_offset[2],time_offset[1])
    else
      total_shift_max = min_float(time_offset[2],time_offset[1])
    endif
    time_offset[2] = total_shift_max
    
    rem assign maximum step
    
    par_45 = time_offset[2]
    
    total_shift = time_offset[2]
  
  endif
 
endsub
