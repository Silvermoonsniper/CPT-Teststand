rem function to normalize 1D dataset
rem normalization 
rem input args:
rem    vel_max: maximum data
rem    vel_min: min data
rem    vel_local_ local velocity
function normalization(vel_max,vel_min,vel_local) as float
  normalization = (vel_local - vel_min)/(vel_max - vel_min)
endfunction 
rem subroutine to calculate maximum and minimum of 1D data
sub max_min_calcul(fulldata_number,vel_pointer,local_velocity,local_velocity_1,local_control,local_control_1,vel_max,vel_min,control_max,control_min,vel_local,WegInd_RegOut) as float
  rem record velocity data
  if (vel_pointer <= 2) then
    local_velocity[vel_pointer] = vel_local
    local_velocity_1[vel_pointer] = vel_local
    local_control[vel_pointer] = WegInd_RegOut
    local_control_1[vel_pointer] = WegInd_RegOut
    inc(vel_pointer)
  else
    local_velocity[1] = local_velocity[2]
    local_velocity[2] = vel_local
    local_velocity_1[1] = local_velocity_1[2]
    local_velocity_1[2] = vel_local
        
    local_control[1] = local_control[2]
    local_control[2] = WegInd_RegOut
    local_control_1[1] = local_control_1[2]
    local_control_1[2] = WegInd_RegOut
    inc(vel_pointer)
  endif
  rem get maximum velocity and minimum velocity
  if (vel_pointer <= 1*fulldata_number) then
    vel_max = max_float(local_velocity[2],local_velocity[1])
    local_velocity[2] = vel_max  
    vel_min = min_float(local_velocity_1[2],local_velocity_1[1])
    local_velocity_1[2] = vel_min  
    control_max = max_float(local_control[2],local_control[1]) 
    local_control[2] = control_max
    control_min = min_float(local_control_1[2],local_control_1[1])
    local_control_1[2] = control_min  
  
  endif
 
endsub
