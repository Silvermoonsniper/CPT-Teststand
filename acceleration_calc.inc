rem subroutine for acceleration calculation

sub acceleration_cal(accleration_full,intervalp,local_acceleration,acceleration_size,data_sizetrain,accleration[],velocity_data[],local_vel_pointer) as float
  
  
  
  
  
  
  
  if  (local_acceleration <= acceleration_size) then
    if (local_vel_pointer > intervalp) then
      accleration[local_acceleration] =  velocity_data[local_vel_pointer] -  velocity_data[local_vel_pointer-intervalp]
     
    else 
      accleration[local_acceleration] =  velocity_data[local_vel_pointer] -  velocity_data[data_sizetrain + local_vel_pointer-intervalp]
    endif
    local_acceleration = local_acceleration + 1
  else
    rem reinitialie index, when data array is full
        
    local_acceleration = 1
    accleration_full = 1       
  endif 
  
  if (accleration_full = 1) then
    if ((local_acceleration > 1) and (local_acceleration < acceleration_size)) then
      accleration[local_acceleration] = accleration[local_acceleration-1] + (accleration[local_acceleration]- accleration[local_acceleration + 1])/acceleration_size
    else
      if(local_acceleration = 1) then
        accleration[local_acceleration] = accleration[acceleration_size] + (accleration[local_acceleration]- accleration[1])/acceleration_size
      endif
      if (local_acceleration = acceleration_size) then
        accleration[local_acceleration] = accleration[local_acceleration-1] + (accleration[local_acceleration]- accleration[1])/acceleration_size
   
        
      endif
    endif
      
  endif
  
  rem check for extreme values
  if (absf(accleration[local_acceleration]-accleration[local_acceleration-1]) < 0.4) then
  
    Fpar_50 = accleration[local_acceleration]
  endif
  
endsub
