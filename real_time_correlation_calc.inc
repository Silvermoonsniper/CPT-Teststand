rem subroutine to calculate correlation 

sub  cross_correlation(index,fulldata_number,vel_max,vel_min,corr_product[],pointer_vel_correlation,corr_velocity_data[],corr_controlleroutput[],velocity_data,controlleroutput[],WegInd_RegOut,local_vel_pointer)    as float
  rem define local pointer
  
  
  
  rem define shift index
  DIM shift_index as long
  
  rem define flag to indicate if buffer is full
  DIM correlation_full_flag as long
  shift_index = 0
  index = 1
  rem check if we reach the end
  if  ((pointer_vel_correlation <= fulldata_number)) then
    
    
    if ((correlation_full_flag = 0)) then
    
      corr_velocity_data[pointer_vel_correlation] = normalization(vel_max,vel_min,velocity_data[local_vel_pointer])  
      rem current controller output
      corr_controlleroutput[pointer_vel_correlation] = WegInd_RegOut
    endif
  
  
    rem calculate correlation for two signals by using incremental approach
    if (correlation_full_flag = 0) then
      for index = 1 to fulldata_number
        rem obtain velocity data  'shift_index' steps before and then perform correlation calculation
        if ((pointer_vel_correlation > shift_index) ) then
          corr_product[index] = corr_product[index] + corr_velocity_data[pointer_vel_correlation-shift_index]*WegInd_RegOut
        else
          corr_product[index] = corr_product[index] + corr_velocity_data[fulldata_number + (pointer_vel_correlation - shift_index)]*WegInd_RegOut  
        endif 
        inc(shift_index)
        
      next index
      rem reinitialize shift index
      shift_index = 0
      index = 1
    endif
 
    rem if we reach the end of buffer
    if (pointer_vel_correlation = fulldata_number) then 
      rem flag to indicate buffer is full
      correlation_full_flag = 1
    
    endif
    rem when buffer is full
    if (correlation_full_flag = 1) then
      for index = 1 to fulldata_number
        rem remove old value from summation terms
        if ((pointer_vel_correlation > (fulldata_number-1)) ) then
          corr_product[index] = corr_product[index] - corr_velocity_data[pointer_vel_correlation-(fulldata_number-1)]*corr_controlleroutput[pointer_vel_correlation]
        else
          corr_product[index] = corr_product[index] - corr_velocity_data[fulldata_number + pointer_vel_correlation -(fulldata_number-1)]*corr_controlleroutput[pointer_vel_correlation]
        endif
        inc(shift_index)
        
      next index
      index = 1
      rem reinitialize shift index
      shift_index = 0
      
      rem update velocity and controller output data array
      
      corr_velocity_data[pointer_vel_correlation] = normalization(vel_max,vel_min,velocity_data[local_vel_pointer])  
      rem current controller output
      corr_controlleroutput[pointer_vel_correlation] = WegInd_RegOut
    
      
      rem add new product for correlation calculation
      for index = 1 to fulldata_number
        rem obtain velocity data  'shift_index' steps before and then perform correlation calculation, append for products
        if ((pointer_vel_correlation > shift_index) ) then
          corr_product[index] = corr_product[index] + corr_velocity_data[pointer_vel_correlation-shift_index]*WegInd_RegOut
        else
          corr_product[index] = corr_product[index] + corr_velocity_data[fulldata_number + (pointer_vel_correlation - shift_index)]*WegInd_RegOut  
         
        endif 
        inc(shift_index)
       
        
      next index
      index = 1
      rem reinitialize shift index
      shift_index = 0
    endif
    rem update pointer
    pointer_vel_correlation = pointer_vel_correlation + 1
  else
    pointer_vel_correlation = 1
  endif
  
  
  
endsub
