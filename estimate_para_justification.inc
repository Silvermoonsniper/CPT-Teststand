rem subroutine to deal with abnormal estimated parameters
Sub estimated_parameters_justification(Start_Test_Button,full_flag,estimated_parameter_pointer,linear_coff[],estimated_slope_array[],estimated_intercept_array[])   as float

  Dim safe_m as float
  DIM safe_b as float  
  if  (estimated_parameter_pointer <= 13) then 
    estimated_slope_array[estimated_parameter_pointer] = linear_coff[1]
    estimated_intercept_array[estimated_parameter_pointer] = linear_coff[2]                        
    estimated_parameter_pointer = estimated_parameter_pointer + 1
    
    '    
    '    
    if ( full_flag = 0) then
      safe_m =      -0.000301003 
      '       -0.000301003 
          
      safe_b =  12.9918
      '      12.9918         
      '       
      '      
    endif                        
  else
    rem reinitialize index
    estimated_parameter_pointer = 1
    full_flag = 1
    
  endif
 
  
  
 
  dec(estimated_parameter_pointer)
  rem for slope
  if (estimated_parameter_pointer >= 1)  then
    if (absf((estimated_slope_array[estimated_parameter_pointer] - safe_m)/ safe_m) > 0.2) then
      rem justify estimated parameters
      linear_coff[1] = safe_m
    endif
  else
    if (absf((linear_coff[1] - safe_m)/ safe_m)> 0.2) then
        
      linear_coff[1] = safe_m
    endif    
  endif
    
  rem for intercept
  if (estimated_parameter_pointer > 1)  then
    if (absf((estimated_intercept_array[estimated_parameter_pointer] - safe_b)/ safe_b) > 0.2) then
      rem justify estimated parameters
      linear_coff[2] = safe_b  
    endif
  else
    if (absf((estimated_intercept_array[1] - safe_b)/ safe_b) > 0.2) then
      linear_coff[2] = safe_b
    endif    
  endif
 
  
endsub
