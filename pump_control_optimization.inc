'<ADbasic Header, Headerversion 001.001>
'<Header End>


rem subroutine to append for pressure values
rem input args:
rem  axial_prss_array[]:axial pressure array
rem  side_prss_array[]: side pressure array
rem  pore_prss_array[]: pore pressure array
rem  pore_press_pointer: pointer to update index for pore pressure measurement
rem  side_press_pointer:  pointer to update index for side pressure measurement
rem  axial_press_pointer: pointer to update index for axial pressure measurement
rem  arraysize: size of array
rem  PorePress_Digits: pore pressure digits 
rem  SidePress_Digits: side pressure in digits 
rem  AxialPress_Digits: axial pressure in digits




sub press_array_append(axial_prss_array[],side_prss_array[], pore_prss_array[],pore_press_pointer,side_press_pointer,axial_press_pointer, arraysize, PorePress_Digits, SidePress_Digits,AxialPress_Digits) as float
  
  rem ##############################################################################
  rem append pore pressure measuemet into array
  if (pore_press_pointer <= arraysize) then
    pore_prss_array[pore_press_pointer] = (12000 / 65536) *PorePress_Digits - 6000
        
        
              
    inc(pore_press_pointer)
  else
    
    pore_press_pointer = 1
    pore_prss_array[pore_press_pointer] = (12000 / 65536) *PorePress_Digits -6000
    inc(pore_press_pointer)
  endif
  rem append side pressure measuemet into array
  
  
  if (side_press_pointer <= arraysize) then
    side_prss_array[side_press_pointer] = (12000 / 65536) *SidePress_Digits - 6000
        
        
              
    inc(side_press_pointer)
  else
  
    side_press_pointer = 1
    side_prss_array[side_press_pointer] = (12000 / 65536) *SidePress_Digits - 6000
    inc(side_press_pointer)
  endif
  
  rem append axial pressure measuemet into array
  
  
  if (axial_press_pointer <= arraysize) then
    axial_prss_array[axial_press_pointer] = (12000 / 65536) *AxialPress_Digits - 6000
        
        
              
    inc(axial_press_pointer)
  else
    rem check if sufficient number of measurement to calculat moving average is fulflled
   
    axial_press_pointer = 1
    axial_prss_array[pore_press_pointer] = AxialPress_Digits
    inc(pore_press_pointer)
  endif
  
  
  
endsub

rem input args:
rem  velocity_data[]: velocity data
rem  local_vel_pointer: pointer in velocity data array
rem  estimated_pore_press: estimated pore pressure
rem  estimated_side_press: estimated side pressure
rem  estimated_axial_press: estimated axial pressure
rem  arraysize: size of array
rem  axial_prss_array[]: axial pressure array
rem  side_prss_array[]: side pressure array
rem  pore_prss_array[]: pore pressure array
rem  pore_press_pointer: pointer for pore pressure measurement
rem  side_press_pointer: pointer for side pressure measurement
rem  axial_press_pointer: pointer for axial pressure measurement
rem  press_intervalp: interval to calculate pressure gradient
rem  data_sizetrain: size of velocity data array
rem  pore_m: parameter m for pore pressure disturbance
rem  side_m: parameter m for side pressure disturbance
rem  axial_m: parameter m for axial pressure disturbance
rem  pore_b: parameter b for pore pressure disturbance 
rem  side_b: parameter b for side pressure disturbance
rem  axial_b: parameter b for axial pressure disturbance
rem  time_delay_pore: time delay for pore pressure disturbance
rem  time_delay_side: time delay for pore pressure disturbance
rem  time_delay_axial: time delay for pore pressure disturbance

sub pressure_disturbance_estimation( velocity_data[],local_vel_pointer,  estimated_pore_press, estimated_side_press, estimated_axial_press,arraysize,axial_prss_array[],side_prss_array[], pore_prss_array[],pore_press_pointer,side_press_pointer,axial_press_pointer,press_intervalp,data_sizetrain, pore_m,side_m,axial_m,pore_b,side_b,axial_b, time_delay_pore, time_delay_side, time_delay_axial) as float
  DIM forward_velpore as float
  DIM forward_velside as float
  DIM forward_velaxial as float
  DIM porepress_gradient as float
  DIM sidepress_gradient as float
  DIM axialpress_gradient as float
  
  '  dec(local_vel_pointer) 
  rem ###################################################
  if (local_vel_pointer > (time_delay_pore)) then
    rem shifted velocity
    forward_velpore = velocity_data[local_vel_pointer - (time_delay_pore )]
    rem 
    porepress_gradient = pore_m*forward_velpore + pore_b 
    
    
    
  else
    rem shifted velocity
    forward_velpore = velocity_data[data_sizetrain + local_vel_pointer - (time_delay_pore)]
    rem 
    porepress_gradient = pore_m*forward_velpore + pore_b 
    
  endif
  
  rem ###############################################
  
  if (local_vel_pointer > time_delay_side) then
    rem shifted velocity
    forward_velside = velocity_data[local_vel_pointer - (time_delay_side)]
    rem 
    sidepress_gradient = side_m*forward_velpore + side_b 
    
    
    
  else
    rem shifted velocity
    forward_velside = velocity_data[data_sizetrain + local_vel_pointer - (time_delay_side)]
    rem 
    sidepress_gradient = side_m*forward_velside + side_b 
    
  endif
  
  
  rem ##################################################
  
  if (local_vel_pointer > time_delay_axial) then
    rem shifted velocity
    forward_velaxial = velocity_data[local_vel_pointer - (time_delay_axial)]
    rem 
    axialpress_gradient = axial_m*forward_velpore + axial_b 
    
    
    
  else
    rem shifted velocity
    forward_velaxial = velocity_data[data_sizetrain + local_vel_pointer - (time_delay_axial )]
    rem 
    axialpress_gradient = axial_m*forward_velaxial + axial_b 
    
  endif
 
  rem ###################################################
  rem calculate estimated pressure with disturbance
  '  dec(pore_press_pointer)
  '  dec(side_press_pointer)
  '  dec(axial_press_pointer)
  if (pore_press_pointer > press_intervalp) then
    estimated_pore_press =  porepress_gradient*press_intervalp + pore_prss_array[pore_press_pointer-press_intervalp]
  else
    estimated_pore_press =   porepress_gradient*press_intervalp + pore_prss_array[arraysize + pore_press_pointer-press_intervalp]
    
    
  endif
  
  if (side_press_pointer > press_intervalp) then
    estimated_side_press = sidepress_gradient*press_intervalp + side_prss_array[pore_press_pointer-press_intervalp]
  else
    estimated_pore_press = sidepress_gradient*press_intervalp + side_prss_array[arraysize + pore_press_pointer-press_intervalp]
    
    
  endif
  
  if (axial_press_pointer > press_intervalp) then
    estimated_axial_press = axialpress_gradient*press_intervalp + axial_prss_array[pore_press_pointer-press_intervalp]
  else
    estimated_axial_press = axialpress_gradient*press_intervalp + axial_prss_array[arraysize + pore_press_pointer-press_intervalp]
    
    
  endif  
  
  rem # covert digital prssure to Kpa values
  estimated_pore_press  = (estimated_pore_press +6000)/((12000 / 65536))
  estimated_side_press = (estimated_side_press + 6000)/((12000 / 65536))
  estimated_axial_press = (estimated_axial_press + 6000)/((12000 / 65536))
endsub
