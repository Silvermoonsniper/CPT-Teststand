rem subroutine to get velocity and controller output array
rem input args:
rem    Sinusfrequenz_Vibro: vibration frequency
rem    WegInd_RegOut: controller output
rem    local_c: local pointer for controller output array
rem    controlleroutput[]: controller output data array
rem    vel_local: local velocity
rem    velocity_data[]: velocity data array
rem    local_vel_pointer: local velocity pointer
rem    data_sizetrain: size of velocity data array
rem    WegInd_Digits: controller output
rem    fulldata_number: number of datapoints for one period 
sub vel_controloutput_array(Sinusfrequenz_Vibro,WegInd_RegOut, local_c,controlleroutput[],vel_local,velocity_data[],local_vel_pointer, data_sizetrain,WegInd_Digits,fulldata_number) as float
  DIM space_number as float
 
                     
                              
  rem record velocity data into varible for fifo transmission
                           
  if  (local_vel_pointer <= data_sizetrain) then 
   
    velocity_data[local_vel_pointer] = vel_local
    local_vel_pointer =local_vel_pointer + 1    
                      
    rem check if velocity data has spike
    if (absf(vel_local) > 10) then
      if  (local_vel_pointer > 1) then 
        vel_local = velocity_data[local_vel_pointer-1] 
        velocity_data[local_vel_pointer]  = velocity_data[local_vel_pointer-1] 
      else
        vel_local = velocity_data[data_sizetrain] 
        velocity_data[local_vel_pointer]  = velocity_data[local_vel_pointer-1] 
      endif
    endif
                            
  else
    rem reinitialize index
    local_vel_pointer = 1
    velocity_data[local_vel_pointer] = vel_local
    local_vel_pointer =local_vel_pointer + 1 
        
  endif
                 
                      
  '########################################
                          
  rem store controller output data 
                         
                                        
                                
  if  (local_c <= data_sizetrain) then 
   
    controlleroutput[local_c] = WegInd_RegOut
    local_c =local_c + 1
    
  else
    rem reinitialie index, when data array is full
                            
    local_c = 1
                               
  endif
  
  
    
endsub
