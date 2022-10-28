rem subroutine to perform linear regression with incremental approaches
rem   input args:
rem          estimated_slope_array[]: array to store estimated slope 
rem          estimated_intercept_array[]: array to store estimated intercept
rem          estimated_parameter_pointer: pointer to update 
rem          Abtastfrequenz: sampling frequenc of adwin system (5khz in this case)
rem          Sinusfrequenz_Vibro: current vibration frequency
rem          v_fullFLAG: flag to indicate velocity data array is full
rem          local_vel_pointer: index for velocity data array
rem          data_sizetrain: size of velocity and controller output data array
rem          velocity_data[]: velocity data array
rem          linear_coff[]: estimated parameters
rem          newvelocity_data[]: velocity data array used for regression calculation
rem          newcontrolleroutput[]: controller output array used for regression calculation
rem          subxmean: current x mean
rem          subymean: current y mean
rem          subxydiffpsum: current xy sum product 
rem          subxsquaresum: current x square sum 
rem          pointer_vel_regression: pointer for linear regression
rem          WegInd_RegOut: controller output
rem          vel_local: local velocity data
rem          total_shift: time delay
rem          fulldata_number: number of datapoints for one period 
sub sub_blockcalculation(estimated_slope_array[],estimated_intercept_array[],estimated_parameter_pointer,Abtastfrequenz,Sinusfrequenz_Vibro,v_fullFLAG,local_vel_pointer, data_sizetrain,velocity_data[],linear_coff[],newvelocity_data[],newcontrolleroutput[],subxmean,subymean,subxydiffpsum,subxsquaresum,pointer_vel_regression,WegInd_RegOut, vel_local, total_shift,fulldata_number) 
  DIM f as long
 
  rem calculate sub mean, at each cycle we update mean and sum calculations until buffer is full
 
  '  
  rem record velocity and controller output in the array 
  if  ((pointer_vel_regression <= fulldata_number)) then
      
    rem when data buffer are not full
    rem velocity data some steps before, as velocity data is delayed by some time steps
    if ((v_fullFLAG = 0)) then
      if ((local_vel_pointer > total_shift) ) then
        newvelocity_data[pointer_vel_regression] = velocity_data[local_vel_pointer - total_shift]
    
      else
        newvelocity_data[pointer_vel_regression] = velocity_data[data_sizetrain + (local_vel_pointer - total_shift)]
      endif
   
      rem current controller output
      newcontrolleroutput[pointer_vel_regression] = WegInd_RegOut
    endif
    rem update x and y means
    if (v_fullFLAG = 0) then
      subxmean = subxmean + (newcontrolleroutput[pointer_vel_regression]/fulldata_number)
     
      subymean = subymean + (newvelocity_data[pointer_vel_regression]/fulldata_number)
      rem calculate sub xy sum and xx sum for linear regression 
      subxydiffpsum = subxydiffpsum + newcontrolleroutput[pointer_vel_regression]*newvelocity_data[pointer_vel_regression]
      subxsquaresum =  subxsquaresum + newcontrolleroutput[pointer_vel_regression]*newcontrolleroutput[pointer_vel_regression]
    endif
    if (( pointer_vel_regression = fulldata_number) and (v_fullFLAG = 0)) then
      rem set data buffer full flag
      v_fullFLAG = 1
      rem calculate estimated parameters
      linear_coff[1] = (subxydiffpsum - subymean*fulldata_number*subxmean) / (subxsquaresum - fulldata_number*subxmean*subxmean) 'm Steigung  
  
      linear_coff[2] = subymean-linear_coff[1]*subxmean 'b yAchsenabsch
    endif
    
    rem when data buffer are full, subtract old term values and add new term
    if (v_fullFLAG = 1) then
     
      rem subtract old value
      subxmean = subxmean - (newcontrolleroutput[pointer_vel_regression]/fulldata_number)
     
      subymean = subymean - (newvelocity_data[pointer_vel_regression]/fulldata_number)
      subxydiffpsum = subxydiffpsum - newcontrolleroutput[pointer_vel_regression]*newvelocity_data[pointer_vel_regression]
      subxsquaresum =  subxsquaresum - newcontrolleroutput[pointer_vel_regression]*newcontrolleroutput[pointer_vel_regression]
      rem assign current values
      if ((local_vel_pointer > total_shift) ) then
        newvelocity_data[pointer_vel_regression] = velocity_data[local_vel_pointer - total_shift]
    
      else
        newvelocity_data[pointer_vel_regression] = velocity_data[data_sizetrain + (local_vel_pointer - total_shift)]
      endif
      newcontrolleroutput[pointer_vel_regression] = WegInd_RegOut
      
        
      rem add new value
      subxmean = subxmean + (newcontrolleroutput[pointer_vel_regression]/fulldata_number)
      subymean = subymean +  (newvelocity_data[pointer_vel_regression]/fulldata_number)
      subxydiffpsum = subxydiffpsum + newcontrolleroutput[pointer_vel_regression]*newvelocity_data[pointer_vel_regression]
      subxsquaresum =  subxsquaresum + newcontrolleroutput[pointer_vel_regression]*newcontrolleroutput[pointer_vel_regression]
      
      
      rem calculate estimated parameters
      linear_coff[1] = (subxydiffpsum - subymean*fulldata_number*subxmean) / (subxsquaresum - fulldata_number*subxmean*subxmean) 'm Steigung  
  
      linear_coff[2] = subymean-linear_coff[1]*subxmean 'b yAchsenabsch
     
    endif
    rem update index
    pointer_vel_regression = pointer_vel_regression + 1
      
    
  else
    rem if we reach the end of array, reinitialize index 
    pointer_vel_regression = 1
    rem full flag 
    v_fullFLAG = 1
  endif
 
  rem record estimated parameters
  if (estimated_parameter_pointer <= data_sizetrain) then
    estimated_slope_array[estimated_parameter_pointer]  = linear_coff[1]
    estimated_intercept_array[estimated_parameter_pointer]  = linear_coff[2]
    inc(estimated_parameter_pointer)
  else
    estimated_parameter_pointer = 1
  endif
   
endsub
